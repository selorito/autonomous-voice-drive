"""
crane_controller: /crane/command (String) -> vinc + kamyon hareketi + yuk tasima

Komutlar:
    MOVE <yuk> <alan>   yuku alip alandaki bos bir yere birakir (orn: MOVE kirmizi_kasa depo)
    PICK <yuk>          sadece al (kancada kalir)
    PLACE <alan>        kancadaki yuku birak
    HOME                vinci topla, ayaklari kapat, baslangic noktasina sur
    STOP                ANINDA dur, gorevi iptal et (yuk kancadaysa kancada kalir)

Kamyon hedefe yetisemiyorsa otomatik olarak:
    kancayi kaldir -> bomu yatir (surus pozu) -> destek ayaklarini topla -> sur -> ayaklari ac
Kancada yukle SURULMEZ: hedef uzaksa yuk once kamyonun kendi kasasina konur, surulur, kasadan alinip birakilir.
Her yuk kaldirmadan once kanca kamerasindan dogrulama istenir (vision_node calisiyorsa).

Durum: /crane/status (String, okunabilir metin)
       /crane/state  (String, JSON, 10 Hz: IDLE/BUSY/STOPPED, eklemler, kamyon konumu, kancadaki yuk)
       /crane/ack    (String, "<id> <KOD> <aciklama>": komutun sonuna " #<id>" eklenirse)
Sonuc kodlari: OK, BUSY, UNKNOWN_LOAD, UNKNOWN_AREA, AREA_FULL, UNREACHABLE,
               HOOK_OCCUPIED, HOOK_EMPTY, BAD_COMMAND
Mikrofonsuz test:
    ros2 topic pub --once /crane/command std_msgs/msg/String "{data: 'MOVE kirmizi_kasa depo'}"
"""
import json
import math
import os
from collections import deque

import rclpy
import yaml
from builtin_interfaces.msg import Duration
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import crane_geometry as g

HERE = os.path.dirname(os.path.abspath(__file__))
RATE_HZ = 30.0
LOAD_UPDATE_HZ = 15.0
STATE_HZ = 10.0
CRANE_MODEL = "mpg_crane"
DECK = "kamyon"                      # kamyon kasasi: tek yuk yerli ozel "alan"
VERIFY_TIMEOUT = 2.0                 # kamera dogrulamasi icin bekleme (sn)
VERIFY_TOL = 1.0                     # beklenen konumdan bu kadar sapma kabul (m)
# Demo hizi: gercek vinc yavastir; sunumda izlenebilir olsun diye tum eksenler bu katsayiyla hizlanir
SPEED_SCALE = float(os.environ.get("CRANE_SPEED", "2.0"))


class CraneError(Exception):
    """Komut reddi: code makine tarafindan okunur (CAN ACK), mesaj insan icin."""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


class Site:
    """Alanlar, yuk yerleri (slot), kamyon kasasi ve yuklerin nerede oldugu."""

    def __init__(self, path):
        s = yaml.safe_load(open(path, encoding="utf-8"))
        self.areas = {k: (v["x"], v["y"]) for k, v in s["alanlar"].items()}
        self.offsets = s["slot_offsets"]
        self.loads = {}
        self.occupied = {}                       # (alan, slot) -> yuk
        for name, y in s["yukler"].items():
            size = s["yuk_tipleri"][y["tip"]]
            self.loads[name] = {"size": size, "area": y["alan"], "slot": y["slot"]}
            self.occupied[(y["alan"], y["slot"])] = name

    def park_x(self, area):
        """Kamyonun bu alan icin park edecegi yol konumu (kule ekseni alanin x'ine gelir)."""
        return self.areas[area][0] - g.SLEW_REL[0]

    def slot_xy(self, area, slot, bx=None):
        if area == DECK:
            return g.deck_xy(bx)
        ax, ay = self.areas[area]
        return ax + self.offsets[slot][0], ay + self.offsets[slot][1]

    def surface_z(self, area):
        return g.DECK_TOP if area == DECK else g.GROUND_Z

    def free_slot(self, area):
        n = 1 if area == DECK else len(self.offsets)
        for i in range(n):
            if (area, i) not in self.occupied:
                return i
        return None


class Plan:
    """Gorev adimlarini uretir; planlanan durumu (kamyon konumu, ayaklar, kule acisi) adim adim izler."""

    def __init__(self, ctrl):
        self.site = ctrl.site
        self.steps = []
        self.s = dict(ctrl.q)                    # planlanan durum

    def move(self, target, text):
        self.steps.append(("move", dict(target), text))
        self.s.update(target)

    def reachable(self, x, y):
        try:
            g.boom_above(x, y, self.s["slew"], self.s["bx"])
            return True
        except g.Unreachable:
            return False

    def deploy(self):
        if self.s["outrig"] < 1.0:
            self.move({"outrig": 1.0}, "destek ayaklari aciliyor")

    def drive_to(self, bx):
        if abs(bx - self.s["bx"]) < 0.01:
            self.deploy()
            return
        slew0 = round(self.s["slew"] / (2 * math.pi)) * 2 * math.pi       # en yakin "ileri" yon
        self.move({"hoist": g.HOIST_MIN}, "kanca kaldiriliyor")
        self.move({**g.STOW, "slew": slew0}, "bom surus pozuna aliniyor")
        self.move({"outrig": 0.0}, "destek ayaklari toplaniyor")
        self.move({"bx": bx}, f"kamyon suruluyor ({bx:+.0f} m)")
        self.deploy()

    def pick(self, load, area, slot):
        size = self.site.loads[load]["size"]
        x, y = self.site.slot_xy(area, slot, self.s["bx"])
        boom = g.boom_above(x, y, self.s["slew"], self.s["bx"])
        h = g.hoist_for(boom, self.site.surface_z(area) + size[2] + g.SLING, self.s["bx"])
        self.deploy()
        self.move({"hoist": g.HOIST_MIN}, "kanca kaldiriliyor")
        self.move({**boom, "hoist": g.HOIST_MIN}, f"{load} ustune donuluyor")
        self.steps.append(("verify", (load, x, y), "kamera ile yuk dogrulaniyor"))
        self.move({"hoist": h}, "kanca indiriliyor")
        self.steps.append(("attach", load, f"{load} baglandi"))
        self.move({"hoist": g.HOIST_MIN}, "yuk kaldiriliyor")

    def place(self, load, area, slot):
        size = self.site.loads[load]["size"]
        x, y = self.site.slot_xy(area, slot, self.s["bx"])
        boom = g.boom_above(x, y, self.s["slew"], self.s["bx"])
        h = g.hoist_for(boom, self.site.surface_z(area) + size[2] + g.SLING, self.s["bx"])
        where = "kamyon kasasina" if area == DECK else f"{area} alanina"
        self.move({**boom, "hoist": g.HOIST_MIN}, f"yuk {where} tasiniyor")
        self.move({"hoist": h}, "yuk indiriliyor")
        self.steps.append(("detach", (load, area, slot), f"{load} -> {where} birakildi"))
        self.move({"hoist": g.HOIST_MIN}, "kanca kaldiriliyor")

    def deliver(self, load, area, slot):
        """Kancadaki yuku hedefe goturur; uzaksa kasa uzerinden aktarir."""
        x, y = self.site.slot_xy(area, slot)
        if self.reachable(x, y):
            self.place(load, area, slot)
            return
        if self.site.free_slot(DECK) is None:
            raise CraneError("UNREACHABLE", "hedef uzak ve kamyon kasasi dolu")
        self.place(load, DECK, 0)
        self.drive_to(self.site.park_x(area))
        self.pick(load, DECK, 0)
        self.place(load, area, slot)

    def fetch(self, load):
        """Yukun yanina git ve kancaya al."""
        info = self.site.loads[load]
        if info["area"] == DECK:                 # (orn. STOP sonrasi kasada kalmis yuk)
            self.pick(load, DECK, 0)
            return
        x, y = self.site.slot_xy(info["area"], info["slot"])
        if not self.reachable(x, y):
            self.drive_to(self.site.park_x(info["area"]))
        self.pick(load, info["area"], info["slot"])


class CraneController(Node):
    def __init__(self):
        super().__init__("crane_controller")
        self.traj_pub = self.create_publisher(JointTrajectory, "/crane/set_joint_trajectory", 10)
        self.status_pub = self.create_publisher(String, "/crane/status", 10)
        self.state_pub = self.create_publisher(String, "/crane/state", 10)
        self.ack_pub = self.create_publisher(String, "/crane/ack", 10)
        self.create_subscription(String, "/crane/command", self.on_command, 10)
        self.create_subscription(String, "/crane/vision", self.on_vision, 10)
        self.state_cli = self.create_client(SetEntityState, "/set_entity_state")
        self.create_timer(1.0 / RATE_HZ, self.tick)

        self.site = Site(os.path.join(HERE, "sites.yaml"))
        self.q = dict(g.HOME)                    # su anki durum (kamyon + eklemler)
        self.motion = None                       # (q_baslangic, q_hedef, t0, sure)
        self.tasks = deque()                     # sirayla yapilacak adimlar
        self.attached = None                     # kancadaki yuk
        self.verify = None                       # (yuk, x, y, baslangic) - kamera dogrulamasi suruyor
        self.vision = None                       # son /crane/vision mesaji (dict) + alinma zamani
        self.vision_warned = False
        self.last_load_update = 0.0
        self.last_bx_sent = None
        self.last_deck_bx = g.HOME["bx"]
        self.pending = {}                        # varlik adi -> bekleyen set_entity_state cagrisi
        self.stopped = False                     # STOP sonrasi, yeni komuta kadar True
        self.stop_reason = ""                    # OPERATOR | VISION_MISMATCH | HEARTBEAT_LOST
        self.last_state_pub = 0.0
        self.say(f"Vinc hazir. Yukler: {', '.join(self.site.loads)} | Alanlar: {', '.join(self.site.areas)}")

    # ---------- yardimcilar ----------
    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def say(self, text, warn=False):
        (self.get_logger().warn if warn else self.get_logger().info)(text)
        self.status_pub.publish(String(data=text))

    def publish_joints(self):
        msg = JointTrajectory()
        msg.header.frame_id = "world"            # eklenti modeli yerinde tutar
        jp = g.joint_positions(self.q)
        msg.joint_names = list(jp)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in jp.values()]
        pt.time_from_start = Duration(sec=0, nanosec=0)
        msg.points = [pt]
        self.traj_pub.publish(msg)

    def set_entity_pose(self, name, x, y, z, force=False):
        if not self.state_cli.service_is_ready():
            return
        p = self.pending.get(name)
        if p is not None and not p.done() and not force:
            return                               # bir onceki cagri bitmeden yenisini yollama
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = name
        req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = x, y, z
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = "world"
        self.pending[name] = self.state_cli.call_async(req)

    def attached_load_pose(self):
        hx, hy, hz = g.hook_point(self.q)
        size = self.site.loads[self.attached]["size"]
        return hx, hy, hz - g.SLING - size[2]    # yuk modelinin orijini tabaninda

    def on_vision(self, msg):
        try:
            self.vision = {**json.loads(msg.data), "_t": self.now()}
        except ValueError:
            pass

    # ---------- komutlar ----------
    def on_command(self, msg):
        parts = msg.data.strip().split()
        req_id = None
        if parts and parts[-1].startswith("#"):  # "MOVE kirmizi_kasa depo #17" -> ACK icin id
            req_id = parts.pop()[1:]
        if not parts:
            return
        text = " ".join(parts)
        code, info = self.execute_command(parts[0].upper(), [a.lower() for a in parts[1:]], text)
        if req_id is not None:
            self.ack_pub.publish(String(data=f"{req_id} {code} {info}"))

    def execute_command(self, cmd, args, text):
        """Komutu uygular. Doner: (sonuc_kodu, aciklama)."""
        if cmd == "STOP":
            self.tasks.clear()
            self.motion = None                   # eklemler ve kamyon oldugu yerde kalir
            self.verify = None
            self.stopped = True
            self.stop_reason = "HEARTBEAT_LOST" if args and args[0] == "heartbeat" else "OPERATOR"
            self.say("DUR: gorev iptal edildi" + (f" ({self.attached} kancada)" if self.attached else ""))
            return "OK", "durduruldu"
        if self.tasks or self.motion:
            self.say(f"'{text}' reddedildi: vinc mesgul (once STOP)", warn=True)
            return "BUSY", "vinc mesgul"

        plan = Plan(self)
        try:
            if cmd == "MOVE" and len(args) == 2:
                load, area = args
                self.check(load=load, area=area, need_free_hook=True)
                slot = self.site.free_slot(area)
                if slot is None:
                    raise CraneError("AREA_FULL", f"{area} dolu")
                if self.site.loads[load]["area"] == DECK:
                    # yuk zaten kasada (orn. surus sirasinda STOP): once hedefe sur, sonra kasadan al
                    x, y = self.site.slot_xy(area, slot)
                    if not plan.reachable(x, y):
                        plan.drive_to(self.site.park_x(area))
                    plan.pick(load, DECK, 0)
                    plan.place(load, area, slot)
                else:
                    plan.fetch(load)
                    plan.deliver(load, area, slot)
            elif cmd == "PICK" and len(args) == 1:
                self.check(load=args[0], need_free_hook=True)
                plan.fetch(args[0])
            elif cmd == "PLACE" and len(args) == 1:
                if not self.attached:
                    raise CraneError("HOOK_EMPTY", "kancada yuk yok")
                self.check(area=args[0])
                slot = self.site.free_slot(args[0])
                if slot is None:
                    raise CraneError("AREA_FULL", f"{args[0]} dolu")
                plan.deliver(self.attached, args[0], slot)
            elif cmd == "HOME":
                if self.attached:
                    raise CraneError("HOOK_OCCUPIED", f"kancada {self.attached} var, once birak")
                plan.drive_to(g.HOME["bx"])
                plan.move({"hoist": g.HOIST_MIN}, "kanca kaldiriliyor")
                plan.move({**g.STOW, "slew": round(plan.s["slew"] / (2 * math.pi)) * 2 * math.pi},
                          "bom surus pozuna aliniyor")
                plan.move({"outrig": 0.0}, "destek ayaklari toplaniyor")
            else:
                self.say(f"Bilinmeyen komut: '{text}'", warn=True)
                return "BAD_COMMAND", "bilinmeyen komut"
        except (CraneError, g.Unreachable) as e:
            self.say(f"'{text}' reddedildi: {e}", warn=True)
            return getattr(e, "code", "UNREACHABLE"), str(e)
        self.stopped = False
        self.tasks.extend(plan.steps)
        self.tasks.append(("done", None, f"Gorev tamamlandi: {text}"))
        self.say(f"Gorev basladi: {text}  ({len(plan.steps)} adim)")
        return "OK", "gorev basladi"

    def check(self, load=None, area=None, need_free_hook=False):
        if need_free_hook and self.attached:
            raise CraneError("HOOK_OCCUPIED", f"kancada zaten {self.attached} var")
        if load is not None:
            if load not in self.site.loads:
                raise CraneError("UNKNOWN_LOAD", f"bilinmeyen yuk '{load}'")
            if load == self.attached:
                raise CraneError("HOOK_OCCUPIED", f"{load} zaten kancada")
        if area is not None and area not in self.site.areas:
            raise CraneError("UNKNOWN_AREA", f"bilinmeyen alan '{area}'")

    # ---------- dongu ----------
    def start_next_task(self):
        kind, arg, text = self.tasks.popleft()
        if kind == "move":
            goal = {**self.q, **arg}
            # her eksen kendi hiz sinirinda; hepsi birlikte bitsin -> en yavasina gore sure
            T = max(abs(goal[k] - self.q[k]) / (g.SPEED[k] * SPEED_SCALE) for k in self.q)
            if T < 1e-3:
                return
            self.motion = (dict(self.q), goal, self.now(), 1.5 * T + 0.3)
            self.say(text)
        elif kind == "verify":
            load, x, y = arg
            self.verify = (load, x, y, self.now())
            self.say(text)
        elif kind == "attach":
            info = self.site.loads[arg]
            del self.site.occupied[(info["area"], info["slot"])]
            info["area"] = info["slot"] = None
            self.attached = arg
            self.say(text)
        elif kind == "detach":
            load, area, slot = arg
            x, y = self.site.slot_xy(area, slot, self.q["bx"])
            self.site.loads[load].update(area=area, slot=slot)
            self.site.occupied[(area, slot)] = load
            self.attached = None
            self.set_entity_pose(load, x, y, self.site.surface_z(area), force=True)
            if area == DECK:
                self.last_deck_bx = self.q["bx"]
            self.say(text)
        elif kind == "done":
            self.say(text)

    def check_verify(self, now):
        """Kanca kamerasi beklenen yuku goruyor mu? Yanlis yuk gorurse gorevi durdurur."""
        load, x, y, t0 = self.verify
        v = self.vision
        if v is None or now - v["_t"] > 2.0:     # goruntu sistemi calismiyor
            if not self.vision_warned:
                self.say("kamera dogrulamasi atlandi: vision_node calismiyor", warn=True)
                self.vision_warned = True
            self.verify = None
            return
        if v["_t"] < t0 + 0.3:                   # bom durduktan sonraki ilk kareleri bekle
            return
        dets = v.get("detections", [])
        mine = [d for d in dets if d["name"] == load]
        if mine:
            err = min(math.hypot(d["x"] - x, d["y"] - y) for d in mine)
            if err <= VERIFY_TOL:
                self.say(f"kamera: {load} dogrulandi (sapma {err:.2f} m)")
                self.verify = None
                return
        other = [d for d in dets if d["name"] != load and math.hypot(d["x"] - x, d["y"] - y) < 0.8]
        if other:
            self.tasks.clear()
            self.verify = None
            self.stopped = True
            self.stop_reason = "VISION_MISMATCH"
            self.say(f"kamera: beklenen {load} degil, {other[0]['name']} goruldu -> GOREV DURDURULDU", warn=True)
            return
        if now - t0 > VERIFY_TIMEOUT:
            self.say(f"kamera: {load} goruntude bulunamadi, konum bilgisiyle devam ediliyor", warn=True)
            self.verify = None

    def tick(self):
        now = self.now()
        if self.motion is not None:
            q0, q1, t0, T = self.motion
            u = min(1.0, (now - t0) / T)
            s = u * u * (3 - 2 * u)              # yumusak kalkis/durus
            self.q = {k: q0[k] + (q1[k] - q0[k]) * s for k in q0}
            if u >= 1.0:
                self.motion = None
        elif self.verify is not None:
            self.check_verify(now)
        elif self.tasks:
            self.start_next_task()

        self.publish_joints()                    # bostayken de yayinla: eklemler yerinde dursun
        if self.last_bx_sent is None or abs(self.q["bx"] - self.last_bx_sent) > 1e-3:
            self.set_entity_pose(CRANE_MODEL, self.q["bx"], 0.0, 0.0)
            self.last_bx_sent = self.q["bx"]
        if now - self.last_load_update >= 1.0 / LOAD_UPDATE_HZ:
            self.last_load_update = now
            if self.attached:
                self.set_entity_pose(self.attached, *self.attached_load_pose())
        deck_load = self.site.occupied.get((DECK, 0))
        if deck_load and abs(self.q["bx"] - self.last_deck_bx) > 1e-3:   # kasadaki yuk kamyonla gider
            dx, dy = g.deck_xy(self.q["bx"])
            self.set_entity_pose(deck_load, dx, dy, g.DECK_TOP)
            self.last_deck_bx = self.q["bx"]
        if now - self.last_state_pub >= 1.0 / STATE_HZ:
            self.last_state_pub = now
            self.publish_state()

    def publish_state(self):
        busy = bool(self.tasks or self.motion or self.verify)
        state = {
            "state": "BUSY" if busy else ("STOPPED" if self.stopped else "IDLE"),
            "tasks_left": len(self.tasks),
            "attached": self.attached or "",
            "deck": self.site.occupied.get((DECK, 0), ""),
            "q": {"slew_deg": math.degrees(g.wrap(self.q["slew"])), "luff_deg": math.degrees(self.q["luff"]),
                  "tele_m": self.q["tele"], "hoist_m": self.q["hoist"]},
            "q_raw": self.q,                     # vision_node kamera konumunu buradan hesaplar
            "moving": self.motion is not None,
            "stop_reason": self.stop_reason if self.stopped else "",
        }
        self.state_pub.publish(String(data=json.dumps(state)))


def main():
    rclpy.init()
    node = CraneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
