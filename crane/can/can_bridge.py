"""
can_bridge: VINC ECU'SU. CAN hatti (vcan0) <-> crane_controller (ROS).

CAN'den gelen:
    ESTOP (0x010)            -> /crane/command "STOP"         (her seyden once)
    CRANE_CMD (0x100)        -> /crane/command "MOVE kirmizi_kasa depo #<counter>"
    VOICE_HEARTBEAT (0x710)  -> dead-man takibi
CAN'e giden:
    CRANE_ACK (0x201)        her komutun sonucu (/crane/ack'ten)
    CRANE_STATUS (0x200) + CRANE_JOINTS (0x300) 10 Hz (/crane/state'ten)
    CRANE_HEARTBEAT (0x720)  1 Hz

Guvenlik:
    - Ayni Counter ile gelen komut tekrar calistirilmaz (DUPLICATE)
    - CAN'den baslatilan gorev surerken sesli kumandanin heartbeat'i 1 sn kesilirse vinc durur

Calistirma:
    python3 can/can_bridge.py               (vcan0, socketcan)
    CAN_CHANNEL=can0 python3 can/can_bridge.py  (gercek CAN karti)
"""
import json
import os
import sys
import threading
import time

import can
import cantools
import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String

CAN_DIR = os.path.dirname(os.path.abspath(__file__))
CRANE_DIR = os.path.dirname(CAN_DIR)
CAN_INTERFACE = os.environ.get("CAN_INTERFACE", "socketcan")
CAN_CHANNEL = os.environ.get("CAN_CHANNEL", "vcan0")
REQUIRE_HEARTBEAT = os.environ.get("REQUIRE_HEARTBEAT", "1") == "1"
HEARTBEAT_TIMEOUT = 1.0
NMT_OPERATIONAL = 5


class CanBridge(Node):
    def __init__(self, bus=None):
        super().__init__("can_bridge")
        self.db = cantools.database.load_file(os.path.join(CAN_DIR, "crane.dbc"))
        self.check_ids()

        self.cmd_pub = self.create_publisher(String, "/crane/command", 10)
        self.create_subscription(String, "/crane/ack", self.on_ack, 10)
        self.create_subscription(String, "/crane/state", self.on_state, 10)
        self.create_timer(0.1, self.send_status)
        self.create_timer(1.0, self.send_heartbeat)
        self.create_timer(0.1, self.watchdog)

        self.lock = threading.Lock()
        self.state = None                 # son /crane/state (dict)
        self.last_counter = None
        self.voice_hb = None              # son sesli kumanda heartbeat zamani (monotonic)
        self.can_task_active = False      # CAN'den baslatilmis gorev suruyor mu
        self.wd_tripped = False

        self.bus = bus or can.Bus(interface=CAN_INTERFACE, channel=CAN_CHANNEL)
        self.notifier = can.Notifier(self.bus, [self.on_frame])
        self.get_logger().info(f"Vinc ECU hazir: {self.bus.channel_info}, "
                               f"heartbeat {'zorunlu' if REQUIRE_HEARTBEAT else 'kapali'}")

    def check_ids(self):
        """DBC'deki yuk/alan numaralari sites.yaml ile ayni mi? Degilse baslama."""
        s = yaml.safe_load(open(os.path.join(CRANE_DIR, "sites.yaml"), encoding="utf-8"))
        cmd = self.db.get_message_by_name("CRANE_CMD")
        for sig, table in (("LoadId", s["yukler"]), ("AreaId", s["alanlar"])):
            choices = cmd.get_signal_by_name(sig).choices
            for name, v in table.items():
                if str(choices.get(v["can_id"])) != name:
                    sys.exit(f"crane.dbc {sig}={v['can_id']} '{choices.get(v['can_id'])}' != sites.yaml '{name}'")

    def send(self, name, data):
        m = self.db.get_message_by_name(name)
        try:
            self.bus.send(can.Message(arbitration_id=m.frame_id, data=m.encode(data), is_extended_id=False))
        except can.CanError as e:
            self.get_logger().error(f"CAN gonderilemedi ({name}): {e}")

    # ---------- CAN -> ROS (python-can kendi thread'inde cagirir) ----------
    def on_frame(self, frame):
        try:
            msg = self.db.get_message_by_frame_id(frame.arbitration_id)
        except KeyError:
            return                                    # bize ait olmayan mesaj
        if msg.name == "ESTOP":
            self.cmd_pub.publish(String(data="STOP"))
            with self.lock:
                self.can_task_active = False
            self.get_logger().warn("CAN: ACIL DURDURMA")
        elif msg.name == "CRANE_CMD":
            self.on_cmd(msg.decode(frame.data))
        elif msg.name == "VOICE_HEARTBEAT":
            with self.lock:
                self.voice_hb = time.monotonic()

    def on_cmd(self, d):
        counter = int(d["Counter"])
        with self.lock:
            if counter == self.last_counter:
                self.send("CRANE_ACK", {"Counter": counter, "Result": "DUPLICATE"})
                self.get_logger().warn(f"CAN: tekrar eden komut (counter {counter}) yok sayildi")
                return
            self.last_counter = counter
        cmd, load, area = str(d["CmdType"]), str(d["LoadId"]), str(d["AreaId"])
        need = {"MOVE": (load, area), "PICK": (load,), "PLACE": (area,), "HOME": ()}
        if cmd not in need or "NONE" in need[cmd] or any(v.isdigit() for v in need[cmd]):
            self.send("CRANE_ACK", {"Counter": counter, "Result": "BAD_COMMAND"})
            return
        text = " ".join((cmd,) + need[cmd])
        self.get_logger().info(f"CAN: {text} (counter {counter})")
        self.cmd_pub.publish(String(data=f"{text} #{counter}"))

    # ---------- ROS -> CAN ----------
    def on_ack(self, msg):
        parts = msg.data.split(maxsplit=2)
        if len(parts) < 2 or not parts[0].isdigit():
            return                                    # CAN disindan gelen komutun ACK'i
        counter, code = int(parts[0]), parts[1]
        if code == "OK":
            with self.lock:
                self.can_task_active = True
                self.wd_tripped = False
        self.send("CRANE_ACK", {"Counter": counter, "Result": code})

    def on_state(self, msg):
        with self.lock:
            self.state = json.loads(msg.data)
            if self.state["state"] != "BUSY":
                self.can_task_active = False

    def send_status(self):
        with self.lock:
            st = self.state
        if st is None:
            return
        attached = st["attached"] or "NONE"
        self.send("CRANE_STATUS", {"State": st["state"], "TasksLeft": min(255, st["tasks_left"]),
                                   "AttachedLoad": attached})
        q = st["q"]
        self.send("CRANE_JOINTS", {"Slew": round(q["slew_deg"], 2), "Luff": round(q["luff_deg"], 2),
                                   "Tele": round(q["tele_m"], 3), "Hoist": round(q["hoist_m"], 3)})

    def send_heartbeat(self):
        self.send("CRANE_HEARTBEAT", {"NmtState": NMT_OPERATIONAL})

    def watchdog(self):
        """Dead-man: CAN gorevi surerken sesli kumanda susarsa vinci durdur."""
        if not REQUIRE_HEARTBEAT:
            return
        with self.lock:
            active, hb, tripped = self.can_task_active, self.voice_hb, self.wd_tripped
        if active and not tripped and (hb is None or time.monotonic() - hb > HEARTBEAT_TIMEOUT):
            self.cmd_pub.publish(String(data="STOP"))
            with self.lock:
                self.wd_tripped = True
                self.can_task_active = False
            self.get_logger().error(f"Sesli kumanda heartbeat'i {HEARTBEAT_TIMEOUT:.0f} sn yok -> VINC DURDURULDU")

    def close(self):
        self.notifier.stop()
        self.bus.shutdown()


def main():
    rclpy.init()
    node = CanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
