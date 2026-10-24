"""
voice_crane: SESLI VINC KUMANDASI (CAN hattindaki "Voice ECU").
Mikrofon -> Whisper -> crane_commands.parse -> CAN (vcan0)

    "kırmızı kasayı depoya taşı" -> CRANE_CMD (0x100)  MOVE kirmizi_kasa depo
    "dur"                        -> ESTOP (0x010)       en yuksek oncelik
Arka planda 10 Hz VOICE_HEARTBEAT gonderir; vincten gelen ACK/STATUS'u ekrana yazar.

Kullanim:
    python3 voice_crane.py           # bas-konus (Enter -> konus -> Enter)
    python3 voice_crane.py --text    # mikrofonsuz: cumleyi klavyeden yaz (yedek demo modu)
"""
import argparse
import os
import sys
import threading
import time

import can
import cantools
import numpy as np
import yaml

from crane_commands import parse

CRANE_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(CRANE_DIR)
CAN_INTERFACE = os.environ.get("CAN_INTERFACE", "socketcan")
CAN_CHANNEL = os.environ.get("CAN_CHANNEL", "vcan0")
NMT_OPERATIONAL = 5

SONUC = {"OK": "kabul edildi", "BUSY": "vinç meşgul", "UNKNOWN_LOAD": "bilinmeyen yük",
         "UNKNOWN_AREA": "bilinmeyen alan", "AREA_FULL": "alan dolu", "UNREACHABLE": "erişilemiyor",
         "HOOK_OCCUPIED": "kancada zaten yük var", "HOOK_EMPTY": "kancada yük yok",
         "BAD_COMMAND": "geçersiz komut", "DUPLICATE": "tekrar eden komut"}


class VoiceECU:
    def __init__(self, bus=None):
        self.db = cantools.database.load_file(os.path.join(CRANE_DIR, "can", "crane.dbc"))
        s = yaml.safe_load(open(os.path.join(CRANE_DIR, "sites.yaml"), encoding="utf-8"))
        self.load_ids = {k: v["can_id"] for k, v in s["yukler"].items()}
        self.area_ids = {k: v["can_id"] for k, v in s["alanlar"].items()}
        self.bus = bus or can.Bus(interface=CAN_INTERFACE, channel=CAN_CHANNEL)
        self.counter = int(time.time()) % 256          # yeniden baslatinca eski sayaca takilmasin
        self.acks = {}
        self.ack_event = threading.Event()
        self.last_status = None
        self.running = True
        self.heartbeat_on = True
        threading.Thread(target=self.heartbeat_loop, daemon=True).start()
        self.notifier = can.Notifier(self.bus, [self.on_frame])

    def send(self, name, data):
        m = self.db.get_message_by_name(name)
        self.bus.send(can.Message(arbitration_id=m.frame_id, data=m.encode(data), is_extended_id=False))

    def heartbeat_loop(self):
        while self.running:
            if self.heartbeat_on:
                try:
                    self.send("VOICE_HEARTBEAT", {"NmtState": NMT_OPERATIONAL})
                except can.CanError:
                    pass
            time.sleep(0.1)

    def on_frame(self, frame):
        try:
            msg = self.db.get_message_by_frame_id(frame.arbitration_id)
        except KeyError:
            return
        if msg.name == "CRANE_ACK":
            d = msg.decode(frame.data)
            self.acks[int(d["Counter"])] = str(d["Result"])
            self.ack_event.set()
        elif msg.name == "CRANE_STATUS":
            d = msg.decode(frame.data)
            status = (str(d["State"]), str(d["AttachedLoad"]))
            if status != self.last_status:              # sadece degisince yaz
                self.last_status = status
                kanca = "" if status[1] == "NONE" else f", kancada {status[1]}"
                print(f"  [vinç] {status[0]}{kanca}")

    def execute(self, result, timeout=1.0):
        """parse() sonucunu CAN'e gonderir, ACK bekler. Doner: sonuc kodu ya da None."""
        cmd = result["cmd"]
        self.counter = (self.counter + 1) % 256
        if cmd == "ESTOP":
            self.send("ESTOP", {"Source": 1, "Counter": self.counter})
            return "ESTOP"
        self.ack_event.clear()
        self.send("CRANE_CMD", {"CmdType": cmd,
                                "LoadId": self.load_ids.get(result.get("load"), 0),
                                "AreaId": self.area_ids.get(result.get("area"), 0),
                                "Counter": self.counter})
        deadline = time.monotonic() + timeout
        while self.counter not in self.acks and time.monotonic() < deadline:
            self.ack_event.wait(0.05)
            self.ack_event.clear()
        return self.acks.pop(self.counter, None)

    def close(self):
        self.running = False
        self.notifier.stop()
        self.bus.shutdown()


def handle(ecu, text):
    r = parse(text)
    if r["cmd"] is None:
        print(f"  ✗ ANLAŞILMADI: {r['reason']} -> CAN'e bir şey gönderilmedi")
        return
    desc = " ".join(str(v) for v in (r["cmd"], r.get("load"), r.get("area")) if v)
    code = ecu.execute(r)
    if code == "ESTOP":
        print(f"  ■ ACİL DURDURMA gönderildi (0x010, counter {ecu.counter})")
    elif code is None:
        print(f"  ! {desc} gönderildi ama vinçten cevap yok (can_bridge çalışıyor mu?)")
    else:
        mark = "✓" if code == "OK" else "✗"
        print(f"  {mark} {desc} -> vinç: {SONUC.get(code, code)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--text", action="store_true", help="mikrofon yerine klavyeden cumle")
    args = ap.parse_args()

    ecu = VoiceECU()
    print(f"Sesli kumanda CAN'e bağlı: {CAN_INTERFACE}/{CAN_CHANNEL}")

    if args.text:
        transcribe = None
    else:
        sys.path.insert(0, REPO_DIR)
        from faster_whisper import WhisperModel
        from voice_node import MODEL, DEVICE, COMPUTE, SR, record_push_to_talk
        prompt = ("kırmızı kasayı depoya taşı, mavi kasa, beyaz kasa, beton blok, stok, depo, atölye, "
                  "sevkiyat, al, bırak, koy, götür, vinci topla, dur")
        print(f"Model yükleniyor: {MODEL}")
        model = WhisperModel(MODEL, device=DEVICE, compute_type=COMPUTE)
        model.transcribe(np.zeros(SR, dtype=np.float32), language="tr")

        def transcribe():
            input("\n[Enter] konuşmaya başla...")
            print("  ● Kayıt... (bitince Enter)")
            audio = record_push_to_talk()
            t = time.time()
            segs, _ = model.transcribe(audio, language="tr", initial_prompt=prompt,
                                       beam_size=5, vad_filter=True)
            text = " ".join(s.text.strip() for s in segs)
            print(f"  duyulan: '{text}'  ({(time.time() - t) * 1000:.0f} ms)")
            return text

    print("Hazır. Örnek: 'kırmızı kasayı depoya taşı', 'beton bloğu al', 'atölyeye bırak', 'vinci topla', 'dur'")
    try:
        while True:
            text = transcribe() if transcribe else input("\n> ")
            handle(ecu, text)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        ecu.close()


if __name__ == "__main__":
    main()
