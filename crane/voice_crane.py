"""
voice_crane: SESLI VINC KUMANDASI (CAN hattindaki "Voice ECU").
Mikrofon -> Whisper -> crane_commands.parse -> CAN (vcan0)

    "kırmızı kasayı depoya taşı" -> CRANE_CMD (0x100)  MOVE kirmizi_kasa depo
    "dur"                        -> ESTOP (0x010)       en yuksek oncelik
Arka planda 10 Hz VOICE_HEARTBEAT gonderir; vincten gelen ACK/STATUS'u ekrana yazar.

Kullanim:
    python3 voice_crane.py           # SUREKLI dinleme (VAD) + wake word: "Toros, kırmızı kasayı depoya taşı"
    python3 voice_crane.py --ptt     # bas-konus (Enter -> konus -> Enter), wake word gerekmez
    python3 voice_crane.py --text    # mikrofonsuz: cumleyi klavyeden yaz (yedek demo modu)

Yuk hareketi (tasi/al/birak) SESLI ONAY ister: "Kırmızı kasa depo alanına taşınacak. Onaylıyor musun?"
-> "evet" (calisir) / "hayır" ya da 5 sn sessizlik (iptal) / "dur" (acil durdurma). --no-confirm ile kapatilir.
Sesli cevaplar Piper TTS ile (crane/tts.py); ses dosyasi yoksa sadece ekrana yazilir. --no-tts ile kapatilir.

Surekli modda: komut "toros" ile baslamali. Sadece "toros" denirse 6 sn icindeki sonraki cumle kabul edilir.
Wake word degistirmek icin: --wake <kelime>  (ya da asagidaki default="toros" satiri)
"dur" ise wake word ARANMADAN her zaman gecer (acil durdurma).
"""
import argparse
import os
import queue
import sys
import threading
import time

import can
import cantools
import numpy as np
import yaml

from crane_commands import parse, parse_answer
from tts import Speaker

CRANE_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(CRANE_DIR)
CAN_INTERFACE = os.environ.get("CAN_INTERFACE", "socketcan")
CAN_CHANNEL = os.environ.get("CAN_CHANNEL", "vcan0")
NMT_OPERATIONAL = 5
ACK_TIMEOUT = 3.0          # yogun CPU'da (Gazebo + Whisper) vincin cevabi 1 sn'yi gecebilir
# Whisper tum cekirdekleri kullanmasin: Gazebo ve vinc kontrolcusune islemci kalsin
WHISPER_THREADS = int(os.environ.get("WHISPER_THREADS", max(2, (os.cpu_count() or 4) // 2)))

CONFIRM_SEC = 5.0
NEEDS_CONFIRM = {"MOVE", "PICK", "PLACE"}           # yuk hareketi onay ister; HOME ve ESTOP istemez
SONUC_SESLI = {"BUSY": "Vinç meşgul.", "UNKNOWN_LOAD": "Bu yükü tanımıyorum.", "UNKNOWN_AREA": "Bu alanı tanımıyorum.",
               "AREA_FULL": "Hedef alan dolu.", "UNREACHABLE": "Hedefe erişilemiyor.",
               "HOOK_OCCUPIED": "Kancada zaten yük var.", "HOOK_EMPTY": "Kancada yük yok.",
               "BAD_COMMAND": "Geçersiz komut.", "DUPLICATE": "Bu komut zaten alındı."}

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
        self.events = queue.Queue()                    # vincten gelen, sesli soylenecek olaylar
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
            reason = str(d.get("StopReason", "NONE"))
            if status != self.last_status:              # sadece degisince yaz
                prev = self.last_status
                self.last_status = status
                if prev and prev[0] == "BUSY" and status[0] in OLAY_SESLI:
                    text = OLAY_SESLI[status[0]].get(reason)
                    if text:
                        self.events.put(text)
                kanca = "" if status[1] == "NONE" else f", kancada {status[1]}"
                extra = " (vinç kontrolcüsü yanıt vermiyor! crane_controller çalışıyor mu?)" if status[0] == "FAULT" else ""
                print(f"  [vinç] {status[0]}{kanca}{extra}")

    def execute(self, result, timeout=ACK_TIMEOUT):
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


_SITES = yaml.safe_load(open(os.path.join(CRANE_DIR, "sites.yaml"), encoding="utf-8"))
LOAD_SAY = {k: v["isimler"][0] for k, v in _SITES["yukler"].items()}      # "kırmızı kasa"
AREA_SAY = {k: v["isimler"][0] for k, v in _SITES["alanlar"].items()}     # "depo"


def describe(r):
    """Komut -> (onay sorusu, kabul edilince soylenecek cumle)."""
    L = LOAD_SAY.get(r.get("load"), "")
    A = AREA_SAY.get(r.get("area"), "")
    cap = lambda t: t[:1].upper() + t[1:]
    if r["cmd"] == "MOVE":
        return f"{cap(L)} {A} alanına taşınacak. Onaylıyor musun?", f"Tamam. {cap(L)} {A} alanına taşınıyor."
    if r["cmd"] == "PICK":
        return f"{cap(L)} kaldırılacak. Onaylıyor musun?", f"Tamam. {cap(L)} kaldırılıyor."
    if r["cmd"] == "PLACE":
        return f"Yük {A} alanına bırakılacak. Onaylıyor musun?", f"Tamam. Yük {A} alanına bırakılıyor."
    return None, "Vinç toplanıyor."


OLAY_SESLI = {                                         # gorev bitince / kendiliginden durunca vinc konusur
    "IDLE": {"NONE": "Görev tamamlandı."},
    "STOPPED": {"VISION_MISMATCH": "Kamera farklı bir yük gördü. Görev durduruldu.",
                "HEARTBEAT_LOST": "Kumanda bağlantısı koptu. Vinç durduruldu."},
}

SABIT_CUMLELER = ["Durduruldu.", "Dinliyorum.", "Anlaşılmadı.", "İptal edildi.", "Onay gelmedi, iptal edildi.",
                  "Evet ya da hayır de.", "Vinçten cevap yok.", "Komut reddedildi."]


def all_phrases():
    """Vincin soyleyebilecegi TUM cumleler (gen_phrases.py bunlari onceden seslendirir)."""
    out = set(SABIT_CUMLELER) | set(SONUC_SESLI.values())
    out.update(t for d in OLAY_SESLI.values() for t in d.values())
    for load in LOAD_SAY:
        for area in AREA_SAY:
            out.update(describe({"cmd": "MOVE", "load": load, "area": area}))
        out.update(describe({"cmd": "PICK", "load": load}))
    for area in AREA_SAY:
        out.update(describe({"cmd": "PLACE", "area": area}))
    out.update(t for t in describe({"cmd": "HOME"}) if t)
    return sorted(out)


def say_events(ecu, speak):
    while not ecu.events.empty():
        text = ecu.events.get()
        print(f"  [vinç] {text}")
        speak(text)


def estop(ecu, speak):
    ecu.execute({"cmd": "ESTOP"})                 # once gonder, sonra konus: durdurma gecikmesin
    print(f"  ■ ACİL DURDURMA gönderildi (0x010, counter {ecu.counter})")
    speak("Durduruldu.")


def make_voice_confirm(utt, transcribe, speak, timeout=CONFIRM_SEC):
    """Surekli dinleme icin onay fonksiyonu: soruyu sor, cevabi wake word'suz bekle.
    utt: konusma ureteci (sessizlikte None verir). Doner: yes | no | stop | timeout."""
    def confirm(question):
        speak(question)
        deadline, retried = time.monotonic() + timeout, False
        for audio in utt:
            if audio is None:
                if time.monotonic() > deadline:
                    return "timeout"
                continue
            ans = parse_answer(transcribe(audio))
            if ans:
                return ans
            if retried:
                return "no"
            retried = True                          # anlasilmadi: bir kez daha sor
            speak("Evet ya da hayır de.")
            deadline = time.monotonic() + timeout
        return "timeout"
    return confirm


def handle(ecu, text, gate=None, confirm=None, speak=None):
    """Bir cumleyi isler. gate: wake word kapisi (surekli mod). confirm(soru) -> 'yes'|'no'|'stop'|'timeout'."""
    speak = speak or (lambda t: print(f"  🔊 {t}"))
    if not text.strip():                       # bos Enter / sessiz kayit: hicbir sey yapma
        return
    r = parse(text)
    if gate is not None:                       # surekli dinleme: wake word kapisi
        status, rest = gate.check(text, is_stop=(r["cmd"] == "ESTOP"))
        if status == "no_wake":
            print("  · wake word yok -> yok sayıldı")
            return
        if status == "armed":
            print("  ◉ dinliyorum... (komutu söyle)")
            speak("Dinliyorum.")
            return
        if status == "ok":
            r = parse(rest)
    if r["cmd"] == "ESTOP":
        estop(ecu, speak)
        return
    if r["cmd"] is None:
        print(f"  ✗ ANLAŞILMADI: {r['reason']} -> CAN'e bir şey gönderilmedi")
        speak("Anlaşılmadı.")
        return

    desc = " ".join(str(v) for v in (r["cmd"], r.get("load"), r.get("area")) if v)
    question, ok_text = describe(r)
    if confirm is not None and r["cmd"] in NEEDS_CONFIRM:
        ans = confirm(question)
        if ans == "stop":
            estop(ecu, speak)
            return
        if ans != "yes":
            print(f"  ✗ {desc}: onay yok ({ans}) -> iptal, CAN'e bir şey gönderilmedi")
            speak("İptal edildi." if ans == "no" else "Onay gelmedi, iptal edildi.")
            return
        print("  ✓ onaylandı")

    code = ecu.execute(r)
    if code is None:
        print(f"  ! {desc} gönderildi ama vinçten cevap yok "
              f"(kontrol: 'ros2 node list' -> /crane_controller ve /can_bridge birer kez olmalı)")
        speak("Vinçten cevap yok.")
    elif code == "OK":
        print(f"  ✓ {desc} -> vinç: {SONUC['OK']}")
        speak(ok_text)
    else:
        print(f"  ✗ {desc} -> vinç: {SONUC.get(code, code)}")
        speak(SONUC_SESLI.get(code, "Komut reddedildi."))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--text", action="store_true", help="mikrofon yerine klavyeden cumle")
    ap.add_argument("--ptt", action="store_true", help="bas-konus (Enter) modu, wake word yok")
    ap.add_argument("--wake", default="toros", help="wake word (varsayilan: toros)")
    ap.add_argument("--no-confirm", action="store_true", help="yuk hareketlerinde onay sorma")
    ap.add_argument("--no-tts", action="store_true", help="sesli cevap verme, sadece ekrana yaz")
    ap.add_argument("--voice", default="tr_TR-dfki-medium", help="Piper ses modeli (crane/voices/<ad>.onnx)")
    args = ap.parse_args()
    speaker = Speaker(voice=args.voice, enabled=not args.no_tts)

    ecu = VoiceECU()
    print(f"Sesli kumanda CAN'e bağlı: {CAN_INTERFACE}/{CAN_CHANNEL}")

    sys.path.insert(0, REPO_DIR)
    if args.text:
        print("Hazır (klavye modu). Örnek: 'kırmızı kasayı depoya taşı', 'beton bloğu al', 'dur'")

        def confirm_text(question):
            speaker.say(question)
            return parse_answer(input("  (evet / hayır) > ")) or "no"
        try:
            while True:
                line = input("\n> ")
                say_events(ecu, speaker.say)              # klavye modunda olaylar Enter'da soylenir
                handle(ecu, line, confirm=None if args.no_confirm else confirm_text, speak=speaker.say)
        except (KeyboardInterrupt, EOFError):
            pass
        finally:
            ecu.close()
        return

    from faster_whisper import WhisperModel
    from voice_node import MODEL, DEVICE, COMPUTE, SR, record_push_to_talk
    prompt = (f"{args.wake}, kırmızı kasayı depoya taşı, mavi kasa, beyaz kasa, beton blok, stok, depo, "
              "atölye, sevkiyat, al, bırak, koy, götür, vinci topla, dur, evet, hayır, iptal, onaylıyorum")
    print(f"Model yükleniyor: {MODEL} ({WHISPER_THREADS} çekirdek; değiştirmek için WHISPER_THREADS=...)")
    model = WhisperModel(MODEL, device=DEVICE, compute_type=COMPUTE, cpu_threads=WHISPER_THREADS)
    model.transcribe(np.zeros(SR, dtype=np.float32), language="tr")

    def transcribe(audio):
        t = time.time()
        segs, _ = model.transcribe(audio, language="tr", initial_prompt=prompt, beam_size=5, vad_filter=True)
        text = " ".join(s.text.strip() for s in segs)
        print(f"  duyulan: '{text}'  ({len(audio) / SR:.1f} sn ses, {(time.time() - t) * 1000:.0f} ms)")
        return text

    try:
        if args.ptt:
            print("Hazır (bas-konuş). Enter -> konuş -> Enter.")

            def confirm_ptt(question):
                speaker.say(question)
                input("  [Enter] cevabını söyle (evet / hayır)...")
                print("  ● Kayıt... (bitince Enter)")
                return parse_answer(transcribe(record_push_to_talk())) or "no"
            while True:
                input("\n[Enter] konuşmaya başla...")
                say_events(ecu, speaker.say)
                print("  ● Kayıt... (bitince Enter)")
                handle(ecu, transcribe(record_push_to_talk()),
                       confirm=None if args.no_confirm else confirm_ptt, speak=speaker.say)
        else:
            from audio_listener import SpeechListener, WakeGate
            gate = WakeGate([args.wake])
            listener = SpeechListener()
            speaker.listener = listener                 # konusurken mikrofonu durdur
            utt = listener.utterances(idle=True)        # konusma yokken None verir (zaman asimi icin)

            confirm_voice = make_voice_confirm(utt, transcribe, speaker.say)
            print(f"Hazır (sürekli dinleme). Komutlar '{args.wake}' ile başlar: "
                  f"'{args.wake.capitalize()}, kırmızı kasayı depoya taşı'. 'Dur' her zaman geçer.")
            for audio in utt:
                if audio is None:
                    say_events(ecu, speaker.say)
                    continue
                print("\n  ▶ konuşma algılandı")
                handle(ecu, transcribe(audio), gate,
                       confirm=None if args.no_confirm else confirm_voice, speak=speaker.say)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        ecu.close()


if __name__ == "__main__":
    main()
