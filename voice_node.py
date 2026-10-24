"""
voice_node: mikrofon -> Whisper -> komut esleme -> /voice/command (String)

Modlar:
    python3 voice_node.py         SUREKLI dinleme (Silero VAD) + wake word: "Robot, atölyeye git"
                                  Sadece "robot" denirse 6 sn icindeki sonraki cumle kabul edilir.
                                  "dur" wake word ARANMADAN her zaman gecer.
    python3 voice_node.py --ptt   bas-konus: Enter -> konus -> Enter (wake word gerekmez)

Calistirma (Gazebo + motion_node acikken).
Cikmak icin: Ctrl+C
"""
import argparse
import os
import re
import threading
import time

import numpy as np
import rclpy
import sounddevice as sd
import yaml
from faster_whisper import WhisperModel
from rapidfuzz import fuzz
from rclpy.node import Node
from std_msgs.msg import String

# ---- Ayarlar (whisper_test.py ile ayni) ----
MODEL = "small"
DEVICE = "cpu"
COMPUTE = "int8"
SR = 16000
MAX_RECORD_SEC = 6.0    # bundan uzun kayitlarin fazlasi atilir
ESIK = 85

# ---- Hedef yerler (motion_node ile ayni dosya) ----
PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
LOCATIONS_FILE = os.environ.get("LOCATIONS_FILE",
                                os.path.join(PROJECT_DIR, "worlds", "demo_locations.yaml"))
with open(LOCATIONS_FILE, encoding="utf-8") as _f:
    # {"atolye": ["atölye", "kırmızı"], "depo": [...], ...}
    YER_ISIMLERI = {k: [n.lower() for n in v["isimler"]]
                    for k, v in yaml.safe_load(_f)["konumlar"].items()}

# Whisper'a yer isimlerini de ipucu olarak ver
PROMPT = "robot, ileri git, geri git, sola dön, sağa dön, dur, " + \
         ", ".join(isim for isimler in YER_ISIMLERI.values() for isim in isimler)

KOMUTLAR = {
    "FORWARD":  ["ileri git", "ileriye git", "ileri gidin", "ilerle", "ileri"],
    "BACKWARD": ["geri git", "geriye git", "geri gidin", "geriye gidin", "geri gel", "geri"],
    "LEFT":     ["sola dön", "sola git", "sol"],
    "RIGHT":    ["sağa dön", "sağa git", "sağ"],
}
STOP_KELIMELERI = {"dur", "durun", "dursun", "durdur", "stop"}
OLUMSUZ = {"gitme", "gitmeyin", "dönme", "dönmeyin", "gelme", "ilerleme"}
# GOTO icin cumlede bunlardan biri olmali: "atolye nerede" araci hareket ettirmesin
HAREKET_FIILLERI = {"git", "gidin", "gel", "gelin", "dön", "dönün", "ilerle", "sür", "götür"}
EK_PAYI = 5     # isimden sonra en fazla bu kadar harf ek kabul edilir (atölye + "sine")


# ---- whisper_test.py'deki fonksiyonlar (degismedi) ----
def normalize(text):
    text = text.replace("İ", "i").replace("I", "ı").lower()
    text = re.sub(r"[^\w\s]", " ", text)
    words = text.split()
    words = [w for i, w in enumerate(words) if i == 0 or w != words[i - 1]]
    return " ".join(words)


def find_locations(words, esik=ESIK):
    """Cumledeki yer isimlerini bulur. Turkce ek icin kelimenin bas kismina bakar:
    'depoya'[:4] = 'depo', 'kırmızıya'[:7] = 'kırmızı'."""
    found = {}
    for w in words:
        for yer, isimler in YER_ISIMLERI.items():
            for isim in isimler:
                if not (len(isim) <= len(w) <= len(isim) + EK_PAYI):
                    continue
                skor = fuzz.ratio(isim, w[:len(isim)])
                if skor >= esik:
                    found[yer] = max(skor, found.get(yer, 0))
    return found


def match(text, esik=ESIK):
    words = text.split()
    if not words:
        return None, 0
    if STOP_KELIMELERI & set(words):
        return "STOP", 100
    if OLUMSUZ & set(words):
        return None, 0

    hareket = match_motion(words, esik)
    yerler = find_locations(words, esik)
    if yerler:
        if len(yerler) > 1:                          # "depoya sonra atolyeye git" -> belirsiz
            return None, 0
        if not (HAREKET_FIILLERI & set(words)):      # "atolye nerede" -> hareket yok
            return None, 0
        if set(hareket) - {"FORWARD"}:               # "sola don depoya git" -> celiskili
            return None, 0
        yer, skor = next(iter(yerler.items()))
        return f"GOTO {yer}", skor

    if len(hareket) != 1:
        return None, 0
    return next(iter(hareket.items()))


def match_motion(words, esik=ESIK):
    """Ileri/geri/sol/sag komutlarini bulur -> {komut: skor} (onceki match mantigi)."""
    # Her kelime penceresi icin EN IYI komutu sec:
    #   "geriye git" -> BACKWARD'a 100, FORWARD'a ("ileriye git") 86 benzer -> pencere BACKWARD sayilir
    pencereler = {}     # (baslangic, uzunluk) -> (komut, skor)
    for komut, ifadeler in KOMUTLAR.items():
        for ifade in ifadeler:
            n = len(ifade.split())
            for i in range(len(words) - n + 1):
                skor = fuzz.ratio(ifade, " ".join(words[i:i + n]))
                if skor >= esik and skor > pencereler.get((i, n), (None, 0))[1]:
                    pencereler[(i, n)] = (komut, skor)
    bulunan = {}
    for komut, skor in pencereler.values():
        bulunan[komut] = max(skor, bulunan.get(komut, 0))
    # Birden fazla FARKLI komut varsa match() belirsiz sayar ve hicbir sey yapmaz
    return bulunan


# ---- Bas-konus kayit ----
def record_push_to_talk():
    """Enter'a basilana kadar kaydeder; MAX_RECORD_SEC'ten uzun kisim atilir."""
    chunks = []

    def callback(indata, frames, t, status):
        # Bu fonksiyon ses kartindan her yeni parca geldiginde ayri bir thread'de cagrilir
        chunks.append(indata.copy())

    with sd.InputStream(samplerate=SR, channels=1, dtype="float32", callback=callback):
        input()     # kayit arka planda surerken Enter'i bekle

    if not chunks:
        return np.zeros(0, dtype=np.float32)
    audio = np.concatenate(chunks).flatten()
    return audio[: int(MAX_RECORD_SEC * SR)]


class VoiceNode(Node):
    def __init__(self, model):
        super().__init__("voice_node")
        self.model = model
        self.pub = self.create_publisher(String, "/voice/command", 10)

    def transcribe(self, audio):
        segments, _ = self.model.transcribe(
            audio, language="tr", initial_prompt=PROMPT, beam_size=5, vad_filter=True,
        )
        return " ".join(s.text.strip() for s in segments)   # generator burada tuketiliyor

    def process_audio(self, audio, gate=None):
        t = time.time()
        text = self.transcribe(audio)
        asr_ms = (time.time() - t) * 1000

        komut, skor = match(normalize(text))
        if gate is not None:                     # surekli dinleme: wake word kapisi
            status, rest = gate.check(text, is_stop=(komut == "STOP"))
            if status == "no_wake":
                self.get_logger().info(f"wake word yok -> yok sayildi  '{text}'")
                return
            if status == "armed":
                self.get_logger().info("dinliyorum... (komutu soyle)")
                return
            if status == "ok":
                komut, skor = match(normalize(rest))
        if komut is None:
            self.get_logger().warn(f"ANLAŞILMADI  '{text}'  ({asr_ms:.0f} ms) -> araca komut gonderilmedi")
            return

        self.pub.publish(String(data=komut))
        self.get_logger().info(f"{komut}  <- '{text}'  (skor {skor:.0f}, {asr_ms:.0f} ms)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ptt", action="store_true", help="bas-konus (Enter) modu, wake word yok")
    ap.add_argument("--wake", default="robot", help="wake word (varsayilan: robot)")
    args = ap.parse_args()

    print(f"Model yükleniyor: {MODEL} / {DEVICE} / {COMPUTE}")
    model = WhisperModel(MODEL, device=DEVICE, compute_type=COMPUTE)
    model.transcribe(np.zeros(SR, dtype=np.float32), language="tr")   # isinma

    rclpy.init()
    node = VoiceNode(model)
    # ROS'u arka planda dondur, ana thread klavye + mikrofonla ilgilensin
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    try:
        if args.ptt:
            print("\nHazır (bas-konuş). Enter'a bas -> konuş -> tekrar Enter.  (Çıkış: Ctrl+C)\n")
            while rclpy.ok():
                input("[Enter] konuşmaya başla...")
                print("  ● Kayıt... (bitince Enter)")
                audio = record_push_to_talk()
                print(f"  Kayıt süresi: {len(audio) / SR:.1f} sn, işleniyor...")
                node.process_audio(audio)
                print()
        else:
            from audio_listener import SpeechListener, WakeGate
            gate = WakeGate([args.wake])
            print(f"\nHazır (sürekli dinleme). Komutlar '{args.wake}' ile başlar: "
                  f"'{args.wake.capitalize()}, atölyeye git'. 'Dur' her zaman geçer.  (Çıkış: Ctrl+C)\n")
            for audio in SpeechListener().utterances():
                if not rclpy.ok():
                    break
                node.process_audio(audio, gate)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
