"""
voice_node: mikrofon -> Whisper -> komut esleme -> /voice/command (String)

Bas-konus (push-to-talk) mantigi:
    Enter -> kayit baslar, konus, Enter -> kayit biter ve islenir
    (Sabit 3 sn yerine sadece konustugun kadar kayit -> daha dusuk gecikme)

Calistirma (Gazebo + motion_node acikken):
    python3 voice_node.py
Cikmak icin: Ctrl+C
"""
import re
import threading
import time

import numpy as np
import rclpy
import sounddevice as sd
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
PROMPT = "ileri git, geri git, sola dön, sağa dön, dur"
ESIK = 85

KOMUTLAR = {
    "FORWARD":  ["ileri git", "ileriye git", "ileri gidin", "ilerle", "ileri"],
    "BACKWARD": ["geri git", "geriye git", "geri gidin", "geriye gidin", "geri gel", "geri"],
    "LEFT":     ["sola dön", "sola git", "sol"],
    "RIGHT":    ["sağa dön", "sağa git", "sağ"],
}
STOP_KELIMELERI = {"dur", "durun", "dursun", "durdur", "stop"}
OLUMSUZ = {"gitme", "gitmeyin", "dönme", "dönmeyin", "gelme", "ilerleme"}


# ---- whisper_test.py'deki fonksiyonlar (degismedi) ----
def normalize(text):
    text = text.replace("İ", "i").replace("I", "ı").lower()
    text = re.sub(r"[^\w\s]", " ", text)
    words = text.split()
    words = [w for i, w in enumerate(words) if i == 0 or w != words[i - 1]]
    return " ".join(words)


def match(text, esik=ESIK):
    words = text.split()
    if not words:
        return None, 0
    if STOP_KELIMELERI & set(words):
        return "STOP", 100
    if OLUMSUZ & set(words):
        return None, 0
    bulunan = {}
    for komut, ifadeler in KOMUTLAR.items():
        for ifade in ifadeler:
            n = len(ifade.split())
            for i in range(len(words) - n + 1):
                skor = fuzz.ratio(ifade, " ".join(words[i:i + n]))
                if skor >= esik and skor > bulunan.get(komut, 0):
                    bulunan[komut] = skor
    if len(bulunan) != 1:
        return None, 0
    return next(iter(bulunan.items()))


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

    def process_audio(self, audio):
        t = time.time()
        text = self.transcribe(audio)
        asr_ms = (time.time() - t) * 1000

        komut, skor = match(normalize(text))
        if komut is None:
            self.get_logger().warn(f"ANLAŞILMADI  '{text}'  ({asr_ms:.0f} ms) -> araca komut gonderilmedi")
            return

        self.pub.publish(String(data=komut))
        self.get_logger().info(f"{komut}  <- '{text}'  (skor {skor:.0f}, {asr_ms:.0f} ms)")


def main():
    print(f"Model yükleniyor: {MODEL} / {DEVICE} / {COMPUTE}")
    model = WhisperModel(MODEL, device=DEVICE, compute_type=COMPUTE)
    model.transcribe(np.zeros(SR, dtype=np.float32), language="tr")   # isinma

    rclpy.init()
    node = VoiceNode(model)
    # ROS'u arka planda dondur, ana thread klavye + mikrofonla ilgilensin
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print("\nHazır. Enter'a bas -> konuş -> tekrar Enter.  (Çıkış: Ctrl+C)\n")
    try:
        while rclpy.ok():
            input("[Enter] konuşmaya başla...")
            print("  ● Kayıt... (bitince Enter)")
            audio = record_push_to_talk()
            print(f"  Kayıt süresi: {len(audio) / SR:.1f} sn, işleniyor...")
            node.process_audio(audio)
            print()
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
