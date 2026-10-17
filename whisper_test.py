"""
Whisper + komut esleme testi: mikrofondan 3 sn kaydet -> metne cevir -> komuta esle.
Kullanim:
    python whisper_test.py              # mikrofonla canli test
    python whisper_test.py kayit.wav    # hazir ses dosyasiyla test
Cikmak icin: Ctrl+C
"""
import re
import sys
import time

import numpy as np
import sounddevice as sd
from faster_whisper import WhisperModel
from rapidfuzz import fuzz

# ---- Ayarlar ----
MODEL = "small"     # tiny / base / small  (once small dene, yavassa base)
DEVICE = "cpu"      # NVIDIA GPU varsa "cuda"
COMPUTE = "int8"    # GPU'da "float16"
SR = 16000          # Whisper 16 kHz mono bekler
RECORD_SEC = 3
PROMPT = "ileri git, geri git, sola dön, sağa dön, dur"   # karsilastirma icin None da dene
ESIK = 85           # pencere skoru bunun altindaysa komut sayilmaz

# ---- Hareket komutlari: her komut icin es anlamli ifadeler ----
KOMUTLAR = {
    "FORWARD":  ["ileri git", "ileriye git", "ileri gidin", "ilerle", "ileri"],
    "BACKWARD": ["geri git", "geriye git", "geri gidin", "geriye gidin", "geri gel", "geri"],
    "LEFT":     ["sola dön", "sola git", "sol"],
    "RIGHT":    ["sağa dön", "sağa git", "sağ"],
}
# STOP ayri tutulur: cumlenin neresinde gecerse gecsin STOP (birebir kelime, fuzzy degil -> "durum" yakalanmaz)
STOP_KELIMELERI = {"dur", "durun", "dursun", "durdur", "stop"}
# Olumsuz fiil varsa hareket komutu verilmez ("sakin ileri gitme")
OLUMSUZ = {"gitme", "gitmeyin", "dönme", "dönmeyin", "gelme", "ilerleme"}

# ---- Model yukleme (ilk seferde indirir) ----
print(f"Model yükleniyor: {MODEL} / {DEVICE} / {COMPUTE}")
t0 = time.time()
model = WhisperModel(MODEL, device=DEVICE, compute_type=COMPUTE)
print(f"Yüklendi: {time.time() - t0:.1f} sn\n")


def transcribe(audio):
    t = time.time()
    segments, info = model.transcribe(
        audio,
        language="tr",
        initial_prompt=PROMPT,
        beam_size=5,
        vad_filter=True,    # sessiz kisimlari at, model sadece konusmayi gorsun
    )
    # DIKKAT: segments bir generator. Asil hesaplama asagidaki satirda yapilir,
    # gecikmeyi bu satirdan SONRA olcmek gerekir.
    segments = list(segments)
    dt_ms = (time.time() - t) * 1000

    text = " ".join(s.text.strip() for s in segments)
    # Guven gostergeleri: avg_logprob 0'a yakinsa model emin; no_speech_prob yuksekse muhtemelen konusma yok
    logprob = np.mean([s.avg_logprob for s in segments]) if segments else float("nan")
    no_speech = max((s.no_speech_prob for s in segments), default=float("nan"))
    return text, dt_ms, logprob, no_speech


def normalize(text):
    text = text.replace("İ", "i").replace("I", "ı").lower()   # Turkce buyuk/kucuk harf tuzagi
    text = re.sub(r"[^\w\s]", " ", text)                      # noktalama -> bosluk
    words = text.split()
    words = [w for i, w in enumerate(words) if i == 0 or w != words[i - 1]]  # "dur dur dur" -> "dur"
    return " ".join(words)


def match(text, esik=ESIK):
    words = text.split()
    if not words:
        return None, 0

    # 1) STOP her seyden once: cumlenin herhangi bir yerinde gecerse gerisine bakma
    if STOP_KELIMELERI & set(words):
        return "STOP", 100

    # 2) Olumsuz fiil varsa hareket etme
    if OLUMSUZ & set(words):
        return None, 0

    # 3) Hareket: her ifadeyi cumledeki ayni uzunluktaki kelime pencereleriyle karsilastir
    #    "ileri git siviya cikalim" -> pencereler: "ileri git", "git siviya", "siviya cikalim"
    bulunan = {}
    for komut, ifadeler in KOMUTLAR.items():
        for ifade in ifadeler:
            n = len(ifade.split())
            for i in range(len(words) - n + 1):
                pencere = " ".join(words[i:i + n])
                skor = fuzz.ratio(ifade, pencere)
                if skor >= esik and skor > bulunan.get(komut, 0):
                    bulunan[komut] = skor

    # Hic komut yok ya da birden fazla FARKLI komut var -> belirsiz, hicbir sey yapma
    if len(bulunan) != 1:
        return None, 0
    komut, skor = next(iter(bulunan.items()))
    return komut, skor


def show(text, dt_ms, logprob, no_speech):
    temiz = normalize(text)
    komut, skor = match(temiz)
    print(f"  Metin      : '{text}'")
    print(f"  Temiz      : '{temiz}'")
    print(f"  Gecikme    : {dt_ms:.0f} ms")
    print(f"  avg_logprob: {logprob:.2f}   no_speech_prob: {no_speech:.2f}")
    print(f"  Komut      : {komut or 'ANLAŞILMADI'}  (skor {skor:.0f})\n")


if len(sys.argv) > 1:
    show(*transcribe(sys.argv[1]))
else:
    # Ilk cagri her zaman yavas olur (isinma). Olcumleri bozmasin diye bos sesle bir kez calistir.
    transcribe(np.zeros(SR, dtype=np.float32))

    while True:
        input(f"Enter'a bas ve {RECORD_SEC} sn konuş...")
        audio = sd.rec(int(RECORD_SEC * SR), samplerate=SR, channels=1, dtype="float32")
        sd.wait()
        show(*transcribe(audio.flatten()))
