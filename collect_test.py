"""
Test verisi toplama: ekranda yazan cumleyi soyle -> kaydet -> tahmin et -> CSV'ye yaz.
ROS gerektirmez. voice_node.py'deki fonksiyonlari kullanir (ayni mantik test edilir).

Kullanim:
    python3 collect_test.py --speaker selman --noise sessiz
    python3 collect_test.py --speaker ahmet  --noise muzik
Her kisi icin ~3 dakika. Sonuclar: results.csv + recordings/*.wav
"""
import argparse
import csv
import os
import random
import time
import wave

import numpy as np
from faster_whisper import WhisperModel

from voice_node import (MODEL, DEVICE, COMPUTE, SR, PROMPT,
                        normalize, match, record_push_to_talk)

# (soylenecek cumle, beklenen komut)  -- None = arac hareket etmemeli
TEST_SET = [
    ("ileri git", "FORWARD"), ("ilerle", "FORWARD"),
    ("geri git", "BACKWARD"), ("geriye git", "BACKWARD"),
    ("sola dön", "LEFT"), ("sola dön", "LEFT"),
    ("sağa dön", "RIGHT"), ("sağa dön", "RIGHT"),
    ("dur", "STOP"), ("dur", "STOP"),
    # Hedefe gitme
    ("atölyeye git", "GOTO atolye"), ("depoya git", "GOTO depo"), ("merkeze dön", "GOTO merkez"),
    # Tuzaklar
    ("bekle biraz, dur", "STOP"),
    ("sakın ileri gitme", None),
    ("sola dön sonra ileri git", None),
    ("atölye nerede", None),
    # Alakasiz cumleler
    ("bugün hava çok güzel", None),
    ("durum raporu ver", None),
    ("saat kaç oldu", None),
]

CSV_PATH = "results.csv"
REC_DIR = "recordings"
FIELDS = ["time", "speaker", "noise", "model", "prompt", "expected", "heard",
          "predicted", "score", "correct", "asr_ms", "audio_sec", "wav"]


def save_wav(path, audio):
    pcm = (np.clip(audio, -1, 1) * 32767).astype(np.int16)
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SR)
        w.writeframes(pcm.tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speaker", required=True)
    ap.add_argument("--noise", default="sessiz", help="sessiz / muzik / motor / ...")
    args = ap.parse_args()

    os.makedirs(REC_DIR, exist_ok=True)
    new_file = not os.path.exists(CSV_PATH)

    print(f"Model yükleniyor: {MODEL}")
    model = WhisperModel(MODEL, device=DEVICE, compute_type=COMPUTE)
    model.transcribe(np.zeros(SR, dtype=np.float32), language="tr")  # isinma

    items = TEST_SET[:]
    random.shuffle(items)   # siralamayi ezberlemesinler

    with open(CSV_PATH, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        if new_file:
            writer.writeheader()

        for i, (prompt, expected) in enumerate(items, 1):
            input(f"\n[{i}/{len(items)}] Söyle: \"{prompt}\"   (Enter -> konuş -> Enter)")
            print("  ● Kayıt...")
            audio = record_push_to_talk()

            t = time.time()
            segs, _ = model.transcribe(audio, language="tr", initial_prompt=PROMPT,
                                       beam_size=5, vad_filter=True)
            heard = " ".join(s.text.strip() for s in segs)
            asr_ms = (time.time() - t) * 1000

            predicted, score = match(normalize(heard))
            correct = predicted == expected

            wav_name = f"{args.speaker}_{args.noise}_{int(time.time())}_{i}.wav"
            save_wav(os.path.join(REC_DIR, wav_name), audio)

            writer.writerow({
                "time": time.strftime("%Y-%m-%d %H:%M:%S"), "speaker": args.speaker,
                "noise": args.noise, "model": MODEL, "prompt": prompt,
                "expected": expected or "NONE", "heard": heard,
                "predicted": predicted or "NONE", "score": f"{score:.0f}",
                "correct": int(correct), "asr_ms": f"{asr_ms:.0f}",
                "audio_sec": f"{len(audio) / SR:.2f}", "wav": wav_name,
            })
            f.flush()
            mark = "✓" if correct else "✗"
            print(f"  {mark} duyulan='{heard}'  tahmin={predicted or 'NONE'}  "
                  f"beklenen={expected or 'NONE'}  ({asr_ms:.0f} ms)")

    print(f"\nBitti. Sonuçlar {CSV_PATH} dosyasına eklendi. Rapor: python3 report.py")


if __name__ == "__main__":
    main()
