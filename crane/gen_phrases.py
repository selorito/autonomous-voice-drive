"""
Vincin soyleyebilecegi tum cumleleri BIR KEZ dogal sesle uretir -> crane/voices/phrases/*.wav
Calisirken (demo) internet gerekmez; tts.py bu kayitlari calar.

Motorlar (--motor):
  elevenlabs  En dogal, asistan tarzi.  export ELEVENLABS_API_KEY=...
              --ses <voice_id>  (ElevenLabs Voice Library'den Turkce bir ses sec, ID'sini kopyala)
  openai      Cok dogal, tarif edilebilir ton.  export OPENAI_API_KEY=...
              --ses coral|nova|alloy|sage|ash|...   --tarz "sakin, net, profesyonel bir asistan"
  edge        Ucretsiz (pip install edge-tts), daha yapay.  --ses tr-TR-AhmetNeural | tr-TR-EmelNeural

    python3 crane/gen_phrases.py --motor openai --ses coral --yeniden
    python3 crane/gen_phrases.py --motor elevenlabs --ses <VOICE_ID> --yeniden

sites.yaml'a yeni yuk/alan ekledikten sonra tekrar calistir (var olanlar yeniden uretilmez).
Not: edge-tts, Microsoft Edge'in sesli okuma servisini kullanan resmi olmayan bir pakettir (demo/portfoy icin).
"""
import argparse
import asyncio
import hashlib
import json
import os
import sys
import tempfile
import urllib.error
import urllib.request
import wave

import numpy as np

CRANE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, CRANE_DIR)
from tts import PHRASES_DIR          # noqa: E402
from voice_crane import all_phrases  # noqa: E402

SR = 24000


def decode_mp3(path, sr=SR):
    """mp3 -> float32 mono (PyAV; faster-whisper ile zaten kurulu)."""
    import av
    resampler = av.AudioResampler(format="s16", layout="mono", rate=sr)
    parts = []
    with av.open(path) as c:
        for frame in c.decode(audio=0):
            parts += [f.to_ndarray() for f in resampler.resample(frame)]
        parts += [f.to_ndarray() for f in resampler.resample(None)]      # kalanlari bosalt
    return np.concatenate(parts, axis=1).reshape(-1).astype(np.float32) / 32768


def save_wav(path, audio):
    pcm = (np.clip(audio, -1, 1) * 32767).astype(np.int16)
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SR)
        w.writeframes(pcm.tobytes())


async def synth(text, voice, rate, mp3_path):
    import edge_tts
    await edge_tts.Communicate(text, voice, rate=rate).save(mp3_path)


def _post(url, headers, body):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), method="POST",
                                 headers={"Content-Type": "application/json", **headers})
    try:
        with urllib.request.urlopen(req, timeout=60) as r:
            return r.read()
    except urllib.error.HTTPError as e:
        sys.exit(f"API hatasi {e.code}: {e.read().decode(errors='ignore')[:300]}")


def pcm16_to_float(raw):
    return np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768


def synth_openai(text, voice, style):
    """OpenAI TTS -> 24 kHz 16-bit mono PCM."""
    key = os.environ.get("OPENAI_API_KEY") or sys.exit("OPENAI_API_KEY tanimli degil")
    raw = _post("https://api.openai.com/v1/audio/speech", {"Authorization": f"Bearer {key}"},
                {"model": "gpt-4o-mini-tts", "voice": voice, "input": text,
                 "instructions": f"Türkçe konuş. {style}", "response_format": "pcm"})
    return pcm16_to_float(raw)


def synth_elevenlabs(text, voice_id):
    """ElevenLabs -> 24 kHz 16-bit mono PCM (cok dilli model, Turkce destekler)."""
    key = os.environ.get("ELEVENLABS_API_KEY") or sys.exit("ELEVENLABS_API_KEY tanimli degil")
    raw = _post(f"https://api.elevenlabs.io/v1/text-to-speech/{voice_id}?output_format=pcm_24000",
                {"xi-api-key": key},
                {"text": text, "model_id": "eleven_multilingual_v2"})   # dil metinden algilanir
    return pcm16_to_float(raw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--motor", default="edge", choices=["edge", "openai", "elevenlabs"])
    ap.add_argument("--ses", default=None, help="ses adi/ID'si (motora gore, yukariya bak)")
    ap.add_argument("--hiz", default="+0%", help="sadece edge: konusma hizi, orn. +10%% ya da -5%%")
    ap.add_argument("--tarz", default="Sakin, net ve kendinden emin, profesyonel bir sesli asistan gibi konuş.",
                    help="sadece openai: konusma tarzi tarifi")
    ap.add_argument("--yeniden", action="store_true", help="hepsini bastan uret (ses degistirince)")
    args = ap.parse_args()
    if args.ses is None:
        args.ses = {"edge": "tr-TR-AhmetNeural", "openai": "coral"}.get(args.motor) or \
            sys.exit("elevenlabs icin --ses <voice_id> gerekli (ElevenLabs Voice Library'den)")

    os.makedirs(PHRASES_DIR, exist_ok=True)
    index_path = os.path.join(PHRASES_DIR, "index.json")
    index = {} if args.yeniden or not os.path.exists(index_path) else json.load(open(index_path, encoding="utf-8"))

    phrases = all_phrases()
    todo = [p for p in phrases if p not in index or not os.path.exists(os.path.join(PHRASES_DIR, index[p]))]
    print(f"{len(phrases)} cümle, {len(todo)} tanesi üretilecek ({args.motor}: {args.ses})")

    with tempfile.TemporaryDirectory() as tmp:
        for i, text in enumerate(todo, 1):
            name = hashlib.sha1(f"{args.motor}|{args.ses}|{args.hiz}|{args.tarz}|{text}".encode()).hexdigest()[:16] + ".wav"
            if args.motor == "openai":
                audio = synth_openai(text, args.ses, args.tarz)
            elif args.motor == "elevenlabs":
                audio = synth_elevenlabs(text, args.ses)
            else:
                mp3 = os.path.join(tmp, "x.mp3")
                asyncio.run(synth(text, args.ses, args.hiz, mp3))
                audio = decode_mp3(mp3)
            save_wav(os.path.join(PHRASES_DIR, name), audio)
            index[text] = name
            print(f"  [{i}/{len(todo)}] {text}")
            with open(index_path, "w", encoding="utf-8") as f:     # her adimda kaydet: yarida kalsa da kaybolmaz
                json.dump(index, f, ensure_ascii=False, indent=1)
    print(f"Hazır: {PHRASES_DIR}")


if __name__ == "__main__":
    main()
