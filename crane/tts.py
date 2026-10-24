"""
Sesli geri bildirim. Oncelik sirasi:
  1) Hazir kayitli cumleler (crane/voices/phrases/): gen_phrases.py ile BIR KEZ uretilir (dogal ses),
     calisirken internet GEREKMEZ.
  2) Piper TTS (Turkce, internetsiz): listede olmayan cumleler icin.
  3) Hicbiri yoksa: sadece ekrana yazar (COKMEZ).

Kurulum:
    python3 crane/gen_phrases.py                       # dogal ses (bir kez, internet gerekir)
    pip install piper-tts                              # yedek ses
    python3 -m piper.download_voices --download-dir crane/voices tr_TR-dfki-medium
Konusurken mikrofonu durdurur, bitince biriken sesi atar -> sistem kendi sesini komut sanmaz.
"""
import json
import os
import time
import wave

import numpy as np

CRANE_DIR = os.path.dirname(os.path.abspath(__file__))
VOICES_DIR = os.path.join(CRANE_DIR, "voices")
DEFAULT_VOICE = "tr_TR-dfki-medium"
PHRASES_DIR = os.path.join(VOICES_DIR, "phrases")


def load_wav(path):
    with wave.open(path) as w:
        pcm = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16)
        return pcm.astype(np.float32) / 32768, w.getframerate()


class Speaker:
    def __init__(self, voice=DEFAULT_VOICE, enabled=True, listener=None):
        self.voice = None
        self.listener = listener                 # konusurken durdurulacak SpeechListener
        self.cache = {}                          # sik cumleler bir kez sentezlensin
        self.phrases = {}                        # hazir kayitlar: cumle -> wav dosyasi
        if not enabled:
            return
        index = os.path.join(PHRASES_DIR, "index.json")
        if os.path.exists(index):
            self.phrases = json.load(open(index, encoding="utf-8"))
            print(f"  (hazır ses kayıtları: {len(self.phrases)} cümle)")
        path = os.path.join(VOICES_DIR, f"{voice}.onnx")
        try:
            from piper import PiperVoice
        except ImportError:
            if not self.phrases:
                print("  (TTS kapali: 'pip install piper-tts' kurulu degil -> cevaplar sadece ekrana yazilir)")
            return
        if not os.path.exists(path):
            if not self.phrases:
                print(f"  (TTS kapali: ses dosyasi yok: {path}\n"
                      f"   indirmek icin: python3 -m piper.download_voices --download-dir crane/voices {voice})")
            return
        self.voice = PiperVoice.load(path)

    def synth(self, text):
        """Doner: (ses, ornekleme_hizi) ya da None."""
        if text in self.phrases:                 # 1) hazir dogal ses kaydi
            if text not in self.cache:
                self.cache[text] = load_wav(os.path.join(PHRASES_DIR, self.phrases[text]))
            return self.cache[text]
        if self.voice is None:                   # 3) ses yok
            return None
        if text not in self.cache:               # 2) Piper
            chunks = list(self.voice.synthesize(text))
            audio = np.concatenate([c.audio_float_array for c in chunks]).astype(np.float32)
            self.cache[text] = (audio, chunks[0].sample_rate)
        return self.cache[text]

    def say(self, text):
        print(f"  🔊 {text}")
        out = self.synth(text)
        if out is None:
            return
        import sounddevice as sd
        audio, sr = out
        if self.listener is not None:
            self.listener.paused = True          # kendi sesimizi dinlemeyelim
        try:
            sd.play(audio, sr)
            sd.wait()
            time.sleep(0.15)                     # hoparlor yankisi sonmesi icin
        finally:
            if self.listener is not None:
                self.listener.reset()
                self.listener.paused = False
