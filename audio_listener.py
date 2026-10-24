"""
Ortak dinleme modulu: SUREKLI mikrofon -> Silero VAD -> konusma parcalari -> (Whisper) -> wake word kapisi.
Hem TurtleBot kumandasi (voice_node.py, ROS) hem vinc kumandasi (crane/voice_crane.py, CAN) kullanir.

    listener = SpeechListener()
    for audio in listener.utterances():       # her biri: bir konusmanin 16 kHz float32 sesi
        text = whisper(audio)
        ok, komut = gate.check(text, is_stop=...)

Silero VAD modeli faster-whisper'in icinde geliyor (ek kurulum yok). 32 ms'lik parcalarla calisir,
LSTM durumu (h, c) parcalar arasinda tasinir -> gercek zamanli.
"""
import os
import queue
import re
import time
from collections import deque

import numpy as np
from rapidfuzz import fuzz

SR = 16000
CHUNK = 512                      # 32 ms (Silero 16 kHz'de 512 ornek ister)
CONTEXT = 64                     # her parcanin onune onceki parcanin son 64 ornegi eklenir


def _vad_model_path():
    import faster_whisper
    assets = os.path.join(os.path.dirname(faster_whisper.__file__), "assets")
    onnx = [f for f in os.listdir(assets) if f.startswith("silero") and f.endswith(".onnx")]
    if not onnx:
        raise FileNotFoundError(f"Silero VAD modeli bulunamadi: {assets}")
    return os.path.join(assets, sorted(onnx)[-1])


class StreamingVAD:
    """Parca parca calisan Silero VAD. Her 512 ornekte bir konusma olasiligi (0..1) dondurur."""

    def __init__(self):
        import onnxruntime
        opts = onnxruntime.SessionOptions()
        opts.inter_op_num_threads = 1
        opts.intra_op_num_threads = 1
        opts.log_severity_level = 4
        self.session = onnxruntime.InferenceSession(_vad_model_path(), providers=["CPUExecutionProvider"],
                                                    sess_options=opts)
        self.reset()

    def reset(self):
        self.h = np.zeros((1, 1, 128), dtype=np.float32)
        self.c = np.zeros((1, 1, 128), dtype=np.float32)
        self.context = np.zeros(CONTEXT, dtype=np.float32)

    def __call__(self, chunk):
        x = np.concatenate([self.context, chunk.astype(np.float32)])[None, :]
        prob, self.h, self.c = self.session.run(None, {"input": x, "h": self.h, "c": self.c})
        self.context = chunk[-CONTEXT:].astype(np.float32)
        return float(np.asarray(prob).reshape(-1)[0])


class Segmenter:
    """VAD olasiliklarindan konusma baslangic/bitisini bulur (histerezisli durum makinesi).

    threshold     : konusma basladi esigi
    neg_threshold : konusma bitti esigi (daha dusuk -> kelime aralarindaki kisa dususlerde kesmez)
    min_silence_ms: bu kadar sessizlik -> konusma bitti
    min_speech_ms : bundan kisa sesler (oksuruk, tik) atilir
    pre_roll_ms   : baslangictan once tutulan ses (ilk hece kesilmesin)
    max_speech_s  : bundan uzun konusma zorla kesilir
    """

    def __init__(self, threshold=0.5, neg_threshold=0.35, min_silence_ms=600, min_speech_ms=250,
                 pre_roll_ms=300, max_speech_s=8.0):
        ms = 1000 * CHUNK / SR
        self.threshold, self.neg_threshold = threshold, neg_threshold
        self.min_silence = int(min_silence_ms / ms)
        self.min_speech = int(min_speech_ms / ms)
        self.max_chunks = int(max_speech_s * 1000 / ms)
        self.pre_roll = deque(maxlen=int(pre_roll_ms / ms))
        self.vad = StreamingVAD()
        self._reset()

    def _reset(self):
        self.speaking = False
        self.buf = []
        self.speech_chunks = 0
        self.silence = 0

    def push(self, chunk):
        """Bir 512'lik parca ver. Bir konusma bittiyse sesini dondurur, yoksa None."""
        p = self.vad(chunk)
        if not self.speaking:
            self.pre_roll.append(chunk)
            if p >= self.threshold:
                self.speaking = True
                self.buf = list(self.pre_roll)
                self.pre_roll.clear()
                self.speech_chunks, self.silence = 1, 0
            return None

        self.buf.append(chunk)
        if p >= self.neg_threshold:
            self.speech_chunks += 1
            self.silence = 0
        else:
            self.silence += 1

        ended = self.silence >= self.min_silence
        too_long = len(self.buf) >= self.max_chunks
        if not (ended or too_long):
            return None
        audio = np.concatenate(self.buf[: len(self.buf) - max(0, self.silence - 3)])   # sondaki sessizligi kirp
        keep = self.speech_chunks >= self.min_speech
        self._reset()
        return audio if keep else None


class SpeechListener:
    """Mikrofonu surekli dinler, her konusmayi ayri bir ses parcasi olarak verir."""

    def __init__(self, **segmenter_kwargs):
        self.segmenter = Segmenter(**segmenter_kwargs)
        self.q = queue.Queue()
        self.paused = False

    def _callback(self, indata, frames, t, status):
        if not self.paused:
            self.q.put(indata[:, 0].copy())

    def utterances(self, idle=False):
        """Konusmalari verir. idle=True ise konusma yokken ~0.1 sn'de bir None verir
        (cagiran taraf zaman asimi kontrol edebilsin, orn. onay beklerken)."""
        import sounddevice as sd
        with sd.InputStream(samplerate=SR, channels=1, dtype="float32", blocksize=CHUNK,
                            callback=self._callback):
            while True:
                try:
                    chunk = self.q.get(timeout=0.1)
                except queue.Empty:
                    if idle:
                        yield None
                    continue
                if len(chunk) != CHUNK:
                    continue
                audio = self.segmenter.push(chunk)
                if audio is not None:
                    yield audio

    def drain(self):
        """Isleme sirasinda biriken sesi at (orn. sistemin kendi sesli cevabi duyulmasin)."""
        while not self.q.empty():
            self.q.get_nowait()

    def reset(self):
        """Yarim kalmis konusmayi ve VAD durumunu sifirla (sistem konustuktan sonra)."""
        self.drain()
        self.segmenter._reset()
        self.segmenter.pre_roll.clear()
        self.segmenter.vad.reset()


def utterances_from_array(audio, **segmenter_kwargs):
    """Test icin: bir ses dosyasini mikrofondan geliyormus gibi parca parca isler.
    Doner: [(baslangic_sn, bitis_sn, ses), ...]"""
    seg = Segmenter(**segmenter_kwargs)
    out, n = [], len(audio) // CHUNK
    for i in range(n):
        a = seg.push(audio[i * CHUNK:(i + 1) * CHUNK])
        if a is not None:
            end = (i + 1) * CHUNK / SR
            out.append((end - len(a) / SR, end, a))
    return out


# ---------------- Wake word kapisi ----------------

def _norm(text):
    text = text.replace("İ", "i").replace("I", "ı").lower()
    return re.sub(r"[^\w\s]", " ", text).split()


class WakeGate:
    """Komut sadece wake word ile baslarsa gecer: "toros, kırmızı kasayı depoya taşı".

    - "dur" (is_stop=True) wake word ARANMADAN gecer: acil durdurma her zaman dinlenir.
    - Sadece wake word soylenirse ("vinç") ARM_SEC saniye boyunca sonraki cumle wake word'suz kabul edilir.
    - Wake word ilk 3 kelime icinde aranir ("tamam vinç ..."), ekleri tolere edilir ("vince", "robotum").
    """
    ARM_SEC = 6.0
    SEARCH_WORDS = 3

    YUMUSAMA = {"k": "ğ", "ç": "c", "t": "d", "p": "b"}     # vinç -> vince, robot -> robodu

    def __init__(self, wake_words, esik=80):
        self.wake = []
        for w in (w.lower() for w in wake_words):
            self.wake.append(w)
            if w[-1] in self.YUMUSAMA:
                self.wake.append(w[:-1] + self.YUMUSAMA[w[-1]])
        self.esik = esik
        self.armed_until = 0.0

    def _wake_index(self, words):
        for i, w in enumerate(words[: self.SEARCH_WORDS]):
            for ww in self.wake:
                if len(ww) <= len(w) <= len(ww) + 3 and fuzz.ratio(ww, w[:len(ww)]) >= self.esik:
                    return i
        return None

    def check(self, text, is_stop=False, now=None):
        """Doner: (durum, komut_metni). durum: 'ok' | 'armed' | 'stop' | 'no_wake'."""
        now = time.monotonic() if now is None else now
        if is_stop:
            self.armed_until = 0.0
            return "stop", text
        words = _norm(text)
        i = self._wake_index(words)
        if i is not None:
            rest = " ".join(words[i + 1:])
            if not rest:
                self.armed_until = now + self.ARM_SEC
                return "armed", ""
            self.armed_until = 0.0
            return "ok", rest
        if now < self.armed_until:
            self.armed_until = 0.0
            return "ok", " ".join(words)
        return "no_wake", ""
