"""
Turkce cumle -> vinc komutu. ROS/CAN'den bagimsiz (test edilebilir).

    parse("kırmızı kasayı depoya taşı") -> {"cmd": "MOVE", "load": "kirmizi_kasa", "area": "depo"}
    parse("beton bloğu al")              -> {"cmd": "PICK", "load": "beton_blok"}
    parse("atölyeye bırak")              -> {"cmd": "PLACE", "area": "atolye"}
    parse("vinci topla")                 -> {"cmd": "HOME"}
    parse("dur")                         -> {"cmd": "ESTOP"}
    parse("kırmızı kasa nerede")         -> {"cmd": None, "reason": "..."}

Kurallar (TurtleBot tarafiyla ayni felsefe):
    - "dur" cumlenin neresinde olursa olsun -> ESTOP
    - olumsuz fiil ("taşıma", "bırakma") -> hicbir sey
    - fiil sart: yuk/alan adi tek basina hareket ettirmez
    - birden fazla yuk ya da birden fazla hedef alan -> belirsiz, hicbir sey
    - "-dan/-den" ekli alan KAYNAK sayilir: "stoktan kırmızı kasayı al" -> PICK
"""
import os

import yaml
from rapidfuzz import fuzz

HERE = os.path.dirname(os.path.abspath(__file__))
ESIK = 85
EK_PAYI = 6

STOP = {"dur", "durun", "dursun", "durdur", "stop"}
TASI = {"taşı", "taşıyın", "götür", "götürün", "aktar", "aktarın", "naklet"}
KOY = {"koy", "koyun", "bırak", "bırakın", "yerleştir", "yerleştirin", "indir", "indirin"}
AL = {"al", "alın", "kaldır", "kaldırın", "tut", "bağla"}
TOPLA = {"topla", "toplayın", "toplan", "park"}
OLUMSUZ = {"taşıma", "götürme", "koyma", "bırakma", "alma", "kaldırma", "indirme", "yerleştirme"}
KAYNAK_EKLERI = ("dan", "den", "tan", "ten")
# Turkce unsuz yumusamasi: blok -> bloğu, ağaç -> ağacı
YUMUSAMA = {"k": "ğ", "ç": "c", "t": "d", "p": "b"}


def normalize(text):
    import re
    text = text.replace("İ", "i").replace("I", "ı").lower()
    text = re.sub(r"[^\w\s]", " ", text)
    return " ".join(text.split())


def load_names(path=os.path.join(HERE, "sites.yaml")):
    s = yaml.safe_load(open(path, encoding="utf-8"))
    loads = {k: [n.lower().split() for n in v["isimler"]] for k, v in s["yukler"].items()}
    areas = {k: [n.lower().split() for n in v["isimler"]] for k, v in s["alanlar"].items()}
    return loads, areas


def word_match(name_word, word):
    """'kasa' ~ 'kasayı', 'blok' ~ 'bloğu'. Doner: (eslesti_mi, ek)."""
    variants = [name_word]
    if name_word[-1] in YUMUSAMA:
        variants.append(name_word[:-1] + YUMUSAMA[name_word[-1]])
    for v in variants:
        if len(v) <= len(word) <= len(v) + EK_PAYI and fuzz.ratio(v, word[:len(v)]) >= ESIK:
            return True, word[len(v):]
    return False, ""


def find(words, table):
    """Cumledeki varliklari bulur -> {anahtar: son_kelimenin_eki}."""
    found = {}
    for key, names in table.items():
        for name in names:
            n = len(name)
            for i in range(len(words) - n + 1):
                results = [word_match(nw, w) for nw, w in zip(name, words[i:i + n])]
                if all(ok for ok, _ in results):
                    found[key] = results[-1][1]
    return found


LOADS, AREAS = load_names()


def parse(text, loads=None, areas=None):
    loads, areas = loads or LOADS, areas or AREAS
    words = normalize(text).split()
    ws = set(words)
    if not words:
        return {"cmd": None, "reason": "boş"}
    if STOP & ws:
        return {"cmd": "ESTOP"}
    if OLUMSUZ & ws:
        return {"cmd": None, "reason": "olumsuz cümle"}

    found_loads = find(words, loads)
    found_areas = find(words, areas)
    targets = {a for a, ek in found_areas.items() if not ek.startswith(KAYNAK_EKLERI)}

    if len(found_loads) > 1:
        return {"cmd": None, "reason": f"birden fazla yük: {', '.join(found_loads)}"}
    if len(targets) > 1:
        return {"cmd": None, "reason": f"birden fazla hedef alan: {', '.join(targets)}"}
    load = next(iter(found_loads), None)
    area = next(iter(targets), None)

    if TOPLA & ws and not load and not area:
        return {"cmd": "HOME"}
    if load and area and (TASI | KOY) & ws:
        return {"cmd": "MOVE", "load": load, "area": area}
    if load and not area and AL & ws:
        return {"cmd": "PICK", "load": load}
    if area and not load and KOY & ws:
        return {"cmd": "PLACE", "area": area}
    if load and not area and (TASI | KOY) & ws:
        return {"cmd": None, "reason": "nereye? hedef alan yok"}
    return {"cmd": None, "reason": "komut anlaşılmadı (fiil veya yük/alan eksik)"}


# ---------------- Onay cevabi ----------------
EVET = {"evet", "onaylıyorum", "onayla", "onaylı", "onay", "tamam", "olur", "doğru", "eveet"}
HAYIR = {"hayır", "iptal", "vazgeç", "vazgeçtim", "olmaz", "istemiyorum", "yok"}


def parse_answer(text):
    """Onay sorusuna cevap -> 'yes' | 'no' | 'stop' | None (anlasilmadi).
    'dur' her zaman 'stop'; hem evet hem hayir iceren cumle belirsiz sayilir."""
    words = set(normalize(text).split())
    if STOP & words:
        return "stop"
    yes, no = bool(EVET & words), bool(HAYIR & words)
    if yes and not no:
        return "yes"
    if no and not yes:
        return "no"
    return None
