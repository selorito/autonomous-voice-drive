"""
Vinc olculeri + kinematik (ROS'tan bagimsiz, tek kaynak).
gen_world.py dunyayi bu olculerle uretir, crane_controller.py ayni olculerle hareket hesaplar.

Eklemler (q):
    slew  : kule donusu (rad, sinirsiz)
    luff  : bom acisi, yataydan yukari (rad, 0..75 derece)
    tele  : teleskop uzamasi (m, 0..TELE_MAX)
    hoist : halat uzunlugu, makaradan son halat parcasina (m, HOIST_MIN..HOIST_MAX)

Dunya koordinatlari: kamyon orijinde, burnu +x yonunde.
"""
import math

# ---- Kamyon ----
TRUCK_LEN, TRUCK_WID = 9.0, 2.5
DECK_TOP = 1.6                       # ust tabla yuksekligi (kule bunun ustunde)

# ---- Vinc ----
SLEW_POS = (-2.0, 0.0, DECK_TOP)     # kule donme ekseninin tabani (dunya)
PIVOT_FWD = 0.9                      # bom mafsali, kule eksenine yatay uzaklik
PIVOT_UP = 1.2                       # bom mafsali, kule tabanindan yukseklik
H0 = SLEW_POS[2] + PIVOT_UP          # bom mafsalinin yerden yuksekligi
BOOM_BASE_LEN = 7.6                  # tele=0 iken mafsal -> bom ucu
TELE_MAX = 6.0                       # -> bom 7.6 .. 13.6 m
LUFF_MIN, LUFF_MAX = math.radians(0), math.radians(75)
THETA_PREF = math.radians(40)        # tercih edilen calisma acisi

# ---- Halat + kanca ----
N_ROPE = 20                          # halat, birbirine gecen 20 parcadan olusur (gorsel)
ROPE_SEG = 0.6
HOIST_MAX = N_ROPE * ROPE_SEG        # 12 m
HOIST_MIN = 0.8                      # tasima sirasinda kanca bu kadar yukarida
HOOK_DROP = 0.9                      # son halat parcasi -> kanca ucu
SLING = 0.4                          # kanca ucu -> yukun ust yuzeyi (sapan)

# ---- Hizlar (gercekci ama izlenebilir) ----
SPEED = {"slew": 0.25, "luff": 0.15, "tele": 0.6, "hoist": 0.8}   # rad/s veya m/s

HOME = {"slew": 0.0, "luff": math.radians(12), "tele": 0.0, "hoist": 1.5}


class Unreachable(Exception):
    pass


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def tip(q):
    """Bom ucu (makara) dunya konumu."""
    L = BOOM_BASE_LEN + q["tele"]
    r = PIVOT_FWD + L * math.cos(q["luff"])
    return (SLEW_POS[0] + r * math.cos(q["slew"]),
            SLEW_POS[1] + r * math.sin(q["slew"]),
            H0 + L * math.sin(q["luff"]))


def hook_point(q):
    """Kanca ucu dunya konumu (halat her zaman dik)."""
    x, y, z = tip(q)
    return (x, y, z - q["hoist"] - HOOK_DROP)


def boom_above(x, y, current_slew=0.0):
    """Bom ucunu (x, y) noktasinin tam ustune getiren slew/luff/tele.
    Once tercih edilen aciyi dener; bom boyu yetmezse aciyi degistirir."""
    dx, dy = x - SLEW_POS[0], y - SLEW_POS[1]
    rho = math.hypot(dx, dy) - PIVOT_FWD          # mafsaldan yatay mesafe
    if rho <= 0:
        raise Unreachable("hedef vincin cok yakininda")
    L = rho / math.cos(THETA_PREF)
    L = min(max(L, BOOM_BASE_LEN), BOOM_BASE_LEN + TELE_MAX)
    if rho > L:
        raise Unreachable(f"hedef cok uzak ({math.hypot(dx, dy):.1f} m)")
    theta = math.acos(rho / L)
    if not (LUFF_MIN <= theta <= LUFF_MAX):
        raise Unreachable("bom acisi sinir disinda")
    # en kisa yoldan don
    slew = current_slew + wrap(math.atan2(dy, dx) - current_slew)
    return {"slew": slew, "luff": theta, "tele": L - BOOM_BASE_LEN}


def hoist_for(boom, z_hook):
    """Verilen bom pozunda kanca ucunu z_hook yuksekligine indiren halat uzunlugu."""
    h = tip({**boom, "hoist": 0.0})[2] - HOOK_DROP - z_hook
    if not (HOIST_MIN <= h <= HOIST_MAX):
        raise Unreachable(f"halat boyu yetmiyor ({h:.1f} m)")
    return h


def joint_positions(q):
    """Gazebo eklem adlari -> deger. Makara bomun tersine doner ki halat dik kalsin.
    luff_joint +y ekseninde: pozitif aci bomu asagi indirir, bu yuzden eksi."""
    j = {
        "slew_joint": q["slew"],
        "luff_joint": -q["luff"],
        "tele_joint": q["tele"],
        "sheave_joint": q["luff"],
    }
    for i in range(N_ROPE):
        j[f"rope_joint_{i}"] = q["hoist"] / N_ROPE
    return j
