"""
Vinc olculeri + kinematik (ROS'tan bagimsiz, tek kaynak).
gen_world.py dunyayi bu olculerle uretir, crane_controller.py ve vision_node.py ayni olculeri kullanir.

Durum (q):
    bx     : kamyonun yol uzerindeki konumu (m). Yol dunya x ekseni boyunca (y=0), kamyon +x'e bakar.
    outrig : destek ayaklari 0 = toplu (surus), 1 = acik (kaldirma)
    slew   : kule donusu, kamyona gore (rad, sinirsiz)
    luff   : bom acisi, yataydan yukari (rad, 0..75 derece)
    tele   : teleskop uzamasi (m, 0..TELE_MAX)
    hoist  : halat uzunlugu, makaradan son halat parcasina (m, HOIST_MIN..HOIST_MAX)
"""
import math

# ---- Kamyon ----
DECK_TOP = 1.6                       # ust tabla yuksekligi
GROUND_Z = 0.03                      # asfalt yuzeyi (yukler bunun ustune konur)
WHEEL_R = 0.5
AXLES_X = [3.2, 1.9, -1.6, -3.0]     # aks konumlari (kamyona gore)
OUTRIG_X = [2.4, -4.0]               # destek ayaklari (on, arka)
OUTRIG_STROKE = 1.2                  # ayak acilma mesafesi (m, her yana)
DECK_SLOT = (1.1, 0.0)               # kamyon kasasinda yuk yeri (kamyona gore x, y)

# ---- Vinc ----
SLEW_REL = (-2.0, 0.0)               # kule donme ekseni, kamyona gore (x, y)
PIVOT_FWD = 0.9                      # bom mafsali, kule eksenine yatay uzaklik
PIVOT_UP = 1.2                       # bom mafsali, kule tabanindan yukseklik
H0 = DECK_TOP + PIVOT_UP             # bom mafsalinin yerden yuksekligi
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

# ---- Kanca kamerasi (makara bloguna bagli, hep asagi bakar) ----
CAM_OFFSET = (0.0, 0.6, -0.3)        # makaraya gore (x ileri, y sol, z): halatin yanina takili
CAM_W, CAM_H = 640, 480
CAM_HFOV = 1.2                       # rad (~69 derece)

# ---- Hizlar (gercekci ama izlenebilir) ----
SPEED = {"bx": 3.0, "outrig": 0.6, "slew": 0.25, "luff": 0.15, "tele": 0.6, "hoist": 0.8}

STOW = {"slew": 0.0, "luff": math.radians(8), "tele": 0.0, "hoist": HOIST_MIN}   # surus pozu
HOME = {"bx": 0.0, "outrig": 0.0, **STOW, "hoist": 1.5}


class Unreachable(Exception):
    pass


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def slew_axis(bx):
    """Kule donme ekseninin dunya (x, y) konumu."""
    return bx + SLEW_REL[0], SLEW_REL[1]


def tip(q):
    """Bom ucu (makara) dunya konumu."""
    L = BOOM_BASE_LEN + q["tele"]
    r = PIVOT_FWD + L * math.cos(q["luff"])
    sx, sy = slew_axis(q["bx"])
    return (sx + r * math.cos(q["slew"]), sy + r * math.sin(q["slew"]), H0 + L * math.sin(q["luff"]))


def hook_point(q):
    """Kanca ucu dunya konumu (halat her zaman dik)."""
    x, y, z = tip(q)
    return (x, y, z - q["hoist"] - HOOK_DROP)


def camera_pose(q):
    """Kanca kamerasinin dunya konumu ve yonu (yaw). Kamera tam asagi bakar."""
    x, y, z = tip(q)
    c, s = math.cos(q["slew"]), math.sin(q["slew"])
    ox, oy, oz = CAM_OFFSET
    return x + c * ox - s * oy, y + s * ox + c * oy, z + oz, q["slew"]


def deck_xy(bx):
    """Kamyon kasasindaki yuk yerinin dunya konumu."""
    return bx + DECK_SLOT[0], DECK_SLOT[1]


def boom_above(x, y, current_slew=0.0, bx=0.0):
    """Kamyon bx'teyken bom ucunu (x, y) noktasinin tam ustune getiren slew/luff/tele.
    Once tercih edilen aciyi dener; bom boyu yetmezse aciyi degistirir."""
    sx, sy = slew_axis(bx)
    dx, dy = x - sx, y - sy
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
    slew = current_slew + wrap(math.atan2(dy, dx) - current_slew)   # en kisa yoldan don
    return {"slew": slew, "luff": theta, "tele": L - BOOM_BASE_LEN}


def hoist_for(boom, z_hook, bx=0.0):
    """Verilen bom pozunda kanca ucunu z_hook yuksekligine indiren halat uzunlugu."""
    h = tip({**boom, "hoist": 0.0, "bx": bx})[2] - HOOK_DROP - z_hook
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
    for i in range(len(AXLES_X)):                  # tekerlekler: yuvarlanma acisi = yol / yaricap
        j[f"wheel_joint_{i}"] = q["bx"] / WHEEL_R
    for i in range(len(OUTRIG_X)):
        for side in ("l", "r"):
            j[f"outrig_joint_{i}{side}"] = q["outrig"] * OUTRIG_STROKE
    return j
