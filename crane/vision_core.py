"""
Kanca kamerasi goruntu isleme (ROS'tan bagimsiz, test edilebilir). vision_node.py bunu kullanir.

Kamera bom ucundaki makaraya bagli, hep TAM ASAGI bakar (makara bomun tersine doner).
Gazebo kamera yonu: goruntunun ustu = makaranin ileri (+x) yonu, goruntunun sagi = makaranin sagi (-y).

Yontem:
  1) Zemin koyu (asfalt/beton), yukler parlak -> parlaklik esigi. Sari/turuncu (boya, vinc, kanca) dislanir.
  2) Kontur -> boyut (kamera yuksekligine gore beklenen piksel alani), sekil (en/boy, doluluk), kenara degmeme filtresi
  3) Sinif: doygun renk -> ton (kirmizi/mavi), doygunsuz -> parlaklik (beyaz/beton). Siniflar sites.yaml renklerinden.
  4) Piksel -> saha koordinati: kamera yuksekligi, yonu ve gorus acisindan (kasadaki yukte kasa yuksekligi hesaba katilir)
"""
import colorsys
import math

import cv2
import numpy as np

import crane_geometry as g

F = (g.CAM_W / 2) / math.tan(g.CAM_HFOV / 2)          # odak uzakligi (piksel)
CX, CY = g.CAM_W / 2, g.CAM_H / 2

BRIGHT_V = 0.38 * 255         # bundan parlak = nesne (zemin ~0.33, golgede ~0.23; beton blok golgede ~0.45)
SAT_COLOR = 0.35 * 255        # bundan doygun = renkli
WHITE_V = 0.85 * 255          # yedek: cevre olculemezse mutlak esik (beton ~0.64, beyaz ~1.0)
WHITE_RATIO = 2.6             # asil kural: yuk / cevre zemin parlakligi. Golgeden bagimsiz:
                              #   beyaz kasa 3.0-3.6, beton blok 1.9-2.3 (gunes ya da golge fark etmez)
DECK_RATIO = 0.82             # kamyon kasasinda: yuk / sari kasa parlakligi (beyaz ~1.0, beton ~0.64)
BG_MIN_V = 0.21 * 255         # oran sadece beton alan / asfalt zeminde (yol daha koyu, ~0.19)


def color_class(rgb):
    """(r, g, b) 0..1 ya da HSV ortalamasindan sinif adi: kirmizi | mavi | beyaz | gri | sari | diger."""
    h, s, v = colorsys.rgb_to_hsv(*rgb)
    return _classify(h * 180, s * 255, v * 255)


def _classify(h, s, v, bg_v=None):
    """bg_v: yukun etrafindaki zemin parlakligi (golgeye karsi beyaz/gri ayrimi icin)."""
    if s >= SAT_COLOR:
        if h < 10 or h > 165:
            return "kirmizi"
        if 95 <= h <= 135:
            return "mavi"
        if 10 <= h <= 40:
            return "sari"
        return "diger"
    if bg_v:
        return "beyaz" if v / bg_v >= WHITE_RATIO else "gri"
    return "beyaz" if v >= WHITE_V else "gri"


def load_classes(sites):
    """sites.yaml yuklerinden: sinif -> [yuk adi], yuk adi -> boyut."""
    classes, sizes = {}, {}
    for name, y in sites["yukler"].items():
        rgb = [float(c) for c in y["renk"].split()[:3]]
        classes.setdefault(color_class(rgb), []).append(name)
        sizes[name] = sites["yuk_tipleri"][y["tip"]]
    return classes, sizes


# ---------- kamera geometrisi ----------
def pixel_to_world(u, v, q, plane_z):
    cx, cy, cz, yaw = g.camera_pose(q)
    Z = cz - plane_z
    fwd = -(v - CY) * Z / F
    left = -(u - CX) * Z / F
    c, s = math.cos(yaw), math.sin(yaw)
    return cx + c * fwd - s * left, cy + s * fwd + c * left


def world_to_pixel(x, y, z, q):
    cx, cy, cz, yaw = g.camera_pose(q)
    Z = cz - z
    if Z <= 0.1:
        return None
    dx, dy = x - cx, y - cy
    c, s = math.cos(yaw), math.sin(yaw)
    fwd, left = c * dx + s * dy, -s * dx + c * dy
    return CX - left * F / Z, CY - fwd * F / Z


# ---------- tespit ----------
def detect(img_rgb, q, sites):
    """Doner: (tespitler, maske). Her tespit: name, cls, x, y, u, v, box(4 kose), err_size."""
    classes, sizes = load_classes(sites)
    hsv = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)
    H, S, V = hsv[..., 0].astype(np.int32), hsv[..., 1], hsv[..., 2]
    yellowish = (S >= SAT_COLOR) & (H >= 10) & (H <= 40)
    mask = ((V >= BRIGHT_V) & ~yellowish).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    cam_z = g.camera_pose(q)[2]
    z_top = g.GROUND_Z + 1.0                              # tipik yuk ustu
    side = 1.2 * F / max(0.5, cam_z - z_top)              # 1.2 m'lik kasanin beklenen kenari (px)
    a_min, a_max = 0.2 * side ** 2, 3.0 * side ** 2

    out = []
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        area = cv2.contourArea(c)
        if not (a_min <= area <= a_max):
            continue
        x, y, w, h = cv2.boundingRect(c)
        if x <= 2 or y <= 2 or x + w >= g.CAM_W - 2 or y + h >= g.CAM_H - 2:
            continue                                      # kenara degiyor: bina/kesik nesne
        rect = cv2.minAreaRect(c)
        rw, rh = rect[1]
        if min(rw, rh) < 1 or max(rw, rh) / min(rw, rh) > 2.2 or area / (rw * rh) < 0.6:
            continue                                      # tabela, cizgi, duzensiz sekil
        m = np.zeros(mask.shape, np.uint8)
        cv2.drawContours(m, [c], -1, 255, -1)
        mh, ms, mv = (float(np.median(ch[m > 0])) for ch in (H, S, V))
        ring = cv2.dilate(m, np.ones((15, 15), np.uint8)) > 0
        ring &= (cv2.dilate(m, np.ones((5, 5), np.uint8)) == 0) & (mask == 0)
        ground, yellow = ring & ~yellowish, ring & yellowish
        if ground.sum() > 40 and np.median(V[ground]) >= BG_MIN_V:          # beton/asfalt zemin uzerinde
            cls = _classify(mh, ms, mv, float(np.median(V[ground])))
        elif yellow.sum() > 40 and ms < SAT_COLOR:                         # sari kamyon kasasi uzerinde
            cls = "beyaz" if mv / float(np.median(V[yellow])) >= DECK_RATIO else "gri"
        else:
            cls = _classify(mh, ms, mv)                                    # mutlak esik
        names = classes.get(cls, [])
        if len(names) != 1:
            continue                                      # tanimsiz ya da belirsiz renk
        name = names[0]
        mom = cv2.moments(c)
        u, v = mom["m10"] / mom["m00"], mom["m01"] / mom["m00"]
        top = g.GROUND_Z + sizes[name][2]
        wx, wy = pixel_to_world(u, v, q, top)
        dx, dy = g.deck_xy(q["bx"])
        if math.hypot(wx - dx, wy - dy) < 1.5:            # kamyon kasasinda: daha yuksekte
            top = g.DECK_TOP + sizes[name][2]
            wx, wy = pixel_to_world(u, v, q, top)
        exp_side = min(sizes[name][:2]) * F / max(0.5, cam_z - top)
        out.append({"name": name, "cls": cls, "x": round(wx, 3), "y": round(wy, 3),
                    "u": u, "v": v, "box": cv2.boxPoints(rect).tolist(),
                    "size_ratio": round(min(rw, rh) / exp_side, 2)})
    best = {}                                             # ayni yuk iki kez cikarsa: boyutu en uygun olan
    for d in out:
        if d["name"] not in best or abs(d["size_ratio"] - 1) < abs(best[d["name"]]["size_ratio"] - 1):
            best[d["name"]] = d
    return list(best.values()), mask


# ---------- gorsellestirme ----------
CLS_BGR = {"kirmizi": (60, 60, 255), "mavi": (255, 140, 40), "beyaz": (240, 240, 240), "gri": (180, 180, 180)}


def annotate(img_rgb, q, dets, state_text=""):
    """Etiketli goruntu (BGR, cv2.imshow icin)."""
    img = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    hx, hy, _ = g.hook_point(q)
    p = world_to_pixel(hx, hy, g.GROUND_Z, q)              # kancanin zemindeki izdusumu = hedef
    if p is not None:
        u, v = int(p[0]), int(p[1])
        cv2.circle(img, (u, v), 18, (0, 255, 255), 2)
        cv2.line(img, (u - 30, v), (u - 8, v), (0, 255, 255), 2)
        cv2.line(img, (u + 8, v), (u + 30, v), (0, 255, 255), 2)
        cv2.line(img, (u, v - 30), (u, v - 8), (0, 255, 255), 2)
        cv2.line(img, (u, v + 8), (u, v + 30), (0, 255, 255), 2)
    for d in dets:
        col = CLS_BGR.get(d["cls"], (0, 255, 0))
        box = np.array(d["box"], np.int32)
        cv2.polylines(img, [box], True, (0, 255, 0), 2)
        x0, y0 = box[:, 0].min(), box[:, 1].min()
        label = f"{d['name']}  ({d['x']:.1f}, {d['y']:.1f})"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(img, (x0, y0 - th - 8), (x0 + tw + 6, y0), (0, 0, 0), -1)
        cv2.putText(img, label, (x0 + 3, y0 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
    cam_z = g.camera_pose(q)[2]
    bar = f"KANCA KAMERASI | yukseklik {cam_z:.1f} m | {len(dets)} yuk | {state_text}"
    cv2.rectangle(img, (0, 0), (g.CAM_W, 24), (0, 0, 0), -1)
    cv2.putText(img, bar, (8, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
    return img
