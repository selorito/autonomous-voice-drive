"""
crane_geometry.py + sites.yaml -> worlds/crane_site.world (Gazebo Classic SDF 1.6)

Olculeri veya sahneyi degistirdiginde tekrar calistir:
    python3 gen_world.py
"""
import math
import os

import yaml

import crane_geometry as g

HERE = os.path.dirname(os.path.abspath(__file__))
SITES = yaml.safe_load(open(os.path.join(HERE, "sites.yaml"), encoding="utf-8"))

# Renkler. Kanca kamerasi icin: zemin KOYU, yukler parlak/renkli; sahada beyaz/doymus kirmizi-mavi yok.
CRANE_COLOR = "0.93 0.74 0.28 1"     # MPG tarzi hardal sarisi
CRANE_DARK = "0.75 0.55 0.15 1"
CAB_COLOR = "0.90 0.70 0.25 1"
DARK = "0.12 0.12 0.12 1"
DARK_GREY = "0.25 0.25 0.27 1"
GREY = "0.40 0.40 0.42 1"
RIM = "0.55 0.55 0.58 1"
GLASS = "0.15 0.22 0.32 1"
WOOD = "0.55 0.38 0.20 1"
LIGHT = "0.95 0.90 0.60 1"
BEACON = "0.95 0.55 0.05 1"
ASPHALT = "0.22 0.22 0.23 1"
ROAD = "0.15 0.15 0.16 1"
CONCRETE = "0.26 0.26 0.27 1"          # aydinlatmayla ~0.33: kamera esiginin (0.45) altinda kalsin
PAINT = "0.95 0.80 0.10 1"            # sari zemin boyasi
DOOR = "0.30 0.32 0.35 1"
FENCE = "0.35 0.40 0.35 1"


def mat(rgba):
    return f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>"


def fmt_pose(pose):
    """SDF <pose> her zaman 6 deger ister: x y z roll pitch yaw."""
    pose = tuple(pose) + (0,) * (6 - len(pose))
    return " ".join(f"{v:g}" for v in pose)


def box(name, pose, size, rgba, collision=False):
    p = fmt_pose(pose)
    s = " ".join(f"{v:g}" for v in size)
    out = f"""
        <visual name="{name}"><pose>{p}</pose>
          <geometry><box><size>{s}</size></box></geometry>{mat(rgba)}</visual>"""
    if collision:
        out += f"""
        <collision name="{name}_col"><pose>{p}</pose>
          <geometry><box><size>{s}</size></box></geometry></collision>"""
    return out


def cyl(name, pose, radius, length, rgba):
    p = fmt_pose(pose)
    return f"""
        <visual name="{name}"><pose>{p}</pose>
          <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>{mat(rgba)}</visual>"""


def link(name, pose, body, gravity=True, mass=1.0, kinematic=False):
    p = fmt_pose(pose)
    return f"""
      <link name="{name}">
        <pose>{p}</pose>
        <kinematic>{1 if kinematic else 0}</kinematic>
        <gravity>{1 if gravity else 0}</gravity>
        <self_collide>0</self_collide>
        <inertial><mass>{mass}</mass>
          <inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>{body}
      </link>"""


def joint(name, jtype, parent, child, axis=None, lower=None, upper=None):
    ax = ""
    if axis:
        lim = ""
        if lower is not None:
            lim = f"<limit><lower>{lower:g}</lower><upper>{upper:g}</upper><effort>1e6</effort><velocity>10</velocity></limit>"
        ax = f"<axis><xyz>{' '.join(f'{v:g}' for v in axis)}</xyz>{lim}</axis>"
    return f"""
      <joint name="{name}" type="{jtype}"><parent>{parent}</parent><child>{child}</child>{ax}</joint>"""


def sphere(name, pose, radius, rgba):
    return f"""
        <visual name="{name}"><pose>{fmt_pose(pose)}</pose>
          <geometry><sphere><radius>{radius}</radius></sphere></geometry>{mat(rgba)}</visual>"""


def static_model(name, pose, body, collision_body=""):
    return f"""
    <model name="{name}"><static>true</static><pose>{fmt_pose(pose)}</pose>
      <link name="link">{body}{collision_body}
      </link>
    </model>"""


def crane_model():
    sx, sy = g.SLEW_REL
    sz = g.DECK_TOP
    px, pz = sx + g.PIVOT_FWD, g.H0                 # bom mafsali (kamyona gore)
    tele_x = px + 0.8                               # teleskop kizagi
    tip_x = px + g.BOOM_BASE_LEN                    # bom ucu (tele = 0)

    # ---- Kamyon govdesi (base_link, kinematik: kontrolcu modeli yolda surer) ----
    truck = box("chassis", (0, 0, 1.0), (9.0, 2.2, 0.4), DARK_GREY)
    truck += box("deck", (-1.0, 0, g.DECK_TOP - 0.15), (6.8, 2.5, 0.3), CRANE_COLOR)
    truck += box("deck_rail_l", (1.0, 1.2, g.DECK_TOP + 0.12), (2.6, 0.08, 0.25), CRANE_DARK)
    truck += box("deck_rail_r", (1.0, -1.2, g.DECK_TOP + 0.12), (2.6, 0.08, 0.25), CRANE_DARK)
    truck += box("cab", (3.55, 0, 2.15), (1.9, 2.5, 2.1), CAB_COLOR)
    truck += box("cab_roof", (3.5, 0, 3.25), (1.7, 2.3, 0.1), CRANE_DARK)
    truck += box("windshield", (4.51, 0, 2.55), (0.04, 2.2, 0.8), GLASS)
    truck += box("side_window_l", (3.9, 1.26, 2.6), (0.9, 0.02, 0.6), GLASS)
    truck += box("side_window_r", (3.9, -1.26, 2.6), (0.9, 0.02, 0.6), GLASS)
    truck += box("bumper", (4.45, 0, 1.2), (0.2, 2.5, 0.4), DARK)
    truck += box("headlight_l", (4.56, 0.9, 1.5), (0.04, 0.35, 0.18), LIGHT)
    truck += box("headlight_r", (4.56, -0.9, 1.5), (0.04, 0.35, 0.18), LIGHT)
    truck += box("beacon", (3.5, 0, 3.38), (0.3, 0.3, 0.16), BEACON)
    for i, x in enumerate(g.AXLES_X):
        truck += box(f"fender_l{i}", (x, 1.1, 1.08), (1.15, 0.45, 0.06), DARK_GREY)
        truck += box(f"fender_r{i}", (x, -1.1, 1.08), (1.15, 0.45, 0.06), DARK_GREY)

    # ---- Kule (slew) ----
    turret = box("housing", (-0.2, 0, 0.55), (2.6, 2.2, 1.1), CRANE_COLOR)
    turret += box("counterweight", (-1.7, 0, 0.55), (0.9, 2.3, 1.1), DARK_GREY)
    turret += box("op_cab", (0.3, -1.35, 0.75), (1.3, 0.6, 1.2), CAB_COLOR)
    turret += box("op_window", (0.96, -1.35, 0.95), (0.04, 0.5, 0.6), GLASS)
    turret += box("boom_mount", (g.PIVOT_FWD, 0, 0.9), (0.6, 1.0, 0.8), CRANE_DARK)

    # ---- Bom (luff) + teleskop ----
    boom = box("boom_base", (3.4, 0, 0), (8.0, 0.8, 0.7), CRANE_COLOR)
    boom += box("boom_stripe", (3.4, 0, 0.36), (7.6, 0.5, 0.02), CRANE_DARK)
    tele = box("boom_tele", (3.4, 0, 0), (6.8, 0.56, 0.5), CRANE_COLOR)
    tele += box("boom_head", (6.8, 0, 0), (0.8, 0.65, 0.75), CRANE_DARK)

    # ---- Makara: bomun tersine doner, halat hep dik. Kanca kamerasi burada ----
    sheave = box("sheave_block", (0, 0, -0.05), (0.35, 0.45, 1.4), DARK)
    sheave += cyl("pulley", (0, 0, 0, 1.5708, 0, 0), 0.32, 0.5, GREY)
    cx, cy, cz = g.CAM_OFFSET
    sheave += box("camera_housing", (cx, cy, cz + 0.08), (0.22, 0.18, 0.16), DARK)
    sheave += box("camera_arm", (cx, cy / 2, cz + 0.1), (0.06, abs(cy), 0.06), DARK_GREY)
    camera = f"""
        <sensor name="hook_camera" type="camera">
          <pose>{cx} {cy} {cz} 0 1.5708 0</pose>
          <always_on>1</always_on>
          <update_rate>10</update_rate>
          <camera>
            <horizontal_fov>{g.CAM_HFOV}</horizontal_fov>
            <image><width>{g.CAM_W}</width><height>{g.CAM_H}</height><format>R8G8B8</format></image>
            <clip><near>0.2</near><far>80</far></clip>
          </camera>
          <plugin name="hook_camera_plugin" filename="libgazebo_ros_camera.so">
            <ros><namespace>/crane</namespace></ros>
            <camera_name>hook_camera</camera_name>
            <frame_name>hook_camera</frame_name>
          </plugin>
        </sensor>"""

    links = link("base_link", (0, 0, 0), truck, gravity=False, mass=20000, kinematic=True)
    links += link("turret", (sx, sy, sz), turret, gravity=False)
    links += link("boom", (px, 0, pz), boom, gravity=False)
    links += link("tele", (tele_x, 0, pz), tele, gravity=False)
    links += link("sheave", (tip_x, 0, pz), sheave + camera, gravity=False)

    joints = joint("slew_joint", "revolute", "base_link", "turret", (0, 0, 1), -1e6, 1e6)
    joints += joint("luff_joint", "revolute", "turret", "boom", (0, 1, 0), -g.LUFF_MAX, -g.LUFF_MIN)
    joints += joint("tele_joint", "prismatic", "boom", "tele", (1, 0, 0), 0, g.TELE_MAX)
    joints += joint("sheave_joint", "revolute", "tele", "sheave", (0, 1, 0), g.LUFF_MIN, g.LUFF_MAX)

    # Tekerlekler: aks basina bir link, donusu gorunsun diye jant ustunde capraz cubuklar
    for i, x in enumerate(g.AXLES_X):
        body = ""
        for j, y in enumerate([-1.05, 1.05]):
            body += cyl(f"tyre_{j}", (0, y, 0, 1.5708, 0, 0), g.WHEEL_R, 0.4, DARK)
            body += cyl(f"rim_{j}", (0, y * 1.2, 0, 1.5708, 0, 0), 0.28, 0.02, RIM)
            body += box(f"spoke_a{j}", (0, y * 1.21, 0), (0.5, 0.02, 0.07), DARK_GREY)
            body += box(f"spoke_b{j}", (0, y * 1.21, 0), (0.07, 0.02, 0.5), DARK_GREY)
        links += link(f"axle_{i}", (x, 0, g.WHEEL_R), body, gravity=False)
        joints += joint(f"wheel_joint_{i}", "revolute", "base_link", f"axle_{i}", (0, 1, 0), -1e9, 1e9)

    # Destek ayaklari (outrigger): yana dogru acilir; toplu iken kamyon genisligi icinde
    for i, x in enumerate(g.OUTRIG_X):
        for side, s in (("l", 1), ("r", -1)):
            body = box("beam", (0, s * 0.45, 1.0), (0.35, 1.6, 0.3), CRANE_COLOR)
            body += box("stripe", (0, s * 1.15, 1.16), (0.36, 0.2, 0.02), BEACON)
            body += box("jack", (0, s * 1.15, 0.55), (0.25, 0.25, 0.9), GREY)
            body += box("pad", (0, s * 1.15, 0.07), (0.7, 0.7, 0.08), DARK)
            name = f"outrig_{i}{side}"
            links += link(name, (x, s * 0.1, 0), body, gravity=False)
            joints += joint(f"outrig_joint_{i}{side}", "prismatic", "base_link", name, (0, s, 0), 0, g.OUTRIG_STROKE)

    # Halat: N parca, her biri yukari dogru ROPE_SEG uzunlugunda; parcalar birbirine gecerek uzar
    parent = "sheave"
    for i in range(g.N_ROPE):
        body = cyl("rope", (0, 0, g.ROPE_SEG / 2, 0, 0, 0), 0.03, g.ROPE_SEG, DARK)
        links += link(f"rope_{i}", (tip_x, 0, pz), body, gravity=False, mass=0.1)
        joints += joint(f"rope_joint_{i}", "prismatic", parent, f"rope_{i}", (0, 0, -1), 0, g.ROPE_SEG)
        parent = f"rope_{i}"

    hook = box("hook_block", (0, 0, -0.3), (0.5, 0.35, 0.55), CRANE_COLOR)
    hook += box("hook_stripe", (0, 0, -0.3), (0.52, 0.37, 0.12), CRANE_DARK)
    hook += cyl("hook", (0, 0, -0.72, 1.5708, 0, 0), 0.12, 0.08, DARK)
    hook += cyl("hook_shank", (0, 0, -0.62, 0, 0, 0), 0.04, 0.2, DARK)
    links += link("hook", (tip_x, 0, pz), hook, gravity=False)
    joints += joint("hook_fix", "fixed", parent, "hook")

    joint_names = list(g.joint_positions({**g.HOME}).keys())
    js = "".join(f"<joint_name>{n}</joint_name>" for n in joint_names)
    plugins = f"""
      <!-- /crane/set_joint_trajectory: eklemleri dogrudan konuma ayarlar (kinematik kontrol) -->
      <plugin name="crane_joint_pose" filename="libgazebo_ros_joint_pose_trajectory.so">
        <ros><namespace>/crane</namespace></ros>
        <update_rate>50</update_rate>
      </plugin>
      <plugin name="crane_joint_states" filename="libgazebo_ros_joint_state_publisher.so">
        <ros><namespace>/crane</namespace></ros>
        <update_rate>20</update_rate>
        {js}
      </plugin>"""
    return f"""
    <model name="mpg_crane">
      <pose>{g.HOME['bx']} 0 0 0 0 0</pose>{links}{joints}{plugins}
    </model>"""


def park_x(area):
    """Kamyonun bu alan icin park edecegi yol konumu: kule ekseni alanin x'ine gelir."""
    return SITES["alanlar"][area]["x"] - g.SLEW_REL[0]


def ground_models():
    """Asfalt saha, yol, seritler."""
    W, H = 110.0, 80.0
    body = box("asfalt", (0, 0, 0.01), (W, H, 0.02), ASPHALT)
    body += box("yol", (0, 0, 0.022), (W - 6, 8.0, 0.004), ROAD)
    body += box("serit_l", (0, 3.85, 0.026), (W - 6, 0.15, 0.004), PAINT)
    body += box("serit_r", (0, -3.85, 0.026), (W - 6, 0.15, 0.004), PAINT)
    x = -W / 2 + 5
    k = 0
    while x < W / 2 - 5:                            # kesikli orta serit
        body += box(f"orta_{k}", (x, 0, 0.026), (3.0, 0.15, 0.004), PAINT)
        x += 6.0
        k += 1
    return static_model("saha", (0, 0, 0), body)


def area_models():
    out = ""
    for name, a in SITES["alanlar"].items():
        side = 1 if a["y"] > 0 else -1             # tabela yoldan uzak kosede
        body = box("zemin", (0, 0, 0.024), (6.0, 6.0, 0.006), CONCRETE)
        for k, (pos, size) in enumerate([((0, 3.0), (6.15, 0.15)), ((0, -3.0), (6.15, 0.15)),
                                         ((3.0, 0), (0.15, 6.15)), ((-3.0, 0), (0.15, 6.15))]):
            body += box(f"cizgi_{k}", (pos[0], pos[1], 0.029), (size[0], size[1], 0.004), PAINT)
        body += cyl("direk", (3.6, side * 3.6, 1.5, 0, 0, 0), 0.07, 3.0, GREY)
        body += box("tabela", (3.6, side * 3.6, 3.2), (2.2, 0.08, 1.0), a["renk"])
        body += box("tabela_cerceve", (3.6, side * 3.6, 3.2), (2.3, 0.06, 1.1), DARK_GREY)
        out += static_model(f"alan_{name}", (a["x"], a["y"], 0), body)
        # yoldaki park yeri isareti (kamyonun duracagi yer)
        px = park_x(name)
        p = ""
        for k, (pos, size) in enumerate([((0, 1.7), (10.2, 0.12)), ((0, -1.7), (10.2, 0.12)),
                                         ((5.05, 0), (0.12, 3.4)), ((-5.05, 0), (0.12, 3.4))]):
            p += box(f"park_{k}", (pos[0], pos[1], 0.03), (size[0], size[1], 0.004), PAINT)
        out += static_model(f"park_{name}", (px, 0, 0), p)
    return out


def load_models():
    out = ""
    offs = SITES["slot_offsets"]
    for name, y in SITES["yukler"].items():
        sx, sy, sz = SITES["yuk_tipleri"][y["tip"]]
        a = SITES["alanlar"][y["alan"]]
        x0, y0 = a["x"] + offs[y["slot"]][0], a["y"] + offs[y["slot"]][1]
        if y["tip"] == "kasa":
            body = box("palet", (0, 0, 0.075), (sx + 0.1, sy + 0.1, 0.15), WOOD)
            body += box("kasa", (0, 0, 0.15 + (sz - 0.15) / 2), (sx, sy, sz - 0.15), y["renk"])
        else:
            body = box("blok", (0, 0, sz / 2), (sx, sy, sz), y["renk"])
        body += box("col", (0, 0, sz / 2), (sx, sy, sz), "0 0 0 0", collision=True).split("</visual>", 1)[1]
        # Yuk modelinin orijini TABANINDA: kontrolcu z = zemin yuksekligi olarak yerlestirir
        out += f"""
    <model name="{name}"><pose>{x0} {y0} {g.GROUND_Z} 0 0 0</pose>
      <link name="link">
        <kinematic>1</kinematic><gravity>0</gravity>
        <inertial><mass>500</mass>
          <inertia><ixx>100</ixx><iyy>100</iyy><izz>100</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>{body}
      </link>
    </model>"""
    return out


def building(name, x, y, size, color, door_side):
    bx, by, bz = size
    body = box("duvar", (0, 0, bz / 2), (bx, by, bz), color, collision=True)
    body += box("cati", (0, 0, bz + 0.15), (bx + 0.6, by + 0.6, 0.3), DARK_GREY)
    body += box("kapi", (0, door_side * (by / 2 + 0.02), 2.25), (5.0, 0.04, 4.5), DOOR)
    for k in range(int(bx // 3)):                   # pencere sirasi
        wx = -bx / 2 + 1.5 + 3 * k
        if abs(wx) < 3.2:
            continue
        body += box(f"pencere_{k}", (wx, door_side * (by / 2 + 0.02), bz - 2.0), (1.6, 0.04, 1.1), GLASS)
    return static_model(name, (x, y, 0), body)


def scenery_models():
    out = ""
    for name, o in SITES.get("engeller", {}).items():
        out += building(name, o["x"], o["y"], o["boyut"], o["renk"], 1 if o["y"] < 0 else -1)
    out += building("ofis", -44.0, 20.0, (12.0, 8.0, 6.0), "0.80 0.74 0.62 1", -1)

    # Kemer catili hangar
    h = box("taban", (0, 0, 2.0), (24.0, 14.0, 4.0), "0.50 0.53 0.50 1", collision=True)
    h += cyl("kemer", (0, 0, 4.0, 0, 1.5708, 0), 7.0, 24.0, "0.55 0.58 0.55 1")
    h += box("hangar_kapi", (12.02, 0, 3.5), (0.04, 10.0, 7.0), DOOR)
    out += static_model("hangar", (40.0, -26.0, 0), h)

    # Konteyner yiginlari (alanlardan uzakta: kanca kamerasina girmesin)
    colors = ["0.70 0.35 0.15 1", "0.25 0.35 0.45 1", "0.30 0.45 0.30 1", "0.55 0.50 0.40 1"]
    c = ""
    k = 0
    for row in range(3):
        for lvl in range(2 if row != 1 else 3):
            c += box(f"konteyner_{k}", (0, row * 2.6, 1.3 + lvl * 2.6), (6.1, 2.44, 2.59), colors[k % 4])
            k += 1
    out += static_model("konteynerler", (-44.0, -26.0, 0), c)
    out += static_model("konteynerler_2", (30.0, 26.0, 0),
                        c.replace('name="konteyner_', 'name="k2_'))

    # Aydinlatma direkleri (yol kenari)
    p = ""
    for k, x in enumerate(range(-48, 49, 16)):
        for side in (1, -1):
            yy = side * 5.2
            p += cyl(f"direk_{k}_{side}", (x, yy, 4.0, 0, 0, 0), 0.12, 8.0, GREY)
            p += box(f"kol_{k}_{side}", (x, yy - side * 0.8, 7.9), (0.12, 1.6, 0.12), GREY)
            p += box(f"lamba_{k}_{side}", (x, yy - side * 1.5, 7.8), (0.5, 0.3, 0.12), DARK_GREY)
    out += static_model("direkler", (0, 0, 0), p)

    # Agaclar (cevre)
    t = ""
    k = 0
    for x in range(-50, 51, 10):
        for y in (37.0, -37.0):
            t += cyl(f"govde_{k}", (x, y, 1.5, 0, 0, 0), 0.25, 3.0, "0.40 0.27 0.15 1")
            t += sphere(f"yaprak_{k}", (x, y, 4.2), 2.0, "0.20 0.45 0.18 1")
            k += 1
    out += static_model("agaclar", (0, 0, 0), t)

    # Cevre citi
    f = ""
    for k, (pos, size) in enumerate([((0, 39.5), (110.0, 0.08, 2.2)), ((0, -39.5), (110.0, 0.08, 2.2)),
                                     ((54.5, 0), (0.08, 79.0, 2.2)), ((-54.5, 0), (0.08, 79.0, 2.2))]):
        f += box(f"cit_{k}", (pos[0], pos[1], 1.1), size, FENCE)
    out += static_model("cit", (0, 0, 0), f)
    return out


def camera_pose(eye, target):
    dx, dy, dz = (t - e for t, e in zip(target, eye))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))
    return f"{eye[0]} {eye[1]} {eye[2]} 0 {pitch:.3f} {yaw:.3f}"


def main():
    world = f"""<?xml version="1.0"?>
<!-- OTOMATIK URETILDI: gen_world.py (elle duzenleme; sites.yaml / crane_geometry.py'yi degistir) -->
<sdf version="1.6">
  <world name="default">
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>

    <!-- /set_entity_state servisi: kontrolcu yukleri kancayla birlikte tasir -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/</namespace></ros>
      <update_rate>1.0</update_rate>
    </plugin>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.70 0.80 0.95 1</background>
      <shadows>true</shadows>
      <sky><clouds><speed>3</speed></clouds></sky>
    </scene>

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>{camera_pose((-20, -48, 30), (-2, 0, 0))}</pose>
      </camera>
    </gui>
{ground_models()}
{area_models()}
{scenery_models()}
{crane_model()}
{load_models()}
  </world>
</sdf>
"""
    os.makedirs(os.path.join(HERE, "worlds"), exist_ok=True)
    path = os.path.join(HERE, "worlds", "crane_site.world")
    with open(path, "w", encoding="utf-8") as f:
        f.write(world)
    print(f"yazildi: {path}")


if __name__ == "__main__":
    main()
