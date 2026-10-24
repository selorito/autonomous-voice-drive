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

CRANE_COLOR = "0.93 0.74 0.28 1"     # MPG tarzi hardal sarisi
WHITE = "0.95 0.95 0.95 1"
DARK = "0.15 0.15 0.15 1"
GREY = "0.40 0.40 0.42 1"
GLASS = "0.20 0.30 0.45 1"
WOOD = "0.55 0.38 0.20 1"


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


def link(name, pose, body, gravity=True, mass=1.0):
    p = fmt_pose(pose)
    return f"""
      <link name="{name}">
        <pose>{p}</pose>
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


def crane_model():
    sx, sy, sz = g.SLEW_POS
    px, pz = sx + g.PIVOT_FWD, g.H0                 # bom mafsali
    tele_x = px + 0.8                               # teleskop kizagi
    tip_x = px + g.BOOM_BASE_LEN                    # bom ucu (tele = 0)

    # ---- Kamyon (base_link) ----
    truck = ""
    for i, x in enumerate([3.2, 1.9, -1.6, -3.0]):
        for j, y in enumerate([-1.05, 1.05]):
            truck += cyl(f"wheel_{i}_{j}", (x, y, 0.5, 1.5708, 0, 0), 0.5, 0.4, DARK)
    truck += box("chassis", (0, 0, 1.0), (9.0, 2.2, 0.4), GREY)
    truck += box("deck", (-1.0, 0, g.DECK_TOP - 0.15), (6.8, 2.5, 0.3), CRANE_COLOR)
    truck += box("cab", (3.55, 0, 2.15), (1.9, 2.5, 2.1), WHITE)
    truck += box("windshield", (4.51, 0, 2.55), (0.04, 2.2, 0.8), GLASS)
    truck += box("bumper", (4.45, 0, 1.2), (0.2, 2.5, 0.4), DARK)
    for i, x in enumerate([2.4, -4.0]):             # acik ayaklar (outrigger)
        for j, s in enumerate([-1, 1]):
            truck += box(f"outr_beam_{i}_{j}", (x, s * 1.9, 1.0), (0.35, 1.6, 0.3), CRANE_COLOR)
            truck += box(f"outr_jack_{i}_{j}", (x, s * 2.7, 0.55), (0.25, 0.25, 0.9), GREY)
            truck += box(f"outr_pad_{i}_{j}", (x, s * 2.7, 0.04), (0.7, 0.7, 0.08), DARK)

    # ---- Kule (slew) ----
    turret = box("housing", (-0.2, 0, 0.55), (2.6, 2.2, 1.1), CRANE_COLOR)
    turret += box("counterweight", (-1.7, 0, 0.55), (0.9, 2.3, 1.1), GREY)
    turret += box("op_cab", (0.3, -1.35, 0.75), (1.3, 0.6, 1.2), WHITE)
    turret += box("op_window", (0.96, -1.35, 0.95), (0.04, 0.5, 0.6), GLASS)
    turret += box("boom_mount", (g.PIVOT_FWD, 0, 0.9), (0.6, 1.0, 0.8), CRANE_COLOR)

    # ---- Bom (luff) + teleskop ----
    boom = box("boom_base", (3.4, 0, 0), (8.0, 0.8, 0.7), CRANE_COLOR)
    tele = box("boom_tele", (3.4, 0, 0), (6.8, 0.56, 0.5), CRANE_COLOR)
    tele += box("boom_head", (6.8, 0, 0), (0.8, 0.65, 0.75), CRANE_COLOR)

    # ---- Makara: bomun tersine doner, halat hep dik ----
    sheave = box("sheave_block", (0, 0, -0.05), (0.35, 0.45, 1.4), DARK)
    sheave += cyl("pulley", (0, 0, 0, 1.5708, 0, 0), 0.32, 0.5, GREY)

    links = link("base_link", (0, 0, 0), truck, gravity=True, mass=20000)
    links += link("turret", (sx, sy, sz), turret, gravity=False)
    links += link("boom", (px, 0, pz), boom, gravity=False)
    links += link("tele", (tele_x, 0, pz), tele, gravity=False)
    links += link("sheave", (tip_x, 0, pz), sheave, gravity=False)

    joints = joint("world_fix", "fixed", "world", "base_link")
    joints += joint("slew_joint", "revolute", "base_link", "turret", (0, 0, 1), -1e6, 1e6)
    joints += joint("luff_joint", "revolute", "turret", "boom", (0, 1, 0), -g.LUFF_MAX, -g.LUFF_MIN)
    joints += joint("tele_joint", "prismatic", "boom", "tele", (1, 0, 0), 0, g.TELE_MAX)
    joints += joint("sheave_joint", "revolute", "tele", "sheave", (0, 1, 0), g.LUFF_MIN, g.LUFF_MAX)

    # Halat: N parca, her biri yukari dogru ROPE_SEG uzunlugunda; parcalar birbirine gecerek uzar
    parent = "sheave"
    for i in range(g.N_ROPE):
        body = cyl("rope", (0, 0, g.ROPE_SEG / 2, 0, 0, 0), 0.03, g.ROPE_SEG, DARK)
        links += link(f"rope_{i}", (tip_x, 0, pz), body, gravity=False, mass=0.1)
        joints += joint(f"rope_joint_{i}", "prismatic", parent, f"rope_{i}", (0, 0, -1), 0, g.ROPE_SEG)
        parent = f"rope_{i}"

    hook = box("hook_block", (0, 0, -0.3), (0.5, 0.35, 0.55), CRANE_COLOR)
    hook += cyl("hook", (0, 0, -0.72, 1.5708, 0, 0), 0.12, 0.08, DARK)
    hook += cyl("hook_shank", (0, 0, -0.62, 0, 0, 0), 0.04, 0.2, DARK)
    links += link("hook", (tip_x, 0, pz), hook, gravity=False)
    joints += joint("hook_fix", "fixed", parent, "hook")

    joint_names = ["slew_joint", "luff_joint", "tele_joint", "sheave_joint"] + \
                  [f"rope_joint_{i}" for i in range(g.N_ROPE)]
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
      <pose>0 0 0 0 0 0</pose>{links}{joints}{plugins}
    </model>"""


def area_models():
    out = ""
    for name, a in SITES["alanlar"].items():
        body = box("zemin", (0, 0, 0.011), (5.0, 5.0, 0.02), a["renk"])
        body += cyl("direk", (2.6, 2.6, 1.0, 0, 0, 0), 0.05, 2.0, GREY)
        body += box("tabela", (2.6, 2.6, 2.1), (1.4, 0.06, 0.7), a["renk"])
        out += f"""
    <model name="alan_{name}"><static>true</static><pose>{a['x']} {a['y']} 0 0 0 0</pose>
      <link name="link">{body}
      </link>
    </model>"""
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
    <model name="{name}"><pose>{x0} {y0} 0 0 0 0</pose>
      <link name="link">
        <kinematic>1</kinematic><gravity>0</gravity>
        <inertial><mass>500</mass>
          <inertia><ixx>100</ixx><iyy>100</iyy><izz>100</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>{body}
      </link>
    </model>"""
    return out


def obstacle_models():
    out = ""
    for name, o in SITES.get("engeller", {}).items():
        bx, by, bz = o["boyut"]
        body = box("bina", (0, 0, bz / 2), (bx, by, bz), o["renk"], collision=True)
        body += box("kapi", (bx / 2 + 0.01, 0, 1.6), (0.02, 3.0, 3.2), GREY)
        body += box("cati", (0, 0, bz + 0.1), (bx + 0.4, by + 0.4, 0.2), GREY)
        out += f"""
    <model name="{name}"><static>true</static><pose>{o['x']} {o['y']} 0 0 0 0</pose>
      <link name="link">{body}
      </link>
    </model>"""
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

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>{camera_pose((14, -20, 15), (-4, 0, 3))}</pose>
      </camera>
    </gui>
{crane_model()}
{area_models()}
{load_models()}
{obstacle_models()}
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
