"""
Kanca kamerasi goruntu isleme dugumu.

  /crane/hook_camera/image_raw  (Gazebo kamerasi, bom ucunda, tam asagi bakar)
  /crane/state                  (vincin eklem durumu -> kameranin o anki yeri ve yonu)
        |
        v   vision_core.detect(): renk + boyut + sekil -> yuk adi + saha koordinati
        |
  /crane/vision                 JSON {"detections": [{name, cls, x, y, size_ratio}], "moving": bool}
  /crane/hook_camera/annotated  etiketli goruntu (rqt_image_view ile de izlenebilir)
  "Kanca kamerasi" penceresi    sunumda ekranda acik dursun

Vinc hareket halindeyken kare ile eklem durumu tam ayni ana ait olmayabilir; bu yuzden
hareket sirasinda ve durduktan sonraki ilk 0.3 s tespit yayinlanmaz (goruntu yine gosterilir).

    python3 crane/vision_node.py               # pencere ile
    python3 crane/vision_node.py --no-window   # sadece topic'ler
"""
import argparse
import json
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

CRANE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, CRANE_DIR)
import vision_core as vc  # noqa: E402

PROCESS_HZ = 10.0
SETTLE_S = 0.3
WINDOW = "Kanca kamerasi"


class VisionNode(Node):
    def __init__(self, window):
        super().__init__("crane_vision")
        with open(os.path.join(CRANE_DIR, "sites.yaml"), encoding="utf-8") as f:
            self.sites = yaml.safe_load(f)
        self.window = window
        self.q = None
        self.state_text = "vinc durumu bekleniyor"
        self.last_moving = 0.0
        self.last_proc = 0.0
        self.frame = None
        self.n_frames = 0
        self.vis_pub = self.create_publisher(String, "/crane/vision", 10)
        self.img_pub = self.create_publisher(Image, "/crane/hook_camera/annotated", 2)
        self.create_subscription(String, "/crane/state", self.on_state, 10)
        self.create_subscription(Image, "/crane/hook_camera/image_raw", self.on_image, 2)
        self.create_timer(5.0, self.health)
        self.get_logger().info("kanca kamerasi bekleniyor: /crane/hook_camera/image_raw")

    def on_state(self, msg):
        s = json.loads(msg.data)
        self.q = s["q_raw"]
        if s.get("moving"):
            self.last_moving = time.monotonic()
        self.state_text = s["state"] + (f" | kancada {s['attached']}" if s.get("attached") else "")

    def on_image(self, msg):
        self.n_frames += 1
        now = time.monotonic()
        if now - self.last_proc < 1.0 / PROCESS_HZ:
            return
        self.last_proc = now
        img = to_rgb(msg)
        if img is None:
            return
        if self.q is None:
            self.frame = img[..., ::-1].copy()
            return
        settled = now - self.last_moving > SETTLE_S
        dets, _ = vc.detect(img, self.q, self.sites)
        out = [{k: d[k] for k in ("name", "cls", "x", "y", "size_ratio")} for d in dets]
        self.vis_pub.publish(String(data=json.dumps(
            {"detections": out if settled else [], "moving": not settled})))
        ann = vc.annotate(img, self.q, dets, self.state_text + ("" if settled else " | hareket"))
        self.frame = ann
        self.img_pub.publish(from_bgr(ann, msg.header))

    def health(self):
        if self.n_frames == 0:
            self.get_logger().warn("kamera goruntusu gelmiyor (Gazebo calisiyor mu, dunya yeniden uretildi mi?)")
        self.n_frames = 0


def to_rgb(msg):
    """sensor_msgs/Image -> numpy RGB (cv_bridge gerekmez)."""
    ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(msg.encoding)
    if ch is None:
        return None
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * ch]
    a = a.reshape(msg.height, msg.width, ch)[..., :3]
    return a[..., ::-1] if msg.encoding.startswith("bgr") else a


def from_bgr(img, header):
    m = Image()
    m.header = header
    m.height, m.width = img.shape[:2]
    m.encoding = "bgr8"
    m.step = m.width * 3
    m.data = np.ascontiguousarray(img).tobytes()
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-window", action="store_true", help="pencere acma, sadece topic yayinla")
    args, _ = ap.parse_known_args()
    window = not args.no_window and bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    rclpy.init()
    node = VisionNode(window)
    try:
        if not window:
            rclpy.spin(node)
            return
        import cv2
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 640, 480)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            if node.frame is not None:
                cv2.imshow(WINDOW, node.frame)
            if cv2.waitKey(1) & 0xFF == 27:              # ESC: pencereyi kapat, dugum calismaya devam etsin
                cv2.destroyAllWindows()
                rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
