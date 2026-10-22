"""
motion_node: /voice/command (String) -> /cmd_vel (Twist)

Komutlar:
    FORWARD / BACKWARD : sabit hizla surekli git (yeni komut veya STOP gelene kadar)
    LEFT / RIGHT       : yerinde ~90 derece don, sonra dur
    GOTO <yer>         : yer dosyasindaki noktaya git (/odom ile: once hedefe don, sonra ilerle)
    STOP               : aninda sifir hiz (her seyden oncelikli)

Guvenlik:
    - 10 Hz'de surekli yayin: aracin durumu her an bu node'un kontrolunde
    - Watchdog: WATCHDOG_SEC boyunca yeni komut gelmezse otomatik dur
    - Lidar: gidilen yonde SAFE_DIST'ten yakin engel varsa dur / komutu reddet
    - Fail-safe: lidar verisi SCAN_TIMEOUT'tan uzun sure gelmezse ileri/geri hareket edilmez
    - Node kapanirken sifir hiz gonderilir

Calistirma:
    python3 motion_node.py
Test (mikrofonsuz):
    ros2 topic pub --once /voice/command std_msgs/msg/String "{data: FORWARD}"
"""
import math
import os

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

LINEAR_SPEED = 0.2              # m/s   (burger max ~0.22)
ANGULAR_SPEED = 0.5             # rad/s
TURN_ANGLE = math.radians(90)   # LEFT/RIGHT icin donus acisi
WATCHDOG_SEC = 15.0             # bu kadar sure komut gelmezse dur
RATE_HZ = 10.0

SAFE_DIST = 0.5                 # m: gidilen yonde bundan yakin engel varsa dur
SECTOR = math.radians(30)       # on/arka icin +-30 derecelik koni
SCAN_TIMEOUT = 1.0              # sn: lidar bu kadar susarsa "kor" kabul et

# GOTO
GOAL_TOL = 0.15                 # m: hedefe bu kadar yaklasinca "ulasildi"
DRIVE_ANGLE = math.radians(20)  # yon hatasi bundan buyukse yerinde don, kucukse ilerle
K_ANGULAR = 1.5                 # yon duzeltme kazanci (P kontrol)
MAX_ANGULAR = 0.8               # rad/s
ODOM_TIMEOUT = 1.0              # sn: odometri bu kadar susarsa GOTO iptal

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
LOCATIONS_FILE = os.environ.get("LOCATIONS_FILE",
                                os.path.join(PROJECT_DIR, "worlds", "demo_locations.yaml"))


def wrap(a):
    """Aciyi [-pi, pi] araligina getirir."""
    return math.atan2(math.sin(a), math.cos(a))


class MotionNode(Node):
    def __init__(self):
        super().__init__("motion_node")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(String, "/voice/command", self.on_command, 10)
        # Gazebo /scan'i "best effort" yayinlar; sensor QoS kullanmazsak mesajlar gelmez
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_timer(1.0 / RATE_HZ, self.loop)

        with open(LOCATIONS_FILE, encoding="utf-8") as f:
            konumlar = yaml.safe_load(f)["konumlar"]
        self.locations = {k: (float(v["x"]), float(v["y"])) for k, v in konumlar.items()}
        self.goal = None                # (isim, x, y, son_tarih) ya da None
        self.pose = None                # (x, y, yaw)
        self.odom_time = None

        self.twist = Twist()            # su an uygulanan hiz
        self.turn_end = None            # donus bitis zamani (saniye), donmuyorsa None
        self.last_cmd_time = self.now()
        self.front_dist = float("inf")  # ondeki en yakin engel (m)
        self.rear_dist = float("inf")   # arkadaki en yakin engel (m)
        self.scan_time = None           # son lidar mesajinin geldigi an
        self.get_logger().info(f"motion_node hazir. Yerler: {', '.join(self.locations)}")

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_scan(self, scan):
        front, rear = float("inf"), float("inf")
        for i, r in enumerate(scan.ranges):
            if not (scan.range_min < r < scan.range_max):   # inf / 0 / gecersiz olcumleri at
                continue
            a = scan.angle_min + i * scan.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))        # aciyi [-pi, pi] araligina getir
            if abs(a) <= SECTOR:
                front = min(front, r)
            elif abs(a) >= math.pi - SECTOR:
                rear = min(rear, r)
        self.front_dist, self.rear_dist = front, rear
        self.scan_time = self.now()

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        self.odom_time = self.now()

    def blocked(self, linear_x):
        """Bu yonde gitmek guvenli degilse sebebini dondurur, guvenliyse None."""
        if self.scan_time is None or self.now() - self.scan_time > SCAN_TIMEOUT:
            return "lidar verisi yok (fail-safe)"
        if linear_x > 0 and self.front_dist < SAFE_DIST:
            return f"onde engel: {self.front_dist:.2f} m"
        if linear_x < 0 and self.rear_dist < SAFE_DIST:
            return f"arkada engel: {self.rear_dist:.2f} m"
        return None

    def stop(self, reason):
        self.twist = Twist()
        self.turn_end = None
        self.goal = None
        self.pub.publish(self.twist)    # timer'i bekleme, hemen gonder
        self.get_logger().info(f"DUR ({reason})")

    def on_command(self, msg):
        cmd = msg.data.strip().upper()
        self.last_cmd_time = self.now()

        if cmd == "STOP":
            self.stop("STOP komutu")
            return

        if cmd.startswith("GOTO"):
            self.start_goto(cmd.split(maxsplit=1)[1].lower() if " " in cmd else "")
            return

        t = Twist()
        self.turn_end = None
        self.goal = None                # elle verilen her komut hedefi iptal eder
        if cmd == "FORWARD":
            t.linear.x = LINEAR_SPEED
        elif cmd == "BACKWARD":
            t.linear.x = -LINEAR_SPEED
        elif cmd in ("LEFT", "RIGHT"):
            t.angular.z = ANGULAR_SPEED if cmd == "LEFT" else -ANGULAR_SPEED
            self.turn_end = self.now() + TURN_ANGLE / ANGULAR_SPEED
        else:
            self.get_logger().warn(f"Bilinmeyen komut: '{cmd}' -> yok sayildi")
            return

        if t.linear.x != 0.0:
            reason = self.blocked(t.linear.x)
            if reason:
                self.stop(f"{cmd} reddedildi, {reason}")
                return

        self.twist = t
        self.pub.publish(self.twist)
        self.get_logger().info(f"{cmd}  (lin={t.linear.x:.2f} m/s, ang={t.angular.z:.2f} rad/s)")

    def start_goto(self, name):
        if name not in self.locations:
            self.get_logger().warn(f"Bilinmeyen yer: '{name}' -> yok sayildi")
            return
        if self.pose is None or self.now() - self.odom_time > ODOM_TIMEOUT:
            self.stop(f"GOTO {name} reddedildi, odometri yok")
            return
        gx, gy = self.locations[name]
        dist = math.hypot(gx - self.pose[0], gy - self.pose[1])
        # Zaman asimi: duz gidis suresinin 2 kati + donus payi
        deadline = self.now() + 2 * dist / LINEAR_SPEED + 10.0
        self.twist = Twist()
        self.turn_end = None
        self.goal = (name, gx, gy, deadline)
        self.get_logger().info(f"GOTO {name}  -> ({gx:.1f}, {gy:.1f}), mesafe {dist:.2f} m")

    def drive_to_goal(self, now):
        name, gx, gy, deadline = self.goal
        if self.odom_time is None or now - self.odom_time > ODOM_TIMEOUT:
            self.stop(f"{name}: odometri kesildi")
            return
        if now > deadline:
            self.stop(f"{name}: zaman asimi")
            return

        x, y, yaw = self.pose
        dist = math.hypot(gx - x, gy - y)
        if dist < GOAL_TOL:
            self.stop(f"hedefe ulasildi: {name}")
            return

        err = wrap(math.atan2(gy - y, gx - x) - yaw)    # hedefe gore yon hatasi
        t = Twist()
        t.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, K_ANGULAR * err))
        if abs(err) < DRIVE_ANGLE:                      # yon dogru -> ilerle (yonu duzelterek)
            t.linear.x = LINEAR_SPEED if dist > 0.4 else max(0.05, LINEAR_SPEED * dist / 0.4)
            reason = self.blocked(t.linear.x)
            if reason:
                self.stop(f"{name} yolunda {reason}")
                return
        self.twist = t
        self.pub.publish(t)

    def loop(self):
        now = self.now()
        if self.goal is not None:
            self.drive_to_goal(now)
            return
        moving = self.twist.linear.x != 0.0 or self.twist.angular.z != 0.0

        if self.turn_end is not None and now >= self.turn_end:
            self.stop("donus tamamlandi")
            return
        if moving and now - self.last_cmd_time > WATCHDOG_SEC:
            self.stop(f"watchdog: {WATCHDOG_SEC:.0f} sn komut yok")
            return
        if self.twist.linear.x != 0.0:
            reason = self.blocked(self.twist.linear.x)
            if reason:
                self.stop(reason)
                return

        self.pub.publish(self.twist)


def main():
    rclpy.init()
    node = MotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())       # kapanirken araci durdur
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
