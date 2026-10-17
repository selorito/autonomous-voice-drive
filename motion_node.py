"""
motion_node: /voice/command (String) -> /cmd_vel (Twist)

Komutlar:
    FORWARD / BACKWARD : sabit hizla surekli git (yeni komut veya STOP gelene kadar)
    LEFT / RIGHT       : yerinde ~90 derece don, sonra dur
    STOP               : aninda sifir hiz (her seyden oncelikli)

Guvenlik:
    - 10 Hz'de surekli yayin: aracin durumu her an bu node'un kontrolunde
    - Watchdog: WATCHDOG_SEC boyunca yeni komut gelmezse otomatik dur
    - Node kapanirken sifir hiz gonderilir

Calistirma:
    python3 motion_node.py
Test (mikrofonsuz):
    ros2 topic pub --once /voice/command std_msgs/msg/String "{data: FORWARD}"
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

LINEAR_SPEED = 0.2              # m/s   (burger max ~0.22)
ANGULAR_SPEED = 0.5             # rad/s
TURN_ANGLE = math.radians(90)   # LEFT/RIGHT icin donus acisi
WATCHDOG_SEC = 15.0             # bu kadar sure komut gelmezse dur
RATE_HZ = 10.0


class MotionNode(Node):
    def __init__(self):
        super().__init__("motion_node")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(String, "/voice/command", self.on_command, 10)
        self.create_timer(1.0 / RATE_HZ, self.loop)

        self.twist = Twist()            # su an uygulanan hiz
        self.turn_end = None            # donus bitis zamani (saniye), donmuyorsa None
        self.last_cmd_time = self.now()
        self.get_logger().info("motion_node hazir, /voice/command bekleniyor")

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def stop(self, reason):
        self.twist = Twist()
        self.turn_end = None
        self.pub.publish(self.twist)    # timer'i bekleme, hemen gonder
        self.get_logger().info(f"DUR ({reason})")

    def on_command(self, msg):
        cmd = msg.data.strip().upper()
        self.last_cmd_time = self.now()

        if cmd == "STOP":
            self.stop("STOP komutu")
            return

        t = Twist()
        self.turn_end = None
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

        self.twist = t
        self.pub.publish(self.twist)
        self.get_logger().info(f"{cmd}  (lin={t.linear.x:.2f} m/s, ang={t.angular.z:.2f} rad/s)")

    def loop(self):
        now = self.now()
        moving = self.twist.linear.x != 0.0 or self.twist.angular.z != 0.0

        if self.turn_end is not None and now >= self.turn_end:
            self.stop("donus tamamlandi")
            return
        if moving and now - self.last_cmd_time > WATCHDOG_SEC:
            self.stop(f"watchdog: {WATCHDOG_SEC:.0f} sn komut yok")
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
