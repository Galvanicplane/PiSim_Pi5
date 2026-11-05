#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pi5_ros2_test_sender.py - Pi 5 Saf ROS 2 Yayın ve Dinleme Test Scripti
======================================================================
Bu script, Raspberry Pi 5 üzerinde saf ROS 2 (`rclpy`) kütüphanesini kullanarak:
1. `/pisim/chat` topic'ine saf `std_msgs/msg/String` mesajı basar.
2. `/rover_01/cmd_vel` topic'ine saf `geometry_msgs/msg/Twist` sürüş komutu basar.
3. `/rover_01/imu` ve `/rover_01/gps` topic'lerini saf ROS 2 aboneliği ile ekrana basar.

Kullanım (Pi 5 Terminali):
  source /opt/ros/humble/setup.bash (veya jazzy/iron)
  python3 pi5_ros2_test_sender.py --chat "Merhaba Unreal Engine!"
  python3 pi5_ros2_test_sender.py --drive 1.5 0.5
  python3 pi5_ros2_test_sender.py --listen
"""

import argparse
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from sensor_msgs.msg import Imu, NavSatFix
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


class Pi5Ros2Tester(Node if HAS_ROS2 else object):
    def __init__(self):
        super().__init__('pi5_ros2_tester')
        self.chat_pub = self.create_publisher(String, '/pisim/chat', 10)
        self.twist_pub = self.create_publisher(Twist, '/rover_01/cmd_vel', 10)

    def send_chat(self, text: str):
        msg = String()
        msg.data = text
        self.chat_pub.publish(msg)
        self.get_logger().info(f"📤 [ROS 2 String] '/pisim/chat' topic'ine basıldı: '{text}'")

    def send_drive(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.twist_pub.publish(msg)
        self.get_logger().info(f"🚗 [ROS 2 Twist] '/rover_01/cmd_vel' basıldı: Linear={linear} m/s, Angular={angular} rad/s")

    def start_listening(self):
        self.get_logger().info("👂 ROS 2 Topic'leri Dinleniyor (/rover_01/imu, /rover_01/gps, /pisim/chat)...")

        def _imu_cb(msg: Imu):
            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z
            print(f"📥 [ROS 2 IMU] Accel: ({ax:.2f}, {ay:.2f}, {az:.2f}) m/s²", end="\r")

        def _gps_cb(msg: NavSatFix):
            print(f"\n📍 [ROS 2 GPS] Lat: {msg.latitude:.6f}, Lon: {msg.longitude:.6f}, Alt: {msg.altitude:.1f}m")

        def _chat_cb(msg: String):
            print(f"\n💬 [ROS 2 Chat]: {msg.data}")

        self.create_subscription(Imu, '/rover_01/imu', _imu_cb, 10)
        self.create_subscription(NavSatFix, '/rover_01/gps', _gps_cb, 10)
        self.create_subscription(String, '/pisim/chat', _chat_cb, 10)


def main():
    if not HAS_ROS2:
        print("❌ 'rclpy' modülü bulunamadı! Bu script Raspberry Pi 5 üzerinde ROS 2 ortamında çalıştırılmalıdır.")
        print("   Örnek: source /opt/ros/humble/setup.bash && python3 pi5_ros2_test_sender.py")
        sys.exit(1)

    parser = argparse.ArgumentParser(description="Raspberry Pi 5 Gerçek ROS 2 Test Aracı")
    parser.add_argument("--chat", type=str, default=None, help="ROS 2 /pisim/chat topic'ine String gönder")
    parser.add_argument("--drive", nargs=2, type=float, metavar=('LINEAR', 'ANGULAR'), help="ROS 2 /rover_01/cmd_vel Twist komutu bas (örn: 1.5 0.5)")
    parser.add_argument("--listen", action="store_true", help="ROS 2 sensör ve sohbet topic'lerini canlı dinle")

    args = parser.parse_args()

    rclpy.init()
    node = Pi5Ros2Tester()

    if args.chat:
        node.send_chat(args.chat)
        time.sleep(0.5)
    elif args.drive:
        node.send_drive(args.drive[0], args.drive[1])
        time.sleep(0.5)
    elif args.listen:
        node.start_listening()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    else:
        print("ℹ️ Lütfen bir işlem seçin (--chat, --drive veya --listen). Yardım için: --help")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

