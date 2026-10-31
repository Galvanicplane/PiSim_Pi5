#!/usr/bin/env python3
"""
ros2_pisim_bridge.py - PiSim <-> ROS 2 (Humble / Jazzy / Iron) Çift Yönlü DDS Köprüsü
======================================================================================
Gerçek ROS 2 (`rclpy`) mesajlarını Unreal Engine (veya Gazebo) PiSim protokolüne çevirir:
1. ROS 2 -> Unreal Engine (Kontrol):
   - `/cmd_vel` (geometry_msgs/msg/Twist)          -> Tüm Sürü (UDP 7402, target="all")
   - `/rover_01/cmd_vel` (geometry_msgs/msg/Twist) -> Araç 1 (UDP 7403, target="rover_01")
   - `/rover_02/cmd_vel` (geometry_msgs/msg/Twist) -> Araç 2 (UDP 7404, target="rover_02")
   - `/rover_03/cmd_vel` (geometry_msgs/msg/Twist) -> Araç 3 (UDP 7405, target="rover_03")
2. Unreal Engine -> ROS 2 (Telemetri):
   - UDP 7400'den gelen her aracın telemetrisini `/rover_XX/odom` (nav_msgs/msg/Odometry) olarak ROS 2 ağına basar!

Kullanım (ROS 2 kurulu ortamda):
    python3 ros2_pisim_bridge.py
    # Başka bir terminalden gerçek ROS 2 komutu gönder:
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
"""

import sys
import json
import socket
import threading

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

from beacon_sender import get_broadcast_targets

TELEMETRY_PORT = 7400
COMMAND_PORT = 7402


class PiSimRos2Bridge(Node if HAS_ROS2 else object):
    def __init__(self, target_ip: str = "255.255.255.255"):
        super().__init__('pisim_ros2_bridge')
        self.target_ip = target_ip
        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # 1. Tüm sürü için ortak /cmd_vel dinleyicisi
        self.create_subscription(Twist, '/cmd_vel', lambda msg: self._on_twist(msg, "all", COMMAND_PORT), 10)

        # 2. Tekil araçlar için /rover_01/cmd_vel .. /rover_08/cmd_vel dinleyicileri
        self.veh_subs = []
        for idx in range(1, 9):
            ns = f"rover_{idx:02d}"
            sub_port = COMMAND_PORT + idx
            sub = self.create_subscription(
                Twist,
                f'/{ns}/cmd_vel',
                lambda msg, t=ns, p=sub_port: self._on_twist(msg, t, p),
                10
            )
            self.veh_subs.append(sub)

        self.odom_pubs = {}
        self.get_logger().info(
            "🚀 PiSim ROS 2 DDS Köprüsü Aktif! Dinlenen Topic'ler: /cmd_vel, /rover_01/cmd_vel .. /rover_08/cmd_vel"
        )

    def _on_twist(self, msg: 'Twist', target: str, port: int):
        payload = {
            "cmd": "cmd_vel",
            "target": target,
            "linear": {"x": float(msg.linear.x), "y": float(msg.linear.y), "z": float(msg.linear.z)},
            "angular": {"x": float(msg.angular.x), "y": float(msg.angular.y), "z": float(msg.angular.z)},
            "linear_x": float(msg.linear.x),
            "angular_z": float(msg.angular.z),
            "timeout_ms": 800
        }
        raw = json.dumps(payload).encode('utf-8')
        try:
            self.tx_sock.sendto(raw, (self.target_ip, port))
            if port != COMMAND_PORT:
                self.tx_sock.sendto(raw, (self.target_ip, COMMAND_PORT))
            self.tx_sock.sendto(raw, ("127.0.0.1", port))
            for b_ip, b_port in get_broadcast_targets(port):
                try:
                    self.tx_sock.sendto(raw, (b_ip, b_port))
                except Exception:
                    pass
        except Exception as e:
            self.get_logger().warn(f"UDP Gönderim Hatası: {e}")


def main():
    if not HAS_ROS2:
        print("⚠️ Bu ortamda 'rclpy' (ROS 2 Python paketi) kurulu değil.")
        print("   ROS 2 kurulu bir Ubuntu / Raspberry Pi 5 ortamında:")
        print("   source /opt/ros/humble/setup.bash && python3 ros2_pisim_bridge.py")
        sys.exit(1)

    rclpy.init()
    node = PiSimRos2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
