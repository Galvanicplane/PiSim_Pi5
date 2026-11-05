#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pi5_ros2_zenoh_bridge.py - Raspberry Pi 5 Gerçek ROS 2 <-> Zenoh Çift Yönlü Köprüsü
===================================================================================
Bu script, Raspberry Pi 5 üzerinde çalışan GERÇEK ROS 2 ortamı (rclpy) ile
Windows / Unreal Engine / Docker ağında çalışan Zenoh topic mimarisini birbirine bağlar.

İletişim Yönleri:
1. Pi 5 (Gerçek ROS 2) -> Zenoh (Windows / Unreal Engine):
   - ROS 2: `/<rover>/cmd_vel` (geometry_msgs/msg/Twist) -> Zenoh: `pisim/<rover>/cmd_vel`
   - ROS 2: `/pisim/chat`      (std_msgs/msg/String)       -> Zenoh: `pisim/chat`

2. Zenoh (Unreal Engine / Sensörler) -> Pi 5 (Gerçek ROS 2):
   - Zenoh: `pisim/<rover>/imu` -> ROS 2: `/<rover>/imu` (sensor_msgs/msg/Imu)
   - Zenoh: `pisim/<rover>/gps` -> ROS 2: `/<rover>/gps` (sensor_msgs/msg/NavSatFix)
   - Zenoh: `pisim/chat`        -> ROS 2: `/pisim/chat`  (std_msgs/msg/String)

Kullanım (Pi 5 Terminali):
  source /opt/ros/humble/setup.bash (veya jazzy/iron)
  python3 pi5_ros2_zenoh_bridge.py

Test (Pi 5'te başka bir terminalden gerçek ROS 2 komutu):
  ros2 topic list
  ros2 topic echo /pisim/chat
  ros2 topic pub /rover_01/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 0.3}}"
"""

import json
import os
import sys
import threading
import time

# Windows ve Linux konsol kod sayfası desteği
if sys.stdout and hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from sensor_msgs.msg import Imu, NavSatFix
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

try:
    import zenoh
    HAS_ZENOH = True
except ImportError:
    HAS_ZENOH = False


class Pi5Ros2ZenohBridge(Node if HAS_ROS2 else object):
    def __init__(self, router_endpoint: str = None):
        if not HAS_ROS2:
            print("❌ 'rclpy' bulunamadı! Bu betik ROS 2 kurulu Raspberry Pi 5 ortamında çalıştırılmalıdır.")
            sys.exit(1)
        if not HAS_ZENOH:
            print("❌ 'eclipse-zenoh' modülü bulunamadı! 'pip3 install eclipse-zenoh' çalıştırın.")
            sys.exit(1)

        super().__init__('pi5_ros2_zenoh_bridge')
        self.get_logger().info("🚀 Raspberry Pi 5 ROS 2 <-> Zenoh Köprüsü Başlatılıyor...")

        # Zenoh Oturumu
        conf = zenoh.Config()
        env_router = os.environ.get("ZENOH_ROUTER", router_endpoint)
        if env_router:
            conf.insert_json5("mode", json.dumps("client"))
            conf.insert_json5("connect/endpoints", json.dumps([env_router]))
            self.get_logger().info(f"🔗 Zenoh Router'a Bağlanılıyor: {env_router}")
        else:
            conf.insert_json5("mode", json.dumps("peer"))
            self.get_logger().info("📡 Zenoh P2P Mesh Modu Aktif.")

        self.zenoh_session = zenoh.open(conf)
        self.get_logger().info("✅ Zenoh Oturumu Açıldı.")

        # ROS 2 Publisher'ları (Zenoh -> ROS 2)
        self.ros_chat_pub = self.create_publisher(String, '/pisim/chat', 10)
        self.ros_imu_pubs = {}
        self.ros_gps_pubs = {}

        # ROS 2 Subscriber'ları (ROS 2 -> Zenoh)
        self.ros_chat_sub = self.create_subscription(String, '/pisim/chat', self._on_ros_chat, 10)

        # 8 Araca kadar /rover_XX/cmd_vel dinle
        self.cmdvel_subs = []
        for i in range(1, 9):
            code = f"rover_{i:02d}"
            sub = self.create_subscription(
                Twist,
                f'/{code}/cmd_vel',
                lambda msg, c=code: self._on_ros_cmdvel(c, msg),
                10
            )
            self.cmdvel_subs.append(sub)

        # Ortak /cmd_vel dinleyicisi
        self.cmdvel_all_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            lambda msg: self._on_ros_cmdvel("all", msg),
            10
        )

        # Zenoh Subscriber'ları (Zenoh -> ROS 2)
        self.zenoh_sub_chat = self.zenoh_session.declare_subscriber("pisim/chat", self._on_zenoh_chat)
        self.zenoh_sub_imu = self.zenoh_session.declare_subscriber("pisim/*/imu", self._on_zenoh_imu)
        self.zenoh_sub_gps = self.zenoh_session.declare_subscriber("pisim/*/gps", self._on_zenoh_gps)

        self.get_logger().info("🎉 Köprü Hazır! ROS 2 topic'leri ile Zenoh ağı bağlandı.")

    # ------------------ ROS 2 -> ZENOH ------------------

    def _on_ros_chat(self, msg: String):
        """ROS 2 `/pisim/chat` topic'inden gelen mesajı Zenoh'a fırlatır."""
        try:
            # Döngüyü engellemek için kendi fırlattığımız mesajları kontrol et
            payload = {
                "sender": "pi5_native_ros2",
                "device": "RaspberryPi5",
                "text": msg.data,
                "timestamp": time.time()
            }
            self.zenoh_session.put("pisim/chat", json.dumps(payload))
        except Exception as e:
            self.get_logger().error(f"Zenoh chat iletim hatası: {e}")

    def _on_ros_cmdvel(self, rover_code: str, msg: Twist):
        """ROS 2 `/<rover>/cmd_vel` topic'inden gelen Twist'i Zenoh'a fırlatır."""
        try:
            target_topic = f"pisim/{rover_code}/cmd_vel" if rover_code != "all" else "pisim/cmd_vel"
            payload = {
                "linear": {"x": float(msg.linear.x), "y": float(msg.linear.y), "z": float(msg.linear.z)},
                "angular": {"x": float(msg.angular.x), "y": float(msg.angular.y), "z": float(msg.angular.z)},
                "linear_x": float(msg.linear.x),
                "angular_z": float(msg.angular.z)
            }
            self.zenoh_session.put(target_topic, json.dumps(payload))
        except Exception as e:
            self.get_logger().error(f"Zenoh cmd_vel iletim hatası: {e}")

    # ------------------ ZENOH -> ROS 2 ------------------

    def _on_zenoh_chat(self, sample):
        """Zenoh `pisim/chat`'ten gelen mesajı yerel ROS 2 `/pisim/chat` topic'ine basar."""
        try:
            pkt = json.loads(sample.payload.to_string())
            sender = pkt.get("sender", "")
            if sender == "pi5_native_ros2":
                return  # Kendimiz attıysak geri besleme yapma
            text = pkt.get("text", "")
            ros_msg = String()
            ros_msg.data = f"[{sender}]: {text}"
            self.ros_chat_pub.publish(ros_msg)
        except Exception:
            pass

    def _on_zenoh_imu(self, sample):
        """Zenoh `pisim/<rover>/imu` verisini gerçek ROS 2 `sensor_msgs/msg/Imu`'ya çevirir."""
        try:
            topic_str = str(sample.key_expr)  # örn: pisim/rover_01/imu
            parts = topic_str.split("/")
            rover_code = parts[1] if len(parts) >= 2 else "rover_01"

            if rover_code not in self.ros_imu_pubs:
                self.ros_imu_pubs[rover_code] = self.create_publisher(Imu, f'/{rover_code}/imu', 10)

            data = json.loads(sample.payload.to_string())
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = f"{rover_code}_base_link"

            # İvme
            acc = data.get("linear_acceleration", {})
            imu_msg.linear_acceleration.x = float(acc.get("x", 0.0))
            imu_msg.linear_acceleration.y = float(acc.get("y", 0.0))
            imu_msg.linear_acceleration.z = float(acc.get("z", 0.0))

            # Açısal Hız
            gyr = data.get("angular_velocity", {})
            imu_msg.angular_velocity.x = float(gyr.get("x", 0.0))
            imu_msg.angular_velocity.y = float(gyr.get("y", 0.0))
            imu_msg.angular_velocity.z = float(gyr.get("z", 0.0))

            self.ros_imu_pubs[rover_code].publish(imu_msg)
        except Exception:
            pass

    def _on_zenoh_gps(self, sample):
        """Zenoh `pisim/<rover>/gps` verisini gerçek ROS 2 `sensor_msgs/msg/NavSatFix`'e çevirir."""
        try:
            topic_str = str(sample.key_expr)
            parts = topic_str.split("/")
            rover_code = parts[1] if len(parts) >= 2 else "rover_01"

            if rover_code not in self.ros_gps_pubs:
                self.ros_gps_pubs[rover_code] = self.create_publisher(NavSatFix, f'/{rover_code}/gps', 10)

            data = json.loads(sample.payload.to_string())
            gps_msg = NavSatFix()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = f"{rover_code}_gps_link"
            gps_msg.latitude = float(data.get("latitude", 0.0))
            gps_msg.longitude = float(data.get("longitude", 0.0))
            gps_msg.altitude = float(data.get("altitude", 0.0))

            self.ros_gps_pubs[rover_code].publish(gps_msg)
        except Exception:
            pass

    def destroy_node(self):
        if self.zenoh_session:
            self.zenoh_session.close()
        super().destroy_node()


def main(args=None):
    if not HAS_ROS2:
        print("⚠️ Pi 5 üzerinde ROS 2 kurulu olmalıdır.")
        sys.exit(1)

    rclpy.init(args=args)
    bridge = Pi5Ros2ZenohBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

