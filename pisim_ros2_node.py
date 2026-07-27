#!/usr/bin/env python3
"""
pisim_ros2_node.py - Standalone ROS 2 Native Node for PiSim Platform
Subscribes to ROS 2 topics:
- /cmd_vel (geometry_msgs/msg/Twist) -> Sends to UE5
- /sim/imu (sensor_msgs/msg/Imu) -> Publishes ROS 2 Topic
- /sim/camera/image_raw/compressed (sensor_msgs/msg/CompressedImage) -> Publishes ROS 2 Topic
"""

import socket
import struct
import threading
import time
import sys

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Imu, CompressedImage
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("[WARNING] rclpy/ROS 2 libraries not detected in current Python environment.")
    print("[INFO] Fallback mode active. Use 'python3 pisim_core.py' for non-ROS2 standalone UDP teleop.")

UE5_CONTROL_PORT = 7400
PI5_TELEMETRY_PORT = 7401
PI5_VIDEO_PORT = 5000
TARGET_HOST = "192.168.1.10"

TWIST_FORMAT = "<6d"
IMU_FORMAT = "<10d"


if ROS2_AVAILABLE:
    class PiSimROS2Node(Node):
        def __init__(self, target_host=TARGET_HOST):
            super().__init__('pisim_simulation_node')
            self.target_host = target_host

            # ROS 2 Subscriptions & Publishers
            self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
            self.imu_pub = self.create_publisher(Imu, '/sim/imu', 10)
            self.image_pub = self.create_publisher(CompressedImage, '/sim/camera/image_raw/compressed', 10)

            # UDP Sockets
            self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.recv_sock.bind(("0.0.0.0", PI5_TELEMETRY_PORT))

            self.video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.video_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.video_sock.bind(("0.0.0.0", PI5_VIDEO_PORT))

            self.running = True
            self.telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
            self.video_thread = threading.Thread(target=self._video_loop, daemon=True)

            self.telemetry_thread.start()
            self.video_thread.start()

            self.get_logger().info(f"PiSim Native ROS 2 Node initialized. Target UE5 Host: {self.target_host}")

        def cmd_vel_callback(self, msg: Twist):
            # Encode geometry_msgs/Twist to CDR binary payload
            packet = struct.pack(
                TWIST_FORMAT,
                msg.linear.x, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y, msg.angular.z
            )
            self.send_sock.sendto(packet, (self.target_host, UE5_CONTROL_PORT))

        def _telemetry_loop(self):
            while self.running and rclpy.ok():
                try:
                    data, addr = self.recv_sock.recvfrom(1024)
                    if len(data) >= struct.calcsize(IMU_FORMAT):
                        unpacked = struct.unpack(IMU_FORMAT, data[:struct.calcsize(IMU_FORMAT)])
                        
                        imu_msg = Imu()
                        imu_msg.header.stamp = self.get_clock().now().to_msg()
                        imu_msg.header.frame_id = "imu_link"

                        imu_msg.orientation.x = unpacked[0]
                        imu_msg.orientation.y = unpacked[1]
                        imu_msg.orientation.z = unpacked[2]
                        imu_msg.orientation.w = unpacked[3]

                        imu_msg.angular_velocity.x = unpacked[4]
                        imu_msg.angular_velocity.y = unpacked[5]
                        imu_msg.angular_velocity.z = unpacked[6]

                        imu_msg.linear_acceleration.x = unpacked[7]
                        imu_msg.linear_acceleration.y = unpacked[8]
                        imu_msg.linear_acceleration.z = unpacked[9]

                        self.imu_pub.publish(imu_msg)
                except Exception:
                    pass

        def _video_loop(self):
            frame_chunks = {}
            while self.running and rclpy.ok():
                try:
                    data, addr = self.video_sock.recvfrom(65535)
                    if len(data) < 4:
                        continue
                    frame_seq, chunk_idx, total_chunks = struct.unpack(">HBB", data[:4])
                    chunk_data = data[4:]

                    if frame_seq not in frame_chunks:
                        frame_chunks[frame_seq] = {}
                    frame_chunks[frame_seq][chunk_idx] = chunk_data

                    if len(frame_chunks[frame_seq]) == total_chunks:
                        full_jpeg = b"".join(frame_chunks[frame_seq][i] for i in range(total_chunks))
                        
                        img_msg = CompressedImage()
                        img_msg.header.stamp = self.get_clock().now().to_msg()
                        img_msg.header.frame_id = "camera_optical_frame"
                        img_msg.format = "jpeg"
                        img_msg.data = list(full_jpeg)

                        self.image_pub.publish(img_msg)
                        frame_chunks = {}
                except Exception:
                    pass


def main():
    if not ROS2_AVAILABLE:
        sys.exit(1)

    rclpy.init()
    node = PiSimROS2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
