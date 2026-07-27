#!/usr/bin/env python3
"""
pisim_ros2_node.py - All-In-One Unified ROS 2 & Teleop Node for PiSim Platform
Features in ONE single terminal & window:
1. Interactive Non-Blocking Single-Key WASD Driving (W/A/S/D/Space/Q)
2. Live OpenCV FPV Video Stream Window (UDP 5000 / ROS 2 CompressedImage)
3. Live IMU Sensor Telemetry Stream Display (UDP 7401 / ROS 2 Imu)
4. Native ROS 2 Topic Publisher (/sim/imu, /sim/camera) and Subscriber (/cmd_vel)
"""

import socket
import struct
import threading
import time
import sys
import numpy as np
import cv2

# Platform keyboard non-blocking input
try:
    import termios
    import tty
    import select
    WINDOWS = False
except ImportError:
    import msvcrt
    WINDOWS = True

# Optional ROS 2 Imports
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Imu, CompressedImage
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

UE5_CONTROL_PORT = 7400
PI5_TELEMETRY_PORT = 7401
PI5_VIDEO_PORT = 5000
TARGET_HOST = "192.168.1.10"

TWIST_FORMAT = "<6d"
IMU_FORMAT = "<10d"


def get_key():
    """ Read single keypress without waiting for Enter key """
    if WINDOWS:
        if msvcrt.kbhit():
            ch = msvcrt.getch()
            try:
                return ch.decode('utf-8').lower()
            except Exception:
                return ""
        return ""
    else:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if rlist:
                ch = sys.stdin.read(1)
                return ch.lower()
            return ""
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


class UnifiedPiSimROS2Bridge:
    def __init__(self, target_host=TARGET_HOST):
        self.target_host = target_host
        
        # Sockets
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.recv_sock.bind(("0.0.0.0", PI5_TELEMETRY_PORT))

        self.video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.video_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.video_sock.bind(("0.0.0.0", PI5_VIDEO_PORT))

        self.running = True
        self.linear_vel = 0.0
        self.angular_vel = 0.0

        # ROS 2 Node setup if available
        self.ros2_node = None
        if ROS2_AVAILABLE:
            try:
                rclpy.init(args=None)
                self.ros2_node = Node('pisim_unified_node')
                self.imu_pub = self.ros2_node.create_publisher(Imu, '/sim/imu', 10)
                self.image_pub = self.ros2_node.create_publisher(CompressedImage, '/sim/camera/image_raw/compressed', 10)
                self.cmd_vel_sub = self.ros2_node.create_subscription(Twist, '/cmd_vel', self._on_ros2_cmd_vel, 10)
            except Exception:
                self.ros2_node = None

        print("\n========================================================")
        print("    PiSim ALL-IN-ONE Unified ROS 2 & Teleop Node        ")
        print("========================================================")
        print(f"  [+] Target UE5 Host IP    : {self.target_host}")
        print(f"  [+] Sending /cmd_vel TO   : UDP Port {UE5_CONTROL_PORT}")
        print(f"  [+] Listening /sim/imu ON : UDP Port {PI5_TELEMETRY_PORT}")
        print(f"  [+] Listening FPV Video ON: UDP Port {PI5_VIDEO_PORT}")
        print(f"  [+] ROS 2 Integration     : {'ACTIVE (/cmd_vel, /sim/imu, /sim/camera)' if ROS2_AVAILABLE else 'STANDALONE UDP'}")
        print("--------------------------------------------------------")
        print("  🎮 INSTANT WASD TELEOP CONTROLS (No Enter needed):")
        print("     [W] Forward   | [S] Backward")
        print("     [A] Turn Left | [D] Turn Right")
        print("     [SPACE] Stop  | [Q] Quit Application")
        print("========================================================\n")

    def _on_ros2_cmd_vel(self, msg):
        self.send_twist(msg.linear.x, msg.angular.z)

    def send_twist(self, linear_x, angular_z):
        packet = struct.pack(TWIST_FORMAT, linear_x, 0.0, 0.0, 0.0, 0.0, angular_z)
        self.send_sock.sendto(packet, (self.target_host, UE5_CONTROL_PORT))

    def _telemetry_loop(self):
        last_print = 0
        while self.running:
            try:
                data, addr = self.recv_sock.recvfrom(1024)
                if len(data) >= IMU_SIZE:
                    unpacked = struct.unpack(IMU_FORMAT, data[:IMU_SIZE])
                    qx, qy, qz, qw = unpacked[0:4]
                    wx, wy, wz = unpacked[4:7]
                    ax, ay, az = unpacked[7:10]

                    # Publish to ROS 2 if available
                    if self.ros2_node:
                        imu_msg = Imu()
                        imu_msg.header.stamp = self.ros2_node.get_clock().now().to_msg()
                        imu_msg.header.frame_id = "imu_link"
                        imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = qx, qy, qz, qw
                        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = wx, wy, wz
                        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = ax, ay, az
                        self.imu_pub.publish(imu_msg)

                    now = time.time()
                    if now - last_print >= 0.5:
                        last_print = now
                        sys.stdout.write(f"\r<-- [RX IMU] Accel: (X:{ax:+.2f}, Y:{ay:+.2f}, Z:{az:+.2f}) m/s² | Gyro Z: {wz:+.2f} rad/s   ")
                        sys.stdout.flush()
            except Exception:
                pass

    def _video_loop(self):
        window_name = "PiSim Live FPV Camera Stream (ROS 2 / UDP 5000)"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 640, 480)

        frame_chunks = {}
        frame_seq_count = 0

        while self.running:
            try:
                data, addr = self.video_sock.recvfrom(65535)
                if len(data) < 4:
                    continue

                frame_seq, chunk_idx, total_chunks = struct.unpack(">HBB", data[:4])
                chunk_payload = data[4:]

                if frame_seq not in frame_chunks:
                    frame_chunks[frame_seq] = {}

                frame_chunks[frame_seq][chunk_idx] = chunk_payload

                if len(frame_chunks[frame_seq]) == total_chunks:
                    full_jpeg = b"".join(frame_chunks[frame_seq][i] for i in range(total_chunks))
                    np_arr = np.frombuffer(full_jpeg, dtype=np.uint8)
                    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                    if frame is not None:
                        frame_seq_count += 1
                        cv2.imshow(window_name, frame)
                        cv2.waitKey(1)

                        # Publish to ROS 2 if available
                        if self.ros2_node:
                            img_msg = CompressedImage()
                            img_msg.header.stamp = self.ros2_node.get_clock().now().to_msg()
                            img_msg.header.frame_id = "camera_optical_frame"
                            img_msg.format = "jpeg"
                            img_msg.data = list(full_jpeg)
                            self.image_pub.publish(img_msg)

                    frame_chunks = {}
            except Exception:
                pass

        cv2.destroyAllWindows()

    def run(self):
        t_telemetry = threading.Thread(target=self._telemetry_loop, daemon=True)
        t_video = threading.Thread(target=self._video_loop, daemon=True)

        t_telemetry.start()
        t_video.start()

        step_lin = 0.5
        step_ang = 0.5

        try:
            while self.running:
                # Spin ROS 2 callbacks once if ROS 2 node is active
                if self.ros2_node:
                    rclpy.spin_once(self.ros2_node, timeout_sec=0.01)

                key = get_key()
                if key:
                    if key == 'w':
                        self.linear_vel += step_lin
                        print(f"\n[Teleop] FORWARD  | Lin: {self.linear_vel:.2f} m/s, Ang: {self.angular_vel:.2f} rad/s")
                    elif key == 's':
                        self.linear_vel -= step_lin
                        print(f"\n[Teleop] BACKWARD | Lin: {self.linear_vel:.2f} m/s, Ang: {self.angular_vel:.2f} rad/s")
                    elif key == 'a':
                        self.angular_vel += step_ang
                        print(f"\n[Teleop] LEFT     | Lin: {self.linear_vel:.2f} m/s, Ang: {self.angular_vel:.2f} rad/s")
                    elif key == 'd':
                        self.angular_vel -= step_ang
                        print(f"\n[Teleop] RIGHT    | Lin: {self.linear_vel:.2f} m/s, Ang: {self.angular_vel:.2f} rad/s")
                    elif key == ' ':
                        self.linear_vel = 0.0
                        self.angular_vel = 0.0
                        print(f"\n[Teleop] STOP     | Emergency Brake Applied")
                    elif key == 'q':
                        print("\n[Teleop] Exiting Application...")
                        self.running = False
                        break

                    self.send_twist(self.linear_vel, self.angular_vel)
                else:
                    time.sleep(0.02)
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            if self.ros2_node:
                self.ros2_node.destroy_node()
                rclpy.shutdown()
            print("\n[PiSim] Application terminated cleanly.")


if __name__ == '__main__':
    bridge = UnifiedPiSimROS2Bridge()
    bridge.run()
