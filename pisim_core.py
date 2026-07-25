#!/usr/bin/env python3
"""
pisim_core.py - Python Core Bridge for PiSim Bridge Platform
Communicates with UE5 simulation over UDP Port 7400 (Control & IMU Telemetry) and UDP Port 5000 (FPV Live Video).
Publishes velocity commands (/cmd_vel), receives IMU telemetry (/sim/imu), and displays live FPV camera feed.
"""

import socket
import struct
import threading
import time
import sys

# Dual-Port Configuration
CONTROL_PORT = 7400
VIDEO_PORT = 5000
TARGET_HOST = "127.0.0.1"

# Binary Struct Formats matching C++ #pragma pack(push, 1)
TWIST_FORMAT = "<6d"
TWIST_SIZE = struct.calcsize(TWIST_FORMAT)

IMU_FORMAT = "<10d"
IMU_SIZE = struct.calcsize(IMU_FORMAT)


class PiSimCoreBridge:
    def __init__(self, target_host=TARGET_HOST, control_port=CONTROL_PORT, video_port=VIDEO_PORT):
        self.target_host = target_host
        self.control_port = control_port
        self.video_port = video_port
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", self.control_port))
        
        self.running = False
        self.recv_thread = None
        self.video_thread = None
        self.enable_video = False
        
        print(f"[*] PiSim Core Bridge initialized.")
        print(f"    - Control & Telemetry Port: {self.control_port} UDP")
        print(f"    - FPV Video Stream Port   : {self.video_port} UDP")

    def start(self, enable_video=True):
        self.running = True
        self.enable_video = enable_video
        
        # Telemetry receive thread
        self.recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.recv_thread.start()
        print(f"[*] Listening for /sim/imu telemetry on UDP Port {self.control_port}...")

        # Video stream receive thread
        if self.enable_video:
            self.video_thread = threading.Thread(target=self._video_loop, daemon=True)
            self.video_thread.start()

    def stop(self):
        self.running = False
        if self.sock:
            self.sock.close()
        print("[*] PiSim Core Bridge shut down.")

    def publish_cmd_vel(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_x=0.0, angular_y=0.0, angular_z=0.0):
        """Packs and transmits geometry_msgs/msg/Twist CDR payload over UDP."""
        data = struct.pack(TWIST_FORMAT, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)
        self.sock.sendto(data, (self.target_host, self.control_port))
        print(f"[TX /cmd_vel] Linear: ({linear_x:.2f}, {linear_y:.2f}, {linear_z:.2f}) m/s | Angular: ({angular_z:.2f}) rad/s")

    def _receive_loop(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(4096)
                if len(data) == IMU_SIZE:
                    unpacked = struct.unpack(IMU_FORMAT, data)
                    qx, qy, qz, qw = unpacked[0:4]
                    gx, gy, gz = unpacked[4:7]
                    ax, ay, az = unpacked[7:10]
                    
                    print(f"[RX /sim/imu] Orient Quat: ({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f}) | Gyro: ({gx:.2f}, {gy:.2f}, {gz:.2f}) rad/s | Accel: ({ax:.2f}, {ay:.2f}, {az:.2f}) m/s²")
            except Exception as e:
                if not self.running:
                    break

    def _video_loop(self):
        """Listens on UDP Port 5000 for JPEG video frames and displays window using OpenCV if available."""
        try:
            import cv2
            import numpy as np
        except ImportError:
            print("[!] OpenCV (cv2) or numpy not installed. FPV Video streaming window skipped.")
            return

        video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        video_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        video_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        video_sock.bind(("0.0.0.0", self.video_port))

        print(f"[*] FPV Video receiver listening on UDP Port {self.video_port}...")
        window_name = "PiSim Pi5 Live FPV Stream (Port 5000)"

        try:
            while self.running:
                data, addr = video_sock.recvfrom(65507)
                if not data:
                    continue
                np_arr = np.frombuffer(data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if frame is not None:
                    cv2.imshow(window_name, frame)
                    cv2.waitKey(1)
        except Exception:
            pass
        finally:
            video_sock.close()
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass


def interactive_cli(bridge):
    print("\n==========================================")
    print("   PiSim Bridge - Control & Video CLI     ")
    print("==========================================")
    print("Commands:")
    print("  w - Move Forward (1.0 m/s)")
    print("  s - Move Backward (-1.0 m/s)")
    print("  a - Yaw Left (0.5 rad/s)")
    print("  d - Yaw Right (-0.5 rad/s)")
    print("  space / x - Stop")
    print("  q - Quit")
    print("==========================================\n")

    try:
        while True:
            cmd = input("Enter command (w/a/s/d/x/q): ").strip().lower()
            if cmd == 'w':
                bridge.publish_cmd_vel(linear_x=1.0)
            elif cmd == 's':
                bridge.publish_cmd_vel(linear_x=-1.0)
            elif cmd == 'a':
                bridge.publish_cmd_vel(angular_z=0.5)
            elif cmd == 'd':
                bridge.publish_cmd_vel(angular_z=-0.5)
            elif cmd in ['x', ' ']:
                bridge.publish_cmd_vel(linear_x=0.0, angular_z=0.0)
            elif cmd == 'q':
                break
            else:
                print("Unknown command. Use w, a, s, d, x, or q.")
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    bridge = PiSimCoreBridge()
    
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("[*] Running automated /cmd_vel test sequence...")
        bridge.start(enable_video=False)
        bridge.publish_cmd_vel(linear_x=1.0)
        time.sleep(1.0)
        bridge.publish_cmd_vel(angular_z=0.5)
        time.sleep(1.0)
        bridge.publish_cmd_vel(linear_x=0.0, angular_z=0.0)
        bridge.stop()
    else:
        bridge.start(enable_video=True)
        interactive_cli(bridge)
        bridge.stop()
