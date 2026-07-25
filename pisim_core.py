#!/usr/bin/env python3
"""
pisim_core.py - Python Core Bridge for PiSim Bridge Platform
Communicates with UE5 simulation over UDP:
- Transmits /cmd_vel (Twist) TO UE5 on UDP Port 7400
- Listens for /sim/imu (IMU Telemetry) FROM UE5 on UDP Port 7401
- Listens for /sim/camera (FPV Live Video) FROM UE5 on UDP Port 5000
"""

import socket
import struct
import threading
import time
import sys

# Port Architecture Configuration
UE5_CONTROL_PORT = 7400     # UE5 listens for /cmd_vel on 7400
PI5_TELEMETRY_PORT = 7401   # Pi5 listens for /sim/imu on 7401
PI5_VIDEO_PORT = 5000       # Pi5 listens for video frames on 5000
TARGET_HOST = "127.0.0.1"   # UE5 Host IP address

# Binary Struct Formats matching C++ #pragma pack(push, 1)
TWIST_FORMAT = "<6d"
TWIST_SIZE = struct.calcsize(TWIST_FORMAT)

IMU_FORMAT = "<10d"
IMU_SIZE = struct.calcsize(IMU_FORMAT)


class PiSimCoreBridge:
    def __init__(self, target_host=TARGET_HOST, control_port=UE5_CONTROL_PORT, telemetry_port=PI5_TELEMETRY_PORT, video_port=PI5_VIDEO_PORT):
        self.target_host = target_host
        self.control_port = control_port
        self.telemetry_port = telemetry_port
        self.video_port = video_port
        
        # Sender Socket (for transmitting /cmd_vel to UE5)
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Receiver Socket (for listening to /sim/imu on 7401)
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.recv_sock.bind(("0.0.0.0", self.telemetry_port))
        
        self.running = False
        self.recv_thread = None
        self.video_thread = None
        self.enable_video = False
        
        print(f"[*] PiSim Core Bridge Initialized.")
        print(f"    - Target UE5 Host IP    : {self.target_host}")
        print(f"    - Sending /cmd_vel TO   : UDP Port {self.control_port}")
        print(f"    - Listening /sim/imu ON : UDP Port {self.telemetry_port}")
        print(f"    - Listening FPV Video ON: UDP Port {self.video_port}")

    def start(self, enable_video=True):
        self.running = True
        self.enable_video = enable_video
        
        # Start Telemetry Receiver Thread
        self.recv_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self.recv_thread.start()

        # Start Video Receiver Thread
        if self.enable_video:
            self.video_thread = threading.Thread(target=self._video_loop, daemon=True)
            self.video_thread.start()

    def stop(self):
        self.running = False
        if self.send_sock:
            self.send_sock.close()
        if self.recv_sock:
            self.recv_sock.close()
        print("[*] PiSim Core Bridge shut down.")

    def publish_cmd_vel(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_x=0.0, angular_y=0.0, angular_z=0.0):
        """Packs and transmits geometry_msgs/msg/Twist CDR payload over UDP."""
        data = struct.pack(TWIST_FORMAT, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)
        self.send_sock.sendto(data, (self.target_host, self.control_port))
        print(f"[TX /cmd_vel] Linear: ({linear_x:.2f}, {linear_y:.2f}, {linear_z:.2f}) m/s | Angular: ({angular_z:.2f}) rad/s")

    def _telemetry_loop(self):
        print(f"[*] Listening for /sim/imu telemetry on UDP Port {self.telemetry_port}...")
        while self.running:
            try:
                data, addr = self.recv_sock.recvfrom(4096)
                if len(data) == IMU_SIZE:
                    unpacked = struct.unpack(IMU_FORMAT, data)
                    qx, qy, qz, qw = unpacked[0:4]
                    gx, gy, gz = unpacked[4:7]
                    ax, ay, az = unpacked[7:10]
                    
                    print(f"[RX /sim/imu] Orient: ({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f}) | Gyro: ({gx:.2f}, {gy:.2f}, {gz:.2f}) | Accel: ({ax:.2f}, {ay:.2f}, {az:.2f})")
            except Exception as e:
                if not self.running:
                    break

    def _video_loop(self):
        """Listens on UDP Port 5000 for JPEG video frames and displays window using OpenCV."""
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

        print(f"[*] FPV Video receiver active on UDP Port {self.video_port}...")
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
    print("Commands (press Enter after letter):")
    print("  w - Move Forward (1.0 m/s)")
    print("  s - Move Backward (-1.0 m/s)")
    print("  a - Yaw Left (0.5 rad/s)")
    print("  d - Yaw Right (-0.5 rad/s)")
    print("  space / x - Stop")
    print("  q - Quit")
    print("==========================================\n")

    try:
        while True:
            cmd = input("Command > ").strip().lower()
            if cmd == 'w':
                bridge.publish_cmd_vel(linear_x=1.0)
            elif cmd == 's':
                bridge.publish_cmd_vel(linear_x=-1.0)
            elif cmd == 'a':
                bridge.publish_cmd_vel(angular_z=0.5)
            elif cmd == 'd':
                bridge.publish_cmd_vel(angular_z=-0.5)
            elif cmd in ['x', ' ', '']:
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
