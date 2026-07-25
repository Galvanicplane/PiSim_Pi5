#!/usr/bin/env python3
"""
pisim_core.py - Python Core Bridge for PiSim Bridge Platform
Communicates with UE5 simulation over UDP:
- Transmits /cmd_vel (Twist) TO UE5 on UDP Port 7400 (Real-time non-blocking WASD controls)
- Listens for /sim/imu (IMU Telemetry) FROM UE5 on UDP Port 7401
- Listens for /sim/camera (FPV Live Video) FROM UE5 on UDP Port 5000
"""

import socket
import struct
import threading
import time
import sys

# Port Architecture Configuration
UE5_CONTROL_PORT = 7400      # UE5 listens for /cmd_vel on 7400
PI5_TELEMETRY_PORT = 7401    # Pi5 listens for /sim/imu on 7401
PI5_VIDEO_PORT = 5000        # Pi5 listens for video frames on 5000
TARGET_HOST = "192.168.1.10"  # UE5 Host PC IP address (Ethernet)

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
        
        print("\n========================================================")
        print("    PiSim Core Bridge - Real-Time Control & Telemetry   ")
        print("========================================================")
        print(f"  [+] Target UE5 Host IP    : {self.target_host}")
        print(f"  [+] Sending /cmd_vel TO   : UDP Port {self.control_port}")
        print(f"  [+] Listening /sim/imu ON : UDP Port {self.telemetry_port}")
        print(f"  [+] Listening FPV Video ON: UDP Port {self.video_port}")
        print("========================================================\n")

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
        print(f"--> [TX /cmd_vel] Linear X: {linear_x:+.2f} m/s | Angular Z: {angular_z:+.2f} rad/s")

    def _telemetry_loop(self):
        print(f"[*] Telemetry listener active on UDP Port {self.telemetry_port}...")
        while self.running:
            try:
                data, addr = self.recv_sock.recvfrom(4096)
                if len(data) == IMU_SIZE:
                    unpacked = struct.unpack(IMU_FORMAT, data)
                    qx, qy, qz, qw = unpacked[0:4]
                    gx, gy, gz = unpacked[4:7]
                    ax, ay, az = unpacked[7:10]
                    
                    print(f"<-- [RX /sim/imu] Orient: ({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f}) | Accel Z: {az:+.2f} m/s²")
            except Exception:
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

        frame_count = 0
        try:
            while self.running:
                data, addr = video_sock.recvfrom(65507)
                if not data:
                    continue
                np_arr = np.frombuffer(data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if frame is not None:
                    frame_count += 1
                    if frame_count % 30 == 0:
                        print(f"<-- [RX FPV Frame] #{frame_count} ({frame.shape[1]}x{frame.shape[0]}) from {addr[0]}")
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


def get_key_nonblocking():
    """Reads a single keypress instantly without pressing Enter (Linux/Pi5 & Windows)."""
    if sys.platform == "win32":
        import msvcrt
        if msvcrt.kbhit():
            try:
                ch = msvcrt.getch().decode('utf-8', errors='ignore').lower()
                return ch
            except Exception:
                return ""
        return ""
    else:
        # Linux / Raspberry Pi 5
        import select
        import tty
        import termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.02)
            if rlist:
                ch = sys.stdin.read(1).lower()
                return ch
            return ""
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def realtime_teleop_cli(bridge):
    print("\n========================================================")
    print("   🎮 Instant WASD Real-Time Teleoperation Console      ")
    print("========================================================")
    print("  [W] -> Move Forward (1.5 m/s)")
    print("  [S] -> Move Backward (-1.5 m/s)")
    print("  [A] -> Yaw Left (+0.8 rad/s)")
    print("  [D] -> Yaw Right (-0.8 rad/s)")
    print("  [SPACE / X] -> Immediate Emergency Stop (0 m/s)")
    print("  [Q] -> Exit Teleoperation")
    print("========================================================\n")
    print("[*] Press keys directly (NO ENTER NEEDED). Ready!\n")

    current_linear = 0.0
    current_angular = 0.0

    try:
        while bridge.running:
            key = get_key_nonblocking()
            if key:
                if key == 'w':
                    current_linear = 1.5
                    current_angular = 0.0
                    bridge.publish_cmd_vel(linear_x=current_linear, angular_z=current_angular)
                elif key == 's':
                    current_linear = -1.5
                    current_angular = 0.0
                    bridge.publish_cmd_vel(linear_x=current_linear, angular_z=current_angular)
                elif key == 'a':
                    current_angular = 0.8
                    bridge.publish_cmd_vel(linear_x=current_linear, angular_z=current_angular)
                elif key == 'd':
                    current_angular = -0.8
                    bridge.publish_cmd_vel(linear_x=current_linear, angular_z=current_angular)
                elif key in [' ', 'x']:
                    current_linear = 0.0
                    current_angular = 0.0
                    bridge.publish_cmd_vel(linear_x=0.0, angular_z=0.0)
                elif key == 'q':
                    print("\n[*] Exiting teleoperation console...")
                    bridge.publish_cmd_vel(linear_x=0.0, angular_z=0.0)
                    break
            time.sleep(0.02)
    except KeyboardInterrupt:
        bridge.publish_cmd_vel(linear_x=0.0, angular_z=0.0)


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
        realtime_teleop_cli(bridge)
        bridge.stop()
