#!/usr/bin/env python3
"""
pisim_dashboard.py - Interactive Live Terminal Telemetry & Control Dashboard
Universal Edge Bridge for Raspberry Pi 5, NVIDIA Jetson, Linux & Windows PCs.

Features:
- Live ANSI Cyber-Robotics Terminal Dashboard
- Zero-latency Non-Blocking Single-Key WASD Teleoperation (No Enter required)
- Auto-Handshake Beacon on start (UE5 locks onto Sender IP automatically)
- Real-time IMU Telemetry (Orientation Quaternion, Gyro, Accel, Euler Roll/Pitch/Yaw)
- Packet Rate (Hz), Round-Trip Heartbeat, and Diagnostic Counters
"""

import socket
import struct
import threading
import time
import sys
import os
import math

# ==============================================================================
# DEFAULT NETWORK CONFIGURATION
# ==============================================================================
DEFAULT_UE5_HOST = "127.0.0.1"    # Default: local host (change to PC IP if on Pi 5 Ethernet)
CONTROL_PORT = 7400                # UE5 listens on UDP 7400
TELEMETRY_PORT = 7401              # Pi 5 listens on UDP 7401

# Binary Struct Formats (matching UE5 #pragma pack(push, 1))
TWIST_FORMAT = "<6d"               # 48 bytes (LinearX, Y, Z, AngularX, Y, Z)
TWIST_SIZE = struct.calcsize(TWIST_FORMAT)

IMU_FORMAT = "<10d"                # 80 bytes (QuatX, Y, Z, W, GyroX, Y, Z, AccX, Y, Z)
IMU_SIZE = struct.calcsize(IMU_FORMAT)


def quaternion_to_euler(x, y, z, w):
    """Converts quaternion to Euler angles (Roll, Pitch, Yaw) in degrees."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def get_key_nonblocking():
    """Reads a single keypress without waiting for Enter."""
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


class PiSimLiveDashboard:
    def __init__(self, target_host=DEFAULT_UE5_HOST):
        self.target_host = target_host
        self.control_port = CONTROL_PORT
        self.telemetry_port = TELEMETRY_PORT

        # Sockets
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.recv_sock.bind(("0.0.0.0", self.telemetry_port))

        self.running = False
        self.recv_thread = None

        # Control States
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0

        # Telemetry States
        self.rx_count = 0
        self.tx_count = 0
        self.last_rx_time = 0.0
        self.rx_rate_hz = 0.0
        self.tx_rate_hz = 0.0

        self.quat = (0.0, 0.0, 0.0, 1.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.accel = (0.0, 0.0, 0.0)
        self.euler = (0.0, 0.0, 0.0)

    def start(self):
        self.running = True

        # Start Telemetry Listener Thread
        self.recv_thread = threading.Thread(target=self._telemetry_listener, daemon=True)
        self.recv_thread.start()

        # Send initial handshake beacon so UE5 locks onto our IP
        self.send_command(0.0, 0.0)

    def stop(self):
        self.running = False
        self.send_command(0.0, 0.0)
        if self.send_sock:
            self.send_sock.close()
        if self.recv_sock:
            self.recv_sock.close()

    def send_command(self, linear_x, angular_z):
        """Packs and sends geometry_msgs/Twist over UDP Port 7400."""
        self.cmd_linear_x = linear_x
        self.cmd_angular_z = angular_z
        data = struct.pack(TWIST_FORMAT, linear_x, 0.0, 0.0, 0.0, 0.0, angular_z)
        try:
            self.send_sock.sendto(data, (self.target_host, self.control_port))
            self.tx_count += 1
        except Exception as e:
            pass

    def _telemetry_listener(self):
        """Receives live IMU telemetry from UE5 on UDP Port 7401."""
        last_calc_time = time.time()
        rx_window = 0

        while self.running:
            try:
                self.recv_sock.settimeout(0.2)
                data, addr = self.recv_sock.recvfrom(2048)
                if len(data) == IMU_SIZE:
                    unpacked = struct.unpack(IMU_FORMAT, data)
                    self.quat = unpacked[0:4]
                    self.gyro = unpacked[4:7]
                    self.accel = unpacked[7:10]
                    self.euler = quaternion_to_euler(*self.quat)

                    self.rx_count += 1
                    rx_window += 1
                    self.last_rx_time = time.time()

                now = time.time()
                dt = now - last_calc_time
                if dt >= 0.5:
                    self.rx_rate_hz = rx_window / dt
                    rx_window = 0
                    last_calc_time = now
            except socket.timeout:
                pass
            except Exception:
                if not self.running:
                    break

    def render_dashboard(self):
        """Draws the rich ANSI terminal dashboard."""
        os.system('cls' if os.name == 'nt' else 'clear')

        time_since_rx = time.time() - self.last_rx_time if self.last_rx_time > 0 else 999.0
        is_linked = time_since_rx < 2.0

        status_badge = "\033[92m● LINK ACTIVE (CONNECTED)\033[0m" if is_linked else "\033[93m○ WAITING FOR SIMULATOR...\033[0m"

        print("\033[96m╔══════════════════════════════════════════════════════════════════════════════════╗\033[0m")
        print("\033[96m║\033[0m       \033[1;97mPiSim // RASPBERRY PI 5 HARDWARE-IN-THE-LOOP TERMINAL DASHBOARD\033[0m            \033[96m║\033[0m")
        print("\033[96m╠══════════════════════════════════════════════════════════════════════════════════╣\033[0m")
        print(f"\033[96m║\033[0m  \033[1mLink Status\033[0m     : {status_badge:<48}  \033[96m║\033[0m")
        print(f"\033[96m║\033[0m  \033[1mTarget Host\033[0m     : \033[94m{self.target_host}:{self.control_port}\033[0m (Control TX) | Port \033[94m{self.telemetry_port}\033[0m (Telemetry RX)  \033[96m║\033[0m")
        print(f"\033[96m║\033[0m  \033[1mPacket Counters\033[0m : TX Commands: \033[92m{self.tx_count}\033[0m | RX Telemetry: \033[92m{self.rx_count}\033[0m (\033[93m{self.rx_rate_hz:4.1f} Hz\033[0m)            \033[96m║\033[0m")
        print("\033[96m╠══════════════════════════════════════════════════════════════════════════════════╣\033[0m")
        print("\033[96m║\033[0m  \033[1;92m🎮 CURRENT COMMAND OUTPUT (TX ➔ UE5)\033[0m                                            \033[96m║\033[0m")
        print(f"\033[96m║\033[0m     • Commanded Linear X  : \033[1;97m{self.cmd_linear_x:+6.2f} m/s\033[0m                                        \033[96m║\033[0m")
        print(f"\033[96m║\033[0m     • Commanded Angular Z : \033[1;97m{self.cmd_angular_z:+6.2f} rad/s\033[0m                                      \033[96m║\033[0m")
        print("\033[96m╠══════════════════════════════════════════════════════════════════════════════════╣\033[0m")
        print("\033[96m║\033[0m  \033[1;93m🏎️ LIVE TELEMETRY STREAM (RX ➔ Pi 5)\033[0m                                            \033[96m║\033[0m")
        print(f"\033[96m║\033[0m     • Linear Accel (m/s²) : X:\033[97m{self.accel[0]:+6.2f}\033[0m  Y:\033[97m{self.accel[1]:+6.2f}\033[0m  Z:\033[97m{self.accel[2]:+6.2f}\033[0m                   \033[96m║\033[0m")
        print(f"\033[96m║\033[0m     • Gyro Rate   (rad/s) : X:\033[97m{self.gyro[0]:+6.2f}\033[0m  Y:\033[97m{self.gyro[1]:+6.2f}\033[0m  Z:\033[97m{self.gyro[2]:+6.2f}\033[0m                   \033[96m║\033[0m")
        print(f"\033[96m║\033[0m     • Euler Angles (deg)  : Roll:\033[97m{self.euler[0]:+6.1f}°\033[0m Pitch:\033[97m{self.euler[1]:+6.1f}°\033[0m Yaw:\033[97m{self.euler[2]:+6.1f}°\033[0m             \033[96m║\033[0m")
        print("\033[96m╠══════════════════════════════════════════════════════════════════════════════════╣\033[0m")
        print("\033[96m║\033[0m  \033[1m⌨️  TELEOPERATION CONTROLS (DIRECT KEYPRESS - NO ENTER NEEDED)\033[0m                  \033[96m║\033[0m")
        print("\033[96m║\033[0m     [W] Forward (+1.5 m/s)   |   [S] Backward (-1.5 m/s)                         \033[96m║\033[0m")
        print("\033[96m║\033[0m     [A] Turn Left (+0.8 r/s) |   [D] Turn Right (-0.8 r/s)                       \033[96m║\033[0m")
        print("\033[96m║\033[0m     [SPACE / X] Stop (0 m/s) |   [Q] Exit Dashboard                              \033[96m║\033[0m")
        print("\033[96m╚══════════════════════════════════════════════════════════════════════════════════╝\033[0m")


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_UE5_HOST
    dashboard = PiSimLiveDashboard(target_host=target)
    dashboard.start()

    last_render = 0.0
    try:
        while dashboard.running:
            key = get_key_nonblocking()
            if key:
                if key == 'w':
                    dashboard.send_command(linear_x=1.5, angular_z=0.0)
                elif key == 's':
                    dashboard.send_command(linear_x=-1.5, angular_z=0.0)
                elif key == 'a':
                    dashboard.send_command(linear_x=0.0, angular_z=0.8)
                elif key == 'd':
                    dashboard.send_command(linear_x=0.0, angular_z=-0.8)
                elif key in [' ', 'x']:
                    dashboard.send_command(linear_x=0.0, angular_z=0.0)
                elif key == 'q':
                    dashboard.send_command(linear_x=0.0, angular_z=0.0)
                    break

            now = time.time()
            if now - last_render >= 0.1:  # Render at 10 FPS
                dashboard.render_dashboard()
                last_render = now

            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        dashboard.stop()
        print("\n[*] Dashboard closed gracefully.")


if __name__ == "__main__":
    main()
