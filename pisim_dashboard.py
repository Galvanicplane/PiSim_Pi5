#!/usr/bin/env python3
"""
pisim_dashboard.py - Interactive Live Terminal Telemetry & Control Dashboard
Universal Edge Bridge for Raspberry Pi 5, NVIDIA Jetson, Linux & Windows PCs.

Features:
- Continuous 20 Hz UDP Heartbeat & Teleoperation (Never times out)
- Live Bidirectional Connection Diagnostic Event Log (Shows exact send/receive events)
- Real-time IMU Telemetry (Orientation Quaternion, Gyro, Accel, Euler Roll/Pitch/Yaw)
- Round-Trip Ping & Network Health Monitoring
- Standalone '--test' Probe Mode for instant port and socket debugging
"""

import socket
import struct
import threading
import time
import sys
import os
import math
from collections import deque
from datetime import datetime

# Prevent Windows console cp1254 UnicodeEncodeError
try:
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    if hasattr(sys.stderr, 'reconfigure'):
        sys.stderr.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

# ==============================================================================
# DEFAULT NETWORK CONFIGURATION
# ==============================================================================
# Intelligent Host Detection: If on Raspberry Pi/Linux use 192.168.1.10 (PC Ethernet), else 127.0.0.1
DEFAULT_UE5_HOST = "192.168.1.10" if sys.platform.startswith("linux") else "127.0.0.1"
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
        self.send_thread = None

        # Control States
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0

        # Telemetry States
        self.rx_count = 0
        self.tx_count = 0
        self.last_rx_time = 0.0
        self.last_tx_time = 0.0
        self.rx_rate_hz = 0.0
        self.tx_rate_hz = 0.0
        self.latency_ms = 0.0

        self.quat = (0.0, 0.0, 0.0, 1.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.accel = (0.0, 0.0, 0.0)
        self.euler = (0.0, 0.0, 0.0)

        # Connection Event Logs (Max 6 entries)
        self.event_logs = deque(maxlen=6)
        self.add_log(f"Soketler hazir: TX -> {self.target_host}:{self.control_port} | RX: 0.0.0.0:{self.telemetry_port}")

    def add_log(self, message):
        """Adds a timestamped event log."""
        ts = datetime.now().strftime("%H:%M:%S")
        self.event_logs.append(f"[{ts}] {message}")

    def start(self):
        self.running = True

        # Start Telemetry Listener Thread
        self.recv_thread = threading.Thread(target=self._telemetry_listener, daemon=True)
        self.recv_thread.start()

        # Start Continuous 20 Hz Command & Heartbeat Sender Thread
        self.send_thread = threading.Thread(target=self._sender_loop, daemon=True)
        self.send_thread.start()

        self.add_log("Heartbeat ve telemetri dinleyici baslatildi.")

    def stop(self):
        self.running = False
        self.send_command_immediate(0.0, 0.0)
        if self.send_sock:
            self.send_sock.close()
        if self.recv_sock:
            self.recv_sock.close()
        self.add_log("Dashboard durduruldu.")

    def set_target_command(self, linear_x, angular_z, key_name=""):
        """Sets the active command that is continuously streamed by the sender loop."""
        self.cmd_linear_x = linear_x
        self.cmd_angular_z = angular_z
        if key_name:
            self.add_log(f"🎮 [TUS '{key_name}'] Hedef Hız: {linear_x:+.2f} m/s | Donus: {angular_z:+.2f} r/s")

    def send_command_immediate(self, linear_x, angular_z):
        """Packs and immediately sends geometry_msgs/Twist over UDP Port 7400."""
        data = struct.pack(TWIST_FORMAT, linear_x, 0.0, 0.0, 0.0, 0.0, angular_z)
        try:
            self.last_tx_time = time.time()
            self.send_sock.sendto(data, (self.target_host, self.control_port))
            self.tx_count += 1
        except Exception as e:
            self.add_log(f"❌ TX Hata: {e}")

    def _sender_loop(self):
        """Continuously streams command at 20 Hz (50ms interval) to keep connection alive."""
        last_calc = time.time()
        tx_window = 0

        while self.running:
            self.send_command_immediate(self.cmd_linear_x, self.cmd_angular_z)
            tx_window += 1

            now = time.time()
            dt = now - last_calc
            if dt >= 0.5:
                self.tx_rate_hz = tx_window / dt
                tx_window = 0
                last_calc = now

            time.sleep(0.05)  # 20 Hz loop

    def _telemetry_listener(self):
        """Receives live IMU telemetry from UE5 on UDP Port 7401."""
        last_calc_time = time.time()
        rx_window = 0
        first_packet = True

        while self.running:
            try:
                self.recv_sock.settimeout(0.2)
                data, addr = self.recv_sock.recvfrom(2048)
                if len(data) == IMU_SIZE:
                    now = time.time()
                    self.last_rx_time = now

                    # Measure RTT latency
                    if self.last_tx_time > 0:
                        self.latency_ms = max(0.5, (now - self.last_tx_time) * 1000.0)

                    unpacked = struct.unpack(IMU_FORMAT, data)
                    self.quat = unpacked[0:4]
                    self.gyro = unpacked[4:7]
                    self.accel = unpacked[7:10]
                    self.euler = quaternion_to_euler(*self.quat)

                    self.rx_count += 1
                    rx_window += 1

                    if first_packet:
                        first_packet = False
                        self.add_log(f"🟢 ILK PAKET ALINDI! {addr[0]}:{addr[1]} baglandi ({len(data)} byte)")

                now = time.time()
                dt = now - last_calc_time
                if dt >= 0.5:
                    self.rx_rate_hz = rx_window / dt
                    rx_window = 0
                    last_calc_time = now
            except socket.timeout:
                pass
            except Exception as e:
                if not self.running:
                    break

    def render_dashboard(self):
        """Draws the rich ANSI terminal dashboard."""
        os.system('cls' if os.name == 'nt' else 'clear')

        time_since_rx = time.time() - self.last_rx_time if self.last_rx_time > 0 else 999.0
        is_linked = time_since_rx < 2.0

        stage1 = f"\033[92m🟢 ACIK (0.0.0.0:{self.telemetry_port} Dinleniyor)\033[0m"
        stage2 = f"\033[92m🟢 HAZIR ({self.target_host}:{self.control_port})\033[0m"
        stage3 = f"\033[92m🟢 ALINDI (Gecikme: {self.latency_ms:4.1f} ms)\033[0m" if is_linked else f"\033[93m🟡 BEKLENIYOR... (Hedef: {self.target_host})\033[0m"
        stage4 = f"\033[92m🟢 AKTIF ({self.tx_rate_hz:4.1f} Hz TX | {self.rx_rate_hz:4.1f} Hz RX)\033[0m" if is_linked else "\033[93m🟡 VERI AKISI BEKLEMEDE\033[0m"

        print("\033[96m╔══════════════════════════════════════════════════════════════════════════════════╗\033[0m")
        print("\033[96m║\033[0m       \033[1;97mPiSim // RASPBERRY PI 5 HARDWARE-IN-THE-LOOP TERMINAL DASHBOARD\033[0m            \033[96m║\033[0m")
        print("\033[96m╠══════════════════════════════════════════════════════════════════════════════════╣\033[0m")
        print(f"\033[96m║\033[0m  \033[1;96m🔗 PI 5 ➔ UE5 BAGLANTI ASAMALARI (1 - 4)\033[0m                                        \033[96m║\033[0m")
        print(f"\033[96m║\033[0m     • Asama 1: Pi 5 Dinleme Soketi : {stage1:<52} \033[96m║\033[0m")
        print(f"\033[96m║\033[0m     • Asama 2: Hedef UE5 Host      : {stage2:<52} \033[96m║\033[0m")
        print(f"\033[96m║\033[0m     • Asama 3: UE5 Telemetri Yaniti: {stage3:<52} \033[96m║\033[0m")
        print(f"\033[96m║\033[0m     • Asama 4: Canli Cift Yon Akis : {stage4:<52} \033[96m║\033[0m")
        print("\033[96m╠══════════════════════════════════════════════════════════════════════════════════╣\033[0m")
        print("\033[96m║\033[0m  \033[1;92m🎮 ANLIK AKTÜATÖR KOMUTLARI (Pi 5 ➔ UE5 Port 7400)\033[0m                              \033[96m║\033[0m")
        print(f"\033[96m║\033[0m     • Iletilen Linear X   : \033[1;97m{self.cmd_linear_x:+6.2f} m/s\033[0m  (Toplam TX: {self.tx_count} Paket)             \033[96m║\033[0m")
        print(f"\033[96m║\033[0m     • Iletilen Angular Z  : \033[1;97m{self.cmd_angular_z:+6.2f} rad/s\033[0m                                      \033[96m║\033[0m")
        print("\033[96m╠══════════════════════════════════════════════════════════════════════════════════╣\033[0m")
        print("\033[96m║\033[0m  \033[1;93m🏎️ CANLI IMU & KINEMATIK TELEMETRISI (UE5 ➔ Pi 5 Port 7401)\033[0m                    \033[96m║\033[0m")
        print(f"\033[96m║\033[0m     • Paket Sayaci (RX)   : \033[1;97m{self.rx_count}\033[0m Paket ({self.rx_rate_hz:4.1f} Hz) - 80 Bayt CDR                   \033[96m║\033[0m")
        print(f"\033[96m║\033[0m     • Lineer Ivme (m/s²)  : X:\033[97m{self.accel[0]:+6.2f}\033[0m  Y:\033[97m{self.accel[1]:+6.2f}\033[0m  Z:\033[97m{self.accel[2]:+6.2f}\033[0m                   \033[96m║\033[0m")
        print(f"\033[96m║\033[0m     • Jiroskop    (rad/s) : X:\033[97m{self.gyro[0]:+6.2f}\033[0m  Y:\033[97m{self.gyro[1]:+6.2f}\033[0m  Z:\033[97m{self.gyro[2]:+6.2f}\033[0m                   \033[96m║\033[0m")
        print(f"\033[96m║\033[0m     • Euler Acilari (deg) : Roll:\033[97m{self.euler[0]:+6.1f}°\033[0m Pitch:\033[97m{self.euler[1]:+6.1f}°\033[0m Yaw:\033[97m{self.euler[2]:+6.1f}°\033[0m             \033[96m║\033[0m")
        print("\033[96m╠══════════════════════════════════════════════════════════════════════════════════╣\033[0m")
        print("\033[96m║\033[0m  \033[1;95m📋 CANLI BAGLANTI VE OLAY GÜNLÜĞÜ (DEBUG LOG)\033[0m                                   \033[96m║\033[0m")
        for log in list(self.event_logs)[-5:]:
            print(f"\033[96m║\033[0m   {log:<78} \033[96m║\033[0m")
        for _ in range(5 - len(self.event_logs)):
            print(f"\033[96m║\033[0m   {'':<78} \033[96m║\033[0m")
        print("\033[96m╠══════════════════════════════════════════════════════════════════════════════════╣\033[0m")
        print("\033[96m║\033[0m  \033[1m⌨️  TELEOPERASYON TUSLARI (DIREKT BAS - ENTER GEREKMEZ)\033[0m                         \033[96m║\033[0m")
        print("\033[96m║\033[0m     [W] Ileri (+1.5 m/s)     |   [S] Geri (-1.5 m/s)                             \033[96m║\033[0m")
        print("\033[96m║\033[0m     [A] Sola Don (+0.8 r/s)  |   [D] Saga Don (-0.8 r/s)                         \033[96m║\033[0m")
        print("\033[96m║\033[0m     [SPACE / X] Acil Durdur  |   [Q] Cikis Yap                                   \033[96m║\033[0m")
        print("\033[96m╚══════════════════════════════════════════════════════════════════════════════════╝\033[0m")


def run_diagnostic_test(target_host=DEFAULT_UE5_HOST):
    """Probes the network connection with raw logs without clearing screen."""
    print(f"\n========================================================")
    print(f"  🔍 PiSim UDP Teşhis Probu (Diagnostic Probe)")
    print(f"========================================================")
    print(f"  [+] Hedef UE5 PC       : {target_host}:{CONTROL_PORT}")
    print(f"  [+] Dinlenen Port (RX) : 0.0.0.0:{TELEMETRY_PORT}")
    print(f"========================================================\n")

    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    recv_sock.bind(("0.0.0.0", TELEMETRY_PORT))
    recv_sock.settimeout(1.0)

    for i in range(1, 6):
        data = struct.pack(TWIST_FORMAT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        send_sock.sendto(data, (target_host, CONTROL_PORT))
        print(f"[{i}/5] 📤 Sinyal gönderildi -> {target_host}:{CONTROL_PORT} (Boyut: {len(data)} byte)")

        try:
            resp, addr = recv_sock.recvfrom(2048)
            print(f"       ✅ CEVAP ALINDI! <- {addr[0]}:{addr[1]} ({len(resp)} byte IMU paketi)")
        except socket.timeout:
            print(f"       ⏳ Yanıt yok (Zaman aşımı)... UE5'te 'Play' açık mı?")

        time.sleep(0.5)

    send_sock.close()
    recv_sock.close()
    print("\n[*] Teşhis tamamlandı.\n")


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        target = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_UE5_HOST
        run_diagnostic_test(target_host=target)
        return

    target = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_UE5_HOST
    dashboard = PiSimLiveDashboard(target_host=target)
    dashboard.start()

    last_render = 0.0
    try:
        while dashboard.running:
            key = get_key_nonblocking()
            if key:
                if key == 'w':
                    dashboard.set_target_command(linear_x=1.5, angular_z=0.0, key_name='W')
                elif key == 's':
                    dashboard.set_target_command(linear_x=-1.5, angular_z=0.0, key_name='S')
                elif key == 'a':
                    dashboard.set_target_command(linear_x=0.0, angular_z=0.8, key_name='A')
                elif key == 'd':
                    dashboard.set_target_command(linear_x=0.0, angular_z=-0.8, key_name='D')
                elif key in [' ', 'x']:
                    dashboard.set_target_command(linear_x=0.0, angular_z=0.0, key_name='SPACE')
                elif key == 'q':
                    dashboard.set_target_command(linear_x=0.0, angular_z=0.0, key_name='Q')
                    break

            now = time.time()
            if now - last_render >= 0.1:  # 10 FPS screen refresh
                dashboard.render_dashboard()
                last_render = now

            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        dashboard.stop()
        print("\n[*] Dashboard sonlandirildi.")


if __name__ == "__main__":
    main()
