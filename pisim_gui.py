#!/usr/bin/env python3
"""
pisim_gui.py - Standalone GUI Cockpit & Ground Station for Raspberry Pi 5
Opens a dedicated high-tech graphical application window displaying:
- Live FPV Camera Stream (UDP Port 5000, MTU Chunk Reassembly) with Fighter Jet HUD / Artificial Horizon
- Real-Time IMU Sensor Telemetry (UDP Port 7401 - Pitch, Roll, Yaw, Gyro, Accel)
- Motor Actuation RPMs & Vehicle Forward Speed
- Interactive WASD Teleoperation Control Pad (UDP Port 7400 - geometry_msgs/Twist)
- 4-Stage Live Connection Diagnostics

Run on Raspberry Pi 5 (or PC for testing):
    python pisim_gui.py [TARGET_UE5_IP]
Example:
    python pisim_gui.py 192.168.1.10
"""

import socket
import struct
import threading
import time
import sys
import math
import cv2
import numpy as np

# Prevent Windows console encoding issues
try:
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

# Network Configuration
UE5_CONTROL_PORT = 7400       # Port UE5 listens for /cmd_vel
PI5_TELEMETRY_PORT = 7401     # Port Pi5 listens for /sim/imu
PI5_VIDEO_PORT = 5000         # Port Pi5 listens for FPV video chunks

# Intelligent Default Host Detection
DEFAULT_TARGET_HOST = "192.168.1.10" if sys.platform.startswith("linux") else "127.0.0.1"

# Binary Struct Formats (matches UE5 C++ #pragma pack(push, 1))
TWIST_FORMAT = "<6d"          # geometry_msgs/msg/Twist (48 bytes: linear.xyz, angular.xyz)
TWIST_SIZE = struct.calcsize(TWIST_FORMAT)

IMU_FORMAT = "<10d"           # sensor_msgs/msg/Imu (80 bytes: quat.xyzw, gyro.xyz, accel.xyz)
IMU_SIZE = struct.calcsize(IMU_FORMAT)


def quat_to_euler(x, y, z, w):
    """Converts quaternion (x, y, z, w) to Euler angles in degrees (roll, pitch, yaw)."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class PiSimCockpitGUI:
    def __init__(self, target_host=DEFAULT_TARGET_HOST):
        self.target_host = target_host
        self.running = True

        # Telemetry State
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.imu_yaw = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.quat = (0.0, 0.0, 0.0, 1.0)
        self.telemetry_packet_count = 0
        self.telemetry_hz = 0.0
        self.last_telemetry_time = 0.0
        self.is_connected = False

        # Video State
        self.latest_frame = None
        self.video_frame_count = 0
        self.video_fps = 0.0
        self.last_video_time = 0.0
        self.video_chunks = {}
        self.current_frame_seq = None

        # Teleop State (WASD)
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.active_keys = set()
        self.left_rpm = 0.0
        self.right_rpm = 0.0
        self.speed_kmh = 0.0

        # Sockets
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.telemetry_sock.bind(("0.0.0.0", PI5_TELEMETRY_PORT))

        self.video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.video_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.video_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
        self.video_sock.bind(("0.0.0.0", PI5_VIDEO_PORT))

        # Start Background Threads
        self.telemetry_thread = threading.Thread(target=self._telemetry_worker, daemon=True)
        self.video_thread = threading.Thread(target=self._video_worker, daemon=True)
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_worker, daemon=True)

        self.telemetry_thread.start()
        self.video_thread.start()
        self.heartbeat_thread.start()

        # Send immediate handshake ping to UE5
        self.send_twist(0.0, 0.0)

    def send_twist(self, linear_x, angular_z):
        """Transmits ROS2 geometry_msgs/msg/Twist packet to UE5 on Port 7400."""
        self.target_linear_x = linear_x
        self.target_angular_z = angular_z

        # Differential kinematics estimation for display
        track_width = 0.35
        wheel_radius = 0.08
        v_l = linear_x - (angular_z * track_width * 0.5)
        v_r = linear_x + (angular_z * track_width * 0.5)
        self.left_rpm = (v_l / (2.0 * math.pi * wheel_radius)) * 60.0
        self.right_rpm = (v_r / (2.0 * math.pi * wheel_radius)) * 60.0
        self.speed_kmh = abs(linear_x) * 3.6

        payload = struct.pack(TWIST_FORMAT, linear_x, 0.0, 0.0, 0.0, 0.0, angular_z)
        try:
            self.send_sock.sendto(payload, (self.target_host, UE5_CONTROL_PORT))
        except Exception:
            pass

    def _heartbeat_worker(self):
        """Sends periodic cmd_vel to keep UE5 connection active and prevent 3-second timeout."""
        while self.running:
            self.send_twist(self.target_linear_x, self.target_angular_z)
            time.sleep(0.05)  # 20 Hz

    def _telemetry_worker(self):
        """Receives ROS2 sensor_msgs/msg/Imu telemetry packets on UDP 7401."""
        pkt_window = 0
        window_start = time.time()

        while self.running:
            try:
                data, addr = self.telemetry_sock.recvfrom(2048)
                if len(data) == IMU_SIZE:
                    unpacked = struct.unpack(IMU_FORMAT, data)
                    qx, qy, qz, qw = unpacked[0:4]
                    gx, gy, gz = unpacked[4:7]
                    ax, ay, az = unpacked[7:10]

                    self.quat = (qx, qy, qz, qw)
                    self.gyro_x, self.gyro_y, self.gyro_z = gx, gy, gz
                    self.accel_x, self.accel_y, self.accel_z = ax, ay, az

                    # Convert to Euler
                    self.imu_roll, self.imu_pitch, self.imu_yaw = quat_to_euler(qx, qy, qz, qw)

                    self.telemetry_packet_count += 1
                    self.last_telemetry_time = time.time()
                    self.is_connected = True

                    pkt_window += 1
                    now = time.time()
                    if now - window_start >= 0.5:
                        self.telemetry_hz = pkt_window / (now - window_start)
                        pkt_window = 0
                        window_start = now
            except Exception:
                if not self.running:
                    break

    def _video_worker(self):
        """Reassembles MTU-safe JPEG video chunks on UDP Port 5000."""
        frame_window = 0
        window_start = time.time()

        while self.running:
            try:
                data, addr = self.video_sock.recvfrom(65507)
                if len(data) < 4:
                    continue

                frame_seq = (data[0] << 8) | data[1]
                chunk_idx = data[2]
                total_chunks = data[3]
                payload = data[4:]

                if frame_seq != self.current_frame_seq:
                    self.current_frame_seq = frame_seq
                    self.video_chunks = {}

                self.video_chunks[chunk_idx] = payload

                if len(self.video_chunks) == total_chunks:
                    full_jpeg = b"".join(self.video_chunks[i] for i in range(total_chunks) if i in self.video_chunks)
                    np_arr = np.frombuffer(full_jpeg, np.uint8)
                    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                    if frame is not None:
                        self.latest_frame = frame
                        self.video_frame_count += 1
                        self.last_video_time = time.time()

                        frame_window += 1
                        now = time.time()
                        if now - window_start >= 1.0:
                            self.video_fps = frame_window / (now - window_start)
                            frame_window = 0
                            window_start = now

                    self.video_chunks = {}
            except Exception:
                if not self.running:
                    break

    def _draw_hud_overlay(self, img):
        """Draws a fighter jet / SpaceX style OSD HUD directly onto the camera frame."""
        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2

        # 1) Center Reticle Crosshair
        reticle_col = (0, 255, 200)  # Neon Cyan
        cv2.drawMarker(img, (cx, cy), reticle_col, markerType=cv2.MARKER_CROSS, markerSize=18, thickness=1)
        cv2.circle(img, (cx, cy), 28, reticle_col, 1)

        # 2) Artificial Horizon Line (Roll & Pitch)
        # Roll tilts the line; Pitch moves the line vertically
        roll_rad = math.radians(-self.imu_roll)
        pitch_offset = int(self.imu_pitch * 2.5)  # Pixels per degree of pitch

        horizon_cy = cy + pitch_offset
        horizon_cy = max(30, min(h - 30, horizon_cy))

        line_len = 110
        dx = int(line_len * math.cos(roll_rad))
        dy = int(line_len * math.sin(roll_rad))

        pt1 = (cx - dx, horizon_cy - dy)
        pt2 = (cx + dx, horizon_cy + dy)
        cv2.line(img, pt1, pt2, (0, 240, 255), 2)  # Glowing Horizon Line
        # Pitch ticks
        cv2.line(img, (cx - dx // 2, horizon_cy - dy // 2), (cx - dx // 2, horizon_cy - dy // 2 - 8), (0, 240, 255), 1)
        cv2.line(img, (cx + dx // 2, horizon_cy + dy // 2), (cx + dx // 2, horizon_cy + dy // 2 - 8), (0, 240, 255), 1)

        # 3) Heading Compass Bar at Top
        cv2.rectangle(img, (cx - 100, 10), (cx + 100, 36), (20, 20, 20), -1)
        cv2.rectangle(img, (cx - 100, 10), (cx + 100, 36), (0, 220, 255), 1)
        heading_deg = (self.imu_yaw) % 360.0
        heading_text = f"HDG: {heading_deg:05.1f}°"
        cv2.putText(img, heading_text, (cx - 58, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1, cv2.LINE_AA)

        # 4) Speed Tape on Left
        cv2.rectangle(img, (12, cy - 45), (82, cy + 45), (20, 20, 20), -1)
        cv2.rectangle(img, (12, cy - 45), (82, cy + 45), (0, 255, 120), 1)
        cv2.putText(img, "SPD", (28, cy - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 180, 180), 1)
        cv2.putText(img, f"{self.speed_kmh:4.1f}", (20, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 255, 120), 2)
        cv2.putText(img, "km/h", (26, cy + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 180, 180), 1)

        # 5) Vertical Accel Tape on Right
        cv2.rectangle(img, (w - 82, cy - 45), (w - 12, cy + 45), (20, 20, 20), -1)
        cv2.rectangle(img, (w - 82, cy - 45), (w - 12, cy + 45), (255, 180, 0), 1)
        cv2.putText(img, "ACC-Z", (w - 74, cy - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1)
        cv2.putText(img, f"{self.accel_z:+5.1f}", (w - 78, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 200, 50), 2)
        cv2.putText(img, "m/s²", (w - 68, cy + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1)

        # 6) Bottom FPV Status Bar
        status_txt = f"FPV LIVE | {w}x{h} @ {self.video_fps:4.1f} FPS | PKTS: {self.video_frame_count}"
        cv2.putText(img, status_txt, (18, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 180), 1, cv2.LINE_AA)

        return img

    def _render_no_signal_frame(self, w=640, h=480):
        """Renders an animated radar standby pattern when no video stream is active."""
        frame = np.full((h, w, 3), 18, dtype=np.uint8)
        cx, cy = w // 2, h // 2

        # Concentric Radar Rings
        for r in [60, 120, 180, 240]:
            cv2.circle(frame, (cx, cy), r, (35, 45, 35), 1)

        # Sweeping Radar Line
        angle = (time.time() * 2.5) % (2 * math.pi)
        sweep_x = int(cx + 240 * math.cos(angle))
        sweep_y = int(cy + 240 * math.sin(angle))
        cv2.line(frame, (cx, cy), (sweep_x, sweep_y), (0, 160, 60), 2)

        # Crosshair axes
        cv2.line(frame, (cx - 250, cy), (cx + 250, cy), (30, 40, 30), 1)
        cv2.line(frame, (cx, cy - 250), (cx, cy + 250), (30, 40, 30), 1)

        # Status text
        cv2.putText(frame, "PISIM FPV STREAM STANDBY", (cx - 165, cy - 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.70, (0, 230, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, f"Waiting for UE5 Port 5000 UDP Chunks...", (cx - 170, cy + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (160, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(frame, f"Target Host: {self.target_host}", (cx - 100, cy + 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 120), 1, cv2.LINE_AA)

        return frame

    def _draw_side_panel(self, canvas, start_x=640, width=384, height=640):
        """Draws the rich telemetry, motor gauges, connection stages, and WASD pad on the right."""
        # Dark tech background
        cv2.rectangle(canvas, (start_x, 0), (start_x + width, height), (15, 18, 24), -1)
        cv2.line(canvas, (start_x, 0), (start_x, height), (40, 60, 90), 2)

        x0 = start_x + 16
        y = 28

        # --- HEADER ---
        cv2.putText(canvas, "PISIM PI 5 GROUND STATION", (x0, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 220, 255), 2, cv2.LINE_AA)
        y += 24
        cv2.line(canvas, (x0, y), (start_x + width - 16, y), (0, 160, 200), 1)
        y += 18

        # --- CONNECTION BADGE ---
        now = time.time()
        is_active = (now - self.last_telemetry_time < 2.5) and self.telemetry_packet_count > 0

        if is_active:
            badge_bg = (20, 80, 20)
            badge_border = (0, 255, 100)
            badge_txt = f"STAGE 4: CONNECTED & STREAMING"
            txt_col = (100, 255, 120)
        else:
            badge_bg = (25, 45, 75)
            badge_border = (0, 180, 255)
            badge_txt = f"STAGE 3: WAITING FOR HANDSHAKE"
            txt_col = (0, 200, 255)

        cv2.rectangle(canvas, (x0, y), (x0 + width - 32, y + 26), badge_bg, -1)
        cv2.rectangle(canvas, (x0, y), (x0 + width - 32, y + 26), badge_border, 1)
        cv2.putText(canvas, badge_txt, (x0 + 8, y + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, txt_col, 1, cv2.LINE_AA)
        y += 38

        # --- NETWORK SPECS ---
        cv2.putText(canvas, f"Target UE5 IP : {self.target_host}:{UE5_CONTROL_PORT}", (x0, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 200, 220), 1)
        y += 18
        cv2.putText(canvas, f"Telemetry RX  : {self.telemetry_hz:4.1f} Hz (UDP {PI5_TELEMETRY_PORT})", (x0, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 200, 220), 1)
        y += 18
        cv2.putText(canvas, f"Video Stream  : {self.video_fps:4.1f} FPS (UDP {PI5_VIDEO_PORT})", (x0, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 200, 220), 1)
        y += 24
        cv2.line(canvas, (x0, y), (start_x + width - 16, y), (35, 45, 60), 1)
        y += 18

        # --- SECTION: SENSOR TELEMETRY (ROS2 /sim/imu) ---
        cv2.putText(canvas, "IMU SENSOR TELEMETRY (sensor_msgs/Imu)", (x0, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 200), 1, cv2.LINE_AA)
        y += 20

        # Orientation Roll / Pitch / Yaw
        cv2.putText(canvas, f"Roll  : {self.imu_roll:+6.1f}°   Pitch: {self.imu_pitch:+6.1f}°", (x0 + 8, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        y += 18
        cv2.putText(canvas, f"Yaw   : {self.imu_yaw:+6.1f}° (Heading)", (x0 + 8, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        y += 20

        # Accel & Gyro
        cv2.putText(canvas, f"Accel : [{self.accel_x:+5.2f}, {self.accel_y:+5.2f}, {self.accel_z:+5.2f}] m/s²", (x0 + 8, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 220, 255), 1)
        y += 18
        cv2.putText(canvas, f"Gyro  : [{self.gyro_x:+5.2f}, {self.gyro_y:+5.2f}, {self.gyro_z:+5.2f}] deg/s", (x0 + 8, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 220, 255), 1)
        y += 24
        cv2.line(canvas, (x0, y), (start_x + width - 16, y), (35, 45, 60), 1)
        y += 18

        # --- SECTION: ACTUATORS & WHEEL MOTORS ---
        cv2.putText(canvas, "ACTUATOR DIFFERENTIAL DRIVE", (x0, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 190, 0), 1, cv2.LINE_AA)
        y += 20
        cv2.putText(canvas, f"Forward Speed : {self.speed_kmh:4.1f} km/h", (x0 + 8, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        y += 18
        cv2.putText(canvas, f"Left Wheels   : {self.left_rpm:+6.1f} RPM", (x0 + 8, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 120), 1)
        y += 18
        cv2.putText(canvas, f"Right Wheels  : {self.right_rpm:+6.1f} RPM", (x0 + 8, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 120), 1)
        y += 24
        cv2.line(canvas, (x0, y), (start_x + width - 16, y), (35, 45, 60), 1)
        y += 18

        # --- SECTION: WASD TELEOPERATION PAD ---
        cv2.putText(canvas, "WASD TELEOPERATION (geometry_msgs/Twist)", (x0, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 220, 255), 1, cv2.LINE_AA)
        y += 16

        pad_cx = start_x + width // 2
        key_size = 32
        
        # Key coordinates
        key_w = (pad_cx - key_size // 2, y, key_size, key_size, "W", 'w' in self.active_keys)
        y += key_size + 4
        key_a = (pad_cx - key_size * 2, y, key_size, key_size, "A", 'a' in self.active_keys)
        key_s = (pad_cx - key_size // 2, y, key_size, key_size, "S", 's' in self.active_keys)
        key_d = (pad_cx + key_size, y, key_size, key_size, "D", 'd' in self.active_keys)
        y += key_size + 6
        key_space = (pad_cx - key_size * 2, y, key_size * 4, 22, "SPACE (BRAKE)", ' ' in self.active_keys)
        y += 32

        # Draw WASD Keys
        for kx, ky, kw, kh, label, pressed in [key_w, key_a, key_s, key_d, key_space]:
            bg = (0, 160, 80) if pressed else (30, 36, 48)
            border = (0, 255, 120) if pressed else (80, 100, 130)
            txt_c = (255, 255, 255) if pressed else (180, 190, 210)
            cv2.rectangle(canvas, (kx, ky), (kx + kw, ky + kh), bg, -1)
            cv2.rectangle(canvas, (kx, ky), (kx + kw, ky + kh), border, 1)
            
            # Center label inside box
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.40, 1)
            tx = kx + (kw - tw) // 2
            ty = ky + (kh + th) // 2
            cv2.putText(canvas, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.40, txt_c, 1, cv2.LINE_AA)

        # Current Published Values
        cv2.putText(canvas, f"Linear X : {self.target_linear_x:+4.2f} m/s | Angular Z: {self.target_angular_z:+4.2f} rad/s",
                    (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 200), 1)
        y += 24

        # Footer Hotkeys
        cv2.putText(canvas, "Controls: WASD=Drive | SPACE=Brake | Q/ESC=Quit", (x0, height - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, (120, 140, 160), 1)

    def run(self):
        """Main application GUI loop: opens persistent window and processes events."""
        window_name = "PiSim Ground Station - FPV Cockpit & Robot Teleop"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1024, 640)

        print("\n========================================================")
        print("  PiSim Ground Station GUI Started Successfully!        ")
        print("========================================================")
        print(f"  Target UE5 Host : {self.target_host}:{UE5_CONTROL_PORT}")
        print(f"  Telemetry Port  : UDP {PI5_TELEMETRY_PORT}")
        print(f"  FPV Video Port  : UDP {PI5_VIDEO_PORT}")
        print("  Focus the graphical window to drive with WASD keys.")
        print("  Press 'q' or ESC in the window to quit.")
        print("========================================================\n")

        canvas = np.zeros((640, 1024, 3), dtype=np.uint8)

        try:
            while self.running:
                # 1) Prepare Left 640x480 (or 640x640) Area for Video
                if self.latest_frame is not None and (time.time() - self.last_video_time < 3.0):
                    # Resize incoming frame to 640x480
                    video_disp = cv2.resize(self.latest_frame, (640, 480), interpolation=cv2.INTER_LINEAR)
                    video_disp = self._draw_hud_overlay(video_disp)
                else:
                    video_disp = self._render_no_signal_frame(640, 480)

                # Place video in top-left
                canvas[0:480, 0:640] = video_disp

                # Bottom left panel: ROS2 Topic Inspector & Status
                cv2.rectangle(canvas, (0, 480), (640, 640), (10, 12, 18), -1)
                cv2.line(canvas, (0, 480), (640, 480), (35, 50, 70), 2)

                cv2.putText(canvas, "ROS2 PROTOCOL TOPIC BINDINGS (Proven DDS Standard)", (16, 504),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 220, 255), 1, cv2.LINE_AA)
                cv2.putText(canvas, "• /cmd_vel     [TX -> UE5 7400] : geometry_msgs/msg/Twist  (6x float64, 48 Bytes)", (20, 528),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 220, 240), 1)
                cv2.putText(canvas, "• /sim/imu     [RX <- UE5 7401] : sensor_msgs/msg/Imu      (10x float64, 80 Bytes)", (20, 550),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 220, 240), 1)
                cv2.putText(canvas, "• /camera/raw  [RX <- UE5 5000] : sensor_msgs/CompressedImage (JPEG MTU Chunks)", (20, 572),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 220, 240), 1)
                cv2.putText(canvas, "• /gps/fix     [Future Support] : sensor_msgs/msg/NavSatFix  (Lat, Lon, Alt)", (20, 594),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (140, 160, 180), 1)
                cv2.putText(canvas, "Status: High-Performance C++ Rendering Engine Connected", (20, 622),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 120), 1)

                # 2) Draw Right Side Telemetry & WASD Panel (384x640)
                self._draw_side_panel(canvas, start_x=640, width=384, height=640)

                # 3) Show Canvas Window
                cv2.imshow(window_name, canvas)

                # 4) Process Interactive Keyboard Input
                key = cv2.waitKey(20) & 0xFF
                if key == ord('q') or key == 27:
                    print("[*] Exit requested by user.")
                    break
                elif key == ord('w'):
                    self.active_keys = {'w'}
                    self.send_twist(1.2, 0.0)
                elif key == ord('s'):
                    self.active_keys = {'s'}
                    self.send_twist(-1.0, 0.0)
                elif key == ord('a'):
                    self.active_keys = {'a'}
                    self.send_twist(0.0, -1.2)
                elif key == ord('d'):
                    self.active_keys = {'d'}
                    self.send_twist(0.0, 1.2)
                elif key == 32:  # Space bar (Brake)
                    self.active_keys = {' '}
                    self.send_twist(0.0, 0.0)
                elif key == 255:  # No key pressed in this cycle
                    if self.active_keys:
                        # Gradual decay / stop
                        self.active_keys.clear()
                        self.send_twist(0.0, 0.0)

        except KeyboardInterrupt:
            print("\n[*] Application stopped by Ctrl+C.")
        finally:
            self.running = False
            self.send_sock.close()
            self.telemetry_sock.close()
            self.video_sock.close()
            cv2.destroyAllWindows()
            print("[*] PiSim Ground Station GUI successfully closed.")


if __name__ == "__main__":
    target_ip = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_TARGET_HOST
    cockpit = PiSimCockpitGUI(target_host=target_ip)
    cockpit.run()
