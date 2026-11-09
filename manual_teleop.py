#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
manual_teleop.py - PiSim Canlı Manuel Klavye Teleoperasyon Sürücüsü
===================================================================
Raspberry Pi 5 terminalinden, Windows'tan veya SSH üzerinden Unreal Engine'deki
herhangi bir aracı gerçek zamanlı olarak klavye ile manuel sürmenizi sağlar.

Kontroller:
  [ W ] veya [ ↑ ] : Gaz Artır (İleri Git)
  [ S ] veya [ ↓ ] : Gaz Azalt / Fren / Geri Git
  [ A ] veya [ ← ] : Sola Direksiyon Kır
  [ D ] veya [ → ] : Sağa Direksiyon Kır
  [ X ]            : Direksiyonu Merkeze Sıfırla
  [ SPACE / BOŞLUK]: ACİL DURDURMA (Gaz = 0, Direksiyon = 0)
  [ Q ] veya [ESC] : Çıkış

Kullanım:
  python3 manual_teleop.py --vehicle RV01-8AD131
  python3 manual_teleop.py --vehicle auto
"""

import argparse
import json
import math
import os
import sys
import threading
import time
from typing import Dict, Optional

# Windows terminal Türkçe kod sayfası desteği
if sys.stdout and hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

try:
    import zenoh
    HAS_ZENOH = True
except ImportError:
    HAS_ZENOH = False

from agent_supervisor import auto_discover_zenoh_endpoint, normalize_zenoh_endpoint
from vehicle_hardware import VehicleHardware


class KeyboardReader:
    """Windows ve Linux (Pi 5) için engellemesiz (non-blocking) tek tuş okuyucu."""

    def __init__(self):
        self.is_windows = (os.name == 'nt')
        if not self.is_windows:
            import termios
            import tty
            self.termios = termios
            self.tty = tty
            self.old_settings = None

    def __enter__(self):
        if not self.is_windows:
            try:
                self.old_settings = self.termios.tcgetattr(sys.stdin)
                self.tty.setcbreak(sys.stdin.fileno())
            except Exception:
                pass
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if not self.is_windows and self.old_settings:
            try:
                self.termios.tcsetattr(sys.stdin, self.termios.TCSADRAIN, self.old_settings)
            except Exception:
                pass

    def get_key(self) -> Optional[str]:
        if self.is_windows:
            import msvcrt
            if msvcrt.kbhit():
                ch = msvcrt.getch()
                if ch in (b'\x00', b'\xe0'):  # Özel ok tuşları
                    ch2 = msvcrt.getch()
                    if ch2 == b'H': return 'UP'
                    if ch2 == b'P': return 'DOWN'
                    if ch2 == b'K': return 'LEFT'
                    if ch2 == b'M': return 'RIGHT'
                try:
                    return ch.decode('utf-8', errors='ignore').lower()
                except Exception:
                    return None
            return None
        else:
            import select
            dr, _, _ = select.select([sys.stdin], [], [], 0.05)
            if dr:
                ch = sys.stdin.read(1)
                if ch == '\x1b':  # ANSI Kaçış Dizisi (Ok tuşları)
                    dr2, _, _ = select.select([sys.stdin], [], [], 0.02)
                    if dr2:
                        ch2 = sys.stdin.read(1)
                        if ch2 == '[':
                            ch3 = sys.stdin.read(1)
                            if ch3 == 'A': return 'UP'
                            if ch3 == 'B': return 'DOWN'
                            if ch3 == 'D': return 'LEFT'
                            if ch3 == 'C': return 'RIGHT'
                    return 'ESC'
                return ch.lower()
            return None


class ManualTeleopController:
    def __init__(self, vehicle_code: str, router_endpoint: Optional[str] = None):
        self.vehicle_code = vehicle_code
        self.router_endpoint = router_endpoint
        self.running = False
        self.session = None

        self.throttle = 0.0      # -1.0 .. +1.0
        self.steering = 0.0      # -1.0 .. +1.0 (-1: Sol, +1: Sağ)
        self.throttle_step = 0.15
        self.steering_step = 0.20

        self.hardware = VehicleHardware(vehicle_code=self.vehicle_code, drive_mode="ACKERMANN")

        # Canlı Telemetri Geri Bildirimi
        self.last_lat = 0.0
        self.last_lon = 0.0
        self.last_yaw = 0.0
        self.last_speed = 0.0
        self.telemetry_rx_count = 0
        self.discovered_vehicles = set()

    def _init_zenoh(self) -> bool:
        if not HAS_ZENOH:
            print("❌ 'eclipse-zenoh' modülü bulunamadı! 'pip install eclipse-zenoh' çalıştırın.")
            return False

        if not self.router_endpoint:
            self.router_endpoint = auto_discover_zenoh_endpoint()

        conf = zenoh.Config()
        norm_ep = normalize_zenoh_endpoint(self.router_endpoint)
        if norm_ep:
            conf.insert_json5("mode", json.dumps("client"))
            conf.insert_json5("connect/endpoints", json.dumps([norm_ep]))
            self.router_endpoint = norm_ep
        else:
            conf.insert_json5("mode", json.dumps("peer"))

        try:
            self.session = zenoh.open(conf)
            return True
        except Exception as e:
            print(f"❌ Zenoh oturumu açılamadı: {e}")
            return False

    def _on_gps(self, sample):
        try:
            parts = str(sample.key_expr).split("/")
            if len(parts) >= 2:
                v = parts[1]
                self.discovered_vehicles.add(v)
                if self.vehicle_code.lower() == "auto" and not self.running:
                    self.vehicle_code = v
                if self.vehicle_code == v:
                    data = json.loads(sample.payload.to_string())
                    self.last_lat = float(data.get("latitude", 0.0))
                    self.last_lon = float(data.get("longitude", 0.0))
                    self.last_speed = float(data.get("speed", 0.0))
                    self.telemetry_rx_count += 1
        except Exception:
            pass

    def _on_imu(self, sample):
        try:
            parts = str(sample.key_expr).split("/")
            if len(parts) >= 2:
                v = parts[1]
                if self.vehicle_code == v:
                    data = json.loads(sample.payload.to_string())
                    q = data.get("orientation", {})
                    qx = float(q.get("x", 0.0))
                    qy = float(q.get("y", 0.0))
                    qz = float(q.get("z", 0.0))
                    qw = float(q.get("w", 1.0))
                    yaw_rad = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
                    deg = math.degrees(yaw_rad)
                    if deg < 0: deg += 360.0
                    self.last_yaw = deg
        except Exception:
            pass

    def _publish_current_state(self):
        if not self.session:
            return

        # 1. Doğrudan Motor ve Direksiyon Komutları (VehicleHardware)
        motors = self.hardware.compute_actuators(throttle=self.throttle, steering=self.steering)
        # 2. Standart ROS 2 Twist
        twist = {
            "linear": {"x": round(self.throttle * 3.0, 2), "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": round(self.steering * 1.5, 2)}
        }

        try:
            self.session.put(f"pisim/{self.vehicle_code}/actuators", json.dumps(motors))
            self.session.put(f"pisim/{self.vehicle_code}/cmd_vel", json.dumps(twist))
        except Exception:
            pass

    def _draw_hud(self):
        # Sabit Terminal Ekranı (ASCII Barlar)
        sys.stdout.write("\033[H")

        box_width = 80
        line_bar = "═" * (box_width - 2)

        # Gaz Çubuğu
        th_pct = int(self.throttle * 100)
        bar_len = 20
        if self.throttle >= 0:
            filled = int(self.throttle * bar_len)
            th_bar = "[" + "█" * filled + " " * (bar_len - filled) + f"]  +{th_pct:>3}%"
        else:
            filled = int(abs(self.throttle) * bar_len)
            th_bar = "[" + " " * (bar_len - filled) + "▒" * filled + f"]  {th_pct:>3}%"

        # Direksiyon Çubuğu (Merkezli)
        st_pct = int(self.steering * 100)
        half = 10
        if self.steering < 0:
            left_fill = int(abs(self.steering) * half)
            st_bar = "[" + " " * (half - left_fill) + "◄" * left_fill + "|" + " " * half + f"]  {st_pct:>3}% SOL"
        elif self.steering > 0:
            right_fill = int(self.steering * half)
            st_bar = "[" + " " * half + "|" + "►" * right_fill + " " * (half - right_fill) + f"]  +{st_pct:>3}% SAĞ"
        else:
            st_bar = "[" + " " * half + "|" + " " * half + "]     0% DÜZ"

        lines = [
            f"╔{line_bar}╗",
            f"║  🕹️ PISIM CANLI MANUEL KLAVYE SÜRÜCÜSÜ (TELEOP) | ARAÇ: [{self.vehicle_code:<14}]  ║",
            f"╠{line_bar}╣",
            f"║  GAZ (THROTTLE)    : {th_bar.ljust(box_width - 27)} ║",
            f"║  DİREKSİYON (STEER): {st_bar.ljust(box_width - 27)} ║",
            f"╠{line_bar}╣",
            f"║  GERİ BİLDİRİM TELEMETRİSİ:                                                  ║",
            f"║    * Konum : Lat: {self.last_lat:.6f}, Lon: {self.last_lon:.6f}                             ║",
            f"║    * Pusula: {self.last_yaw:>5.1f}° | Hız: {self.last_speed:>4.1f} m/s | Paket: {self.telemetry_rx_count:>5}                    ║",
            f"╠{line_bar}╣",
            f"║  KONTROLLER:                                                                 ║",
            f"║    [W / ↑]: Gaz Ver   | [S / ↓]: Fren/Geri | [A / ←]: Sola Kır | [D / →]: Sağa ║",
            f"║    [SPACE]: ACİL DUR  | [X]: Direksiyonu Düzle   | [Q / ESC]: Çıkış          ║",
            f"╚{line_bar}╝",
        ]
        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()

    def run(self):
        if not self._init_zenoh():
            return

        self.running = True
        self.session.declare_subscriber("pisim/*/gps", self._on_gps)
        self.session.declare_subscriber("pisim/*/imu", self._on_imu)

        # Eğer auto ise ilk aracı bekle
        if self.vehicle_code.lower() == "auto":
            print("🔍 Unreal Engine'deki aktif araç aranıyor...")
            for _ in range(25):
                if self.vehicle_code.lower() != "auto":
                    break
                time.sleep(0.1)

        sys.stdout.write("\033[2J\033[H")
        sys.stdout.flush()

        with KeyboardReader() as reader:
            last_cmd_pub = 0.0
            last_hud_draw = 0.0

            try:
                while self.running:
                    now = time.time()
                    key = reader.get_key()

                    if key:
                        if key in ('w', 'UP'):
                            self.throttle = min(1.0, round(self.throttle + self.throttle_step, 2))
                        elif key in ('s', 'DOWN'):
                            self.throttle = max(-1.0, round(self.throttle - self.throttle_step, 2))
                        elif key in ('a', 'LEFT'):
                            self.steering = max(-1.0, round(self.steering - self.steering_step, 2))
                        elif key in ('d', 'RIGHT'):
                            self.steering = min(1.0, round(self.steering + self.steering_step, 2))
                        elif key == 'x':
                            self.steering = 0.0
                        elif key in (' ', 'space'):
                            self.throttle = 0.0
                            self.steering = 0.0
                        elif key in ('q', 'esc', 'ESC'):
                            break

                        self._publish_current_state()

                    # 20 Hz sürekli komut akışı (Watchdog düşmesin)
                    if now - last_cmd_pub >= 0.05:
                        last_cmd_pub = now
                        self._publish_current_state()

                    # 10 Hz HUD çizimi
                    if now - last_hud_draw >= 0.10:
                        last_hud_draw = now
                        self._draw_hud()

                    time.sleep(0.01)

            except KeyboardInterrupt:
                pass
            finally:
                self.throttle = 0.0
                self.steering = 0.0
                self._publish_current_state()
                if self.session:
                    self.session.close()
                sys.stdout.write("\n🛑 Manuel sürüş sonlandırıldı. Araç durduruldu.\n")
                sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(description="PiSim Canlı Manuel Klavye Teleoperasyon Sürücüsü")
    parser.add_argument("--vehicle", "-v", type=str, default="auto", help="Sürülecek araç kodu (örn: RV01-8AD131 veya auto)")
    parser.add_argument("--router", "-r", type=str, default=None, help="Zenoh router adresi (opsiyonel)")
    args = parser.parse_args()

    controller = ManualTeleopController(vehicle_code=args.vehicle, router_endpoint=args.router)
    controller.run()


if __name__ == "__main__":
    main()
