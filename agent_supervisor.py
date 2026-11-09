#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
agent_supervisor.py - PiSim Ajan Koordinatörü ve Canlı Terminal Monitörü (Dosya 2)
==================================================================================
Bu modül, her bir Docker konteynerinin (veya bağımsız Pi 5 düğümünün) ana yöneticisidir:

Sorumlulukları:
  1. Sürekli Bağlantı Kontrolü (Connection Watchdog):
     - Unreal Engine'den gelen `pisim/<VEHICLE_CODE>/gps` ve `imu` topic'lerini izler.
     - Araç henüz bağlanmadıysa veya bağlantı koptuysa motorları durdurur (Fail-safe)
       ve bağlantı arama modunda bekler.
  2. Görev Koordinasyonu (Mission Supervisor):
     - Araç bağlandığı anda Dosya 1'den (`mission_planner.py`) parametrik olarak
       kendine özgü 4 adet (her biri başlangıçtan ve birbirinden 100m uzakta) hedef alır.
  3. Otopilot Kılavuzluğu (Driver Integration):
     - Görev noktalarını sırayla Dosya 3'e (`autopilot_driver.py`) iletir.
     - ArduPilot L1 Nonlinear Guidance algoritmasıyla aracı gerçekçi olarak sürer.
  4. Sabit (Aşağı Kaymayan) Canlı Terminal Ekranı:
     - Terminalde şelale gibi akmak yerine, ANSI escape sekansları ile tek bir sabit
       gösterge paneli (HUD) olarak anlık durumu yeniler.
"""

import argparse
import json
import math
import os
import signal
import sys
import time
from typing import Dict, Optional, Tuple

# sys.path düzenlemesi
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR and SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

# UTF-8 terminal desteği
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

from autopilot_driver import ArduPilotDriver, DriveCommand
from mission_planner import MissionPlan, Waypoint, generate_random_mission


def normalize_zenoh_endpoint(ep: Optional[str], default_port: int = 7447) -> Optional[str]:
    """tcp:// ve udp:// formatlarını Zenoh tcp/ veya udp/ formatına dönüştürür."""
    if not ep:
        return None
    ep = ep.strip()
    if ep.startswith("tcp://"):
        ep = "tcp/" + ep[6:]
    elif ep.startswith("udp://"):
        ep = "udp/" + ep[6:]
    elif not (ep.startswith("tcp/") or ep.startswith("udp/")):
        if ":" not in ep:
            ep = f"tcp/{ep}:{default_port}"
        else:
            ep = f"tcp/{ep}"
    return ep


class AgentSupervisor:
    """
    Konteyner ve araç koordinatörü:
    Bağlantı takibi + Görev yönetimi + Otopilot sürüşü + Sabit Terminal HUD
    """

    def __init__(
        self,
        vehicle_code: str = "rover_01",
        router_endpoint: Optional[str] = None,
        listen_endpoint: Optional[str] = None,
        min_distance_m: float = 100.0,
        num_waypoints: int = 4,
        min_turn_radius_m: float = 4.0,
        cruise_speed_mps: float = 2.2,
    ):
        self.vehicle_code = vehicle_code
        self.router_endpoint = normalize_zenoh_endpoint(router_endpoint)
        self.listen_endpoint = normalize_zenoh_endpoint(listen_endpoint)
        self.min_distance_m = float(min_distance_m)
        self.num_waypoints = int(num_waypoints)

        self.running = False
        self.session = None

        # Sensör ve Bağlantı Durumu
        self.last_gps: Optional[Dict] = None
        self.last_imu: Optional[Dict] = None
        self.last_sensor_time: float = 0.0
        self.connected: bool = False
        self.connection_timeout_sec: float = 2.5
        self.connection_search_start: float = time.time()

        # Otopilot Sürücüsü (Dosya 3)
        self.driver = ArduPilotDriver(
            vehicle_code=self.vehicle_code,
            min_turn_radius_m=min_turn_radius_m,
            cruise_speed_mps=cruise_speed_mps,
            wp_radius_m=3.5,
        )

        # Aktif Görev Planı (Dosya 1)
        self.mission: Optional[MissionPlan] = None
        self.current_wp_idx: int = 0
        self.completed_missions_count: int = 0
        self.last_cmd: Optional[DriveCommand] = None

        # Canlı HUD Çizim Zamanlayıcısı
        self.last_hud_draw_time: float = 0.0
        self.hud_refresh_interval_sec: float = 0.20  # 5 Hz terminal yenileme

    def _init_zenoh(self) -> bool:
        """Zenoh oturumunu açar ve ilgili topic'lere abone olur."""
        if not HAS_ZENOH:
            print("❌ 'eclipse-zenoh' modülü bulunamadı! 'pip install eclipse-zenoh' kurun.")
            return False

        conf = zenoh.Config()

        if self.listen_endpoint:
            conf.insert_json5("mode", json.dumps("router"))
            conf.insert_json5("listen/endpoints", json.dumps([self.listen_endpoint]))

        if self.router_endpoint:
            if not self.listen_endpoint:
                conf.insert_json5("mode", json.dumps("client"))
            conf.insert_json5("connect/endpoints", json.dumps([self.router_endpoint]))
        elif not self.listen_endpoint:
            conf.insert_json5("mode", json.dumps("peer"))

        try:
            self.session = zenoh.open(conf)

            # Topic abonelikleri
            self.session.declare_subscriber(f"pisim/{self.vehicle_code}/gps", self._on_gps)
            self.session.declare_subscriber(f"pisim/{self.vehicle_code}/imu", self._on_imu)
            self.session.declare_subscriber(f"pisim/{self.vehicle_code}/ping", self._on_ping)

            return True
        except Exception as e:
            print(f"❌ Zenoh oturumu açılamadı: {e}")
            return False

    def _on_gps(self, sample):
        try:
            payload = json.loads(sample.payload.to_string())
            self.last_gps = payload
            self.last_sensor_time = time.time()
        except Exception:
            pass

    def _on_imu(self, sample):
        try:
            payload = json.loads(sample.payload.to_string())
            self.last_imu = payload
            self.last_sensor_time = time.time()
        except Exception:
            pass

    def _on_ping(self, sample):
        """Unreal veya diğer düğümlerden gelen ping çağrılarına yanıt verir."""
        try:
            if self.session:
                resp = {
                    "status": "pong",
                    "agent": self.vehicle_code,
                    "connected": self.connected,
                    "timestamp": time.time(),
                }
                self.session.put(f"pisim/{self.vehicle_code}/pong", json.dumps(resp))
        except Exception:
            pass

    def _extract_yaw_deg(self) -> float:
        """IMU kuaterniyonundan Pusula Baş Açısını (Yaw) derece olarak çıkarır."""
        if not self.last_imu:
            return 0.0
        q = self.last_imu.get("orientation", {})
        qx = float(q.get("x", 0.0))
        qy = float(q.get("y", 0.0))
        qz = float(q.get("z", 0.0))
        qw = float(q.get("w", 1.0))
        yaw_rad = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        yaw_deg = math.degrees(yaw_rad)
        if yaw_deg < 0:
            yaw_deg += 360.0
        return yaw_deg

    def _publish_commands(self, cmd: DriveCommand):
        """Üretilen sürüş komutlarını Zenoh üzerinden Unreal'a ve ağa basar."""
        if not self.session:
            return
        try:
            # 1. Eyleyici Motor Değerleri (Unreal doğrudan sürüş)
            self.session.put(f"pisim/{self.vehicle_code}/actuators", json.dumps(cmd.motors))

            # 2. ROS 2 Twist cmd_vel
            self.session.put(f"pisim/{self.vehicle_code}/cmd_vel", json.dumps(cmd.twist))

            # 3. Filo Durumu (Fleet state mesh)
            now = time.time()
            cur_lat = float(self.last_gps.get("latitude", 0.0)) if self.last_gps else 0.0
            cur_lon = float(self.last_gps.get("longitude", 0.0)) if self.last_gps else 0.0
            state = {
                "vehicle": self.vehicle_code,
                "connected": self.connected,
                "lat": cur_lat,
                "lon": cur_lon,
                "wp_index": self.current_wp_idx,
                "total_wps": self.mission.total_waypoints if self.mission else 0,
                "dist_m": cmd.dist_to_target_m,
                "steer_deg": cmd.steer_deg,
                "throttle": cmd.throttle,
                "timestamp": now,
            }
            self.session.put(f"fleet/{self.vehicle_code}/state", json.dumps(state))
        except Exception:
            pass

    def start(self):
        """Ajan ana döngüsünü başlatır."""
        self.running = True
        self.connection_search_start = time.time()

        if not self._init_zenoh():
            print("❌ Zenoh başlatılamadığı için program sonlandırılıyor.")
            return

        # Terminal ekranını temizle ve sabit HUD'ı başlat
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.flush()

        step_delay = 0.05  # 20 Hz kontrol döngüsü

        try:
            while self.running:
                now = time.time()

                # -------------------------------------------------------------
                # 1. BAĞLANTI DURUMU KONTROLÜ
                # -------------------------------------------------------------
                has_recent_data = (now - self.last_sensor_time) <= self.connection_timeout_sec

                if has_recent_data and not self.connected:
                    # YENİ BAĞLANTI KURULDU!
                    self.connected = True
                    cur_lat = float(self.last_gps.get("latitude", 39.9334))
                    cur_lon = float(self.last_gps.get("longitude", 32.8597))

                    # Dosya 1'den araca özgü 4 hedefli görev planı al
                    self.mission = generate_random_mission(
                        origin_lat=cur_lat,
                        origin_lon=cur_lon,
                        num_waypoints=self.num_waypoints,
                        min_distance_m=self.min_distance_m,
                        vehicle_code=self.vehicle_code,
                    )
                    self.current_wp_idx = 1
                    first_wp = self.mission.waypoints[0]
                    self.driver.set_target_waypoint(first_wp.lat, first_wp.lon, index=1)

                elif not has_recent_data and self.connected:
                    # BAĞLANTI KOPTU! Acil durdurma uygula
                    self.connected = False
                    self.connection_search_start = now
                    cmd_stop = self.driver.emergency_stop()
                    self._publish_commands(cmd_stop)
                    self.last_cmd = cmd_stop

                # -------------------------------------------------------------
                # 2. GÖREV VE OTOPİLOT YÜRÜTME DÖNGÜSÜ
                # -------------------------------------------------------------
                if self.connected and self.mission and self.last_gps:
                    cur_lat = float(self.last_gps.get("latitude", 39.9334))
                    cur_lon = float(self.last_gps.get("longitude", 32.8597))
                    cur_yaw = self._extract_yaw_deg()
                    cur_spd = float(self.last_gps.get("speed", 0.0))

                    cmd = self.driver.compute_step(
                        cur_lat=cur_lat,
                        cur_lon=cur_lon,
                        cur_yaw_deg=cur_yaw,
                        cur_speed_mps=cur_spd,
                    )
                    self.last_cmd = cmd

                    # Hedefe ulaşıldı mı?
                    if cmd.is_arrived:
                        if self.current_wp_idx < self.mission.total_waypoints:
                            self.current_wp_idx += 1
                            next_wp = self.mission.waypoints[self.current_wp_idx - 1]
                            self.driver.set_target_waypoint(next_wp.lat, next_wp.lon, index=self.current_wp_idx)
                        else:
                            # 4 HEDEFİN TÜMÜ TAMAMLANDI!
                            self.completed_missions_count += 1
                            # Anlık konumdan yeni 4 hedef oluştur ve devriyeye devam et
                            self.mission = generate_random_mission(
                                origin_lat=cur_lat,
                                origin_lon=cur_lon,
                                num_waypoints=self.num_waypoints,
                                min_distance_m=self.min_distance_m,
                                vehicle_code=self.vehicle_code,
                            )
                            self.current_wp_idx = 1
                            first_wp = self.mission.waypoints[0]
                            self.driver.set_target_waypoint(first_wp.lat, first_wp.lon, index=1)
                    else:
                        # Komutları yayınla
                        self._publish_commands(cmd)

                # -------------------------------------------------------------
                # 3. SABİT (AŞAĞI KAYMAYAN) CANLI HUD EKRANI
                # -------------------------------------------------------------
                if now - self.last_hud_draw_time >= self.hud_refresh_interval_sec:
                    self.last_hud_draw_time = now
                    self._draw_fixed_hud()

                time.sleep(step_delay)

        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def _draw_fixed_hud(self):
        """Terminalde aşağı kaymayan (in-place) sabit gösterge paneli çizer."""
        now = time.time()
        lines = []

        box_width = 86
        line_bar = "═" * (box_width - 2)
        sub_bar = "─" * (box_width - 2)

        lines.append(f"╔{line_bar}╗")
        title = f" 🚀 PISIM OTONOM AJAN BEYNİ | Araç: [{self.vehicle_code}] "
        lines.append(f"║ {title.ljust(box_width - 4)} ║")
        lines.append(f"╠{line_bar}╣")

        # 1. Bağlantı Durumu Satırı
        if self.connected:
            sec_ago = max(0.0, now - self.last_sensor_time)
            status_text = f"🟢 BAĞLANDI (Unreal Engine Aktif | Son Veri: {sec_ago:.2f}s önce)"
        else:
            elapsed = max(0.0, now - self.connection_search_start)
            status_text = f"🔴 BAĞLANTI BEKLENİYOR... ({elapsed:4.1f} sn geçti - Araç bekleniyor)"

        lines.append(f"║ DURUM      : {status_text.ljust(box_width - 17)} ║")

        # 2. Zenoh Ağ Satırı
        router_info = self.router_endpoint if self.router_endpoint else "Peer-to-Peer Mesh"
        net_text = f"Router: {router_info} | Topic: pisim/{self.vehicle_code}/*"
        lines.append(f"║ ZENOH AĞI  : {net_text[:box_width-17].ljust(box_width - 17)} ║")
        lines.append(f"╠{line_bar}╣")

        if self.connected and self.mission and self.last_gps and self.last_cmd:
            cur_lat = float(self.last_gps.get("latitude", 0.0))
            cur_lon = float(self.last_gps.get("longitude", 0.0))
            cur_spd = float(self.last_gps.get("speed", 0.0))
            cur_yaw = self._extract_yaw_deg()

            wp = self.mission.waypoints[self.current_wp_idx - 1]
            total_wps = self.mission.total_waypoints
            rem_m = self.last_cmd.dist_to_target_m
            brg = self.last_cmd.bearing_deg
            eta = self.last_cmd.heading_err_deg
            steer = self.last_cmd.steer_deg
            throttle = int(self.last_cmd.throttle * 100)

            # İlerleme Çubuğu (Progress Bar)
            total_leg = max(1.0, wp.target_dist_m)
            progress_pct = max(0.0, min(100.0, (1.0 - (rem_m / total_leg)) * 100.0))
            bar_len = 16
            filled = int((progress_pct / 100.0) * bar_len)
            p_bar = "█" * filled + "░" * (bar_len - filled)

            steer_arrow = "◄◄ SOL" if steer < -10 else ("◄ sol" if steer < -2 else ("SAĞ ►►" if steer > 10 else ("sağ ►" if steer > 2 else "▲ DÜZ")))

            lines.append(f"║ GÖREV PLANI: Hedef [{self.current_wp_idx} / {total_wps}]  (Asgari Mesafe Kuralı: {self.min_distance_m:.1f} m) | Tur #{self.completed_missions_count+1}".ljust(box_width-2) + " ║")
            lines.append(f"║ HEDEF GPS  : 📍 ({wp.lat:.6f}°N, {wp.lon:.6f}°E)".ljust(box_width-2) + " ║")
            lines.append(f"║ ARAÇ GPS   : 📍 ({cur_lat:.6f}°N, {cur_lon:.6f}°E) | Hız: {cur_spd:3.1f} m/s ({cur_spd*3.6:4.1f} km/h)".ljust(box_width-2) + " ║")
            lines.append(f"║ HEDEFE MES : 📏 {rem_m:5.1f} m  [{p_bar}] %{progress_pct:3.0f}".ljust(box_width-2) + " ║")
            lines.append(f"║ PUSULA (Y) : 🧭 Araç: {cur_yaw:05.1f}° ➔ Hedef: {brg:05.1f}° | Yön Hatası (η): {eta:+5.1f}°".ljust(box_width-2) + " ║")
            lines.append(f"║ OTOPİLOT L1: 🛞 Direksiyon: {steer:+5.1f}° ({steer_arrow}) | Gaz: %{throttle:2d} | ω: {self.last_cmd.omega_cmd:+4.2f} rad/s".ljust(box_width-2) + " ║")

            # Eyleyici Özeti
            m = self.last_cmd.motors
            s_val = m.get("M_Steer_FL", 0.0)
            w_val = m.get("M_Wheel_FL", 0.0)
            lines.append(f"║ AKTUATÖRLER: Servolar: {s_val:+0.2f} | Tekerlekler: {w_val:+0.2f} | Topic: pisim/{self.vehicle_code}/actuators".ljust(box_width-2) + " ║")
        else:
            lines.append(f"║ BİLGİ      : Araç Unreal Engine'de bağlandığında otopilot göreve başlayacaktır.".ljust(box_width-2) + " ║")
            lines.append(f"║ OTOPİLOT   : ⏸️ BEKLEMEDE (Motorlar ve direksiyon emniyet modunda kilitli)".ljust(box_width-2) + " ║")
            lines.append(f"║ BEKLEME    : Dinleniyor -> pisim/{self.vehicle_code}/gps ve /imu".ljust(box_width-2) + " ║")

        lines.append(f"╚{line_bar}╝")
        lines.append("\033[K(Durdurmak için Ctrl+C)")

        # ANSI Escape ile imleci başa al ve aşağı kaymadan üzerine çiz
        sys.stdout.write("\033[H" + "\n".join(lines) + "\n")
        sys.stdout.flush()

    def stop(self):
        """Tüm motorları sıfırlar ve Zenoh oturumunu kapatır."""
        self.running = False
        if self.session:
            try:
                cmd_stop = self.driver.emergency_stop()
                self._publish_commands(cmd_stop)
                self.session.close()
            except Exception:
                pass
        sys.stdout.write("\n🛑 Ajan güvenle durduruldu ve kapatıldı.\n")
        sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(description="PiSim Otonom Ajan Koordinatörü (Dosya 2)")
    parser.add_argument("--vehicle", "-v", type=str, default=os.environ.get("VEHICLE_CODE", "rover_01"))
    parser.add_argument("--router", "-r", type=str, default=os.environ.get("ZENOH_ROUTER", None))
    parser.add_argument("--listen", "-l", type=str, default=os.environ.get("ZENOH_LISTEN_ENDPOINT", None))
    parser.add_argument("--distance", "-d", type=float, default=100.0, help="Noktalar arası ve başlangıçtan asgari mesafe (varsayılan: 100.0 m)")
    parser.add_argument("--count", "-c", type=int, default=4, help="Hedef sayısı (varsayılan: 4)")
    parser.add_argument("--turn-radius", type=float, default=4.0, help="Asgari dönüş yarıçapı (varsayılan: 4.0 m)")
    parser.add_argument("--speed", type=float, default=2.2, help="Seyir hızı (m/s, varsayılan: 2.2)")

    args = parser.parse_args()

    agent = AgentSupervisor(
        vehicle_code=args.vehicle,
        router_endpoint=args.router,
        listen_endpoint=args.listen,
        min_distance_m=args.distance,
        num_waypoints=args.count,
        min_turn_radius_m=args.turn_radius,
        cruise_speed_mps=args.speed,
    )

    def handle_signal(sig, frame):
        agent.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    agent.start()


if __name__ == "__main__":
    main()
