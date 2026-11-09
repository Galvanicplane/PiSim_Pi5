#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
agent_supervisor.py - PiSim Ajan Koordinatörü ve Canlı Terminal Monitörü (Dosya 2)
==================================================================================
Bu modül, her bir Docker konteynerinin (veya bağımsız Pi 5 düğümünün) ana yöneticisidir:

Özellikler:
  1. Hangi Araca Bağlanacağını Belirleme Yöntemleri:
     A) Otomatik Keşif (Auto-Discovery):
        `--vehicle auto` verildiğinde Unreal Engine'de yayınlanan aktif araçları
        (pisim/*/gps) otomatik dinler ve ilk aktif aracı bulup bağlanır!
     B) Terminalden Elle Belirtme (--vehicle rover_01):
        İstediğiniz aracın kodunu doğrudan parametre olarak verebilirsiniz.
     C) İnteraktif Seçim Modu (--select):
        Ağdaki aktif Unreal araçlarını tarar, ekrana liste çıkarır ve hangisine
        bağlanmak istediğinizi klavyeden seçtirir.
     D) Çalışırken Dinamik Araç Değiştirme (Topic ile Atama):
        Konteyner çalışırken bile `pisim/<agent_id>/assign` topic'ine yeni araç kodu
        gönderildiğinde (veya `python agent_supervisor.py --assign rover_02`), konteyner
        hiç kapatılmadan anında yeni araca geçer!

  2. Sürekli Bağlantı Kontrolü (Connection Watchdog):
     - Hedef aracın `gps` ve `imu` topic'lerini izler.
     - Araç henüz bağlanmadıysa veya bağlantı koptuysa motorları durdurur (Fail-safe)
       ve bağlantı arama modunda bekler.

  3. Görev Koordinasyonu ve Otopilot (Mission & ArduPilot L1 Driver):
     - Araç bağlandığı anda Dosya 1'den (`mission_planner.py`) araca özgü 100m aralıklı
       4 hedef üretir ve Dosya 3'e (`autopilot_driver.py`) ileterek aracı sürer.

  4. Sabit (Aşağı Kaymayan) Canlı Terminal Ekranı:
     - Terminalde şelale gibi akmak yerine, tek bir sabit gösterge paneli (HUD) olarak
       anlık durumu yeniler.
"""

import argparse
import json
import math
import os
import signal
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR and SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

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


def scan_active_vehicles(router_endpoint: Optional[str], scan_duration: float = 2.0) -> List[str]:
    """Zenoh ağını tarayarak Unreal'da aktif olan araç kodlarını listeler."""
    if not HAS_ZENOH:
        return []

    conf = zenoh.Config()
    norm_ep = normalize_zenoh_endpoint(router_endpoint)
    if norm_ep:
        conf.insert_json5("mode", json.dumps("client"))
        conf.insert_json5("connect/endpoints", json.dumps([norm_ep]))
    else:
        conf.insert_json5("mode", json.dumps("peer"))

    found_vehicles = set()
    try:
        s = zenoh.open(conf)

        def cb(sample):
            parts = str(sample.key_expr).split("/")
            if len(parts) >= 2 and parts[0] == "pisim":
                found_vehicles.add(parts[1])

        sub1 = s.declare_subscriber("pisim/*/gps", cb)
        sub2 = s.declare_subscriber("pisim/*/imu", cb)
        time.sleep(scan_duration)
        s.close()
    except Exception:
        pass

    return sorted(list(found_vehicles))


def send_assignment(router_endpoint: Optional[str], target_agent: str, target_vehicle: str) -> bool:
    """Çalışan bir Docker konteynerine uzaktan yeni araç ataması yapar."""
    if not HAS_ZENOH:
        print("❌ 'eclipse-zenoh' modülü bulunamadı!")
        return False

    conf = zenoh.Config()
    norm_ep = normalize_zenoh_endpoint(router_endpoint)
    if norm_ep:
        conf.insert_json5("mode", json.dumps("client"))
        conf.insert_json5("connect/endpoints", json.dumps([norm_ep]))

    try:
        s = zenoh.open(conf)
        key = f"pisim/{target_agent}/assign"
        payload = json.dumps({"target": target_vehicle, "agent": target_agent, "ts": time.time()})
        s.put(key, payload)
        # Genel atama kanalına da bas
        s.put("pisim/assign", payload)
        time.sleep(0.3)
        s.close()
        print(f"✅ Araç atama mesajı gönderildi: [{target_agent}] ➔ Hedef Araç: [{target_vehicle}]")
        return True
    except Exception as e:
        print(f"❌ Atama mesajı gönderilemedi: {e}")
        return False


class AgentSupervisor:
    """
    Konteyner ve araç koordinatörü:
    Bağlantı takibi + Görev yönetimi + Otopilot sürüşü + Sabit Terminal HUD
    """

    def __init__(
        self,
        agent_id: str = "rover_04",
        target_vehicle: str = "auto",
        router_endpoint: Optional[str] = None,
        listen_endpoint: Optional[str] = None,
        min_distance_m: float = 100.0,
        num_waypoints: int = 4,
        min_turn_radius_m: float = 4.0,
        cruise_speed_mps: float = 2.2,
    ):
        self.agent_id = agent_id.strip()
        self.target_vehicle = target_vehicle.strip() if target_vehicle else "auto"
        self.auto_discovery = (self.target_vehicle.lower() in ("auto", "keşif", "kesif", ""))
        self.router_endpoint = normalize_zenoh_endpoint(router_endpoint)
        self.listen_endpoint = normalize_zenoh_endpoint(listen_endpoint)
        self.min_distance_m = float(min_distance_m)
        self.num_waypoints = int(num_waypoints)

        self.running = False
        self.session = None
        self.lock = threading.Lock()

        # Sensör ve Bağlantı Durumu
        self.last_gps: Optional[Dict] = None
        self.last_imu: Optional[Dict] = None
        self.last_sensor_time: float = 0.0
        self.connected: bool = False
        self.connection_timeout_sec: float = 2.5
        self.connection_search_start: float = time.time()
        self.discovered_vehicles: set = set()

        # Otopilot Sürücüsü (Dosya 3)
        self.driver = ArduPilotDriver(
            vehicle_code=self.target_vehicle if not self.auto_discovery else "waiting",
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

        # Zenoh Abonelikleri
        self.subs: List[Any] = []

    def set_target_vehicle(self, new_target: str, user_assigned: bool = False):
        """Çalışırken hedef aracı dinamik olarak değiştirir."""
        with self.lock:
            if not new_target or new_target.lower() == "auto":
                self.auto_discovery = True
                self.target_vehicle = "auto"
                return

            if self.target_vehicle == new_target and self.connected:
                return

            old_vehicle = self.target_vehicle
            self.target_vehicle = new_target
            self.auto_discovery = False

            # Önceki aracı güvenle durdur
            if self.session and old_vehicle not in ("auto", "waiting"):
                try:
                    cmd_stop = self.driver.emergency_stop()
                    self.session.put(f"pisim/{old_vehicle}/actuators", json.dumps(cmd_stop.motors))
                except Exception:
                    pass

            # Durumu sıfırla
            self.connected = False
            self.last_gps = None
            self.last_imu = None
            self.last_sensor_time = 0.0
            self.mission = None
            self.current_wp_idx = 0
            self.last_cmd = None
            self.connection_search_start = time.time()

            # Sürücüyü yeni araç için güncelle
            self.driver = ArduPilotDriver(
                vehicle_code=self.target_vehicle,
                min_turn_radius_m=self.driver.min_turn_radius_m,
                cruise_speed_mps=self.driver.cruise_speed_mps,
                wp_radius_m=self.driver.wp_radius_m,
            )

            # Yeni araç topic'lerine abone ol
            self._update_subscriptions()

    def _init_zenoh(self) -> bool:
        """Zenoh oturumunu açar."""
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
            self._update_subscriptions()

            # Dinamik araç atama topic'lerini dinle
            self.session.declare_subscriber(f"pisim/{self.agent_id}/assign", self._on_assign)
            self.session.declare_subscriber("pisim/assign", self._on_assign)

            return True
        except Exception as e:
            print(f"❌ Zenoh oturumu açılamadı: {e}")
            return False

    def _update_subscriptions(self):
        """Hedef araca göre Zenoh aboneliklerini açar."""
        if not self.session:
            return

        # Varsa eski sensör aboneliklerini temizle
        for s in self.subs:
            try:
                s.undeclare()
            except Exception:
                pass
        self.subs.clear()

        if self.auto_discovery:
            # Otomatik keşif modu: Tüm araçların GPS ve IMU topic'lerini dinle
            s_gps = self.session.declare_subscriber("pisim/*/gps", self._on_gps_wildcard)
            s_imu = self.session.declare_subscriber("pisim/*/imu", self._on_imu_wildcard)
            self.subs.extend([s_gps, s_imu])
        else:
            # Belirli araca doğrudan abone ol
            s_gps = self.session.declare_subscriber(f"pisim/{self.target_vehicle}/gps", self._on_gps_direct)
            s_imu = self.session.declare_subscriber(f"pisim/{self.target_vehicle}/imu", self._on_imu_direct)
            s_ping = self.session.declare_subscriber(f"pisim/{self.target_vehicle}/ping", self._on_ping)
            self.subs.extend([s_gps, s_imu, s_ping])

    def _on_assign(self, sample):
        """Uzaktan gelen 'şu araca bağlan' komutunu işler."""
        try:
            text = sample.payload.to_string()
            data = json.loads(text) if text.startswith("{") else {"target": text.strip()}
            target = data.get("target") or data.get("vehicle")
            agent = data.get("agent")
            if target and (not agent or agent == self.agent_id or agent == "all"):
                self.set_target_vehicle(target, user_assigned=True)
        except Exception:
            pass

    def _on_gps_wildcard(self, sample):
        try:
            parts = str(sample.key_expr).split("/")
            if len(parts) >= 2 and parts[0] == "pisim":
                v_code = parts[1]
                self.discovered_vehicles.add(v_code)
                if self.auto_discovery and not self.connected:
                    # Otomatik keşfedilen ilk araca bağlan!
                    self.set_target_vehicle(v_code)
                    self._on_gps_direct(sample)
        except Exception:
            pass

    def _on_imu_wildcard(self, sample):
        try:
            parts = str(sample.key_expr).split("/")
            if len(parts) >= 2 and parts[0] == "pisim":
                v_code = parts[1]
                self.discovered_vehicles.add(v_code)
                if self.target_vehicle == v_code:
                    self._on_imu_direct(sample)
        except Exception:
            pass

    def _on_gps_direct(self, sample):
        try:
            payload = json.loads(sample.payload.to_string())
            self.last_gps = payload
            self.last_sensor_time = time.time()
        except Exception:
            pass

    def _on_imu_direct(self, sample):
        try:
            payload = json.loads(sample.payload.to_string())
            self.last_imu = payload
            self.last_sensor_time = time.time()
        except Exception:
            pass

    def _on_ping(self, sample):
        try:
            if self.session:
                resp = {
                    "status": "pong",
                    "agent": self.agent_id,
                    "target_vehicle": self.target_vehicle,
                    "connected": self.connected,
                    "timestamp": time.time(),
                }
                self.session.put(f"pisim/{self.target_vehicle}/pong", json.dumps(resp))
        except Exception:
            pass

    def _extract_yaw_deg(self) -> float:
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
        if not self.session or self.target_vehicle in ("auto", "waiting"):
            return
        try:
            # 1. Eyleyici Motor Değerleri (Unreal doğrudan sürüş)
            self.session.put(f"pisim/{self.target_vehicle}/actuators", json.dumps(cmd.motors))

            # 2. ROS 2 Twist cmd_vel
            self.session.put(f"pisim/{self.target_vehicle}/cmd_vel", json.dumps(cmd.twist))

            # 3. Filo Durumu
            now = time.time()
            cur_lat = float(self.last_gps.get("latitude", 0.0)) if self.last_gps else 0.0
            cur_lon = float(self.last_gps.get("longitude", 0.0)) if self.last_gps else 0.0
            state = {
                "agent_id": self.agent_id,
                "target_vehicle": self.target_vehicle,
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
            self.session.put(f"fleet/{self.agent_id}/state", json.dumps(state))
        except Exception:
            pass

    def start(self):
        """Ajan ana döngüsünü başlatır."""
        self.running = True
        self.connection_search_start = time.time()

        if not self._init_zenoh():
            print("❌ Zenoh başlatılamadığı için program sonlandırılıyor.")
            return

        sys.stdout.write("\033[2J\033[H")
        sys.stdout.flush()

        step_delay = 0.05  # 20 Hz kontrol döngüsü

        try:
            while self.running:
                now = time.time()

                # -------------------------------------------------------------
                # 1. BAĞLANTI DURUMU KONTROLÜ
                # -------------------------------------------------------------
                has_recent_data = (now - self.last_sensor_time) <= self.connection_timeout_sec and self.last_sensor_time > 0

                if has_recent_data and not self.connected and self.target_vehicle not in ("auto", "waiting"):
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
                        vehicle_code=self.target_vehicle,
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
                            self.mission = generate_random_mission(
                                origin_lat=cur_lat,
                                origin_lon=cur_lon,
                                num_waypoints=self.num_waypoints,
                                min_distance_m=self.min_distance_m,
                                vehicle_code=self.target_vehicle,
                            )
                            self.current_wp_idx = 1
                            first_wp = self.mission.waypoints[0]
                            self.driver.set_target_waypoint(first_wp.lat, first_wp.lon, index=1)
                    else:
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
        now = time.time()
        lines = []

        box_width = 88
        line_bar = "═" * (box_width - 2)

        lines.append(f"╔{line_bar}╗")
        target_display = self.target_vehicle if not self.auto_discovery else "🔍 OTOMATİK KEŞİF BEKLENİYOR"
        title = f" 🚀 PISIM DOCKER AJANI: [{self.agent_id}] ➔ HEDEF ARAÇ: [{target_display}] "
        lines.append(f"║ {title.ljust(box_width - 4)} ║")
        lines.append(f"╠{line_bar}╣")

        # 1. Bağlantı Durumu Satırı
        if self.connected:
            sec_ago = max(0.0, now - self.last_sensor_time)
            status_text = f"🟢 BAĞLANDI: Unreal '{self.target_vehicle}' aktif (Son Veri: {sec_ago:.2f}s önce)"
        elif self.auto_discovery:
            disc_str = ", ".join(self.discovered_vehicles) if self.discovered_vehicles else "Taranıyor..."
            status_text = f"🔍 OTOMATİK KEŞİF: Unreal'da araç aranıyor... (Görülen: {disc_str})"
        else:
            elapsed = max(0.0, now - self.connection_search_start)
            status_text = f"🔴 BAĞLANTI BEKLENİYOR... ({elapsed:4.1f}s geçti - '{self.target_vehicle}' aranıyor)"

        lines.append(f"║ DURUM      : {status_text[:box_width-17].ljust(box_width - 17)} ║")

        # 2. Zenoh Ağ ve Atama Bilgisi Satırı
        router_info = self.router_endpoint if self.router_endpoint else "Peer-to-Peer Mesh"
        net_text = f"Router: {router_info} | Atama Kanalı: pisim/{self.agent_id}/assign"
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

            m = self.last_cmd.motors
            s_val = m.get("M_Steer_FL", 0.0)
            w_val = m.get("M_Wheel_FL", 0.0)
            lines.append(f"║ AKTUATÖRLER: Servolar: {s_val:+0.2f} | Tekerlekler: {w_val:+0.2f} | Topic: pisim/{self.target_vehicle}/actuators".ljust(box_width-2) + " ║")
        else:
            disc_str = ", ".join(self.discovered_vehicles) if self.discovered_vehicles else "Yok (Unreal bekleniyor)"
            lines.append(f"║ AĞDA GÖRÜLEN : {disc_str}".ljust(box_width-2) + " ║")
            lines.append(f"║ BİLGİ        : Araç bağlandığında otopilot 100m aralıklı 4 hedefini çizip sürer.".ljust(box_width-2) + " ║")
            lines.append(f"║ ARAÇ DEĞİŞTİR: 'python agent_supervisor.py --assign <araç>' ile anında araç ata".ljust(box_width-2) + " ║")

        lines.append(f"╚{line_bar}╝")
        lines.append("\033[K(Durdurmak için Ctrl+C)")

        sys.stdout.write("\033[H" + "\n".join(lines) + "\n")
        sys.stdout.flush()

    def stop(self):
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
    parser.add_argument("--agent", "-a", type=str, default=os.environ.get("VEHICLE_CODE", "rover_04"), help="Konteyner/Ajan kimliği (varsayılan: rover_04)")
    parser.add_argument("--vehicle", "-v", type=str, default=os.environ.get("TARGET_VEHICLE", "auto"), help="Bağlanılacak Unreal araç kodu ('auto', 'rover_01' vb.)")
    parser.add_argument("--select", "-s", action="store_true", help="Ağdaki aktif araçları tara ve interaktif olarak bağlanılacak aracı seç")
    parser.add_argument("--assign", type=str, default=None, help="Çalışan bir ajana uzaktan araç ata (örn: --assign rover_01)")
    parser.add_argument("--router", "-r", type=str, default=os.environ.get("ZENOH_ROUTER", None))
    parser.add_argument("--listen", "-l", type=str, default=os.environ.get("ZENOH_LISTEN_ENDPOINT", None))
    parser.add_argument("--distance", "-d", type=float, default=100.0, help="Noktalar arası ve başlangıçtan asgari mesafe (varsayılan: 100.0 m)")
    parser.add_argument("--count", "-c", type=int, default=4, help="Hedef sayısı (varsayılan: 4)")
    parser.add_argument("--turn-radius", type=float, default=4.0, help="Asgari dönüş yarıçapı (varsayılan: 4.0 m)")
    parser.add_argument("--speed", type=float, default=2.2, help="Seyir hızı (m/s, varsayılan: 2.2)")

    args = parser.parse_args()

    # 1. Uzaktan Atama Modu
    if args.assign:
        send_assignment(router_endpoint=args.router, target_agent=args.agent, target_vehicle=args.assign)
        return

    # 2. İnteraktif Seçim Modu
    chosen_vehicle = args.vehicle
    if args.select:
        print("\n🔍 Zenoh ağı taranıyor (Unreal'daki aktif araçlar aranıyor)...")
        vehicles = scan_active_vehicles(args.router, scan_duration=2.5)
        print("----------------------------------------------------------------")
        if vehicles:
            print("🚗 Unreal Engine'de Tespit Edilen Aktif Araçlar:")
            for idx, v in enumerate(vehicles, 1):
                print(f"   [{idx}] {v}")
            print("----------------------------------------------------------------")
            choice = input(f"Bağlanmak istediğiniz araç numarası veya kodu [1-{len(vehicles)} / rover_01]: ").strip()
            if choice.isdigit() and 1 <= int(choice) <= len(vehicles):
                chosen_vehicle = vehicles[int(choice) - 1]
            elif choice:
                chosen_vehicle = choice
            else:
                chosen_vehicle = vehicles[0]
        else:
            print("⚠️ Ağda henüz aktif yayın yapan araç bulunamadı (Unreal henüz başlatılmamış olabilir).")
            val = input("Bağlanılacak araç kodunu elle girin (veya otomatik keşif için 'auto'): ").strip()
            chosen_vehicle = val if val else "auto"

    agent = AgentSupervisor(
        agent_id=args.agent,
        target_vehicle=chosen_vehicle,
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
