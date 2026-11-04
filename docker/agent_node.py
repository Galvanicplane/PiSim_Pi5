#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
agent_node.py - PiSim Bağımsız Otonom Araç Beyni (Autonomous Swarm Agent)
==========================================================================
Bu modül, her biri bağımsız bir Docker konteynerinde veya fiziksel bir Raspberry Pi 5'te
çalışan otonom araç beynini temsil eder:

1. Gerçek ROS 2 Mesaj Formatları:
   - Dinler:
     * `pisim/<VEHICLE_CODE>/imu`          (sensor_msgs/msg/Imu)
     * `pisim/<VEHICLE_CODE>/gps`          (sensor_msgs/msg/NavSatFix)
     * `pisim/<VEHICLE_CODE>/joint_states` (sensor_msgs/msg/JointState)
     * `pisim/<VEHICLE_CODE>/ping`         (std_msgs/msg/String / Ping)
   - Yayınlar:
     * `pisim/<VEHICLE_CODE>/actuators`    (Eyleyici normalize sürüş komutları)
     * `pisim/<VEHICLE_CODE>/cmd_vel`      (geometry_msgs/msg/Twist)
     * `fleet/<VEHICLE_CODE>/state`        (Dağıtık bellek durum güncellemesi)

2. ArduPilot L1 Navigasyon Güdüm Algoritması (AP_L1_Control):
   - Unreal'dan gelen gerçek GPS (WGS84) ve IMU pusula baş açısı (Yaw) ile çalışır.
   - Ankara merkezli (veya anlık konum etrafında) rastgele otonom devriye hedefleri (Waypoints) üretir.
   - Geniş dönüş yarıçapı (`r_min = 4.0m`) ve L1 yanal ivme kontrolü ile hedeflere doğru
     ön tekerlek servolarını fiziksel olarak açılı çevirip tekerlekleri sürer.

3. Çok Atlamalı (Multi-Hop) Zenoh P2P Ağ Desteği:
   - Wi-Fi'a doğrudan erişimi olmayan izole konteynerler ara düğüm (`agent_rover_02`) üzerinden
     Zenoh köprüsüyle Unreal Engine'e kesintisiz ulaşır.
"""

import json
import logging
import math
import os
import random
import signal
import sys
import time
from typing import Dict, Optional, Tuple

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s][%(name)s] %(message)s",
    datefmt="%H:%M:%S"
)

try:
    import zenoh
    HAS_ZENOH = True
except ImportError:
    HAS_ZENOH = False

from vehicle_hardware import VehicleHardware

# Çevre Değişkenleri (Environment Variables)
VEHICLE_CODE = os.environ.get("VEHICLE_CODE", "rover_01")
UE5_HOST = os.environ.get("UE5_HOST", "host.docker.internal")
ZENOH_ROUTER = os.environ.get("ZENOH_ROUTER", None)
ZENOH_LISTEN = os.environ.get("ZENOH_LISTEN_ENDPOINT", None)

logger = logging.getLogger(f"Brain-{VEHICLE_CODE}")

EARTH_RADIUS_M = 6378137.0


def haversine_distance_and_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> Tuple[float, float, float, float]:
    """İki GPS koordinatı arasındaki mesafe ve pusula kerterizini hesaplar."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * (math.sin(dlambda / 2.0) ** 2)
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))
    dist_m = EARTH_RADIUS_M * c

    d_north_m = dphi * EARTH_RADIUS_M
    d_east_m = dlambda * EARTH_RADIUS_M * math.cos((phi1 + phi2) * 0.5)
    bearing_deg = math.degrees(math.atan2(d_east_m, d_north_m))
    return d_north_m, d_east_m, dist_m, bearing_deg


def offset_gps_by_meters(origin_lat: float, origin_lon: float, d_north_m: float, d_east_m: float) -> Tuple[float, float]:
    """Belirtilen metre kadar ötedeki GPS koordinatını hesaplar."""
    dlat = math.degrees(d_north_m / EARTH_RADIUS_M)
    dlon = math.degrees(d_east_m / (EARTH_RADIUS_M * max(1e-6, math.cos(math.radians(origin_lat)))))
    return origin_lat + dlat, origin_lon + dlon


class AutonomousSwarmBrain:
    def __init__(self, vehicle_code: str):
        self.vehicle_code = vehicle_code
        self.running = False
        self.hardware = VehicleHardware(vehicle_code=vehicle_code, drive_mode="ACKERMANN")
        self.session = None

        # Sensör Verileri
        self.last_gps: Optional[Dict] = None
        self.last_imu: Optional[Dict] = None
        self.last_joints: Optional[Dict] = None
        self.last_sensor_time = 0.0

        # Navigasyon Hedefi (Waypoint)
        self.current_wp: Optional[Tuple[float, float]] = None
        self.wp_count = 0
        self.cruise_speed_mps = 1.8
        self.min_turn_radius_m = 4.0
        self.wp_radius_m = 2.0

    def start(self):
        self.running = True
        logger.info(f"==================================================")
        logger.info(f" 🚀 OTONOM AJAN BEYNİ AKTİF: {self.vehicle_code}")
        logger.info(f"    Sürüş Modu: ACKERMANN (Ön Servolar Açılı Döner)")
        logger.info(f"    Algoritma  : ArduPilot L1 Nonlinear Guidance")
        logger.info(f"==================================================")

        if HAS_ZENOH:
            conf = zenoh.Config()
            if ZENOH_LISTEN:
                # Ara aktarıcı düğüm ise gelen bağlantıları dinle
                conf.insert_json5("listen/endpoints", json.dumps([ZENOH_LISTEN]))
                logger.info(f"📡 Zenoh Dinleme Noktası Açıldı: {ZENOH_LISTEN}")

            if ZENOH_ROUTER:
                # İzole düğüm ise ara aktarıcıya bağlan
                conf.insert_json5("mode", json.dumps("client"))
                conf.insert_json5("connect/endpoints", json.dumps([ZENOH_ROUTER]))
                logger.info(f"🔗 Zenoh Köprüsüne Bağlanılıyor: {ZENOH_ROUTER}")
            else:
                conf.insert_json5("mode", json.dumps("peer"))

            try:
                self.session = zenoh.open(conf)
                logger.info("✅ Zenoh Oturumu Başarıyla Açıldı.")

                # Kendi aracımıza ait topic'lere abone ol
                self.session.declare_subscriber(f"pisim/{self.vehicle_code}/gps", self._on_gps)
                self.session.declare_subscriber(f"pisim/{self.vehicle_code}/imu", self._on_imu)
                self.session.declare_subscriber(f"pisim/{self.vehicle_code}/joint_states", self._on_joints)
                self.session.declare_subscriber(f"pisim/{self.vehicle_code}/ping", self._on_ping)

            except Exception as e:
                logger.error(f"❌ Zenoh oturum hatası: {e}")
                self.session = None

        self._run_mission_loop()

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

    def _on_joints(self, sample):
        try:
            payload = json.loads(sample.payload.to_string())
            self.last_joints = payload
            self.last_sensor_time = time.time()
        except Exception:
            pass

    def _on_ping(self, sample):
        """Test amaçlı ping mesajlarına yanıt verir."""
        p = sample.payload.to_string()
        logger.info(f"🏓 [PING ALINDI]: {p}")
        if self.session:
            resp = {
                "status": "pong",
                "agent": self.vehicle_code,
                "msg": f"Hello from Docker Agent {self.vehicle_code}!",
                "timestamp": time.time()
            }
            self.session.put(f"pisim/{self.vehicle_code}/pong", json.dumps(resp))

    def _generate_new_waypoint(self, cur_lat: float, cur_lon: float) -> Tuple[float, float]:
        """Anlık konumun 15 ila 35 metre uzağında rastgele yeni bir hedef üretir."""
        self.wp_count += 1
        dist = random.uniform(18.0, 35.0)
        angle_rad = random.uniform(0, 2 * math.pi)
        dn = dist * math.cos(angle_rad)
        de = dist * math.sin(angle_rad)
        new_lat, new_lon = offset_gps_by_meters(cur_lat, cur_lon, dn, de)
        logger.info(f"🎯 [YENİ GÖREV #{self.wp_count}] Hedef Koordinat: ({new_lat:.6f}°N, {new_lon:.6f}°E) | Mesafe: {dist:.1f}m")
        return new_lat, new_lon

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
        return yaw_deg

    def _run_mission_loop(self):
        """Ana kontrol ve ArduPilot L1 navigasyon döngüsü (20 Hz)."""
        logger.info(f"🌀 {self.vehicle_code} Navigasyon Karar Döngüsü Başlatıldı.")
        step = 0
        last_log_time = 0.0

        # Eğer Unreal henüz başlamamışsa simüle edilmiş Ankara başlangıç konumu
        sim_lat = 39.9334
        sim_lon = 32.8597

        while self.running:
            now = time.time()
            has_real_sensors = (now - self.last_sensor_time) < 2.5

            if has_real_sensors and self.last_gps:
                cur_lat = float(self.last_gps.get("latitude", sim_lat))
                cur_lon = float(self.last_gps.get("longitude", sim_lon))
                cur_yaw = self._extract_yaw_deg()
            else:
                # Unreal açık değilse test amaçlı iç döngü
                cur_lat, cur_lon = sim_lat, sim_lon
                cur_yaw = 0.0

            # 1. Hedef Waypoint Yoksa veya Tamamlandıysa Yeni Hedef Belirle
            if self.current_wp is None:
                self.current_wp = self._generate_new_waypoint(cur_lat, cur_lon)

            # 2. Hedefe Mesafe ve Yön Hesabı
            target_lat, target_lon = self.current_wp
            dn, de, dist_m, target_bearing_deg = haversine_distance_and_bearing(
                cur_lat, cur_lon, target_lat, target_lon
            )

            # 3. Varış Kontrolü (WP Arrival)
            if dist_m <= self.wp_radius_m:
                logger.info(f"🏁 [HEDEF #{self.wp_count} TAMAMLANDI!] Hedefe ulaşıldı (Kalan: {dist_m:.2f}m).")
                self.current_wp = self._generate_new_waypoint(cur_lat, cur_lon)
                continue

            # 4. ARDUPILOT L1 NONLINEAR GUIDANCE HESABI
            # Yön farkı (Heading Error)
            heading_err = target_bearing_deg - cur_yaw
            while heading_err > 180.0: heading_err -= 360.0
            while heading_err < -180.0: heading_err += 360.0

            # L1 Lookahead mesafesi
            V = self.cruise_speed_mps
            L1_dist = max(2.5 * self.min_turn_radius_m, 3.0 * V)
            eta_rad = math.radians(heading_err)

            # ArduPilot İstenen Açısal Hız: omega = (2 * V / L1) * sin(eta)
            omega_cmd = (2.0 * V / L1_dist) * math.sin(eta_rad)

            # Geniş dönüş yarıçapı sınırlaması (|omega| <= V / r_min)
            omega_max = max(0.1, V / self.min_turn_radius_m)
            omega_cmd = max(-omega_max, min(omega_max, omega_cmd))

            # 5. Ackermann Direksiyon Açısı (Delta = atan(Wheelbase * Omega / V))
            wheelbase_m = self.hardware.wheelbase_m
            steer_rad = math.atan((wheelbase_m * omega_cmd) / max(0.2, V))
            steer_deg = math.degrees(steer_rad)
            steer_norm = max(-1.0, min(1.0, steer_deg / self.hardware.max_steer_deg))

            throttle_norm = 0.55 if abs(steer_norm) < 0.4 else 0.35

            # 6. Eyleyici Motor Değerlerini Hesapla (Ön Servolar + Tahrik Tekerlekleri)
            motors = self.hardware.compute_actuators(throttle=throttle_norm, steering=steer_norm)

            # 7. Zenoh Üzerinden Unreal'a Bas
            if self.session:
                # A. Aktüatör Topic'i (Unreal IPiSimActuatorInterface doğrudan sürüş)
                actuator_key = f"pisim/{self.vehicle_code}/actuators"
                self.session.put(actuator_key, json.dumps(motors))

                # B. ROS 2 geometry_msgs/msg/Twist cmd_vel Topic'i
                cmd_vel_key = f"pisim/{self.vehicle_code}/cmd_vel"
                twist_msg = {
                    "linear": {"x": float(throttle_norm * 2.5), "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": float(omega_cmd)}
                }
                self.session.put(cmd_vel_key, json.dumps(twist_msg))

                # C. Dağıtık Durum Mesh'i (In-Memory Fleet State)
                fleet_key = f"fleet/{self.vehicle_code}/state"
                state_data = {
                    "code": self.vehicle_code,
                    "connected": has_real_sensors,
                    "lat": cur_lat,
                    "lon": cur_lon,
                    "yaw": cur_yaw,
                    "target_dist_m": round(dist_m, 1),
                    "target_bearing": round(target_bearing_deg, 1),
                    "steer_deg": round(steer_deg, 1),
                    "throttle": throttle_norm,
                    "timestamp": now
                }
                self.session.put(fleet_key, json.dumps(state_data))

            # Düzenli Terminal Logu (Her 1.5 saniyede bir detaylı durum)
            if now - last_log_time >= 1.5:
                last_log_time = now
                steer_arrow = "◄" if steer_norm < -0.1 else ("►" if steer_norm > 0.1 else "▲")
                conn_icon = "🟢" if has_real_sensors else "🟡 (Simüle)"
                logger.info(
                    f"{conn_icon} Hedefe Kalan: {dist_m:5.1f}m | Pusula: {cur_yaw:+5.1f}° -> {target_bearing_deg:+5.1f}° | "
                    f"🛞 {steer_arrow} Direksiyon: {steer_deg:+4.1f}° | Gaz: %{int(throttle_norm*100)}"
                )

            step += 1
            time.sleep(0.05)  # 20 Hz

    def stop(self):
        self.running = False
        if self.session:
            try:
                # Motorları durdur
                stop_motors = self.hardware.compute_actuators(0.0, 0.0)
                self.session.put(f"pisim/{self.vehicle_code}/actuators", json.dumps(stop_motors))
                self.session.close()
            except Exception:
                pass
        logger.info(f"🛑 {self.vehicle_code} kapatıldı.")


def main():
    brain = AutonomousSwarmBrain(VEHICLE_CODE)

    def handle_sig(sig, frame):
        brain.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sig)
    signal.signal(signal.SIGTERM, handle_sig)
    brain.start()


if __name__ == "__main__":
    main()
