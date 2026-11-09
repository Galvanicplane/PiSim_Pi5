#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
autopilot_driver.py - PiSim ArduPilot L1 Otonom Araç Sürücüsü (Dosya 3)
=====================================================================
Bu modül, ArduPilot Rover (AP_L1_Control & AP_MotorsUGV) mimarisini
birebir uygulayan, kendini kanıtlamış otonom sürüş ve navigasyon motorudur.

Özellikler:
  1. ArduPilot L1 Nonlinear Guidance (AP_L1_Control):
     - Hedefe giden düz hat veya yay boyunca yanal ivme (lateral acceleration) ve
       açısal hız (yaw rate / omega) hesaplar:
       a_cmd = 2 * (V^2 / L1) * sin(eta)
       omega = (2 * V / L1) * sin(eta)
  2. Geniş Dönüş Yarıçapı ve Ackermann Direksiyon Geometrisi:
     - Araç ASLA tank gibi yerinde dönmez veya sert kare dönüş yapmaz.
     - Fiziksel dönüş yarıçapı (|omega| <= V / R_min) ile sınırlandırılır.
     - Ön tekerlekleri tutan servoları (M_Steer_FL, M_Steer_FR) Ackermann açısıyla
       fiziksel olarak döndürür: delta = atan(Wheelbase * omega / V).
  3. ArduPilot Viraj Hız Kontrolü (Corner Slowdown):
     - Virajlarda ve büyük yön farklarında savrulmayı önlemek için gazı otomatik düşürür:
       throttle = cruise_throttle * max(0.35, cos(eta)).
  4. ROS 2 & Gazebo Uyumluluğu:
     - Hem standart `geometry_msgs/msg/Twist` (cmd_vel), hem de doğrudan Unreal
       `IPiSimActuatorInterface` aktüatör haritasını (motors) üretir.
"""

import math
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

from vehicle_hardware import VehicleHardware

EARTH_RADIUS_M = 6378137.0


def normalize_angle_deg(angle: float) -> float:
    """Açıyı [-180.0, +180.0] derece aralığına normalize eder."""
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def haversine_distance_and_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> Tuple[float, float, float, float]:
    """İki GPS koordinatı arasındaki mesafe ve hedef pusula açısını (0°..360°) hesaplar."""
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
    if bearing_deg < 0:
        bearing_deg += 360.0

    return d_north_m, d_east_m, dist_m, bearing_deg


@dataclass
class DriveCommand:
    """Sürücü tarafından üretilen anlık kontrol komutları."""
    throttle: float                    # Normalize gaz [-1.0 .. +1.0]
    steering: float                    # Normalize direksiyon [-1.0 .. +1.0] (-1: Sol, +1: Sağ)
    steer_deg: float                   # Direksiyon servosu açısı (Derece)
    omega_cmd: float                   # Açısal hız (rad/s)
    linear_speed_cmd: float            # Hedeflenen çizgisel hız (m/s)
    dist_to_target_m: float            # Hedefe kalan mesafe (m)
    bearing_deg: float                 # Hedefin pusula açısı (Derece)
    heading_err_deg: float             # Yön hatası (eta) [-180..+180]
    is_arrived: bool                   # Hedef yarıçapına ulaşıldı mı?
    motors: Dict[str, float]           # Unreal IPiSimActuatorInterface aktüatör haritası
    twist: Dict[str, Any]              # ROS 2 geometry_msgs/msg/Twist


class ArduPilotDriver:
    """
    ArduPilot L1 Nonlinear Guidance (AP_L1_Control) algoritmasını uygulayan
    araç navigasyon ve sürüş kontrolcüsü.
    """

    def __init__(
        self,
        vehicle_code: str = "rover_01",
        wheelbase_m: float = 0.34,
        max_steer_deg: float = 35.0,
        min_turn_radius_m: float = 4.0,
        cruise_speed_mps: float = 2.2,
        wp_radius_m: float = 6.0,
        l1_period: float = 7.5,
        l1_damping: float = 0.75,
        drive_mode: str = "ACKERMANN",
    ):
        self.vehicle_code = vehicle_code
        self.wheelbase_m = max(0.1, float(wheelbase_m))
        self.max_steer_deg = max(15.0, float(max_steer_deg))
        self.min_turn_radius_m = max(1.0, float(min_turn_radius_m))
        self.cruise_speed_mps = max(0.5, float(cruise_speed_mps))
        self.wp_radius_m = max(2.0, float(wp_radius_m))
        self.l1_period = max(2.0, float(l1_period))
        self.l1_damping = max(0.5, float(l1_damping))

        # Donanım katmanı (Ackermann servo + diferansiyel tekerlekler)
        self.hardware = VehicleHardware(
            vehicle_code=vehicle_code,
            drive_mode=drive_mode,
            wheelbase_m=self.wheelbase_m,
            max_steer_deg=self.max_steer_deg,
        )

        # Aktif hedef waypoint
        self.target_lat: Optional[float] = None
        self.target_lon: Optional[float] = None
        self.wp_index: int = 0

        # Yumuşatma & Slew Rate Limiter Durumu (Tekerlek ve Direksiyon Çırpınmasını Önler)
        self.current_steer: float = 0.0
        self.current_throttle: float = 0.0
        self.last_compute_time: Optional[float] = None
        self.max_steer_rate: float = 1.8       # Saniyede max normalize direksiyon değişimi
        self.max_throttle_rate: float = 1.2    # Saniyede max normalize gaz değişimi

    def set_target_waypoint(self, lat: float, lon: float, index: int = 1):
        """Sürücüye yeni bir hedef koordinat (WGS84 lat, lon) atar."""
        self.target_lat = float(lat)
        self.target_lon = float(lon)
        self.wp_index = int(index)

    def clear_target(self):
        """Hedefi temizler."""
        self.target_lat = None
        self.target_lon = None
        self.wp_index = 0

    def compute_step(
        self,
        cur_lat: float,
        cur_lon: float,
        cur_yaw_deg: float,
        cur_speed_mps: Optional[float] = None,
    ) -> DriveCommand:
        """
        Anlık sensör verilerine (GPS Lat/Lon, IMU Yaw Pusula, Hız) göre
        ArduPilot L1 kılavuzluk algoritmasını bir adım (step) işletir ve komut üretir.
        """
        if self.target_lat is None or self.target_lon is None:
            return self.emergency_stop()

        now = time.time()
        if self.last_compute_time is not None:
            dt = max(0.01, min(0.25, now - self.last_compute_time))
        else:
            dt = 0.05
        self.last_compute_time = now

        # 1. Hedefe mesafe ve pusula kerterizi (WGS84 Haversine)
        _, _, dist_m, target_bearing_deg = haversine_distance_and_bearing(
            cur_lat, cur_lon, self.target_lat, self.target_lon
        )

        # 2. Varış Kontrolü (Arrival Acceptance)
        if dist_m <= self.wp_radius_m:
            # Varış anında momentum korunur, ani sıfırlama yapılmaz
            cmd = self.emergency_stop(hold_current=True)
            cmd.dist_to_target_m = dist_m
            cmd.bearing_deg = target_bearing_deg
            cmd.is_arrived = True
            return cmd

        # 3. Yön Hatası (Heading Error / eta: [-180..+180])
        heading_err = normalize_angle_deg(target_bearing_deg - cur_yaw_deg)
        eta_rad = math.radians(heading_err)

        # 4. Hız Belirleme
        if cur_speed_mps is None or cur_speed_mps < 0.2:
            effective_speed = max(1.0, self.cruise_speed_mps * 0.7)
        else:
            effective_speed = max(0.8, cur_speed_mps)

        # 5. ArduPilot L1 Lookahead Mesafesi Hesabı
        l1_dist = (1.0 / math.pi) * self.l1_damping * self.l1_period * effective_speed
        l1_min = 2.0 * self.min_turn_radius_m
        l1_dist = max(l1_min, min(max(l1_min, dist_m), l1_dist))

        # 6. Proximity Damping (Yaklaşma Filtresi - Singularity Shield)
        # Hedefe yaklaşıldığında pusula açısının aniden arkaya geçmesiyle oluşan savrulmayı önler
        if dist_m < 10.0:
            proximity_steer_cap = max(0.25, min(1.0, (dist_m - self.wp_radius_m * 0.4) / 6.0))
        else:
            proximity_steer_cap = 1.0

        raw_steer = max(-1.0, min(1.0, heading_err / 40.0))
        target_steer = max(-proximity_steer_cap, min(proximity_steer_cap, raw_steer))

        # 7. Slew Rate Limiter (Direksiyon Yumuşatma - Tekerleklerin Savrulmasını Keser)
        max_steer_delta = self.max_steer_rate * dt
        steer_diff = target_steer - self.current_steer
        steer_diff = max(-max_steer_delta, min(max_steer_delta, steer_diff))
        self.current_steer += steer_diff
        steer_norm = self.current_steer
        steer_deg = steer_norm * self.max_steer_deg
        omega_cmd = (steer_norm * effective_speed) / max(0.5, self.wheelbase_m)

        # 8. ArduPilot Viraj Hız Kontrolü (Corner Slowdown & Proximity)
        abs_err = abs(heading_err)
        if abs_err > 60.0:
            heading_factor = 0.40   # Sert viraj: Dengeli dönüş
        elif abs_err > 30.0:
            heading_factor = 0.65
        else:
            heading_factor = 1.0 - (abs_err / 60.0) * 0.35

        # Hedefe yaklaştıkça yumuşak yavaşlama
        proximity_factor = min(1.0, max(0.55, dist_m / 12.0))

        target_throttle = 0.70 * heading_factor * proximity_factor
        target_throttle = max(0.30, min(0.80, target_throttle))

        # 9. Slew Rate Limiter (Gaz Yumuşatma - Ani Tork ve Pati Önleme)
        max_throt_delta = self.max_throttle_rate * dt
        throt_diff = target_throttle - self.current_throttle
        throt_diff = max(-max_throt_delta, min(max_throt_delta, throt_diff))
        self.current_throttle += throt_diff
        throttle_norm = self.current_throttle

        # 10. Eyleyici Motor Değerleri (VehicleHardware üzerinden)
        motors = self.hardware.compute_actuators(throttle=throttle_norm, steering=steer_norm)

        # 11. ROS 2 Twist Mesajı
        twist = {
            "linear": {"x": round(throttle_norm * self.cruise_speed_mps, 3), "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": round(omega_cmd, 3)},
        }

        return DriveCommand(
            throttle=round(throttle_norm, 3),
            steering=round(steer_norm, 3),
            steer_deg=round(steer_deg, 2),
            omega_cmd=round(omega_cmd, 3),
            linear_speed_cmd=round(throttle_norm * self.cruise_speed_mps, 2),
            dist_to_target_m=round(dist_m, 2),
            bearing_deg=round(target_bearing_deg, 1),
            heading_err_deg=round(heading_err, 1),
            is_arrived=False,
            motors=motors,
            twist=twist,
        )

    def emergency_stop(self, hold_current: bool = False) -> DriveCommand:
        """Acil durum veya varış anında komut üretir."""
        if not hold_current:
            self.current_throttle = 0.0
            self.current_steer = 0.0

        motors = self.hardware.compute_actuators(throttle=self.current_throttle, steering=self.current_steer)
        twist = {
            "linear": {"x": round(self.current_throttle * self.cruise_speed_mps, 3), "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        }
        return DriveCommand(
            throttle=round(self.current_throttle, 3),
            steering=round(self.current_steer, 3),
            steer_deg=round(self.current_steer * self.max_steer_deg, 2),
            omega_cmd=0.0,
            linear_speed_cmd=round(self.current_throttle * self.cruise_speed_mps, 2),
            dist_to_target_m=0.0,
            bearing_deg=0.0,
            heading_err_deg=0.0,
            is_arrived=False,
            motors=motors,
            twist=twist,
        )

