"""
vehicle_hardware.py
PiSim - Araç Donanım ve İskelet Soyutlama Katmanı (Hardware Abstraction Layer - HAL)

Bu modül, araç kemik yapısını (Skeleton/Bones) ve motor rollerini (Wheel, Servo) tanır.
Üst seviye kontrolcülerden ("ileri git", "sağa şu radius ile dön") gelen komutları
fiziksel aktüatör haritasına dönüştürür.

Özellikler:
  - Sürüş Tipi: ACKERMANN (Ön direksiyon servoları döner, tekerlekler ileri çeker)
                veya DIFFERENTIAL (Tank modu: servolar 0°, tekerlekler zıt devir döner).
  - Kullanıcı istediği zaman sürüş tipini değiştirebilir.
  - JSON profilinden veya telemetriden gelen kemik listesinden otomatik rol algılar.
  - Unreal Engine'e (Gazebo gibi saf fizik motoruna) doğrudan aktüatör komutları üretir.
"""

import json
import math
import os
from typing import Dict, List, Optional, Tuple


class VehicleHardware:
    """
    Bir PiSim aracının iskelet, direksiyon servoları ve tekerlek donanımını yöneten sınıf.
    """

    def __init__(
        self,
        vehicle_code: str = "rover_01",
        drive_mode: str = "ACKERMANN",
        wheelbase_m: float = 0.34,
        track_width_m: float = 0.32,
        wheel_radius_m: float = 0.18,
        max_steer_deg: float = 35.0,
    ):
        self.vehicle_code = vehicle_code
        self.drive_mode = drive_mode.upper()  # 'ACKERMANN' veya 'DIFFERENTIAL'
        self.wheelbase_m = wheelbase_m
        self.track_width_m = track_width_m
        self.wheel_radius_m = wheel_radius_m
        self.max_steer_deg = max_steer_deg

        # Tespit edilmiş veya tanımlanmış kemik isimleri
        self.steer_servos: List[str] = ["M_Steer_FL", "M_Steer_FR"]
        self.front_wheels_left: List[str] = ["M_Wheel_FL"]
        self.front_wheels_right: List[str] = ["M_Wheel_FR"]
        self.rear_wheels_left: List[str] = ["M_Wheel_RL"]
        self.rear_wheels_right: List[str] = ["M_Wheel_RR"]

    def set_drive_mode(self, mode: str):
        """Sürüş tipini değiştirir: 'ACKERMANN' veya 'DIFFERENTIAL'."""
        mode_upper = mode.upper()
        if mode_upper in ("ACKERMANN", "DIFFERENTIAL"):
            self.drive_mode = mode_upper
            print(f"[HARDWARE] Sürüş Modu Güncellendi: {self.drive_mode}")
        else:
            print(f"[UYARI] Bilinmeyen sürüş modu: {mode}. 'ACKERMANN' veya 'DIFFERENTIAL' olmalı.")

    def auto_detect_from_bones(self, bone_names: List[str]):
        """
        Unreal'dan veya profilden gelen kemik listesini analiz ederek
        hangi parçanın direksiyon servosu, hangi parçanın tekerlek olduğunu otomatik belirler.
        (Unreal Baker.cpp / MotorConfigurator ile aynı mantıkta çalışır).
        """
        steers = []
        fw_l, fw_r = [], []
        rw_l, rw_r = [], []

        for b in bone_names:
            lower = b.lower()
            is_steer = any(k in lower for k in ("steer", "servo", "knuckle", "yon", "joint"))
            is_wheel = any(k in lower for k in ("wheel", "teker", "tyre", "caster"))

            if is_steer:
                steers.append(b)
            elif is_wheel:
                is_front = any(k in lower for k in ("fl", "fr", "front", "on"))
                is_left = any(k in lower for k in ("fl", "rl", "_l", "left", "sol"))
                
                if is_front:
                    if is_left:
                        fw_l.append(b)
                    else:
                        fw_r.append(b)
                else:
                    if is_left:
                        rw_l.append(b)
                    else:
                        rw_r.append(b)

        if steers:
            self.steer_servos = steers
        if fw_l:
            self.front_wheels_left = fw_l
        if fw_r:
            self.front_wheels_right = fw_r
        if rw_l:
            self.rear_wheels_left = rw_l
        if rw_r:
            self.rear_wheels_right = rw_r

        # Eğer direksiyon servosu varsa varsayılan ACKERMANN, yoksa DIFFERENTIAL
        if self.steer_servos:
            self.drive_mode = "ACKERMANN"
        else:
            self.drive_mode = "DIFFERENTIAL"

    def load_from_profile(self, profile_dict: dict):
        """JSON araç profilinden donanım yapılandırmasını yükler."""
        if "DriveMode" in profile_dict:
            self.drive_mode = str(profile_dict["DriveMode"]).upper()

        modules = profile_dict.get("Modules", [])
        bones = [m.get("BoneName", "") for m in modules if m.get("BoneName")]
        if bones:
            self.auto_detect_from_bones(bones)

    def compute_actuators(
        self,
        throttle: float,
        steering: float,
        turn_radius_m: Optional[float] = None,
    ) -> Dict[str, float]:
        """
        Üst seviye gaz (throttle) ve direksiyon (steering) girdilerini
        aracın model kemiklerine atanacak eyleyici değerlerine (-1.0 .. +1.0) dönüştürür.

        Parametreler:
          - throttle: İleri/Geri sürüş gücü [-1.0 .. +1.0]
          - steering: Direksiyon açısı [-1.0 .. +1.0] (-1.0: Tam Sol, +1.0: Tam Sağ)
          - turn_radius_m: İsteğe bağlı belirli bir yarıçap ile dönüş

        Dönüş:
          {"M_Steer_FL": 0.55, "M_Steer_FR": 0.55, "M_Wheel_FL": 0.7, ...}
        """
        throttle = max(-1.0, min(1.0, float(throttle)))
        steering = max(-1.0, min(1.0, float(steering)))

        motors: Dict[str, float] = {}

        if self.drive_mode == "ACKERMANN":
            # =================================================================
            # ACKERMANN SÜRÜŞÜ:
            # - Ön direksiyon servoları fiziksel olarak döner!
            # - Tekerlekler İLERİ akar, asla diferansiyel zıt dönmez!
            # =================================================================
            if turn_radius_m is not None and turn_radius_m > 0.05:
                # Belirli bir yarıçap ile dönüş: delta = atan(L / R)
                delta_rad = math.atan(self.wheelbase_m / turn_radius_m)
                steer_deg = math.degrees(delta_rad) * (1.0 if steering >= 0 else -1.0)
            else:
                steer_deg = steering * self.max_steer_deg

            steer_deg = max(-self.max_steer_deg, min(self.max_steer_deg, steer_deg))
            # Unreal'da -1.0..+1.0 aralığı 45 dereceye karşılık gelir:
            steer_norm = round(steer_deg / 45.0, 4)

            # 1. Direksiyon Servolarını Açılı Olarak Döndür
            for servo in self.steer_servos:
                motors[servo] = steer_norm

            # 2. Tekerlek Devirleri (Ackermann yumuşak diferansiyel akışı)
            # İki taraf da aynı yönde ileri döner, dış tekerlek hafifçe hızlı akar
            if abs(steering) > 0.05 and abs(throttle) > 0.01:
                # Yumuşak hız farkı: Aşırı tekerlek kayması ve çatışmasını önlemek için %10 ile sınırlı
                diff_factor = 0.10 * abs(steering)
                if steering > 0:  # Sağa dönüş
                    v_left = throttle * (1.0 + diff_factor)
                    v_right = throttle * (1.0 - diff_factor)
                else:  # Sola dönüş
                    v_left = throttle * (1.0 - diff_factor)
                    v_right = throttle * (1.0 + diff_factor)
            else:
                v_left = throttle
                v_right = throttle

            v_left = round(max(-1.0, min(1.0, v_left)), 4)
            v_right = round(max(-1.0, min(1.0, v_right)), 4)

            for w in self.front_wheels_left + self.rear_wheels_left:
                motors[w] = v_left
            for w in self.front_wheels_right + self.rear_wheels_right:
                motors[w] = v_right

        else:
            # =================================================================
            # DİFERANSİYEL (TANK) SÜRÜŞÜ:
            # - Direksiyon servoları 0°'de kilitli kalır.
            # - Tekerlekler sağ/sol zıt devir farkı ile aracı döndürür.
            # =================================================================
            for servo in self.steer_servos:
                motors[servo] = 0.0

            v_left = max(-1.0, min(1.0, throttle - (steering * 0.7)))
            v_right = max(-1.0, min(1.0, throttle + (steering * 0.7)))

            v_left = round(v_left, 4)
            v_right = round(v_right, 4)

            for w in self.front_wheels_left + self.rear_wheels_left:
                motors[w] = v_left
            for w in self.front_wheels_right + self.rear_wheels_right:
                motors[w] = v_right

        return motors

    def turn_with_radius(
        self, speed: float, radius_m: float, direction: str = "right"
    ) -> Dict[str, float]:
        """
        Aracın tam olarak istenen yarıçapta (radius_m) dönmesini sağlayan aktüatör haritasını üretir.
        """
        steer_sign = 1.0 if direction.lower() in ("right", "sag", "r") else -1.0
        return self.compute_actuators(
            throttle=speed, steering=steer_sign, turn_radius_m=radius_m
        )
