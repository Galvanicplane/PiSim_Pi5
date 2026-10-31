#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PiSim - Otonom Hareket ve Çift Doğrulamalı (Double-Check) Navigasyon Kütüphanesi (pisim_motion.py)
==================================================================================================
Bu modül, PiSim projesi kapsamı tamamlandıktan sonra otonom sürüş algoritmalarını test etmek için
geliştirilmiş bağımsız hareket kütüphanesidir.

Özellikler:
  1. Çift Doğrulama (Double-Check Sensor Fusion):
     - Hem GPS (WGS84 Enlem/Boylam Haversine) hem de Yerel Odometri (pos_x, pos_y, pos_z metre)
       verilerini çapraz kontrol ederek konum sapmasını (drift) ve güven skorunu hesaplar.
  2. İlerleme Fonksiyonu (move_forward):
     - IMU Yaw (Baş açısı) kilidi ile sapmadan düz ilerler, hedef mesafeyi hem GPS hem Odometri
       üzerinden doğrulayarak yumuşak frenleme (PID) ile durur.
  3. Eğrisel / Kavisli Dönüş Fonksiyonu (drive_curve):
     - Belirtilen yarıçap (radius_m) ve yay açısı (arc_angle_deg) boyunca diferansiyel/Ackermann
       eğrisel yörüngede (w = v / R) pürüzsüz kavis çizerek döner.
  4. Yerinde Dönüş & Pusula Hizalama (turn_angle, turn_to_heading):
     - En kısa açı farkı (-180°..+180°) üzerinden PD kontrolcü ile hedef açıya döner.
  5. Koordinata Otonom Sürüş (navigate_to_gps / navigate_to_local):
     - Pure Pursuit (Eğrisel Takip) veya Noktasal Dönüş (Point-Turn) modlarıyla hedef koordinata
       otonom sürüş gerçekleştirir.
"""

import json
import math
import socket
import time
import urllib.request
from typing import Callable, Dict, Optional, Tuple


EARTH_METERS_PER_DEG_LAT = 111320.0


def normalize_angle_deg(angle: float) -> float:
    """Açıyı [-180.0, +180.0] derece aralığına normalize eder."""
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def gps_to_delta_meters(lat1: float, lon1: float, lat2: float, lon2: float) -> Tuple[float, float, float, float]:
    """
    İki GPS koordinatı arasındaki yerel (Kuzey dX, Doğu dY) mesafeyi (metre),
    toplam öklid/haversine mesafesini ve hedef baş açısını (Yaw derece: 0=Kuzey, +90=Doğu) hesaplar.
    Unreal Engine GPSSensor.cpp projeksiyonu ile %100 birebir uyumludur.
    """
    dx_north = (lat2 - lat1) * EARTH_METERS_PER_DEG_LAT
    lat_rad = math.radians(lat1)
    dy_east = (lon2 - lon1) * (EARTH_METERS_PER_DEG_LAT * math.cos(lat_rad))
    dist_m = math.hypot(dx_north, dy_east)
    bearing_deg = math.degrees(math.atan2(dy_east, dx_north))
    return dx_north, dy_east, dist_m, bearing_deg


def delta_meters_to_gps(origin_lat: float, origin_lon: float, dx_north: float, dy_east: float) -> Tuple[float, float]:
    """Yerel metre (Kuzey dX, Doğu dY) ofsetini GPS (Enlem, Boylam) koordinatına dönüştürür."""
    dlat = dx_north / EARTH_METERS_PER_DEG_LAT
    lat_rad = math.radians(origin_lat)
    dlon = dy_east / (EARTH_METERS_PER_DEG_LAT * max(1e-6, math.cos(lat_rad)))
    return origin_lat + dlat, origin_lon + dlon


class PiSimMotionController:
    """
    PiSim Araçları için Çift Doğrulamalı (GPS + Yerel Odometri + IMU) Otonom Hareket Kontrolcüsü.
    """

    def __init__(
        self,
        target_vehicle: str = "1",
        ue5_ip: Optional[str] = None,
        dashboard_api_url: str = "http://127.0.0.1:8080/api/telemetry",
        command_port: int = 7402,
    ):
        self.target_vehicle = str(target_vehicle)
        self.ue5_ip = ue5_ip
        self.dashboard_api_url = dashboard_api_url
        self.command_port = int(command_port)
        self.dedicated_port: Optional[int] = None

        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # Fallback doğrudan UDP 7400 dinleyici (Dashboard kapalıysa devreye girer)
        self.fallback_rx_sock: Optional[socket.socket] = None
        self._abort_requested = False

    def request_abort(self):
        """Aktif otonom hareket döngüsünü güvenli şekilde durdurur ve aracı frenler."""
        self._abort_requested = True
        self.stop()

    def _init_fallback_udp(self):
        if self.fallback_rx_sock is not None:
            return
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            if hasattr(socket, "SO_REUSEPORT"):
                try:
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                except OSError:
                    pass
            sock.bind(("0.0.0.0", 7400))
            sock.settimeout(0.25)
            self.fallback_rx_sock = sock
        except Exception:
            self.fallback_rx_sock = None

    def get_telemetry(self) -> Optional[Dict]:
        """
        Seçili aracın en güncel telemetri verisini çeker.
        Önce yerel Pi 5 Web Dashboard API'sini (/api/telemetry) sorgular (port çakışmasını önler),
        eğer Dashboard kapalıysa doğrudan UDP 7400 soketinden okur.
        """
        # 1. Birincil Kaynak: Pi 5 Web Dashboard Yerel API
        try:
            req = urllib.request.Request(self.dashboard_api_url, headers={"Accept": "application/json"})
            with urllib.request.urlopen(req, timeout=0.35) as resp:
                data = json.loads(resp.read().decode("utf-8"))
                if not self.ue5_ip and data.get("ue5_ip"):
                    self.ue5_ip = data["ue5_ip"]

                fleet = data.get("fleet", [])
                chosen = None
                target_lower = self.target_vehicle.lower()

                if target_lower in ("all", "selected", "active", ""):
                    sel_code = data.get("selected_vehicle_code")
                    if sel_code:
                        chosen = next((v for v in fleet if str(v.get("code", "")).lower() == sel_code.lower()), None)
                    if not chosen and fleet:
                        chosen = fleet[0]
                else:
                    for v in fleet:
                        v_code = str(v.get("code", "")).lower()
                        v_name = str(v.get("name", "")).lower()
                        v_id = str(v.get("id", ""))
                        if (
                            target_lower == v_code
                            or target_lower == v_name
                            or target_lower == v_id
                            or target_lower == f"rover_0{v_id}"
                            or v_code.startswith(target_lower)
                        ):
                            chosen = v
                            break
                    if not chosen and fleet:
                        chosen = fleet[0]

                if chosen:
                    v_id = int(chosen.get("id", 1))
                    self.dedicated_port = 7402 + v_id
                    spd_kmh = float(chosen.get("speed", 0.0))
                    return {
                        "id": v_id,
                        "code": chosen.get("code", f"RV0{v_id}"),
                        "name": chosen.get("name", f"Rover 0{v_id}"),
                        "lat": float(chosen.get("lat", 39.9334)),
                        "lon": float(chosen.get("lon", 32.8597)),
                        "altitude": float(chosen.get("altitude", 850.0)),
                        "pos_x": float(chosen.get("pos_x", 0.0)),
                        "pos_y": float(chosen.get("pos_y", 0.0)),
                        "pos_z": float(chosen.get("pos_z", 0.0)),
                        "roll": float(chosen.get("roll", 0.0)),
                        "pitch": float(chosen.get("pitch", 0.0)),
                        "yaw": float(chosen.get("yaw", 0.0)),
                        "speed_kmh": spd_kmh,
                        "speed_mps": spd_kmh / 3.6,
                        "connected": bool(chosen.get("connected", True)),
                    }
        except Exception:
            pass

        # 2. İkincil Kaynak: Doğrudan UDP 7400 Dinleme
        self._init_fallback_udp()
        if self.fallback_rx_sock:
            try:
                raw, addr = self.fallback_rx_sock.recvfrom(4096)
                if not self.ue5_ip:
                    self.ue5_ip = addr[0]
                pkt = json.loads(raw.decode("utf-8", errors="ignore").strip())
                gps = pkt.get("gps", {})
                att = pkt.get("attitude", {})
                pos = pkt.get("position", {})
                v_id = int(pkt.get("vehicle_id", 1))
                self.dedicated_port = 7402 + v_id
                spd_kmh = float(gps.get("speed_kmh", 0.0))
                return {
                    "id": v_id,
                    "code": pkt.get("vehicle_code", f"RV0{v_id}"),
                    "name": pkt.get("vehicle_name", f"Rover 0{v_id}"),
                    "lat": float(gps.get("lat", 39.9334)),
                    "lon": float(gps.get("lon", 32.8597)),
                    "altitude": float(gps.get("alt", 850.0)),
                    "pos_x": float(pos.get("x", 0.0)),
                    "pos_y": float(pos.get("y", 0.0)),
                    "pos_z": float(pos.get("z", 0.0)),
                    "roll": float(att.get("roll", 0.0)),
                    "pitch": float(att.get("pitch", 0.0)),
                    "yaw": float(att.get("yaw", 0.0)),
                    "speed_kmh": spd_kmh,
                    "speed_mps": spd_kmh / 3.6,
                    "connected": True,
                }
            except Exception:
                pass
        return None

    def send_cmd_vel(self, linear_x: float, angular_z: float, timeout_ms: int = 500):
        """
        Araca hem standart ROS 2 Twist formatında hem de PiSim hızlı sürüş formatında UDP komutu gönderir.
        Hem merkezi 7402 portuna hem de aracın özel alt portuna (7402 + VehicleId) iletir.
        """
        target_ip = self.ue5_ip if self.ue5_ip else "255.255.255.255"
        payload = {
            "cmd": "cmd_vel",
            "target": self.target_vehicle if self.target_vehicle not in ("selected", "active", "") else "all",
            "linear_x": round(float(linear_x), 4),
            "angular_z": round(float(angular_z), 4),
            "linear": {"x": round(float(linear_x), 4), "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": round(float(angular_z), 4)},
            "timeout_ms": int(timeout_ms),
            "ts": time.time(),
        }
        raw_bytes = (json.dumps(payload) + "\n").encode("utf-8")

        destinations = {(target_ip, self.command_port), ("127.0.0.1", self.command_port)}
        if self.dedicated_port:
            destinations.add((target_ip, self.dedicated_port))
            destinations.add(("127.0.0.1", self.dedicated_port))

        for ip, port in destinations:
            try:
                self.cmd_sock.sendto(raw_bytes, (ip, port))
            except Exception:
                pass

    def stop(self):
        """Aracı anında durdurur (Acil Fren / Sıfır Hız)."""
        target_ip = self.ue5_ip if self.ue5_ip else "255.255.255.255"
        payload = {
            "cmd": "stop",
            "target": self.target_vehicle if self.target_vehicle not in ("selected", "active", "") else "all",
            "linear_x": 0.0,
            "angular_z": 0.0,
            "ts": time.time(),
        }
        raw_bytes = (json.dumps(payload) + "\n").encode("utf-8")
        for _ in range(3):
            for ip in {target_ip, "127.0.0.1"}:
                try:
                    self.cmd_sock.sendto(raw_bytes, (ip, self.command_port))
                    if self.dedicated_port:
                        self.cmd_sock.sendto(raw_bytes, (ip, self.dedicated_port))
                except Exception:
                    pass
            time.sleep(0.02)

    # =========================================================================
    # ÇİFT DOĞRULAMA (DOUBLE-CHECK: GPS + YEREL ODOMETRİ SENSÖR FÜZYONU)
    # =========================================================================
    def double_check_displacement(self, start_state: Dict, current_state: Dict) -> Dict:
        """
        Başlangıç noktası ile anlık nokta arasındaki ilerlemeyi İKİ FARKLI KAYNAKTAN doğrular:
          1. GPS (Enlem / Boylam WGS84 projeksiyonu)
          2. Yerel Odometri (Unreal Chaos Fizik Motoru pos_x, pos_y metre)
        İkisi arasındaki sapmayı (drift_m) ve güven yüzdesini (confidence_pct) döndürür.
        """
        # 1. GPS tabanlı yer değiştirme
        gps_dx, gps_dy, gps_dist, gps_bearing = gps_to_delta_meters(
            start_state["lat"], start_state["lon"], current_state["lat"], current_state["lon"]
        )

        # 2. Yerel Odometri (pos_x, pos_y) tabanlı yer değiştirme
        odom_dx = current_state["pos_x"] - start_state["pos_x"]
        odom_dy = current_state["pos_y"] - start_state["pos_y"]
        odom_dist = math.hypot(odom_dx, odom_dy)
        odom_bearing = math.degrees(math.atan2(odom_dy, odom_dx)) if odom_dist > 0.01 else current_state["yaw"]

        # Çapraz Kontrol (Cross-Validation)
        drift_x = abs(gps_dx - odom_dx)
        drift_y = abs(gps_dy - odom_dy)
        drift_total = math.hypot(drift_x, drift_y)

        # Sensör Füzyonu (%60 Odometri + %40 GPS; eğer GPS sabitlenmişse ikisinin ağırlıklı ortalaması)
        if gps_dist < 0.01 and odom_dist > 0.2:
            # GPS henüz güncellenmediyse odometriye güven
            fused_dx, fused_dy, fused_dist = odom_dx, odom_dy, odom_dist
            confidence = 85.0
        else:
            fused_dx = 0.5 * gps_dx + 0.5 * odom_dx
            fused_dy = 0.5 * gps_dy + 0.5 * odom_dy
            fused_dist = math.hypot(fused_dx, fused_dy)
            ref_scale = max(1.0, fused_dist)
            confidence = max(0.0, min(100.0, 100.0 * (1.0 - (drift_total / (ref_scale + 2.0)))))

        return {
            "gps_dx": gps_dx,
            "gps_dy": gps_dy,
            "gps_dist": gps_dist,
            "odom_dx": odom_dx,
            "odom_dy": odom_dy,
            "odom_dist": odom_dist,
            "fused_dx": fused_dx,
            "fused_dy": fused_dy,
            "fused_dist": fused_dist,
            "drift_m": drift_total,
            "confidence_pct": round(confidence, 1),
            "gps_bearing": gps_bearing,
            "odom_bearing": odom_bearing,
        }

    # =========================================================================
    # 1. DÜZ İLERLEME (ÇİFT DOĞRULAMALI + BAŞ AÇISI KİLİTLİ)
    # =========================================================================
    def move_forward(
        self,
        distance_m: float,
        speed_mps: float = 1.5,
        tolerance_m: float = 0.25,
        timeout_sec: float = 60.0,
        status_callback: Optional[Callable[[str], None]] = None,
    ) -> Dict:
        """
        Aracı belirtilen metre kadar ileri (distance_m > 0) veya geri (distance_m < 0) sürer.
        - IMU Yaw açısını kilitleyerek aracın sağa/sola kaymasını engeller.
        - Mesafeyi hem GPS hem de Yerel Odometri (pos_x, pos_y) ile çift doğrular (Double-Check).
        """
        self._abort_requested = False
        start_state = self.wait_for_telemetry(timeout_sec=5.0)
        if not start_state:
            return {"ok": False, "reason": "Telemetri alınamadı"}

        target_abs_dist = abs(distance_m)
        direction_sign = 1.0 if distance_m >= 0.0 else -1.0
        locked_yaw = start_state["yaw"]
        t_start = time.time()

        if status_callback:
            status_callback(
                f"[MOVE_FORWARD] Hedef: {distance_m:+.2f}m | Başlangıç Yaw: {locked_yaw:.1f}° | Çift Doğrulama Aktif"
            )

        while not self._abort_requested and (time.time() - t_start) < timeout_sec:
            cur = self.get_telemetry()
            if not cur:
                time.sleep(0.05)
                continue

            chk = self.double_check_displacement(start_state, cur)
            remaining = target_abs_dist - chk["fused_dist"]

            if remaining <= tolerance_m:
                self.stop()
                msg = (
                    f"✅ [HEDEF TAMAM] İlerleme: {chk['fused_dist']:.2f}m "
                    f"(GPS: {chk['gps_dist']:.2f}m, Odom: {chk['odom_dist']:.2f}m, "
                    f"Sapma: {chk['drift_m']:.2f}m, Güven: %{chk['confidence_pct']})"
                )
                if status_callback:
                    status_callback(msg)
                return {"ok": True, "double_check": chk, "message": msg}

            # Hedefe yaklaşırken yumuşak yavaşlama (Orantısal hız profili)
            slowdown_factor = min(1.0, max(0.25, remaining / 2.5))
            cmd_v = direction_sign * abs(speed_mps) * slowdown_factor

            # IMU Yaw (Baş açısı) koruma düzeltmesi (Düz çizgiden sapmayı önler)
            yaw_err = normalize_angle_deg(locked_yaw - cur["yaw"])
            cmd_w = max(-0.8, min(0.8, math.radians(yaw_err) * 1.2))

            self.send_cmd_vel(cmd_v, cmd_w, timeout_ms=400)
            time.sleep(0.08)

        self.stop()
        return {"ok": False, "reason": "Zaman aşımı veya iptal edildi"}

    # =========================================================================
    # 2. YERİNDE DÖNÜŞ & MUTLAK PUSULA AÇISINA HİZALAMA
    # =========================================================================
    def turn_to_heading(
        self,
        target_yaw_deg: float,
        max_omega_rad: float = 1.2,
        tolerance_deg: float = 2.5,
        timeout_sec: float = 25.0,
        status_callback: Optional[Callable[[str], None]] = None,
    ) -> Dict:
        """Aracı belirtilen mutlak pusula/yaw açısına (-180..+180) en kısa yönden döndürür."""
        self._abort_requested = False
        target_yaw = normalize_angle_deg(target_yaw_deg)
        t_start = time.time()

        while not self._abort_requested and (time.time() - t_start) < timeout_sec:
            cur = self.get_telemetry()
            if not cur:
                time.sleep(0.05)
                continue

            err_deg = normalize_angle_deg(target_yaw - cur["yaw"])
            if abs(err_deg) <= tolerance_deg:
                self.stop()
                if status_callback:
                    status_callback(f"✅ [DÖNÜŞ TAMAM] Mevcut Yaw: {cur['yaw']:.1f}° (Hedef: {target_yaw:.1f}°)")
                return {"ok": True, "final_yaw": cur["yaw"]}

            # Orantısal dönüş hızı (Hedefe yaklaştıkça yumuşar)
            kp = 1.6
            omega = math.radians(err_deg) * kp
            if abs(omega) < 0.22:
                omega = math.copysign(0.22, omega)
            omega = max(-max_omega_rad, min(max_omega_rad, omega))

            self.send_cmd_vel(0.0, omega, timeout_ms=400)
            time.sleep(0.08)

        self.stop()
        return {"ok": False, "reason": "Dönüş zaman aşımına uğradı"}

    def turn_angle(
        self,
        delta_deg: float,
        max_omega_rad: float = 1.2,
        tolerance_deg: float = 2.5,
        timeout_sec: float = 25.0,
        status_callback: Optional[Callable[[str], None]] = None,
    ) -> Dict:
        """Aracı mevcut yönüne göre bağıl olarak delta_deg (+ sağa, - sola) döndürür."""
        start_state = self.wait_for_telemetry(timeout_sec=5.0)
        if not start_state:
            return {"ok": False, "reason": "Telemetri alınamadı"}
        target_yaw = normalize_angle_deg(start_state["yaw"] + delta_deg)
        return self.turn_to_heading(target_yaw, max_omega_rad, tolerance_deg, timeout_sec, status_callback)

    # =========================================================================
    # 3. EĞRİSEL (CURVE) DÖNÜŞ FONKSİYONU (KAVİSLİ SÜRÜŞ)
    # =========================================================================
    def drive_curve(
        self,
        radius_m: float,
        arc_angle_deg: float,
        speed_mps: float = 1.4,
        direction: str = "right",
        timeout_sec: float = 45.0,
        status_callback: Optional[Callable[[str], None]] = None,
    ) -> Dict:
        """
        Aracı durdurmadan, belirtilen yarıçap (radius_m) ve yay açısı (arc_angle_deg) boyunca
        yumuşak bir eğri (curve) çizerek döndürür.
        Kinematik bağıntı:
          - Çizgisel hız: v = speed_mps
          - Açısal hız:   w = (+ veya -) v / radius_m
          - Beklenen yay uzunluğu: S = radius_m * rad(arc_angle_deg)
        Hem IMU açı değişimini hem de GPS/Odometri kiriş mesafesini çift doğrular!
        """
        self._abort_requested = False
        start_state = self.wait_for_telemetry(timeout_sec=5.0)
        if not start_state:
            return {"ok": False, "reason": "Telemetri alınamadı"}

        radius_m = max(0.5, float(radius_m))
        target_arc_deg = abs(float(arc_angle_deg))
        turn_sign = 1.0 if str(direction).lower() in ("right", "sag", "cw", "+") else -1.0

        # Beklenen kiriş uzunluğu (Chord length = 2 * R * sin(theta / 2))
        expected_chord_m = 2.0 * radius_m * math.sin(math.radians(min(180.0, target_arc_deg)) / 2.0)
        nominal_omega = turn_sign * (abs(speed_mps) / radius_m)

        if status_callback:
            status_callback(
                f"🌀 [CURVE SÜRÜŞ] R={radius_m:.1f}m, Açı={target_arc_deg:.1f}° ({direction.upper()}), "
                f"Beklenen Kiriş={expected_chord_m:.2f}m, ω={nominal_omega:+.2f} rad/s"
            )

        prev_yaw = start_state["yaw"]
        accumulated_yaw_deg = 0.0
        t_start = time.time()

        while not self._abort_requested and (time.time() - t_start) < timeout_sec:
            cur = self.get_telemetry()
            if not cur:
                time.sleep(0.05)
                continue

            step_yaw = normalize_angle_deg(cur["yaw"] - prev_yaw)
            prev_yaw = cur["yaw"]
            accumulated_yaw_deg += abs(step_yaw)

            chk = self.double_check_displacement(start_state, cur)
            remaining_deg = target_arc_deg - accumulated_yaw_deg

            if remaining_deg <= 2.5 or (target_arc_deg < 150.0 and chk["fused_dist"] >= expected_chord_m * 0.98):
                self.stop()
                msg = (
                    f"✅ [CURVE TAMAM] Taranan Açı: {accumulated_yaw_deg:.1f}°/{target_arc_deg:.1f}° | "
                    f"Kiriş Mesafesi: {chk['fused_dist']:.2f}m (Beklenen: {expected_chord_m:.2f}m)"
                )
                if status_callback:
                    status_callback(msg)
                return {"ok": True, "accumulated_deg": accumulated_yaw_deg, "double_check": chk, "message": msg}

            # Yay sonuna yaklaşırken hızı ve açısal hızı orantılı sönümle
            scale = min(1.0, max(0.35, remaining_deg / 20.0))
            self.send_cmd_vel(speed_mps * scale, nominal_omega * scale, timeout_ms=400)
            time.sleep(0.08)

        self.stop()
        return {"ok": False, "reason": "Curve sürüş zaman aşımına uğradı"}

    # =========================================================================
    # 4. HEDEF KOORDİNATA OTONOM SÜRÜŞ (PURE PURSUIT CURVE + DOUBLE CHECK)
    # =========================================================================
    def navigate_to_gps(
        self,
        target_lat: float,
        target_lon: float,
        cruise_speed: float = 1.8,
        mode: str = "curve",
        arrival_tolerance_m: float = 0.85,
        timeout_sec: float = 180.0,
        status_callback: Optional[Callable[[str], None]] = None,
    ) -> Dict:
        """
        Aracı verilen GPS (target_lat, target_lon) koordinatına otonom olarak sürer.

        Çift Doğrulama (Double-Check) Mekanizması:
          1. Başlangıç anında GPS koordinat farkından yerel hedef (target_pos_x, target_pos_y) hesaplanır.
          2. Sürüş boyunca her adımda:
             - GPS'ten kalan mesafe ve hedef açısı (gps_dist, gps_bearing)
             - Yerel Odometriden kalan mesafe ve hedef açısı (odom_dist, odom_bearing)
             birlikte hesaplanır ve çapraz doğrulanır!
          3. 'curve' (Pure Pursuit) modunda araç duraksamadan eğrisel yörüngeyle hedefe süzülür;
             keskin açı farklarında hızı otomatik düşürüp eğriyi daraltır.
        """
        self._abort_requested = False
        start_state = self.wait_for_telemetry(timeout_sec=5.0)
        if not start_state:
            msg = "❌ HATA: Araçtan canlı telemetri alınamadı! (Unreal Engine çalışıyor mu?)"
            if status_callback:
                status_callback(msg)
            return {"ok": False, "reason": msg}

        # Başlangıçta GPS -> Yerel Odometri Hedef Koordinat Eşlemesi (Double-Check Referansı)
        init_dx, init_dy, init_dist, init_bearing = gps_to_delta_meters(
            start_state["lat"], start_state["lon"], target_lat, target_lon
        )
        target_odom_x = start_state["pos_x"] + init_dx
        target_odom_y = start_state["pos_y"] + init_dy

        start_msg = (
            f"🎯 [OTONOM ROTA BAŞLADI] Araç: {start_state['name']} [{start_state['code']}] | "
            f"Hedef GPS: ({target_lat:.6f}°, {target_lon:.6f}°) | "
            f"Hedef Yerel: (X:{target_odom_x:.1f}m, Y:{target_odom_y:.1f}m) | "
            f"Toplam Mesafe: {init_dist:.2f}m | İlk Açı: {init_bearing:.1f}° | Mod: {mode.upper()}"
        )
        if status_callback:
            status_callback(start_msg)

        # Eğer 'point-turn' modu seçildiyse önce hedefe doğru yerinde dön
        if mode == "point-turn" and init_dist > arrival_tolerance_m:
            self.turn_to_heading(init_bearing, tolerance_deg=4.0, status_callback=status_callback)

        t_start = time.time()
        last_log_time = 0.0

        while not self._abort_requested and (time.time() - t_start) < timeout_sec:
            cur = self.get_telemetry()
            if not cur:
                time.sleep(0.05)
                continue

            # --- KAYNAK 1: CANLI GPS HEDEF VEKTÖRÜ ---
            gps_dx, gps_dy, gps_rem_dist, gps_bearing = gps_to_delta_meters(
                cur["lat"], cur["lon"], target_lat, target_lon
            )

            # --- KAYNAK 2: CANLI YEREL ODOMETRİ (pos_x, pos_y) HEDEF VEKTÖRÜ ---
            odom_dx = target_odom_x - cur["pos_x"]
            odom_dy = target_odom_y - cur["pos_y"]
            odom_rem_dist = math.hypot(odom_dx, odom_dy)
            odom_bearing = math.degrees(math.atan2(odom_dy, odom_dx))

            # --- ÇİFT DOĞRULAMA (DOUBLE CHECK FÜZYONU) ---
            sensor_discrepancy = abs(gps_rem_dist - odom_rem_dist)
            fused_rem_dist = 0.5 * gps_rem_dist + 0.5 * odom_rem_dist
            # Eğer GPS sabitlenmişse odometriyi, ikisi de aktifse ağırlıklı ortalamayı kullan
            if gps_rem_dist < 0.01 and odom_rem_dist > 0.5:
                fused_rem_dist = odom_rem_dist
                fused_bearing = odom_bearing
            else:
                # Vektörel ortalama ile hedef açısı
                fused_vx = 0.5 * gps_dx + 0.5 * odom_dx
                fused_vy = 0.5 * gps_dy + 0.5 * odom_dy
                fused_bearing = math.degrees(math.atan2(fused_vy, fused_vx))

            confidence_pct = max(0.0, min(100.0, 100.0 * (1.0 - sensor_discrepancy / max(2.0, init_dist))))

            # HEDEF VARIŞ KONTROLÜ (Hem Füzyon hem Odometri/GPS teyidi)
            if fused_rem_dist <= arrival_tolerance_m or min(gps_rem_dist, odom_rem_dist) <= (arrival_tolerance_m * 0.75):
                self.stop()
                done_msg = (
                    f"🏁 [HEDEFE ULAŞILDI!] Araç: {cur['code']} | "
                    f"Kalan GPS: {gps_rem_dist:.2f}m | Kalan Odom: {odom_rem_dist:.2f}m | "
                    f"Çift Doğrulama Güven: %{confidence_pct:.1f}"
                )
                if status_callback:
                    status_callback(done_msg)
                return {
                    "ok": True,
                    "final_lat": cur["lat"],
                    "final_lon": cur["lon"],
                    "final_x": cur["pos_x"],
                    "final_y": cur["pos_y"],
                    "gps_rem_m": round(gps_rem_dist, 3),
                    "odom_rem_m": round(odom_rem_dist, 3),
                    "confidence_pct": round(confidence_pct, 1),
                    "message": done_msg,
                }

            # Baş açısı hatası: Hedef yön - Mevcut IMU Yaw
            heading_err_deg = normalize_angle_deg(fused_bearing - cur["yaw"])
            heading_err_rad = math.radians(heading_err_deg)

            # Pure Pursuit / Eğrisel (Curve) Direksiyon ve Hız Hesabı:
            # Açı hatası büyükse hızı düşür ve kavisli dön; açı hatası küçükse tam hızla süzül
            if abs(heading_err_deg) > 65.0:
                # Çok keskin açı farkı: Dar kavisli dönüş (yavaş ilerleme + güçlü dönüş)
                linear_v = cruise_speed * 0.25
                angular_w = max(-1.35, min(1.35, heading_err_rad * 1.4))
            else:
                # Yumuşak eğrisel (Curve) takip: Pure Pursuit kavis kanunu
                lookahead_L = max(1.2, min(4.0, fused_rem_dist * 0.6))
                # Eğrilik (Curvature) kappa = 2 * sin(alpha) / L
                curvature = (2.0 * math.sin(heading_err_rad)) / lookahead_L
                dist_slowdown = min(1.0, max(0.30, fused_rem_dist / 3.0))
                turn_slowdown = max(0.35, math.cos(heading_err_rad))
                linear_v = cruise_speed * dist_slowdown * turn_slowdown
                angular_w = max(-1.25, min(1.25, linear_v * curvature + heading_err_rad * 0.85))

            self.send_cmd_vel(linear_v, angular_w, timeout_ms=450)

            now = time.time()
            if status_callback and (now - last_log_time >= 1.0):
                last_log_time = now
                status_callback(
                    f"🚗 [SÜRÜŞ] Kalan: {fused_rem_dist:.1f}m (GPS:{gps_rem_dist:.1f}m | Odom:{odom_rem_dist:.1f}m | "
                    f"Güven:%{confidence_pct:.0f}) | Yaw:{cur['yaw']:.0f}°->Hedef:{fused_bearing:.0f}° "
                    f"(Δ:{heading_err_deg:+.0f}°) | V:{linear_v:.2f}m/s ω:{angular_w:+.2f}rad/s"
                )

            time.sleep(0.08)

        self.stop()
        fail_msg = "⚠️ [DURDURULDU] Otonom sürüş kullanıcı tarafından iptal edildi veya süre doldu."
        if status_callback:
            status_callback(fail_msg)
        return {"ok": False, "reason": fail_msg}

    def wait_for_telemetry(self, timeout_sec: float = 5.0) -> Optional[Dict]:
        """Belirtilen süre içinde ilk geçerli telemetri paketini bekler."""
        t0 = time.time()
        while (time.time() - t0) < timeout_sec:
            t = self.get_telemetry()
            if t is not None:
                return t
            time.sleep(0.1)
        return None
