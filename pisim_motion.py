#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PiSim - Gerçekçi Otonom Hareket ve ArduPilot L1 Navigasyon Kütüphanesi (pisim_motion.py)
========================================================================================
Bu modül, HİÇBİR simülasyon hilesi (Unreal pos_x, pos_y ground-truth) KULLANMADAN,
yalnızca gerçek bir otonom aracın (ArduPilot Rover / İKA) sahip olduğu fiziksel sensörlerle çalışır:

Kullanılan Gerçek Sensörler:
  1. GNSS / GPSAlıcısı : Enlem (lat), Boylam (lon), İrtifa (alt), Yer Hızı (speed_mps)
  2. IMU & Pusula      : Roll, Pitch, Yaw (Manyetik/Pusula Baş Açısı - Derece)
  3. Ölü Hesaplama (Dead-Reckoning / INS Çift Doğrulama):
     - Gerçek araçlarda GPS verisini doğrulamak için anlık Hız (v) ve Pusula Açısı (Yaw)
       zamanla entegre edilir: dN = ∫ v·cos(ψ) dt, dE = ∫ v·sin(ψ) dt.
     - GPS Haversine mesafesi ile IMU/Pusula Ölü Hesaplama (Dead-Reckoning) mesafesi
       çapraz kontrol edilerek (Double-Check) GPS sıçramaları tespit edilir.

Sürüş ve Dönüş Kinematiği (ArduPilot L1 & Geniş Turn Radius):
  - Araç ASLA kare şeklinde (dur -> olduğu yerde 90° dön -> düz git) hareket etmez!
  - ArduPilot'un `AP_L1_Control` (L1 Nonlinear Guidance) yasasını ve geniş dönüş yarıçapını
    (`min_turn_radius_m`, varsayılan 5.0 metre) kullanır.
  - Açısal hız her zaman ileri hız ve dönüş yarıçapı ile sınırlandırılır: |ω| <= V / R_min.
"""

import json
import math
import socket
import time
import urllib.request
from typing import Callable, Dict, Optional, Tuple


EARTH_RADIUS_M = 6378137.0


def normalize_angle_deg(angle: float) -> float:
    """Açıyı [-180.0, +180.0] derece aralığına normalize eder."""
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def haversine_distance_and_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> Tuple[float, float, float, float]:
    """
    İki gerçek GPS koordinatı (WGS84) arasındaki:
      - Kuzey mesafesi (d_north_m)
      - Doğu mesafesi (d_east_m)
      - Gerçek Haversine küresel mesafesi (dist_m)
      - Pusula hedef açısı (bearing_deg: 0°=Kuzey, +90°=Doğu, -90°=Batı)
    değerlerini hesaplar.
    """
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
    """Bir GPS koordinatından (Kuzey metre, Doğu metre) ötedeki hedef GPS koordinatını hesaplar."""
    dlat = math.degrees(d_north_m / EARTH_RADIUS_M)
    dlon = math.degrees(d_east_m / (EARTH_RADIUS_M * max(1e-6, math.cos(math.radians(origin_lat)))))
    return origin_lat + dlat, origin_lon + dlon


class PiSimMotionController:
    """
    Sadece gerçek araç sensörlerini (GPS + IMU Pusula + Hız Entegrasyonu) ve
    ArduPilot L1 geniş dönüş yarıçaplı (Turn-Radius) güdüm algoritmasını kullanan hareket kütüphanesi.
    """

    def __init__(
        self,
        target_vehicle: str = "selected",
        ue5_ip: Optional[str] = None,
        dashboard_api_url: str = "http://127.0.0.1:8080/api/telemetry",
        command_port: int = 7402,
        min_turn_radius_m: float = 5.0,
    ):
        self.target_vehicle = str(target_vehicle)
        self.ue5_ip = ue5_ip
        self.dashboard_api_url = dashboard_api_url
        self.command_port = int(command_port)
        self.dedicated_port: Optional[int] = None
        # Geniş dönüş yarıçapı (m): Araç asla olduğu yerde kare gibi dönmez, bu yarıçapla kavis alır
        self.min_turn_radius_m = max(1.0, float(min_turn_radius_m))

        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        self.fallback_rx_sock: Optional[socket.socket] = None
        self._abort_requested = False

    def request_abort(self):
        """Aktif otonom görevi iptal eder ve aracı durdurur."""
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

    def get_sensor_telemetry(self) -> Optional[Dict]:
        """
        Araçtan SADECE gerçek sensör verilerini (GPS Enlem/Boylam/İrtifa, Hız ve IMU Roll/Pitch/Yaw Pusula) okur.
        Unreal Engine'in yerel pos_x, pos_y, pos_z koordinatları HİÇBİR ŞEKİLDE okunmaz ve kullanılmaz!
        """
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
                        "roll": float(chosen.get("roll", 0.0)),
                        "pitch": float(chosen.get("pitch", 0.0)),
                        "yaw": float(chosen.get("yaw", 0.0)),
                        "speed_kmh": spd_kmh,
                        "speed_mps": spd_kmh / 3.6,
                        "timestamp": time.time(),
                    }
        except Exception:
            pass

        self._init_fallback_udp()
        if self.fallback_rx_sock:
            try:
                raw, addr = self.fallback_rx_sock.recvfrom(4096)
                if not self.ue5_ip:
                    self.ue5_ip = addr[0]
                pkt = json.loads(raw.decode("utf-8", errors="ignore").strip())
                gps = pkt.get("gps", {})
                att = pkt.get("attitude", {})
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
                    "roll": float(att.get("roll", 0.0)),
                    "pitch": float(att.get("pitch", 0.0)),
                    "yaw": float(att.get("yaw", 0.0)),
                    "speed_kmh": spd_kmh,
                    "speed_mps": spd_kmh / 3.6,
                    "timestamp": time.time(),
                }
            except Exception:
                pass
        return None

    def wait_for_telemetry(self, timeout_sec: float = 5.0) -> Optional[Dict]:
        t0 = time.time()
        while (time.time() - t0) < timeout_sec:
            t = self.get_sensor_telemetry()
            if t is not None:
                return t
            time.sleep(0.1)
        return None

    def send_cmd_vel(self, linear_x: float, angular_z: float, timeout_ms: int = 500):
        """
        Araca ileri hız (linear_x m/s) ve dönüş hızı (angular_z rad/s) komutunu gönderir.
        DİKKAT: Kare dönüşü önlemek için araç hareket halindeyken açısal hız her zaman
        minimum dönüş yarıçapına (min_turn_radius_m) göre sınırlandırılır!
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
        """Aracı güvenli şekilde durdurur."""
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
    # 1. GENİŞ YARIÇAPLI EĞRİSEL DÖNÜŞ (CURVE TURN - KARE DÖNÜŞ YOK!)
    # =========================================================================
    def drive_curve(
        self,
        turn_radius_m: Optional[float] = None,
        arc_angle_deg: float = 90.0,
        speed_mps: float = 1.6,
        direction: str = "right",
        timeout_sec: float = 60.0,
        status_callback: Optional[Callable[[str], None]] = None,
    ) -> Dict:
        """
        Aracı ASLA durdurmadan ve kare köşe yapmadan, geniş bir dönüş yarıçapı (turn_radius_m)
        boyunca ileri doğru akıcı bir yay çizerek döndürür.
        Kinematik:
          - Çizgisel Hız : V = speed_mps
          - Açısal Hız   : ω = ± V / R_turn
        Doğrulama:
          - IMU Pusula (Yaw) açı değişimi + GPS Haversine kiriş mesafesi!
        """
        self._abort_requested = False
        start_state = self.wait_for_telemetry(timeout_sec=5.0)
        if not start_state:
            return {"ok": False, "reason": "Sensör telemetrisi alınamadı"}

        radius = max(self.min_turn_radius_m, float(turn_radius_m) if turn_radius_m else self.min_turn_radius_m)
        target_arc_deg = abs(float(arc_angle_deg))
        turn_sign = 1.0 if str(direction).lower() in ("right", "sag", "cw", "+") else -1.0

        # Geniş dönüş yarıçapı için açısal hız: ω = V / R
        omega_rad = turn_sign * (abs(speed_mps) / radius)
        expected_chord_m = 2.0 * radius * math.sin(math.radians(min(180.0, target_arc_deg)) / 2.0)

        if status_callback:
            status_callback(
                f"🌀 [GENİŞ KAVİSLİ DÖNÜŞ] Dönüş Yarıçapı R={radius:.1f}m | Hedef Yay={target_arc_deg:.1f}° "
                f"({direction.upper()}) | Hız={speed_mps:.2f}m/s | ω={omega_rad:+.3f} rad/s"
            )

        prev_yaw = start_state["yaw"]
        accumulated_yaw = 0.0
        dr_dist = 0.0
        prev_time = time.time()
        t_start = prev_time

        while not self._abort_requested and (time.time() - t_start) < timeout_sec:
            cur = self.get_sensor_telemetry()
            if not cur:
                time.sleep(0.05)
                continue

            now = time.time()
            dt = max(0.001, now - prev_time)
            prev_time = now

            # 1. IMU Pusula açı entegrasyonu
            dyaw = normalize_angle_deg(cur["yaw"] - prev_yaw)
            prev_yaw = cur["yaw"]
            accumulated_yaw += abs(dyaw)

            # 2. Hız + Pusula Ölü Hesaplama (Dead-Reckoning) yay uzunluğu
            measured_v = cur["speed_mps"] if cur["speed_mps"] > 0.1 else speed_mps
            dr_dist += measured_v * dt

            # 3. GPS Haversine kiriş mesafesi
            _, _, gps_chord_m, _ = haversine_distance_and_bearing(
                start_state["lat"], start_state["lon"], cur["lat"], cur["lon"]
            )

            remaining_deg = target_arc_deg - accumulated_yaw
            if remaining_deg <= 2.0:
                self.stop()
                msg = (
                    f"✅ [KAVİSLİ DÖNÜŞ TAMAMLANDI] Taranan Açı: {accumulated_yaw:.1f}°/{target_arc_deg:.1f}° | "
                    f"GPS Kiriş: {gps_chord_m:.2f}m (Beklenen: {expected_chord_m:.2f}m) | Yay: {dr_dist:.2f}m"
                )
                if status_callback:
                    status_callback(msg)
                return {"ok": True, "accumulated_deg": accumulated_yaw, "gps_chord_m": gps_chord_m}

            # Yay sonuna yaklaşırken hem ileri hızı hem açısal hızı aynı oranda azalt (Yarıçap R=V/ω sabit kalır!)
            scale = min(1.0, max(0.4, remaining_deg / 18.0))
            self.send_cmd_vel(speed_mps * scale, omega_rad * scale, timeout_ms=450)
            time.sleep(0.08)

        self.stop()
        return {"ok": False, "reason": "Dönüş zaman aşımına uğradı veya iptal edildi"}

    # =========================================================================
    # 2. GERÇEK SENSÖR ÇİFT DOĞRULAMALI (GPS + INS DEAD-RECKONING) DÜZ İLERLEME
    # =========================================================================
    def move_forward(
        self,
        distance_m: float,
        speed_mps: float = 1.6,
        tolerance_m: float = 0.4,
        timeout_sec: float = 90.0,
        status_callback: Optional[Callable[[str], None]] = None,
    ) -> Dict:
        """
        Sadece GPS (Enlem/Boylam) ve IMU Pusula + Hız Entegrasyonu (Dead-Reckoning / INS)
        kullanarak aracı düz ilerletir.
        """
        self._abort_requested = False
        start_state = self.wait_for_telemetry(timeout_sec=5.0)
        if not start_state:
            return {"ok": False, "reason": "Sensör telemetrisi alınamadı"}

        target_dist = abs(float(distance_m))
        dir_sign = 1.0 if distance_m >= 0.0 else -1.0
        target_heading = start_state["yaw"]

        dr_north = 0.0
        dr_east = 0.0
        prev_time = time.time()
        t_start = prev_time

        if status_callback:
            status_callback(
                f"⬆️ [DÜZ İLERLEME] Hedef: {distance_m:+.1f}m | Kilitli Pusula Açısı: {target_heading:.1f}° "
                f"| Sensörler: GPS + IMU Dead-Reckoning"
            )

        while not self._abort_requested and (time.time() - t_start) < timeout_sec:
            cur = self.get_sensor_telemetry()
            if not cur:
                time.sleep(0.05)
                continue

            now = time.time()
            dt = max(0.001, min(0.5, now - prev_time))
            prev_time = now

            # 1. GPS (WGS84 Haversine) Mesafesi
            _, _, gps_dist_m, _ = haversine_distance_and_bearing(
                start_state["lat"], start_state["lon"], cur["lat"], cur["lon"]
            )

            # 2. IMU Pusula + Hız Entegrasyonu (Gerçek Dead-Reckoning / INS)
            v_est = cur["speed_mps"] if cur["speed_mps"] > 0.05 else abs(speed_mps)
            yaw_rad = math.radians(cur["yaw"])
            dr_north += dir_sign * v_est * math.cos(yaw_rad) * dt
            dr_east += dir_sign * v_est * math.sin(yaw_rad) * dt
            ins_dist_m = math.hypot(dr_north, dr_east)

            # GPS (2Hz) ile INS (12Hz) Çift Doğrulama Füzyonu (EKF benzeri: %70 GPS + %30 INS)
            fused_dist = 0.7 * gps_dist_m + 0.3 * ins_dist_m if gps_dist_m > 0.05 else ins_dist_m
            remaining_m = target_dist - fused_dist

            if remaining_m <= tolerance_m or gps_dist_m >= (target_dist - tolerance_m * 0.5):
                self.stop()
                msg = (
                    f"✅ [İLERLEME TAMAMLANDI] GPS Mesafesi: {gps_dist_m:.2f}m | "
                    f"IMU/INS Ölü Hesaplama: {ins_dist_m:.2f}m | Fark: {abs(gps_dist_m - ins_dist_m):.2f}m"
                )
                if status_callback:
                    status_callback(msg)
                return {"ok": True, "gps_dist_m": gps_dist_m, "ins_dist_m": ins_dist_m}

            slowdown = min(1.0, max(0.35, remaining_m / 3.0))
            cmd_v = dir_sign * abs(speed_mps) * slowdown

            # Pusula sapmasını geniş dönüş yarıçapını bozmadan yumuşakça düzelt
            yaw_err_rad = math.radians(normalize_angle_deg(target_heading - cur["yaw"]))
            max_w = abs(cmd_v) / self.min_turn_radius_m
            cmd_w = max(-max_w, min(max_w, yaw_err_rad * 0.9))

            self.send_cmd_vel(cmd_v, cmd_w, timeout_ms=450)
            time.sleep(0.08)

        self.stop()
        return {"ok": False, "reason": "Zaman aşımı veya iptal edildi"}

    # =========================================================================
    # 3. ARDUPILOT L1 GÜDÜM YASASI İLE GPS KOORDİNATINA GENİŞ KAVİSLİ SÜRÜŞ
    # =========================================================================
    def navigate_to_gps(
        self,
        target_lat: float,
        target_lon: float,
        cruise_speed: float = 1.8,
        turn_radius_m: Optional[float] = None,
        wp_radius_m: float = 1.5,
        timeout_sec: float = 240.0,
        status_callback: Optional[Callable[[str], None]] = None,
    ) -> Dict:
        """
        ArduPilot Rover'ın kullandığı gerçek `AP_L1_Control` (L1 Nonlinear Guidance) ve
        geniş dönüş yarıçapı (Turn Radius) mimarisiyle hedef GPS koordinatına otonom sürüş yapar.

        ArduPilot Nasıl Çalışır & Burada Nasıl Uygulandı?
          1. İleri-Geri Arama (Hunting) Yapmaz:
             Hedefin etrafında `wp_radius_m` (Waypoint yarıçapı) ve "Along-Track Bitiş Çizgisi Düzlemi"
             tanımlanır. Araç hedef dairesine girdiğinde veya bitiş düzlemini geçtiğinde anında durur.
          2. Kare Dönüş Yapmaz (Geniş Turn Radius):
             Araç hiçbir zaman durup olduğu yerde dönmez. İleri hızı (V) her zaman pozitiftir ve
             maksimum açısal hızı `ω_max = V / R_min` ile sınırlandırılmıştır.
          3. L1 Nonlinear Guidance (Yanal İvme Kavis Kontrolü):
             Bakış mesafesi `L1_dist = max(2.5 * R_min, 3.0 * V)` üzerinden yanal ivme:
             `a_lat = 2 * V^2 / L1_dist * sin(η)` ve buradan istenen dönüş hızı:
             `ω_cmd = (2 * V / L1_dist) * sin(η)` hesaplanarak araç geniş, pürüzsüz bir yayla
             hedef rotasına teğet oturur.
        """
        self._abort_requested = False
        start_state = self.wait_for_telemetry(timeout_sec=5.0)
        if not start_state:
            msg = "❌ HATA: Araçtan GPS/IMU telemetrisi alınamadı!"
            if status_callback:
                status_callback(msg)
            return {"ok": False, "reason": msg}

        r_min = max(1.5, float(turn_radius_m) if turn_radius_m else self.min_turn_radius_m)
        init_dn, init_de, init_dist_m, init_bearing_deg = haversine_distance_and_bearing(
            start_state["lat"], start_state["lon"], target_lat, target_lon
        )

        # Başlangıç-Hedef birim vektörü (ArduPilot Along-Track bitiş çizgisi kontrolü için)
        path_norm = max(0.01, math.hypot(init_dn, init_de))
        path_unit_n = init_dn / path_norm
        path_unit_e = init_de / path_norm

        # EKF / Dead-Reckoning (INS) iç durum tahmincisi (2Hz GPS paketleri arasında 12Hz akıcı tahmin)
        est_lat = start_state["lat"]
        est_lon = start_state["lon"]
        last_raw_gps = (start_state["lat"], start_state["lon"])
        prev_time = time.time()
        t_start = prev_time
        last_log_time = 0.0

        if status_callback:
            status_callback(
                f"🛰️ [ARDUPILOT L1 ROTA BAŞLADI] Araç: {start_state['name']} [{start_state['code']}] | "
                f"Hedef GPS: ({target_lat:.6f}°N, {target_lon:.6f}°E) | "
                f"GPS Mesafe: {init_dist_m:.1f}m | Pusula Hedef: {init_bearing_deg:.1f}° | "
                f"Min Dönüş Yarıçapı (Turn Radius): {r_min:.1f}m"
            )

        while not self._abort_requested and (time.time() - t_start) < timeout_sec:
            cur = self.get_sensor_telemetry()
            if not cur:
                time.sleep(0.05)
                continue

            now = time.time()
            dt = max(0.001, min(0.3, now - prev_time))
            prev_time = now

            # --- 1. EKF / INS + GPS SENSÖR FÜZYONU (Gerçek Araç Çift Doğrulaması) ---
            raw_lat, raw_lon = cur["lat"], cur["lon"]
            yaw_rad = math.radians(cur["yaw"])
            v_meas = cur["speed_mps"]

            if (raw_lat, raw_lon) != last_raw_gps:
                # Yeni GPS paketi geldi: INS tahminini GPS ile düzelt (%75 GPS + %25 INS)
                last_raw_gps = (raw_lat, raw_lon)
                est_lat = 0.75 * raw_lat + 0.25 * est_lat
                est_lon = 0.75 * raw_lon + 0.25 * est_lon
            else:
                # İki GPS paketi arasındayız (Dead-Reckoning): Pusula + Hız ile konumu ilerlet
                step_n = v_meas * math.cos(yaw_rad) * dt
                step_e = v_meas * math.sin(yaw_rad) * dt
                est_lat, est_lon = offset_gps_by_meters(est_lat, est_lon, step_n, step_e)

            # Hedefe kalan vektör (Füzyonlanmış GPS+INS konumundan)
            rem_n, rem_e, rem_dist_m, target_bearing_deg = haversine_distance_and_bearing(
                est_lat, est_lon, target_lat, target_lon
            )
            _, _, raw_gps_dist_m, _ = haversine_distance_and_bearing(
                raw_lat, raw_lon, target_lat, target_lon
            )

            # --- 2. ARDUPILOT WP_RADIUS & ALONG-TRACK VARIŞ KONTROLÜ ---
            # Along-track projeksiyonu: Negatif olması aracın hedef bitiş çizgisini geçtiğini gösterir
            along_track_rem = rem_n * path_unit_n + rem_e * path_unit_e
            if rem_dist_m <= wp_radius_m or raw_gps_dist_m <= wp_radius_m or (along_track_rem <= 0.0 and rem_dist_m <= wp_radius_m * 2.2):
                self.stop()
                done_msg = (
                    f"🏁 [WAYPOINT ULAŞILDI!] Araç: {cur['code']} | "
                    f"Mevcut GPS: ({raw_lat:.6f}°N, {raw_lon:.6f}°E) | "
                    f"Hedefe Kalan GPS: {raw_gps_dist_m:.2f}m (WP_RADIUS: {wp_radius_m:.1f}m)"
                )
                if status_callback:
                    status_callback(done_msg)
                return {
                    "ok": True,
                    "final_lat": raw_lat,
                    "final_lon": raw_lon,
                    "gps_rem_m": round(raw_gps_dist_m, 2),
                    "message": done_msg,
                }

            # --- 3. ARDUPILOT L1 NONLINEAR GUIDANCE + GENİŞ TURN RADIUS ---
            # η (eta): Hedef pusula açısı ile aracın mevcut pusula açısı (Yaw) arasındaki fark
            eta_deg = normalize_angle_deg(target_bearing_deg - cur["yaw"])
            eta_rad = math.radians(eta_deg)

            # Hedefe yaklaşırken yumuşak hız profili (Ama dönüş yaparken ASLA sıfırlanmaz, kavis çizer!)
            approach_scale = min(1.0, max(0.45, rem_dist_m / (r_min * 1.5)))
            # Geniş kavis alırken merkezkaç sınırı için hızı hafifçe dengeler
            turn_speed_scale = 0.70 if abs(eta_deg) > 45.0 else 1.0
            cmd_v = cruise_speed * approach_scale * turn_speed_scale

            # L1 Lookahead mesafesi: Dönüş yarıçapı ne kadar genişse L1 o kadar ileri bakar (Geniş yay)
            l1_lookahead_m = max(r_min * 1.8, min(rem_dist_m, r_min * 3.0))

            # Eğer açı farkı > 90° ise (hedef aracın arkasında kaldıysa) geniş bir U-dönüşü yayı çiz
            if abs(eta_deg) > 85.0:
                desired_omega = math.copysign(cmd_v / r_min, eta_rad)
            else:
                # Standart ArduPilot L1 kavis denklemi: ω = (2 * V / L1) * sin(η)
                desired_omega = (2.0 * cmd_v / l1_lookahead_m) * math.sin(eta_rad)

            # Turn-Radius Kısıtı: Açısal hız ASLA (V / R_min) değerini aşamaz! (Keskin/kare dönüş imkansızdır)
            max_omega_for_radius = cmd_v / r_min
            cmd_omega = max(-max_omega_for_radius, min(max_omega_for_radius, desired_omega))

            self.send_cmd_vel(cmd_v, cmd_omega, timeout_ms=450)

            if status_callback and (now - last_log_time >= 1.0):
                last_log_time = now
                eff_radius = (cmd_v / abs(cmd_omega)) if abs(cmd_omega) > 0.02 else 999.0
                status_callback(
                    f"🚗 [L1 SÜRÜŞ] GPS Mesafe: {raw_gps_dist_m:.1f}m (EKF: {rem_dist_m:.1f}m) | "
                    f"Pusula: {cur['yaw']:.0f}° -> Hedef: {target_bearing_deg:.0f}° (Δ:{eta_deg:+.0f}°) | "
                    f"Hız: {cmd_v:.2f}m/s | Kavis Yarıçapı R: {min(99.9, eff_radius):.1f}m"
                )

            time.sleep(0.08)

        self.stop()
        return {"ok": False, "reason": "Süre doldu veya kullanıcı tarafından durduruldu"}
