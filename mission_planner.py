#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mission_planner.py - PiSim Otonom Görev ve Rota Planlayıcı (Dosya 1)
===================================================================
Bu modül, araçlara ve Docker konteynerlerine özgü rastgele otonom devriye
ve seyir görevleri üretir.

Temel Kurallar:
  1. Başlangıç (veya mevcut araç) koordinatını referans alır.
  2. Belirtilen sayıda (varsayılan 4 adet) hedef waypoint üretir.
  3. Üretilen her hedef nokta, hem başlangıç noktasından hem de daha önce
     oluşturulmuş diğer tüm hedef noktalardan parametrik olarak en az
     `min_distance_m` (varsayılan 100.0 metre) uzaktadır.
  4. Hem Python kodu içerisinden modül olarak import edilip çağrılabilir,
     hem de doğrudan terminalden (CLI) bağımsız çalıştırılabilir.

Kullanım:
  - Python API:
      from mission_planner import generate_random_mission, MissionPlan
      mission = generate_random_mission(lat=39.9334, lon=32.8597, min_distance_m=100.0, num_waypoints=4)
      print(mission.to_json())

  - CLI:
      python mission_planner.py --lat 39.9334 --lon 32.8597 --distance 100 --count 4
"""

import argparse
import json
import math
import os
import random
import sys
import time
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Tuple

EARTH_RADIUS_M = 6378137.0


def haversine_distance_and_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> Tuple[float, float, float, float]:
    """
    İki WGS84 GPS koordinatı arasındaki:
      - Kuzey mesafesini (d_north_m)
      - Doğu mesafesini (d_east_m)
      - Haversine küresel yüzey mesafesini (dist_m)
      - Pusula hedef açısını (bearing_deg, 0°=Kuzey, 90°=Doğu)
    hesaplar.
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
    if bearing_deg < 0:
        bearing_deg += 360.0

    return d_north_m, d_east_m, dist_m, bearing_deg


def offset_gps_by_meters(origin_lat: float, origin_lon: float, d_north_m: float, d_east_m: float) -> Tuple[float, float]:
    """Bir GPS koordinatını Kuzey ve Doğu metre kadar öteler."""
    dlat = math.degrees(d_north_m / EARTH_RADIUS_M)
    dlon = math.degrees(d_east_m / (EARTH_RADIUS_M * max(1e-6, math.cos(math.radians(origin_lat)))))
    return origin_lat + dlat, origin_lon + dlon


@dataclass
class Waypoint:
    index: int
    lat: float
    lon: float
    target_dist_m: float
    bearing_from_prev_deg: float
    dist_to_origin_m: float

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class MissionPlan:
    mission_id: str
    vehicle_code: str
    origin_lat: float
    origin_lon: float
    min_distance_m: float
    created_at: float = field(default_factory=time.time)
    waypoints: List[Waypoint] = field(default_factory=list)

    @property
    def total_waypoints(self) -> int:
        return len(self.waypoints)

    def to_dict(self) -> Dict:
        return {
            "mission_id": self.mission_id,
            "vehicle_code": self.vehicle_code,
            "origin_lat": self.origin_lat,
            "origin_lon": self.origin_lon,
            "min_distance_m": self.min_distance_m,
            "created_at": self.created_at,
            "waypoints": [wp.to_dict() for wp in self.waypoints],
        }

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), indent=indent, ensure_ascii=False)

    def save_file(self, filepath: str) -> bool:
        try:
            with open(filepath, "w", encoding="utf-8") as f:
                f.write(self.to_json(indent=2))
            return True
        except Exception as e:
            print(f"❌ Görev dosyası kaydedilemedi ({filepath}): {e}")
            return False

    @classmethod
    def from_dict(cls, data: Dict) -> "MissionPlan":
        wps = [
            Waypoint(
                index=w["index"],
                lat=w["lat"],
                lon=w["lon"],
                target_dist_m=w.get("target_dist_m", 0.0),
                bearing_from_prev_deg=w.get("bearing_from_prev_deg", 0.0),
                dist_to_origin_m=w.get("dist_to_origin_m", 0.0),
            )
            for w in data.get("waypoints", [])
        ]
        return cls(
            mission_id=data.get("mission_id", f"msn_{int(time.time())}"),
            vehicle_code=data.get("vehicle_code", "rover_01"),
            origin_lat=float(data.get("origin_lat", 39.9334)),
            origin_lon=float(data.get("origin_lon", 32.8597)),
            min_distance_m=float(data.get("min_distance_m", 100.0)),
            created_at=float(data.get("created_at", time.time())),
            waypoints=wps,
        )

    @classmethod
    def load_file(cls, filepath: str) -> Optional["MissionPlan"]:
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                data = json.load(f)
            return cls.from_dict(data)
        except Exception as e:
            print(f"❌ Görev dosyası okunamadı ({filepath}): {e}")
            return None


def generate_random_mission(
    origin_lat: float = 39.9334,
    origin_lon: float = 32.8597,
    num_waypoints: int = 4,
    min_distance_m: float = 100.0,
    vehicle_code: str = "rover_01",
    seed: Optional[int] = None,
    max_leg_distance_m: Optional[float] = None,
) -> MissionPlan:
    """
    Belirtilen başlangıç noktasından itibaren parametrik otonom devriye görevi üretir.

    Kural:
      - Toplam `num_waypoints` (varsayılan 4) nokta oluşturulur.
      - Her nokta:
          1. Başlangıç noktasından EN AZ `min_distance_m` (varsayılan 100m) uzakta olmalıdır.
          2. Daha önce oluşturulmuş diğer tüm hedef noktalardan EN AZ `min_distance_m` uzakta olmalıdır.
      - Araç için gerçekçi bir seyir rotası oluşturmak adına ardışık bacaklar ~[min_dist, max_leg_dist]
        mesafesinde bağlanır.
    """
    if seed is not None:
        rnd = random.Random(seed)
    else:
        rnd = random.Random()

    if max_leg_distance_m is None:
        max_leg_distance_m = min_distance_m * 1.35  # Örn: 100m için [100m - 135m] aralığı

    waypoints: List[Waypoint] = []
    points_to_avoid: List[Tuple[float, float]] = [(origin_lat, origin_lon)]

    cur_lat, cur_lon = origin_lat, origin_lon
    mission_id = f"msn_{vehicle_code}_{int(time.time())}_{rnd.randint(100, 999)}"

    # İlk yön için rastgele bir başlangıç açısı belirle
    base_angle = rnd.uniform(0.0, 360.0)

    for i in range(1, num_waypoints + 1):
        found = False
        attempts = 0
        cand_lat, cand_lon = cur_lat, cur_lon
        cand_dist = min_distance_m
        cand_bearing = 0.0

        # Geniş poligon rotası için hedef açı adımı (her noktada yön ~60° - 110° kayarak yay çizer)
        sector_center = (base_angle + (i - 1) * (360.0 / max(1, num_waypoints))) % 360.0

        while attempts < 1000:
            attempts += 1
            # Açı ve bacak mesafesi seçimi
            angle_jitter = rnd.uniform(-40.0, 40.0)
            cand_bearing = (sector_center + angle_jitter) % 360.0
            cand_dist = rnd.uniform(min_distance_m, max_leg_distance_m)

            # Kuzey - Doğu ötelemesi
            rad = math.radians(cand_bearing)
            dn = cand_dist * math.cos(rad)
            de = cand_dist * math.sin(rad)
            cand_lat, cand_lon = offset_gps_by_meters(cur_lat, cur_lon, dn, de)

            # 1. Başlangıç noktasına mesafe kontrolü (>= min_distance_m)
            _, _, d_origin, _ = haversine_distance_and_bearing(origin_lat, origin_lon, cand_lat, cand_lon)
            if d_origin < (min_distance_m * 0.98):
                continue

            # 2. Daha önce üretilmiş TÜM noktalara mesafe kontrolü (>= min_distance_m)
            valid_against_all = True
            for prev_lat, prev_lon in points_to_avoid:
                _, _, d_prev, _ = haversine_distance_and_bearing(prev_lat, prev_lon, cand_lat, cand_lon)
                if d_prev < (min_distance_m * 0.98):
                    valid_against_all = False
                    break

            if valid_against_all:
                found = True
                break

        # Eğer çok dar bir alanda sıkışırsa toleransı hafif esneterek garantile
        if not found:
            angle_rad = math.radians((sector_center) % 360.0)
            cand_dist = min_distance_m * 1.15
            cand_lat, cand_lon = offset_gps_by_meters(
                origin_lat, origin_lon, cand_dist * math.cos(angle_rad), cand_dist * math.sin(angle_rad)
            )
            _, _, cand_dist, cand_bearing = haversine_distance_and_bearing(cur_lat, cur_lon, cand_lat, cand_lon)
            _, _, d_origin, _ = haversine_distance_and_bearing(origin_lat, origin_lon, cand_lat, cand_lon)
        else:
            _, _, d_origin, _ = haversine_distance_and_bearing(origin_lat, origin_lon, cand_lat, cand_lon)

        wp = Waypoint(
            index=i,
            lat=round(cand_lat, 7),
            lon=round(cand_lon, 7),
            target_dist_m=round(cand_dist, 2),
            bearing_from_prev_deg=round(cand_bearing, 1),
            dist_to_origin_m=round(d_origin, 2),
        )
        waypoints.append(wp)
        points_to_avoid.append((cand_lat, cand_lon))
        cur_lat, cur_lon = cand_lat, cand_lon

    plan = MissionPlan(
        mission_id=mission_id,
        vehicle_code=vehicle_code,
        origin_lat=origin_lat,
        origin_lon=origin_lon,
        min_distance_m=min_distance_m,
        waypoints=waypoints,
    )
    return plan


def main():
    if sys.stdout and hasattr(sys.stdout, "reconfigure"):
        try:
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass

    parser = argparse.ArgumentParser(
        description="PiSim Otonom Görev ve Rota Oluşturucu (Dosya 1)"
    )
    parser.add_argument("--lat", type=float, default=39.9334, help="Başlangıç GPS Enlemi (varsayılan: 39.9334)")
    parser.add_argument("--lon", type=float, default=32.8597, help="Başlangıç GPS Boylamı (varsayılan: 32.8597)")
    parser.add_argument("--distance", "-d", type=float, default=100.0, help="Noktalar arası ve merkeze minimum mesafe (metre, varsayılan: 100.0)")
    parser.add_argument("--count", "-c", type=int, default=4, help="Üretilecek hedef nokta sayısı (varsayılan: 4)")
    parser.add_argument("--vehicle", "-v", type=str, default="rover_01", help="Araç kodu (varsayılan: rover_01)")
    parser.add_argument("--save", "-s", type=str, default=None, help="Görevi JSON dosyası olarak kaydet")
    parser.add_argument("--json", action="store_true", help="Yalnızca JSON çıktısı ver")

    args = parser.parse_args()

    plan = generate_random_mission(
        origin_lat=args.lat,
        origin_lon=args.lon,
        num_waypoints=args.count,
        min_distance_m=args.distance,
        vehicle_code=args.vehicle,
    )

    if args.json:
        print(plan.to_json())
    else:
        print("=" * 72)
        print(f"🎯 PISIM OTONOM GÖREV PLANI: {plan.mission_id}")
        print(f"   Araç: {plan.vehicle_code} | Asgari Mesafe Kuralı: {plan.min_distance_m:.1f} m")
        print(f"   Başlangıç Koordinatı: ({plan.origin_lat:.6f}°N, {plan.origin_lon:.6f}°E)")
        print("-" * 72)
        for wp in plan.waypoints:
            print(
                f"   [Hedef #{wp.index}] 📍 ({wp.lat:.6f}°N, {wp.lon:.6f}°E) | "
                f"Bacak: {wp.target_dist_m:5.1f}m (Yön: {wp.bearing_from_prev_deg:5.1f}°) | "
                f"Merkeze Mesafe: {wp.dist_to_origin_m:5.1f}m"
            )
        print("=" * 72)

    if args.save:
        if plan.save_file(args.save):
            print(f"💾 Görev başarıyla kaydedildi: {args.save}")


if __name__ == "__main__":
    main()
