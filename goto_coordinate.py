#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PiSim - Koordinata Otonom Sürüş ve Hareket Test Aracı (goto_coordinate.py)
==========================================================================
Bu script, `pisim_motion.py` içerisindeki `PiSimMotionController` hareket kütüphanesini
kullanarak seçili aracı gerçek sensörlerle (GPS + IMU Pusula + Hız Entegrasyonu) ve
ArduPilot L1 geniş dönüş yarıçaplı (Turn-Radius) güdüm yasasıyla hedefe sürer.

Özellikler:
  - Hiçbir simülasyon hilesi (Unreal pos_x, pos_y ground-truth) kullanmaz.
  - Sadece gerçek bir araçta bulunan GPS (lat, lon, alt, speed) ve IMU/Pusula (roll, pitch, yaw) verilerini okur.
  - Asla kare dönüş yapmaz; belirlenen geniş dönüş yarıçapıyla (--turn-radius, varsayılan 5.0m) yay çizerek döner.

Kullanım Örnekleri:
  1. Belirli bir GPS koordinatına geniş kavisli (L1 Guidance) otonom sürüş:
     python3 goto_coordinate.py --lat 39.933850 --lon 32.860420 --target 1 --turn-radius 6.0

  2. Mevcut GPS konumundan (Kuzey, Doğu metre) ötedeki GPS koordinatına otonom sürüş:
     python3 goto_coordinate.py --north 25.0 --east 15.0 --turn-radius 5.0

  3. Geniş yarıçaplı eğrisel (Curve) dönüş testi (Yarıçap=6m, 90 derece sağa kavisli dönüş):
     python3 goto_coordinate.py --curve-radius 6.0 --curve-angle 90 --curve-dir right

  4. Gerçek sensör çift doğrulamalı (GPS + IMU Dead-Reckoning) düz ilerleme testi (15 metre):
     python3 goto_coordinate.py --forward 15.0

  5. İnteraktif Konsol Modu (Argümansız çalıştırıldığında):
     python3 goto_coordinate.py
"""

import argparse
import signal
import sys
from pisim_motion import (
    PiSimMotionController,
    haversine_distance_and_bearing,
    offset_gps_by_meters,
)


def main():
    parser = argparse.ArgumentParser(
        description="PiSim ArduPilot L1 Otonom Koordinata Sürüş & Gerçek Sensör Hareket Aracı"
    )
    parser.add_argument(
        "--target", "-t", type=str, default="selected",
        help="Hedef araç kodu veya ID (örn: '1', 'RV01-8F3A9C', 'selected')"
    )
    parser.add_argument("--ip", type=str, default=None, help="Unreal Engine IP adresi (boşsa otomatik keşfedilir)")
    parser.add_argument("--lat", type=float, default=None, help="Hedef GPS Enlem (Latitude, örn: 39.933850)")
    parser.add_argument("--lon", type=float, default=None, help="Hedef GPS Boylam (Longitude, örn: 32.860420)")
    parser.add_argument("--north", type=float, default=None, help="Mevcut GPS'ten Kuzey yönünde hedef mesafe (metre)")
    parser.add_argument("--east", type=float, default=None, help="Mevcut GPS'ten Doğu yönünde hedef mesafe (metre)")
    parser.add_argument("--speed", type=float, default=1.8, help="Seyir hızı (m/s, varsayılan: 1.8)")
    parser.add_argument(
        "--turn-radius", type=float, default=5.0,
        help="Minimum geniş dönüş yarıçapı (metre, varsayılan: 5.0m - kare dönüşü engeller)"
    )
    parser.add_argument(
        "--wp-radius", type=float, default=1.5,
        help="ArduPilot WP_RADIUS hedefe varış kabul yarıçapı (metre, varsayılan: 1.5m)"
    )

    # Doğrudan hareket kütüphanesi (pisim_motion.py) test bayrakları
    parser.add_argument("--forward", type=float, default=None, help="GPS + IMU INS çift doğrulamalı düz ilerleme (metre)")
    parser.add_argument("--curve-radius", type=float, default=None, help="Geniş kavisli (Curve) dönüş yarıçapı (metre)")
    parser.add_argument("--curve-angle", type=float, default=90.0, help="Geniş kavisli (Curve) dönüş yay açısı (derece)")
    parser.add_argument("--curve-dir", type=str, default="right", help="Kavisli dönüş yönü (right / left)")

    args = parser.parse_args()

    # Tüm hareket ve güdüm işlemleri pisim_motion.py içerisindeki PiSimMotionController sınıfı üzerinden yürütülür
    motion = PiSimMotionController(
        target_vehicle=args.target,
        ue5_ip=args.ip,
        min_turn_radius_m=args.turn_radius,
    )

    def handle_sigint(sig, frame):
        print("\n🛑 [ACİL DURDURMA] Ctrl+C algılandı! Araç durduruluyor...")
        motion.request_abort()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)

    print("=" * 80)
    print(" 🛰️  PISIM ARDUPILOT L1 OTONOM NAVİGASYON (SADECE GERÇEK GPS + IMU SENSÖRLERİ)")
    print("=" * 80)

    telem = motion.wait_for_telemetry(timeout_sec=4.0)
    if telem:
        print(
            f" ✅ Bağlı Araç   : {telem['name']} [{telem['code']}] (ID: {telem['id']}, Özel Port: {motion.dedicated_port})\n"
            f" 📍 Mevcut GPS   : {telem['lat']:.6f}° N, {telem['lon']:.6f}° E (İrtifa: {telem['altitude']:.1f}m)\n"
            f" 🧭 IMU / Pusula : Yaw={telem['yaw']:.1f}° | Roll={telem['roll']:.1f}° | Pitch={telem['pitch']:.1f}°\n"
            f" ↪️ Dönüş Yarıçapı: Min R = {motion.min_turn_radius_m:.1f}m (Kare dönüş kapalı, geniş yay aktif)"
        )
    else:
        print(" ⚠️  Uyarı: Henüz canlı sensör paketi alınamadı, bağlantı bekleniyor...")

    print("-" * 80)

    # 1. GPS Koordinatına Geniş Kavisli (L1 Guidance) Sürüş (--lat ve --lon verilmişse)
    if args.lat is not None and args.lon is not None:
        res = motion.navigate_to_gps(
            target_lat=args.lat,
            target_lon=args.lon,
            cruise_speed=args.speed,
            turn_radius_m=args.turn_radius,
            wp_radius_m=args.wp_radius,
            status_callback=print,
        )
        sys.exit(0 if res.get("ok") else 1)

    # 2. Mevcut GPS Konumundan (Kuzey/Doğu Metre) Ötedeki GPS Noktasına Sürüş
    if args.north is not None or args.east is not None:
        if not telem:
            print("❌ Başlangıç GPS koordinatı okunamadığı için bağıl hedef hesaplanamadı!")
            sys.exit(1)
        dn = float(args.north or 0.0)
        de = float(args.east or 0.0)
        t_lat, t_lon = offset_gps_by_meters(telem["lat"], telem["lon"], dn, de)
        res = motion.navigate_to_gps(
            target_lat=t_lat,
            target_lon=t_lon,
            cruise_speed=args.speed,
            turn_radius_m=args.turn_radius,
            wp_radius_m=args.wp_radius,
            status_callback=print,
        )
        sys.exit(0 if res.get("ok") else 1)

    # 3. Geniş Yarıçaplı Eğrisel (Curve) Dönüş Testi
    if args.curve_radius is not None:
        res = motion.drive_curve(
            turn_radius_m=args.curve_radius,
            arc_angle_deg=args.curve_angle,
            speed_mps=args.speed,
            direction=args.curve_dir,
            status_callback=print,
        )
        sys.exit(0 if res.get("ok") else 1)

    # 4. Gerçek Sensör Çift Doğrulamalı (GPS + IMU Dead-Reckoning) Düz İlerleme Testi
    if args.forward is not None:
        res = motion.move_forward(
            distance_m=args.forward,
            speed_mps=args.speed,
            status_callback=print,
        )
        sys.exit(0 if res.get("ok") else 1)

    # 5. Argüman verilmediyse İnteraktif Otonom Komut Kabuğu
    print(" 💡 İnteraktif Otonom Sürüş Konsolu Aktif! Komutlar:")
    print("    • gps <enlem> <boylam> [turn_r]  -> Belirtilen GPS koordinatına geniş kavisle (L1) sür")
    print("    • git <kuzey_m> <dogu_m>         -> Mevcut GPS'ten (Kuzey, Doğu) metre öteye sür")
    print("    • ileri <metre>                  -> Çift doğrulamalı (GPS + IMU INS) düz ilerle")
    print("    • curve <yaricap_m> <aci_deg>    -> Geniş yarıçaplı yay çizerek dön (örn: curve 6 90 right)")
    print("    • yaricap <metre>                -> Minimum dönüş yarıçapını değiştir (örn: yaricap 8.0)")
    print("    • durum                          -> Anlık gerçek GPS + IMU Pusula sensörlerini yazdır")
    print("    • dur                            -> Aracı acil durdur")
    print("    • q                              -> Çıkış")
    print("-" * 80)

    while True:
        try:
            cmd_line = input("PiSim-Otonom> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not cmd_line:
            continue
        parts = cmd_line.split()
        cmd = parts[0].lower()

        if cmd in ("q", "quit", "exit"):
            motion.stop()
            break
        elif cmd == "dur":
            motion.stop()
            print("🛑 Araç durduruldu.")
        elif cmd == "yaricap" and len(parts) >= 2:
            motion.min_turn_radius_m = max(1.5, float(parts[1]))
            print(f"↪️ Minimum dönüş yarıçapı (Turn Radius) {motion.min_turn_radius_m:.1f}m olarak güncellendi.")
        elif cmd == "durum":
            t = motion.get_sensor_telemetry()
            if t:
                print(
                    f"📡 [{t['code']}] GPS: ({t['lat']:.6f}°N, {t['lon']:.6f}°E, İrtifa={t['altitude']:.1f}m) | "
                    f"Pusula Yaw={t['yaw']:.1f}° (Roll={t['roll']:.1f}°, Pitch={t['pitch']:.1f}°) | "
                    f"Hız={t['speed_kmh']:.1f} km/s"
                )
            else:
                print("⚠️ Sensör telemetrisi alınamadı.")
        elif cmd == "gps" and len(parts) >= 3:
            lat = float(parts[1])
            lon = float(parts[2])
            r_custom = float(parts[3]) if len(parts) >= 4 else motion.min_turn_radius_m
            motion.navigate_to_gps(
                lat, lon,
                cruise_speed=args.speed,
                turn_radius_m=r_custom,
                wp_radius_m=args.wp_radius,
                status_callback=print,
            )
        elif cmd == "git" and len(parts) >= 3:
            dn = float(parts[1])
            de = float(parts[2])
            t = motion.get_sensor_telemetry()
            if t:
                t_lat, t_lon = offset_gps_by_meters(t["lat"], t["lon"], dn, de)
                _, _, dist_m, brg = haversine_distance_and_bearing(t["lat"], t["lon"], t_lat, t_lon)
                print(f"🧭 Hesaplanan Hedef GPS: ({t_lat:.6f}°N, {t_lon:.6f}°E) | Mesafe: {dist_m:.1f}m | Açı: {brg:.1f}°")
                motion.navigate_to_gps(
                    t_lat, t_lon,
                    cruise_speed=args.speed,
                    turn_radius_m=motion.min_turn_radius_m,
                    wp_radius_m=args.wp_radius,
                    status_callback=print,
                )
            else:
                print("⚠️ Başlangıç GPS verisi alınamadı!")
        elif cmd == "ileri" and len(parts) >= 2:
            dist = float(parts[1])
            motion.move_forward(dist, speed_mps=args.speed, status_callback=print)
        elif cmd == "curve" and len(parts) >= 3:
            r = float(parts[1])
            ang = float(parts[2])
            d = parts[3] if len(parts) >= 4 else "right"
            motion.drive_curve(
                turn_radius_m=r,
                arc_angle_deg=ang,
                speed_mps=args.speed,
                direction=d,
                status_callback=print,
            )
        else:
            print("❓ Bilinmeyen komut. Örnek: 'gps 39.93385 32.86042 6.0', 'git 20 10', 'ileri 15', 'curve 6 90 right'")


if __name__ == "__main__":
    main()
