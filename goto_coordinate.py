#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PiSim - Koordinata Otonom Sürüş ve Hareket Test Aracı (goto_coordinate.py)
==========================================================================
`pisim_motion.py` hareket kütüphanesini kullanarak seçili aracı:
  - Haritadan tıklanan veya girilen bir GPS koordinatına (--lat, --lon),
  - Yerel simülasyon koordinatına (--x, --y metre),
  - Eğrisel dönüş (curve) veya düz ilerleme (double-check) testlerine
otonom olarak sürer.

Kullanım Örnekleri:
  1. Belirli bir GPS koordinatına otonom sürüş (Pure Pursuit Eğrisel + Çift Doğrulama):
     python3 goto_coordinate.py --lat 39.933850 --lon 32.860420 --target 1

  2. Yerel (X, Y metre) koordinatına otonom sürüş:
     python3 goto_coordinate.py --x 30.0 --y 15.0 --target 1

  3. Eğrisel (Curve) dönüş testi (Yarıçap=6m, 90 derece sağa kavisli dönüş):
     python3 goto_coordinate.py --curve-radius 6.0 --curve-angle 90 --curve-dir right

  4. Çift doğrulamalı (GPS + Odometri) düz ilerleme testi (15 metre):
     python3 goto_coordinate.py --forward 15.0

  5. İnteraktif Konsol Modu (Argümansız çalıştırıldığında):
     python3 goto_coordinate.py
"""

import argparse
import signal
import sys
from pisim_motion import PiSimMotionController, delta_meters_to_gps


def main():
    parser = argparse.ArgumentParser(
        description="PiSim Otonom Koordinata Sürüş & Çift Doğrulamalı Hareket Test Scripti"
    )
    parser.add_argument(
        "--target", "-t", type=str, default="selected",
        help="Hedef araç kodu veya ID (örn: '1', 'RV01-8F3A9C', 'selected')"
    )
    parser.add_argument("--ip", type=str, default=None, help="Unreal Engine IP adresi (boşsa otomatik keşfedilir)")
    parser.add_argument("--lat", type=float, default=None, help="Hedef Enlem (Latitude, örn: 39.93385)")
    parser.add_argument("--lon", type=float, default=None, help="Hedef Boylam (Longitude, örn: 32.86040)")
    parser.add_argument("--x", type=float, default=None, help="Hedef Yerel X (Kuzey metre)")
    parser.add_argument("--y", type=float, default=None, help="Hedef Yerel Y (Doğu metre)")
    parser.add_argument("--speed", type=float, default=1.8, help="Seyir hızı (m/s, varsayılan: 1.8)")
    parser.add_argument(
        "--mode", type=str, choices=["curve", "point-turn"], default="curve",
        help="Sürüş modu: 'curve' (Eğrisel Pure-Pursuit) veya 'point-turn' (Önce dön sonra düz git)"
    )
    parser.add_argument("--tolerance", type=float, default=0.85, help="Hedefe varış toleransı (metre)")

    # Doğrudan hareket kütüphanesi test bayrakları
    parser.add_argument("--forward", type=float, default=None, help="Çift doğrulamalı düz ilerleme mesafesi (metre)")
    parser.add_argument("--turn", type=float, default=None, help="Yerinde dönüş açısı (derece, +sağ / -sol)")
    parser.add_argument("--curve-radius", type=float, default=None, help="Eğrisel (Curve) dönüş yarıçapı (metre)")
    parser.add_argument("--curve-angle", type=float, default=90.0, help="Eğrisel (Curve) dönüş yay açısı (derece)")
    parser.add_argument("--curve-dir", type=str, default="right", help="Eğrisel dönüş yönü (right / left)")

    args = parser.parse_args()

    motion = PiSimMotionController(target_vehicle=args.target, ue5_ip=args.ip)

    def handle_sigint(sig, frame):
        print("\n🛑 [ACİL DURDURMA] Ctrl+C algılandı! Araç durduruluyor...")
        motion.request_abort()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)

    print("=" * 78)
    print(" 🛰️  PISIM OTONOM NAVİGASYON VE ÇİFT DOĞRULAMALI HAREKET SİSTEMİ")
    print("=" * 78)

    telem = motion.wait_for_telemetry(timeout_sec=4.0)
    if telem:
        print(
            f" ✅ Bağlı Araç : {telem['name']} [{telem['code']}] (ID: {telem['id']}, Özel Port: {motion.dedicated_port})\n"
            f" 📍 Mevcut GPS : {telem['lat']:.6f}° N, {telem['lon']:.6f}° E (İrtifa: {telem['altitude']:.1f}m)\n"
            f" 📐 Yerel Konum: X={telem['pos_x']:.2f}m, Y={telem['pos_y']:.2f}m, Z={telem['pos_z']:.2f}m | Yaw={telem['yaw']:.1f}°"
        )
    else:
        print(" ⚠️  Uyarı: Henüz canlı telemetri paketi alınamadı, bağlantı bekleniyor...")

    print("-" * 78)

    # 1. GPS Koordinatına Sürüş (--lat ve --lon verilmişse)
    if args.lat is not None and args.lon is not None:
        res = motion.navigate_to_gps(
            target_lat=args.lat,
            target_lon=args.lon,
            cruise_speed=args.speed,
            mode=args.mode,
            arrival_tolerance_m=args.tolerance,
            status_callback=print,
        )
        sys.exit(0 if res.get("ok") else 1)

    # 2. Yerel Metre Koordinatına Sürüş (--x ve --y verilmişse)
    if args.x is not None and args.y is not None:
        if not telem:
            print("❌ Yerel koordinat hesabı için başlangıç telemetrisi gerekli!")
            sys.exit(1)
        dx = args.x - telem["pos_x"]
        dy = args.y - telem["pos_y"]
        t_lat, t_lon = delta_meters_to_gps(telem["lat"], telem["lon"], dx, dy)
        res = motion.navigate_to_gps(
            target_lat=t_lat,
            target_lon=t_lon,
            cruise_speed=args.speed,
            mode=args.mode,
            arrival_tolerance_m=args.tolerance,
            status_callback=print,
        )
        sys.exit(0 if res.get("ok") else 1)

    # 3. Eğrisel (Curve) Dönüş Testi
    if args.curve_radius is not None:
        res = motion.drive_curve(
            radius_m=args.curve_radius,
            arc_angle_deg=args.curve_angle,
            speed_mps=args.speed,
            direction=args.curve_dir,
            status_callback=print,
        )
        sys.exit(0 if res.get("ok") else 1)

    # 4. Çift Doğrulamalı Düz İlerleme Testi
    if args.forward is not None:
        res = motion.move_forward(
            distance_m=args.forward,
            speed_mps=args.speed,
            tolerance_m=0.25,
            status_callback=print,
        )
        sys.exit(0 if res.get("ok") else 1)

    # 5. Yerinde Dönüş Testi
    if args.turn is not None:
        res = motion.turn_angle(
            delta_deg=args.turn,
            status_callback=print,
        )
        sys.exit(0 if res.get("ok") else 1)

    # 6. Argüman verilmediyse İnteraktif Otonom Komut Kabuğu
    print(" 💡 İnteraktif Otonom Sürüş Konsolu Aktif! Komutlar:")
    print("    • gps <enlem> <boylam>        -> Belirtilen GPS koordinatına otonom sür")
    print("    • git <dx_metre> <dy_metre>   -> Mevcut konumdan (dX Kuzey, dY Doğu) metre uzağa sür")
    print("    • ileri <metre>               -> Çift doğrulamalı (GPS+Odom) düz ilerle")
    print("    • curve <yaricap_m> <aci_deg> -> Eğrisel (kavisli) dönüş yap (örn: curve 5 90)")
    print("    • don <aci_deg>               -> Yerinde belirtilen açı kadar dön")
    print("    • durum                       -> Anlık GPS + Odometri telemetrisini yazdır")
    print("    • dur                         -> Aracı acil durdur")
    print("    • q                           -> Çıkış")
    print("-" * 78)

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
        elif cmd == "durum":
            t = motion.get_telemetry()
            if t:
                print(
                    f"📡 [{t['code']}] GPS: ({t['lat']:.6f}°, {t['lon']:.6f}°) | "
                    f"Yerel: X={t['pos_x']:.2f}m Y={t['pos_y']:.2f}m Z={t['pos_z']:.2f}m | "
                    f"Yaw={t['yaw']:.1f}° | Hız={t['speed_kmh']:.1f} km/s"
                )
            else:
                print("⚠️ Telemetri alınamadı.")
        elif cmd == "gps" and len(parts) >= 3:
            lat = float(parts[1])
            lon = float(parts[2])
            motion.navigate_to_gps(lat, lon, cruise_speed=args.speed, status_callback=print)
        elif cmd == "git" and len(parts) >= 3:
            dx = float(parts[1])
            dy = float(parts[2])
            t = motion.get_telemetry()
            if t:
                t_lat, t_lon = delta_meters_to_gps(t["lat"], t["lon"], dx, dy)
                motion.navigate_to_gps(t_lat, t_lon, cruise_speed=args.speed, status_callback=print)
            else:
                print("⚠️ Başlangıç telemetrisi alınamadı!")
        elif cmd == "ileri" and len(parts) >= 2:
            dist = float(parts[1])
            motion.move_forward(dist, speed_mps=args.speed, status_callback=print)
        elif cmd == "curve" and len(parts) >= 3:
            r = float(parts[1])
            ang = float(parts[2])
            d = parts[3] if len(parts) >= 4 else "right"
            motion.drive_curve(r, ang, speed_mps=args.speed, direction=d, status_callback=print)
        elif cmd == "don" and len(parts) >= 2:
            ang = float(parts[1])
            motion.turn_angle(ang, status_callback=print)
        else:
            print("❓ Bilinmeyen komut. Örnek: 'gps 39.9338 32.8602', 'git 20 10', 'ileri 10', 'curve 5 90', 'don 45'")


if __name__ == "__main__":
    main()
