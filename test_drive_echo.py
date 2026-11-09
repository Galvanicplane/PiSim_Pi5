#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_drive_echo.py - PiSim Canlı Sürüş & Echo Test Doğrulayıcısı
================================================================
Bu script, Unreal Engine'deki aracın Zenoh üzerinden komut alıp almadığını
ve hareket edip etmediğini 3 saniyede doğrudan test eder.

Nasıl Çalışır?
  1. Zenoh'a bağlanır (Auto-Discovery ile IP gerekmez).
  2. Hedef araca (varsayılan rover_01) 2 saniye boyunca test sürüş komutu basar
     (Sağa direksiyon + gaz).
  3. Aynı anda Unreal'dan dönen `gps` ve `imu` verilerini dinler.
  4. Aracın pusula açısındaki (Yaw) veya hızındaki değişimi ölçerek:
     - "Unreal komutu aldı mı?"
     - "Araç fiziksel olarak tepki verdi mi?"
     sorularını ekranda yeşil/kırmızı olarak net bir şekilde doğrular.

Kullanım:
  python test_drive_echo.py
  python test_drive_echo.py --vehicle rover_01
"""

import argparse
import json
import math
import sys
import time
from typing import Optional

try:
    import zenoh
    HAS_ZENOH = True
except ImportError:
    HAS_ZENOH = False

from agent_supervisor import auto_discover_zenoh_endpoint, normalize_zenoh_endpoint


def main():
    if sys.stdout and hasattr(sys.stdout, "reconfigure"):
        try:
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass

    parser = argparse.ArgumentParser(description="PiSim Canlı Sürüş ve Echo Testi")
    parser.add_argument("--vehicle", "-v", type=str, default="rover_01", help="Test edilecek araç (varsayılan: rover_01)")
    parser.add_argument("--router", "-r", type=str, default=None, help="İsteğe bağlı Zenoh router adresi")
    args = parser.parse_args()

    v_code = args.vehicle
    print("\n" + "=" * 65)
    print(f"🚗 PISIM CANLI SÜRÜŞ & ECHO DOĞRULAYICI: [{v_code}]")
    print("=" * 65)

    if not HAS_ZENOH:
        print("❌ 'eclipse-zenoh' modülü bulunamadı!")
        sys.exit(1)

    # 1. Otomatik Ağ Keşfi
    ep = args.router or auto_discover_zenoh_endpoint(verbose=False)
    conf = zenoh.Config()
    if ep:
        norm_ep = normalize_zenoh_endpoint(ep)
        conf.insert_json5("mode", json.dumps("client"))
        conf.insert_json5("connect/endpoints", json.dumps([norm_ep]))
        print(f"🔗 Bağlanılan Zenoh Noktası: {norm_ep}")
    else:
        conf.insert_json5("mode", json.dumps("peer"))
        print("🌐 Zenoh P2P Dağınık Ağ Modu Açıldı (Multicast)")

    session = zenoh.open(conf)
    print("✅ Zenoh Oturumu Başarıyla Açıldı.")

    # 2. Sensör Dinleyicileri (Akıllı Kod Eşleştirme)
    actual_vehicle_code = v_code
    initial_yaw: Optional[float] = None
    latest_yaw: Optional[float] = None
    latest_speed: float = 0.0
    received_imu_count = 0
    received_gps_count = 0

    def on_imu_wildcard(sample):
        nonlocal initial_yaw, latest_yaw, received_imu_count, actual_vehicle_code
        try:
            parts = str(sample.key_expr).split("/")
            if len(parts) >= 3 and parts[0] == "pisim":
                cand_code = parts[1]
                if v_code.lower() == "auto" or cand_code == v_code:
                    actual_vehicle_code = cand_code
                    data = json.loads(sample.payload.to_string())
                    q = data.get("orientation", {})
                    qx, qy, qz, qw = q.get("x", 0.0), q.get("y", 0.0), q.get("z", 0.0), q.get("w", 1.0)
                    yaw_rad = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
                    deg = math.degrees(yaw_rad)
                    if deg < 0: deg += 360.0
                    if initial_yaw is None:
                        initial_yaw = deg
                    latest_yaw = deg
                    received_imu_count += 1
        except Exception:
            pass

    def on_gps_wildcard(sample):
        nonlocal received_gps_count, latest_speed, actual_vehicle_code
        try:
            parts = str(sample.key_expr).split("/")
            if len(parts) >= 3 and parts[0] == "pisim":
                cand_code = parts[1]
                if v_code.lower() == "auto" or cand_code == v_code:
                    actual_vehicle_code = cand_code
                    data = json.loads(sample.payload.to_string())
                    received_gps_count += 1
                    latest_speed = float(data.get("speed", 0.0))
        except Exception:
            pass

    session.declare_subscriber("pisim/*/imu", on_imu_wildcard)
    session.declare_subscriber("pisim/*/gps", on_gps_wildcard)

    # 3. Önce Mevcut Sensör Durumunu Kontrol Et (1 saniye)
    print(f"\n📡 1. Adım: Unreal Engine sensörleri dinleniyor ('{v_code}' aranıyor)...")
    time.sleep(1.2)

    if received_imu_count == 0 and received_gps_count == 0:
        print(f"❌ UYARI: Unreal Engine'den '{v_code}' ile eşleşen HİÇBİR SENSÖR GELMEDİ!")
        print("   Olası Sebepler:")
        print("   1) Unreal Engine açık değil veya 'Play' tuşuna basılmadı.")
        print("   2) Sahnede aktif olan aracın kodu farklı.")
        print("   3) Windows'ta 'zenoh_mesh.py' çalışmıyor.")
        session.close()
        return

    v_code = actual_vehicle_code
    print(f"  └── 🟢 Sensörler Alınıyor! Hedef: [{v_code}] (IMU: {received_imu_count} paket, Başlangıç Açısı: {initial_yaw:.1f}°)")

    # 4. Sürüş Test Komutu Gönder (Sağa %50 Direksiyon + %60 Gaz)
    print(f"\n🕹️ 2. Adım: Unreal'a Test Sürüş Darbesi Gönderiliyor (Sağa Direksiyon: +0.6, Gaz: %60)...")

    # Hem actuators hem de cmd_vel formatında bas
    test_motors = {
        "M_Steer_FL": 0.60,
        "M_Steer_FR": 0.60,
        "M_Wheel_FL": 0.60,
        "M_Wheel_FR": 0.40,
        "M_Wheel_RL": 0.60,
        "M_Wheel_RR": 0.40,
    }
    test_twist = {
        "linear": {"x": 2.0, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.5},
    }

    t0 = time.time()
    while (time.time() - t0) < 2.5:
        session.put(f"pisim/{v_code}/actuators", json.dumps(test_motors))
        session.put(f"pisim/{v_code}/cmd_vel", json.dumps(test_twist))
        time.sleep(0.08)

    # Motorları tekrar sıfırla (Durdur)
    stop_motors = {"M_Steer_FL": 0.0, "M_Steer_FR": 0.0, "M_Wheel_FL": 0.0, "M_Wheel_FR": 0.0, "M_Wheel_RL": 0.0, "M_Wheel_RR": 0.0}
    stop_twist = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
    session.put(f"pisim/{v_code}/actuators", json.dumps(stop_motors))
    session.put(f"pisim/{v_code}/cmd_vel", json.dumps(stop_twist))

    # 5. Sonuç Değerlendirmesi
    time.sleep(0.5)
    yaw_delta = abs((latest_yaw or 0.0) - (initial_yaw or 0.0))
    if yaw_delta > 180.0:
        yaw_delta = 360.0 - yaw_delta

    print("\n" + "=" * 65)
    print("📊 TEST SONUÇ RAPORU:")
    print("-----------------------------------------------------")
    print(f"  * Başlangıç Pusula Açısı : {initial_yaw:.1f}°")
    print(f"  * Bitiş Pusula Açısı     : {latest_yaw:.1f}°")
    print(f"  * Açı Değişimi (Delta)   : {yaw_delta:.2f}°")
    print(f"  * Son Hız                : {latest_speed:.2f} m/s")

    if yaw_delta > 0.8 or latest_speed > 0.1:
        print("-----------------------------------------------------")
        print("  🎉 TEBRİKLER! Unreal Engine komutları aldı ve araç FİZİKSEL OLARAK HAREKET ETTİ!")
        print("  ✅ İletişim zinciri (Zenoh -> TopicBridge -> Eyleyiciler) kusursuz çalışıyor.")
    else:
        print("-----------------------------------------------------")
        print("  ⚠️ DİKKAT: Unreal Engine sensör verisi gönderiyor ANCAK araç henüz hareket etmedi.")
        print("  Muhtemel Nedenler:")
        print("  1) Unreal'da aracın el freni (Handbrake) çekili olabilir.")
        print("  2) Aracın tekerlek kemik isimleri (M_Wheel_FL vb.) modelde farklı adlandırılmış olabilir.")
        print("  3) zenoh_mesh.py'nin çalıştığından emin olun.")
    print("=" * 65 + "\n")

    session.close()


if __name__ == "__main__":
    main()

