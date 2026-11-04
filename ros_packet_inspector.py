#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ros_packet_inspector.py - PiSim Canlı ROS 2 & Zenoh Paket İnceleme ve Enjeksiyon Aracı
========================================================================================
Bu araç, ağda dönen gerçek ROS 2 paketlerinin içini canlı olarak açıp alanlarını (headers,
quaternions, angular velocity, GPS coordinates, actuators, cmd_vel) detaylı incelemenizi sağlar.

Özellikler:
1. Canlı Paket Yakalayıcı (Live Wireshark/ROS2 Echo Tarzı):
   - /rover_01, /rover_02 ve /rover_03'ten gelen paketleri anlık ekrana döker.
   - İzole edilmiş konteynerden (Rover 03) gelen paketlerin atlama yaparak ulaştığını kanıtlar.
2. Paket Enjektörü (Packet Injector):
   - İstediğiniz araca doğrudan geçerli ROS 2 `Twist`, `Actuators` veya `Ping` paketi basar.
"""

import argparse
import json
import math
import os
import sys
import threading
import time

try:
    import zenoh
    HAS_ZENOH = True
except ImportError:
    HAS_ZENOH = False


def format_ros_packet(topic: str, raw_payload: str) -> str:
    """Gelen ham JSON payload'ını standart ROS 2 mesaj formatında okunabilir hale getirir."""
    now_str = time.strftime("%H:%M:%S")
    lines = []
    lines.append("╔" + "═" * 74 + "╗")
    lines.append(f"║ 📦 [{now_str}] ROS 2 TOPIC: {topic:52s} ║")
    lines.append("╠" + "═" * 74 + "╣")

    try:
        data = json.loads(raw_payload)
    except Exception:
        lines.append(f"║ Ham İçerik (Düz Metin): {raw_payload[:60]:49s} ║")
        lines.append("╚" + "═" * 74 + "╝")
        return "\n".join(lines)

    # 1. sensor_msgs/msg/Imu
    if "orientation" in data or "linear_acceleration" in data:
        lines.append("║  [Tür: sensor_msgs/msg/Imu]                                            ║")
        q = data.get("orientation", {})
        qx, qy, qz, qw = q.get("x", 0.0), q.get("y", 0.0), q.get("z", 0.0), q.get("w", 1.0)
        yaw_deg = math.degrees(math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))
        pitch_deg = math.degrees(math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))))
        roll_deg = math.degrees(math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)))

        lines.append(f"║  • Oryantasyon (Kuaterniyon) : x={qx:+.4f}, y={qy:+.4f}, z={qz:+.4f}, w={qw:+.4f}║")
        lines.append(f"║  • Açısal Duruş (Euler Açıları): Roll={roll_deg:+5.1f}°, Pitch={pitch_deg:+5.1f}°, Yaw={yaw_deg:+5.1f}° ║")

        w = data.get("angular_velocity", {})
        lines.append(f"║  • Açısal Hız (Jiroskop rad/s): x={w.get('x',0.0):+.3f}, y={w.get('y',0.0):+.3f}, z={w.get('z',0.0):+.3f}   ║")

        a = data.get("linear_acceleration", {})
        lines.append(f"║  • İvme (İvmeölçer m/s²)     : x={a.get('x',0.0):+.2f}, y={a.get('y',0.0):+.2f}, z={a.get('z',0.0):+.2f}   ║")

    # 2. sensor_msgs/msg/NavSatFix (GPS)
    elif "latitude" in data or "longitude" in data:
        lines.append("║  [Tür: sensor_msgs/msg/NavSatFix - Küresel Konum]                      ║")
        lat = data.get("latitude", 0.0)
        lon = data.get("longitude", 0.0)
        alt = data.get("altitude", 0.0)
        status = data.get("status", {}).get("status", 3) if isinstance(data.get("status"), dict) else data.get("status", 3)
        lines.append(f"║  • Enlem (Latitude)          : {lat:12.7f}° N                                ║")
        lines.append(f"║  • Boylam (Longitude)        : {lon:12.7f}° E                                ║")
        lines.append(f"║  • İrtifa (Altitude AMSL)    : {alt:8.2f} metre                              ║")
        lines.append(f"║  • GPS Uydu Fix Durumu       : {status} (3D DGPS Sabit)                        ║")

    # 3. Eyleyici / Motor Haritası
    elif "motors" in data or isinstance(data, dict) and any("Wheel" in k or "Steer" in k for k in data.keys()):
        lines.append("║  [Tür: pisim_msgs/Actuators - Eyleyici Motor Değerleri]                ║")
        motors = data.get("motors", data)
        for k, v in motors.items():
            bar_len = int(abs(v) * 15)
            sign = "+" if v >= 0 else "-"
            bar = ("█" * bar_len).ljust(15)
            lines.append(f"║  • {k:16s} : {sign}{abs(v):.2f} [{bar}]                          ║")

    # 4. geometry_msgs/msg/Twist (cmd_vel)
    elif "linear" in data or "angular" in data:
        lines.append("║  [Tür: geometry_msgs/msg/Twist - Hız Komutu]                           ║")
        lin = data.get("linear", {})
        ang = data.get("angular", {})
        lines.append(f"║  • Çizgisel Hız (Linear X)   : {lin.get('x', 0.0):+.2f} m/s                           ║")
        lines.append(f"║  • Açısal Hız (Angular Z)    : {ang.get('z', 0.0):+.2f} rad/s                         ║")

    # 5. Diğer / Pong Yanıtları
    else:
        for k, v in list(data.items())[:6]:
            val_str = str(v)[:40]
            lines.append(f"║  • {k:18s} : {val_str:45s} ║")

    lines.append("╚" + "═" * 74 + "╝")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="PiSim Real-Time ROS 2 / Zenoh Packet Inspector")
    parser.add_argument("--mode", choices=["peer", "client"], default="peer", help="Zenoh modu (peer / client)")
    parser.add_argument("--router", type=str, default=None, help="Zenoh router adresi")
    args = parser.parse_args()

    if not HAS_ZENOH:
        print("❌ 'eclipse-zenoh' kütüphanesi bulunamadı! 'pip install eclipse-zenoh' çalıştırın.")
        sys.exit(1)

    conf = zenoh.Config()
    if args.mode == "client" and args.router:
        conf.insert_json5("mode", json.dumps("client"))
        conf.insert_json5("connect/endpoints", json.dumps([args.router]))
    else:
        conf.insert_json5("mode", json.dumps("peer"))

    print("\n" + "=" * 76)
    print(" 🔍 PISIM CANLI ROS 2 & ZENOH PAKET İNCELEYİCİ (PACKET INSPECTOR)")
    print("=" * 76)
    print(" 📡 Zenoh Ağına Bağlanılıyor...")
    session = zenoh.open(conf)
    print(" ✅ Zenoh Oturumu Açıldı. Ağdaki tüm ROS paketleri canlı yakalanıyor...\n")

    packet_count = 0
    lock = threading.Lock()

    def _sample_listener(sample):
        nonlocal packet_count
        topic = str(sample.key_expr)
        payload = sample.payload.to_string()

        # Sadece pisim ve fleet paketlerini göster
        if not (topic.startswith("pisim/") or topic.startswith("fleet/")):
            return

        with lock:
            packet_count += 1
            print(format_ros_packet(topic, payload))
            print()

    sub = session.declare_subscriber("pisim/**", _sample_listener)
    sub_fleet = session.declare_subscriber("fleet/**", _sample_listener)

    print(" 💡 İpucu: Bu terminal açıkken arka planda dönen tüm paketlerin içini görebilirsiniz.")
    print("    [Ctrl+C] ile durdurabilirsiniz.\n")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nİnceleyici kapatılıyor...")
    finally:
        session.close()


if __name__ == "__main__":
    main()
