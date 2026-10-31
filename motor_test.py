#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PiSim - Raspberry Pi 5 Motor & Servo Bağımsız Test Aracı (motor_test.py)
========================================================================
Bu araç, Unreal Engine simülasyonundaki aracın her bir motorunu, tekerleğini
ve direksiyon servosunu bağımsız olarak Pi 5 üzerinden UDP ile test etmenizi sağlar.

Özellikler:
  1. Direksiyon Testi : Ön servo açısını derece cinsinden (-35° .. +35°) hassas kontrol
  2. Tekil Tekerlek    : FL, FR, RL, RR tekerleklerini veya herhangi bir kemik motorunu tek tek döndürme
  3. Diferansiyel Hız : Sol ve sağ tekerleklere farklı hızlar vererek dönüş kabiliyetini sınama
  4. Rampa & Süpürme  : Direksiyon açı taraması (Sweep) ve tekerlek hız rampa (Ramp) testleri
  5. Çift Modlu       : Hem argümanlarla tek satırda hem de interaktif REPL konsolunda çalışabilir

Kullanım Örnekleri:
  1. İnteraktif Konsol Modu:
     python3 motor_test.py

  2. Direksiyonu +25 dereceye çevirme (2 saniye):
     python3 motor_test.py --steer 25 --time 2.0

  3. Tüm tekerlekleri ileri %40 güçle döndürme:
     python3 motor_test.py --drive 0.4 --time 3.0

  4. Sadece Sol Ön Tekerleği (FL) döndürme:
     python3 motor_test.py --wheel fl --val 0.5 --time 2.0

  5. Diferansiyel Hız Testi (Sol: %30, Sağ: %70):
     python3 motor_test.py --diff 0.3 0.7 --time 3.0

  6. Direksiyon Açı Tarama Testi (-35° -> +35° -> 0°):
     python3 motor_test.py --sweep-steer

  7. Acil Durdurma ve Motorları Sıfırlama:
     python3 motor_test.py --stop
"""

import argparse
try:
    import readline  # Otomatik tamamlama ve geçmiş desteği (Linux / Pi5)
except ImportError:
    readline = None
import signal
import sys
import time
from typing import Optional
from pisim_motion import PiSimMotionController


def print_banner(motion: PiSimMotionController):
    print("\n" + "=" * 78)
    print(" 🛠️  PİSİM RASPBERRY Pİ 5 MOTOR & SERVO DİAGNOSTİK TEST ARACI")
    print("=" * 78)
    print(f" 🎯 Hedef Araç : {motion.target_vehicle}")
    print(f" 🌐 UE5 IP     : {motion.ue5_ip or 'Otomatik Keşif (Broadcast)'}")
    print(f" 📡 Komut Port : {motion.command_port} (Özel Port: {motion.dedicated_port or 'Bekleniyor'})")
    print("-" * 78)
    print(" Komutlar:")
    print("   steer <açı>          : Direksiyonu çevir (örn: 'steer 25' veya 'steer -20') [-35°..+35°]")
    print("   drive <hız>          : Tüm tekerlekleri sür (örn: 'drive 0.4' veya 'drive -0.3') [-1..+1]")
    print("   diff <sol> <sağ>     : Diferansiyel hız testi (örn: 'diff 0.3 0.8')")
    print("   fl / fr / rl / rr <hız>: Tekil tekerlek testi (örn: 'fl 0.5', 'rr -0.4')")
    print("   bone <kemik> <değer> : İsimle kemik testi (örn: 'bone Wheel_FL 0.6')")
    print("   sweep                : Direksiyon sağ-sol tarama testi (-35° .. +35°)")
    print("   ramp                 : Tekerlek hız rampa testi (%0 -> %80 -> %0)")
    print("   status / durum       : Canlı telemetri ve GPS/Pusula verisini göster")
    print("   stop                 : Acil durdurma ve motorları sıfırlama")
    print("   reset                : Motor test değerlerini temizle")
    print("   help / ?             : Bu yardım menüsünü yazdır")
    print("   quit / exit / q      : Çıkış")
    print("=" * 78 + "\n")


def run_sweep_steer(motion: PiSimMotionController):
    print("🔄 [DİREKSİYON TARAMA TESTİ BAŞLADI] -35°'den +35°'ye yumuşak geçiş...")
    try:
        steps = 40
        # 0 -> -35°
        for i in range(steps // 2):
            deg = - (i / (steps / 2)) * 35.0
            motion.send_steer_test(deg, timeout_sec=1.0)
            time.sleep(0.04)
        # -35° -> +35°
        for i in range(steps):
            deg = -35.0 + (i / steps) * 70.0
            motion.send_steer_test(deg, timeout_sec=1.0)
            time.sleep(0.04)
        # +35° -> 0°
        for i in range(steps // 2):
            deg = 35.0 - (i / (steps / 2)) * 35.0
            motion.send_steer_test(deg, timeout_sec=1.0)
            time.sleep(0.04)
        motion.send_steer_test(0.0, timeout_sec=1.0)
        print("✅ [DİREKSİYON TARAMA TAMAMLANDI] Direksiyon 0° merkezlendi.")
    except KeyboardInterrupt:
        motion.send_steer_test(0.0)
        print("\n🛑 Tarama kullanıcı tarafından durduruldu.")


def run_ramp_drive(motion: PiSimMotionController):
    print("📈 [SÜRÜŞ RAMPA TESTİ BAŞLADI] %0'dan %60'a hızlanma ve yavaşlama...")
    try:
        steps = 30
        # 0 -> 0.6
        for i in range(steps):
            val = (i / steps) * 0.6
            motion.send_drive_wheels_test(val, val, timeout_sec=1.0)
            time.sleep(0.06)
        time.sleep(0.5)
        # 0.6 -> 0.0
        for i in range(steps):
            val = 0.6 - (i / steps) * 0.6
            motion.send_drive_wheels_test(val, val, timeout_sec=1.0)
            time.sleep(0.06)
        motion.send_drive_wheels_test(0.0, 0.0)
        print("✅ [SÜRÜŞ RAMPA TESTİ TAMAMLANDI] Araç durduruldu.")
    except KeyboardInterrupt:
        motion.send_drive_wheels_test(0.0, 0.0)
        motion.stop()
        print("\n🛑 Rampa testi kullanıcı tarafından durduruldu.")


def resolve_wheel_alias(alias: str) -> str:
    alias_lower = alias.lower().strip()
    mapping = {
        "fl": "Wheel_FL",
        "front_left": "Wheel_FL",
        "sol_on": "Wheel_FL",
        "fr": "Wheel_FR",
        "front_right": "Wheel_FR",
        "sag_on": "Wheel_FR",
        "rl": "Wheel_RL",
        "rear_left": "Wheel_RL",
        "sol_arka": "Wheel_RL",
        "rr": "Wheel_RR",
        "rear_right": "Wheel_RR",
        "sag_arka": "Wheel_RR",
    }
    return mapping.get(alias_lower, alias)


def interactive_repl(motion: PiSimMotionController):
    print_banner(motion)

    # İlk bağlantı kontrolü
    t0 = time.time()
    telem = motion.get_sensor_telemetry()
    if telem:
        print(f"🟢 [BAĞLANTI BAŞARILI] Araç: {telem['name']} [{telem['code']}] | Hız: {telem['speed_kmh']:.1f} km/h | Yaw: {telem['yaw']:.1f}°\n")
    else:
        print("🟡 Telemetri bekleniyor... Komutlar hazır.\n")

    while True:
        try:
            prompt_str = f"pisim@{motion.target_vehicle}> "
            cmd_line = input(prompt_str).strip()
            if not cmd_line:
                continue

            parts = cmd_line.split()
            cmd = parts[0].lower()

            if cmd in ("quit", "exit", "q"):
                motion.send_reset_motors()
                motion.stop()
                print("👋 Motor test konsolundan çıkılıyor. Motorlar sıfırlandı.")
                break

            elif cmd in ("help", "?"):
                print_banner(motion)

            elif cmd in ("stop", "estop", "dur"):
                motion.send_reset_motors()
                motion.stop()
                print("🛑 [ACİL DURDURMA] Tüm motorlar sıfırlandı ve durduruldu.")

            elif cmd in ("reset", "sifirla"):
                motion.send_reset_motors()
                print("🔄 Motor test değerleri temizlendi.")

            elif cmd in ("status", "durum", "telem"):
                t = motion.get_sensor_telemetry()
                if t:
                    print(
                        f"📊 [DURUM] Araç: {t['name']} [{t['code']}] (ID: {t['id']})\n"
                        f"   📍 GPS : {t['lat']:.6f}° N, {t['lon']:.6f}° E (İrtifa: {t['altitude']:.1f}m)\n"
                        f"   🧭 IMU : Yaw={t['yaw']:.1f}° | Roll={t['roll']:.1f}° | Pitch={t['pitch']:.1f}°\n"
                        f"   ⚡ Hız : {t['speed_kmh']:.2f} km/h ({t['speed_mps']:.2f} m/s)\n"
                        f"   📡 Port: 7402 (Broadcast) / {motion.dedicated_port or 'N/A'} (Özel)"
                    )
                else:
                    print("⚠️ Telemetri paketi henüz alınamadı.")

            elif cmd in ("steer", "s"):
                if len(parts) < 2:
                    print("⚠️ Kullanım: steer <açı_derece> (örn: steer 20, steer -15)")
                    continue
                try:
                    angle = float(parts[1])
                    angle = max(-35.0, min(35.0, angle))
                    motion.send_steer_test(angle, timeout_sec=4.0)
                    print(f"🎯 Direksiyon: {angle:+.1f}° konumuna ayarlandı (4s watchdog).")
                except ValueError:
                    print("❌ Hata: Geçerli bir sayı giriniz.")

            elif cmd in ("drive", "d"):
                if len(parts) < 2:
                    print("⚠️ Kullanım: drive <hız> (örn: drive 0.5, drive -0.3) [-1.0 .. +1.0]")
                    continue
                try:
                    val = float(parts[1])
                    val = max(-1.0, min(1.0, val))
                    motion.send_drive_wheels_test(val, val, timeout_sec=4.0)
                    print(f"🚀 Tüm tekerlekler: {val:+.2f} hızla sürülüyor (4s watchdog).")
                except ValueError:
                    print("❌ Hata: Geçerli bir sayı giriniz.")

            elif cmd in ("diff", "skid"):
                if len(parts) < 3:
                    print("⚠️ Kullanım: diff <sol> <sağ> (örn: diff 0.3 0.8)")
                    continue
                try:
                    left = max(-1.0, min(1.0, float(parts[1])))
                    right = max(-1.0, min(1.0, float(parts[2])))
                    motion.send_drive_wheels_test(left, right, timeout_sec=4.0)
                    print(f"🔄 Diferansiyel Hız: Sol={left:+.2f} | Sağ={right:+.2f} (4s watchdog).")
                except ValueError:
                    print("❌ Hata: Geçerli sayılar giriniz.")

            elif cmd in ("fl", "fr", "rl", "rr"):
                if len(parts) < 2:
                    print(f"⚠️ Kullanım: {cmd} <değer> (örn: {cmd} 0.5)")
                    continue
                try:
                    val = max(-1.0, min(1.0, float(parts[1])))
                    bone = resolve_wheel_alias(cmd)
                    motion.send_motor_test(bone, val, timeout_sec=4.0)
                    print(f"⚙️ Tekerlek '{bone}' [{cmd.upper()}]: {val:+.2f} hızla sürülüyor.")
                except ValueError:
                    print("❌ Hata: Geçerli bir sayı giriniz.")

            elif cmd in ("wheel", "teker"):
                if len(parts) < 3:
                    print("⚠️ Kullanım: wheel <fl/fr/rl/rr/kemik_adı> <değer> (örn: wheel fl 0.5)")
                    continue
                try:
                    bone = resolve_wheel_alias(parts[1])
                    val = max(-1.0, min(1.0, float(parts[2])))
                    motion.send_motor_test(bone, val, timeout_sec=4.0)
                    print(f"⚙️ Tekerlek '{bone}': {val:+.2f} hızla sürülüyor.")
                except ValueError:
                    print("❌ Hata: Geçerli bir sayı giriniz.")

            elif cmd in ("bone", "kemik", "motor"):
                if len(parts) < 3:
                    print("⚠️ Kullanım: bone <kemik_adı> <değer> (örn: bone Wheel_FL 0.5)")
                    continue
                try:
                    bone = parts[1]
                    val = max(-1.0, min(1.0, float(parts[2])))
                    motion.send_motor_test(bone, val, timeout_sec=4.0)
                    print(f"⚡ Kemik/Motor '{bone}': {val:+.2f} değeri uygulandı.")
                except ValueError:
                    print("❌ Hata: Geçerli bir sayı giriniz.")

            elif cmd == "sweep":
                run_sweep_steer(motion)

            elif cmd == "ramp":
                run_ramp_drive(motion)

            else:
                print(f"❓ Bilinmeyen komut: '{cmd}'. Komut listesi için 'help' yazınız.")

        except (KeyboardInterrupt, EOFError):
            print("\n🛑 Çıkış komutu algılandı.")
            motion.send_reset_motors()
            motion.stop()
            break


def main():
    parser = argparse.ArgumentParser(description="PiSim Raspberry Pi 5 Motor & Servo Bağımsız Test Aracı")
    parser.add_argument("--target", "-t", type=str, default="selected", help="Hedef araç (örn: '1', 'RV01', 'selected')")
    parser.add_argument("--ip", type=str, default=None, help="Unreal Engine IP adresi (boşsa broadcast/otomatik)")
    parser.add_argument("--steer", "-s", type=float, default=None, help="Direksiyon açısı (-35 .. +35 derece)")
    parser.add_argument("--drive", "-d", type=float, default=None, help="Tüm tekerlekler için sürüş gücü (-1.0 .. +1.0)")
    parser.add_argument("--diff", nargs=2, type=float, default=None, metavar=("SOL", "SAG"), help="Diferansiyel tekerlek testi")
    parser.add_argument("--bone", type=str, default=None, help="Doğrudan test edilecek kemik adı")
    parser.add_argument("--wheel", type=str, default=None, help="Test edilecek tekerlek (fl, fr, rl, rr veya kemik adı)")
    parser.add_argument("--val", "-v", type=float, default=0.5, help="Kemik/tekerlek için test değeri (-1.0 .. +1.0)")
    parser.add_argument("--time", type=float, default=2.0, help="Test süresi (saniye, varsayılan: 2.0s)")
    parser.add_argument("--sweep-steer", action="store_true", help="Direksiyon tarama testi (-35° -> +35°)")
    parser.add_argument("--ramp-drive", action="store_true", help="Tekerlek hız rampa testi")
    parser.add_argument("--stop", action="store_true", help="Tüm motorları durdur ve sıfırla")
    parser.add_argument("--status", action="store_true", help="Araç sensör ve telemetri durumunu yazdır")
    parser.add_argument("--interactive", "-i", action="store_true", help="İnteraktif test konsolunu başlat")

    args = parser.parse_args()

    motion = PiSimMotionController(target_vehicle=args.target, ue5_ip=args.ip)

    def handle_sigint(sig, frame):
        print("\n🛑 [ACİL DURDURMA] Motorlar sıfırlanıyor...")
        motion.send_reset_motors()
        motion.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)

    # Argüman kontrolü: Eğer hiçbir test parametresi verilmediyse veya -i seçildiyse interaktif moda geç
    has_action = any([
        args.steer is not None,
        args.drive is not None,
        args.diff is not None,
        args.bone is not None,
        args.wheel is not None,
        args.sweep_steer,
        args.ramp_drive,
        args.stop,
        args.status,
    ])

    if not has_action or args.interactive:
        interactive_repl(motion)
        return

    # Tek seferlik argüman modu
    if args.stop:
        print("🛑 Motorlar durduruluyor ve sıfırlanıyor...")
        motion.send_reset_motors()
        motion.stop()
        print("✅ Tamamlandı.")

    elif args.status:
        t = motion.wait_for_telemetry(timeout_sec=3.0)
        if t:
            print(
                f"📊 [DURUM] Araç: {t['name']} [{t['code']}]\n"
                f"   GPS : {t['lat']:.6f}° N, {t['lon']:.6f}° E (İrtifa: {t['altitude']:.1f}m)\n"
                f"   IMU : Yaw={t['yaw']:.1f}° | Roll={t['roll']:.1f}° | Pitch={t['pitch']:.1f}°\n"
                f"   Hız : {t['speed_kmh']:.2f} km/h"
            )
        else:
            print("❌ Telemetri alınamadı.")

    elif args.sweep_steer:
        run_sweep_steer(motion)

    elif args.ramp_drive:
        run_ramp_drive(motion)

    elif args.steer is not None:
        angle = max(-35.0, min(35.0, args.steer))
        print(f"🎯 Direksiyon {angle:+.1f}° açısına ayarlanıyor (Süre: {args.time}s)...")
        t_end = time.time() + args.time
        while time.time() < t_end:
            motion.send_steer_test(angle, timeout_sec=1.5)
            time.sleep(0.1)
        motion.send_steer_test(0.0)
        print("✅ Tamamlandı (Merkeze döndürüldü).")

    elif args.diff is not None:
        left, right = args.diff[0], args.diff[1]
        print(f"🔄 Diferansiyel Hız: Sol={left:+.2f} | Sağ={right:+.2f} (Süre: {args.time}s)...")
        t_end = time.time() + args.time
        while time.time() < t_end:
            motion.send_drive_wheels_test(left, right, timeout_sec=1.5)
            time.sleep(0.1)
        motion.send_drive_wheels_test(0.0, 0.0)
        motion.stop()
        print("✅ Tamamlandı.")

    elif args.drive is not None:
        spd = max(-1.0, min(1.0, args.drive))
        print(f"🚀 Tüm tekerlekler {spd:+.2f} hızla sürülüyor (Süre: {args.time}s)...")
        t_end = time.time() + args.time
        while time.time() < t_end:
            motion.send_drive_wheels_test(spd, spd, timeout_sec=1.5)
            time.sleep(0.1)
        motion.send_drive_wheels_test(0.0, 0.0)
        motion.stop()
        print("✅ Tamamlandı.")

    elif args.wheel is not None:
        bone = resolve_wheel_alias(args.wheel)
        val = max(-1.0, min(1.0, args.val))
        print(f"⚙️ Tekerlek '{bone}': {val:+.2f} değeri uygulanıyor (Süre: {args.time}s)...")
        t_end = time.time() + args.time
        while time.time() < t_end:
            motion.send_motor_test(bone, val, timeout_sec=1.5)
            time.sleep(0.1)
        motion.send_motor_test(bone, 0.0)
        print("✅ Tamamlandı.")

    elif args.bone is not None:
        val = max(-1.0, min(1.0, args.val))
        print(f"⚡ Kemik '{args.bone}': {val:+.2f} değeri uygulanıyor (Süre: {args.time}s)...")
        t_end = time.time() + args.time
        while time.time() < t_end:
            motion.send_motor_test(args.bone, val, timeout_sec=1.5)
            time.sleep(0.1)
        motion.send_motor_test(args.bone, 0.0)
        print("✅ Tamamlandı.")


if __name__ == "__main__":
    main()
