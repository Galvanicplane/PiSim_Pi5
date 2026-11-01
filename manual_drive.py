#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PiSim - Raspberry Pi 5 Manuel Klavye Sürüş Aracı (manual_drive.py)
==================================================================
Bu araç, Pi 5 üzerinden seçili aracı WASD tuşları ile doğrudan, gecikmesiz
ve telemetri geri bildirimli olarak manuel sürmenizi sağlar.

Tuş Kontrolleri:
  [ W ] veya [ ▲ ] : İleri Gaz Ver (+ Hız)
  [ S ] veya [ ▼ ] : Geri / Fren (- Hız)
  [ A ] veya [ ◄ ] : Sola Kır (Direksiyon Sol)
  [ D ] veya [ ► ] : Sağa Kır (Direksiyon Sağ)
  [ SPACE ]        : Anında Acil Fren / Durdur
  [ C ]            : Direksiyonu Ortala (0°)
  [ Q ] / [ ESC ]  : Güvenli Çıkış

Kullanım:
  python3 manual_drive.py
  python3 manual_drive.py --target RV01 --speed 2.5
"""

import argparse
import math
import os
import sys
import time
from pisim_motion import PiSimMotionController


def setup_keyboard():
    """İşletim sistemine göre (Linux/Pi5 veya Windows) tuş okuyucuyu ayarlar."""
    if os.name == 'nt':
        import msvcrt

        def get_key_nonblocking():
            if msvcrt.kbhit():
                ch = msvcrt.getch()
                if ch in (b'\x00', b'\xe0'):  # Özel tuşlar (Ok tuşları)
                    ch2 = msvcrt.getch()
                    if ch2 == b'H': return 'up'
                    if ch2 == b'P': return 'down'
                    if ch2 == b'K': return 'left'
                    if ch2 == b'M': return 'right'
                try:
                    return ch.decode('utf-8', errors='ignore').lower()
                except Exception:
                    return None
            return None

        def cleanup_keyboard():
            pass

        return get_key_nonblocking, cleanup_keyboard
    else:
        # Linux (Raspberry Pi 5)
        import select
        import termios
        import tty

        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        def get_key_nonblocking():
            rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
            if rlist:
                ch = sys.stdin.read(1)
                if ch == '\x1b':  # ANSI escape / Ok tuşları
                    ch2 = sys.stdin.read(1)
                    if ch2 == '[':
                        ch3 = sys.stdin.read(1)
                        if ch3 == 'A': return 'up'
                        if ch3 == 'B': return 'down'
                        if ch3 == 'D': return 'left'
                        if ch3 == 'C': return 'right'
                    return 'esc'
                return ch.lower()
            return None

        def cleanup_keyboard():
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        return get_key_nonblocking, cleanup_keyboard


def main():
    parser = argparse.ArgumentParser(description="PiSim Pi 5 Manuel Klavye Sürüş Aracı (WASD Teleop)")
    parser.add_argument("--target", "-t", type=str, default="selected", help="Hedef araç kodu (örn: 'RV01', 'all', 'selected')")
    parser.add_argument("--ip", type=str, default=None, help="Unreal Engine IP adresi (boşsa otomatik keşfedilir)")
    parser.add_argument("--speed", type=float, default=2.2, help="Azami ileri hız (m/s, varsayılan: 2.2)")
    parser.add_argument("--steer-max", type=float, default=32.0, help="Azami direksiyon açısı (derece, varsayılan: 32°)")
    args = parser.parse_args()

    motion = PiSimMotionController(target_vehicle=args.target, ue5_ip=args.ip)
    get_key, cleanup = setup_keyboard()

    print("=" * 76)
    print(" 🎮 PISIM PI 5 - MANUEL KLAVYE SÜRÜŞ KONSOLU (WASD TELEOP)")
    print("=" * 76)
    print(" Kontroller:")
    print("   [ W / ▲ ] : İleri Gaz   | [ S / ▼ ] : Geri / Fren")
    print("   [ A / ◄ ] : Sola Kır    | [ D / ► ] : Sağa Kır")
    print("   [ SPACE ] : Acil Fren   | [ C ]     : Direksiyon Ortala")
    print("   [ Q / ESC]: Çıkış")
    print("-" * 76)
    print(" 📡 Unreal Engine telemetrisi bekleniyor...")

    telem = motion.wait_for_telemetry(timeout_sec=3.0)
    veh_code = telem.get("code", args.target) if telem else args.target
    veh_name = telem.get("name", "Rover") if telem else "Rover"
    print(f" ✅ Bağlantı Kuruldu: {veh_name} [{veh_code}]")
    print("-" * 76)

    # Durum değişkenleri
    cur_v = 0.0
    cur_steer_deg = 0.0
    max_v = max(0.5, float(args.speed))
    max_steer = max(10.0, float(args.steer_max))
    v_step = 0.35
    steer_step = 6.0

    try:
        last_tx_time = 0.0
        while True:
            k = get_key()
            if k:
                if k in ('q', 'esc', '\x03'):
                    print("\n🛑 Güvenli çıkış yapılıyor... Motorlar durduruldu.")
                    motion.stop()
                    break
                elif k in ('w', 'up'):
                    cur_v = min(max_v, cur_v + v_step)
                elif k in ('s', 'down'):
                    cur_v = max(-max_v * 0.65, cur_v - v_step)
                elif k in ('a', 'left'):
                    cur_steer_deg = max(-max_steer, cur_steer_deg - steer_step)
                elif k in ('d', 'right'):
                    cur_steer_deg = min(max_steer, cur_steer_deg + steer_step)
                elif k in (' ', 'space'):
                    cur_v = 0.0
                    cur_steer_deg = 0.0
                    motion.stop()
                elif k == 'c':
                    cur_steer_deg = 0.0

            # 15Hz periyodik UDP komut gönderimi
            now = time.time()
            if now - last_tx_time >= 0.065:
                last_tx_time = now
                # Ackermann açısını açısal hıza çevir: ω = V * tan(steer) / L
                wheelbase_m = 0.34
                steer_rad = math.radians(cur_steer_deg)
                if abs(cur_v) > 0.05:
                    omega_cmd = (cur_v / wheelbase_m) * math.tan(steer_rad)
                else:
                    omega_cmd = math.radians(cur_steer_deg) * 1.5

                motion.send_cmd_vel(cur_v, omega_cmd, timeout_ms=300)

                # Canlı HUD
                steer_icon = "◄◄" if cur_steer_deg < -15 else ("◄" if cur_steer_deg < -3 else ("►►" if cur_steer_deg > 15 else ("►" if cur_steer_deg > 3 else "▲▲")))
                line = (
                    f"\r\033[K[{veh_code}] ⚡ Gaz: {cur_v:+4.1f} m/s (Azami: {max_v:.1f}) | "
                    f"🛞 {steer_icon} Direksiyon: {cur_steer_deg:+5.1f}° (ω: {omega_cmd:+4.2f} rad/s) | "
                    f"[W/A/S/D - SPACE:Fren - Q:Çık]"
                )
                sys.stdout.write(line)
                sys.stdout.flush()

            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n🛑 [Ctrl+C] Çıkış. Motorlar durduruluyor...")
        motion.stop()
    finally:
        cleanup()


if __name__ == "__main__":
    main()
