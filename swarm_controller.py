#!/usr/bin/env python3
"""
swarm_controller.py - PiSim Profesyonel Dağıtık Sürü Kontrolcüsü
ROS 2 / MAVLink endüstriyel standartlarında UDP Broadcast (Port 7402) üzerinden
merkezi olmayan (decentralized) sürü hareket ve formasyon orkestratörü.

Özellikler:
1. Brokerless & Peer-to-Peer: Hiçbir aracı/master düğüm olmadan doğrudan ağ kartından araçlara ulaşır.
2. Bağımsız Soketler (SO_REUSEADDR): Ağdaki tüm araçlar 0ms senkronizasyonla paketi aynı anda alır.
3. Çift Mod:
   - Tekil Araç Kontrolü: target="rover_01"
   - Tüm Sürü Senkron Kontrolü: target="all"
4. Emniyet (Watchdog): Komut akışı kesilirse araçlar belirlenen timeout_ms sonunda otomatik durur.
5. WASD İnteraktif Sürüş & Otonom Formasyon Modları.
"""

import sys
import time
import socket
import json
import argparse
from pathlib import Path

# Ensure UTF-8 output on all consoles
try:
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    if hasattr(sys.stderr, 'reconfigure'):
        sys.stderr.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

COMMAND_PORT = 7402

class PiSimSwarm:
    """PiSim Sürü Hareket ve Kinematik Yöneticisi"""

    def __init__(self, target_ip: str = "255.255.255.255", command_port: int = COMMAND_PORT):
        self.target_ip = target_ip
        self.command_port = command_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def send_raw_command(self, payload: dict, override_port: int = None):
        """UDP Broadcast / Unicast üzerinden araçların portlarına doğrudan komut fırlatır."""
        port = override_port if override_port is not None else self.command_port
        try:
            msg_bytes = json.dumps(payload).encode('utf-8')
            # 1. Hedef IP veya Küresel Broadcast
            self.sock.sendto(msg_bytes, (self.target_ip, port))
            # 2. Localhost döngü kontrolü
            if self.target_ip != "127.0.0.1":
                try:
                    self.sock.sendto(msg_bytes, ("127.0.0.1", port))
                except Exception:
                    pass
            # 3. Raspberry Pi 5 / Linux Wi-Fi alt ağ broadcast desteği (192.168.x.255)
            if self.target_ip == "255.255.255.255":
                try:
                    from beacon_sender import get_broadcast_targets
                    for b_ip, b_port in get_broadcast_targets(port):
                        try:
                            self.sock.sendto(msg_bytes, (b_ip, b_port))
                        except Exception:
                            pass
                except Exception:
                    pass
        except Exception as e:
            print(f"⚠️ [swarm_controller] Gönderim hatası: {e}")

    def drive(self, target: str, linear_x: float, angular_z: float = 0.0, timeout_ms: int = 600):
        """
        Belirtilen araca veya tüm sürüye hız vektörü (cmd_vel) gönderir.
        
        :param target: "rover_01", "rover_02" veya "all" (bütün sürü)
        :param linear_x: m/s cinsinden ileri (+) / geri (-) hız
        :param angular_z: rad/s cinsinden dönüş hızı (+ sol, - sağ)
        :param timeout_ms: Emniyet süresi (komut kesilirse otomatik durur)
        """
        payload = {
            "cmd": "cmd_vel",
            "target": str(target),
            "linear_x": round(float(linear_x), 3),
            "angular_z": round(float(angular_z), 3),
            "timeout_ms": int(timeout_ms),
            "timestamp": time.time()
        }
        self.send_raw_command(payload)

    def drive_all(self, linear_x: float, angular_z: float = 0.0, timeout_ms: int = 600):
        """Tüm sürüdeki araçları tek bir UDP broadcast paketiyle senkronize hareket ettirir."""
        self.drive(target="all", linear_x=linear_x, angular_z=angular_z, timeout_ms=timeout_ms)

    def stop(self, target: str = "all"):
        """Belirtilen aracı veya tüm sürüyü anında durdurur (E-Stop)."""
        payload = {
            "cmd": "stop",
            "target": str(target),
            "timestamp": time.time()
        }
        self.send_raw_command(payload)

    def stop_all(self):
        """Tüm sürüyü anında durdurur."""
        self.stop(target="all")

    def spin_in_place(self, target: str = "all", angular_z: float = 1.0, duration_sec: float = 2.0):
        """Kendi etrafında tank dönüşü manevrası yaptırır."""
        t_end = time.time() + duration_sec
        while time.time() < t_end:
            self.drive(target=target, linear_x=0.0, angular_z=angular_z, timeout_ms=400)
            time.sleep(0.1)
        self.stop(target=target)

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


def run_interactive_wasd(swarm: PiSimSwarm, target: str = "all"):
    """Terminal üzerinden klavye ile anlık gerçek zamanlı sürü kontrolü."""
    print("\n" + "=" * 70)
    print(f"  🎮 PISIM İNTERAKTİF SÜRÜ KONTROLÜ [Hedef: {target.upper()}]")
    print("=" * 70)
    print("  Kontroller (Tuş + Enter):")
    print("    [W] : İleri (1.5 m/s)         [S] : Geri (-1.0 m/s)")
    print("    [A] : Sola Dön (+0.8 rad/s)   [D] : Sağa Dön (-0.8 rad/s)")
    print("    [Q] : Sola İleri Kavis        [E] : Sağa İleri Kavis")
    print("    [X] veya Boşluk : ANINDA DURDUR (E-Stop)")
    print("    [C] : Çıkış")
    print("=" * 70 + "\n")

    current_linear = 0.0
    current_angular = 0.0

    try:
        while True:
            cmd = input(f"[{target}] Komut (W/A/S/D/X) > ").strip().lower()
            if not cmd:
                continue

            if cmd == 'c':
                print("🛑 Kontrolden çıkılıyor, sürü durduruluyor...")
                swarm.stop(target)
                break
            elif cmd == 'w':
                current_linear = 1.5
                current_angular = 0.0
                print(f"⬆️  [İLERİ] Hız: {current_linear} m/s")
            elif cmd == 's':
                current_linear = -1.0
                current_angular = 0.0
                print(f"⬇️  [GERİ] Hız: {current_linear} m/s")
            elif cmd == 'a':
                current_linear = 0.0
                current_angular = 1.0
                print(f"↩️  [SOLA DÖNÜŞ] Açısal Hız: +{current_angular} rad/s")
            elif cmd == 'd':
                current_linear = 0.0
                current_angular = -1.0
                print(f"↪️  [SAĞA DÖNÜŞ] Açısal Hız: {current_angular} rad/s")
            elif cmd == 'q':
                current_linear = 1.2
                current_angular = 0.6
                print(f"↖️  [SOLA KAVİS]")
            elif cmd == 'e':
                current_linear = 1.2
                current_angular = -0.6
                print(f"↗️  [SAĞA KAVİS]")
            elif cmd in ('x', ' ', 'stop'):
                current_linear = 0.0
                current_angular = 0.0
                swarm.stop(target)
                print(f"🛑 [DURDURULDU] {target}")
                continue
            else:
                print(f"⚠️ Bilinmeyen tuş: '{cmd}'. (W/A/S/D/X kullanınız)")
                continue

            # 2 saniyelik emniyet paketi fırlat
            swarm.drive(target=target, linear_x=current_linear, angular_z=current_angular, timeout_ms=2500)

    except KeyboardInterrupt:
        print("\n🛑 Çıkış yapıldı. Sürü durduruluyor...")
        swarm.stop(target)


def main():
    parser = argparse.ArgumentParser(description="PiSim Profesyonel Dağıtık Sürü Kontrolcüsü (UDP 7402)")
    parser.add_argument("--target", type=str, default="all", help="Hedef araç: 'all', 'rover_01', 'rover_02' vb.")
    parser.add_argument("--linear", type=float, default=None, help="Doğrusal hız (m/s) [İleri (+) / Geri (-)]")
    parser.add_argument("--angular", type=float, default=0.0, help="Açısal hız (rad/s) [Sol (+) / Sağ (-)]")
    parser.add_argument("--duration", type=float, default=3.0, help="Komut yürütme süresi (saniye)")
    parser.add_argument("--stop", action="store_true", help="Hedef aracı veya tüm sürüyü anında durdur")
    parser.add_argument("--interactive", "-i", action="store_true", help="İnteraktif klavye sürüş konsolunu aç")
    parser.add_argument("--demo-circle", action="store_true", help="Daire çizme otonom manevra testi")
    parser.add_argument("--ip", type=str, default="255.255.255.255", help="Hedef IP veya broadcast adresi")
    parser.add_argument("--port", type=int, default=COMMAND_PORT, help="Komut portu (7402=Ortak Sürü, 7403=Araç 1, 7404=Araç 2...)")

    args = parser.parse_args()
    swarm = PiSimSwarm(target_ip=args.ip, command_port=args.port)

    if args.stop:
        print(f"🛑 [E-STOP] '{args.target}' aracı/sürüsü durduruluyor...")
        swarm.stop(args.target)
        return

    if args.interactive:
        run_interactive_wasd(swarm, target=args.target)
        return

    if args.demo_circle:
        print(f"🔄 [DEMO] '{args.target}' için daire çizme manevrası başlatılıyor ({args.duration}s)...")
        t_end = time.time() + args.duration
        while time.time() < t_end:
            swarm.drive(target=args.target, linear_x=1.5, angular_z=0.6, timeout_ms=500)
            time.sleep(0.1)
        swarm.stop(args.target)
        print("✅ Daire manevrası tamamlandı ve araç durduruldu.")
        return

    if args.linear is not None:
        print(f"🚀 [SÜRÜŞ] Hedef: '{args.target}' | Linear: {args.linear} m/s | Angular: {args.angular} rad/s | Süre: {args.duration}s")
        t_end = time.time() + args.duration
        while time.time() < t_end:
            swarm.drive(target=args.target, linear_x=args.linear, angular_z=args.angular, timeout_ms=500)
            time.sleep(0.1)
        swarm.stop(args.target)
        print(f"🛑 Süre bitti, '{args.target}' durduruldu.")
        return

    # Argüman verilmemişse yardım ve interaktif menü göster
    print("\nℹ️  PiSim Sürü Kontrolcüsü Hazır.")
    print("   Örnek Kullanımlar:")
    print("     python swarm_controller.py --interactive               (Tüm sürüyü klavye ile sür)")
    print("     python swarm_controller.py --target rover_01 -i        (Sadece rover_01'i klavye ile sür)")
    print("     python swarm_controller.py --linear 2.0 --duration 5   (Tüm sürüyü 5 saniye 2 m/s ileri sür)")
    print("     python swarm_controller.py --demo-circle               (Dairesel otonom sürüş testi)")
    print("     python swarm_controller.py --stop                      (Acil durum freni)\n")


if __name__ == "__main__":
    main()

