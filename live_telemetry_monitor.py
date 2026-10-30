#!/usr/bin/env python3
"""
live_telemetry_monitor.py - PiSim Canlı Sabit Ekran (In-Place) Telemetri & SQL Satır Monitörü

Özellikler:
1. Şelale gibi aşağı akan loglar YERİNE, ekranı sabit tutarak (ANSI Cursor Home) her benzersiz (unique)
   aracı kendi satırında canlı olarak günceller.
2. Yeni bir araç (2., 3. araç vb.) ağa katıldığında otomatik olarak tablonun altına yeni satır olarak eklenir.
3. Tüm SQL / Telemetri sütunlarını (Eşsiz Kod, Araç Adı, ID, Konum X/Y/Z, Roll/Pitch/Yaw, Hız, İrtifa,
   Batarya, GPS Lat/Lon, Mod, Paket Sayısı) tek tabloda gösterir.
4. Bir araçtan 5 saniye (OFFLINE_TIMEOUT_SEC) boyunca mesaj gelmezse satırı silmez; KIRMIZI renkle
   "🔴 BAĞLANTI KOPTU (PASİF)" olarak işaretler.
5. Hem tek başına (UDP 7400 + 7499 Beacon) hem de pisim_web_dashboard.py ile aynı anda çalışabilir.
"""

import os
import sys
import time
import json
import socket
import threading
import urllib.request

try:
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

# Windows konsolunda ANSI renk ve ekran sabitleme kodlarını etkinleştir
if os.name == 'nt':
    try:
        import ctypes
        kernel32 = ctypes.windll.kernel32
        kernel32.SetConsoleMode(kernel32.GetStdHandle(-11), 7)
    except Exception:
        pass

TELEMETRY_PORT = 7400
DISCOVERY_PORT = 7499
OFFLINE_TIMEOUT_SEC = 5.0  # 5 saniye haber alınamazsa kırmızı / BAĞLANTI KOPTU göster

# ANSI Renk Kodları
RESET = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"
GREEN = "\033[92m"
RED = "\033[91m"
BG_RED = "\033[41;97m"
CYAN = "\033[96m"
YELLOW = "\033[93m"
WHITE = "\033[97m"
CURSOR_HOME = "\033[H"
CLEAR_SCREEN = "\033[2J\033[H"
CLEAR_TO_END = "\033[J"

vehicles_lock = threading.Lock()
unique_vehicles = {}  # vehicle_code -> dict
total_packets_received = 0
running = True


def get_local_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def update_vehicle_from_packet(msg: dict, sender_ip: str = "AĞ"):
    global total_packets_received
    now = time.time()
    v_id = int(msg.get("vehicle_id", msg.get("id", 1)))
    v_code = str(msg.get("vehicle_code") or msg.get("code") or f"RV{v_id:02d}")
    v_name = str(msg.get("vehicle_name") or msg.get("name") or v_code)

    pos = msg.get("position", [])
    pos_x = float(msg.get("pos_x", pos[0] if isinstance(pos, list) and len(pos) > 0 else 0.0))
    pos_y = float(msg.get("pos_y", pos[1] if isinstance(pos, list) and len(pos) > 1 else 0.0))
    pos_z = float(msg.get("pos_z", pos[2] if isinstance(pos, list) and len(pos) > 2 else msg.get("altitude", 0.0)))

    roll = float(msg.get("roll", 0.0))
    pitch = float(msg.get("pitch", 0.0))
    yaw = float(msg.get("yaw", 0.0))
    if roll == 0.0 and "attitude" in msg and isinstance(msg["attitude"], list) and len(msg["attitude"]) >= 3:
        roll = msg["attitude"][0] * 57.2958
        pitch = msg["attitude"][1] * 57.2958
        yaw = msg["attitude"][2] * 57.2958

    speed = float(msg.get("speed", float(msg.get("airspeed", 0.0)) * 3.6))
    altitude = float(msg.get("altitude", pos_z))
    battery = float(msg.get("battery", 98.0))
    lat = float(msg.get("lat", 39.9334))
    lon = float(msg.get("lon", 32.8597))
    mode = str(msg.get("flight_mode", "AKTİF"))
    greeting = str(msg.get("msg", f"HI FROM {v_name}"))

    with vehicles_lock:
        total_packets_received += 1
        prev = unique_vehicles.get(v_code, {})
        rx_count = int(msg.get("rx_count", prev.get("rx_count", 0) + 1))
        last_rx = float(msg.get("last_rx_time", now))

        unique_vehicles[v_code] = {
            "vehicle_code": v_code,
            "vehicle_name": v_name,
            "vehicle_id": v_id,
            "pos_x": round(pos_x, 2),
            "pos_y": round(pos_y, 2),
            "pos_z": round(pos_z, 2),
            "roll": round(roll, 1),
            "pitch": round(pitch, 1),
            "yaw": round(yaw, 1),
            "speed": round(speed, 1),
            "altitude": round(altitude, 2),
            "battery": round(battery, 0),
            "lat": round(lat, 5),
            "lon": round(lon, 5),
            "flight_mode": mode,
            "msg": greeting,
            "sender_ip": str(msg.get("ue5_ip") or sender_ip),
            "last_rx_time": last_rx,
            "rx_count": rx_count,
            "last_time_str": time.strftime("%H:%M:%S", time.localtime(last_rx))
        }


def udp_listener_worker():
    """UDP Port 7400 üzerinden gelen ham telemetri paketlerini doğrudan yakalar."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except Exception:
            pass
    try:
        sock.bind(("0.0.0.0", TELEMETRY_PORT))
    except Exception:
        # Port 7400 başka süreç tarafından kilitliyse HTTP API yedek işçisi devralır
        return

    sock.settimeout(1.0)
    while running:
        try:
            data, addr = sock.recvfrom(65535)
            if not data:
                continue
            msg = json.loads(data.decode("utf-8", errors="ignore"))
            update_vehicle_from_packet(msg, sender_ip=addr[0])
        except socket.timeout:
            continue
        except Exception:
            continue


def beacon_sender_worker():
    """Unreal Engine'in bu cihazın IP adresini otomatik keşfetmesi için UDP 7499 Beacon yayını yapar."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    local_ip = get_local_ip()
    while running:
        try:
            beacon = json.dumps({
                "magic": "PISIM_BEACON",
                "device": "Raspberry Pi 5 (Companion)",
                "ip": local_ip,
                "telemetry_port": TELEMETRY_PORT,
                "msg": "HI FROM PI5"
            }).encode("utf-8")
            sock.sendto(beacon, ("255.255.255.255", DISCOVERY_PORT))
        except Exception:
            pass
        time.sleep(2.0)


def dashboard_api_fallback_worker():
    """Eğer pisim_web_dashboard.py aynı cihazda çalışıyorsa oradaki canlı filoyu da senkronize eder."""
    while running:
        try:
            req = urllib.request.Request("http://127.0.0.1:8080/api/telemetry", headers={"Cache-Control": "no-cache"})
            with urllib.request.urlopen(req, timeout=0.6) as resp:
                data = json.loads(resp.read().decode("utf-8"))
                fleet = data.get("fleet", [])
                for v in fleet:
                    v_code = v.get("code") or v.get("vehicle_code")
                    if not v_code:
                        continue
                    with vehicles_lock:
                        existing = unique_vehicles.get(v_code)
                        v_rx = float(v.get("last_rx_time", 0.0))
                        if not existing or v_rx > existing.get("last_rx_time", 0.0):
                            update_vehicle_from_packet(v, sender_ip=data.get("ue5_ip") or "127.0.0.1")
        except Exception:
            pass
        time.sleep(0.35)


def render_fixed_screen():
    local_ip = get_local_ip()
    sys.stdout.write(CLEAR_SCREEN)
    sys.stdout.flush()

    while running:
        now = time.time()
        with vehicles_lock:
            v_list = list(unique_vehicles.values())
            v_list.sort(key=lambda x: (x.get("vehicle_id", 0), x.get("vehicle_code", "")))
            pkt_total = total_packets_received

        active_count = sum(1 for v in v_list if (now - v["last_rx_time"]) < OFFLINE_TIMEOUT_SEC)
        offline_count = len(v_list) - active_count

        lines = []
        lines.append(f"{BOLD}{CYAN}========================================================================================================================================{RESET}")
        lines.append(
            f" 📡 {BOLD}{WHITE}PISIM CANLI TELEMETRİ & SQL SATIR MONİTÖRÜ (SABİT EKRAN){RESET}   | "
            f"Yerel IP: {YELLOW}{local_ip}:{TELEMETRY_PORT}{RESET} | "
            f"Toplam Araç: {BOLD}{len(v_list)}{RESET} ({GREEN}{active_count} Aktif{RESET}, {RED}{offline_count} Pasif{RESET}) | "
            f"Paket: {CYAN}{pkt_total}{RESET}"
        )
        lines.append(f"{BOLD}{CYAN}========================================================================================================================================{RESET}")

        # Tablo Başlığı (Tüm SQL Sütunları)
        header = (
            f"{BOLD}{WHITE}"
            f"{'DURUM':<22} | "
            f"{'EŞSİZ KOD (UID)':<14} | "
            f"{'ARAÇ ADI':<18} | "
            f"{'ID':<3} | "
            f"{'KONUM (X, Y, Z m)':<24} | "
            f"{'R / P / Y (°)':<18} | "
            f"{'HIZ':<8} | "
            f"{'GPS (LAT, LON)':<21} | "
            f"{'PAKET / SAAT':<14}"
            f"{RESET}"
        )
        lines.append(header)
        lines.append(f"{CYAN}{'-' * 136}{RESET}")

        if not v_list:
            lines.append(f"\n   {YELLOW}⏳ Henüz hiçbir araçtan veri gelmedi. Unreal Engine simülasyonunun (UDP Port 7400) başlaması bekleniyor...{RESET}\n")
        else:
            for v in v_list:
                age = now - v["last_rx_time"]
                is_online = age < OFFLINE_TIMEOUT_SEC

                xyz_str = f"X:{v['pos_x']:>6.1f} Y:{v['pos_y']:>6.1f} Z:{v['pos_z']:>5.1f}"
                rpy_str = f"{v['roll']:>4.0f}/{v['pitch']:>4.0f}/{v['yaw']:>4.0f}"
                spd_str = f"{v['speed']:>4.1f}km/s"
                gps_str = f"{v['lat']:.4f},{v['lon']:.4f}"
                pkt_str = f"#{v['rx_count']:<4} {v['last_time_str']}"

                if is_online:
                    status_col = f"{GREEN}{BOLD}● AKTİF (CANLI 2Hz){RESET}   "
                    row_color = WHITE
                    code_color = CYAN + BOLD
                else:
                    status_col = f"{BG_RED}{BOLD} BAĞLANTI KOPTU ({age:>3.0f}s) {RESET}"
                    row_color = RED
                    code_color = RED + BOLD

                row_str = (
                    f"{status_col} | "
                    f"{code_color}{v['vehicle_code']:<14}{RESET}{row_color} | "
                    f"{v['vehicle_name']:<18} | "
                    f"#{v['vehicle_id']:<2} | "
                    f"{xyz_str:<24} | "
                    f"{rpy_str:<18} | "
                    f"{spd_str:<8} | "
                    f"{gps_str:<21} | "
                    f"{pkt_str:<14}{RESET}"
                )
                lines.append(row_str)

            lines.append(f"{CYAN}{'-' * 136}{RESET}")
            lines.append(f"{BOLD}{YELLOW} 📌 SON GELEN TAM SQL SATIRI DETAYLARI (DOĞRULAMA KANITI):{RESET}")
            for v in v_list:
                age = now - v["last_rx_time"]
                c_tag = GREEN if age < OFFLINE_TIMEOUT_SEC else RED
                state_txt = "ONLINE" if age < OFFLINE_TIMEOUT_SEC else f"OFFLINE ({age:.1f} sn önce kesildi!)"
                lines.append(
                    f"   {c_tag}▸ [{v['vehicle_code']}] {v['vehicle_name']}{RESET} ({state_txt}) -> "
                    f"Konum: {BOLD}X={v['pos_x']}m, Y={v['pos_y']}m, Z={v['pos_z']}m{RESET} | "
                    f"GPS: {v['lat']}°N, {v['lon']}°E | "
                    f"Hız: {v['speed']} km/s | İrtifa: {v['altitude']}m | Pil: %{v['battery']:.0f} | "
                    f"Mesaj: \"{v['msg']}\" (Kaynak: {v['sender_ip']})"
                )

        lines.append(f"\n{DIM}[Çıkış için Ctrl+C tuşlarına basın | Eşsiz araçlar sabit satırda güncellenir, {OFFLINE_TIMEOUT_SEC:.0f} sn sinyal gelmezse KIRMIZI olur]{RESET}")

        output = CURSOR_HOME + "\n".join(lines) + CLEAR_TO_END
        sys.stdout.write(output)
        sys.stdout.flush()
        time.sleep(0.25)


def main():
    global running
    t1 = threading.Thread(target=udp_listener_worker, daemon=True)
    t2 = threading.Thread(target=beacon_sender_worker, daemon=True)
    t3 = threading.Thread(target=dashboard_api_fallback_worker, daemon=True)
    t1.start()
    t2.start()
    t3.start()

    try:
        render_fixed_screen()
    except KeyboardInterrupt:
        running = False
        print(f"\n{YELLOW}🛑 Canlı telemetri monitörü kapatıldı.{RESET}")


if __name__ == "__main__":
    main()
