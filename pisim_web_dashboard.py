#!/usr/bin/env python3
"""
pisim_web_dashboard.py - Raspberry Pi 5 & Cross-Platform Tactical Web Dashboard Runner
Universal GCS (Ground Control Station) runner for PiSim Platform.

Runs the HTML5/CSS/JS tactical dashboard on Raspberry Pi 5 (Linux ARM64), Windows, or Jetson:
- Serves local WebUI files on http://localhost:8080 (also accessible from phone/tablet on same LAN)
- Bases on pure dashboard.html (Tactical Ground Control Station)
- Automatically broadcasts UDP Beacons (Port 7499) so Unreal Engine instantly discovers this Pi 5
- Listens for UDP Telemetry (Port 7400/7401) and feeds live state via /api/telemetry
- Launches Chromium / Chrome in standalone App Mode (No URL bar, looks 100% like a native desktop app)
- Pulls updates directly via 'git pull' with zero compiling!
"""

import http.server
import socketserver
import threading
import socket
import json
import time
import sys
import os
import subprocess
import webbrowser
from pathlib import Path
from urllib.parse import urlparse, parse_qs

# Import discovery service from beacon_sender
from beacon_sender import PiSimDiscoveryService, get_local_ip, get_all_local_ips, send_probe_burst, DISCOVERY_PORT

# Ensure UTF-8 output on all consoles
try:
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    if hasattr(sys.stderr, 'reconfigure'):
        sys.stderr.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

PORT = 8080
WEBUI_DIR = Path(__file__).parent / "webui"
TELEMETRY_PORT = 7400
VIDEO_PORT = 7401
UE5_CONTROL_PORT = 9002

latest_telemetry = {
    "timestamp": 0.0,
    "vehicle_id": 0,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "altitude": 0.0,
    "speed": 0.0,
    "battery": 98.0,
    "lat": 39.9334,
    "lon": 32.8597,
    "flight_mode": "BEKLİYOR",
    "connected": False,
    "ue5_ip": None,
    "msg": "HI FROM UNREAL"
}
telemetry_lock = threading.Lock()
discovered_ue5_ip = None
discovery_service_ref = None


class PiSimRequestHandler(http.server.SimpleHTTPRequestHandler):
    """Custom HTTP handler serving webui and providing live REST telemetry & command APIs"""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(WEBUI_DIR), **kwargs)

    def end_headers(self):
        # TÜM DOSYALAR (HTML/CSS/JS/API) İÇİN ÖNBELLEĞİ KESİNLİKLE KAPAT (CACHE-BUSTING)
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        super().end_headers()

    def do_GET(self):
        global discovered_ue5_ip
        parsed_url = urlparse(self.path)
        clean_path = parsed_url.path

        # 1. Kök dizini doğrudan dashboard.html sayfasına yönlendir
        if clean_path in ('/', '/index.html'):
            self.send_response(302)
            self.send_header('Location', '/dashboard.html')
            self.end_headers()
            return

        # 2. Canlı telemetri REST API uç noktası
        if clean_path == '/api/telemetry':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            with telemetry_lock:
                latest_telemetry["local_ip"] = get_local_ip()
                latest_telemetry["ue5_ip"] = discovered_ue5_ip
                payload = json.dumps(latest_telemetry).encode('utf-8')
            self.wfile.write(payload)
            return

        # 3. Anlık keşif probu fırlatma REST API uç noktası
        if clean_path == '/api/probe':
            target_ip = "255.255.255.255"
            qs = parse_qs(parsed_url.query)
            if 'ip' in qs:
                target_ip = qs['ip'][0]
            send_probe_burst(target_ip=target_ip, port=DISCOVERY_PORT)

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            res = {
                "status": "PROBE_SENT",
                "target": target_ip,
                "local_ip": get_local_ip(),
                "timestamp": time.time()
            }
            self.wfile.write(json.dumps(res).encode('utf-8'))
            return

        # 4. Manuel hedef IP / Port belirleme uç noktası (Kullanıcı alt panelden elle girdiğinde)
        if clean_path == '/api/set_target':
            qs = parse_qs(parsed_url.query)
            target_ip = qs.get('ip', [''])[0].strip()
            target_port = int(qs.get('port', [7400])[0])
            if target_ip:
                discovered_ue5_ip = target_ip
                send_probe_burst(target_ip=target_ip, port=DISCOVERY_PORT)

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            res = {"status": "OK", "target_ip": target_ip, "target_port": target_port, "local_ip": get_local_ip()}
            self.wfile.write(json.dumps(res).encode('utf-8'))
            return

        # 5. Araç komut fırlatma REST API (ARM, DISARM, ESTOP)
        if clean_path == '/api/command':
            qs = parse_qs(parsed_url.query)
            cmd = qs.get('cmd', [''])[0]
            target_host = discovered_ue5_ip or "127.0.0.1"

            # UDP komut paketi gönder (Port 9002 veya 7400)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                cmd_payload = json.dumps({"command": cmd, "timestamp": time.time()}).encode('utf-8')
                sock.sendto(cmd_payload, (target_host, UE5_CONTROL_PORT))
            except Exception:
                pass
            finally:
                sock.close()

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK", "command": cmd, "target": target_host}).encode('utf-8'))
            return

        return super().do_GET()

    def log_message(self, format, *args):
        # Konsolu HTTP GET/POST loglarıyla kirletme, tamamen sessiz tut
        return


def run_udp_telemetry_listener():
    """Unreal Engine'den gelen UDP 7400 telemetri paketlerini dinler"""
    global latest_telemetry, discovered_ue5_ip
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", TELEMETRY_PORT))
        print(f"[Telemetri Dinleyici] 🎧 UDP Port {TELEMETRY_PORT} dinleniyor (Unreal Engine bekleniyor...)")
    except Exception as e:
        print(f"[Telemetri Dinleyici] ⚠️ UDP Port {TELEMETRY_PORT} bağlanamadı: {e}")
        return

    was_connected = False
    while True:
        try:
            data, addr = sock.recvfrom(4096)
            if not data:
                continue

            sender_ip = addr[0]
            local_ip = get_local_ip()
            all_ips = get_all_local_ips()
            # Kendisinden gelen paketleri reddet
            if (sender_ip == "127.0.0.1" or sender_ip == local_ip or sender_ip in all_ips) and local_ip != "127.0.0.1":
                continue

            discovered_ue5_ip = sender_ip
            if discovery_service_ref and discovery_service_ref.target_ue5_ip != sender_ip:
                discovery_service_ref.target_ue5_ip = sender_ip

            text = data.decode('utf-8', errors='ignore')
            try:
                msg = json.loads(text)
                with telemetry_lock:
                    latest_telemetry["timestamp"] = msg.get("timestamp", time.time())
                    latest_telemetry["vehicle_id"] = msg.get("vehicle_id", 0)
                    latest_telemetry["connected"] = True
                    latest_telemetry["ue5_ip"] = addr[0]
                    if "msg" in msg:
                        latest_telemetry["msg"] = str(msg["msg"])
                    else:
                        latest_telemetry["msg"] = "HI FROM UNREAL"

                    if not was_connected:
                        was_connected = True
                        print(f"\n========================================================")
                        print(f"  ✅ [BAĞLANTI KURULDU] Unreal Engine telemetrisi alınıyor!")
                        print(f"     Kaynak IP: {sender_ip} -> Dinlenen Port: {TELEMETRY_PORT}")
                        print(f"========================================================\n")

                    # ArduPilot SITL / PiSim JSON Veri Formatı
                    if "attitude" in msg:
                        att = msg["attitude"]
                        if len(att) >= 3:
                            latest_telemetry["roll"] = round(att[0] * 57.2958, 2)
                            latest_telemetry["pitch"] = round(att[1] * 57.2958, 2)
                            latest_telemetry["yaw"] = round(att[2] * 57.2958, 2)

                    if "airspeed" in msg:
                        latest_telemetry["speed"] = round(msg["airspeed"] * 3.6, 1)

                    if "position" in msg:
                        pos = msg["position"]
                        if len(pos) >= 3:
                            latest_telemetry["altitude"] = round(-pos[2], 1)

                    if "battery" in msg:
                        latest_telemetry["battery"] = round(msg["battery"], 1)

                    if "flight_mode" in msg:
                        latest_telemetry["flight_mode"] = str(msg["flight_mode"])
                    else:
                        latest_telemetry["flight_mode"] = "AKTİF"

            except json.JSONDecodeError:
                pass
        except Exception:
            pass


def launch_in_app_mode(url: str):
    """
    Sayfayı native Uygulama Modunda (Kenarlıksız, adres çubuğu olmadan) açar.
    Raspberry Pi 5 (Chromium), Linux ve Windows ile tam uyumludur.
    """
    app_launched = False
    # Tarayıcının disk önbelleğini kesinlikle atlaması için zaman damgası ekle
    cache_bust_url = f"{url}?t={int(time.time())}"

    # 1. Raspberry Pi 5 / Linux: chromium-browser veya chromium
    linux_browsers = ["chromium-browser", "chromium", "google-chrome"]
    for b in linux_browsers:
        try:
            cmd = [b, f"--app={cache_bust_url}", "--window-size=1280,720", "--start-maximized", "--disable-application-cache"]
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(f"[Launcher] 🚀 '{b}' Uygulama Modunda başlatıldı!")
            app_launched = True
            break
        except FileNotFoundError:
            continue

    # 2. Windows Fallback
    if not app_launched and sys.platform.startswith("win"):
        win_candidates = [
            r"C:\Program Files\Google\Chrome\Application\chrome.exe",
            r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
            r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
            r"C:\Program Files\Microsoft\Edge\Application\msedge.exe"
        ]
        for path in win_candidates:
            if os.path.exists(path):
                subprocess.Popen([path, f"--app={cache_bust_url}", "--window-size=1280,720", "--disable-application-cache"])
                print(f"[Launcher] 🚀 Windows App Modunda başlatıldı: {path}")
                app_launched = True
                break

    # 3. Genel Fallback: Varsayılan tarayıcı
    if not app_launched:
        print("[Launcher] 🌐 Standart web tarayıcısında açılıyor...")
        webbrowser.open(cache_bust_url)


def main():
    local_ip = get_local_ip()
    print("=================================================================")
    print("  🚀 PiSim Studio - Raspberry Pi 5 Tactical Web Dashboard GCS    ")
    print("=================================================================")
    print(f" [+] WebUI Klasörü : {WEBUI_DIR}")
    print(f" [+] Sayfa         : dashboard.html (Taktik Kokpit)")
    print(f" [+] Yerel Adres   : http://localhost:{PORT}/dashboard.html")
    print(f" [+] Ağ Adresi     : http://{local_ip}:{PORT}/dashboard.html")
    print(f" [+] Keşif Radarı  : UDP Port {DISCOVERY_PORT} (Aktif Beacon & Prob)")
    print(f" [+] Telemetri RX  : UDP Port {TELEMETRY_PORT}")
    print("=================================================================")

    global discovery_service_ref
    # 1. Otomatik Keşif Dinleyici Servisini Başlat (Pi 5 durur, dinler, probe atmaz)
    discovery = PiSimDiscoveryService(port=DISCOVERY_PORT)
    discovery_service_ref = discovery

    def on_probe_discovered(ue_ip):
        global discovered_ue5_ip
        discovered_ue5_ip = ue_ip
        with telemetry_lock:
            latest_telemetry["ue5_ip"] = ue_ip
            latest_telemetry["connected"] = True

    discovery.on_probe_received = on_probe_discovered
    discovery.start()

    # 2. UDP Telemetri Dinleyicisini Başlat (Port 7400)
    t = threading.Thread(target=run_udp_telemetry_listener, daemon=True)
    t.start()

    # 3. Yerel HTTP Sunucusunu Başlat (Port 8080)
    class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
        allow_reuse_address = True

    server = ThreadedTCPServer(("0.0.0.0", PORT), PiSimRequestHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    # 4. Tarayıcıyı Taktik Dashboard Modunda Aç
    time.sleep(0.5)
    url = f"http://localhost:{PORT}/dashboard.html"
    launch_in_app_mode(url)

    print("\n💡 İpucu: Dashboard'u kapatmak için bu terminalde Ctrl+C tuşlarına basın.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[PiSim] Dashboard sunucusu kapatılıyor...")
        discovery.stop()
        server.shutdown()
        server.server_close()
        print("[PiSim] Kapatıldı. İyi uçuşlar!")


if __name__ == "__main__":
    main()
