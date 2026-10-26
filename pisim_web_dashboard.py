#!/usr/bin/env python3
"""
pisim_web_dashboard.py - Raspberry Pi 5 & Cross-Platform Tactical Web Dashboard Runner
Universal GCS (Ground Control Station) runner for PiSim Platform.

Runs the HTML5/CSS/JS tactical dashboard on Raspberry Pi 5 (Linux ARM64), Windows, or Jetson:
- Serves local WebUI files on http://localhost:8080 (also accessible from phone/tablet on same LAN)
- Listens for UDP Telemetry (Port 7400/7401) and feeds live state
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

# Ensure UTF-8 output on all consoles
try:
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

PORT = 8080
WEBUI_DIR = Path(__file__).parent / "webui"
TELEMETRY_PORT = 7400
VIDEO_PORT = 7401

latest_telemetry = {
    "timestamp": time.time(),
    "vehicle_id": 0,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "altitude": 0.0,
    "speed": 0.0,
    "battery": 98.0,
    "lat": 39.9334,
    "lon": 32.8597,
    "flight_mode": "READY"
}
telemetry_lock = threading.Lock()

class PiSimRequestHandler(http.server.SimpleHTTPRequestHandler):
    """Custom HTTP handler serving webui and providing live REST telemetry APIs"""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(WEBUI_DIR), **kwargs)

    def do_GET(self):
        # Redirect root to dashboardUnreal.html
        if self.path in ('/', '/index.html'):
            self.send_response(302)
            self.send_header('Location', '/dashboardUnreal.html')
            self.end_headers()
            return

        # Live telemetry API endpoint
        if self.path == '/api/telemetry':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            with telemetry_lock:
                payload = json.dumps(latest_telemetry).encode('utf-8')
            self.wfile.write(payload)
            return

        return super().do_GET()

    def log_message(self, format, *args):
        # Quiet logger for static asset requests
        if args and any(ext in args[0] for ext in ('.css', '.js', '.png', '.jpg', '.ico')):
            return
        super().log_message(format, *args)


def run_udp_telemetry_listener():
    """Listens for UDP telemetry packets from Unreal Engine or SITL"""
    global latest_telemetry
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", TELEMETRY_PORT))
        print(f"[TelemetryListener] 📡 UDP Port {TELEMETRY_PORT} dinleniyor...")
    except Exception as e:
        print(f"[TelemetryListener] ⚠️ UDP Port {TELEMETRY_PORT} bağlanamadı: {e}")
        return

    while True:
        try:
            data, addr = sock.recvfrom(4096)
            if not data:
                continue

            text = data.decode('utf-8', errors='ignore')
            try:
                msg = json.loads(text)
                with telemetry_lock:
                    latest_telemetry["timestamp"] = msg.get("timestamp", time.time())
                    latest_telemetry["vehicle_id"] = msg.get("vehicle_id", 0)

                    # ArduPilot SITL JSON Parsing
                    if "imu" in msg:
                        gyro = msg["imu"].get("gyro", [0, 0, 0])
                        accel = msg["imu"].get("accel_body", [0, 0, 0])
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
            except json.JSONDecodeError:
                pass
        except Exception:
            pass


def launch_in_app_mode(url: str):
    """
    Opens the URL in native App Mode (Borderless, standalone window, no URL bar).
    Compatible with Raspberry Pi 5 (Chromium), Linux, and Windows.
    """
    app_launched = False

    # 1. Raspberry Pi 5 / Linux: chromium-browser or chromium
    linux_browsers = ["chromium-browser", "chromium", "google-chrome"]
    for b in linux_browsers:
        try:
            cmd = [b, f"--app={url}", "--window-size=1280,720", "--start-maximized"]
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(f"[Launcher] 🚀 '{b}' Uygulama Modunda başlatıldı!")
            app_launched = True
            break
        except FileNotFoundError:
            continue

    # 2. Windows fallback
    if not app_launched and sys.platform.startswith("win"):
        win_candidates = [
            r"C:\Program Files\Google\Chrome\Application\chrome.exe",
            r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
            r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
            r"C:\Program Files\Microsoft\Edge\Application\msedge.exe"
        ]
        for path in win_candidates:
            if os.path.exists(path):
                subprocess.Popen([path, f"--app={url}", "--window-size=1280,720"])
                print(f"[Launcher] 🚀 Windows App Modunda başlatıldı: {path}")
                app_launched = True
                break

    # 3. Universal Fallback: Default Browser
    if not app_launched:
        print("[Launcher] 🌐 Standart web tarayıcısında açılıyor...")
        webbrowser.open(url)


def main():
    print("=================================================================")
    print("  🚀 PiSim Studio - Raspberry Pi 5 & Universal Web Dashboard GCS ")
    print("=================================================================")
    print(f" [+] WebUI Klasörü : {WEBUI_DIR}")
    print(f" [+] Yerel Adres   : http://localhost:{PORT}/dashboardUnreal.html")
    print(f" [+] Ağ Adresi     : http://<PI5_IP_ADRESI>:{PORT}/dashboardUnreal.html")
    print("=================================================================")

    # Start UDP telemetry background listener
    t = threading.Thread(target=run_udp_telemetry_listener, daemon=True)
    t.start()

    # Start Local HTTP Server
    class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
        allow_reuse_address = True

    server = ThreadedTCPServer(("0.0.0.0", PORT), PiSimRequestHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    # Automatically launch native-looking dashboard window
    time.sleep(0.5)
    url = f"http://localhost:{PORT}/dashboardUnreal.html"
    launch_in_app_mode(url)

    print("\n💡 İpucu: Dashboard'u kapatmak için bu terminalde Ctrl+C tuşlarına basın.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[PiSim] Dashboard sunucusu kapatılıyor...")
        server.shutdown()
        server.server_close()
        print("[PiSim] Kapatıldı. İyi uçuşlar!")

if __name__ == "__main__":
    main()
