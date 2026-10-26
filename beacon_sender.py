#!/usr/bin/env python3
"""
beacon_sender.py
PiSim Raspberry Pi 5 Otomatik Keşif, Sinyal Verici & Prob Servisi (Auto-Discovery Broadcaster & Probe)

Raspberry Pi 5 veya harici Companion PC çalıştığında:
1. Yerel ağa (UDP Broadcast 255.255.255.255:7499) periyodik olarak 'PISIM_BEACON' sinyali basar.
2. Unreal Engine (PiSim Studio) veya diğer uç cihazlardan gelen 'PISIM_PROBE' isteklerini dinler
   ve anında doğrudan istekçiye kimlik paketini döner.
3. İstenirse tek başına veya pisim_web_dashboard.py içinden arka plan thread'i olarak çalıştırılabilir.
"""

import socket
import json
import time
import sys
import threading

# Prevent Windows console cp1254 UnicodeEncodeError
try:
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    if hasattr(sys.stderr, 'reconfigure'):
        sys.stderr.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

DISCOVERY_PORT = 7499
TELEMETRY_PORT = 7400
VIDEO_PORT = 7401
DASHBOARD_PORT = 8080

def get_local_ip() -> str:
    """Yerel ağdaki gerçek IP adresini tespit eder."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip

def build_beacon_payload(local_ip: str = None) -> dict:
    """PiSim Beacon JSON kimlik paketini oluşturur."""
    if not local_ip:
        local_ip = get_local_ip()
    return {
        "magic": "PISIM_BEACON",
        "device": "Raspberry Pi 5",
        "name": "Raspberry Pi 5 (Companion)",
        "ip": local_ip,
        "telemetry_port": TELEMETRY_PORT,
        "video_port": VIDEO_PORT,
        "dashboard_port": DASHBOARD_PORT,
        "status": "ONLINE",
        "timestamp": time.time()
    }

def send_probe_burst(target_ip: str = "255.255.255.255", port: int = DISCOVERY_PORT, count: int = 3):
    """Ağa hemen keşif probu (PISIM_PROBE) fırlatır."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    try:
        local_ip = get_local_ip()
        probe_msg = json.dumps({
            "magic": "PISIM_PROBE",
            "device": "Raspberry Pi 5",
            "ip": local_ip,
            "timestamp": time.time()
        }).encode('utf-8')

        targets = [target_ip]
        if target_ip == "255.255.255.255":
            targets.append("127.0.0.1")

        for _ in range(count):
            for t in targets:
                try:
                    sock.sendto(probe_msg, (t, port))
                except Exception:
                    pass
            time.sleep(0.05)
        print(f"📡 [BeaconProbe] Keşif probu gönderildi -> {target_ip}:{port}")
    finally:
        sock.close()

class PiSimDiscoveryService:
    """Hem aktif Beacon basan hem de gelen Probelara anında yanıt veren tam çift yönlü keşif servisi."""
    def __init__(self, port: int = DISCOVERY_PORT, broadcast_interval: float = 2.0):
        self.port = port
        self.broadcast_interval = broadcast_interval
        self.local_ip = get_local_ip()
        self.running = False
        self.broadcast_thread = None
        self.listener_thread = None
        self.discovered_peers = {}
        self.lock = threading.Lock()

    def start(self):
        self.running = True
        self.listener_thread = threading.Thread(target=self._listener_loop, daemon=True)
        self.listener_thread.start()

        self.broadcast_thread = threading.Thread(target=self._broadcast_loop, daemon=True)
        self.broadcast_thread.start()

        print("======================================================")
        print("🛰️ [PiSim Auto-Discovery] Aktif Beacon & Prob Servisi Başlatıldı")
        print(f"   Yerel Cihaz    : Raspberry Pi 5 ({self.local_ip})")
        print(f"   Keşif Portu    : UDP {self.port} (Broadcast + Dinleme)")
        print(f"   Yayın Aralığı  : Her {self.broadcast_interval} saniyede bir")
        print("======================================================")

    def stop(self):
        self.running = False

    def _broadcast_loop(self):
        """Yerel ağa periyodik 'Ben buradayım!' beacon paketi fırlatır."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        while self.running:
            try:
                payload = build_beacon_payload(self.local_ip)
                data = json.dumps(payload).encode('utf-8')

                # Hem ağ broadcast'ine hem de yerel loopback'e fırlat
                sock.sendto(data, ("255.255.255.255", self.port))
                sock.sendto(data, ("127.0.0.1", self.port))
            except Exception as e:
                pass
            time.sleep(self.broadcast_interval)

        sock.close()

    def _listener_loop(self):
        """Gelen PISIM_PROBE veya diğer eşlerin BEACON paketlerini dinler."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("0.0.0.0", self.port))
        except Exception as e:
            print(f"⚠️ [BeaconListener] Port {self.port} dinlenemedi: {e}")
            return

        sock.settimeout(1.0)
        while self.running:
            try:
                data, addr = sock.recvfrom(2048)
                if not data:
                    continue

                text = data.decode('utf-8', errors='ignore').strip()
                try:
                    msg = json.loads(text)
                except json.JSONDecodeError:
                    msg = {"raw": text}

                # Kendi yayınladığımız beacon ise yoksay
                if addr[0] == self.local_ip and msg.get("device") == "Raspberry Pi 5":
                    continue

                magic = msg.get("magic", "")

                # 1. Dashboard veya Unreal'dan gelen PROBE isteği
                if "PROBE" in magic or "PISIM_PROBE" in str(msg) or "DISCOVER" in str(msg):
                    print(f"[{time.strftime('%H:%M:%S')}] 🔔 Keşif probu alındı <- {addr[0]}:{addr[1]}")
                    resp = build_beacon_payload(self.local_ip)
                    sock.sendto(json.dumps(resp).encode('utf-8'), addr)
                    with self.lock:
                        self.discovered_peers[addr[0]] = {"last_seen": time.time(), "type": "PROBE_REQUESTER"}

                # 2. Unreal veya başka bir eşten gelen BEACON yanıtı
                elif "BEACON" in magic:
                    with self.lock:
                        self.discovered_peers[addr[0]] = {
                            "last_seen": time.time(),
                            "name": msg.get("name", "Bilinmeyen"),
                            "data": msg
                        }
                    print(f"[{time.strftime('%H:%M:%S')}] 📡 Eş Keşfedildi: {msg.get('device', 'Cihaz')} ({addr[0]})")

            except socket.timeout:
                continue
            except Exception:
                if not self.running:
                    break

        sock.close()

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--probe":
        target = sys.argv[2] if len(sys.argv) > 2 else "255.255.255.255"
        send_probe_burst(target_ip=target)
        sys.exit(0)

    service = PiSimDiscoveryService()
    service.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n🛑 Keşif servisi durduruluyor...")
        service.stop()
