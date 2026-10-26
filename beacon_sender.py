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

import subprocess

def get_all_local_ips() -> list:
    """Tüm aktif yerel ağ IPv4 adreslerini tespit eder (İnternet bağlantısı olmasa dahi eth/wifi adresini bulur)."""
    found_ips = []

    # 1. Linux / Raspberry Pi: hostname -I komutu (En garantisi, internet gerektirmez)
    if sys.platform.startswith("linux"):
        try:
            out = subprocess.check_output(["hostname", "-I"], timeout=1).decode('utf-8', errors='ignore').strip()
            for ip in out.split():
                if not ip.startswith("127.") and ":" not in ip:
                    found_ips.append(ip)
        except Exception:
            pass

    # 2. Standart soket denemeleri
    probe_targets = [('8.8.8.8', 80), ('192.168.1.1', 80), ('10.0.0.1', 80)]
    for target in probe_targets:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(target)
            ip = s.getsockname()[0]
            s.close()
            if ip and not ip.startswith("127."):
                found_ips.append(ip)
                break
        except Exception:
            pass

    # 3. gethostbyname_ex fallback
    try:
        host = socket.gethostname()
        for ip in socket.gethostbyname_ex(host)[2]:
            if not ip.startswith("127."):
                found_ips.append(ip)
    except Exception:
        pass

    clean_ips = list(dict.fromkeys(found_ips))
    return clean_ips if clean_ips else ["127.0.0.1"]

def get_local_ip() -> str:
    """Yerel ağdaki gerçek IP adresini tespit eder."""
    ips = get_all_local_ips()
    return ips[0]

def build_beacon_payload(local_ip: str = None) -> dict:
    """PiSim Beacon JSON kimlik paketini oluşturur."""
    if not local_ip or local_ip == "127.0.0.1":
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

def get_broadcast_targets(port: int = DISCOVERY_PORT) -> list:
    """Yerel ağın subnet broadcast (örn: 192.168.1.255) ve global broadcast adreslerini döner."""
    targets = [("255.255.255.255", port), ("127.0.0.1", port)]
    for ip in get_all_local_ips():
        if ip and not ip.startswith("127."):
            parts = ip.split('.')
            if len(parts) == 4:
                targets.append((f"{parts[0]}.{parts[1]}.{parts[2]}.255", port))
    return list(set(targets))


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

        targets = get_broadcast_targets(port)
        if target_ip not in ("255.255.255.255", "127.0.0.1"):
            targets.append((target_ip, port))

        for _ in range(count):
            for t in set(targets):
                try:
                    sock.sendto(probe_msg, t)
                except Exception:
                    pass
            time.sleep(0.05)
        print(f"📡 [BeaconProbe] Keşif probu gönderildi -> Hedefler: {set(targets)}")
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

                for target in get_broadcast_targets(self.port):
                    try:
                        sock.sendto(data, target)
                    except Exception:
                        pass
            except Exception:
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
