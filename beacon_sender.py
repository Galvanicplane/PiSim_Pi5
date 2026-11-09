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

_cached_local_ips = []
_last_ip_check_time = 0.0

def get_all_local_ips(force_refresh: bool = False) -> list:
    """Tüm aktif yerel ağ IPv4 adreslerini tespit eder (Bellekte önbelleklenir, subprocess/DNS gecikmesi yaratmaz)."""
    global _cached_local_ips, _last_ip_check_time
    now = time.time()
    if not force_refresh and _cached_local_ips and (now - _last_ip_check_time < 30.0):
        return _cached_local_ips

    found_ips = []

    # 1. Yerel alt ağ arayüz yoklaması (En hızlı yöntem: < 0.2ms, subprocess veya DNS gerektirmez)
    local_probes = [('192.168.1.1', 80), ('192.168.0.1', 80), ('10.0.0.1', 80), ('172.16.0.1', 80)]
    for target in local_probes:
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

    # 2. Linux / Raspberry Pi: hostname -I komutu (Sadece 30 saniyede bir çalışır)
    if sys.platform.startswith("linux"):
        try:
            out = subprocess.check_output(["hostname", "-I"], timeout=0.4).decode('utf-8', errors='ignore').strip()
            for ip in out.split():
                if not ip.startswith("127.") and ":" not in ip:
                    found_ips.append(ip)
        except Exception:
            pass

    # 3. gethostbyname_ex fallback (Yalnızca yukarıdakiler bulamadıysa)
    if not found_ips:
        try:
            host = socket.gethostname()
            for ip in socket.gethostbyname_ex(host)[2]:
                if not ip.startswith("127."):
                    found_ips.append(ip)
        except Exception:
            pass

    clean_ips = list(dict.fromkeys(found_ips))
    _cached_local_ips = clean_ips if clean_ips else ["127.0.0.1"]
    _last_ip_check_time = now
    return _cached_local_ips

def get_local_ip() -> str:
    """Yerel ağdaki gerçek IP adresini tespit eder (Önbellekten anında döner)."""
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
    targets = [("255.255.255.255", port)]
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
    """Pi 5 durur ve dinler (Port 7499). Unreal Engine probe attığında anında ve sürekli 'HI FROM PI5' döner."""
    def __init__(self, port: int = DISCOVERY_PORT):
        self.port = port
        self.local_ip = get_local_ip()
        self.running = False
        self.listener_thread = None
        self.heartbeat_thread = None
        self.target_ue5_ip = None
        self.on_probe_received = None
        self.on_sql_rows_received = None

    def request_sql_table(self, target_ip: str = None):
        """Unreal Engine'den UDP Port 7499 üzerinden SQL telemetri tablosunu talep eder."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            req_msg = json.dumps({
                "magic": "PISIM_REQ_SQL_TABLE",
                "client_ip": self.local_ip,
                "port": self.port,
                "timestamp": time.time()
            }).encode('utf-8')

            targets = get_broadcast_targets(self.port)
            dest_ip = target_ip or self.target_ue5_ip
            if dest_ip and dest_ip not in ("255.255.255.255", "127.0.0.1"):
                targets.append((dest_ip, self.port))

            for t in set(targets):
                try:
                    sock.sendto(req_msg, t)
                except Exception:
                    pass
        finally:
            sock.close()

    def start(self):
        self.running = True
        self.local_ip = get_local_ip()
        self.listener_thread = threading.Thread(target=self._listener_loop, daemon=True)
        self.listener_thread.start()
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self.heartbeat_thread.start()
        print(f"[Keşif Servisi] 🎧 Pi 5 dinlemede (UDP Port {self.port} - Unreal Engine probe'u bekleniyor...)")

    def stop(self):
        self.running = False

    def _heartbeat_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        while self.running:
            if self.target_ue5_ip and (self.target_ue5_ip != self.local_ip or self.target_ue5_ip == "127.0.0.1"):
                try:
                    hb_msg = json.dumps({
                        "magic": "PISIM_HEARTBEAT",
                        "device": "Raspberry Pi 5",
                        "name": "Raspberry Pi 5 (Companion)",
                        "ip": self.local_ip,
                        "telemetry_port": TELEMETRY_PORT,
                        "msg": "HI FROM PI5",
                        "status": "ONLINE",
                        "timestamp": time.time()
                    }).encode('utf-8')
                    sock.sendto(hb_msg, (self.target_ue5_ip, self.port))
                except Exception:
                    pass
            time.sleep(1.5)
        sock.close()

    def _listener_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("0.0.0.0", self.port))
        except Exception as e:
            print(f"⚠️ [Keşif Servisi] UDP Port {self.port} bağlanamadı: {e}")
            return

        sock.settimeout(1.0)
        while self.running:
            try:
                data, addr = sock.recvfrom(2048)
                if not data:
                    continue

                sender_ip = addr[0]
                if sender_ip == self.local_ip and self.local_ip != "127.0.0.1":
                    continue

                text = data.decode('utf-8', errors='ignore').strip()
                try:
                    msg = json.loads(text)
                except Exception:
                    msg = {"raw": text}

                # Unreal Engine probe veya heartbeat isteği yakalandığında
                if msg.get("magic") in ("PISIM_PROBE", "PISIM_HEARTBEAT") or "UE5" in msg.get("source", "") or "UNREAL" in str(msg):
                    is_first = (self.target_ue5_ip != sender_ip)
                    self.target_ue5_ip = sender_ip
                    resp = {
                        "magic": "PISIM_BEACON",
                        "device": "Raspberry Pi 5",
                        "name": "Raspberry Pi 5 (Companion)",
                        "ip": self.local_ip,
                        "telemetry_port": TELEMETRY_PORT,
                        "status": "ONLINE",
                        "timestamp": time.time()
                    }
                    sock.sendto(json.dumps(resp).encode('utf-8'), addr)
                    v_name = msg.get("vehicle_name") or "Rover"
                    v_code = msg.get("vehicle_code") or "rover"
                    rx_msg = msg.get("msg") or f"HI FROM {v_name}"
                    if is_first:
                        print(f"📡 [Keşif RX ← {v_name} ({v_code}) @ {sender_ip}]: \"{rx_msg}\" (Port {self.port})")
                        print(f"📤 [Keşif TX → {v_name} ({sender_ip})]: Pi 5 Beacon iletildi (Port {self.port})")
                    if self.on_probe_received:
                        self.on_probe_received(sender_ip, msg)

                elif msg.get("magic") == "PISIM_ACK_SUB":
                    print(f"🎯 [Abonelik Onayı RX ← UE5 ({sender_ip})]: Araç '{msg.get('vehicle_code')}' abone yapıldı!")
                elif msg.get("magic") == "PISIM_ACK_VIDEO":
                    print(f"📹 [Video Onayı RX ← UE5 ({sender_ip})]: Araç '{msg.get('vehicle_code')}' video: {msg.get('subscribed')}")
                elif msg.get("magic") == "PISIM_RESP_SQL_TABLE":
                    table_data = msg.get("data", {})
                    rows = table_data.get("rows", [])
                    if rows:
                        import telemetry_db
                        added = telemetry_db.add_sql_rows_from_network(rows)
                        if added > 0:
                            print(f"📥 [SQL Ağ Senkronizasyonu RX ← UE5 ({sender_ip})]: +{added} yeni telemetri satırı SQLite veritabanına işlendi!")
                        if self.on_sql_rows_received:
                            self.on_sql_rows_received(added, len(rows), sender_ip)

            except socket.timeout:
                continue
            except Exception:
                pass

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
