#!/usr/bin/env python3
"""
beacon_sender.py
PiSim Raspberry Pi 5 Otomatik Keşif & Beacon Yayını (Auto-Discovery Broadcaster)

Raspberry Pi 5 veya harici Companion PC ağa bağlandığında, yerel ağa (UDP Broadcast)
düzenli aralıklarla "Ben buradayım!" paketi fırlatır.
Unreal Engine (PiSim Studio) bu paketi duyduğu an ekranda:
"⚡ Yeni Donanım Bulundu: Raspberry Pi 5 (192.168.1.X) -> [EŞLEŞTİR]"
butonunu çıkartır ve kullanıcı IP yazma derdinden kurtulur.
"""

import socket
import json
import time
import sys

def get_local_ip():
    """Yerel ağdaki gerçek IP adresini tespit eder."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Harici bir hedefe dummy bağlantı açarak arayüz IP'sini öğren
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip

def start_secure_beacon_service(listen_port: int = 7499, telemetry_port: int = 7400, video_port: int = 7401):
    """
    Güvenli Dinleme Modu (Secure Probe-Response):
    Sürekli ağa broadcast spam atmaz. 
    Sadece operatör Dashboard'u açıp 'PISIM_PROBE' keşif paketi gönderdiğinde
    doğrudan Dashboard'un IP adresine kimliğini yanıt olarak döner.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        sock.bind(('0.0.0.0', listen_port))
    except Exception as e:
        print(f"❌ [Hata] Port {listen_port} dinlenemedi: {e}")
        return

    local_ip = get_local_ip()
    print("======================================================")
    print("🛡️ [PiSim Beacon] Güvenli Keşif Servisi Aktif (Gizli Mod)")
    print(f"   Cihaz: Raspberry Pi 5 (Companion)")
    print(f"   Yerel IP: {local_ip}")
    print(f"   Dinleme Portu: UDP {listen_port}")
    print("   Durum: Ağ kirliliği yapmadan Dashboard probu bekleniyor...")
    print("======================================================")

    try:
        while True:
            data, addr = sock.recvfrom(1024)
            msg = data.decode('utf-8', errors='ignore').strip()

            # Dashboard açıldığında gelen Probe isteği
            if "PISIM_PROBE" in msg or "DISCOVER" in msg:
                print(f"[{time.strftime('%H:%M:%S')}] 🔔 Dashboard keşif probu yakalandı! İstekçi: {addr[0]}:{addr[1]}")
                
                response_payload = {
                    "magic": "PISIM_BEACON",
                    "device": "Raspberry Pi 5",
                    "ip": local_ip,
                    "telemetry_port": telemetry_port,
                    "video_port": video_port,
                    "status": "ONLINE",
                    "timestamp": time.time()
                }

                resp_bytes = json.dumps(response_payload).encode('utf-8')
                sock.sendto(resp_bytes, addr)
                print(f"   ↳ Yanıt doğrudan {addr[0]}:{addr[1]} adresine iletildi. (Güvenli)")

    except KeyboardInterrupt:
        print("\n🛑 Güvenli Beacon servisi kapatıldı.")
    finally:
        sock.close()

if __name__ == "__main__":
    b_port = 7499
    if len(sys.argv) > 1:
        b_port = int(sys.argv[1])
    start_secure_beacon_service(listen_port=b_port)
