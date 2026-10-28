#!/usr/bin/env python3
"""
pisim_web_dashboard.py - Raspberry Pi 5 & Cross-Platform Tactical Web Dashboard Runner
Universal GCS (Ground Control Station) runner for PiSim Platform.

Runs the HTML5/CSS/JS tactical dashboard on Raspberry Pi 5 (Linux ARM64), Windows, or Jetson:
- Serves local WebUI files on http://localhost:8080 (also accessible from phone/tablet on same LAN)
- Automatically listens for UDP Beacons / Probes (Port 7499)
- Pulls ONLY NEW data from SQL veritabanı (her 60 saniyede bir ve başlangıçta)
- Sol panelden seçilen araca abone olur (SUB_VEHICLE) ve canlı 50Hz UDP telemetrisini alır (Port 7400)
- Sağ panelden video simgesine tıklandığında canlı videoya abone olur (SUB_VIDEO, Port 7401)
- Hiçbir yerde sahte / fake veri veya mesaj üretmez; tüm veriler gerçektir!
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
from beacon_sender import PiSimDiscoveryService, get_local_ip, get_all_local_ips, get_broadcast_targets, send_probe_burst, DISCOVERY_PORT
import telemetry_db

# Ensure UTF-8 output on all consoles (prevent cp1254 crashes on Windows)
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

# GERÇEK TELEMETRİ DURUMU (Sahte/Fake varsayılanlar kesinlikle kaldırıldı)
latest_telemetry = {
    "timestamp": 0.0,
    "last_rx_time": 0.0,
    "vehicle_code": None,
    "vehicle_id": None,
    "vehicle_name": None,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "altitude": 0.0,
    "speed": 0.0,
    "battery": 0.0,
    "lat": 0.0,
    "lon": 0.0,
    "flight_mode": "BEKLEMEDE",
    "connected": False,      # UE5 köprüsü aktif mi
    "streaming": False,      # Seçili araçtan canlı 50Hz akış var mı
    "ue5_ip": None,
    "msg": None,
    "fleet": [],
    "recent_logs": []
}

fleet_telemetry = {}         # Canlı UDP paketleriyle güncellenen aktif araçlar {code: entry}
cached_db_fleet = []         # SQL'den 60 saniyede bir çekilen taban araç listesi
selected_vehicle_code = None # Kullanıcının sol panelden seçtiği aktif araç kodu
telemetry_lock = threading.Lock()
discovered_ue5_ip = None
last_ue5_contact_time = 0.0
discovery_service_ref = None

# Canlı Video Çerçevesi Belleği (Port 7401)
latest_jpeg_frame = None
video_lock = threading.Lock()
is_video_subscribed = False

# Sistem ve Terminal Log Kuyruğu (Dashboard alt log panelinde ve terminalde gösterilir)
system_log_queue = []
system_log_lock = threading.Lock()


def add_system_log(msg: str):
    """Terminalde ve Dashboard alt log panelinde görüntülenecek log mesajı ekler."""
    with system_log_lock:
        t_str = time.strftime("%H:%M:%S")
        entry = f"[{t_str}] {msg}"
        system_log_queue.append(entry)
        if len(system_log_queue) > 60:
            system_log_queue.pop(0)


def pop_recent_system_logs() -> list:
    """Henüz arayüze sunulmamış veya en son eklenen logları liste olarak döner."""
    with system_log_lock:
        return list(system_log_queue[-20:])


def run_sql_sync_worker():
    """
    KURAL: SQL'den ilk açılışta ve sonrasında da her 60 saniyede bir SADECE YENİ GELEN VERİLERİ çeker.
    (Bu aşamada aktif araç olmadığı için hiçbir araç aktif olarak veri aktarmayacak ama biz verileri çektikçe loga yazıcaz).
    """
    global cached_db_fleet
    first_run = True

    while True:
        try:
            res = telemetry_db.sync_sql_data(discovered_ue5_ip)
            new_count = res.get("new_rows_count", 0)
            total = res.get("total_logged", 0)
            v_list = res.get("vehicles", [])
            fleet_data = res.get("fleet", [])

            with telemetry_lock:
                cached_db_fleet = fleet_data

            if first_run:
                first_run = False
                msg = f"[SQL BAŞLANGIÇ] İlk açılış: {new_count} satır yeni veri çekildi (Toplam: {total} satır, Filo: {len(fleet_data)} araç)"
                print(f"🛢️ {msg} | Araçlar: {', '.join(v_list) if v_list else 'Kayıt bulunamadı'}")
                add_system_log(msg)
            else:
                msg = f"[SQL 60sn SENKRONİZASYON] {new_count} yeni satır çekildi (Toplam: {total} satır, Filo: {len(fleet_data)} araç)"
                print(f"🛢️ {msg} | Filodaki Araçlar: {', '.join(v_list) if v_list else 'Henüz kayıt yok'}")
                add_system_log(msg)

        except Exception as e:
            print(f"⚠️ [SQL Sync Hatası]: {e}")

        # Her 60 saniyede bir çalışır
        time.sleep(60.0)


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
        global discovered_ue5_ip, selected_vehicle_code, is_video_subscribed
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
                now = time.time()
                latest_telemetry["local_ip"] = get_local_ip()
                latest_telemetry["ue5_ip"] = discovered_ue5_ip or latest_telemetry.get("ue5_ip")
                latest_telemetry["now"] = now

                # 🛢️ Taban filo listesi (SQL'den 60 saniyede bir çekilen önbellek)
                fleet_map = {}
                for item in cached_db_fleet:
                    code = item["code"]
                    vrx = float(item.get("last_rx_time", 0.0))
                    item_copy = dict(item)
                    item_copy["is_selected"] = (code == selected_vehicle_code)
                    item_copy["connected"] = (now - vrx < 4.0) if vrx > 0 else False
                    item_copy["has_sql"] = True
                    fleet_map[code] = item_copy

                # Sonra bellekteki canlı UDP telemetri ile güncelle / yeni araç varsa ekle
                for code, vdata in fleet_telemetry.items():
                    vrx = float(vdata.get("last_rx_time", 0.0))
                    is_conn = (now - vrx < 4.0)
                    fleet_map[code] = {
                        "code": code,
                        "id": vdata.get("vehicle_id", 0),
                        "name": vdata.get("vehicle_name", f"Rover {vdata.get('vehicle_id', 0) + 1:02d}"),
                        "speed": vdata.get("speed", 0.0),
                        "altitude": vdata.get("altitude", 0.0),
                        "battery": vdata.get("battery", 0.0),
                        "flight_mode": vdata.get("flight_mode", "AKTİF"),
                        "lat": vdata.get("lat", 0.0),
                        "lon": vdata.get("lon", 0.0),
                        "roll": vdata.get("roll", 0.0),
                        "pitch": vdata.get("pitch", 0.0),
                        "yaw": vdata.get("yaw", 0.0),
                        "is_selected": (code == selected_vehicle_code),
                        "connected": is_conn,
                        "has_sql": True,
                        "last_rx_time": vrx
                    }

                fleet_list = list(fleet_map.values())
                # Çevrimiçi araçları öne, ardından araç ID sırasına göre diz
                fleet_list.sort(key=lambda x: (not x.get("connected", False), x.get("id", 0)))

                # Eğer henüz araç seçilmemişse ve filoda araç varsa, ilk aracı varsayılan olarak seç
                if not selected_vehicle_code and fleet_list:
                    selected_vehicle_code = fleet_list[0]["code"]

                for f in fleet_list:
                    f["is_selected"] = (f["code"] == selected_vehicle_code)

                # Seçili aracın durumunu göstergelere yansıt
                sel_is_streaming = False
                if selected_vehicle_code and selected_vehicle_code in fleet_map:
                    sel_item = fleet_map[selected_vehicle_code]
                    vrx = float(sel_item.get("last_rx_time", 0.0))
                    sel_is_streaming = (now - vrx < 4.0 and vrx > 0.0)

                    latest_telemetry["vehicle_code"] = sel_item["code"]
                    latest_telemetry["vehicle_id"] = sel_item["id"]
                    latest_telemetry["vehicle_name"] = sel_item["name"]
                    latest_telemetry["roll"] = sel_item["roll"]
                    latest_telemetry["pitch"] = sel_item["pitch"]
                    latest_telemetry["yaw"] = sel_item["yaw"]
                    latest_telemetry["altitude"] = sel_item["altitude"]
                    latest_telemetry["speed"] = sel_item["speed"]
                    latest_telemetry["battery"] = sel_item["battery"]
                    latest_telemetry["lat"] = sel_item["lat"]
                    latest_telemetry["lon"] = sel_item["lon"]
                    latest_telemetry["flight_mode"] = sel_item["flight_mode"]
                    latest_telemetry["msg"] = sel_item.get("msg")

                # BAĞLANTI DURUMU: UE5'ten keşif sinyali alınmışsa köprü bağlıdır (4 saniyede bir kopmaz!)
                is_ue5_online = (discovered_ue5_ip is not None) or (now - last_ue5_contact_time < 15.0) or (len(fleet_list) > 0)
                latest_telemetry["connected"] = is_ue5_online
                latest_telemetry["streaming"] = sel_is_streaming
                latest_telemetry["fleet"] = fleet_list
                latest_telemetry["selected_vehicle_code"] = selected_vehicle_code
                latest_telemetry["video_subscribed"] = is_video_subscribed
                latest_telemetry["recent_logs"] = pop_recent_system_logs()

                payload = json.dumps(latest_telemetry).encode('utf-8')

            self.wfile.write(payload)
            return

        # 2b. Canlı SQL Telemetri Tablosu REST API uç noktası
        if clean_path == '/api/sql_table':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            qs = parse_qs(parsed_url.query)
            max_rows = int(qs.get('max_rows', [50])[0])
            v_code = qs.get('code', [None])[0]
            stats = telemetry_db.get_stats()
            rows = telemetry_db.get_recent_rows(max_rows, vehicle_code=v_code)
            res = {
                "status": "OK",
                "total_logged": stats["total_logged"],
                "db_path": stats["db_path"],
                "sql_dump_path": stats["sql_dump_path"],
                "rows": rows
            }
            self.wfile.write(json.dumps(res).encode('utf-8'))
            return

        # 2c. Belirtilen araca abone olma REST API (Sol panelden araca tıklandığında)
        if clean_path == '/api/select_vehicle':
            qs = parse_qs(parsed_url.query)
            code = qs.get('code', [''])[0].strip()
            v_id = int(qs.get('id', [-1])[0])
            target_host = discovered_ue5_ip or "127.0.0.1"

            with telemetry_lock:
                if code:
                    selected_vehicle_code = code
                    # Varsa son bilinen değerleri latest_telemetry'ye kopyala
                    if code in fleet_telemetry:
                        latest_telemetry.update(fleet_telemetry[code])
                    else:
                        sql_rows = telemetry_db.get_recent_rows(max_rows=1, vehicle_code=code)
                        if sql_rows:
                            r = sql_rows[0]
                            latest_telemetry.update({
                                "vehicle_code": r["vehicle_code"],
                                "vehicle_id": r["vehicle_id"],
                                "vehicle_name": f"Rover {r['vehicle_id'] + 1:02d}",
                                "roll": r["roll"],
                                "pitch": r["pitch"],
                                "yaw": r["yaw"],
                                "altitude": r["altitude"],
                                "speed": r["speed"],
                                "battery": r["battery"],
                                "lat": r["lat"],
                                "lon": r["lon"],
                                "flight_mode": r["mode"],
                                "msg": f"SQL_SON_KAYIT ({r['vehicle_code']})"
                            })

            # KURAL: Sol panelde seçilen araç aktif araç olur ve onun telemetrisine abone olunur (Port 7400)
            sub_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sub_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            try:
                cmd_dict = {
                    "cmd": "SUB_VEHICLE",
                    "vehicle_code": code,
                    "vehicle_id": v_id,
                    "client_ip": get_local_ip(),
                    "telemetry_port": TELEMETRY_PORT
                }
                payload_bytes = json.dumps(cmd_dict).encode('utf-8')
                # Hedefe doğrudan at
                sub_sock.sendto(payload_bytes, (target_host, DISCOVERY_PORT))
                # Tüm yerel ağ broadcast hedeflerine de at
                for b_tgt, b_prt in get_broadcast_targets(DISCOVERY_PORT):
                    try:
                        sub_sock.sendto(payload_bytes, (b_tgt, b_prt))
                    except Exception:
                        pass

                msg_log = f"🎯 [ABONELİK TALEBİ] Araç '{code}' seçildi -> Unreal Engine'e abone olma isteği gönderildi (Port {TELEMETRY_PORT})"
                print(msg_log)
                add_system_log(msg_log)

            except Exception as e:
                print(f"⚠️ [SUB_VEHICLE Hatası]: {e}")
            finally:
                sub_sock.close()

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK", "selected": code, "target": target_host}).encode('utf-8'))
            return

        # 2d. Canlı Video Aboneliği Açma/Kapatma REST API (Sağ paneldeki video simgesine tıklandığında)
        if clean_path == '/api/subscribe_video':
            qs = parse_qs(parsed_url.query)
            code = qs.get('code', [''])[0].strip() or selected_vehicle_code or ""
            subscribe_str = qs.get('subscribe', ['true'])[0].strip().lower()
            b_sub = (subscribe_str in ('true', '1', 'yes'))
            is_video_subscribed = b_sub

            target_host = discovered_ue5_ip or "127.0.0.1"
            sub_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sub_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            try:
                cmd_dict = {
                    "cmd": "SUB_VIDEO",
                    "vehicle_code": code,
                    "subscribe": b_sub,
                    "client_ip": get_local_ip(),
                    "video_port": VIDEO_PORT
                }
                payload_bytes = json.dumps(cmd_dict).encode('utf-8')
                sub_sock.sendto(payload_bytes, (target_host, DISCOVERY_PORT))
                for b_tgt, b_prt in get_broadcast_targets(DISCOVERY_PORT):
                    try:
                        sub_sock.sendto(payload_bytes, (b_tgt, b_prt))
                    except Exception:
                        pass

                status_txt = "AÇILDI (Port 7401 On-Demand)" if b_sub else "KAPATILDI"
                msg_log = f"📹 [VİDEO ABONELİK] Araç '{code}' video yayını: {status_txt}"
                print(msg_log)
                add_system_log(msg_log)

            except Exception as e:
                print(f"⚠️ [SUB_VIDEO Hatası]: {e}")
            finally:
                sub_sock.close()

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK", "video_subscribed": b_sub, "vehicle_code": code}).encode('utf-8'))
            return

        # 2e. Canlı JPEG Video Karesi REST API (Sağ paneldeki küçük pencere için)
        if clean_path == '/api/video_frame':
            with video_lock:
                frame = latest_jpeg_frame

            if frame:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.end_headers()
                self.wfile.write(frame)
            else:
                self.send_response(204) # No Content
                self.end_headers()
            return

        # 2f. Eski / Çevrimdışı Araçları Temizleme REST API
        if clean_path == '/api/clear_fleet':
            with telemetry_lock:
                fleet_telemetry.clear()
                selected_vehicle_code = None
                telemetry_db.clear_fleet()
                add_system_log("🧹 Filo kayıtları ve SQL veritabanı temizlendi.")

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK", "message": "Filo ve SQL temizlendi"}).encode('utf-8'))
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

        # 4. Manuel hedef IP / Port belirleme uç noktası
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

            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                cmd_payload = json.dumps({
                    "command": cmd,
                    "vehicle_code": selected_vehicle_code,
                    "timestamp": time.time()
                }).encode('utf-8')
                sock.sendto(cmd_payload, (target_host, UE5_CONTROL_PORT))
                add_system_log(f"[KOMUT TX] '{cmd}' -> Araç: {selected_vehicle_code}")
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
        # Konsolu gereksiz HTTP GET/POST loglarıyla kirletme
        return


def run_udp_telemetry_listener():
    """
    Unreal Engine'den gelen UDP 7400 canlı telemetri paketlerini dinler.
    KURAL: Araç sadece abone olunduğunda buraya paket atar. Gelen paketler %100 gerçektir.
    """
    global latest_telemetry, discovered_ue5_ip, last_ue5_contact_time, selected_vehicle_code
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", TELEMETRY_PORT))
        print(f"[Telemetri Dinleyici] 🎧 UDP Port {TELEMETRY_PORT} dinleniyor (Abone olunan araç bekleniyor...)")
    except Exception as e:
        print(f"[Telemetri Dinleyici] ⚠️ UDP Port {TELEMETRY_PORT} bağlanamadı: {e}")
        return

    was_connected = False
    last_print_time = 0.0

    while True:
        try:
            data, addr = sock.recvfrom(4096)
            if not data:
                continue

            sender_ip = addr[0]
            discovered_ue5_ip = sender_ip
            last_ue5_contact_time = time.time()

            if discovery_service_ref and discovery_service_ref.target_ue5_ip != sender_ip:
                discovery_service_ref.target_ue5_ip = sender_ip

            text = data.decode('utf-8', errors='ignore')
            try:
                msg = json.loads(text)
                with telemetry_lock:
                    now_ts = time.time()
                    v_id = int(msg.get("vehicle_id", 0))
                    v_code = str(msg.get("vehicle_code", f"rover_{v_id+1:02d}"))
                    v_name = str(msg.get("vehicle_name", f"Rover {v_id+1:02d}"))
                    rx_msg_content = str(msg.get("msg") or f"TELEMETRİ_{v_code}")

                    if not was_connected:
                        was_connected = True
                        print(f"\n========================================================")
                        print(f"  ✅ [CANLI YAYIN BAŞLADI] Unreal Engine'den telemetri akıyor!")
                        print(f"     Kaynak IP  : {sender_ip} -> Dinlenen Port: {TELEMETRY_PORT}")
                        print(f"     Araç Kodu  : {v_code} ({v_name})")
                        print(f"     Selamlama  : \"{rx_msg_content}\"")
                        print(f"========================================================\n")
                        add_system_log(f"✅ [CANLI AKIŞ] Unreal Engine ({sender_ip}) telemetrisi alınıyor: '{rx_msg_content}'")

                    # 1. Oryantasyon / Tutum (Roll, Pitch, Yaw)
                    roll, pitch, yaw = 0.0, 0.0, 0.0
                    if "roll" in msg:
                        roll = round(float(msg.get("roll", 0.0)), 2)
                        pitch = round(float(msg.get("pitch", 0.0)), 2)
                        yaw = round(float(msg.get("yaw", 0.0)), 2)
                    elif "attitude" in msg:
                        att = msg["attitude"]
                        if isinstance(att, list) and len(att) >= 3:
                            roll = round(att[0] * 57.2958, 2)
                            pitch = round(att[1] * 57.2958, 2)
                            yaw = round(att[2] * 57.2958, 2)

                    # 2. Hız (km/s)
                    speed = 0.0
                    if "speed" in msg:
                        speed = round(float(msg["speed"]), 1)
                    elif "airspeed" in msg:
                        speed = round(float(msg["airspeed"]) * 3.6, 1)

                    # 3. İrtifa (m)
                    altitude = 0.0
                    if "altitude" in msg:
                        altitude = round(float(msg["altitude"]), 1)
                    elif "position" in msg and len(msg["position"]) >= 3:
                        altitude = round(-msg["position"][2], 1)

                    # 4. Batarya (%)
                    battery = round(float(msg.get("battery", 0.0)), 1)
                    if "ros2_battery" in msg and isinstance(msg["ros2_battery"], dict):
                        pct = msg["ros2_battery"].get("percentage", 0.0)
                        battery = round(float(pct) * 100.0 if pct <= 1.0 else float(pct), 1)

                    # 5. GPS Koordinatları
                    lat = float(msg.get("lat", 0.0))
                    lon = float(msg.get("lon", 0.0))
                    if "ros2_navsat" in msg and isinstance(msg["ros2_navsat"], dict):
                        lat = float(msg["ros2_navsat"].get("latitude", lat))
                        lon = float(msg["ros2_navsat"].get("longitude", lon))

                    flight_mode = str(msg.get("flight_mode", "AKTİF"))

                    # Gerçek canlı araç kaydı
                    entry = {
                        "timestamp": now_ts,
                        "sim_timestamp": float(msg.get("timestamp", 0.0)),
                        "last_rx_time": now_ts,
                        "vehicle_code": v_code,
                        "vehicle_id": v_id,
                        "vehicle_name": v_name,
                        "connected": True,
                        "ue5_ip": sender_ip,
                        "msg": rx_msg_content,
                        "roll": roll,
                        "pitch": pitch,
                        "yaw": yaw,
                        "speed": speed,
                        "altitude": altitude,
                        "battery": battery,
                        "lat": lat,
                        "lon": lon,
                        "flight_mode": flight_mode
                    }

                    fleet_telemetry[v_code] = entry

                    # Eğer henüz seçili araç yoksa gelen bu araca kilitlen
                    if not selected_vehicle_code:
                        selected_vehicle_code = v_code

                    # Seçili araç bu araçsa latest_telemetry güncelle
                    if selected_vehicle_code == v_code:
                        latest_telemetry.update(entry)
                        latest_telemetry["connected"] = True
                        latest_telemetry["streaming"] = True

                # Terminal log basımı (1.5 saniyede bir)
                current_time = time.time()
                if current_time - last_print_time >= 1.5:
                    last_print_time = current_time
                    stats = telemetry_db.get_stats()
                    print(f"📩 [RX ← UE5 ({sender_ip})]: Araç \"{v_code}\" | \"{rx_msg_content}\" (Port {TELEMETRY_PORT})")
                    print(f"🛢️ [SQL VERİTABANI]: {stats['total_logged']} satır kayıtlı | R:{roll}° P:{pitch}° Alt:{altitude}m | Seçili: {selected_vehicle_code}")
                    add_system_log(f"RX UE5 ({sender_ip}): Araç '{v_code}' -> \"{rx_msg_content}\" (Hız: {speed}km/s, İrtifa: {altitude}m)")

            except json.JSONDecodeError:
                pass
        except Exception:
            pass


def run_udp_video_listener():
    """
    Unreal Engine'den gelen UDP 7401 JPEG video karelerini dinler.
    KURAL: Kendi videosuna abone olanlara videosunu yayınlar (hiçbir yere depolamadan).
    """
    global latest_jpeg_frame
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", VIDEO_PORT))
        print(f"[Video Dinleyici] 📹 UDP Port {VIDEO_PORT} dinleniyor (On-Demand JPEG video yayını)...")
    except Exception as e:
        print(f"[Video Dinleyici] ⚠️ UDP Port {VIDEO_PORT} bağlanamadı: {e}")
        return

    while True:
        try:
            data, addr = sock.recvfrom(65535)
            if data and len(data) > 64:
                # Başlangıç SOI (FF D8) kontrolü
                if data[:2] == b'\xff\xd8':
                    with video_lock:
                        latest_jpeg_frame = data
        except Exception:
            pass


def launch_in_app_mode(url: str):
    """
    Sayfayı native Uygulama Modunda (Kenarlıksız, adres çubuğu olmadan) açar.
    Raspberry Pi 5 (Chromium), Linux ve Windows ile tam uyumludur.
    """
    app_launched = False
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
    print(f" [+] Keşif Radarı  : UDP Port {DISCOVERY_PORT} (Beacon & Probe)")
    print(f" [+] Telemetri RX  : UDP Port {TELEMETRY_PORT}")
    print(f" [+] Canlı Video RX: UDP Port {VIDEO_PORT}")
    print("=================================================================")

    global discovery_service_ref
    # 1. Keşif Dinleyici Servisini Başlat (UDP Port 7499)
    discovery = PiSimDiscoveryService(port=DISCOVERY_PORT)
    discovery_service_ref = discovery

    def on_probe_discovered(ue_ip):
        global discovered_ue5_ip, last_ue5_contact_time
        discovered_ue5_ip = ue_ip
        last_ue5_contact_time = time.time()
        with telemetry_lock:
            latest_telemetry["ue5_ip"] = ue_ip
            latest_telemetry["connected"] = True
        add_system_log(f"[KEŞİF] Unreal Engine eşleştirildi: {ue_ip}")

    discovery.on_probe_received = on_probe_discovered
    discovery.start()

    # 2. Arka Plan SQL Senkronizasyon İşçisini Başlat (Her 60 saniyede bir ve başlangıçta)
    sql_thread = threading.Thread(target=run_sql_sync_worker, daemon=True, name="SqlSyncWorker")
    sql_thread.start()

    # 3. UDP Canlı Telemetri Dinleyicisini Başlat (Port 7400)
    t_telemetry = threading.Thread(target=run_udp_telemetry_listener, daemon=True, name="TelemetryListener")
    t_telemetry.start()

    # 4. UDP Canlı Video Dinleyicisini Başlat (Port 7401)
    t_video = threading.Thread(target=run_udp_video_listener, daemon=True, name="VideoListener")
    t_video.start()

    # 5. Yerel HTTP Sunucusunu Başlat (Port 8080)
    class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
        allow_reuse_address = True

    server = ThreadedTCPServer(("0.0.0.0", PORT), PiSimRequestHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True, name="HttpServer")
    server_thread.start()

    # 6. Tarayıcıyı Taktik Dashboard Modunda Aç
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
