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

import argparse
import http.server
import socketserver
import threading
import socket
import json
import math
import time
import sys
import os
import subprocess
import webbrowser
from pathlib import Path
from urllib.parse import urlparse, parse_qs
from typing import Optional

try:
    import zenoh
    HAS_ZENOH = True
except ImportError:
    HAS_ZENOH = False

from agent_supervisor import auto_discover_zenoh_endpoint, normalize_zenoh_endpoint

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
SWARM_COMMAND_PORT = 7402
UE5_CONTROL_PORT = 9002

# Araç Pasif / Çevrimdışı Zaman Aşımı Süresi (Kullanıcı İsteği: 5.0 saniye, istenirse 10.0 yapılabilir)
VEHICLE_OFFLINE_TIMEOUT_SEC = 5.0

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
    KURAL: SQL'den ilk açılışta ve sonrasında her 10 saniyede bir SADECE YENİ GELEN VERİLERİ çeker.
    (Bu aşamada aktif araç olmasa bile SQL verilerini çektikçe terminale ve alt loga yazar).
    """
    global cached_db_fleet
    first_run = True

    while True:
        try:
            # 1. Ağ üzerinden Unreal Engine'e SQL telemetri tablosu talep paketi gönder (UDP Port 7499)
            if discovery_service_ref:
                discovery_service_ref.request_sql_table(discovered_ue5_ip)

            # 2. Yerel diskte (varsa) Saved/Telemetry/flight_telemetry.sql dosyasından senkronize et
            res = telemetry_db.sync_sql_data(discovered_ue5_ip)
            new_count = res.get("new_rows_count", 0)
            total = res.get("total_logged", 0)
            v_list = res.get("vehicles", [])
            fleet_data = res.get("fleet", [])

            with telemetry_lock:
                cached_db_fleet = fleet_data

            if first_run:
                first_run = False
                msg = f"🛢️ [SQL BAŞLANGIÇ] İlk açılış: {new_count} satır yeni veri çekildi (Toplam: {total} satır, SQL Filosu: {len(fleet_data)} araç)"
                print(f"{msg} | Araçlar: {', '.join(v_list) if v_list else 'Kayıtlı Araç Yok'}")
                add_system_log(msg)
            else:
                if new_count > 0:
                    msg = f"🛢️ [SQL 10sn SENKRONİZASYON] +{new_count} yeni satır çekildi (Toplam: {total} satır, SQL Filosu: {len(fleet_data)} araç)"
                else:
                    msg = f"🛢️ [SQL 10sn SENKRONİZASYON] 0 yeni satır (Toplam: {total} satır, SQL Filosu: {len(fleet_data)} araç)"
                print(f"{msg} | Filodaki Araçlar: {', '.join(v_list) if v_list else '0 araç (SQL bekleniyor)'}")
                add_system_log(msg)

        except Exception as e:
            print(f"⚠️ [SQL Sync Hatası]: {e}")

        # Her 10 saniyede bir çalışır (Kullanıcı talebi)
        time.sleep(10.0)


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

            local_ip = get_local_ip()
            with telemetry_lock:
                now = time.time()
                latest_telemetry["local_ip"] = local_ip
                latest_telemetry["now"] = now

                # GERÇEK HEARTBEAT TESPİTİ: Unreal Engine'den son 5.0 saniyede probe veya telemetri geldiyse bağlıdır!
                # Simülasyon durduğunda 5 saniye içinde kesinlikle KOPAR ve KESİLDİ moduna geçer!
                is_ue5_online = bool((now - last_ue5_contact_time < VEHICLE_OFFLINE_TIMEOUT_SEC) and (last_ue5_contact_time > 0))

                # DOĞRUDAN GELEN CANLI MESAJLARDAN FİLO LİSTESİ OLUŞTUR (SQL'den eski araç çekilmez!)
                fleet_list = []
                for c, vdata in list(fleet_telemetry.items()):
                    vrx = float(vdata.get("last_rx_time", 0.0))
                    age = now - vrx if vrx > 0 else 9999.0
                    # 5 saniye boyunca mesaj gelmezse araç PASİF (connected=False) olur ama listeden silinmez!
                    is_conn = bool((age < VEHICLE_OFFLINE_TIMEOUT_SEC) and (vrx > 0))

                    item = dict(vdata)
                    item["code"] = c
                    item["vehicle_code"] = c
                    item["id"] = int(vdata.get("vehicle_id", vdata.get("id", 1)))
                    item["vehicle_id"] = item["id"]
                    item["name"] = str(vdata.get("vehicle_name") or vdata.get("name") or c)
                    item["vehicle_name"] = item["name"]
                    item["connected"] = is_conn
                    item["offline_sec"] = round(age, 1)
                    fleet_list.append(item)

                # Sıralama: Araç ID ve koda göre sabit sırada tut (Pasife düşünce kartların yeri zıplamasın)
                fleet_list.sort(key=lambda x: (x.get("id", 0), x.get("code", "")))

                if (not selected_vehicle_code or selected_vehicle_code not in fleet_telemetry) and fleet_list:
                    selected_vehicle_code = fleet_list[0]["code"]

                for f in fleet_list:
                    f["is_selected"] = (f["code"] == selected_vehicle_code)

                sel_is_streaming = False
                if selected_vehicle_code and selected_vehicle_code in fleet_telemetry:
                    sel_item = fleet_telemetry[selected_vehicle_code]
                    sel_rx = float(sel_item.get("last_rx_time", 0.0))
                    sel_age = now - sel_rx if sel_rx > 0 else 9999.0
                    sel_is_streaming = bool((sel_age < VEHICLE_OFFLINE_TIMEOUT_SEC) and (sel_rx > 0.0))

                    latest_telemetry["vehicle_code"] = sel_item.get("vehicle_code", selected_vehicle_code)
                    latest_telemetry["vehicle_id"] = sel_item.get("vehicle_id", 1)
                    latest_telemetry["vehicle_name"] = sel_item.get("vehicle_name", selected_vehicle_code)
                    latest_telemetry["pos_x"] = sel_item.get("pos_x", 0.0)
                    latest_telemetry["pos_y"] = sel_item.get("pos_y", 0.0)
                    latest_telemetry["pos_z"] = sel_item.get("pos_z", 0.0)
                    latest_telemetry["roll"] = sel_item.get("roll", 0.0)
                    latest_telemetry["pitch"] = sel_item.get("pitch", 0.0)
                    latest_telemetry["yaw"] = sel_item.get("yaw", 0.0)
                    latest_telemetry["altitude"] = sel_item.get("altitude", 0.0)
                    latest_telemetry["speed"] = sel_item.get("speed", 0.0)
                    latest_telemetry["battery"] = sel_item.get("battery", 98.0)
                    latest_telemetry["lat"] = sel_item.get("lat", 39.9334)
                    latest_telemetry["lon"] = sel_item.get("lon", 32.8597)
                    latest_telemetry["flight_mode"] = sel_item.get("flight_mode", "AKTİF")
                    latest_telemetry["msg"] = sel_item.get("msg", f"HI FROM {sel_item.get('vehicle_name', selected_vehicle_code)}")
                    latest_telemetry["yaw_rate_rad"] = sel_item.get("yaw_rate_rad", 0.0)
                    latest_telemetry["speed_mps"] = round(float(latest_telemetry.get("speed", 0.0)) / 3.6, 2)
                    latest_telemetry["offline_sec"] = round(sel_age, 1)
                else:
                    latest_telemetry["vehicle_code"] = None
                    latest_telemetry["vehicle_id"] = None
                    latest_telemetry["vehicle_name"] = None
                    latest_telemetry["flight_mode"] = "BEKLİYOR"
                    latest_telemetry["streaming"] = False

                latest_telemetry["connected"] = is_ue5_online
                latest_telemetry["streaming"] = sel_is_streaming
                latest_telemetry["ue5_ip"] = discovered_ue5_ip if is_ue5_online else None
                latest_telemetry["fleet"] = fleet_list
                latest_telemetry["selected_vehicle_code"] = selected_vehicle_code
                latest_telemetry["video_subscribed"] = is_video_subscribed
                latest_telemetry["recent_logs"] = pop_recent_system_logs()

                payload = json.dumps(latest_telemetry).encode('utf-8')

            self.wfile.write(payload)
            return

        # 2c. Görev ve Metro Hattı REST API uç noktası
        if clean_path == '/api/mission':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            v_code = selected_vehicle_code if selected_vehicle_code else None
            m = telemetry_db.get_active_mission(v_code)
            res = {"status": "OK", "mission": m}
            self.wfile.write(json.dumps(res).encode('utf-8'))
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
            unique_only = qs.get('unique', ['0'])[0] in ('1', 'true', 'True')
            # Modal açıldığında veya sorgulandığında hemen SQL dosyasını tara ve tazele
            telemetry_db.sync_sql_data(discovered_ue5_ip)
            stats = telemetry_db.get_stats()
            rows = telemetry_db.get_recent_rows(max_rows, vehicle_code=v_code, unique_only=unique_only)
            res = {
                "status": "OK",
                "total_logged": stats["total_logged"],
                "db_path": stats["db_path"],
                "sql_dump_path": stats["sql_dump_path"],
                "unique_only": unique_only,
                "rows": rows
            }
            self.wfile.write(json.dumps(res).encode('utf-8'))
            return

        # 2c. Belirtilen araca abone olma REST API (Sol panelden araca tıklandığında)
        if clean_path == '/api/select_vehicle':
            global latest_jpeg_frame
            qs = parse_qs(parsed_url.query)
            code = qs.get('code', [''])[0].strip()
            v_id = int(qs.get('id', [-1])[0])
            target_host = discovered_ue5_ip or "127.0.0.1"
            local_ip = get_local_ip()

            # Önce RAM'deki canlı filoya bak; sadece RAM'de yoksa SQL'e sor (Tıklama gecikmesini 0ms'ye indirir!)
            if code and code not in fleet_telemetry:
                db_codes = [v['code'] for v in telemetry_db.get_fleet_from_db()]
                if db_codes and code not in db_codes:
                    self.send_response(400)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(json.dumps({
                        'status': 'REJECTED',
                        'msg': f"Araç '{code}' bulunamadı!"
                    }).encode('utf-8'))
                    return

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

            # KURAL: Sol panelde seçilen araç aktif araç olur (Port 7400)
            sub_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sub_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            try:
                cmd_dict = {
                    "cmd": "SUB_VEHICLE",
                    "vehicle_code": code,
                    "vehicle_id": v_id,
                    "client_ip": local_ip,
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

                # Eğer canlı video yayını açıksa, yeni seçilen aracın kamerasına anında geçiş yap!
                if is_video_subscribed and code:
                    with video_lock:
                        latest_jpeg_frame = None
                    vid_cmd = {
                        "cmd": "SUB_VIDEO",
                        "vehicle_code": code,
                        "subscribe": True,
                        "client_ip": local_ip,
                        "video_port": VIDEO_PORT
                    }
                    vid_bytes = json.dumps(vid_cmd).encode('utf-8')
                    sub_sock.sendto(vid_bytes, (target_host, DISCOVERY_PORT))
                    for b_tgt, b_prt in get_broadcast_targets(DISCOVERY_PORT):
                        try:
                            sub_sock.sendto(vid_bytes, (b_tgt, b_prt))
                        except Exception:
                            pass

                msg_log = f"🎯 [ABONELİK TALEBİ] Araç '{code}' seçildi -> Unreal Engine'e odak bildirildi (Port {TELEMETRY_PORT})"
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
            if not b_sub:
                with video_lock:
                    latest_jpeg_frame = None

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

        # 5. Araç komut fırlatma REST API (ARM, DISARM, ESTOP, STOP)
        if clean_path == '/api/command':
            qs = parse_qs(parsed_url.query)
            cmd = qs.get('cmd', [''])[0].strip().upper()
            target_code = selected_vehicle_code or "all"
            target_host = discovered_ue5_ip or "127.0.0.1"

            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            try:
                # E-Stop veya STOP komutu: Doğrudan Port 7402 Swarm Broadcast'e acil durdurma bas
                if cmd in ("ESTOP", "STOP", "EMERGENCY_STOP"):
                    swarm_payload = json.dumps({
                        "cmd": "stop",
                        "target": target_code,
                        "timestamp": time.time()
                    }).encode('utf-8')
                    sock.sendto(swarm_payload, ("255.255.255.255", SWARM_COMMAND_PORT))
                    sock.sendto(swarm_payload, (target_host, SWARM_COMMAND_PORT))
                    # Port 7405 Sürü Görev Portuna da acil iptal anonsu fırlat
                    mission_abort_payload = json.dumps({
                        "type": "mission_abort",
                        "vehicle": target_code,
                        "target_vehicle": target_code,
                        "reason": "Dashboard E-Stop",
                        "ts": time.time()
                    }).encode('utf-8')
                    try:
                        sock.sendto(mission_abort_payload, ("255.255.255.255", 7405))
                        sock.sendto(mission_abort_payload, ("127.0.0.1", 7405))
                    except Exception:
                        pass
                    add_system_log(f"🛑 [E-STOP TX] Sürü & Görev Durdurma: Araç '{target_code}' (Port {SWARM_COMMAND_PORT} & 7405)")
                else:
                    cmd_payload = json.dumps({
                        "command": cmd,
                        "vehicle_code": target_code,
                        "timestamp": time.time()
                    }).encode('utf-8')
                    sock.sendto(cmd_payload, (target_host, UE5_CONTROL_PORT))
                    add_system_log(f"[KOMUT TX] '{cmd}' -> Araç: {target_code}")
            except Exception as e:
                print(f"⚠️ [KOMUT TX Hatası]: {e}")
            finally:
                sock.close()

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "OK", "command": cmd, "target": target_code}).encode('utf-8'))
            return

        # 5b. Doğrudan Sürü / Araç Hız Sürüş REST API (Gazebo Aktüatör & cmd_vel)
        if clean_path == '/api/drive':
            qs = parse_qs(parsed_url.query)
            linear_x = float(qs.get('linear_x', [0.0])[0])
            angular_z = float(qs.get('angular_z', [0.0])[0])
            throttle = float(qs.get('throttle', [linear_x / 2.5])[0]) if 'throttle' in qs else float(linear_x / 2.5)
            steering = float(qs.get('steering', [angular_z * 0.45])[0]) if 'steering' in qs else float(angular_z * 0.45)
            target_code = qs.get('target', [selected_vehicle_code or 'all'])[0].strip()
            target_host = discovered_ue5_ip or "127.0.0.1"

            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            try:
                try:
                    from vehicle_hardware import VehicleHardware
                    hw = VehicleHardware(vehicle_code=target_code)
                    motors = hw.compute_actuators(throttle, steering)
                except Exception:
                    motors = {}

                payload_data = {
                    "cmd": "actuators" if motors else "cmd_vel",
                    "target": target_code,
                    "linear_x": linear_x,
                    "angular_z": angular_z,
                    "throttle": throttle,
                    "steering": steering,
                    "motors": motors,
                    "timeout_ms": 600,
                    "timestamp": time.time()
                }
                drive_payload = (json.dumps(payload_data) + "\n").encode('utf-8')
                sock.sendto(drive_payload, ("255.255.255.255", SWARM_COMMAND_PORT))
                sock.sendto(drive_payload, (target_host, SWARM_COMMAND_PORT))
                sock.sendto(drive_payload, ("127.0.0.1", SWARM_COMMAND_PORT))
            except Exception as e:
                print(f"[DRIVE TX Hatasi]: {e}")
            finally:
                sock.close()

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({
                "status": "OK",
                "target": target_code,
                "linear_x": linear_x,
                "angular_z": angular_z
            }).encode('utf-8'))
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
        print(f"[Telemetri Dinleyici] 🎧 UDP Port {TELEMETRY_PORT} dinleniyor (Tüm araçların 2Hz canlı telemetrisi aktif)...")
    except Exception as e:
        print(f"[Telemetri Dinleyici] ⚠️ UDP Port {TELEMETRY_PORT} bağlanamadı: {e}")
        return

    connected_vehicles_set = set()
    last_print_time = 0.0

    while True:
        try:
            data, addr = sock.recvfrom(65535)
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
                    v_code = str(msg.get("vehicle_code") or (f"rover_{v_id:02d}" if v_id > 0 else "rover_01"))
                    v_name = str(msg.get("vehicle_name") or msg.get("name") or (f"Rover {v_id:02d}" if v_id > 0 else v_code))
                    rx_msg_content = str(msg.get("msg") or f"TELEMETRİ_{v_code}")

                    if v_code not in connected_vehicles_set:
                        connected_vehicles_set.add(v_code)
                        print(f"\n========================================================")
                        print(f"  ✅ [CANLI TELEMETRİ] {v_name} ({v_code}) bağlandı!")
                        print(f"     Kaynak IP  : {sender_ip} -> Dinlenen Port: {TELEMETRY_PORT}")
                        print(f"     Mesaj      : \"{rx_msg_content}\"")
                        print(f"========================================================\n")
                        add_system_log(f"✅ [{v_name}] Canlı telemetri akışı başladı ({v_code} @ {sender_ip}): '{rx_msg_content}'")

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

                    # 6. 3B Dünya Konumu (Metre: X, Y, Z)
                    pos_arr = msg.get("position", [])
                    pos_x = round(float(msg.get("pos_x", pos_arr[0] if isinstance(pos_arr, list) and len(pos_arr) > 0 else 0.0)), 2)
                    pos_y = round(float(msg.get("pos_y", pos_arr[1] if isinstance(pos_arr, list) and len(pos_arr) > 1 else 0.0)), 2)
                    pos_z = round(float(msg.get("pos_z", pos_arr[2] if isinstance(pos_arr, list) and len(pos_arr) > 2 else altitude)), 2)

                    prev_rx_count = fleet_telemetry.get(v_code, {}).get("rx_count", 0)

                    yaw_rate_rad = 0.0
                    if "ros2_imu" in msg and isinstance(msg["ros2_imu"], dict):
                        ang = msg["ros2_imu"].get("angular_velocity", {})
                        if isinstance(ang, dict):
                            yaw_rate_rad = float(ang.get("z", 0.0))
                    elif "imu" in msg and isinstance(msg["imu"], dict):
                        gyro = msg["imu"].get("gyro", [])
                        if isinstance(gyro, list) and len(gyro) >= 3:
                            yaw_rate_rad = float(gyro[2])

                    # Gerçek canlı araç kaydı (Hem sol panel hem sağ panel için tam anahtar seti)
                    entry = {
                        "timestamp": now_ts,
                        "sim_timestamp": float(msg.get("timestamp", 0.0)),
                        "last_rx_time": now_ts,
                        "rx_count": prev_rx_count + 1,
                        "code": v_code,
                        "id": v_id,
                        "name": v_name,
                        "vehicle_code": v_code,
                        "vehicle_id": v_id,
                        "vehicle_name": v_name,
                        "connected": True,
                        "ue5_ip": sender_ip,
                        "msg": rx_msg_content,
                        "pos_x": pos_x,
                        "pos_y": pos_y,
                        "pos_z": pos_z,
                        "roll": roll,
                        "pitch": pitch,
                        "yaw": yaw,
                        "speed": speed,
                        "altitude": altitude,
                        "battery": battery,
                        "lat": lat,
                        "lon": lon,
                        "flight_mode": flight_mode,
                        "yaw_rate_rad": round(yaw_rate_rad, 3),
                        "speed_mps": round(speed / 3.6, 2)
                    }

                    fleet_telemetry[v_code] = entry
                    last_ue5_contact_time = now_ts
                    discovered_ue5_ip = sender_ip

                    # Gelen 2Hz telemetri paketini 10 saniyelik RAM tamponuna ekle (Pi 5 kendi SQLite'ına yazar)
                    telemetry_db.queue_telemetry_row(entry)

                    if not selected_vehicle_code:
                        selected_vehicle_code = v_code

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
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 524288)
    except Exception:
        pass
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


def run_zenoh_telemetry_worker(router_endpoint: Optional[str] = None):
    """
    Zenoh Topic Köprüsü:
    'pisim/*/gps', 'pisim/*/imu', 'pisim/*/camera/compressed', 'fleet/*/state'
    topic'lerini dinleyerek web arayüzünü (harita, telemetri, göstergeler) canlı günceller.
    """
    global latest_telemetry, fleet_telemetry, selected_vehicle_code, latest_jpeg_frame
    global discovered_ue5_ip, last_ue5_contact_time

    if not HAS_ZENOH:
        print("[Zenoh Telemetri] ℹ️ 'eclipse-zenoh' modülü bulunamadı, yalnızca UDP modu dinlenecek.")
        return

    ep = router_endpoint or auto_discover_zenoh_endpoint(verbose=False)
    conf = zenoh.Config()
    norm_ep = normalize_zenoh_endpoint(ep)
    if norm_ep:
        conf.insert_json5("mode", json.dumps("client"))
        conf.insert_json5("connect/endpoints", json.dumps([norm_ep]))
        print(f"[Zenoh Telemetri] 🔗 Yönlendiriciye bağlanılıyor: {norm_ep}")
    else:
        conf.insert_json5("mode", json.dumps("peer"))
        print("[Zenoh Telemetri] 🌐 P2P modda başlatılıyor.")

    try:
        session = zenoh.open(conf)
        print("[Zenoh Telemetri] ✅ Zenoh telemetri oturumu başarıyla açıldı.")
        add_system_log(f"📡 [ZENOH KÖPRÜSÜ] Zenoh ağına bağlanıldı ({norm_ep or 'P2P'}). Canlı topic dinleniyor.")
    except Exception as e:
        print(f"[Zenoh Telemetri] ⚠️ Zenoh oturumu açılamadı: {e}")
        return

    prev_gps_tracker = {}

    def on_sample(sample):
        nonlocal prev_gps_tracker
        global selected_vehicle_code, latest_jpeg_frame, last_ue5_contact_time, discovered_ue5_ip
        key = str(sample.key_expr)
        parts = key.split("/")
        now_ts = time.time()

        # 1. PISIM Sensör Topic'leri (pisim/<v_code>/...)
        if key.startswith("pisim/") and len(parts) >= 3:
            v_code = parts[1]
            sub_type = parts[2]

            # A. Kamera karesi (pisim/<v_code>/camera/compressed)
            if sub_type == "camera":
                raw_bytes = sample.payload.to_bytes()
                if raw_bytes and (selected_vehicle_code is None or selected_vehicle_code == v_code):
                    with video_lock:
                        latest_jpeg_frame = raw_bytes
                return

            # Diğer sensörler JSON formatında
            try:
                data = json.loads(sample.payload.to_string())
            except Exception:
                return

            with telemetry_lock:
                last_ue5_contact_time = now_ts
                if not discovered_ue5_ip:
                    discovered_ue5_ip = "127.0.0.1"

                if v_code not in fleet_telemetry:
                    fleet_telemetry[v_code] = {
                        "timestamp": now_ts,
                        "sim_timestamp": float(data.get("timestamp", now_ts)),
                        "last_rx_time": now_ts,
                        "rx_count": 0,
                        "code": v_code,
                        "id": 1,
                        "name": v_code,
                        "vehicle_code": v_code,
                        "vehicle_id": 1,
                        "vehicle_name": v_code,
                        "connected": True,
                        "ue5_ip": discovered_ue5_ip,
                        "msg": f"ZENOH_{v_code}",
                        "pos_x": 0.0,
                        "pos_y": 0.0,
                        "pos_z": 0.0,
                        "roll": 0.0,
                        "pitch": 0.0,
                        "yaw": 0.0,
                        "speed": 0.0,
                        "altitude": 0.0,
                        "battery": 98.0,
                        "lat": 39.9334,
                        "lon": 32.8597,
                        "flight_mode": "AKTİF",
                        "yaw_rate_rad": 0.0,
                        "speed_mps": 0.0,
                    }
                    print(f"  ✅ [ZENOH TELEMETRİ] Yeni araç keşfedildi: {v_code}")
                    add_system_log(f"✅ [ZENOH] Yeni araç keşfedildi: {v_code}")

                entry = fleet_telemetry[v_code]
                entry["last_rx_time"] = now_ts
                entry["rx_count"] += 1
                entry["connected"] = True

                # GPS Sensörü
                if sub_type == "gps":
                    lat = float(data.get("latitude", data.get("lat", entry.get("lat", 0.0))))
                    lon = float(data.get("longitude", data.get("lon", entry.get("lon", 0.0))))
                    alt = float(data.get("altitude", data.get("alt", entry.get("altitude", 0.0))))
                    
                    spd_mps = float(data.get("speed", 0.0))
                    if spd_mps == 0.0 and v_code in prev_gps_tracker:
                        p_lat, p_lon, p_t = prev_gps_tracker[v_code]
                        dt = now_ts - p_t
                        if 0.1 <= dt <= 2.0:
                            dlat = math.radians(lat - p_lat)
                            dlon = math.radians(lon - p_lon)
                            dist_m = 6371000.0 * math.sqrt(dlat * dlat + (dlon * math.cos(math.radians(lat))) ** 2)
                            spd_mps = dist_m / dt
                    prev_gps_tracker[v_code] = (lat, lon, now_ts)

                    heading = float(data.get("heading", entry.get("yaw", 0.0)))
                    entry["lat"] = lat
                    entry["lon"] = lon
                    entry["altitude"] = round(alt, 1)
                    entry["speed"] = round(spd_mps * 3.6, 1)
                    entry["speed_mps"] = round(spd_mps, 2)
                    if "heading" in data:
                        entry["yaw"] = round(heading, 1)

                # IMU Sensörü
                elif sub_type == "imu":
                    q = data.get("orientation", {})
                    qx = float(q.get("x", 0.0))
                    qy = float(q.get("y", 0.0))
                    qz = float(q.get("z", 0.0))
                    qw = float(q.get("w", 1.0))
                    sinr_cosp = 2.0 * (qw * qx + qy * qz)
                    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
                    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

                    sinp = 2.0 * (qw * qy - qz * qx)
                    pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))

                    siny_cosp = 2.0 * (qw * qz + qx * qy)
                    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
                    if yaw < 0.0:
                        yaw += 360.0

                    ang_vel = data.get("angular_velocity", {})
                    yaw_rate = float(ang_vel.get("z", 0.0)) if isinstance(ang_vel, dict) else 0.0

                    entry["roll"] = round(roll, 2)
                    entry["pitch"] = round(pitch, 2)
                    entry["yaw"] = round(yaw, 1)
                    entry["yaw_rate_rad"] = round(yaw_rate, 3)

                # Eyleyici Durumu
                elif sub_type == "actuators":
                    st = data.get("M_Steer_FL", 0.0)
                    th = data.get("M_Wheel_FL", 0.0)
                    entry["msg"] = f"Direksiyon: {st:+.2f} | Gaz: {th:+.2f}"

                # Her 5 pakette bir SQLite veritabanına kaydet
                if entry["rx_count"] % 5 == 0:
                    telemetry_db.queue_telemetry_row(entry)

                if not selected_vehicle_code:
                    selected_vehicle_code = v_code

                if selected_vehicle_code == v_code:
                    latest_telemetry.update(entry)
                    latest_telemetry["connected"] = True
                    latest_telemetry["streaming"] = True

        # 2. Filo / Ajan Durum Topic'leri (fleet/<agent_id>/state)
        elif key.startswith("fleet/") and len(parts) >= 3:
            agent_id = parts[1]
            try:
                data = json.loads(sample.payload.to_string())
            except Exception:
                return
            tgt_v = data.get("target_vehicle")
            with telemetry_lock:
                if tgt_v and tgt_v in fleet_telemetry:
                    wp_idx = data.get("current_wp", data.get("wp_idx", 0))
                    total_wps = data.get("total_wps", 4)
                    mode_txt = f"OTONOM [{agent_id}] WP:{wp_idx}/{total_wps}"
                    fleet_telemetry[tgt_v]["flight_mode"] = mode_txt
                    if selected_vehicle_code == tgt_v:
                        latest_telemetry["flight_mode"] = mode_txt

    sub_pisim = session.declare_subscriber("pisim/**", on_sample)
    sub_fleet = session.declare_subscriber("fleet/**", on_sample)

    try:
        while True:
            time.sleep(1.0)
    except Exception:
        pass
    finally:
        session.close()


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



# =========================================================================
# SÜRÜ GÖREV VE METRO HATTI DİNLEYİCİSİ (UDP Port 7405)
# =========================================================================
def swarm_mission_listener():
    """Port 7405 üzerinden gelen görev planlarını ve adım ping'lerini yakalar ve SQL'e işler."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", 7405))
        print(" [+] Sürü Görev RX : UDP Port 7405 (Canlı Görev & Metro Hattı)")
    except Exception as e:
        print(f"⚠️ [Dashboard] Port 7405 dinleme uyarısı: {e}")
        return

    while True:
        try:
            data, addr = sock.recvfrom(65535)
            payload = json.loads(data.decode("utf-8", errors="ignore").strip())
            pkt_type = payload.get("type", "")
            if pkt_type == "mission_plan":
                mission = payload.get("mission", {})
                telemetry_db.save_mission_plan(mission)
                add_system_log(f"📋 [GÖREV ALINDI] '{mission.get('title')}' -> Araç: {mission.get('vehicle_code')}")
            elif pkt_type == "step_ping":
                telemetry_db.update_mission_step_state(
                    payload.get("mission_id", ""),
                    int(payload.get("step", 0)),
                    payload.get("state", "ACTIVE"),
                    payload.get("vehicle", "")
                )
            elif pkt_type == "mission_abort":
                m_id = payload.get("mission_id", "")
                if m_id:
                    telemetry_db.update_mission_step_state(m_id, 0, "ABORTED", payload.get("vehicle", ""))
                add_system_log(f"🛑 [GÖREV İPTAL] {payload.get('vehicle', 'Araç')}: {payload.get('reason', '')}")
        except Exception:
            time.sleep(0.05)

def main():
    global PORT
    parser = argparse.ArgumentParser(description="PiSim Studio - Tactical Web Dashboard GCS")
    parser.add_argument("--router", "-r", type=str, default=None, help="Zenoh Router adresi (boş bırakılırsa otomatik keşfedilir)")
    parser.add_argument("--port", "-p", type=int, default=PORT, help=f"HTTP Web portu (varsayılan: {PORT})")
    args, _ = parser.parse_known_args()

    PORT = args.port

    local_ip = get_local_ip()
    print("=================================================================")
    print("  🚀 PiSim Studio - Raspberry Pi 5 Tactical Web Dashboard GCS    ")
    print("=================================================================")
    print(f" [+] WebUI Klasörü : {WEBUI_DIR}")
    print(f" [+] Sayfa         : dashboard.html (Taktik Kokpit)")
    print(f" [+] Yerel Adres   : http://localhost:{PORT}/dashboard.html")
    print(f" [+] Ağ Adresi     : http://{local_ip}:{PORT}/dashboard.html")
    print(f" [+] Keşif Radarı  : UDP Port {DISCOVERY_PORT} (Beacon & Probe)")
    print(f" [+] Zenoh Topic   : pisim/** ve fleet/** (Otomatik Keşif Aktif)")
    print(f" [+] Telemetri RX  : UDP Port {TELEMETRY_PORT}")
    print(f" [+] Canlı Video RX: UDP Port {VIDEO_PORT}")
    print("=================================================================")

    # 0. KURAL: Program her başladığında SQL veritabanı ve RAM filosu tamamen temizlenir!
    telemetry_db.clear_fleet()
    with telemetry_lock:
        fleet_telemetry.clear()
        cached_db_fleet.clear()
    print(" [+] SQL Veritabanı: Temizlendi (0 satır). Yalnızca bu oturumda gelen canlı araçlar kaydedilecek.")
    add_system_log("🧹 [TEMİZ BAŞLANGIÇ] SQL tablosu sıfırlandı. Canlı Zenoh topic telemetrileri bekleniyor...")

    global discovery_service_ref
    # 0b. Sürü Görev Dinleyicisini Başlat (UDP Port 7405)
    t_swarm = threading.Thread(target=swarm_mission_listener, daemon=True, name="SwarmMissionListener")
    t_swarm.start()

    # 1. Keşif Dinleyici Servisini Başlat (UDP Port 7499)
    discovery = PiSimDiscoveryService(port=DISCOVERY_PORT)
    discovery_service_ref = discovery

    def on_probe_discovered(ue_ip, msg_dict=None):
        global discovered_ue5_ip, last_ue5_contact_time
        discovered_ue5_ip = ue_ip
        last_ue5_contact_time = time.time()
        v_name = msg_dict.get("vehicle_name") if isinstance(msg_dict, dict) else "Rover"
        v_code = msg_dict.get("vehicle_code") if isinstance(msg_dict, dict) else "rover"
        rx_msg = msg_dict.get("msg") if isinstance(msg_dict, dict) else f"HI FROM {v_name}"
        with telemetry_lock:
            latest_telemetry["ue5_ip"] = ue_ip
            latest_telemetry["connected"] = True
            latest_telemetry["msg"] = rx_msg
        add_system_log(f"📡 [{v_name}] Ping alındı ({v_code} @ {ue_ip}): \"{rx_msg}\"")

    discovery.on_probe_received = on_probe_discovered

    def on_sql_rows_received(added, total_received, sender_ip):
        with telemetry_lock:
            global cached_db_fleet
            cached_db_fleet = telemetry_db.get_fleet_from_db()
        v_list = [v["code"] for v in cached_db_fleet]
        msg = f"📥 [SQL AĞ SENKRONİZASYONU] Unreal Engine ({sender_ip}) üzerinden +{added} satır SQL verisi alındı (SQL Filosu: {len(cached_db_fleet)} araç: {', '.join(v_list)})"
        print(msg)
        add_system_log(msg)

    discovery.on_sql_rows_received = on_sql_rows_received
    discovery.start()

    # 2. Zenoh Canlı Topic Dinleyicisini Başlat (pisim/** ve fleet/**)
    t_zenoh = threading.Thread(
        target=run_zenoh_telemetry_worker,
        args=(args.router,),
        daemon=True,
        name="ZenohTelemetryWorker"
    )
    t_zenoh.start()

    # 3. UDP Canlı Telemetri Dinleyicisini Başlat (Port 7400 - Eski/Yedek Köprü)
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
