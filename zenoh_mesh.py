#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
zenoh_mesh.py - PiSim Eclipse Zenoh & ROS 2 Binary CDR Köprüsü (Unreal <-> Zenoh Mesh)
======================================================================================
Mimari Akış:
  1. Unreal Engine -> Zenoh Bridge (Tek Port UDP 7500):
     - Unreal içindeki her sensör modülü (`imu`, `gps`, `camera/h264`, `ack`) kendi
       aracının ismiyle (`/<vehicle_code>/<sensor>`) verisini doğrudan Port 7500'e atar.
     - Bu köprü (`zenoh_mesh.py`), gelen paketlerin isimlerini okur ve gelir gelmez
       `ros2_cdr.py` ile **ROS 2 Binary CDR (0x00 0x01 0x00 0x00)** ikili formatına
       çevirip Zenoh ağına (`pisim/<vehicle_code>/<sensor>`) basar.
     - Kamera modülü normalde (Standby) sadece hafif **Mavi Ekran (Blue Screen)** (~160 byte)
       yayınlar. Karşı taraftan `ACK = 3` tetiklemesi geldiğinde tam çözünürlüklü canlı
       **H.264** akışına geçer; `ACK = 4` geldiğinde veya 4.0 saniye boyunca `ACK = 3`
       gelmediğinde otomatik olarak tekrar Mavi Ekran (Standby) moduna döner.

  2. Pi 5 -> Zenoh Ağı -> Zenoh Bridge -> Unreal Engine (Tek Port UDP 7402):
     - Pi 5 tarafı kontrol komutlarını (`channels`: Kanal 1 = Sürüş Tekerlekleri,
       Kanal 2 = Direksiyon Servoları) ve `ack` mesajlarını **Binary CDR** olarak Zenoh'a basar.
     - Köprü tüm bu topic'lere abone olur, Binary CDR verisini çözer ve Unreal'ın
       tekil dinleme portuna (**UDP 7402**) fırlatır.
     - Unreal içindeki her aracın her modülü bu portu dinler; sadece kendi aracına
       (`VehicleCode`) ve kendi kanal indeksine (`ArduPilotChannel`) ait komutu/ACK'i
       ayıklayıp çalıştırır, gerisini çöpe atar.

  3. Canlı Binary CDR + JSON Inspector Özelliği:
     - `--inspect <topic_filtresi>` (örn: `--inspect all`, `--inspect imu`, `--inspect RV01`)
       parametresiyle veya çalışırken klavyeden `i` / topic adı girerek herhangi bir topic'in
       hem **Binary CDR (Hex + Byte)** hem de **JSON** halini canlı olarak ekranda gösterir.
"""

import argparse
import base64
import json
import logging
import socket
import sys
import threading
import time
from typing import Any, Dict, List, Optional

import ros2_cdr

# Windows terminal Türkçe kod sayfası (cp1254/cp437) desteği
if sys.stdout and hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

logging.basicConfig(level=logging.INFO, format="[%(asctime)s][ZenohBridge] %(message)s")
logger = logging.getLogger("ZenohMesh")

try:
    import zenoh
    HAS_ZENOH = True
except ImportError:
    HAS_ZENOH = False
    logger.warning("eclipse-zenoh modülü bulunamadı. Lütfen 'pip install eclipse-zenoh' çalıştırın.")

UNREAL_SENSOR_RX_PORT = 7500   # Unreal'daki tüm sensör modüllerinin veri bastığı tekil UDP portu
UNREAL_SINGLE_TX_PORT = 7402   # Zenoh'tan gelen tüm komut/ACK verilerinin Unreal'a basıldığı tekil UDP portu
UNREAL_BASE_TX_PORT = 7410     # Geriye dönük TopicBridge portu (7410 + VehicleId)
CAMERA_TRIGGER_TIMEOUT_SEC = 4.0  # 4.0 sn boyunca ACK=3 gelmezse kamera Mavi Ekran'a (Standby) döner


class ZenohMeshBridge:
    """
    Unreal Engine (Port 7500 RX / Port 7402 TX) ile Zenoh P2P Ağı arasında
    Binary CDR çevirici, Kamera Standby/H.264 tetikleyici, ACK yöneticisi ve Canlı Inspector.
    """

    def __init__(
        self,
        mode: str = "peer",
        router_endpoint: Optional[str] = None,
        ue5_ip: str = "127.0.0.1",
        listen_endpoint: Optional[str] = "tcp/0.0.0.0:7447",
        inspect_filter: Optional[str] = None,
    ):
        self.mode = mode
        self.router_endpoint = router_endpoint
        self.listen_endpoint = listen_endpoint
        self.ue5_ip = ue5_ip
        self.inspect_filter = inspect_filter.strip() if inspect_filter else None
        self.running = False
        self.session: Optional[Any] = None

        # Dağıtık Araç Belleği ve Topic/ACK Durum Tablosu
        self.fleet_state: Dict[str, Dict[str, Any]] = {}
        self.fleet_lock = threading.Lock()

        # Kamera H.264 Canlı Tetikleme Durumu: {vehicle_code: last_ack3_timestamp}
        self.camera_live_triggers: Dict[str, float] = {}
        self.camera_last_standby_tx: Dict[str, float] = {}

        # Araç Bazlı ACK Tablosu: {vehicle_code: {topic_name: status_code}}
        # 0=Yok, 1=Bozuk Şema, 2=Sağlıklı, 3=Trigger(Kamera Aç), 4=Standby(Mavi Ekran)
        self.vehicle_ack_table: Dict[str, Dict[str, int]] = {}

        # Son İncelenen Topic Paketleri (Binary CDR + JSON Önizleme Belleği)
        self.topic_snapshots: Dict[str, Dict[str, Any]] = {}
        self.last_inspect_print_time: Dict[str, float] = {}

        # UDP Soketleri (Unreal Engine ile haberleşme)
        self.udp_rx_sock: Optional[socket.socket] = None
        self.udp_tx_sock: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_tx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        self.zenoh_subs: List[Any] = []

    def start(self):
        """Zenoh oturumunu açar ve köprü iş parçacıklarını başlatır."""
        self.running = True

        if HAS_ZENOH:
            try:
                conf = zenoh.Config()
                if self.listen_endpoint:
                    conf.insert_json5("listen/endpoints", json.dumps([self.listen_endpoint]))
                    logger.info(f"📡 Zenoh Dinleme Soketi Açıldı: {self.listen_endpoint}")

                if self.mode == "client" and self.router_endpoint:
                    conf.insert_json5("mode", json.dumps("client"))
                    conf.insert_json5("connect/endpoints", json.dumps([self.router_endpoint]))
                    logger.info(f"Zenoh İstemci Modu: Yönlendiriciye bağlanılıyor -> {self.router_endpoint}")
                else:
                    conf.insert_json5("mode", json.dumps("router"))
                    logger.info("📡 Zenoh Dağıtıcı Router Modu Aktif (Binary CDR + P2P Mesh).")

                self.session = zenoh.open(conf)
                logger.info("✅ Zenoh Oturumu Başarıyla Açıldı.")

                # Pi 5 veya Docker'dan gelen Binary CDR Topic'lerine Abone Ol
                sub_channels = self.session.declare_subscriber("pisim/*/channels", self._on_zenoh_channels)
                sub_ack = self.session.declare_subscriber("pisim/*/ack", self._on_zenoh_ack)
                sub_actuators = self.session.declare_subscriber("pisim/*/actuators", self._on_zenoh_actuators)
                sub_cmdvel = self.session.declare_subscriber("pisim/*/cmd_vel", self._on_zenoh_cmd_vel)
                sub_ping = self.session.declare_subscriber("pisim/*/ping", self._on_zenoh_ping)
                sub_fleet = self.session.declare_subscriber("fleet/*/state", self._on_zenoh_fleet_state)
                self.zenoh_subs.extend([sub_channels, sub_ack, sub_actuators, sub_cmdvel, sub_ping, sub_fleet])

            except Exception as e:
                logger.error(f"Zenoh başlatma hatası: {e}. Yerel köprü modunda devam ediliyor.")
                self.session = None

        # Unreal Engine Dinleyici İş Parçacığını Başlat (Port 7500)
        rx_thread = threading.Thread(target=self._unreal_rx_loop, daemon=True)
        rx_thread.start()

        # Kamera Standby Mavi Ekran & ACK Periyodik Denetim İş Parçacığı
        ack_thread = threading.Thread(target=self._ack_and_standby_watchdog_loop, daemon=True)
        ack_thread.start()

        # Otomatik Keşif Beacon İş Parçacığını Başlat (Zero-Config)
        beacon_thread = threading.Thread(target=self._beacon_loop, daemon=True)
        beacon_thread.start()
        logger.info(
            f"🚀 PiSim Zenoh Binary CDR Köprüsü Aktif! "
            f"(Unreal Sensör RX: {UNREAL_SENSOR_RX_PORT} ➔ Zenoh Binary CDR | Zenoh ➔ Unreal Tekil Komut TX: {UNREAL_SINGLE_TX_PORT})"
        )

    def _beacon_loop(self):
        """Yerel ağa (LAN) 'Ben buradayım' diyen UDP broadcast basar (Zero-Config)."""
        b_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        b_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        while self.running:
            try:
                s_ip = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s_ip.connect(("8.8.8.8", 80))
                my_ip = s_ip.getsockname()[0]
                s_ip.close()
            except Exception:
                my_ip = "127.0.0.1"

            beacon = json.dumps({"pisim_zenoh": True, "endpoint": f"tcp/{my_ip}:7447"}).encode("utf-8")
            targets = [("255.255.255.255", 7447), ("255.255.255.255", 7448)]
            if my_ip != "127.0.0.1" and "." in my_ip:
                prefix = ".".join(my_ip.split(".")[:3])
                targets.extend([(f"{prefix}.255", 7447), (f"{prefix}.255", 7448)])

            for tgt in targets:
                try:
                    b_sock.sendto(beacon, tgt)
                except Exception:
                    pass
            time.sleep(0.35)
        b_sock.close()

    def stop(self):
        """Köprüyü güvenli bir şekilde durdurur."""
        self.running = False
        if self.session:
            try:
                self.session.close()
            except Exception:
                pass
            self.session = None
        if self.udp_rx_sock:
            try:
                self.udp_rx_sock.close()
            except Exception:
                pass
        logger.info("Zenoh Mesh Köprüsü durduruldu.")

    def _record_and_maybe_inspect(self, zenoh_key: str, cdr_bytes: bytes, direction: str = "TX_TO_ZENOH"):
        """Paketin hem Binary CDR hem de JSON halini kaydeder; filtre eşleşirse ekrana basar."""
        snap = ros2_cdr.inspect_cdr_packet(zenoh_key, cdr_bytes)
        snap["direction"] = direction
        snap["timestamp"] = time.time()
        with self.fleet_lock:
            self.topic_snapshots[zenoh_key] = snap

        if not self.inspect_filter:
            return

        flt = self.inspect_filter.lower()
        if flt not in ("all", "*") and flt not in zenoh_key.lower():
            return

        now = time.time()
        last_t = self.last_inspect_print_time.get(zenoh_key, 0.0)
        if now - last_t < 0.50:  # Konsolu boğmamak için topic başına saniyede 2 kez bas
            return
        self.last_inspect_print_time[zenoh_key] = now

        print("\n" + "─" * 78)
        print(f"🔎 [BRIDGE INSPECTOR | {direction}] Topic: {zenoh_key}")
        print(f"  ├─ 📦 Format & Boyut : {snap['cdr_header']} | {snap['byte_size']} byte | ACK={snap['ack_label']}")
        print(f"  ├─ 🧬 Binary CDR Hex : {snap['hex_preview']}")
        print(f"  └─ 📄 JSON Karşılığı : {json.dumps(snap['json'], ensure_ascii=False)}")
        print("─" * 78)

    def _update_ack_status(self, v_code: str, topic_suffix: str, status_code: int):
        with self.fleet_lock:
            if v_code not in self.vehicle_ack_table:
                self.vehicle_ack_table[v_code] = {
                    f"/{v_code}/imu": 0,
                    f"/{v_code}/gps": 0,
                    f"/{v_code}/camera/h264": 4,  # 4 = Standby Mavi Ekran
                    f"/{v_code}/channels": 0,
                    f"/{v_code}/actuators": 0,
                }
            full_t = topic_suffix if topic_suffix.startswith("/") else f"/{v_code}/{topic_suffix}"
            self.vehicle_ack_table[v_code][full_t] = int(status_code)

    def is_camera_live_triggered(self, v_code: str) -> bool:
        """Bu araç için son 4.0 saniye içinde ACK=3 (Canlı H.264 Aç) tetiklemesi geldi mi?"""
        now = time.time()
        last_trig = self.camera_live_triggers.get(v_code, -100.0)
        if (now - last_trig) <= CAMERA_TRIGGER_TIMEOUT_SEC:
            return True
        return False

    def _ack_and_standby_watchdog_loop(self):
        """
        1) Kamera ACK=3 zaman aşımını (4.0 sn) denetler; süre dolduysa kamerayı tekrar
           Standby Mavi Ekran (ACK=4) moduna çeker ve Unreal'a (Port 7402) bildirir.
        2) Her aracın kendi `/<vehicle_code>/ack` topic'ini Binary CDR olarak Zenoh'a yayınlar.
        """
        while self.running:
            time.sleep(0.50)
            now = time.time()
            with self.fleet_lock:
                active_codes = [
                    vc for vc, st in self.fleet_state.items()
                    if (now - st.get("last_update", 0.0)) < 5.0
                ]

            for v_code in active_codes:
                live_active = self.is_camera_live_triggered(v_code)
                cam_code = ros2_cdr.ACK_TRIGGER_STREAM if live_active else ros2_cdr.ACK_STANDBY_SLEEP
                self._update_ack_status(v_code, "camera/h264", cam_code)

                # Eğer kamera Standby (Mavi Ekran) modundaysa ve Unreal henüz yeni C++ koduyla
                # derlenmediği için standby paketi göndermediyse, köprü 2 Hz Mavi Ekran CDR basar
                if not live_active and self.session:
                    last_sb = self.camera_last_standby_tx.get(v_code, 0.0)
                    if (now - last_sb) >= 0.50:
                        self.camera_last_standby_tx[v_code] = now
                        blue_bytes = ros2_cdr.get_standby_blue_screen_bytes()
                        cdr_cam = ros2_cdr.serialize_camera_cdr(blue_bytes, mode="standby_blue", codec="h264")
                        try:
                            self.session.put(f"pisim/{v_code}/camera/h264", cdr_cam)
                            self.session.put(f"pisim/{v_code}/camera/compressed", blue_bytes)
                            self._record_and_maybe_inspect(f"pisim/{v_code}/camera/h264", cdr_cam, "TX_STANDBY_BLUE")
                        except Exception:
                            pass

                # Araca özel ACK tablosunu Binary CDR olarak Zenoh'a yayınla
                with self.fleet_lock:
                    ack_map = dict(self.vehicle_ack_table.get(v_code, {}))
                if ack_map and self.session:
                    cdr_ack = ros2_cdr.serialize_ack_cdr(v_code, ack_map, source="bridge")
                    try:
                        self.session.put(f"pisim/{v_code}/ack", cdr_ack)
                        self._record_and_maybe_inspect(f"pisim/{v_code}/ack", cdr_ack, "TX_ACK_CDR")
                    except Exception:
                        pass

    def _unreal_rx_loop(self):
        """
        Unreal Engine'den (Port 7500) gelen tüm sensör paketlerini okur,
        anında Binary CDR formatına çevirir ve Zenoh ağına basar.
        """
        try:
            self.udp_rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.udp_rx_sock.bind(("0.0.0.0", UNREAL_SENSOR_RX_PORT))
            self.udp_rx_sock.settimeout(0.5)
        except Exception as e:
            logger.error(f"Unreal UDP RX Soket açılamadı (Port {UNREAL_SENSOR_RX_PORT}): {e}")
            return

        while self.running:
            try:
                data, addr = self.udp_rx_sock.recvfrom(65535)
                if not data:
                    continue

                text = data.decode("utf-8", errors="ignore").strip()
                if not text:
                    continue

                packet = json.loads(text)
                topic = packet.get("topic", "")
                msg = packet.get("msg", {})

                clean_topic = topic.lstrip("/")
                parts = clean_topic.split("/")
                if len(parts) < 2:
                    continue

                v_code = parts[0]
                sub_topic = "/".join(parts[1:])
                zenoh_key = f"pisim/{clean_topic}"

                # Kamera / H.264 Özel İşleme (Standby Mavi Ekran vs Canlı H.264)
                if sub_topic in ("camera/h264", "camera/compressed"):
                    live_ok = self.is_camera_live_triggered(v_code)
                    msg_mode = msg.get("mode", "") if isinstance(msg, dict) else ""

                    if not live_ok and msg_mode != "standby_blue":
                        # ACK=3 tetiklemesi YOK! Ağın boğulmasını önlemek için ağır görüntüyü çöpe at,
                        # sadece 2 Hz hızında hafif Mavi Ekran (Standby Blue Screen) Binary CDR gönder!
                        now = time.time()
                        if (now - self.camera_last_standby_tx.get(v_code, 0.0)) < 0.45:
                            continue
                        self.camera_last_standby_tx[v_code] = now
                        raw_img = ros2_cdr.get_standby_blue_screen_bytes()
                        cdr_payload = ros2_cdr.serialize_camera_cdr(raw_img, mode="standby_blue", codec="h264")
                        self._update_ack_status(v_code, "camera/h264", ros2_cdr.ACK_STANDBY_SLEEP)
                    else:
                        # ACK=3 aktif (veya zaten Unreal'dan standby_blue geldi):
                        b64_str = msg.get("data", "") if isinstance(msg, dict) else ""
                        raw_img = base64.b64decode(b64_str) if b64_str else ros2_cdr.get_standby_blue_screen_bytes()
                        actual_mode = "standby_blue" if (not live_ok or len(raw_img) <= 350) else "live_h264"
                        cdr_payload = ros2_cdr.serialize_camera_cdr(raw_img, mode=actual_mode, codec="h264")
                        self.camera_last_standby_tx[v_code] = time.time()
                        self._update_ack_status(
                            v_code,
                            "camera/h264",
                            ros2_cdr.ACK_TRIGGER_STREAM if actual_mode == "live_h264" else ros2_cdr.ACK_STANDBY_SLEEP,
                        )

                    if self.session:
                        try:
                            self.session.put(f"pisim/{v_code}/camera/h264", cdr_payload)
                            # Web Dashboard MJPEG uyumluluğu için ham/CDR frame'i camera/compressed'e de bas
                            self.session.put(f"pisim/{v_code}/camera/compressed", cdr_payload)
                        except Exception:
                            pass
                    self._record_and_maybe_inspect(f"pisim/{v_code}/camera/h264", cdr_payload, "TX_TO_ZENOH")
                    continue

                # Diğer tüm sensörleri (imu, gps, ack) Binary CDR'a çevir ve Zenoh'a bas
                cdr_bytes = ros2_cdr.serialize_for_topic(zenoh_key, msg)
                if self.session:
                    try:
                        self.session.put(zenoh_key, cdr_bytes)
                    except Exception:
                        pass

                self._update_ack_status(v_code, sub_topic, ros2_cdr.ACK_HEALTHY_OK)
                self._record_and_maybe_inspect(zenoh_key, cdr_bytes, "TX_TO_ZENOH")

                # Dağıtık Bellek Durumunu Güncelle
                with self.fleet_lock:
                    if v_code not in self.fleet_state:
                        self.fleet_state[v_code] = {
                            "code": v_code,
                            "last_update": time.time(),
                            "online": True,
                        }
                    entry = self.fleet_state[v_code]
                    entry["last_update"] = time.time()
                    entry["online"] = True

                    if sub_topic == "imu" and isinstance(msg, dict):
                        entry["imu"] = msg
                        q = msg.get("orientation", {})
                        qx, qy, qz, qw = q.get("x", 0.0), q.get("y", 0.0), q.get("z", 0.0), q.get("w", 1.0)
                        entry["yaw"] = round(57.2958 * 2.0 * (qw * qz + qx * qy), 2)
                    elif sub_topic == "gps" and isinstance(msg, dict):
                        entry["gps"] = msg
                        entry["lat"] = msg.get("latitude", 0.0)
                        entry["lon"] = msg.get("longitude", 0.0)
                        entry["alt"] = msg.get("altitude", 0.0)
                    elif sub_topic == "ack" and isinstance(msg, dict):
                        t_map = msg.get("topics", {})
                        if isinstance(t_map, dict):
                            if v_code not in self.vehicle_ack_table:
                                self.vehicle_ack_table[v_code] = {}
                            for tk, tv in t_map.items():
                                self.vehicle_ack_table[v_code][tk] = int(tv)

            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    logger.debug(f"RX döngüsü hatası: {e}")

    def _extract_vehicle_id(self, vehicle_code: str) -> int:
        """'RV01-8AD131', 'rover_02', 'pi5_rover01' gibi kodlardan aracın tam sayı ID'sini (1, 2, ...) çıkarır."""
        code = str(vehicle_code).strip()
        if "_" in code and not code.upper().startswith("ROVER_"):
            code = code.split("_")[-1]

        code_upper = code.upper()
        if code_upper.startswith("RV") and "-" in code_upper:
            prefix = code_upper.split("-")[0]
            digits = "".join(filter(str.isdigit, prefix))
            if digits:
                return max(1, int(digits))

        if "-" in code_upper:
            code_upper = code_upper.split("-")[0]

        digits = "".join(filter(str.isdigit, code_upper))
        if digits:
            try:
                val = int(digits)
                if val > 64:
                    val = int(digits[-2:])
                return max(1, val)
            except Exception:
                pass
        return 1

    def _send_to_unreal_single_port(self, v_code: str, packet: Dict[str, Any]):
        """
        Gelen komutu/ACK paketini Unreal Engine'in tekil dinleme portuna (UDP 7402)
        ve yedek TopicBridge portuna iletir. Unreal içindeki her modül kendi aracını
        ve kendi kanalını (CH1: Tekerlek, CH2: Steer Servo) buradan ayıklar.
        """
        v_id = self._extract_vehicle_id(v_code)
        target_port = UNREAL_BASE_TX_PORT + v_id
        dedicated_port = 7402 + v_id
        if not (1024 <= target_port <= 65535):
            target_port = UNREAL_BASE_TX_PORT + 1
        if not (1024 <= dedicated_port <= 65535):
            dedicated_port = 7403

        raw = json.dumps(packet).encode("utf-8")
        try:
            # 1. Ana Tekil Port (7402) - Tüm araçlar ve modüller burayı dinler
            self.udp_tx_sock.sendto(raw, (self.ue5_ip, UNREAL_SINGLE_TX_PORT))
            # 2. TopicBridge ve Dedicated Port yedeklemesi
            self.udp_tx_sock.sendto(raw, (self.ue5_ip, target_port))
            self.udp_tx_sock.sendto(raw, (self.ue5_ip, dedicated_port))
        except Exception as ex:
            logger.debug(f"Unreal UDP iletim hatası ({v_code}): {ex}")

    def _on_zenoh_channels(self, sample):
        """
        Pi 5'ten gelen Binary CDR (veya JSON) ArduPilot Kanal mesajlarını çözer:
          - Channel 1: Sürüş Tekerlekleri (İleri/Geri)
          - Channel 2: Direksiyon Servoları (Sağ/Sol)
        ve Unreal'ın tekil portuna (7402) iletir.
        """
        key = str(sample.key_expr)
        raw_bytes = sample.payload.to_bytes()
        parts = key.split("/")
        if len(parts) < 2:
            return

        v_code = parts[1]
        decoded, ack_status = ros2_cdr.deserialize_from_topic(key, raw_bytes)
        self._update_ack_status(v_code, "channels", ack_status)
        self._record_and_maybe_inspect(key, raw_bytes, "RX_FROM_ZENOH")

        if ack_status == ros2_cdr.ACK_INVALID_SCHEMA:
            return

        ch_map = decoded.get("channels", decoded) if isinstance(decoded, dict) else {}
        ch_norm = decoded.get("channels_normalized", {}) if isinstance(decoded, dict) else {}

        packet = {
            "topic": f"/{v_code}/channels",
            "cmd": "channels",
            "target": v_code,
            "channels": ch_map,
            "channels_normalized": ch_norm,
            "timeout_ms": 600,
        }
        self._send_to_unreal_single_port(v_code, packet)

    def _on_zenoh_ack(self, sample):
        """
        Pi 5 veya Web Dashboard'dan gelen Binary CDR ACK & Trigger mesajlarını işler:
          - Kod 3: Kamerayı tam çözünürlüklü H.264 modunda uyandır (Wake-up)
          - Kod 4: Kamerayı Mavi Ekran (Standby) moduna geri al
          - Kod 0, 1, 2: Topic sağlık durumlarını kaydet ve Unreal'a (Port 7402) ilet
        """
        key = str(sample.key_expr)
        raw_bytes = sample.payload.to_bytes()
        parts = key.split("/")
        if len(parts) < 2:
            return

        v_code = parts[1]
        decoded, ack_status = ros2_cdr.deserialize_from_topic(key, raw_bytes)
        if not isinstance(decoded, dict):
            return

        # Eğer köprünün kendisi yayınladıysa döngüye girmemek için atla
        if decoded.get("source") == "bridge":
            return

        self._record_and_maybe_inspect(key, raw_bytes, "RX_ACK_FROM_PI5")
        topics_map = decoded.get("topics", {})
        if isinstance(topics_map, dict):
            for t_name, code in topics_map.items():
                code_int = int(code)
                self._update_ack_status(v_code, t_name, code_int)
                if "camera" in t_name.lower() or "h264" in t_name.lower():
                    if code_int == ros2_cdr.ACK_TRIGGER_STREAM:
                        self.camera_live_triggers[v_code] = time.time()
                    elif code_int == ros2_cdr.ACK_STANDBY_SLEEP:
                        self.camera_live_triggers[v_code] = -100.0

        packet = {
            "topic": f"/{v_code}/ack",
            "cmd": "ack",
            "target": v_code,
            "topics": topics_map,
        }
        self._send_to_unreal_single_port(v_code, packet)

    def _on_zenoh_actuators(self, sample):
        """Zenoh'tan gelen Binary CDR aktüatör komutunu Unreal'ın tekil portuna (7402) yönlendirir."""
        key = str(sample.key_expr)
        raw_bytes = sample.payload.to_bytes()
        parts = key.split("/")
        if len(parts) < 2:
            return

        v_code = parts[1]
        decoded, ack_status = ros2_cdr.deserialize_from_topic(key, raw_bytes)
        self._update_ack_status(v_code, "actuators", ack_status)
        self._record_and_maybe_inspect(key, raw_bytes, "RX_FROM_ZENOH")

        if ack_status == ros2_cdr.ACK_INVALID_SCHEMA:
            return

        motors = decoded.get("motors", decoded) if isinstance(decoded, dict) else {}
        packet = {
            "topic": f"/{v_code}/actuators",
            "cmd": "actuators",
            "target": v_code,
            "motors": motors,
            "timeout_ms": 600,
        }
        self._send_to_unreal_single_port(v_code, packet)

    def _on_zenoh_cmd_vel(self, sample):
        """Zenoh'tan gelen Binary CDR Twist hız komutunu Unreal'a (Port 7402) iletir."""
        key = str(sample.key_expr)
        raw_bytes = sample.payload.to_bytes()
        parts = key.split("/")
        if len(parts) < 2:
            return

        v_code = parts[1]
        cmd_data, ack_status = ros2_cdr.deserialize_from_topic(key, raw_bytes)
        self._update_ack_status(v_code, "cmd_vel", ack_status)
        self._record_and_maybe_inspect(key, raw_bytes, "RX_FROM_ZENOH")

        if not isinstance(cmd_data, dict):
            return

        packet = {
            "topic": f"/{v_code}/cmd_vel",
            "cmd": "cmd_vel",
            "target": v_code,
            "linear": cmd_data.get("linear", {}),
            "angular": cmd_data.get("angular", {}),
            "linear_x": cmd_data.get("linear", {}).get("x", 0.0),
            "angular_z": cmd_data.get("angular", {}).get("z", 0.0),
            "timeout_ms": 600,
        }
        self._send_to_unreal_single_port(v_code, packet)

    def _on_zenoh_ping(self, sample):
        """Zenoh'tan gelen ping testini Unreal'a iletir."""
        key = str(sample.key_expr)
        payload = sample.payload.to_string()
        parts = key.split("/")
        if len(parts) >= 2:
            v_code = parts[1]
            packet = {"topic": f"/{v_code}/ping", "cmd": "ping", "target": v_code, "msg": payload}
            self._send_to_unreal_single_port(v_code, packet)

    def _on_zenoh_fleet_state(self, sample):
        """Diğer P2P ajanlarından gelen dağıtık bellek durumunu günceller."""
        key = str(sample.key_expr)
        raw_bytes = sample.payload.to_bytes()
        parts = key.split("/")
        if len(parts) >= 2:
            v_code = parts[1]
            try:
                state_obj, _ = ros2_cdr.deserialize_from_topic(key, raw_bytes)
                if isinstance(state_obj, dict):
                    with self.fleet_lock:
                        self.fleet_state[v_code] = state_obj
            except Exception:
                pass

    def publish_channels(self, vehicle_code: str, channels: Dict[int, float]):
        """Kanal komutlarını (CH1: Tekerlek, CH2: Steer Servo) Binary CDR olarak Zenoh ve Unreal'a gönderir."""
        zenoh_key = f"pisim/{vehicle_code}/channels"
        cdr_bytes = ros2_cdr.serialize_channels_cdr(channels)
        if self.session:
            try:
                self.session.put(zenoh_key, cdr_bytes)
            except Exception:
                pass
        decoded = ros2_cdr.deserialize_channels_cdr(cdr_bytes)
        packet = {
            "topic": f"/{vehicle_code}/channels",
            "cmd": "channels",
            "target": vehicle_code,
            "channels": decoded.get("channels", {}),
            "channels_normalized": decoded.get("channels_normalized", {}),
            "timeout_ms": 600,
        }
        self._send_to_unreal_single_port(vehicle_code, packet)

    def publish_actuators(self, vehicle_code: str, motors: Dict[str, float]):
        """Python istemcisinden doğrudan bir aracın aktüatörlerini Binary CDR olarak Zenoh & Unreal'a gönderir."""
        zenoh_key = f"pisim/{vehicle_code}/actuators"
        cdr_bytes = ros2_cdr.serialize_actuators_cdr(motors)
        if self.session:
            try:
                self.session.put(zenoh_key, cdr_bytes)
            except Exception:
                pass
        packet = {"topic": f"/{vehicle_code}/actuators", "cmd": "actuators", "target": vehicle_code, "motors": motors}
        self._send_to_unreal_single_port(vehicle_code, packet)

    def get_fleet_telemetry(self) -> Dict[str, Any]:
        """Tüm filonun canlı dağıtık durumunu anında döndürür."""
        now = time.time()
        with self.fleet_lock:
            res = {}
            for k, v in self.fleet_state.items():
                v_copy = dict(v)
                v_copy["online"] = (now - v_copy.get("last_update", 0.0)) < 4.0
                v_copy["ack_table"] = dict(self.vehicle_ack_table.get(k, {}))
                v_copy["camera_mode"] = "LIVE_H264 (3)" if self.is_camera_live_triggered(k) else "STANDBY_BLUE (4)"
                res[k] = v_copy
            return res


def main():
    parser = argparse.ArgumentParser(description="PiSim Zenoh Mesh & ROS 2 Binary CDR Bridge")
    parser.add_argument("--mode", choices=["peer", "client"], default="peer", help="Zenoh çalışma modu (peer / client)")
    parser.add_argument("--router", type=str, default=None, help="İstemci modunda Zenoh Router adresi (tcp/IP:7447)")
    parser.add_argument("--listen", type=str, default="tcp/0.0.0.0:7447", help="Zenoh dinleme soketi (varsayılan: tcp/0.0.0.0:7447)")
    parser.add_argument("--ue5-ip", type=str, default="127.0.0.1", help="Unreal Engine simülasyon IP adresi")
    parser.add_argument(
        "--inspect",
        type=str,
        default=None,
        help="İstediğiniz topic'in hem Binary CDR hem JSON halini canlı gösterir (örn: --inspect all, --inspect imu, --inspect RV01)",
    )
    args = parser.parse_args()

    bridge = ZenohMeshBridge(
        mode=args.mode,
        router_endpoint=args.router,
        ue5_ip=args.ue5_ip,
        listen_endpoint=args.listen,
        inspect_filter=args.inspect,
    )
    bridge.start()

    print("\n" + "=" * 78)
    print(" 📡 PISIM UNREAL <-> ZENOH BINARY CDR KÖPRÜSÜ ÇALIŞIYOR")
    print(f" -> Unreal Sensör Girişi (Tek Port) : UDP {UNREAL_SENSOR_RX_PORT} (Anında Binary CDR'a çevrilir)")
    print(f" -> Unreal Komut Çıkışı  (Tek Port) : UDP {UNREAL_SINGLE_TX_PORT} (CH1=Tekerlek, CH2=Steer Servo, ACK)")
    print(" -> Kamera Modu                     : Varsayılan Mavi Ekran (Standby) | ACK=3 ile Canlı H.264")
    print(" -> Canlı Binary CDR + JSON Görmek  : '--inspect <topic>' ile başlatın (Örn: --inspect imu)")
    print("=" * 78 + "\n")

    try:
        while True:
            time.sleep(1.0)
            fleet = bridge.get_fleet_telemetry()
            if fleet and not bridge.inspect_filter:
                codes = [
                    f"{c} [Online:{d.get('online', False)} | Cam:{d.get('camera_mode')}]"
                    for c, d in fleet.items()
                ]
                sys.stdout.write(f"\r[ZenohBridge] Aktif Filo: {', '.join(codes)}       ")
                sys.stdout.flush()
    except KeyboardInterrupt:
        print("\nKapatılıyor...")
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()
