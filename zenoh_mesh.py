#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
zenoh_mesh.py - PiSim Eclipse Zenoh & ROS 2 P2P Mesh Köprüsü ve Dağıtık Durum Ağı
===================================================================================
Bu modül, merkezi SQL veritabanı darboğazını ve manuel UDP port karmaşasını tamamen ortadan kaldırır:

1. Saf P2P (Peer-to-Peer) Ağ Mimarisi:
   - Eclipse Zenoh protokolü ile çalışır (DDS'in WAN/Wi-Fi/Docker kısıtlarını aşar).
   - Ağdaki her cihaz (Raspberry Pi 5, Unreal Engine Host PC, Docker konteynerleri, Jetson)
     birbirini otomatik keşfeder (Zero-Config Multicast veya TCP/UDP Peer).
   - SQL veritabanı yerine mikro-saniye gecikmeli "In-Memory Distributed Key-Value Mesh" sunar.

2. Zenoh Topic / Key Yapısı (ROS 2 Eşdeğeri):
   - Telemetri (Unreal -> Pi 5 / Docker):
     * `pisim/<vehicle_code>/imu`          : sensor_msgs/msg/Imu
     * `pisim/<vehicle_code>/gps`          : sensor_msgs/msg/NavSatFix
     * `pisim/<vehicle_code>/joint_states` : sensor_msgs/msg/JointState
   - Kontrol (Pi 5 / Docker / Brain -> Unreal):
     * `pisim/<vehicle_code>/actuators`    : Doğrudan aktüatör normalize değerleri (Servo + Tekerlek)
     * `pisim/<vehicle_code>/channels`     : ArduPilot PWM Kanalları (1..16)
     * `pisim/<vehicle_code>/cmd_vel`      : geometry_msgs/msg/Twist
   - Filo Durumu (Tüm Ajanlar Arası Dağıtık Bellek):
     * `fleet/<vehicle_code>/state`        : Canlı durum özeti (In-Memory Key-Value)

3. Unreal Engine Entegrasyonu:
   - Unreal tarafındaki UPiSimTopicBridge ile yerel/ağ UDP üzerinden çift yönlü köprü kurar.
   - Unreal'dan gelen sensörleri Zenoh topic'lerine enjekte eder; Zenoh'tan gelen aktüatör
     komutlarını Unreal'ın aktüatör arayüzüne (IPiSimActuatorInterface) doğrudan iletir.
"""

import argparse
import json
import logging
import socket
import sys
import threading
import time
from typing import Callable, Dict, List, Optional, Any

# Windows terminal Türkçe kod sayfası (cp1254/cp437) desteği
if sys.stdout and hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

logging.basicConfig(level=logging.INFO, format="[%(asctime)s][ZenohMesh] %(message)s")
logger = logging.getLogger("ZenohMesh")

try:
    import zenoh
    HAS_ZENOH = True
except ImportError:
    HAS_ZENOH = False
    logger.warning("eclipse-zenoh modülü bulunamadı. Lütfen 'pip install eclipse-zenoh' çalıştırın.")

UNREAL_TOPIC_RX_PORT = 7500  # Unreal'ın sensör yayınladığı UDP portu
UNREAL_BASE_TX_PORT = 7410   # Unreal'ın dinlediği temel port (7410 + VehicleId)


class ZenohMeshBridge:
    """
    Unreal Engine PiSimTopicBridge ile Zenoh P2P Ağı arasında çift yönlü köprü ve durum yöneticisi.
    """

    def __init__(self, mode: str = "peer", router_endpoint: Optional[str] = None, ue5_ip: str = "127.0.0.1", listen_endpoint: Optional[str] = "tcp/0.0.0.0:7447"):
        self.mode = mode
        self.router_endpoint = router_endpoint
        self.listen_endpoint = listen_endpoint
        self.ue5_ip = ue5_ip
        self.running = False
        self.session: Optional[Any] = None

        # Dağıtık Araç Belleği (In-Memory Distributed Mesh) - SQL yerine kullanılır
        self.fleet_state: Dict[str, Dict[str, Any]] = {}
        self.fleet_lock = threading.Lock()

        # UDP Soketleri (Unreal Engine ile haberleşme)
        self.udp_rx_sock: Optional[socket.socket] = None
        self.udp_tx_sock: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_tx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # Zenoh Yayıncı ve Aboneleri
        self.zenoh_pubs: Dict[str, Any] = {}
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
                    logger.info("📡 Zenoh Dağıtıcı Router Modu Aktif (Tüm Docker ve Pi5 düğümlerini birbirine bağlar).")

                self.session = zenoh.open(conf)
                logger.info("✅ Zenoh Oturumu Başarıyla Açıldı.")

                # Kontrol Topic'lerine Abone Ol (Pi 5 veya Docker'dan gelen aktüatör komutlarını Unreal'a ilet)
                sub_actuators = self.session.declare_subscriber("pisim/*/actuators", self._on_zenoh_actuators)
                sub_channels = self.session.declare_subscriber("pisim/*/channels", self._on_zenoh_channels)
                sub_cmdvel = self.session.declare_subscriber("pisim/*/cmd_vel", self._on_zenoh_cmd_vel)
                sub_ping = self.session.declare_subscriber("pisim/*/ping", self._on_zenoh_ping)
                self.zenoh_subs.extend([sub_actuators, sub_channels, sub_cmdvel, sub_ping])

                # Filo Durumu Paylaşımına Abone Ol
                sub_fleet = self.session.declare_subscriber("fleet/*/state", self._on_zenoh_fleet_state)
                self.zenoh_subs.append(sub_fleet)

            except Exception as e:
                logger.error(f"Zenoh başlatma hatası: {e}. Yerel köprü modunda devam ediliyor.")
                self.session = None

        # Unreal Engine Dinleyici İş Parçacığını Başlat
        rx_thread = threading.Thread(target=self._unreal_rx_loop, daemon=True)
        rx_thread.start()

        # Otomatik Keşif Beacon İş Parçacığını Başlat (Sıfır IP Girişi / Zero-Config)
        beacon_thread = threading.Thread(target=self._beacon_loop, daemon=True)
        beacon_thread.start()
        logger.info(f"🚀 PiSim Zenoh Mesh Köprüsü Aktif! (Unreal RX Port: {UNREAL_TOPIC_RX_PORT})")

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

    def _unreal_rx_loop(self):
        """Unreal Engine PiSimTopicBridge'den gelen UDP sensör paketlerini dinler ve Zenoh'a basar."""
        try:
            self.udp_rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.udp_rx_sock.bind(("0.0.0.0", UNREAL_TOPIC_RX_PORT))
            self.udp_rx_sock.settimeout(0.5)
        except Exception as e:
            logger.error(f"Unreal UDP RX Soket açılamadı (Port {UNREAL_TOPIC_RX_PORT}): {e}")
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

                # Topic Örneği: "/rover_01/imu" -> Zenoh Anahtarı: "pisim/rover_01/imu"
                clean_topic = topic.lstrip("/")
                zenoh_key = f"pisim/{clean_topic}"

                # 1. Zenoh Ağına Yayınla (DDS/ROS2/Pi5 istemcileri dinleyebilir)
                if self.session:
                    try:
                        self.session.put(zenoh_key, json.dumps(msg))
                    except Exception:
                        pass

                # 2. Dağıtık Bellek Durumunu Güncelle (SQL Veritabanı yerine!)
                parts = clean_topic.split("/")
                if len(parts) >= 2:
                    v_code = parts[0]
                    sub_topic = parts[1]
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

                        if sub_topic == "imu":
                            entry["imu"] = msg
                            # Oryantasyondan yaklaşık açıları çıkar
                            q = msg.get("orientation", {})
                            qx, qy, qz, qw = q.get("x", 0.0), q.get("y", 0.0), q.get("z", 0.0), q.get("w", 1.0)
                            # Roll, pitch, yaw rad -> deg
                            entry["yaw"] = round(57.2958 * 2.0 * (qw * qz + qx * qy), 2)
                        elif sub_topic == "gps":
                            entry["gps"] = msg
                            entry["lat"] = msg.get("latitude", 0.0)
                            entry["lon"] = msg.get("longitude", 0.0)
                            entry["alt"] = msg.get("altitude", 0.0)
                        elif sub_topic == "joint_states":
                            entry["joints"] = msg

            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    logger.debug(f"RX döngüsü hatası: {e}")

    def _extract_vehicle_id(self, vehicle_code: str) -> int:
        """'rover_01' gibi kodlardan tam sayı ID (1) çıkarır."""
        digits = "".join(filter(str.isdigit, vehicle_code))
        return int(digits) if digits else 1

    def _on_zenoh_actuators(self, sample):
        """Zenoh'tan gelen aktüatör komutunu Unreal'ın PiSimTopicBridge'ine yönlendirir."""
        key = str(sample.key_expr)
        payload = sample.payload.to_string()
        parts = key.split("/")
        if len(parts) >= 2:
            v_code = parts[1]
            v_id = self._extract_vehicle_id(v_code)
            target_port = UNREAL_BASE_TX_PORT + v_id

            # UDP Paketi Formatla ve Unreal'a Gönder
            packet = {
                "topic": f"/{v_code}/actuators",
                "cmd": "actuators",
                "target": v_code,
                "motors": json.loads(payload),
                "timeout_ms": 600,
            }
            raw = json.dumps(packet).encode("utf-8")
            # Hem TopicBridge portuna (7410 + id) hem de Araç Dedicated portuna (7402 + id) ilet
            self.udp_tx_sock.sendto(raw, (self.ue5_ip, target_port))
            self.udp_tx_sock.sendto(raw, (self.ue5_ip, 7402 + v_id))
            self.udp_tx_sock.sendto(raw, (self.ue5_ip, 7402))

    def _on_zenoh_channels(self, sample):
        """Zenoh'tan gelen ArduPilot PWM kanallarını Unreal'a iletir."""
        key = str(sample.key_expr)
        payload = sample.payload.to_string()
        parts = key.split("/")
        if len(parts) >= 2:
            v_code = parts[1]
            v_id = self._extract_vehicle_id(v_code)
            target_port = UNREAL_BASE_TX_PORT + v_id

            packet = {
                "topic": f"/{v_code}/channels",
                "cmd": "channels",
                "target": v_code,
                "channels": json.loads(payload),
                "timeout_ms": 600,
            }
            raw = json.dumps(packet).encode("utf-8")
            self.udp_tx_sock.sendto(raw, (self.ue5_ip, target_port))
            self.udp_tx_sock.sendto(raw, (self.ue5_ip, 7402 + v_id))

    def _on_zenoh_cmd_vel(self, sample):
        """Zenoh'tan gelen Twist hız komutunu Unreal'a iletir."""
        key = str(sample.key_expr)
        payload = sample.payload.to_string()
        parts = key.split("/")
        if len(parts) >= 2:
            v_code = parts[1]
            v_id = self._extract_vehicle_id(v_code)
            target_port = UNREAL_BASE_TX_PORT + v_id

            cmd_data = json.loads(payload)
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
            raw = json.dumps(packet).encode("utf-8")
            self.udp_tx_sock.sendto(raw, (self.ue5_ip, target_port))
            self.udp_tx_sock.sendto(raw, (self.ue5_ip, 7402 + v_id))
            self.udp_tx_sock.sendto(raw, (self.ue5_ip, 7402))

    def _on_zenoh_ping(self, sample):
        """Zenoh'tan gelen ping testini Unreal TopicBridge'e iletir."""
        key = str(sample.key_expr)
        payload = sample.payload.to_string()
        parts = key.split("/")
        if len(parts) >= 2:
            v_code = parts[1]
            v_id = self._extract_vehicle_id(v_code)
            target_port = UNREAL_BASE_TX_PORT + v_id
            packet = {"topic": f"/{v_code}/ping", "cmd": "ping", "msg": payload}
            self.udp_tx_sock.sendto(json.dumps(packet).encode("utf-8"), (self.ue5_ip, target_port))

    def _on_zenoh_fleet_state(self, sample):
        """Diğer P2P ajanlarından gelen dağıtık bellek durumunu günceller."""
        key = str(sample.key_expr)
        payload = sample.payload.to_string()
        parts = key.split("/")
        if len(parts) >= 2:
            v_code = parts[1]
            try:
                state_obj = json.loads(payload)
                with self.fleet_lock:
                    self.fleet_state[v_code] = state_obj
            except Exception:
                pass

    def publish_actuators(self, vehicle_code: str, motors: Dict[str, float]):
        """Python istemcisinden doğrudan bir aracın aktüatörlerini Zenoh & Unreal'a gönderir."""
        zenoh_key = f"pisim/{vehicle_code}/actuators"
        payload_str = json.dumps(motors)

        if self.session:
            try:
                self.session.put(zenoh_key, payload_str)
            except Exception:
                pass

        # Doğrudan UDP yedekleme (Zenoh bağımsız çalışması için)
        v_id = self._extract_vehicle_id(vehicle_code)
        target_port = UNREAL_BASE_TX_PORT + v_id
        packet = {"topic": f"/{vehicle_code}/actuators", "motors": motors}
        self.udp_tx_sock.sendto(json.dumps(packet).encode("utf-8"), (self.ue5_ip, target_port))

    def get_fleet_telemetry(self) -> Dict[str, Any]:
        """Tüm filonun canlı dağıtık durumunu anında döndürür (SQL kilitlemesi olmadan!)."""
        now = time.time()
        with self.fleet_lock:
            res = {}
            for k, v in self.fleet_state.items():
                v_copy = dict(v)
                v_copy["online"] = (now - v_copy.get("last_update", 0.0)) < 4.0
                res[k] = v_copy
            return res


def main():
    parser = argparse.ArgumentParser(description="PiSim Zenoh Mesh & ROS 2 Decentralized Bridge")
    parser.add_argument("--mode", choices=["peer", "client"], default="peer", help="Zenoh çalışma modu (peer / client)")
    parser.add_argument("--router", type=str, default=None, help="İstemci modunda Zenoh Router adresi (tcp/IP:7447)")
    parser.add_argument("--listen", type=str, default="tcp/0.0.0.0:7447", help="Zenoh dinleme soketi (varsayılan: tcp/0.0.0.0:7447)")
    parser.add_argument("--ue5-ip", type=str, default="127.0.0.1", help="Unreal Engine simülasyon IP adresi")
    args = parser.parse_args()

    bridge = ZenohMeshBridge(mode=args.mode, router_endpoint=args.router, ue5_ip=args.ue5_ip, listen_endpoint=args.listen)
    bridge.start()

    print("\n" + "=" * 70)
    print(" 📡 PISIM ZENOH DAĞITIK AĞI ÇALIŞIYOR")
    print(" -> Merkezi SQL gerekmez, P2P in-memory durum eşitleme aktif")
    print(" -> Ctrl+C ile durdurabilirsiniz.")
    print("=" * 70 + "\n")

    try:
        while True:
            time.sleep(1.0)
            fleet = bridge.get_fleet_telemetry()
            if fleet:
                codes = [f"{c} (Online: {d.get('online', False)})" for c, d in fleet.items()]
                sys.stdout.write(f"\r[ZenohMesh] Aktif Araçlar: {', '.join(codes)}       ")
                sys.stdout.flush()
    except KeyboardInterrupt:
        print("\nKapatılıyor...")
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()

