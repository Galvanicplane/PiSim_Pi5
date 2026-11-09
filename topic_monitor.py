#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
topic_monitor.py - PiSim Zenoh Ağ, Araç ve Topic Monitörü
=========================================================
Ağda yayınlanan tüm Zenoh topic'lerini, Unreal Engine simülasyon araçlarını
ve Pi 5 Docker ajanlarını bağımsız bir terminalde canlı olarak gösterir.

Temel Kurallar:
  1. Unreal Araçları vs Docker Ajanları Ayrımı:
     - Bir araç SADECE Unreal'dan `gps` veya `imu` sensör yayını geliyorsa
       ve bu yayın son 2.5 saniye içinde aktifse "UNREAL ARAÇLARI" olarak listelenir.
     - Unreal çalışmıyorken Docker'ların açık olması asla "Unreal aracı var" dedirtmez!
     - Docker ajanları ayrı bir satırda "DOCKER AJANLARI" olarak durumlarıyla gösterilir.
  2. Sıfır IP Girişi (Zero-Config Auto Discovery):
     - IP belirtilmesine gerek yoktur!
     - Önce yerel yönlendiriciyi, ardından yerel ağdaki (LAN) yayınları otomatik keşfeder.
"""

import argparse
import json
import os
import socket
import sys
import time
from typing import Any, Dict, List, Optional

if sys.stdout and hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

try:
    import zenoh
    HAS_ZENOH = True
except ImportError:
    HAS_ZENOH = False


def auto_discover_zenoh_endpoint(timeout_sec: float = 0.5) -> Optional[str]:
    """Ağdaki Zenoh router veya yayın noktasını hiçbir IP girmeden otomatik keşfeder."""
    # 1. Yerel bilgisayardaki 7447 portunu kontrol et
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.2)
        if s.connect_ex(('127.0.0.1', 7447)) == 0:
            s.close()
            return "tcp/127.0.0.1:7447"
        s.close()
    except Exception:
        pass

    # 2. Yerel ağdaki (LAN) UDP broadcast beacon dinle (Port 7447)
    try:
        rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, "SO_REUSEPORT"):
            try:
                rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except Exception:
                pass
        rx_sock.bind(("", 7447))
        rx_sock.settimeout(timeout_sec)
        data, addr = rx_sock.recvfrom(2048)
        rx_sock.close()
        text = data.decode("utf-8", errors="ignore").strip()
        if "pisim_zenoh" in text:
            pkt = json.loads(text)
            return pkt.get("endpoint") or f"tcp/{addr[0]}:7447"
    except Exception:
        pass

    return None


class TopicStats:
    def __init__(self, key: str):
        self.key = key
        self.total_packets = 0
        self.total_bytes = 0
        self.last_seen = 0.0
        self.last_hz_calc_time = time.time()
        self.packets_in_window = 0
        self.bytes_in_window = 0
        self.hz = 0.0
        self.kb_per_sec = 0.0
        self.last_preview = ""
        self.msg_type = "Bilinmiyor"

    def record_sample(self, payload_bytes: bytes):
        now = time.time()
        sz = len(payload_bytes)
        self.total_packets += 1
        self.total_bytes += sz
        self.last_seen = now
        self.packets_in_window += 1
        self.bytes_in_window += sz

        dt = now - self.last_hz_calc_time
        if dt >= 1.0:
            self.hz = round(self.packets_in_window / dt, 1)
            self.kb_per_sec = round((self.bytes_in_window / 1024.0) / dt, 1)
            self.packets_in_window = 0
            self.bytes_in_window = 0
            self.last_hz_calc_time = now

        # Mesaj içeriğini analiz et
        if self.key.endswith("/camera/compressed"):
            self.msg_type = "Image/JPEG"
            self.last_preview = f"JPEG: {sz} bayt"
        elif sz > 0:
            try:
                text = payload_bytes.decode("utf-8", errors="ignore").strip()
                if text.startswith("{") and text.endswith("}"):
                    data = json.loads(text)
                    if "latitude" in data:
                        self.msg_type = "sensor_msgs/NavSatFix"
                        self.last_preview = f"Lat: {data['latitude']:.5f}, Lon: {data.get('longitude',0):.5f}"
                    elif "orientation" in data:
                        self.msg_type = "sensor_msgs/Imu"
                        self.last_preview = f"AngVel Z: {data.get('angular_velocity',{}).get('z',0):.2f}"
                    elif "linear" in data and "angular" in data:
                        self.msg_type = "geometry_msgs/Twist"
                        self.last_preview = f"LinX: {data['linear'].get('x',0):.1f}, AngZ: {data['angular'].get('z',0):.2f}"
                    elif "M_Steer_FL" in data or "M_Wheel_FL" in data:
                        self.msg_type = "Actuators/Motors"
                        self.last_preview = f"Steer: {data.get('M_Steer_FL',0):.2f}"
                    elif "agent_id" in data or "target_vehicle" in data:
                        self.msg_type = "AgentState"
                        target = data.get("target_vehicle", "auto")
                        self.last_preview = f"Hedef: {target} (Ajan: {data.get('agent_id')})"
                    else:
                        self.msg_type = "JSON"
                        self.last_preview = (text[:30] + "...") if len(text) > 30 else text
                else:
                    self.msg_type = "String/Raw"
                    self.last_preview = (text[:30] + "...") if len(text) > 30 else text
            except Exception:
                self.msg_type = "Binary"
                self.last_preview = f"{sz} bayt"


class ZenohTopicMonitor:
    def __init__(self, router_endpoint: Optional[str] = None):
        self.router_endpoint = router_endpoint
        self.running = False
        self.session = None
        self.stats: Dict[str, TopicStats] = {}

        # Unreal Simülasyon Araçları: Yalnızca gps/imu gelenler kaydedilir ve zamanı tutulur
        self.unreal_sensor_timestamps: Dict[str, float] = {}

        # Pi 5 Docker Ajanları: fleet/<agent_id>/state gelenler kaydedilir
        self.docker_agents: Dict[str, Dict] = {}

    def _init_zenoh(self) -> bool:
        if not HAS_ZENOH:
            print("❌ 'eclipse-zenoh' modülü bulunamadı! 'pip install eclipse-zenoh' kurun.")
            return False

        # Eğer router belirtilmemişse otomatik keşfet
        if not self.router_endpoint:
            self.router_endpoint = auto_discover_zenoh_endpoint()

        conf = zenoh.Config()

        if self.router_endpoint:
            ep = self.router_endpoint.strip()
            if ep.startswith("tcp://"):
                ep = "tcp/" + ep[6:]
            elif not ep.startswith("tcp/"):
                ep = f"tcp/{ep}:7447" if ":" not in ep else f"tcp/{ep}"
            conf.insert_json5("mode", json.dumps("client"))
            conf.insert_json5("connect/endpoints", json.dumps([ep]))
            self.router_endpoint = ep
        else:
            # Saf P2P Dağınık Ağ Modu (Multicast Zero-Config)
            conf.insert_json5("mode", json.dumps("peer"))

        try:
            self.session = zenoh.open(conf)
            return True
        except Exception as e:
            print(f"❌ Zenoh oturumu açılamadı: {e}")
            return False

    def on_sample(self, sample):
        key = str(sample.key_expr)
        payload = sample.payload.to_bytes()
        now = time.time()

        if key not in self.stats:
            self.stats[key] = TopicStats(key)

        self.stats[key].record_sample(payload)

        # -------------------------------------------------------------
        # 1. UNREAL MOTOR / SİMÜLASYON ARAÇLARINI AYIRT ET:
        # YALNIZCA pisim/<araç>/gps veya pisim/<araç>/imu geliyorsa Unreal aracıdır!
        # -------------------------------------------------------------
        if key.startswith("pisim/") and ("/gps" in key or "/imu" in key or "/camera" in key):
            parts = key.split("/")
            if len(parts) >= 2:
                v_code = parts[1]
                self.unreal_sensor_timestamps[v_code] = now

        # -------------------------------------------------------------
        # 2. PI 5 DOCKER AJANLARINI AYIRT ET:
        # fleet/<agent_id>/state veya pisim/<agent_id>/assign
        # -------------------------------------------------------------
        elif key.startswith("fleet/"):
            parts = key.split("/")
            if len(parts) >= 2:
                agent_id = parts[1]
                try:
                    data = json.loads(payload.decode("utf-8", errors="ignore"))
                except Exception:
                    data = {}
                self.docker_agents[agent_id] = {
                    "last_seen": now,
                    "target": data.get("target_vehicle", "auto"),
                    "connected": data.get("connected", False),
                }

    def run_live_dashboard(self):
        if not self._init_zenoh():
            return

        self.running = True
        self.session.declare_subscriber("pisim/**", self.on_sample)
        self.session.declare_subscriber("fleet/**", self.on_sample)
        self.session.declare_subscriber("**", self.on_sample)

        sys.stdout.write("\033[2J\033[H")
        sys.stdout.flush()

        try:
            while self.running:
                self._draw_dashboard()
                time.sleep(0.35)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def _draw_dashboard(self):
        now = time.time()
        lines = []

        box_width = 98
        line_bar = "═" * (box_width - 2)

        lines.append(f"╔{line_bar}╗")
        router_text = self.router_endpoint if self.router_endpoint else "P2P Dağınık Ağ (Zero-Config Peer)"
        title = f" 📡 PISIM ZENOH CANLI AĞ VE TOPIC MONİTÖRÜ | {router_text} "
        lines.append(f"║ {title.ljust(box_width - 4)} ║")
        lines.append(f"╠{line_bar}╣")

        # -------------------------------------------------------------
        # 1. UNREAL SİMÜLASYON ARAÇLARI (Son 2.5 sn içinde sensör gelenler)
        # -------------------------------------------------------------
        active_unreal = [
            v for v, last_t in self.unreal_sensor_timestamps.items()
            if (now - last_t) <= 2.5
        ]
        if active_unreal:
            v_str = f"🟢 [ {', '.join(sorted(active_unreal))} ]  (Canlı Sensör Akışı Aktif)"
        else:
            v_str = "⏸️ HİÇBİRİ (Unreal Engine Çalışmıyor veya Sensör Yayını Yok)"

        lines.append(f"║ 🎮 UNREAL SİMÜLASYON ARAÇLARI: {v_str[:box_width-34]}".ljust(box_width - 2) + " ║")

        # -------------------------------------------------------------
        # 2. PI 5 DOCKER AJANLARI (Beyinler)
        # -------------------------------------------------------------
        active_agents = []
        for a_id, a_data in self.docker_agents.items():
            if (now - a_data["last_seen"]) <= 4.0:
                conn_icon = "🟢" if a_data.get("connected") else "🟡"
                active_agents.append(f"{a_id} ({conn_icon} ➔ {a_data.get('target', 'auto')})")

        if active_agents:
            a_str = f"🤖 [ {', '.join(sorted(active_agents))} ]"
        else:
            a_str = "Ağda aktif Docker ajanı yok"

        lines.append(f"║ 🧠 PI 5 DOCKER AJANLARI      : {a_str[:box_width-34]}".ljust(box_width - 2) + " ║")
        lines.append(f"╠{line_bar}╣")

        # Tablo Başlığı
        header = f"║ {'TOPIC':<38} │ {'FREKANS':<9} │ {'BANT GEN.':<10} │ {'SON':<6} │ {'TİP / VERİ':<23} ║"
        lines.append(header)
        lines.append(f"╟{'─'*40}┼{'─'*11}┼{'─'*12}┼{'─'*8}┼{'─'*22}╢")

        if not self.stats:
            lines.append(f"║ {'Veri bekleniyor... (Unreal veya Ajanlar başlatıldığında burada listelenir)':<94} ║")
        else:
            for key in sorted(self.stats.keys()):
                st = self.stats[key]
                sec_ago = max(0.0, now - st.last_seen)

                # 3 saniyeden eski ise sönük / kesildi göster
                if sec_ago > 3.0:
                    status_time = f"{sec_ago:4.1f}s"
                    hz_str = "0.0 Hz"
                    msg_preview = f"[KESİLDİ] {st.msg_type}"
                else:
                    status_time = f"{sec_ago:4.2f}s"
                    hz_str = f"{st.hz:4.1f} Hz"
                    msg_preview = f"[{st.msg_type}] {st.last_preview}"

                kb_str = f"{st.kb_per_sec:4.1f} KB/s"
                prev_text = msg_preview[:23]

                row = f"║ {key[:38]:<38} │ {hz_str:<9} │ {kb_str:<10} │ {status_time:<6} │ {prev_text:<23} ║"
                lines.append(row)

        lines.append(f"╚{line_bar}╝")
        lines.append("\033[K(Durdurmak için Ctrl+C)")

        sys.stdout.write("\033[H" + "\n".join(lines) + "\n")
        sys.stdout.flush()

    def run_list(self):
        if not self._init_zenoh():
            return
        print("🔍 Zenoh ağı taranıyor (2 saniye dinleniyor)...")
        self.session.declare_subscriber("pisim/**", self.on_sample)
        self.session.declare_subscriber("fleet/**", self.on_sample)
        time.sleep(2.0)

        now = time.time()
        active_unreal = [v for v, last_t in self.unreal_sensor_timestamps.items() if (now - last_t) <= 2.5]
        active_agents = [a for a, d in self.docker_agents.items() if (now - d["last_seen"]) <= 4.0]

        print("\n=======================================================")
        print(f"🎮 AKTİF UNREAL ARAÇLARI: {active_unreal if active_unreal else 'Hiçbiri (Unreal Açık Değil)'}")
        print(f"🧠 AKTİF DOCKER AJANLARI: {active_agents if active_agents else 'Hiçbiri'}")
        print("-------------------------------------------------------")
        print("📡 AKTİF TOPIC'LER:")
        if not self.stats:
            print("  (Ağda henüz aktif topic bulunamadı)")
        for key in sorted(self.stats.keys()):
            st = self.stats[key]
            print(f"  * {key:<38} | {st.msg_type:<20} | {st.total_packets} paket")
        print("=======================================================\n")
        self.stop()

    def run_echo(self, target_topic: str):
        if not self._init_zenoh():
            return

        print(f"📡 '{target_topic}' topic'i dinleniyor (Çıkmak için Ctrl+C)...")

        def echo_cb(sample):
            try:
                t = sample.payload.to_string()
                timestr = time.strftime("%H:%M:%S")
                print(f"[{timestr}][{sample.key_expr}]:\n{t}\n---")
            except Exception:
                print(f"[Ham Bayt]: {len(sample.payload.to_bytes())} bayt")

        self.session.declare_subscriber(target_topic, echo_cb)
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def stop(self):
        self.running = False
        if self.session:
            try:
                self.session.close()
            except Exception:
                pass


def main():
    parser = argparse.ArgumentParser(description="PiSim Zenoh Topic ve Araç Monitörü")
    parser.add_argument("--router", "-r", type=str, default=None, help="İsteğe bağlı Zenoh Router adresi (boş bırakılırsa otomatik keşfedilir)")
    parser.add_argument("--list", "-l", action="store_true", help="Yalnızca aktif topic ve araçları listele")
    parser.add_argument("--echo", "-e", type=str, default=None, help="Belirli bir topic'i canlı dinle (örn: pisim/rover_01/gps)")

    args = parser.parse_args()

    monitor = ZenohTopicMonitor(router_endpoint=args.router)

    if args.list:
        monitor.run_list()
    elif args.echo:
        monitor.run_echo(args.echo)
    else:
        monitor.run_live_dashboard()


if __name__ == "__main__":
    main()
