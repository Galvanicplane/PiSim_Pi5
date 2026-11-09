#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
topic_monitor.py - PiSim Zenoh Ağ ve Topic Monitörü
===================================================
Ağda yayınlanan tüm Zenoh topic'lerini (Unreal Engine sensörleri, otopilot komutları,
kamera yayınları, filo durumu) bağımsız bir terminalde canlı olarak gösterir.

Özellikler:
  1. Canlı Gösterge Modu (Varsayılan):
     Terminalde sabit, aşağı kaymayan tablo halinde tüm topic'leri, yayın frekanslarını (Hz),
     veri boyutlarını ve Unreal'daki aktif araçları listeler.
  2. Topic Listeleme Modu (--list):
     `ros2 topic list` gibi o an ağda aktif olan topic'lerin listesini yazdırıp çıkar.
  3. Topic Dinleme Modu (--echo <topic>):
     `ros2 topic echo /topic` gibi seçilen topic'e gelen mesajları canlı olarak ekrana basar.

Kullanım Örnekleri:
  - Canlı Monitör (Ayrı bir terminalde açın):
      python topic_monitor.py
      python topic_monitor.py --router tcp/192.168.1.3:7447

  - Aktif Topic Listesi:
      python topic_monitor.py --list

  - Belirli bir Topic'i Canlı İnceleme:
      python topic_monitor.py --echo pisim/rover_01/gps
"""

import argparse
import json
import os
import signal
import sys
import time
from collections import defaultdict
from typing import Any, Dict, Optional

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


def normalize_zenoh_endpoint(ep: Optional[str], default_port: int = 7447) -> Optional[str]:
    if not ep:
        return None
    ep = ep.strip()
    if ep.startswith("tcp://"):
        ep = "tcp/" + ep[6:]
    elif ep.startswith("udp://"):
        ep = "udp/" + ep[6:]
    elif not (ep.startswith("tcp/") or ep.startswith("udp/")):
        if ":" not in ep:
            ep = f"tcp/{ep}:{default_port}"
        else:
            ep = f"tcp/{ep}"
    return ep


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
            self.last_preview = f"JPEG Kare: {sz} bayt"
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
                        self.last_preview = f"Steer: {data.get('M_Steer_FL',0):.2f}, Wheel: {data.get('M_Wheel_FL',0):.2f}"
                    elif "vehicle" in data or "code" in data:
                        self.msg_type = "FleetState"
                        self.last_preview = f"Vehicle: {data.get('vehicle') or data.get('code')}"
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
        self.router_endpoint = normalize_zenoh_endpoint(router_endpoint)
        self.running = False
        self.session = None
        self.stats: Dict[str, TopicStats] = {}
        self.vehicles: set = set()

    def _init_zenoh(self) -> bool:
        if not HAS_ZENOH:
            print("❌ 'eclipse-zenoh' modülü bulunamadı! 'pip install eclipse-zenoh' kurun.")
            return False

        conf = zenoh.Config()

        # Otomatik yerel yönlendirici keşfi (127.0.0.1:7447)
        if not self.router_endpoint:
            try:
                import socket
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(0.3)
                if s.connect_ex(('127.0.0.1', 7447)) == 0:
                    self.router_endpoint = "tcp/127.0.0.1:7447"
                s.close()
            except Exception:
                pass

        if self.router_endpoint:
            conf.insert_json5("mode", json.dumps("client"))
            conf.insert_json5("connect/endpoints", json.dumps([self.router_endpoint]))
        else:
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

        if key not in self.stats:
            self.stats[key] = TopicStats(key)

        self.stats[key].record_sample(payload)

        # Araç kodunu çıkar (Örn: pisim/rover_01/gps -> rover_01)
        parts = key.split("/")
        if len(parts) >= 2 and parts[0] in ("pisim", "fleet"):
            self.vehicles.add(parts[1])

    def run_live_dashboard(self):
        """Terminalde aşağı kaymayan canlı gösterge paneli."""
        if not self._init_zenoh():
            return

        self.running = True
        # pisim/** ve fleet/** altındaki tüm topic'lere joker (wildcard) abone ol
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

        box_width = 96
        line_bar = "═" * (box_width - 2)

        lines.append(f"╔{line_bar}╗")
        router_text = self.router_endpoint if self.router_endpoint else "P2P Mesh (Otomatik)"
        title = f" 📡 PISIM ZENOH CANLI TOPIC VE ARAÇ MONİTÖRÜ | {router_text} "
        lines.append(f"║ {title.ljust(box_width - 4)} ║")
        lines.append(f"╠{line_bar}╣")

        # Aktif Araçlar Satırı
        v_list = ", ".join(sorted(self.vehicles)) if self.vehicles else "Henüz araç tespit edilmedi (Unreal bekleniyor...)"
        lines.append(f"║ 🚗 UNREAL'DAKİ AKTİF ARAÇLAR: [ {v_list} ]".ljust(box_width - 2) + " ║")
        lines.append(f"╠{line_bar}╣")

        # Tablo Başlığı
        header = f"║ {'TOPIC':<36} │ {'FREKANS':<9} │ {'BANT GEN.':<10} │ {'SON':<6} │ {'TİP / VERİ':<24} ║"
        lines.append(header)
        lines.append(f"╟{'─'*38}┼{'─'*11}┼{'─'*12}┼{'─'*8}┼{'─'*23}╢")

        if not self.stats:
            lines.append(f"║ {'Veri bekleniyor... (Unreal Engine veya yayıncı başlatıldığında burada listelenir)':<92} ║")
        else:
            # Topic'leri isme göre sırala
            for key in sorted(self.stats.keys()):
                st = self.stats[key]
                sec_ago = max(0.0, now - st.last_seen)

                # 3 saniyeden eski ise sönük göster
                if sec_ago > 3.0:
                    status_time = f"{sec_ago:4.1f}s"
                    hz_str = "0.0 Hz"
                else:
                    status_time = f"{sec_ago:4.2f}s"
                    hz_str = f"{st.hz:4.1f} Hz"

                kb_str = f"{st.kb_per_sec:4.1f} KB/s"
                prev_text = f"[{st.msg_type}] {st.last_preview}"[:24]

                row = f"║ {key[:36]:<36} │ {hz_str:<9} │ {kb_str:<10} │ {status_time:<6} │ {prev_text:<24} ║"
                lines.append(row)

        lines.append(f"╚{line_bar}╝")
        lines.append("\033[K(Durdurmak için Ctrl+C tuşlayın)")

        # ANSI Escape ile imleci başa al ve aşağı kaymadan üzerine çiz
        sys.stdout.write("\033[H" + "\n".join(lines) + "\n")
        sys.stdout.flush()

    def run_list(self):
        """Aktif topic'leri ve araçları tespit edip listeler."""
        if not self._init_zenoh():
            return
        print("🔍 Zenoh ağı taranıyor (2 saniye dinleniyor)...")
        self.session.declare_subscriber("pisim/**", self.on_sample)
        self.session.declare_subscriber("fleet/**", self.on_sample)
        time.sleep(2.0)

        print("\n=======================================================")
        print(f"🚗 TESPİT EDİLEN ARAÇLAR: {list(self.vehicles) if self.vehicles else 'Hiçbiri'}")
        print("-------------------------------------------------------")
        print("📡 AKTİF TOPIC'LER:")
        if not self.stats:
            print("  (Ağda henüz aktif topic bulunamadı)")
        for key in sorted(self.stats.keys()):
            st = self.stats[key]
            print(f"  * {key:<35} | {st.msg_type:<20} | {st.total_packets} paket")
        print("=======================================================\n")
        self.stop()

    def run_echo(self, target_topic: str):
        """Seçilen topic'i canlı olarak yazdırır."""
        if not self._init_zenoh():
            return

        print(f"📡 '{target_topic}' topic'i dinleniyor (Çıkmak için Ctrl+C)...")

        def echo_cb(sample):
            try:
                t = sample.payload.to_string()
                timestr = time.strftime("%H:%M:%S")
                print(f"[{timestr}][{sample.key_expr}]:\n{t}\n---")
            except Exception as e:
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
    parser.add_argument("--router", "-r", type=str, default=os.environ.get("ZENOH_ROUTER", None), help="Zenoh Router adresi (örn: tcp/192.168.1.3:7447)")
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
