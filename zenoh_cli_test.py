#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
zenoh_cli_test.py - PiSim Zenoh P2P Dağınık Ağ ve İnteraktif Terminal İletişim Aracı
======================================================================================
Bu araç, hem Windows hem de her bir Docker konteyneri içinde ayrı bir terminal olarak
çalıştırılabilir:

Özellikler:
1. Otomatik Cihaz Kimliği:
   - Ortamdaki `VEHICLE_CODE`'a göre (rover_01, rover_02, rover_03) veya hostname ile
     otomatik olarak kendi topic'ini (`pisim/devices/<cihaz_adi>`) ağa duyurur.
2. Dağınık Terminal Sohbet Modu (Live Chat / String Messaging):
   - Herhangi bir Docker terminalinde yazdığınız string mesaj anında diğer Docker ve
     Windows terminallerine düşer.
3. Canlı Topic Dinleme & Yayınlama (Inspect & Publish):
   - Ağdaki tüm aktif topic'leri listeler veya istenen topic'e özel veri basar.
4. Unreal Engine Ping / Echo Testi.

Kullanım:
  Windows:         python zenoh_cli_test.py
  Docker Rover 1:  docker exec -it pisim_agent_rover_01 python zenoh_cli_test.py
  Docker Rover 2:  docker exec -it pisim_agent_rover_02 python zenoh_cli_test.py
  Docker Rover 3:  docker exec -it pisim_agent_rover_03 python zenoh_cli_test.py
"""

import argparse
import json
import os
import socket
import sys
import threading
import time
from typing import Dict, Set

try:
    import zenoh
    HAS_ZENOH = True
except ImportError:
    HAS_ZENOH = False


class ZenohCliTester:
    def __init__(self, mode: str = "peer", router_endpoint: str = None):
        if not HAS_ZENOH:
            print("❌ 'eclipse-zenoh' kütüphanesi bulunamadı! 'pip install eclipse-zenoh' çalıştırın.")
            sys.exit(1)

        self.mode = mode
        self.router_endpoint = router_endpoint
        self.hostname = socket.gethostname()

        # Otomatik Cihaz Kimliği Belirleme (Docker ortam değişkeni veya Hostname)
        env_veh = os.environ.get("VEHICLE_CODE", "")
        if env_veh:
            self.node_id = f"{env_veh}_{self.hostname}"
            self.device_name = env_veh
        else:
            self.node_id = f"host_{self.hostname}"
            self.device_name = "Windows_Host"

        self.running = True

        # Eğer router belirtilmemişse ortam değişkeninden al (Rover 03 için)
        if not self.router_endpoint:
            env_router = os.environ.get("ZENOH_ROUTER", None)
            if env_router:
                self.router_endpoint = env_router
                self.mode = "client"

        listen_ep = os.environ.get("ZENOH_LISTEN_ENDPOINT", None)

        self.discovered_topics: Dict[str, tuple] = {}
        self.topics_lock = threading.Lock()

        # Zenoh Oturumunu Yapılandır
        conf = zenoh.Config()
        if listen_ep:
            conf.insert_json5("listen/endpoints", json.dumps([listen_ep]))
            print(f"📡 Zenoh Dinleme Noktası Açıldı: {listen_ep}")

        if self.mode == "client" and self.router_endpoint:
            conf.insert_json5("mode", json.dumps("client"))
            conf.insert_json5("connect/endpoints", json.dumps([self.router_endpoint]))
            print(f"🔗 Zenoh Yönlendiricisine Bağlanılıyor: {self.router_endpoint}")
        else:
            conf.insert_json5("mode", json.dumps("peer"))
            print(f"📡 Zenoh P2P Mesh Modu Açılıyor...")

        self.session = zenoh.open(conf)
        print(f"✅ [{self.node_id}] Zenoh Ağına Başarıyla Katıldı!")

        # Kendi Cihaz Varlığımızı Ağa Duyur
        dev_topic = f"pisim/devices/{self.device_name}"
        self.session.put(dev_topic, json.dumps({
            "device": self.device_name,
            "node_id": self.node_id,
            "mode": self.mode,
            "status": "ONLINE",
            "time": time.time()
        }))

        # Küresel topic izleyici
        self.wildcard_sub = self.session.declare_subscriber("**", self._on_global_sample)

        # Heartbeat iş parçacığı
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self.heartbeat_thread.start()

    def _on_global_sample(self, sample):
        topic = str(sample.key_expr)
        payload = sample.payload.to_string()
        now = time.time()
        with self.topics_lock:
            self.discovered_topics[topic] = (now, payload)

    def _heartbeat_loop(self):
        while self.running:
            try:
                hb_key = f"pisim/nodes/{self.node_id}"
                msg = {
                    "node": self.node_id,
                    "device": self.device_name,
                    "mode": self.mode,
                    "ts": time.time(),
                }
                self.session.put(hb_key, json.dumps(msg))
            except Exception:
                pass
            time.sleep(2.0)

    def list_topics(self):
        print("\n" + "=" * 70)
        print(f" 📋 KEŞFEDİLEN AKTİF ZENOH TOPIC'LERİ (Düğüm: {self.node_id})")
        print("=" * 70)
        now = time.time()
        with self.topics_lock:
            if not self.discovered_topics:
                print(" ⏳ Henüz ağda bir veri tespit edilmedi.")
                return []

            topics_sorted = sorted(self.discovered_topics.keys())
            for idx, t in enumerate(topics_sorted, 1):
                last_time, sample_data = self.discovered_topics[t]
                age = now - last_time
                preview = sample_data.replace("\n", " ")[:60]
                status = "🟢 Canlı" if age < 4.0 else f"🟡 {age:.1f}s önce"
                print(f" [{idx:02d}] {status:10s} {t:38s} | {preview}...")
            return topics_sorted

    def subscribe_live(self, topic: str):
        print(f"\n📡 '{topic}' topic'i canlı dinleniyor... (Çıkmak için ENTER tuşuna basın)")
        print("-" * 70)

        stop_event = threading.Event()

        def _sub_callback(sample):
            if stop_event.is_set():
                return
            t_str = time.strftime("%H:%M:%S")
            k = str(sample.key_expr)
            p = sample.payload.to_string()
            try:
                parsed = json.loads(p)
                p_fmt = json.dumps(parsed, ensure_ascii=False)
            except Exception:
                p_fmt = p
            print(f"[{t_str}] 📥 [{k}]:\n  {p_fmt}")

        sub = self.session.declare_subscriber(topic, _sub_callback)
        try:
            input()
        except KeyboardInterrupt:
            pass
        finally:
            stop_event.set()
            try:
                sub.undeclare()
            except Exception:
                pass
            print("⏹️ Dinleme sonlandırıldı.\n")

    def publish_message(self, topic: str, message: str):
        print(f"📤 Yayınlanıyor -> Topic: '{topic}'")
        self.session.put(topic, message)
        print("✅ Mesaj ağa başarıyla basıldı.")

    def live_chat_mode(self, chat_topic: str = "pisim/chat"):
        """Terminaller arası anlık string / mesajlaşma konsolu."""
        print("\n" + "=" * 70)
        print(f" 💬 DAĞITIK TERMİNAL MESAJLAŞMA MODU (Topic: {chat_topic})")
        print(f"    Cihazınız: [{self.device_name}] ({self.node_id})")
        print("    Buraya yazacağınız her mesaj tüm diğer Docker ve Windows")
        print("    terminallerine anında iletilir!")
        print("    (Çıkmak için boş mesaj atıp ENTER'a veya '/exit' yazın)")
        print("=" * 70 + "\n")

        stop_chat = threading.Event()

        def _chat_rx(sample):
            if stop_chat.is_set():
                return
            try:
                pkt = json.loads(sample.payload.to_string())
                sender = pkt.get("sender", "Bilinmeyen")
                text = pkt.get("text", "")
                if sender != self.node_id:
                    t_str = time.strftime("%H:%M:%S")
                    sys.stdout.write(f"\r\033[K[{t_str}] 📨 [{sender}]: {text}\n> ")
                    sys.stdout.flush()
            except Exception:
                pass

        sub = self.session.declare_subscriber(chat_topic, _chat_rx)

        # Katılma Bildirimi
        join_msg = {"sender": self.node_id, "text": f"🎉 [{self.device_name}] sohbete katıldı!", "ts": time.time()}
        self.session.put(chat_topic, json.dumps(join_msg))

        try:
            while not stop_chat.is_set():
                msg = input("> ").strip()
                if not msg or msg.lower() in ("/exit", "/quit", "q"):
                    break
                payload = {
                    "sender": self.node_id,
                    "device": self.device_name,
                    "text": msg,
                    "timestamp": time.time()
                }
                self.session.put(chat_topic, json.dumps(payload))
        except (KeyboardInterrupt, EOFError):
            pass
        finally:
            stop_chat.set()
            try:
                sub.undeclare()
            except Exception:
                pass
            print("\nSohbet modundan çıkıldı.\n")

    def ping_node(self, target_veh: str = "rover_01"):
        print(f"\n🏓 '{target_veh}' Hedefine Ping Atılıyor...")
        ping_key = f"pisim/{target_veh}/ping"
        pong_key = f"pisim/{target_veh}/pong"

        pong_received = threading.Event()
        pong_payload = {"data": None, "rtt_ms": 0.0}
        t_start = time.time()

        def _on_pong(sample):
            pong_payload["rtt_ms"] = (time.time() - t_start) * 1000.0
            pong_payload["data"] = sample.payload.to_string()
            pong_received.set()

        sub = self.session.declare_subscriber(pong_key, _on_pong)
        time.sleep(0.1)

        ping_msg = {
            "cmd": "ping",
            "sender": self.node_id,
            "device": self.device_name,
            "ts": t_start,
        }
        self.session.put(ping_key, json.dumps(ping_msg))

        if pong_received.wait(timeout=3.0):
            print(f" 🎉 PONG YANITI ALINDI! (Gecikme: {pong_payload['rtt_ms']:.2f} ms)")
            print(f"    Gelen Yanıt: {pong_payload['data']}")
        else:
            print(f" ⚠️ Zaman aşımı! Hedef '{target_veh}' açık mı?")

        try:
            sub.undeclare()
        except Exception:
            pass

    def run_menu(self):
        while self.running:
            print("\n" + "=" * 60)
            print(f" 🕹️  PISIM ZENOH TEST KONSOLU | Cihaz: [{self.device_name}]")
            print("=" * 60)
            print(" [1] 💬 Canlı Sohbet / String Mesaj Modu (Terminaller Arası)")
            print(" [2] 📋 Aktif Topic'leri Listele (Keşif)")
            print(" [3] 📥 Bir Topic'e Canlı Abone Ol (Canlı Akış)")
            print(" [4] 📤 İstediğin Topic'e Özel Mesaj / Komut Bas")
            print(" [5] 🏓 Hedef Cihaza / Unreal'a Ping At")
            print(" [6] ❌ Çıkış")
            print("-" * 60)

            try:
                choice = input("Seçiminiz [1-6]: ").strip()
            except (KeyboardInterrupt, EOFError):
                break

            if choice == "1":
                self.live_chat_mode()
            elif choice == "2":
                self.list_topics()
            elif choice == "3":
                topics = self.list_topics()
                if topics:
                    sel = input("\nAbone olunacak topic numarasını veya adını girin (Örn: 1 veya pisim/**): ").strip()
                    if sel.isdigit() and 1 <= int(sel) <= len(topics):
                        chosen_topic = topics[int(sel) - 1]
                    else:
                        chosen_topic = sel if sel else "**"
                    self.subscribe_live(chosen_topic)
            elif choice == "4":
                target_topic = input("\nYayınlanacak Topic (Örn: pisim/rover_03/actuators veya test/mesaj): ").strip()
                if not target_topic:
                    target_topic = f"pisim/{self.device_name}/custom"
                msg_body = input("Mesaj Metni / JSON: ").strip()
                if not msg_body:
                    msg_body = f"Selam ben {self.device_name}"
                self.publish_message(target_topic, msg_body)
            elif choice == "5":
                v_code = input("Hedef Cihaz Kodu [varsayılan: rover_01]: ").strip()
                self.ping_node(v_code if v_code else "rover_01")
            elif choice == "6":
                break

        self.running = False
        try:
            self.session.close()
        except Exception:
            pass
        print(f"[{self.device_name}] Kapatıldı.")


def main():
    parser = argparse.ArgumentParser(description="PiSim Zenoh Mesh Interactive Test CLI")
    parser.add_argument("--mode", choices=["peer", "client"], default="peer", help="Zenoh modu (peer / client)")
    parser.add_argument("--router", type=str, default=None, help="Zenoh router adresi")
    args = parser.parse_args()

    tester = ZenohCliTester(mode=args.mode, router_endpoint=args.router)
    tester.run_menu()


if __name__ == "__main__":
    main()
