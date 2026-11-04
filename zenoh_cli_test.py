#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
zenoh_cli_test.py - PiSim Zenoh P2P Dağınık Ağ ve Topic İnteraktif Test Aracı
================================================================================
Bu interaktif CLI aracı, ağdaki Zenoh iletişimini doğrulamak için geliştirilmiştir:
1. Başlatıldığında ağa bir heartbeat duyurusu (`pisim/nodes/<host_id>`) yayınlar.
2. Ağdaki tüm aktif topic'leri (Unreal Engine, Pi5, Docker ajanları) anlık keşfeder.
3. Seçtiğiniz herhangi bir topic'e canlı abone olup gelen verileri ekrana basar.
4. İstediğiniz topic'e özel metin / JSON mesajı basmanızı sağlar.
5. Unreal Engine'e doğrudan 'Ping' atıp 'Pong' cevabı ve gidiş-dönüş gecikmesini (RTT) ölçer.

Kullanım:
  python zenoh_cli_test.py
  python zenoh_cli_test.py --mode peer
  python zenoh_cli_test.py --mode client --router tcp/127.0.0.1:7447
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
        self.node_id = f"tester_{self.hostname}_{os.getpid()}"
        self.running = True

        # Keşfedilen aktif topic'ler ve son mesajları: {topic_str: (last_seen_time, sample_payload)}
        self.discovered_topics: Dict[str, tuple] = {}
        self.topics_lock = threading.Lock()

        # Oturumu Aç
        conf = zenoh.Config()
        if self.mode == "client" and self.router_endpoint:
            conf.insert_json5("mode", json.dumps("client"))
            conf.insert_json5("connect/endpoints", json.dumps([self.router_endpoint]))
        else:
            conf.insert_json5("mode", json.dumps("peer"))

        print(f"📡 Zenoh Oturumu Açılıyor ({self.mode.upper()} modu)...")
        self.session = zenoh.open(conf)
        print("✅ Zenoh Oturumu Başarıyla Açıldı.")

        # Ağdaki tüm topic'leri dinleyen küresel izleme abonesi
        self.wildcard_sub = self.session.declare_subscriber("**", self._on_global_sample)

        # Kendi varlığımızı 2 saniyede bir duyuran iş parçacığı
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
                    "host": self.hostname,
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
                print(" ⏳ Henüz ağda bir veri tespit edilmedi. (Unreal veya ZenohMesh çalışıyor mu?)")
                return []

            topics_sorted = sorted(self.discovered_topics.keys())
            for idx, t in enumerate(topics_sorted, 1):
                last_time, sample_data = self.discovered_topics[t]
                age = now - last_time
                preview = sample_data.replace("\n", " ")[:60]
                status = "🟢 Canlı" if age < 4.0 else f"🟡 {age:.1f}s önce"
                print(f" [{idx:02d}] {status:10s} {t:35s} | Örnek: {preview}...")
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
                # JSON güzelleştir
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

    def ping_unreal(self, vehicle_code: str = "rover_01"):
        print(f"\n🏓 Unreal Engine '{vehicle_code}' TopicBridge Ping Testi...")
        ping_key = f"pisim/{vehicle_code}/ping"
        pong_key = f"pisim/{vehicle_code}/pong"

        pong_received = threading.Event()
        pong_payload = {"data": None, "rtt_ms": 0.0}
        t_start = time.time()

        def _on_pong(sample):
            pong_payload["rtt_ms"] = (time.time() - t_start) * 1000.0
            pong_payload["data"] = sample.payload.to_string()
            pong_received.set()

        sub = self.session.declare_subscriber(pong_key, _on_pong)
        time.sleep(0.1)  # Abonenin ağa duyurulmasını bekle

        ping_msg = {
            "cmd": "ping",
            "sender": self.node_id,
            "ts": t_start,
        }
        self.session.put(ping_key, json.dumps(ping_msg))

        if pong_received.wait(timeout=2.5):
            print(f" 🎉 PONG CEVABI ALINDI! (RTT: {pong_payload['rtt_ms']:.2f} ms)")
            print(f"    Gelen Yanıt: {pong_payload['data']}")
        else:
            print(f" ⚠️ Zaman aşımı! Unreal Engine ({vehicle_code}) veya 'zenoh_mesh.py' açık mı?")

        try:
            sub.undeclare()
        except Exception:
            pass

    def run_menu(self):
        while self.running:
            print("\n" + "=" * 55)
            print(" 🕹️  PISIM ZENOH DAĞITIK AĞ TEST MENÜSÜ")
            print("=" * 55)
            print(" [1] Aktif Topic'leri Listele (Keşif)")
            print(" [2] Bir Topic'e Canlı Abone Ol (Canlı Akış)")
            print(" [3] İstediğin Topic'e Mesaj / Komut Yayınla")
            print(" [4] Unreal Engine Ping / Echo Testi")
            print(" [5] Çıkış")
            print("-" * 55)

            try:
                choice = input("Seçiminiz [1-5]: ").strip()
            except (KeyboardInterrupt, EOFError):
                break

            if choice == "1":
                self.list_topics()
            elif choice == "2":
                topics = self.list_topics()
                if topics:
                    sel = input("\nAbone olmak istediğiniz topic numarasını veya adını girin (Örn: 1 veya pisim/**): ").strip()
                    if sel.isdigit() and 1 <= int(sel) <= len(topics):
                        chosen_topic = topics[int(sel) - 1]
                    else:
                        chosen_topic = sel if sel else "**"
                    self.subscribe_live(chosen_topic)
            elif choice == "3":
                target_topic = input("\nYayınlanacak Topic (Örn: pisim/rover_01/actuators veya test/selam): ").strip()
                if not target_topic:
                    target_topic = "pisim/rover_01/actuators"
                msg_body = input("Yayınlanacak Mesaj / JSON (Örn: {\"M_Steer_FL\": 0.5} veya Selam): ").strip()
                if not msg_body:
                    msg_body = "{\"test\": \"hello_zenoh\"}"
                self.publish_message(target_topic, msg_body)
            elif choice == "4":
                v_code = input("Hedef Araç Kodu [varsayılan: rover_01]: ").strip()
                self.ping_unreal(v_code if v_code else "rover_01")
            elif choice == "5":
                break

        self.running = False
        try:
            self.session.close()
        except Exception:
            pass
        print("Görüşmek üzere!")


def main():
    parser = argparse.ArgumentParser(description="PiSim Zenoh Mesh Interactive Test CLI")
    parser.add_argument("--mode", choices=["peer", "client"], default="peer", help="Zenoh modu (peer / client)")
    parser.add_argument("--router", type=str, default=None, help="Zenoh router adresi")
    args = parser.parse_args()

    tester = ZenohCliTester(mode=args.mode, router_endpoint=args.router)
    tester.run_menu()


if __name__ == "__main__":
    main()
