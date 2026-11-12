#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
zenoh_inspector.py - PiSim Pi 5 & Cross-Platform Zenoh Binary CDR + Video Inspector
===================================================================================
Zenoh ağına bağlanarak istediğiniz herhangi bir topic'e abone olan, mesajların hem
**Binary CDR (Hex/Byte)** hem de **JSON** hallerini canlı gösteren ve topic bir
**kamera/video (`camera/h264`)** yayını ise bunu otomatik tespit edip **`ACK = 3`**
tetiklemesi göndererek canlı video penceresini açan teşhis aracı.

Kullanım Örnekleri:
  1) Ağdaki aktif topic'leri tarayıp menüden seçmek için:
     python zenoh_inspector.py

  2) Belirli bir sensör topic'ini (Binary CDR + JSON) incelemek için:
     python zenoh_inspector.py --topic imu
     python zenoh_inspector.py --topic pisim/*/gps
     python zenoh_inspector.py --topic channels
     python zenoh_inspector.py --topic ack

  3) Kamera yayınını (`camera/h264`) otomatik tespit edip canlı videoyu açmak için:
     python zenoh_inspector.py --topic camera/h264

  4) Kamerayı uyandırmadan (ACK=3 göndermeden) sadece Mavi Ekran (Standby - ACK=4)
     paketlerini incelemek için:
     python zenoh_inspector.py --topic camera/h264 --no-trigger
"""

import argparse
import http.server
import json
import os
import socketserver
import sys
import threading
import time
import webbrowser
from typing import Any, Dict, List, Optional

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR and SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

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

from agent_supervisor import auto_discover_zenoh_endpoint, normalize_zenoh_endpoint
import ros2_cdr

# İsteğe bağlı OpenCV / NumPy desteği (yoksa yerleşik Web Video Oynatıcı devreye girer)
try:
    import cv2
    import numpy as np
    HAS_OPENCV = True
except ImportError:
    HAS_OPENCV = False


class ZenohTopicInspector:
    """
    Zenoh ağındaki herhangi bir topic'e abone olup hem Binary CDR hem JSON içeriğini
    gösteren ve video topic'lerini otomatik tespit edip canlı oynatan sınıf.
    """

    def __init__(
        self,
        topic_expr: Optional[str] = None,
        router_endpoint: Optional[str] = None,
        auto_trigger_video: bool = True,
        video_web_port: int = 8095,
    ):
        self.topic_expr = topic_expr
        self.router_endpoint = normalize_zenoh_endpoint(router_endpoint)
        self.auto_trigger_video = auto_trigger_video
        self.video_web_port = video_web_port

        self.session: Optional[Any] = None
        self.running = False
        self.msg_count = 0
        self.start_time = time.time()
        self.last_rx_time = 0.0
        self.last_print_time = 0.0

        # Video akışı durumu
        self.is_video_topic = False
        self.video_player_started = False
        self.latest_video_frame: bytes = ros2_cdr.get_standby_blue_screen_bytes()
        self.latest_video_meta: Dict[str, Any] = {
            "topic": "",
            "mode": "standby_blue",
            "ack_code": 4,
            "byte_size": len(self.latest_video_frame),
            "fps": 0.0,
            "cdr_hex": "",
        }
        self.video_vehicles: set = set()
        self.frame_timestamps: List[float] = []

    def connect(self) -> bool:
        if not HAS_ZENOH:
            print("❌ 'eclipse-zenoh' kütüphanesi bulunamadı! Lütfen 'pip install eclipse-zenoh' çalıştırın.")
            return False

        if not self.router_endpoint:
            self.router_endpoint = auto_discover_zenoh_endpoint(verbose=True)

        conf = zenoh.Config()
        if self.router_endpoint:
            conf.insert_json5("mode", json.dumps("client"))
            conf.insert_json5("connect/endpoints", json.dumps([self.router_endpoint]))
        else:
            conf.insert_json5("mode", json.dumps("peer"))

        try:
            self.session = zenoh.open(conf)
            return True
        except Exception as e:
            print(f"❌ Zenoh ağına bağlanılamadı: {e}")
            return False

    def discover_topics(self, duration_sec: float = 2.2) -> List[str]:
        """Zenoh ağını kısa süre dinleyerek aktif olan tüm topic anahtarlarını keşfeder."""
        if not self.session:
            return []

        print(f"📡 Zenoh ağındaki aktif topic'ler taranıyor ({duration_sec:.1f} sn)...")
        found = set()

        def _cb(sample):
            found.add(str(sample.key_expr))

        sub_p = self.session.declare_subscriber("pisim/**", _cb)
        sub_f = self.session.declare_subscriber("fleet/**", _cb)
        time.sleep(duration_sec)
        try:
            sub_p.undeclare()
            sub_f.undeclare()
        except Exception:
            pass

        # Eğer kamera standby veya henüz tetiklenmemişse bilinen standart topic şablonlarını da ekle
        vehicles = set()
        for t in list(found):
            parts = t.split("/")
            if len(parts) >= 2 and parts[0] == "pisim":
                vehicles.add(parts[1])
        for v in vehicles:
            found.add(f"pisim/{v}/imu")
            found.add(f"pisim/{v}/gps")
            found.add(f"pisim/{v}/camera/h264")
            found.add(f"pisim/{v}/channels")
            found.add(f"pisim/{v}/ack")

        return sorted(list(found))

    def normalize_topic_filter(self, user_input: str) -> str:
        """Kullanıcının girdiği kısa ifadeyi (örn: 'imu', 'camera/h264', 'RV01/gps') Zenoh key_expr'e çevirir."""
        t = user_input.strip().lstrip("/")
        if not t or t.lower() in ("all", "*"):
            return "pisim/**"
        if t.startswith("pisim/") or t.startswith("fleet/"):
            return t
        if "/" not in t:
            # Sadece sensör adı yazıldıysa (örn: 'imu', 'gps', 'ack', 'channels', 'h264')
            if t.lower() in ("h264", "camera", "video"):
                return "pisim/*/camera/h264"
            return f"pisim/*/{t}"
        if t.lower() in ("camera/h264", "camera/compressed"):
            return f"pisim/*/{t}"
        return f"pisim/{t}"

    def _send_ack_to_vehicle(self, v_code: str, topic_name: str, ack_code: int):
        """Belirtilen araca Binary CDR formatında ACK / Trigger mesajı gönderir."""
        if not self.session or not v_code:
            return
        ack_map = {topic_name: int(ack_code)}
        cdr_ack = ros2_cdr.serialize_ack_cdr(v_code, ack_map, source="pi5_inspector")
        try:
            self.session.put(f"pisim/{v_code}/ack", cdr_ack)
        except Exception:
            pass

    def _video_trigger_heartbeat_loop(self):
        """
        Video topic'i izlenirken her 1.0 saniyede bir ACK=3 (Canlı H.264 Aç) gönderir.
        Böylece kameranın 4 saniyelik watchdog zaman aşımı sıfırlanır ve canlı yayın kesilmez.
        """
        while self.running:
            time.sleep(1.0)
            if self.is_video_topic and self.auto_trigger_video:
                for v_code in list(self.video_vehicles):
                    self._send_ack_to_vehicle(v_code, f"/{v_code}/camera/h264", ros2_cdr.ACK_TRIGGER_STREAM)

    def _start_video_viewer(self):
        """Video tespit edildiğinde canlı görüntü penceresini (OpenCV veya Web Oynatıcı) başlatır."""
        if self.video_player_started:
            return
        self.video_player_started = True

        # Her durumda gömülü HTTP MJPEG/HTML5 canlı video sunucusunu başlat (Tarayıcı ve SSH uyumlu)
        inspector_ref = self

        class VideoHandler(http.server.BaseHTTPRequestHandler):
            def log_message(self, format, *args):
                pass  # Konsolu HTTP loglarıyla kirletme

            def do_GET(self):
                if self.path.startswith("/frame.jpg"):
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
                    self.end_headers()
                    self.wfile.write(inspector_ref.latest_video_frame)
                elif self.path.startswith("/meta.json"):
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(json.dumps(inspector_ref.latest_video_meta).encode("utf-8"))
                elif self.path.startswith("/trigger"):
                    code = 3 if "code=3" in self.path else 4
                    inspector_ref.auto_trigger_video = (code == 3)
                    for vc in list(inspector_ref.video_vehicles):
                        inspector_ref._send_ack_to_vehicle(vc, f"/{vc}/camera/h264", code)
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(json.dumps({"status": "OK", "ack_sent": code}).encode("utf-8"))
                else:
                    html = """<!DOCTYPE html>
<html lang="tr">
<head>
<meta charset="utf-8">
<title>PiSim Zenoh H.264 & Binary CDR Video Inspector</title>
<style>
  body { background:#080c11; color:#e6edf3; font-family:Consolas,monospace; margin:0; padding:18px; display:flex; flex-direction:column; align-items:center; }
  .box { border:1px solid #00dfb6; background:#0d131a; padding:14px; border-radius:6px; max-width:760px; width:100%; box-shadow:0 0 25px rgba(0,223,182,0.18); }
  h2 { margin:0 0 10px 0; font-size:15px; color:#00dfb6; display:flex; justify-content:space-between; }
  .badge { padding:2px 8px; border-radius:4px; font-size:12px; font-weight:bold; }
  .live { background:rgba(0,223,182,0.2); color:#00dfb6; border:1px solid #00dfb6; }
  .standby { background:rgba(10,65,235,0.35); color:#66aaff; border:1px solid #3377ff; }
  img { width:100%; height:380px; object-fit:contain; background:#051535; border:1px solid #1f2937; border-radius:4px; margin:8px 0; }
  .meta { font-size:12px; line-height:1.6; background:#06090d; padding:10px; border-radius:4px; border:1px solid #1f2937; word-break:break-all; }
  .btns { display:flex; gap:10px; margin-top:10px; }
  button { flex:1; padding:9px; font-family:Consolas,monospace; font-weight:bold; cursor:pointer; border-radius:4px; border:1px solid #00dfb6; background:rgba(0,223,182,0.12); color:#00dfb6; }
  button.sleep { border-color:#3377ff; background:rgba(10,65,235,0.2); color:#66aaff; }
</style>
</head>
<body>
  <div class="box">
    <h2>
      <span>📹 PISIM ZENOH H.264 / CDR VIDEO INSPECTOR</span>
      <span id="mode-badge" class="badge standby">MAVİ EKRAN STANDBY (ACK=4)</span>
    </h2>
    <img id="vid" src="/frame.jpg" alt="Zenoh Video Stream" />
    <div class="meta">
      <div><b>Topic:</b> <span id="m-topic">-</span> | <b>FPS:</b> <span id="m-fps">0</span> | <b>Boyut:</b> <span id="m-size">0</span> byte</div>
      <div><b>Binary CDR Hex:</b> <span id="m-hex" style="color:#00e5ff;">-</span></div>
    </div>
    <div class="btns">
      <button onclick="fetch('/trigger?code=3')">🟢 ACK=3 GÖNDER (CANLI H.264 AÇ)</button>
      <button class="sleep" onclick="fetch('/trigger?code=4')">🔵 ACK=4 GÖNDER (MAVİ EKRAN STANDBY'A AL)</button>
    </div>
  </div>
<script>
  setInterval(() => {
    document.getElementById('vid').src = '/frame.jpg?t=' + Date.now();
    fetch('/meta.json').then(r => r.json()).then(m => {
      document.getElementById('m-topic').innerText = m.topic || '-';
      document.getElementById('m-fps').innerText = m.fps || 0;
      document.getElementById('m-size').innerText = m.byte_size || 0;
      document.getElementById('m-hex').innerText = m.cdr_hex || '-';
      const b = document.getElementById('mode-badge');
      if (m.mode === 'live_h264') {
        b.className = 'badge live';
        b.innerText = '🟢 CANLI H.264 AKIŞI (ACK=3)';
      } else {
        b.className = 'badge standby';
        b.innerText = '🔵 MAVİ EKRAN STANDBY (ACK=4)';
      }
    }).catch(()=>{});
  }, 120);
</script>
</body>
</html>"""
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.end_headers()
                    self.wfile.write(html.encode("utf-8"))

        def _serve_http():
            try:
                socketserver.TCPServer.allow_reuse_address = True
                with socketserver.TCPServer(("0.0.0.0", self.video_web_port), VideoHandler) as httpd:
                    httpd.serve_forever()
            except Exception:
                pass

        t_http = threading.Thread(target=_serve_http, daemon=True)
        t_http.start()
        url = f"http://localhost:{self.video_web_port}"
        print(f"\n🎬 [VİDEO TESPİT EDİLDİ] Canlı Video Oynatıcı Başlatıldı -> {url}")

        if HAS_OPENCV and os.environ.get("DISPLAY", "win" if sys.platform.startswith("win") else ""):
            t_cv = threading.Thread(target=self._opencv_window_loop, daemon=True)
            t_cv.start()
        else:
            try:
                webbrowser.open(url)
            except Exception:
                pass

    def _opencv_window_loop(self):
        """OpenCV kuruluysa yerel masaüstü penceresinde canlı H.264 / Mavi Ekran görüntüsünü gösterir."""
        win_name = "PiSim Zenoh Video Inspector (ESC/Q: Cikis | 3: Canli H264 | 4: Mavi Ekran)"
        try:
            while self.running:
                raw = self.latest_video_frame
                if raw:
                    arr = np.frombuffer(raw, dtype=np.uint8)
                    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if img is not None:
                        img_disp = cv2.resize(img, (640, 360), interpolation=cv2.INTER_NEAREST)
                        mode_txt = self.latest_video_meta.get("mode", "standby_blue")
                        color = (0, 225, 180) if mode_txt == "live_h264" else (255, 140, 40)
                        cv2.putText(
                            img_disp,
                            f"Topic: {self.latest_video_meta.get('topic')} | Mode: {mode_txt.upper()}",
                            (12, 26),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55,
                            color,
                            2,
                        )
                        cv2.putText(
                            img_disp,
                            f"Size: {self.latest_video_meta.get('byte_size')} B | FPS: {self.latest_video_meta.get('fps')} | [3]=Canli [4]=Standby",
                            (12, 52),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.48,
                            (230, 230, 230),
                            1,
                        )
                        cv2.imshow(win_name, img_disp)
                k = cv2.waitKey(60) & 0xFF
                if k in (27, ord("q")):
                    break
                elif k == ord("3"):
                    self.auto_trigger_video = True
                    for vc in list(self.video_vehicles):
                        self._send_ack_to_vehicle(vc, f"/{vc}/camera/h264", ros2_cdr.ACK_TRIGGER_STREAM)
                elif k == ord("4"):
                    self.auto_trigger_video = False
                    for vc in list(self.video_vehicles):
                        self._send_ack_to_vehicle(vc, f"/{vc}/camera/h264", ros2_cdr.ACK_STANDBY_SLEEP)
            cv2.destroyAllWindows()
        except Exception:
            webbrowser.open(f"http://localhost:{self.video_web_port}")

    def _on_sample(self, sample):
        key = str(sample.key_expr)
        raw_bytes = sample.payload.to_bytes()
        now = time.time()
        self.msg_count += 1
        self.last_rx_time = now

        # Frekans (Hz) hesabı
        self.frame_timestamps.append(now)
        self.frame_timestamps = [t for t in self.frame_timestamps if (now - t) <= 2.0]
        hz = round(len(self.frame_timestamps) / 2.0, 1)

        parts = key.split("/")
        v_code = parts[1] if len(parts) >= 2 and parts[0] == "pisim" else ""

        # Paket Binary CDR + JSON Analizi
        snap = ros2_cdr.inspect_cdr_packet(key, raw_bytes)

        # Video / Kamera Topic Tespiti
        if "camera" in key.lower() or "h264" in key.lower():
            self.is_video_topic = True
            if v_code:
                self.video_vehicles.add(v_code)
                if self.auto_trigger_video and len(self.video_vehicles) == 1 and self.msg_count <= 2:
                    # İlk yakalandığında hemen ACK=3 (Canlı H.264 Aç) gönder!
                    self._send_ack_to_vehicle(v_code, f"/{v_code}/camera/h264", ros2_cdr.ACK_TRIGGER_STREAM)

            cam_decoded, _ = ros2_cdr.deserialize_from_topic(key, raw_bytes)
            img_bytes = cam_decoded.get("image_bytes", raw_bytes) if isinstance(cam_decoded, dict) else raw_bytes
            cam_mode = cam_decoded.get("mode", "standby_blue") if isinstance(cam_decoded, dict) else "standby_blue"

            self.latest_video_frame = img_bytes
            self.latest_video_meta = {
                "topic": key,
                "mode": cam_mode,
                "ack_code": 3 if cam_mode == "live_h264" else 4,
                "byte_size": len(raw_bytes),
                "fps": hz,
                "cdr_hex": snap["hex_preview"],
            }
            self._start_video_viewer()

        # Diğer sensörler için ACK=2 (Sağlıklı) veya ACK=1 (Hatalı Şema) bildirimi
        elif v_code and (self.msg_count % 15 == 1):
            sub_suffix = "/".join(parts[2:]) if len(parts) >= 3 else parts[-1]
            if sub_suffix != "ack":
                self._send_ack_to_vehicle(v_code, f"/{v_code}/{sub_suffix}", snap["ack_code"])

        # Terminal Çıktısı (Okunabilir hızda: saniyede ~4 kez)
        if (now - self.last_print_time) < 0.25:
            return
        self.last_print_time = now

        print("\n" + "═" * 82)
        print(f"📡 [PI 5 ZENOH INSPECTOR] Topic: {key}  |  Hız: {hz:.1f} Hz  |  Toplam: #{self.msg_count}")
        print(f"├─ 🩺 ACK Durumu     : {snap['ack_label']}")
        print(f"├─ 📦 Binary CDR     : Başlık={snap['cdr_header']} | Boyut={snap['byte_size']} byte")
        print(f"├─ 🧬 CDR Hex Dökümü : {snap['hex_preview']}")
        print(f"└─ 📄 JSON İçeriği   : {json.dumps(snap['json'], ensure_ascii=False)}")
        if self.is_video_topic:
            v_mode = self.latest_video_meta.get("mode", "standby_blue")
            v_tag = "🟢 CANLI H.264 AKIŞI (ACK=3 AKTİF)" if v_mode == "live_h264" else "🔵 MAVİ EKRAN STANDBY (ACK=4)"
            print(f"   🎬 [VİDEO DURUMU] : {v_tag} | İzleme Adresi: http://localhost:{self.video_web_port}")
        print("═" * 82)

    def run(self, max_seconds: Optional[float] = None):
        if not self.connect():
            return

        if not self.topic_expr:
            topics = self.discover_topics(duration_sec=2.0)
            print("\n" + "=" * 72)
            print(" 📋 ZENOH AĞINDA KEŞFEDİLEN TOPIC LİSTESİ")
            print("=" * 72)
            if topics:
                for i, t in enumerate(topics, 1):
                    v_hint = " 📹 [VİDEO / H.264]" if ("camera" in t or "h264" in t) else ""
                    print(f"  [{i:2d}] {t}{v_hint}")
            else:
                print("  ⚠️ Şu an aktif yayın yapan araç bulunamadı (Varsayılan: pisim/** dinlenecek)")
            print("  [ 0] Tüm Topic'leri Dinle (pisim/**)")
            print("=" * 72)

            try:
                choice = input("\n👉 Abone olmak istediğiniz numara veya topic adını girin (Örn: 1 veya camera/h264): ").strip()
            except EOFError:
                choice = "0"

            if choice.isdigit():
                idx = int(choice)
                if 1 <= idx <= len(topics):
                    self.topic_expr = topics[idx - 1]
                else:
                    self.topic_expr = "pisim/**"
            elif choice:
                self.topic_expr = self.normalize_topic_filter(choice)
            else:
                self.topic_expr = "pisim/**"
        else:
            self.topic_expr = self.normalize_topic_filter(self.topic_expr)

        print(f"\n🎯 Abone Olunan Zenoh Topic: [{self.topic_expr}]")
        if "camera" in self.topic_expr.lower() or "h264" in self.topic_expr.lower():
            self.is_video_topic = True
            if self.auto_trigger_video:
                print("🚀 Video Topic Tespit Edildi! Otomatik ACK=3 (Canlı H.264 Trigger) gönderiliyor...")
            else:
                print("🔵 Video Topic (No-Trigger Modu): Kamera Mavi Ekran (Standby - ACK=4) modunda inceleniyor.")

        self.running = True
        sub = self.session.declare_subscriber(self.topic_expr, self._on_sample)

        t_trig = threading.Thread(target=self._video_trigger_heartbeat_loop, daemon=True)
        t_trig.start()

        start_t = time.time()
        try:
            while self.running:
                time.sleep(0.25)
                now = time.time()
                if max_seconds is not None and (now - start_t) >= max_seconds:
                    break
                # Eğer 3 saniye boyunca hiç mesaj gelmediyse ACK=0 (Topic Yok / Bulunamadı) uyarısı bas
                if self.msg_count == 0 and (now - start_t) > 3.0 and int(now - start_t) % 3 == 0:
                    print(f"⏳ [ACK=0: TOPIC_YOK] '{self.topic_expr}' üzerinden henüz veri gelmedi, bekleniyor...")
                    time.sleep(0.8)
        except KeyboardInterrupt:
            print("\n🛑 Inspector durduruluyor...")
        finally:
            self.running = False
            # Eğer canlı video açıldıysa çıkarken ACK=4 göndererek kamerayı tekrar Mavi Ekran (Standby) moduna al!
            if self.is_video_topic and self.auto_trigger_video:
                for vc in list(self.video_vehicles):
                    print(f"🔵 [{vc}] Kameraya ACK=4 (Mavi Ekran Standby) gönderiliyor...")
                    self._send_ack_to_vehicle(vc, f"/{vc}/camera/h264", ros2_cdr.ACK_STANDBY_SLEEP)
                time.sleep(0.15)
            try:
                sub.undeclare()
                self.session.close()
            except Exception:
                pass


def main():
    parser = argparse.ArgumentParser(description="PiSim Pi 5 Zenoh Binary CDR + Video Inspector")
    parser.add_argument(
        "--topic",
        "-t",
        type=str,
        default=None,
        help="Abone olunacak topic (örn: imu, gps, camera/h264, channels, ack, pisim/RV01/imu). Boş bırakılırsa menü açılır.",
    )
    parser.add_argument("--router", "-r", type=str, default=None, help="Zenoh Router adresi (boşsa otomatik keşfedilir)")
    parser.add_argument(
        "--no-trigger",
        action="store_true",
        help="Kamera topic'ine abone olunduğunda ACK=3 göndermez; Mavi Ekran (Standby) yayınını inceler.",
    )
    parser.add_argument(
        "--duration",
        "-d",
        type=float,
        default=None,
        help="Belirtilen saniye kadar dinleyip otomatik çıkar (test amaçlı).",
    )
    parser.add_argument("--web-port", type=int, default=8095, help="Canlı video web oynatıcı portu (varsayılan: 8095)")
    args = parser.parse_args()

    inspector = ZenohTopicInspector(
        topic_expr=args.topic,
        router_endpoint=args.router,
        auto_trigger_video=not args.no_trigger,
        video_web_port=args.web_port,
    )
    inspector.run(max_seconds=args.duration)


if __name__ == "__main__":
    main()
