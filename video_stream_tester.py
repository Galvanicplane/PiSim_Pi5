#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
video_stream_tester.py - PiSim Zenoh Video Yayını ve Canlı İzleme Aracı
========================================================================
Bu bağımsız araç, hem Unreal Engine kamera akışını test etmek, hem de yerel bir
video dosyasını / web kamerasını ağdaki diğer cihazlara (Docker, Pi 5, Windows)
Zenoh topic'i üzerinden gerçek zamanlı (`sensor_msgs/msg/CompressedImage`) olarak
yayınlamak ve canlı izlemek için geliştirilmiştir.

Temel Özellikler:
1. Yayıncı Modu (--publish):
   - Bilgisayardaki mevcut video dosyasını oynatıp Zenoh'a JPEG akışı olarak basar (--file <video.mp4>).
   - Video bittiğinde otomatik olarak başa sarar (döngüsel yayın).
   - Web kamerası (--camera <id>) veya sentetik test radarı/HUD deseni üretebilir.
   - Ayarlanabilir FPS (--fps) ve JPEG kalitesi (--quality).

2. Canlı İzleyici Modu (--view):
   - Belirtilen Zenoh video topic'ine abone olur (örn: pisim/rover_01/camera/compressed).
   - Gelen JPEG karelerini anlık olarak çözer (OpenCV ile) ve ekranda canlı pencere açar.
   - Canlı telemetri HUD'ı: FPS, bant genişliği (KB/s, Mbps), çözünürlük, kare sayısı.
   - Headless mod (--headless): X11/ekranı olmayan Pi 5 veya Docker içinde istatistiksel izleme.
   - Kısayollar: 'q' / ESC ile çıkış, 's' ile ekran görüntüsü kaydetme, 'h' ile HUD aç/kapa.

Kullanım Örnekleri:
  # 1. Bilgisayardaki bir video dosyasını rover_01 kamerası olarak yayınla:
  python video_stream_tester.py --publish --file "C:/Videos/sample.mp4" --topic pisim/rover_01/camera/compressed

  # 2. Sentetik test deseni ile yayın başlat:
  python video_stream_tester.py --publish --topic pisim/rover_01/camera/compressed

  # 3. Başka bir terminalde veya cihazda canlı yayını izle:
  python video_stream_tester.py --view --topic pisim/rover_01/camera/compressed

  # 4. İzole Docker (Rover 03) veya uzak router üzerinden bağlanarak izle:
  python video_stream_tester.py --view --router tcp/127.0.0.1:7447 --topic pisim/rover_01/camera/compressed
"""

import argparse
import base64
import json
import math
import os
import sys
import time
import threading
from typing import Optional

# Windows terminal Türkçe kod sayfası (cp1254/cp437) desteği
if sys.stdout and hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

try:
    import cv2
    import numpy as np
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

try:
    import zenoh
    HAS_ZENOH = True
except ImportError:
    HAS_ZENOH = False


def normalize_zenoh_endpoint(ep: str, default_port: int = 7447) -> str:
    if not ep:
        return ep
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


def create_zenoh_session(mode: str = "peer", router_endpoint: Optional[str] = None, listen_endpoint: Optional[str] = None):
    """Zenoh yapılandırmasını kurar ve oturumu açar."""
    if not HAS_ZENOH:
        print("❌ 'eclipse-zenoh' modülü bulunamadı! Lütfen 'pip install eclipse-zenoh' kurun.")
        sys.exit(1)

    conf = zenoh.Config()

    env_router = os.environ.get("ZENOH_ROUTER", None)
    if not router_endpoint and env_router:
        router_endpoint = env_router
        mode = "client"

    # Eğer açıkça router belirtilmemişse 7447 portunu kontrol et
    if not router_endpoint:
        try:
            import socket
            s_test = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s_test.settimeout(0.3)
            if s_test.connect_ex(('127.0.0.1', 7447)) == 0:
                router_endpoint = "tcp/127.0.0.1:7447"
                mode = "client"
                print("🔌 Aktif Zenoh Yönlendirici (Router) Bulundu (tcp/127.0.0.1:7447). Otomatik bağlanılıyor...")
            s_test.close()
        except Exception:
            pass

    env_listen = os.environ.get("ZENOH_LISTEN_ENDPOINT", None)
    if not listen_endpoint and env_listen:
        listen_endpoint = env_listen

    if listen_endpoint:
        listen_endpoint = normalize_zenoh_endpoint(listen_endpoint)
        conf.insert_json5("listen/endpoints", json.dumps([listen_endpoint]))
        print(f"📡 Zenoh Dinleme Noktası: {listen_endpoint}")

    if router_endpoint:
        router_endpoint = normalize_zenoh_endpoint(router_endpoint)
        if not listen_endpoint:
            conf.insert_json5("mode", json.dumps("client"))
        conf.insert_json5("connect/endpoints", json.dumps([router_endpoint]))
        print(f"🔗 Zenoh Router'a Bağlanılıyor: {router_endpoint}")
    else:
        conf.insert_json5("mode", json.dumps("peer"))
        print("📡 Zenoh P2P Mesh Modu Açılıyor...")

    session = zenoh.open(conf)
    print("✅ Zenoh Oturumu Başarıyla Açıldı.")
    return session


def generate_synthetic_frame(width: int, height: int, frame_idx: int) -> np.ndarray:
    """Video veya kamera bulunamadığında dinamik test deseni ve radar HUD üretir."""
    frame = np.zeros((height, width, 3), dtype=np.uint8)

    # Arka plan ızgarası (Grid)
    grid_size = 40
    frame[::grid_size, :] = (25, 25, 25)
    frame[:, ::grid_size] = (25, 25, 25)

    center_x = width // 2
    center_y = height // 2

    # Dönen Radar Dairesi
    max_radius = min(center_x, center_y) - 50
    for r in range(50, max_radius + 1, 50):
        cv2.circle(frame, (center_x, center_y), r, (0, 80, 0), 1)

    # Dönen Radar Tarama Çizgisi
    angle_rad = (frame_idx * 4 % 360) * (math.pi / 180.0)
    end_x = int(center_x + max_radius * math.cos(angle_rad))
    end_y = int(center_y + max_radius * math.sin(angle_rad))
    cv2.line(frame, (center_x, center_y), (end_x, end_y), (0, 255, 120), 2)

    # Sahte yapay hedefler (Blips)
    t = time.time()
    for i in range(4):
        bx = int(center_x + (max_radius * 0.6) * math.cos(angle_rad * 0.5 + i * 1.5))
        by = int(center_y + (max_radius * 0.6) * math.sin(angle_rad * 0.5 + i * 1.5))
        cv2.circle(frame, (bx, by), 5, (0, 0, 255), -1)
        cv2.putText(frame, f"TGT-{i+1}", (bx + 8, by - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)

    # Bilgi ve Zaman Başlığı
    timestr = time.strftime("%Y-%m-%d %H:%M:%S")
    cv2.rectangle(frame, (10, 10), (width - 10, 50), (15, 15, 15), -1)
    cv2.putText(frame, f"PISIM SYNTHETIC RADAR FEED | {timestr}", (20, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 200), 2)

    # Alt Bilgi Bandı
    cv2.putText(frame, f"FRAME: #{frame_idx} | RES: {width}x{height}", (20, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
    return frame


def run_publisher(args):
    """Video / Kamera veya Sentetik Kaynaktan Zenoh Topic'ine Canlı Yayın Basar."""
    if not HAS_CV2:
        print("❌ 'opencv-python' modülü bulunamadı! 'pip install opencv-python' kurun.")
        sys.exit(1)

    session = create_zenoh_session(mode=args.mode, router_endpoint=args.router, listen_endpoint=args.listen)

    cap = None
    use_synthetic = False

    if args.file:
        if not os.path.exists(args.file):
            print(f"❌ Belirtilen video dosyası bulunamadı: {args.file}")
            sys.exit(1)
        cap = cv2.VideoCapture(args.file)
        print(f"🎬 Video Dosyası Açıldı: {args.file}")
    elif args.camera is not None:
        cap = cv2.VideoCapture(args.camera)
        print(f"📷 Web Kamerası Açıldı (Cihaz #{args.camera})")
    else:
        print("ℹ️ Video dosyası veya kamera seçilmedi. Sentetik PiSim Radar Deseni yayınlanacak.")
        use_synthetic = True

    if cap and not cap.isOpened():
        print("⚠️ Video/Kamera kaynağı açılamadı! Sentetik Radar desenine geçiliyor...")
        use_synthetic = True
        cap = None

    target_topic = args.topic
    print(f"🚀 Canlı Video Akışı Başlatılıyor -> [Zenoh Topic: {target_topic}]")
    print(f"⚡ Ayarlar: FPS={args.fps}, JPEG Kalitesi={args.quality}%, Hedef Boyut={args.width}x{args.height}")
    print("Durdurmak için Ctrl+C tuşlayın.\n")

    frame_interval = 1.0 / max(1, args.fps)
    frame_idx = 0
    total_bytes_sent = 0
    start_time = time.time()
    last_stat_time = time.time()
    stat_frames = 0
    stat_bytes = 0

    try:
        while True:
            t0 = time.time()

            if use_synthetic or cap is None:
                frame = generate_synthetic_frame(args.width, args.height, frame_idx)
            else:
                ret, frame = cap.read()
                if not ret:
                    if args.file:
                        # Video bitti, başa sar
                        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        ret, frame = cap.read()
                        if not ret:
                            break
                    else:
                        print("⚠️ Kamera kare okuma başarısız.")
                        time.sleep(0.1)
                        continue

                # Boyutlandırma
                if args.width > 0 and args.height > 0:
                    if frame.shape[1] != args.width or frame.shape[0] != args.height:
                        frame = cv2.resize(frame, (args.width, args.height))

            # JPEG Kodlama
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), int(args.quality)]
            success, enc_img = cv2.imencode('.jpg', frame, encode_param)
            if not success:
                continue

            jpeg_bytes = enc_img.tobytes()

            # Zenoh Topic'ine Ham JPEG Bytes Bas
            session.put(target_topic, jpeg_bytes)

            frame_idx += 1
            stat_frames += 1
            payload_len = len(jpeg_bytes)
            stat_bytes += payload_len
            total_bytes_sent += payload_len

            # İstatistikleri Her Saniye Konsola Bas
            now = time.time()
            if now - last_stat_time >= 1.0:
                elapsed = now - last_stat_time
                curr_fps = stat_frames / elapsed
                kb_rate = (stat_bytes / 1024.0) / elapsed
                mbps = (stat_bytes * 8.0 / 1_000_000.0) / elapsed
                print(f"📡 [YAYIN] Kare: #{frame_idx:<6} | FPS: {curr_fps:>4.1f} | Boyut: {payload_len // 1024:>3} KB | Hız: {kb_rate:>6.1f} KB/s ({mbps:>4.2f} Mbps)", end="\r")
                last_stat_time = now
                stat_frames = 0
                stat_bytes = 0

            # FPS Hız Sınırlaması
            elapsed = time.time() - t0
            sleep_time = frame_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n🛑 Kullanıcı tarafından durduruldu.")
    finally:
        if cap:
            cap.release()
        session.close()
        print("✅ Yayıncı kapatıldı.")


class VideoViewerState:
    def __init__(self):
        self.latest_frame: Optional[np.ndarray] = None
        self.lock = threading.Lock()
        self.frame_count = 0
        self.total_bytes = 0
        self.last_ts = time.time()
        self.fps = 0.0
        self.bitrate_kb = 0.0
        self.running = True
        self.source_topic = ""
        self.show_hud = True
        self.stat_frames = 0
        self.stat_bytes = 0
        self.last_stat_time = time.time()


import http.server
import socketserver


def start_web_server(port: int, state: VideoViewerState):
    """Gelen video karelerini tarayıcıdan (Chrome/Safari/Edge) izlemek için MJPEG HTTP sunucusu açar."""
    class MJPEGHandler(http.server.BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            pass

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.end_headers()
                html = f"""<!DOCTYPE html>
<html lang="tr">
<head>
    <meta charset="utf-8">
    <title>PiSim Canlı Video Akışı</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {{ background: #0c1017; color: #e6edf3; font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; text-align: center; margin: 0; padding: 24px; }}
        h1 {{ font-size: 1.6rem; color: #00ffaa; margin: 0 0 8px 0; text-shadow: 0 0 20px rgba(0,255,170,0.3); }}
        .meta {{ color: #8b949e; font-size: 0.9rem; margin-bottom: 20px; }}
        .badge {{ background: #00ffaa22; color: #00ffaa; border: 1px solid #00ffaa44; padding: 4px 12px; border-radius: 20px; font-weight: bold; font-size: 0.8rem; display: inline-block; }}
        .video-wrapper {{ display: inline-block; padding: 10px; background: #161b22; border: 2px solid #30363d; border-radius: 12px; box-shadow: 0 12px 40px rgba(0,0,0,0.7); max-width: 95%; }}
        img {{ max-width: 100%; height: auto; border-radius: 8px; display: block; }}
        .footer {{ margin-top: 16px; color: #6e7681; font-size: 0.8rem; }}
    </style>
</head>
<body>
    <h1>🎬 PiSim Canlı Kamera Akışı</h1>
    <div class="meta">
        <span class="badge">● CANLI YAYIN</span> &nbsp; Topic: <code>{state.source_topic or "pisim/rover_01/camera/compressed"}</code>
    </div>
    <div class="video-wrapper">
        <img src="/stream.mjpg" alt="Canlı Kamera Akışı" />
    </div>
    <div class="footer">Docker ve Dağınık Ağ Üzerinden Doğrudan Tarayıcıya Aktarılmaktadır</div>
</body>
</html>"""
                self.wfile.write(html.encode('utf-8'))

            elif self.path == '/stream.mjpg':
                self.send_response(200)
                self.send_header('Age', '0')
                self.send_header('Cache-Control', 'no-cache, private')
                self.send_header('Pragma', 'no-cache')
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
                self.end_headers()
                try:
                    while state.running:
                        jpeg_bytes = None
                        with state.lock:
                            if state.latest_frame is not None:
                                _, enc = cv2.imencode('.jpg', state.latest_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                                jpeg_bytes = enc.tobytes()
                        if jpeg_bytes:
                            self.wfile.write(b'--FRAME\r\n')
                            self.send_header('Content-Type', 'image/jpeg')
                            self.send_header('Content-Length', str(len(jpeg_bytes)))
                            self.end_headers()
                            self.wfile.write(jpeg_bytes)
                            self.wfile.write(b'\r\n')
                        time.sleep(0.04) # ~25 FPS
                except Exception:
                    pass

            elif self.path == '/snapshot.jpg':
                jpeg_bytes = None
                with state.lock:
                    if state.latest_frame is not None:
                        _, enc = cv2.imencode('.jpg', state.latest_frame)
                        jpeg_bytes = enc.tobytes()
                if jpeg_bytes:
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(jpeg_bytes)))
                    self.end_headers()
                    self.wfile.write(jpeg_bytes)
                else:
                    self.send_response(404)
                    self.end_headers()

    class ThreadedHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
        daemon_threads = True

    try:
        server = ThreadedHTTPServer(('0.0.0.0', port), MJPEGHandler)
        print(f"\n🌐 [CANLI WEB YAYINI AÇILDI] Tarayıcınızdan izlemek için açın:")
        print(f"   👉 http://localhost:{port} (veya http://<cihaz_ip>:{port})\n")
        t = threading.Thread(target=server.serve_forever, daemon=True)
        t.start()
    except Exception as e:
        print(f"⚠️ Web sunucusu başlatılamadı (Port {port}): {e}")


def run_viewer(args):
    """Zenoh Topic'inden Gelen Canlı Video Akışını Dinler ve Ekranda Açar."""
    if not HAS_CV2:
        print("❌ 'opencv-python' modülü bulunamadı! 'pip install opencv-python' kurun.")
        sys.exit(1)

    session = create_zenoh_session(mode=args.mode, router_endpoint=args.router, listen_endpoint=args.listen)
    state = VideoViewerState()

    def on_video_sample(sample):
        try:
            raw_payload = sample.payload.to_bytes()
            if not raw_payload:
                return

            frame = None
            # Durum 1: Ham JPEG Binary Akışı
            if raw_payload.startswith(b'\xff\xd8'):
                nparr = np.frombuffer(raw_payload, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            # Durum 2: JSON Zarfı (Base64 JPEG)
            elif raw_payload.startswith(b'{'):
                try:
                    data_obj = json.loads(raw_payload.decode('utf-8'))
                    if "data" in data_obj:
                        img_b64 = data_obj["data"]
                        img_raw = base64.b64decode(img_b64)
                        nparr = np.frombuffer(img_raw, np.uint8)
                        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                except Exception:
                    pass

            # Durum 3: Diğer formatlar
            if frame is None:
                nparr = np.frombuffer(raw_payload, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if frame is not None:
                with state.lock:
                    state.latest_frame = frame
                    state.frame_count += 1
                    state.stat_frames += 1
                    state.total_bytes += len(raw_payload)
                    state.stat_bytes += len(raw_payload)
                    state.source_topic = str(sample.key_expr)

                    now = time.time()
                    dt = now - state.last_stat_time
                    if dt >= 1.0:
                        state.fps = state.stat_frames / dt
                        state.bitrate_kb = (state.stat_bytes / 1024.0) / dt
                        state.stat_frames = 0
                        state.stat_bytes = 0
                        state.last_stat_time = now

        except Exception as e:
            pass

    target_topic = args.topic
    print(f"👂 Canlı Video Aboneliği Açılıyor -> [Zenoh Topic: {target_topic}]")
    sub = session.declare_subscriber(target_topic, on_video_sample)

    if args.web:
        start_web_server(args.web, state)

    window_name = f"PiSim Live Feed: {target_topic}"
    if not args.headless:
        try:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, 960, 540)
            print("📺 Canlı Görüntü Penceresi Açıldı. (Çıkış: 'q' / 'ESC', Fotoğraf: 's', HUD Aç/Kapa: 'h')\n")
        except Exception as e:
            print(f"⚠️ GUI Ekranı bulunamadı ({e}). Headless konsol izleme moduna geçiliyor...\n")
            args.headless = True
    else:
        print("💻 Headless Mod Aktif. Konsol üzerinden telemetri izleniyor...\n")

    last_wait_print = time.time()
    last_save_time = 0.0

    try:
        while state.running:
            frame_to_show = None
            with state.lock:
                if state.latest_frame is not None:
                    frame_to_show = state.latest_frame.copy()

            if frame_to_show is not None:
                h, w = frame_to_show.shape[:2]

                if not args.headless:
                    # Canlı HUD Bilgi Katmanı Çiz
                    if state.show_hud:
                        # Üst Bilgi Şeridi
                        cv2.rectangle(frame_to_show, (10, 10), (w - 10, 45), (15, 15, 15), -1)
                        hud_text = f"TOPIC: {state.source_topic} | FPS: {state.fps:4.1f} | BITRATE: {state.bitrate_kb:5.1f} KB/s | RES: {w}x{h}"
                        cv2.putText(frame_to_show, hud_text, (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 180), 2)

                        # Alt Kısayol Şeridi
                        cv2.putText(frame_to_show, "[s] Ekran Goruntusu | [h] HUD Ac/Kapa | [q] Cikis", (20, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

                    try:
                        cv2.imshow(window_name, frame_to_show)

                        key = cv2.waitKey(1) & 0xFF
                        if key in (27, ord('q'), ord('Q')):
                            print("\n🛑 İzleme durduruldu.")
                            break
                        elif key in (ord('s'), ord('S')):
                            snap_name = f"screenshot_{int(time.time())}.jpg"
                            cv2.imwrite(snap_name, state.latest_frame)
                            print(f"\n📸 Ekran görüntüsü kaydedildi -> {snap_name}")
                        elif key in (ord('h'), ord('H')):
                            state.show_hud = not state.show_hud
                    except Exception:
                        args.headless = True
                else:
                    # Headless konsol çıktısı (anında ekrana bas)
                    mbps = (state.bitrate_kb * 8.0) / 1000.0
                    sys.stdout.write(f"\r📺 [IZLEME] Topic: {state.source_topic} | Kare: #{state.frame_count:<6} | FPS: {state.fps:>4.1f} | {state.bitrate_kb:>6.1f} KB/s ({mbps:>4.2f} Mbps) ")
                    sys.stdout.flush()

                    # Her 2 saniyede bir son kareyi diske de yaz (görsel doğrulama için)
                    now_t = time.time()
                    if now_t - last_save_time >= 2.0:
                        try:
                            cv2.imwrite("/tmp/live_frame.jpg", frame_to_show)
                        except Exception:
                            pass
                        last_save_time = now_t

                    time.sleep(0.04)
            else:
                if not args.headless:
                    # Kare bekleniyor göstergesi
                    blank = np.zeros((400, 700, 3), dtype=np.uint8)
                    timestr = time.strftime("%H:%M:%S")
                    cv2.putText(blank, f"VIDEO YAYINI BEKLENIYOR... ({timestr})", (50, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 200, 255), 2)
                    cv2.putText(blank, f"Hedef Topic: {target_topic}", (50, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1)
                    cv2.putText(blank, "Iptal etmek icin pencerede 'q' basin.", (50, 340), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 120, 120), 1)
                    cv2.imshow(window_name, blank)
                    key = cv2.waitKey(100) & 0xFF
                    if key in (27, ord('q'), ord('Q')):
                        break
                else:
                    now_w = time.time()
                    if now_w - last_wait_print >= 2.0:
                        timestr = time.strftime("%H:%M:%S")
                        sys.stdout.write(f"\r⏳ [{timestr}] Video karesi bekleniyor... [Hedef: {target_topic}] ")
                        sys.stdout.flush()
                        last_wait_print = now_w
                    time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n🛑 Kullanıcı tarafından durduruldu.")
    finally:
        if not args.headless:
            cv2.destroyAllWindows()
        sub.undeclare()
        session.close()
        print("✅ İzleyici kapatıldı.")


def main():
    parser = argparse.ArgumentParser(
        description="PiSim Zenoh Video Yayını ve Canlı İzleme Aracı",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Örnekler:
  python video_stream_tester.py --publish --file "C:/path/to/my_video.mp4"
  python video_stream_tester.py --publish --camera 0
  python video_stream_tester.py --publish   (Sentetik radar deseni yayınlar)
  python video_stream_tester.py --view --topic pisim/rover_01/camera/compressed
  python video_stream_tester.py --view --headless --topic "pisim/*/camera/compressed"
        """
    )

    action_group = parser.add_mutually_exclusive_group(required=True)
    action_group.add_argument("--publish", "-p", action="store_true", help="Video yayını başlatma modu")
    action_group.add_argument("--view", "-v", action="store_true", help="Canlı video izleme modu")

    # Genel Zenoh Ağ Ayarları
    parser.add_argument("--mode", choices=["peer", "client"], default="peer", help="Zenoh çalışma modu (peer veya client)")
    parser.add_argument("--router", type=str, default=None, help="Bağlanılacak Zenoh Router adresi (örn: tcp/127.0.0.1:7447)")
    parser.add_argument("--listen", type=str, default=None, help="Zenoh dinleme soketi (örn: tcp/0.0.0.0:7447)")
    parser.add_argument("--topic", type=str, default="pisim/rover_01/camera/compressed", help="Yayın veya dinleme yapılacak Zenoh topic'i")

    # Yayıncı Özel Parametreleri
    parser.add_argument("--file", "-f", type=str, default=None, help="Yayınlanacak yerel video dosyasının yolu (.mp4, .avi, .mkv)")
    parser.add_argument("--camera", "-c", type=int, default=None, help="Yayınlanacak kamera donanım indeksi (örn: 0, 1)")
    parser.add_argument("--fps", type=int, default=30, help="Hedef yayın FPS değeri (Varsayılan: 30)")
    parser.add_argument("--quality", "-q", type=int, default=75, help="JPEG sıkıştırma kalitesi 1-100 (Varsayılan: 75)")
    parser.add_argument("--width", type=int, default=854, help="Yayın genişliği (Varsayılan: 854)")
    parser.add_argument("--height", type=int, default=480, help="Yayın yüksekliği (Varsayılan: 480)")

    # İzleyici Özel Parametreleri
    parser.add_argument("--headless", action="store_true", help="Grafik arayüzü olmadan sadece konsol metriklerini izle")
    parser.add_argument("--web", "-w", type=int, nargs="?", const=8088, default=None, help="Canlı video akışını dahili web sunucusuyla tarayıcıya açar (Varsayılan port: 8088)")

    args = parser.parse_args()

    if args.publish:
        run_publisher(args)
    elif args.view:
        run_viewer(args)


if __name__ == "__main__":
    main()
