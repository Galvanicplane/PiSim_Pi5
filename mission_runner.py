#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mission_runner.py - PiSim Çoklu Araç & Sürü Görev İcra Motoru
=============================================================
Bu script, JSON formatında tanımlanan çok adımlı görev planlarını (Waypoints,
Sürü Buluşma/Bekleme Şartları, Süre Beklemeleri) adım adım çalıştırır.

Özellikler:
  - Görev planını Port 7405 üzerinden 1 kez yayınlar (Metro Hattı planı).
  - Her adımda ağa ve yerel SQL'e hafif adım ping'i gönderir.
  - Sürü eşinin belirli bir adıma/koşula gelmesini `wait_vehicle` ile bekler.
  - Ctrl+C veya Acil Durdurma geldiğinde motorları zınk diye durdurur ve iptal anonsu yapar.

Kullanım:
  python3 mission_runner.py --file gorev1.json --target RV01
  python3 mission_runner.py --sample  (Örnek 4 adımlı sürü görevini başlatır)
"""

import argparse
import json
import os
import signal
import sys
import time
from pathlib import Path

from pisim_motion import PiSimMotionController, offset_gps_by_meters
from swarm_context import SwarmContext


def make_live_hud():
    """Terminalde sabit satırda kendini güncelleyen HUD."""
    last_print = [0.0]

    def hud(data):
        now = time.time()
        if isinstance(data, dict) and data.get("type") == "progress":
            if (now - last_print[0]) < 0.08:
                return
            last_print[0] = now
            c_lat = data["cur_lat"]
            c_lon = data["cur_lon"]
            t_lat = data["target_lat"]
            t_lon = data["target_lon"]
            rem_m = data["rem_m"]
            spd = data["speed_mps"]
            yaw = data["yaw"]
            brg = data["target_bearing"]
            eta = data["eta_deg"]
            steer = data["steer_ratio"]
            pct = data["progress_pct"]
            code = data.get("code", "RV01")
            steer_icon = "◄◄" if steer < -0.3 else ("◄" if steer < -0.08 else ("►►" if steer > 0.3 else ("►" if steer > 0.08 else "▲▲")))
            line = (
                f"\r\033[K[{code}] 📍({c_lat:.5f},{c_lon:.5f}) ➔ 🎯({t_lat:.5f},{t_lon:.5f}) | "
                f"📏 {rem_m:5.1f}m (%{pct:3.0f}) | ⚡ {spd:3.1f}m/s | "
                f"🧭 {yaw:3.0f}°➔{brg:3.0f}° (Δ{eta:+3.0f}°) | 🛞 {steer_icon} {steer:+.2f}"
            )
            sys.stdout.write(line)
            sys.stdout.flush()
        elif isinstance(data, str):
            if data.startswith("\r"):
                sys.stdout.write(f"\r\033[K{data.lstrip(chr(13))}")
                sys.stdout.flush()
            else:
                sys.stdout.write(f"\n{data}\n")
                sys.stdout.flush()

    return hud


def create_sample_mission(start_lat: float = 39.9334, start_lon: float = 32.8597, vehicle_code: str = "RV01") -> dict:
    """Örnek 4 adımlı sürü görev planı üretir (Mevcut konumdan bağıl)."""
    wp1_lat, wp1_lon = offset_gps_by_meters(start_lat, start_lon, 20.0, 10.0)
    wp2_lat, wp2_lon = offset_gps_by_meters(start_lat, start_lon, 40.0, 25.0)

    return {
        "mission_id": f"SURU_PLAN_{int(time.time()) % 10000}",
        "vehicle_code": vehicle_code,
        "title": "4 Adımlı Taktik Sürü Devriye & Buluşma Planı",
        "steps": [
            {
                "action": "goto",
                "title": "1. Durak: Kavşak Alpha",
                "lat": round(wp1_lat, 6),
                "lon": round(wp1_lon, 6),
                "speed": 1.8,
                "turn_radius": 5.0,
                "wp_radius": 1.5,
            },
            {
                "action": "wait_vehicle",
                "title": "2. Durak: RV02 Buluşma Noktası",
                "target_vehicle": "RV02",
                "condition": "at_step",
                "step": 1,
                "timeout_sec": 40.0,
            },
            {
                "action": "wait_time",
                "title": "3. Durak: 5sn Çevre Gözlemi",
                "duration_sec": 5.0,
            },
            {
                "action": "goto",
                "title": "4. Durak: Tepe Noktası Bravo",
                "lat": round(wp2_lat, 6),
                "lon": round(wp2_lon, 6),
                "speed": 2.0,
                "turn_radius": 5.0,
                "wp_radius": 1.5,
            },
        ],
    }


def main():
    parser = argparse.ArgumentParser(description="PiSim Sürü & Görev İcra Motoru (mission_runner.py)")
    parser.add_argument("--file", "-f", type=str, default=None, help="Yürütülecek görev JSON dosya yolu")
    parser.add_argument("--target", "-t", type=str, default="RV01", help="Bu aracın kodu (örn: 'RV01', 'RV02')")
    parser.add_argument("--sample", action="store_true", help="Örnek 4 duraklı sürü görevini başlat")
    parser.add_argument("--ip", type=str, default=None, help="Unreal Engine IP adresi")
    args = parser.parse_args()

    v_code = str(args.target).upper()
    print("=" * 80)
    print(f" 🚀 PISIM MERKEZİYETSİZ GÖREV MOTORU (Araç: {v_code})")
    print("=" * 80)

    # 1. Sanal Merkez Soyutlama ve Hareket Motorunu Başlat
    swarm = SwarmContext(my_vehicle_code=v_code)
    motion = PiSimMotionController(target_vehicle=v_code, ue5_ip=args.ip)
    live_hud = make_live_hud()

    # Acil durdurma (SIGINT / Ctrl+C)
    def handle_abort(signum, frame):
        print("\n\n🛑 [ACİL DURDURMA TETİKLENDİ!] Motorlar derhal durduruluyor...")
        motion.stop()
        swarm.abort_mission("Kullanıcı Ctrl+C ile iptal etti")
        time.sleep(0.3)
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_abort)
    signal.signal(signal.SIGTERM, handle_abort)

    # 2. Görev Planını Yükle
    mission_plan = None
    if args.file and os.path.exists(args.file):
        with open(args.file, "r", encoding="utf-8") as f:
            mission_plan = json.load(f)
        print(f"📂 Görev Dosyası Yüklendi: {args.file}")
    elif args.sample or not args.file:
        telem = motion.wait_for_telemetry(timeout_sec=5.0)
        start_lat = telem["lat"] if telem else 39.9334
        start_lon = telem["lon"] if telem else 32.8597
        mission_plan = create_sample_mission(start_lat, start_lon, v_code)
        print("💡 Örnek 4 Adımlı Taktik Sürü Planı Hazırlandı.")

    steps = mission_plan.get("steps", [])
    total_steps = len(steps)
    if total_steps == 0:
        print("❌ HATA: Görev planında hiçbir adım bulunamadı!")
        sys.exit(1)

    # 3. Planı Port 7405 Üzerinden 1 KEZ Yayınla ve Yerel SQL'e Kaydet
    swarm.broadcast_mission_plan(mission_plan)

    print("\n🚇 GÖREV PLANI METRO DURAKLARI:")
    for i, st in enumerate(steps):
        act = st.get("action", "").upper()
        title = st.get("title", f"Durak {i+1}")
        print(f"   [{i+1}/{total_steps}] ━━━● {title} ({act})")
    print("-" * 80)

    # 4. Adım Adım İcra Döngüsü
    t_start = time.time()
    for idx, step in enumerate(steps):
        if swarm.is_abort_requested():
            print("\n🛑 Görev durdurma emri nedeniyle iptal edildi.")
            break

        act = str(step.get("action", "")).lower()
        title = step.get("title", f"Adım {idx+1}")
        print(f"\n🚇 [DURAK {idx+1}/{total_steps}: {act.upper()}] {title}")
        print("━" * 60)

        # Ağa ve SQL'e 'Bu durağa başladım' ping'i at
        swarm.report_step(idx, state="ACTIVE", status_msg=f"Başlandı: {title}")

        success = False

        if act == "goto":
            lat = float(step["lat"])
            lon = float(step["lon"])
            spd = float(step.get("speed", 1.8))
            tr = float(step.get("turn_radius", 5.0))
            wp_r = float(step.get("wp_radius", 1.5))

            res = motion.navigate_to_gps(
                target_lat=lat,
                target_lon=lon,
                cruise_speed=spd,
                turn_radius_m=tr,
                wp_radius_m=wp_r,
                status_callback=live_hud,
            )
            success = res.get("ok", False)

        elif act == "wait_vehicle":
            target_veh = step.get("target_vehicle", "RV02")
            cond = step.get("condition", "at_step")
            target_step = int(step.get("step", 1))
            t_out = float(step.get("timeout_sec", 60.0))

            success = swarm.wait_for_vehicle(
                target_vehicle_code=target_veh,
                condition=cond,
                step=target_step,
                timeout_sec=t_out,
                status_callback=print,
            )

        elif act == "wait_time":
            dur = float(step.get("duration_sec", 5.0))
            print(f"⏱️ {dur:.1f} saniye bekleniyor...")
            t_w = time.time()
            while (time.time() - t_w) < dur:
                if swarm.is_abort_requested():
                    break
                time.sleep(0.2)
            success = not swarm.is_abort_requested()

        elif act == "forward":
            dist = float(step.get("distance_m", 10.0))
            spd = float(step.get("speed", 1.6))
            res = motion.move_forward(distance_m=dist, speed_mps=spd, status_callback=live_hud)
            success = res.get("ok", False)

        elif act == "curve":
            r = float(step.get("turn_radius", 5.0))
            ang = float(step.get("arc_angle", 90.0))
            d = str(step.get("direction", "right"))
            spd = float(step.get("speed", 1.6))
            res = motion.drive_curve(turn_radius_m=r, arc_angle_deg=ang, speed_mps=spd, direction=d, status_callback=live_hud)
            success = res.get("ok", False)

        else:
            print(f"⚠️ Bilinmeyen eylem tipi: '{act}', adım atlanıyor.")
            success = True

        if success:
            swarm.report_step(idx, state="PASSED", status_msg=f"Tamamlandı: {title}")
            print(f"\n✅ [DURAK {idx+1} GEÇİLDİ] {title}")
        else:
            swarm.report_step(idx, state="FAILED", status_msg=f"Hata: {title}")
            print(f"\n❌ [DURAK {idx+1} BAŞARISIZ OLDU] Görev durduruluyor.")
            break

    # 5. Görev Sonu
    motion.stop()
    elapsed = time.time() - t_start
    if not swarm.is_abort_requested():
        swarm.report_step(total_steps - 1, state="COMPLETED", status_msg="Görev başarıyla tamamlandı")
        print("\n" + "=" * 80)
        print(f" 🎉 [GÖREV TAMAMLANDI] Toplam Süre: {elapsed:.1f} saniye | Araç: {v_code}")
        print("=" * 80)
    else:
        print("\n" + "=" * 80)
        print(f" 🛑 [GÖREV İPTAL EDİLDİ] Araç: {v_code} güvenli duruşa geçti.")
        print("=" * 80)

    swarm.close()


if __name__ == "__main__":
    main()

