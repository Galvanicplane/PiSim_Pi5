#!/usr/bin/env python3
"""
view_sql.py - Raspberry Pi 5 Canlı SQL Telemetri Tablosu Görüntüleyici
Doğrudan Pi 5 terminalinde 'python view_sql.py' veya 'python view_sql.py --watch' çalıştırılarak SQLite tablosunu görüntüler.
"""

import sys
import os
import time
from pathlib import Path

# UTF-8 console output
try:
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

import telemetry_db

def render_table(watch_mode=False):
    # En güncel satırları almak için SQL dosyasını anında tara
    sync_res = telemetry_db.sync_sql_data()
    stats = telemetry_db.get_stats()
    rows = telemetry_db.get_recent_rows(20)

    if watch_mode:
        os.system('cls' if os.name == 'nt' else 'clear')

    print("\n" + "=" * 105)
    print("  🛢️  PISIM RASPBERRY PI 5 - CANLI SQL TELEMETRİ VERİTABANI (pisim_telemetry.db)")
    print(f"      Toplam Satır: {stats['total_logged']} | Veritabanı: {stats['db_path']}")
    sql_file = telemetry_db.get_ue5_sql_file_path()
    file_info = f"{sql_file} ({sql_file.stat().st_size:,} bayt)" if sql_file.exists() else "Bulunamadı"
    print(f"      Unreal SQL Dosyası: {file_info}")
    if sync_res.get('new_rows_count', 0) > 0:
        print(f"      📥 Son Senkronizasyonda {sync_res['new_rows_count']} Yeni Satır Eklendi!")
    print("=" * 105)

    if not rows:
        print("\n  ⚠️ Henüz veritabanında kayıtlı telemetri satırı bulunmuyor.")
        print("     Unreal Engine simülasyonu başladığında (Port 7400/SQL Batch) veriler otomatik dolar.\n")
        print("=" * 105 + "\n")
        return

    # Tablo Başlığı
    header = f"| {'ID':^6} | {'KAYIT SAATİ':^19} | {'ZAMAN(s)':^9} | {'KOD':^10} | {'ID':^4} | {'ROLL (°)':^9} | {'PITCH (°)':^9} | {'YAW (°)':^9} | {'İRTİFA':^9} | {'HIZ':^8} | {'PİL':^6} | {'MOD':^10} |"
    sep = "+" + "-"*8 + "+" + "-"*21 + "+" + "-"*11 + "+" + "-"*12 + "+" + "-"*6 + "+" + "-"*11 + "+" + "-"*11 + "+" + "-"*11 + "+" + "-"*11 + "+" + "-"*10 + "+" + "-"*8 + "+" + "-"*12 + "+"

    print(sep)
    print(header)
    print(sep)

    for r in rows:
        dt_str = str(r.get('created_at') or '--')[:19]
        t_str = f"{r['time']:.2f}" if r['time'] is not None else "--"
        roll_str = f"{r['roll']:+.2f}°" if r['roll'] is not None else "0.0°"
        pitch_str = f"{r['pitch']:+.2f}°" if r['pitch'] is not None else "0.0°"
        yaw_str = f"{r['yaw']:.2f}°" if r['yaw'] is not None else "0.0°"
        alt_str = f"{r['altitude']:.1f}m" if r['altitude'] is not None else "0.0m"
        speed_str = f"{r['speed']:.1f}" if r['speed'] is not None else "0.0"
        batt_str = f"{r['battery']:.0f}%" if r['battery'] is not None else "--"
        mode_str = f"{str(r['mode'])[:10]}"
        v_code = str(r.get('vehicle_code') or f"rover_{r['vehicle_id']+1}")

        line = f"| {r['id']:^6} | {dt_str:^19} | {t_str:^9} | {v_code:^10} | {r['vehicle_id']:^4} | {roll_str:^9} | {pitch_str:^9} | {yaw_str:^9} | {alt_str:^9} | {speed_str:^8} | {batt_str:^6} | {mode_str:^10} |"
        print(line)

    print(sep)
    t_now = time.strftime('%H:%M:%S')
    print(f"  ✅ En son {len(rows)} satır listelendi [Güncellendi: {t_now}]. Çıkmak için Ctrl+C.")
    print("=" * 105 + "\n")

def main():
    if "--clear" in sys.argv or "--reset" in sys.argv:
        print("🧹 SQLite veritabanı (pisim_telemetry.db) ve flight_telemetry.sql temizleniyor...")
        telemetry_db.clear_fleet()
        # Unreal Engine Saved dosyasını da temizle (varsa)
        ue5_file = telemetry_db.get_ue5_sql_file_path()
        if ue5_file.exists():
            try:
                with open(ue5_file, "w", encoding="utf-8") as f:
                    f.write("-- PiSim Temizlendi\n")
                print(f"🧹 Unreal SQL dosyası da sıfırlandı: {ue5_file}")
            except Exception as e:
                print(f"⚠️ Unreal SQL dosyası sıfırlanamadı: {e}")
        print("✅ Veritabanı ve SQL dosyaları başarıyla sıfırlandı! Artık 0 satır ile test yapabilirsiniz.\n")
        return

    watch = ("--watch" in sys.argv or "-w" in sys.argv)
    if not watch:
        render_table(watch_mode=False)
        print("💡 İpucu: Sürekli canlı yenileme modunda izlemek için: python view_sql.py --watch")
        print("💡 İpucu: Veritabanını sıfırlamak (0 satır yapmak) için: python view_sql.py --clear\n")
        return

    print("🚀 Canlı SQL İzleme Modu başlatıldı (Her 2 saniyede bir güncellenir)...")
    try:
        while True:
            render_table(watch_mode=True)
            time.sleep(2.0)
    except KeyboardInterrupt:
        print("\n🛑 Canlı izleme sonlandırıldı.")

if __name__ == "__main__":
    main()
