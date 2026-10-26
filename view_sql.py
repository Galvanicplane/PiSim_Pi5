#!/usr/bin/env python3
"""
view_sql.py - Raspberry Pi 5 Canlı SQL Telemetri Tablosu Görüntüleyici
Doğrudan Pi 5 terminalinde 'python view_sql.py' çalıştırılarak SQLite tablosunu görüntüler.
"""

import sys
import time

# UTF-8 console output
try:
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

import telemetry_db

def main():
    stats = telemetry_db.get_stats()
    rows = telemetry_db.get_recent_rows(15)

    print("\n" + "=" * 94)
    print("  🛢️  PISIM RASPBERRY PI 5 - SQL TELEMETRİ VERİTABANI (pisim_telemetry.db)")
    print(f"      Toplam Kayıt: {stats['total_logged']} satır | Veritabanı Dosyası: {stats['db_path']}")
    print(f"      SQL Döküm Dosyası: {stats['sql_dump_path']}")
    print("=" * 94)

    if not rows:
        print("\n  ⚠️ Henüz veritabanında kayıtlı telemetri satırı bulunmuyor.")
        print("     Unreal Engine (Port 7400) telemetri yayını başladığında veriler otomatik işlenir.\n")
        print("=" * 94 + "\n")
        return

    # Tablo Başlığı
    header = f"| {'ID':^5} | {'ZAMAN (s)':^10} | {'ARAÇ':^5} | {'ROLL (°)':^9} | {'PITCH (°)':^9} | {'YAW (°)':^9} | {'İRTİFA (m)':^10} | {'HIZ (m/s)':^9} | {'PİL':^7} | {'MOD':^10} |"
    sep = "+" + "-"*7 + "+" + "-"*12 + "+" + "-"*7 + "+" + "-"*11 + "+" + "-"*11 + "+" + "-"*11 + "+" + "-"*12 + "+" + "-"*11 + "+" + "-"*9 + "+" + "-"*12 + "+"

    print(sep)
    print(header)
    print(sep)

    for r in rows:
        t_str = f"{r['time']:.2f}"
        roll_str = f"{r['roll']:+.2f}°"
        pitch_str = f"{r['pitch']:+.2f}°"
        yaw_str = f"{r['yaw']:+.2f}°"
        alt_str = f"{r['altitude']:.2f} m"
        speed_str = f"{r['speed']:.2f}"
        batt_str = f"{r['battery']:.1f}%"
        mode_str = f"{r['mode'][:10]}"

        line = f"| {r['id']:^5} | {t_str:^10} | {r['vehicle_id']:^5} | {roll_str:^9} | {pitch_str:^9} | {yaw_str:^9} | {alt_str:^10} | {speed_str:^9} | {batt_str:^7} | {mode_str:^10} |"
        print(line)

    print(sep)
    print(f"  ✅ Gösterilen: En son {len(rows)} satır (En yeni yukarıda). Sürekli izlemek için 'watch -n 1 python view_sql.py'")
    print("=" * 94 + "\n")

if __name__ == "__main__":
    main()
