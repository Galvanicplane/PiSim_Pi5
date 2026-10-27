#!/usr/bin/env python3
"""
telemetry_db.py - PiSim Raspberry Pi 5 SQLite & SQL Batch Telemetri Günlük Yöneticisi
Unreal Engine (PiSim Studio) UPiSimTelemetryLogger formatı ile %100 uyumlu yerel veritabanı.

Özellikler:
1. Gelen tüm canlı UDP telemetri paketlerini SQLite (pisim_telemetry.db) veritabanına yazar.
2. Eşzamanlı olarak standart SQL formatında (flight_telemetry.sql) TRANSACTION blokları üretir.
3. Pi 5 Web Arayüzü (/api/sql_table) ve terminal CLI araçları için son satırları JSON/dict olarak sunar.
4. Yüksek frekanslı UDP akışını aksatmamak için kuyruklu (thread-safe batch) yazma mimarisi kullanır.
"""

import sqlite3
import threading
import time
from pathlib import Path

DB_FILE = Path(__file__).parent / "pisim_telemetry.db"
SQL_DUMP_FILE = Path(__file__).parent / "flight_telemetry.sql"

_lock = threading.Lock()
_pending_rows = []
_total_logged = 0

def init_db():
    """Veritabanını ve tablo şemasını oluşturur (Her oturumda eski kayıtları temizler)."""
    with _lock:
        conn = sqlite3.connect(str(DB_FILE))
        cursor = conn.cursor()
        cursor.execute("DROP TABLE IF EXISTS vehicle_telemetry")
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS vehicle_telemetry (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp_sec REAL NOT NULL,
                vehicle_id INTEGER NOT NULL,
                roll_deg REAL,
                pitch_deg REAL,
                yaw_deg REAL,
                altitude_m REAL,
                airspeed_ms REAL,
                battery_pct REAL,
                lat REAL,
                lon REAL,
                flight_mode TEXT
            )
        """)
        cursor.execute("CREATE INDEX IF NOT EXISTS idx_telemetry_time ON vehicle_telemetry(timestamp_sec)")
        cursor.execute("CREATE INDEX IF NOT EXISTS idx_telemetry_vehicle ON vehicle_telemetry(vehicle_id)")
        conn.commit()

        global _total_logged
        _total_logged = 0
        conn.close()

    # SQL Döküm dosyasını her oturumda taze olarak sıfırla
    with open(SQL_DUMP_FILE, "w", encoding="utf-8") as f:
        f.write("-- =========================================================================\n")
        f.write("-- PiSim Raspberry Pi 5 Uçuş Telemetri Veritabanı Dökümü\n")
        f.write("-- Oturum Başlangıcında Sıfırlanmıştır\n")
        f.write("-- =========================================================================\n\n")

def queue_telemetry_row(data: dict):
    """Gelen telemetri paketini kuyruğa ekler."""
    now_sec = time.time()
    t_sec = float(data.get("timestamp", now_sec))
    v_id = int(data.get("vehicle_id", 0))

    # Tutum verileri (roll, pitch, yaw)
    att = data.get("attitude", {})
    roll = float(data.get("roll", att.get("roll", 0.0)))
    pitch = float(data.get("pitch", att.get("pitch", 0.0)))
    yaw = float(data.get("yaw", att.get("yaw", 0.0)))

    # İrtifa & Hız
    alt = float(data.get("altitude", data.get("alt", 0.0)))
    speed = float(data.get("speed", data.get("airspeed", 0.0)))
    batt = float(data.get("battery", 98.0))

    # GPS
    gps = data.get("gps", {})
    lat = float(data.get("lat", gps.get("lat", 39.9334)))
    lon = float(data.get("lon", gps.get("lon", 32.8597)))
    mode = str(data.get("flight_mode", "AUTO"))

    row = (t_sec, v_id, roll, pitch, yaw, alt, speed, batt, lat, lon, mode)

    with _lock:
        _pending_rows.append(row)
        # 10 örnek biriktiğinde otomatik batch commit yap
        if len(_pending_rows) >= 10:
            _flush_locked()

def _flush_locked():
    """Kuyruktaki satırları SQLite ve SQL dosyasına atomik olarak yazar (Kilitli çağrılmalıdır)."""
    global _total_logged
    if not _pending_rows:
        return 0

    batch = list(_pending_rows)
    _pending_rows.clear()

    try:
        conn = sqlite3.connect(str(DB_FILE))
        cursor = conn.cursor()
        cursor.executemany("""
            INSERT INTO vehicle_telemetry 
            (timestamp_sec, vehicle_id, roll_deg, pitch_deg, yaw_deg, altitude_m, airspeed_ms, battery_pct, lat, lon, flight_mode)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, batch)
        conn.commit()
        conn.close()

        _total_logged += len(batch)

        # Standart SQL dosyasına da Append et
        with open(SQL_DUMP_FILE, "a", encoding="utf-8") as f:
            f.write(f"\n-- Batch Kaydı: {time.strftime('%Y-%m-%d %H:%M:%S')} ({len(batch)} satır)\n")
            f.write("BEGIN TRANSACTION;\n")
            for r in batch:
                f.write(f"INSERT INTO vehicle_telemetry (timestamp_sec, vehicle_id, roll_deg, pitch_deg, yaw_deg, altitude_m, airspeed_ms, battery_pct, lat, lon, flight_mode) VALUES ({r[0]:.2f}, {r[1]}, {r[2]:.2f}, {r[3]:.2f}, {r[4]:.2f}, {r[5]:.2f}, {r[6]:.2f}, {r[7]:.1f}, {r[8]:.6f}, {r[9]:.6f}, '{r[10]}');\n")
            f.write("COMMIT;\n")

        return len(batch)
    except Exception as e:
        print(f"⚠️ [telemetry_db] Flush hatası: {e}")
        return 0

def flush():
    """Dışarıdan manuel flush çağrısı."""
    with _lock:
        return _flush_locked()

def get_recent_rows(max_rows: int = 50) -> list:
    """En son kaydedilen N satırı liste olarak döner (En yeni satırlar önce)."""
    with _lock:
        _flush_locked()
        try:
            conn = sqlite3.connect(str(DB_FILE))
            conn.row_factory = sqlite3.Row
            cursor = conn.cursor()
            cursor.execute("""
                SELECT id, timestamp_sec, vehicle_id, roll_deg, pitch_deg, yaw_deg, altitude_m, airspeed_ms, battery_pct, lat, lon, flight_mode
                FROM vehicle_telemetry
                ORDER BY id DESC
                LIMIT ?
            """, (max_rows,))
            rows = cursor.fetchall()
            conn.close()

            result = []
            for r in rows:
                result.append({
                    "id": r["id"],
                    "time": r["timestamp_sec"],
                    "vehicle_id": r["vehicle_id"],
                    "roll": r["roll_deg"],
                    "pitch": r["pitch_deg"],
                    "yaw": r["yaw_deg"],
                    "altitude": r["altitude_m"],
                    "speed": r["airspeed_ms"],
                    "battery": r["battery_pct"],
                    "lat": r["lat"],
                    "lon": r["lon"],
                    "mode": r["flight_mode"]
                })
            return result
        except Exception as e:
            print(f"⚠️ [telemetry_db] Okuma hatası: {e}")
            return []

def get_stats() -> dict:
    """Veritabanı özet istatistiklerini döner."""
    with _lock:
        return {
            "total_logged": _total_logged,
            "pending_in_memory": len(_pending_rows),
            "db_path": str(DB_FILE),
            "sql_dump_path": str(SQL_DUMP_FILE)
        }

# Modül ilk yüklendiğinde veritabanını hazırla
init_db()
