#!/usr/bin/env python3
"""
telemetry_db.py - PiSim Raspberry Pi 5 SQLite & SQL Batch Telemetri Günlük Yöneticisi
Unreal Engine (PiSim Studio) UPiSimTelemetryLogger formatı ile %100 uyumlu yerel veritabanı.

Özellikler:
1. Gelen tüm canlı UDP telemetri paketlerini SQLite (pisim_telemetry.db) veritabanına yazar.
2. Eşzamanlı olarak standart SQL formatında (flight_telemetry.sql) TRANSACTION blokları üretir.
3. Pi 5 Web Arayüzü (/api/sql_table) ve terminal CLI araçları için son satırları JSON/dict olarak sunar.
4. Yüksek frekanslı UDP akışını (50+ Hz) asla tıkamamak için asenkron kuyruklu (background batch worker) mimari kullanır.
"""

import sqlite3
import threading
import queue
import time
from pathlib import Path

DB_FILE = Path(__file__).parent / "pisim_telemetry.db"
SQL_DUMP_FILE = Path(__file__).parent / "flight_telemetry.sql"

_pending_queue = queue.Queue(maxsize=20000)
_total_logged = 0
_writer_thread = None
_writer_running = True

def _write_batch_to_disk(batch: list):
    """Kuyruktaki satırları SQLite ve SQL dosyasına atomik batch olarak yazar."""
    global _total_logged
    if not batch:
        return 0

    try:
        conn = sqlite3.connect(str(DB_FILE), timeout=5.0)
        cursor = conn.cursor()
        cursor.executemany("""
            INSERT INTO vehicle_telemetry 
            (timestamp_sec, vehicle_code, vehicle_id, roll_deg, pitch_deg, yaw_deg, altitude_m, airspeed_ms, battery_pct, lat, lon, flight_mode)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, batch)
        conn.commit()
        conn.close()

        _total_logged += len(batch)

        # Standart SQL dosyasına da Append et
        with open(SQL_DUMP_FILE, "a", encoding="utf-8") as f:
            f.write(f"\n-- Batch Kaydı: {time.strftime('%Y-%m-%d %H:%M:%S')} ({len(batch)} satır)\n")
            f.write("BEGIN TRANSACTION;\n")
            for r in batch:
                f.write(f"INSERT INTO vehicle_telemetry (timestamp_sec, vehicle_code, vehicle_id, roll_deg, pitch_deg, yaw_deg, altitude_m, airspeed_ms, battery_pct, lat, lon, flight_mode) VALUES ({r[0]:.2f}, '{r[1]}', {r[2]}, {r[3]:.2f}, {r[4]:.2f}, {r[5]:.2f}, {r[6]:.2f}, {r[7]:.2f}, {r[8]:.1f}, {r[9]:.6f}, {r[10]:.6f}, '{r[11]}');\n")
            f.write("COMMIT;\n")

        return len(batch)
    except Exception as e:
        print(f"⚠️ [telemetry_db] Disk flush hatası: {e}")
        return 0

def _db_writer_worker():
    """Arka planda çalışan ve SQLite yazımlarını batch olarak gerçekleştiren iş parçacığı."""
    while _writer_running:
        rows = []
        try:
            # 0.2 saniye veya veri gelene kadar bekle
            first_row = _pending_queue.get(timeout=0.2)
            rows.append(first_row)
            # Kuyrukta biriken diğer paketleri de al (Maksimum 100 satırlık paketler)
            while len(rows) < 100:
                try:
                    rows.append(_pending_queue.get_nowait())
                except queue.Empty:
                    break
        except queue.Empty:
            pass

        if rows:
            _write_batch_to_disk(rows)

def init_db():
    """Veritabanını ve tablo şemasını oluşturur, eski test araçlarını temizler."""
    global _total_logged, _writer_thread
    conn = sqlite3.connect(str(DB_FILE), timeout=5.0)
    cursor = conn.cursor()
    cursor.execute("PRAGMA journal_mode = WAL")
    cursor.execute("PRAGMA synchronous = NORMAL")
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS vehicle_telemetry (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp_sec REAL NOT NULL,
            vehicle_code TEXT NOT NULL,
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
    cursor.execute("CREATE INDEX IF NOT EXISTS idx_telemetry_code ON vehicle_telemetry(vehicle_code)")
    cursor.execute("CREATE INDEX IF NOT EXISTS idx_telemetry_vehicle ON vehicle_telemetry(vehicle_id)")
    
    # Eski sahte test araçlarını temizle
    cursor.execute("DELETE FROM vehicle_telemetry WHERE vehicle_code LIKE '%2810260100%' OR vehicle_code = 'rover_real_test'")
    conn.commit()

    try:
        cursor.execute("SELECT COUNT(*) FROM vehicle_telemetry")
        _total_logged = cursor.fetchone()[0] or 0
    except Exception:
        _total_logged = 0
    conn.close()

    # SQL Döküm dosyası yoksa oluştur
    if not SQL_DUMP_FILE.exists():
        with open(SQL_DUMP_FILE, "w", encoding="utf-8") as f:
            f.write("-- =========================================================================\n")
            f.write("-- PiSim Raspberry Pi 5 Uçuş Telemetri Veritabanı Dökümü\n")
            f.write("-- =========================================================================\n\n")

    # Arka plan yazıcı iş parçacığını başlat
    if _writer_thread is None or not _writer_thread.is_alive():
        _writer_thread = threading.Thread(target=_db_writer_worker, daemon=True, name="TelemetryDbWriter")
        _writer_thread.start()

def queue_telemetry_row(data: dict):
    """Gelen telemetri paketini non-blocking in-memory kuyruğa ekler (< 0.001ms gecikme)."""
    now_sec = time.time()
    v_id = int(data.get("vehicle_id", 0))
    v_code = str(data.get("vehicle_code", f"rover_{v_id+1:02d}"))

    # Tutum verileri (roll, pitch, yaw)
    att = data.get("attitude", {})
    roll = float(data.get("roll", att.get("roll", 0.0) if isinstance(att, dict) else (att[0]*57.2958 if isinstance(att, list) and len(att)>0 else 0.0)))
    pitch = float(data.get("pitch", att.get("pitch", 0.0) if isinstance(att, dict) else (att[1]*57.2958 if isinstance(att, list) and len(att)>1 else 0.0)))
    yaw = float(data.get("yaw", att.get("yaw", 0.0) if isinstance(att, dict) else (att[2]*57.2958 if isinstance(att, list) and len(att)>2 else 0.0)))

    # İrtifa & Hız
    alt = float(data.get("altitude", data.get("alt", 0.0)))
    speed = float(data.get("speed", data.get("airspeed", 0.0)))
    batt = float(data.get("battery", 98.0))

    # GPS
    gps = data.get("gps", {})
    lat = float(data.get("lat", gps.get("lat", 39.9334) if isinstance(gps, dict) else 39.9334))
    lon = float(data.get("lon", gps.get("lon", 32.8597) if isinstance(gps, dict) else 32.8597))
    mode = str(data.get("flight_mode", "AUTO"))

    # Gerçek sistem zaman damgası (now_sec) ile kaydet (Simülasyon saniyesi değil)
    row = (now_sec, v_code, v_id, roll, pitch, yaw, alt, speed, batt, lat, lon, mode)

    try:
        _pending_queue.put_nowait(row)
    except queue.Full:
        pass # Kuyruk dolarsa eskiyi geç

def get_recent_rows(max_rows: int = 50, vehicle_code: str = None) -> list:
    """En son kaydedilen N satırı liste olarak döner (En yeni satırlar önce)."""
    try:
        conn = sqlite3.connect(str(DB_FILE), timeout=3.0)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()
        if vehicle_code:
            cursor.execute("""
                SELECT id, timestamp_sec, vehicle_code, vehicle_id, roll_deg, pitch_deg, yaw_deg, altitude_m, airspeed_ms, battery_pct, lat, lon, flight_mode
                FROM vehicle_telemetry
                WHERE vehicle_code = ?
                ORDER BY id DESC
                LIMIT ?
            """, (vehicle_code, max_rows))
        else:
            cursor.execute("""
                SELECT id, timestamp_sec, vehicle_code, vehicle_id, roll_deg, pitch_deg, yaw_deg, altitude_m, airspeed_ms, battery_pct, lat, lon, flight_mode
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
                "vehicle_code": r["vehicle_code"],
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

def get_fleet_from_db(max_age_sec: float = None) -> list:
    """Veritabanında kayıtlı tüm araçların en son telemetrisini döner (Doğrudan SQL Tablosundan)."""
    try:
        conn = sqlite3.connect(str(DB_FILE), timeout=3.0)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()
        if max_age_sec is not None:
            min_time = time.time() - max_age_sec
            cursor.execute("""
                SELECT t.*
                FROM vehicle_telemetry t
                INNER JOIN (
                    SELECT vehicle_code, MAX(id) as max_id
                    FROM vehicle_telemetry
                    WHERE timestamp_sec >= ?
                    GROUP BY vehicle_code
                ) m ON t.id = m.max_id
                ORDER BY t.vehicle_id ASC
            """, (min_time,))
        else:
            cursor.execute("""
                SELECT t.*
                FROM vehicle_telemetry t
                INNER JOIN (
                    SELECT vehicle_code, MAX(id) as max_id
                    FROM vehicle_telemetry
                    GROUP BY vehicle_code
                ) m ON t.id = m.max_id
                ORDER BY t.vehicle_id ASC
            """)
        rows = cursor.fetchall()
        conn.close()

        result = []
        for r in rows:
            result.append({
                "code": r["vehicle_code"],
                "id": r["vehicle_id"],
                "name": f"Rover {r['vehicle_id'] + 1:02d}",
                "speed": round(float(r["airspeed_ms"]), 1) if r["airspeed_ms"] is not None else 0.0,
                "altitude": round(float(r["altitude_m"]), 1) if r["altitude_m"] is not None else 0.0,
                "battery": round(float(r["battery_pct"]), 1) if r["battery_pct"] is not None else 98.0,
                "flight_mode": r["flight_mode"] or "AKTİF",
                "lat": float(r["lat"]) if r["lat"] is not None else 39.9334,
                "lon": float(r["lon"]) if r["lon"] is not None else 32.8597,
                "roll": round(float(r["roll_deg"]), 1) if r["roll_deg"] is not None else 0.0,
                "pitch": round(float(r["pitch_deg"]), 1) if r["pitch_deg"] is not None else 0.0,
                "yaw": round(float(r["yaw_deg"]), 1) if r["yaw_deg"] is not None else 0.0,
                "last_rx_time": float(r["timestamp_sec"])
            })
        return result
    except Exception as e:
        print(f"⚠️ [telemetry_db] get_fleet_from_db hatası: {e}")
        return []

def clear_fleet(preserve_codes: list = None) -> bool:
    """Eski veya çevrimdışı araç kayıtlarını temizler."""
    try:
        conn = sqlite3.connect(str(DB_FILE), timeout=5.0)
        cursor = conn.cursor()
        if preserve_codes:
            placeholders = ",".join("?" for _ in preserve_codes)
            cursor.execute(f"DELETE FROM vehicle_telemetry WHERE vehicle_code NOT IN ({placeholders})", preserve_codes)
        else:
            cursor.execute("DELETE FROM vehicle_telemetry")
        conn.commit()
        conn.close()

        # SQL döküm dosyasını da temizle
        with open(SQL_DUMP_FILE, "w", encoding="utf-8") as f:
            f.write("-- =========================================================================\n")
            f.write("-- PiSim Raspberry Pi 5 Uçuş Telemetri Veritabanı Dökümü\n")
            f.write(f"-- Temizleme Zamanı: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("-- =========================================================================\n\n")

        print("🧹 [telemetry_db] Araç kayıtları ve SQL dökümü başarıyla temizlendi.")
        return True
    except Exception as e:
        print(f"⚠️ [telemetry_db] clear_fleet hatası: {e}")
        return False

def get_stats() -> dict:
    """Veritabanı özet istatistiklerini döner."""
    return {
        "total_logged": _total_logged,
        "pending_in_queue": _pending_queue.qsize(),
        "db_path": str(DB_FILE),
        "sql_dump_path": str(SQL_DUMP_FILE)
    }

# Modül yüklendiğinde veritabanını ve yazıcıyı hazırla
init_db()
