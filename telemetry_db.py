#!/usr/bin/env python3
"""
telemetry_db.py - PiSim Raspberry Pi 5 SQLite & SQL Batch Telemetri Günlük Yöneticisi

Özellikler:
1. Gelen tüm canlı UDP 7400 telemetri paketlerini RAM tamponunda biriktirip SQLite (pisim_telemetry.db) veritabanına yazar.
2. Eşzamanlı olarak standart SQL formatında (flight_telemetry.sql) TRANSACTION blokları üretir.
3. Pi 5 Web Arayüzü (/api/sql_table) ve terminal CLI araçları için son satırları veya sadece benzersiz araçları (unique_only) sunar.
4. Program her başladığında clear_fleet() ile eski kayıtları sıfırlayarak %100 temiz başlangıç yapar.
"""

import sqlite3
import threading
import queue
import time
import sys
from pathlib import Path

try:
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    if hasattr(sys.stderr, 'reconfigure'):
        sys.stderr.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

DB_FILE = Path(__file__).parent / "pisim_telemetry.db"
SQL_DUMP_FILE = Path(__file__).parent / "flight_telemetry.sql"

_pending_queue = queue.Queue(maxsize=20000)
_buffered_rows = []
_buffer_lock = threading.Lock()
_total_logged = 0
_writer_thread = None
_writer_running = True

RAM_BUFFER_FLUSH_INTERVAL_SEC = 10.0  # RAM tamponunu 10 sn'de bir diske/SQL'e bas ve RAM'i boşalt


def _write_batch_to_disk(batch: list) -> int:
    """Kuyruktaki satırları SQLite ve SQL dosyasına atomik batch olarak yazar."""
    global _total_logged
    if not batch:
        return 0

    try:
        conn = sqlite3.connect(str(DB_FILE), timeout=5.0)
        cursor = conn.cursor()
        cursor.executemany("""
            INSERT INTO vehicle_telemetry 
            (timestamp_sec, vehicle_code, vehicle_name, vehicle_id, pos_x, pos_y, pos_z, roll_deg, pitch_deg, yaw_deg, altitude_m, airspeed_ms, battery_pct, lat, lon, flight_mode)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, batch)
        conn.commit()
        conn.close()

        _total_logged += len(batch)

        # Standart SQL dosyasına da Append et
        with open(SQL_DUMP_FILE, "a", encoding="utf-8") as f:
            f.write(f"\n-- Batch Kaydı: {time.strftime('%Y-%m-%d %H:%M:%S')} ({len(batch)} satır)\n")
            f.write("BEGIN TRANSACTION;\n")
            for r in batch:
                f.write(
                    f"INSERT INTO vehicle_telemetry "
                    f"(timestamp_sec, vehicle_code, vehicle_name, vehicle_id, pos_x, pos_y, pos_z, roll_deg, pitch_deg, yaw_deg, altitude_m, airspeed_ms, battery_pct, lat, lon, flight_mode) "
                    f"VALUES ({r[0]:.2f}, '{r[1]}', '{r[2]}', {r[3]}, {r[4]:.2f}, {r[5]:.2f}, {r[6]:.2f}, {r[7]:.2f}, {r[8]:.2f}, {r[9]:.2f}, {r[10]:.2f}, {r[11]:.2f}, {r[12]:.1f}, {r[13]:.6f}, {r[14]:.6f}, '{r[15]}');\n"
                )
            f.write("COMMIT;\n")

        return len(batch)
    except Exception as e:
        print(f"⚠️ [telemetry_db] Disk flush hatası: {e}")
        return 0


def flush_now() -> int:
    """RAM kuyruğunda bekleyen tüm telemetri paketlerini anında SQLite'a yazar."""
    with _buffer_lock:
        while not _pending_queue.empty():
            try:
                _buffered_rows.append(_pending_queue.get_nowait())
            except queue.Empty:
                break
        if not _buffered_rows:
            return 0
        to_write = list(_buffered_rows)
        _buffered_rows.clear()
    return _write_batch_to_disk(to_write)


def _db_writer_worker():
    """Gelen telemetri paketlerini RAM'de biriktirir ve her 10 saniyede bir toplu batch olarak SQL'e yazıp RAM'i boşaltır."""
    last_flush_time = time.time()

    while _writer_running:
        try:
            row = _pending_queue.get(timeout=0.5)
            with _buffer_lock:
                _buffered_rows.append(row)
                while not _pending_queue.empty() and len(_buffered_rows) < 5000:
                    try:
                        _buffered_rows.append(_pending_queue.get_nowait())
                    except queue.Empty:
                        break
        except queue.Empty:
            pass

        now = time.time()
        if (now - last_flush_time >= RAM_BUFFER_FLUSH_INTERVAL_SEC):
            count = flush_now()
            last_flush_time = now
            if count > 0:
                print(f"🛢️ [telemetry_db] 10 saniyelik RAM tamponu SQL'e kaydedildi ({count} satır) ve RAM temizlendi.")


def init_db(clean_start: bool = True):
    """Veritabanını ve tablo şemasını oluşturur. clean_start=True ise başlangıçta eski tabloyu sıfırlar."""
    global _total_logged, _writer_thread
    conn = sqlite3.connect(str(DB_FILE), timeout=5.0)
    cursor = conn.cursor()
    cursor.execute("PRAGMA journal_mode = WAL")
    cursor.execute("PRAGMA synchronous = NORMAL")
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS vehicle_telemetry (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            created_at TEXT DEFAULT (datetime('now', 'localtime')),
            timestamp_sec REAL NOT NULL,
            vehicle_code TEXT NOT NULL,
            vehicle_name TEXT,
            vehicle_id INTEGER NOT NULL,
            pos_x REAL DEFAULT 0.0,
            pos_y REAL DEFAULT 0.0,
            pos_z REAL DEFAULT 0.0,
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
    for col_sql in [
        "ALTER TABLE vehicle_telemetry ADD COLUMN created_at TEXT DEFAULT (datetime('now', 'localtime'))",
        "ALTER TABLE vehicle_telemetry ADD COLUMN vehicle_name TEXT",
        "ALTER TABLE vehicle_telemetry ADD COLUMN pos_x REAL DEFAULT 0.0",
        "ALTER TABLE vehicle_telemetry ADD COLUMN pos_y REAL DEFAULT 0.0",
        "ALTER TABLE vehicle_telemetry ADD COLUMN pos_z REAL DEFAULT 0.0",
    ]:
        try:
            cursor.execute(col_sql)
        except Exception:
            pass

    cursor.execute("CREATE INDEX IF NOT EXISTS idx_telemetry_time ON vehicle_telemetry(timestamp_sec)")
    cursor.execute("CREATE INDEX IF NOT EXISTS idx_telemetry_code ON vehicle_telemetry(vehicle_code)")
    cursor.execute("CREATE INDEX IF NOT EXISTS idx_telemetry_vehicle ON vehicle_telemetry(vehicle_id)")
    conn.commit()
    conn.close()

    if clean_start:
        clear_fleet()
    else:
        try:
            conn = sqlite3.connect(str(DB_FILE), timeout=5.0)
            cursor = conn.cursor()
            cursor.execute("SELECT COUNT(*) FROM vehicle_telemetry")
            _total_logged = cursor.fetchone()[0] or 0
            conn.close()
        except Exception:
            _total_logged = 0

    if _writer_thread is None or not _writer_thread.is_alive():
        _writer_thread = threading.Thread(target=_db_writer_worker, daemon=True, name="TelemetryDbWriter")
        _writer_thread.start()


def sync_sql_data(discovered_ue5_ip: str = None) -> dict:
    """
    Tablodaki veriler doğrudan gelen canlı UDP telemetri mesajlarından oluşur.
    Bekleyen RAM tamponunu diske yazar ve güncel durumu döner.
    """
    added = flush_now()
    fleet = get_fleet_from_db()
    return {
        "added_rows": added,
        "total_logged": _total_logged,
        "fleet": fleet
    }


def queue_telemetry_row(data: dict):
    """Gelen telemetri paketini non-blocking in-memory kuyruğa ekler."""
    now_sec = time.time()
    v_id = int(data.get("vehicle_id", data.get("id", 1)))
    v_code = str(data.get("vehicle_code") or data.get("code") or f"RV{v_id:02d}")
    v_name = str(data.get("vehicle_name") or data.get("name") or v_code)

    # 3B Konum (X, Y, Z metre)
    pos = data.get("position", [])
    pos_x = float(data.get("pos_x", pos[0] if isinstance(pos, list) and len(pos) > 0 else 0.0))
    pos_y = float(data.get("pos_y", pos[1] if isinstance(pos, list) and len(pos) > 1 else 0.0))
    pos_z = float(data.get("pos_z", pos[2] if isinstance(pos, list) and len(pos) > 2 else data.get("altitude", 0.0)))

    # Tutum verileri (roll, pitch, yaw)
    att = data.get("attitude", {})
    roll = float(data.get("roll", att.get("roll", 0.0) if isinstance(att, dict) else (att[0]*57.2958 if isinstance(att, list) and len(att)>0 else 0.0)))
    pitch = float(data.get("pitch", att.get("pitch", 0.0) if isinstance(att, dict) else (att[1]*57.2958 if isinstance(att, list) and len(att)>1 else 0.0)))
    yaw = float(data.get("yaw", att.get("yaw", 0.0) if isinstance(att, dict) else (att[2]*57.2958 if isinstance(att, list) and len(att)>2 else 0.0)))

    # İrtifa & Hız
    alt = float(data.get("altitude", pos_z))
    speed = float(data.get("speed", data.get("airspeed", 0.0)))
    batt = float(data.get("battery", 98.0))

    # GPS
    gps = data.get("gps", {})
    lat = float(data.get("lat", gps.get("lat", 39.9334) if isinstance(gps, dict) else 39.9334))
    lon = float(data.get("lon", gps.get("lon", 32.8597) if isinstance(gps, dict) else 32.8597))
    mode = str(data.get("flight_mode", "AKTİF"))

    row = (now_sec, v_code, v_name, v_id, pos_x, pos_y, pos_z, roll, pitch, yaw, alt, speed, batt, lat, lon, mode)

    try:
        _pending_queue.put_nowait(row)
    except queue.Full:
        pass


def get_recent_rows(max_rows: int = 50, vehicle_code: str = None, unique_only: bool = False) -> list:
    """
    En son kaydedilen N satırı liste olarak döner (En yeni satırlar önce).
    unique_only=True ise, veritabanındaki her benzersiz aracın (unique vehicle_code) en son kaydını döner.
    """
    flush_now()
    try:
        conn = sqlite3.connect(str(DB_FILE), timeout=3.0)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()
        if unique_only:
            cursor.execute("""
                SELECT t.id, t.created_at, t.timestamp_sec, t.vehicle_code, t.vehicle_name, t.vehicle_id,
                       t.pos_x, t.pos_y, t.pos_z,
                       t.roll_deg, t.pitch_deg, t.yaw_deg, t.altitude_m, t.airspeed_ms, t.battery_pct,
                       t.lat, t.lon, t.flight_mode
                FROM vehicle_telemetry t
                INNER JOIN (
                    SELECT vehicle_code, MAX(id) as max_id
                    FROM vehicle_telemetry
                    GROUP BY vehicle_code
                ) m ON t.id = m.max_id
                ORDER BY t.vehicle_id ASC, t.id DESC
                LIMIT ?
            """, (max_rows,))
        elif vehicle_code:
            cursor.execute("""
                SELECT id, created_at, timestamp_sec, vehicle_code, vehicle_name, vehicle_id,
                       pos_x, pos_y, pos_z,
                       roll_deg, pitch_deg, yaw_deg, altitude_m, airspeed_ms, battery_pct, lat, lon, flight_mode
                FROM vehicle_telemetry
                WHERE vehicle_code = ?
                ORDER BY id DESC
                LIMIT ?
            """, (vehicle_code, max_rows))
        else:
            cursor.execute("""
                SELECT id, created_at, timestamp_sec, vehicle_code, vehicle_name, vehicle_id,
                       pos_x, pos_y, pos_z,
                       roll_deg, pitch_deg, yaw_deg, altitude_m, airspeed_ms, battery_pct, lat, lon, flight_mode
                FROM vehicle_telemetry
                ORDER BY id DESC
                LIMIT ?
            """, (max_rows,))
        rows = cursor.fetchall()
        conn.close()

        result = []
        for r in rows:
            keys = r.keys()
            v_name = r["vehicle_name"] if ("vehicle_name" in keys and r["vehicle_name"]) else r["vehicle_code"]
            result.append({
                "id": r["id"],
                "created_at": r["created_at"] if "created_at" in keys and r["created_at"] else "--",
                "time": r["timestamp_sec"],
                "vehicle_code": r["vehicle_code"],
                "vehicle_name": v_name,
                "vehicle_id": r["vehicle_id"],
                "pos_x": round(float(r["pos_x"] or 0.0), 2) if "pos_x" in keys else 0.0,
                "pos_y": round(float(r["pos_y"] or 0.0), 2) if "pos_y" in keys else 0.0,
                "pos_z": round(float(r["pos_z"] or 0.0), 2) if "pos_z" in keys else 0.0,
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
    """Veritabanında kayıtlı tüm benzersiz araçların en son telemetrisini döner."""
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
            keys = r.keys()
            v_code = r["vehicle_code"]
            if not v_code:
                continue
            v_name = r["vehicle_name"] if ("vehicle_name" in keys and r["vehicle_name"]) else v_code
            result.append({
                "code": v_code,
                "vehicle_code": v_code,
                "id": r["vehicle_id"],
                "vehicle_id": r["vehicle_id"],
                "name": v_name,
                "vehicle_name": v_name,
                "pos_x": round(float(r["pos_x"] or 0.0), 2) if "pos_x" in keys else 0.0,
                "pos_y": round(float(r["pos_y"] or 0.0), 2) if "pos_y" in keys else 0.0,
                "pos_z": round(float(r["pos_z"] or 0.0), 2) if "pos_z" in keys else 0.0,
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
    """Tüm SQLite tablosunu, RAM tamponunu ve SQL döküm dosyasını sıfırlar."""
    global _total_logged
    with _buffer_lock:
        _buffered_rows.clear()
        while not _pending_queue.empty():
            try:
                _pending_queue.get_nowait()
            except queue.Empty:
                break
    _total_logged = 0
    try:
        conn = sqlite3.connect(str(DB_FILE), timeout=5.0)
        cursor = conn.cursor()
        if preserve_codes:
            placeholders = ",".join("?" for _ in preserve_codes)
            cursor.execute(f"DELETE FROM vehicle_telemetry WHERE vehicle_code NOT IN ({placeholders})", preserve_codes)
            cursor.execute("SELECT COUNT(*) FROM vehicle_telemetry")
            _total_logged = cursor.fetchone()[0] or 0
        else:
            cursor.execute("DELETE FROM vehicle_telemetry")
            try:
                cursor.execute("DELETE FROM sqlite_sequence WHERE name='vehicle_telemetry'")
            except Exception:
                pass
        conn.commit()
        try:
            cursor.execute("PRAGMA wal_checkpoint(TRUNCATE)")
        except Exception:
            pass
        conn.close()

        with open(SQL_DUMP_FILE, "w", encoding="utf-8") as f:
            f.write("-- =========================================================================\n")
            f.write("-- PiSim Raspberry Pi 5 Uçuş Telemetri Veritabanı Dökümü\n")
            f.write(f"-- Temiz Başlangıç Zamanı: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("-- =========================================================================\n\n")

        print("🧹 [telemetry_db] SQL veritabanı ve RAM tamponu sıfırlandı (Temiz Başlangıç).")
        return True
    except Exception as e:
        print(f"⚠️ [telemetry_db] clear_fleet hatası: {e}")
        return False


def get_stats() -> dict:
    """Veritabanı istatistiklerini döner."""
    return {
        "total_logged": _total_logged,
        "queue_size": _pending_queue.qsize() + len(_buffered_rows),
        "db_path": str(DB_FILE.name),
        "sql_dump_path": str(SQL_DUMP_FILE.name)
    }


def add_network_rows(rows: list) -> int:
    """Geriye dönük uyumluluk için."""
    return 0


# Modül yüklendiğinde veritabanını temiz başlat
init_db(clean_start=True)
