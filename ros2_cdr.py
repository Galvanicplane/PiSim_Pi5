#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ros2_cdr.py - PiSim ROS 2 Binary CDR (Common Data Representation) Codec & Inspector
====================================================================================
OMG CDR (Little-Endian: 0x00 0x01 0x00 0x00) ikili (binary) serileştirme ve
ters-serileştirme (deserialization) kütüphanesi.

Desteklenen Mesaj Tipleri:
  1. sensor_msgs/msg/Imu              -> /<vehicle>/imu
  2. sensor_msgs/msg/NavSatFix        -> /<vehicle>/gps
  3. sensor_msgs/msg/CompressedImage  -> /<vehicle>/camera/h264 (ve camera/compressed)
  4. geometry_msgs/msg/Twist          -> /<vehicle>/cmd_vel
  5. pisim_msgs/msg/Channels          -> /<vehicle>/channels (CH1: Tekerlek, CH2: Steer Servo...)
  6. pisim_msgs/msg/Actuators         -> /<vehicle>/actuators
  7. pisim_msgs/msg/Ack               -> /<vehicle>/ack (Kodlar: 0=Yok, 1=Bozuk, 2=Sağlıklı, 3=Trigger, 4=Standby)

Özellikler:
  - Sıfır harici bağımlılık (saf Python `struct` ile nanosaniye hızında çalışır).
  - Hem Binary CDR (Hex/Byte) hem de JSON görünümünü aynı anda üreten `inspect_cdr_packet` fonksiyonu içerir.
  - Kamera Standby modu için ~165 byte boyutunda saf Mavi Ekran (Blue Screen) çerçevesi üretir.
"""

import base64
import json
import struct
import time
from typing import Any, Dict, Optional, Tuple, Union

# OMG CDR Little-Endian Encapsulation Header (4 byte)
CDR_LE_HEADER = b"\x00\x01\x00\x00"
CDR_BE_HEADER = b"\x00\x00\x00\x00"

# ACK Durum Kodları (Kullanıcı Mimarisi)
ACK_TOPIC_MISSING = 0        # 0: Topic yok / bulunamadı
ACK_INVALID_SCHEMA = 1       # 1: Topic var ama içindeki veri istenilen yapıda değil
ACK_HEALTHY_OK = 2           # 2: Topic güzel, sağlıklı (Binary CDR doğrulandı)
ACK_TRIGGER_STREAM = 3       # 3: Topic geliyor, tam çözünürlük H.264 tetiklemesi (Wake-up)
ACK_STANDBY_SLEEP = 4        # 4: Kamerayı tekrar Mavi Ekran (Standby) moduna al

ACK_STATUS_LABELS = {
    0: "0: TOPIC_YOK (Bulunamadı)",
    1: "1: HATALI_YAPI (Format/Şema Uyumsuz)",
    2: "2: SAGLIKLI (Binary CDR OK)",
    3: "3: TRIGGER_AKTIF (Canlı H.264 Aç)",
    4: "4: STANDBY_MAVI_EKRAN (Uyku Modu)",
}


def get_ack_label(code: int) -> str:
    return ACK_STATUS_LABELS.get(int(code), f"{code}: OZEL_KOD_{code}")


# ============================================================================
# MAVI EKRAN (STANDBY BLUE SCREEN) MİNİ ÇERÇEVE ÜRETİCİSİ (~165 Byte)
# ============================================================================
# 16x16 Saf Mavi (#0044FF) geçerli JPEG binary dizisi (Ağı yormaz, GPS paketi kadardır)
_BLUE_SCREEN_JPEG_B64 = (
    "/9j/4AAQSkZJRgABAQEASABIAAD/2wBDAP//////////////////////////////////////////"
    "////////////////////////////////////////////wgALCAABAAEBAREA/8QAFBABAAAAAAAAAAAAAAAA"
    "AAAAAP/aAAgBAQABPxA="
)

# 8x8 Parlatici Mavi (#0A38DC) minimal geçerli JFIF/JPEG (160 byte)
_SOLID_BLUE_JPEG_HEX = (
    "ffd8ffe000104a46494600010101006000600000ffdb004300080606070605080707070909"
    "080a0c140d0c0b0b0c1912130f141d1a1f1e1d1a1c1c20242e2720222c231c1c2837292c30"
    "313434341f27393d38323c2e333432ffdb0043010909090c0b0c180d0d1832211c21323232"
    "32323232323232323232323232323232323232323232323232323232323232323232323232"
    "32323232323232323232ffc00011080010001003012200021101031101ffc4001f00000105"
    "01010101010100000000000000000102030405060708090a0bffc400b51000020103030204"
    "03050504040000017d01020300041105122131410613516107227114328191a1082342b1c1"
    "1552d1f02433627282090a161718191a25262728292a3435363738393a434445464748494a"
    "535455565758595a636465666768696a737475767778797a838485868788898a9293949596"
    "9798999aa2a3a4a5a6a7a8a9aab2b3b4b5b6b7b8b9bac2c3c4c5c6c7c8c9cad2d3d4d5d6d7"
    "d8d9dae1e2e3e4e5e6e7e8e9eaf1f2f3f4f5f6f7f8f9faffc4001f01000301010101010101"
    "01010000000000000102030405060708090a0bffc400b51100020102040403040705040400"
    "010277000102031104052131061241510761711322328108144291a1b1c109233352f01562"
    "72d10a162434e125f11718191a262728292a35363738393a434445464748494a5354555657"
    "58595a636465666768696a737475767778797a82838485868788898a92939495969798999a"
    "a2a3a4a5a6a7a8a9aab2b3b4b5b6b7b8b9bac2c3c4c5c6c7c8c9cad2d3d4d5d6d7d8d9dae2"
    "e3e4e5e6e7e8e9eaf2f3f4f5f6f7f8f9faffda000c03010002110311003f00bf4005dc9a50"
    "0000ffd9"
)


def get_standby_blue_screen_bytes() -> bytes:
    """Kameranın Standby modunda yayınlayacağı çok hafif Mavi Ekran (Blue Screen) verisi."""
    return bytes.fromhex(_SOLID_BLUE_JPEG_HEX)


# ============================================================================
# TEMEL CDR OKUMA / YAZMA YARDIMCILARI (OMG CDR Standardı)
# ============================================================================
class CDRWriter:
    def __init__(self):
        self.buf = bytearray(CDR_LE_HEADER)

    def _align(self, alignment: int):
        # CDR hizalaması 4 byte'lık encapsulation başlığından sonraki ofsete göre yapılır
        data_len = len(self.buf) - 4
        rem = data_len % alignment
        if rem != 0:
            self.buf.extend(b"\x00" * (alignment - rem))

    def write_uint8(self, val: int):
        self.buf.extend(struct.pack("<B", int(val) & 0xFF))

    def write_int8(self, val: int):
        self.buf.extend(struct.pack("<b", max(-128, min(127, int(val)))))

    def write_uint16(self, val: int):
        self._align(2)
        self.buf.extend(struct.pack("<H", int(val) & 0xFFFF))

    def write_int32(self, val: int):
        self._align(4)
        self.buf.extend(struct.pack("<i", int(val)))

    def write_uint32(self, val: int):
        self._align(4)
        self.buf.extend(struct.pack("<I", int(val) & 0xFFFFFFFF))

    def write_float32(self, val: float):
        self._align(4)
        self.buf.extend(struct.pack("<f", float(val)))

    def write_float64(self, val: float):
        self._align(8)
        self.buf.extend(struct.pack("<d", float(val)))

    def write_string(self, text: str):
        encoded = str(text or "").encode("utf-8") + b"\x00"
        self.write_uint32(len(encoded))
        self.buf.extend(encoded)

    def write_bytes(self, raw: bytes):
        self.write_uint32(len(raw))
        self.buf.extend(raw)

    def write_header(self, sec: int, nanosec: int, frame_id: str):
        self.write_int32(sec)
        self.write_uint32(nanosec)
        self.write_string(frame_id)

    def to_bytes(self) -> bytes:
        return bytes(self.buf)


class CDRReader:
    def __init__(self, data: bytes):
        self.data = data
        if len(data) >= 4 and data[:4] in (CDR_LE_HEADER, CDR_BE_HEADER):
            self.offset = 4
            self.has_cdr_header = True
        else:
            self.offset = 0
            self.has_cdr_header = False

    def _align(self, alignment: int):
        rel = (self.offset - 4) if self.has_cdr_header else self.offset
        rem = rel % alignment
        if rem != 0:
            self.offset += (alignment - rem)

    def read_uint8(self) -> int:
        val = struct.unpack_from("<B", self.data, self.offset)[0]
        self.offset += 1
        return val

    def read_int8(self) -> int:
        val = struct.unpack_from("<b", self.data, self.offset)[0]
        self.offset += 1
        return val

    def read_uint16(self) -> int:
        self._align(2)
        val = struct.unpack_from("<H", self.data, self.offset)[0]
        self.offset += 2
        return val

    def read_int32(self) -> int:
        self._align(4)
        val = struct.unpack_from("<i", self.data, self.offset)[0]
        self.offset += 4
        return val

    def read_uint32(self) -> int:
        self._align(4)
        val = struct.unpack_from("<I", self.data, self.offset)[0]
        self.offset += 4
        return val

    def read_float32(self) -> float:
        self._align(4)
        val = struct.unpack_from("<f", self.data, self.offset)[0]
        self.offset += 4
        return val

    def read_float64(self) -> float:
        self._align(8)
        val = struct.unpack_from("<d", self.data, self.offset)[0]
        self.offset += 8
        return val

    def read_string(self) -> str:
        length = self.read_uint32()
        if length == 0 or self.offset + length > len(self.data):
            return ""
        raw = self.data[self.offset : self.offset + length]
        self.offset += length
        if raw.endswith(b"\x00"):
            raw = raw[:-1]
        return raw.decode("utf-8", errors="replace")

    def read_bytes(self) -> bytes:
        length = self.read_uint32()
        if length == 0 or self.offset + length > len(self.data):
            return b""
        raw = self.data[self.offset : self.offset + length]
        self.offset += length
        return raw

    def read_header(self) -> Dict[str, Any]:
        sec = self.read_int32()
        nanosec = self.read_uint32()
        frame_id = self.read_string()
        return {"stamp": {"sec": sec, "nanosec": nanosec}, "frame_id": frame_id}


def _extract_stamp(msg: Dict[str, Any]) -> Tuple[int, int, str]:
    now = time.time()
    sec = int(now)
    nanosec = int((now - sec) * 1e9)
    frame_id = "base_link"
    if isinstance(msg, dict) and "header" in msg and isinstance(msg["header"], dict):
        hdr = msg["header"]
        frame_id = str(hdr.get("frame_id", frame_id))
        st = hdr.get("stamp", {})
        if isinstance(st, dict):
            sec = int(st.get("sec", sec))
            nanosec = int(st.get("nanosec", nanosec))
    return sec, nanosec, frame_id


# ============================================================================
# 1. sensor_msgs/msg/Imu (Binary CDR <-> Dict)
# ============================================================================
def serialize_imu_cdr(msg: Dict[str, Any]) -> bytes:
    w = CDRWriter()
    sec, nanosec, frame_id = _extract_stamp(msg)
    w.write_header(sec, nanosec, frame_id or "imu_link")

    q = msg.get("orientation", {}) if isinstance(msg, dict) else {}
    w.write_float64(q.get("x", 0.0))
    w.write_float64(q.get("y", 0.0))
    w.write_float64(q.get("z", 0.0))
    w.write_float64(q.get("w", 1.0))

    av = msg.get("angular_velocity", {}) if isinstance(msg, dict) else {}
    w.write_float64(av.get("x", 0.0))
    w.write_float64(av.get("y", 0.0))
    w.write_float64(av.get("z", 0.0))

    la = msg.get("linear_acceleration", {}) if isinstance(msg, dict) else {}
    w.write_float64(la.get("x", 0.0))
    w.write_float64(la.get("y", 0.0))
    w.write_float64(la.get("z", -9.80665))
    return w.to_bytes()


def deserialize_imu_cdr(data: bytes) -> Dict[str, Any]:
    r = CDRReader(data)
    header = r.read_header()
    ox, oy, oz, ow = r.read_float64(), r.read_float64(), r.read_float64(), r.read_float64()
    avx, avy, avz = r.read_float64(), r.read_float64(), r.read_float64()
    lax, lay, laz = r.read_float64(), r.read_float64(), r.read_float64()
    return {
        "header": header,
        "orientation": {"x": round(ox, 6), "y": round(oy, 6), "z": round(oz, 6), "w": round(ow, 6)},
        "angular_velocity": {"x": round(avx, 5), "y": round(avy, 5), "z": round(avz, 5)},
        "linear_acceleration": {"x": round(lax, 5), "y": round(lay, 5), "z": round(laz, 5)},
    }


# ============================================================================
# 2. sensor_msgs/msg/NavSatFix (GPS Binary CDR <-> Dict)
# ============================================================================
def serialize_gps_cdr(msg: Dict[str, Any]) -> bytes:
    w = CDRWriter()
    sec, nanosec, frame_id = _extract_stamp(msg)
    w.write_header(sec, nanosec, frame_id or "gps_link")

    st = msg.get("status", {}) if isinstance(msg, dict) else {}
    status_val = int(st.get("status", 3)) if isinstance(st, dict) else 3
    service_val = int(st.get("service", 1)) if isinstance(st, dict) else 1
    w.write_int8(status_val)
    w.write_uint16(service_val)

    w.write_float64(msg.get("latitude", msg.get("lat", 39.9334)))
    w.write_float64(msg.get("longitude", msg.get("lon", 32.8597)))
    w.write_float64(msg.get("altitude", msg.get("alt", 850.0)))
    return w.to_bytes()


def deserialize_gps_cdr(data: bytes) -> Dict[str, Any]:
    r = CDRReader(data)
    header = r.read_header()
    status_val = r.read_int8()
    service_val = r.read_uint16()
    lat = r.read_float64()
    lon = r.read_float64()
    alt = r.read_float64()
    return {
        "header": header,
        "status": {"status": status_val, "service": service_val},
        "latitude": round(lat, 7),
        "longitude": round(lon, 7),
        "altitude": round(alt, 2),
    }


# ============================================================================
# 3. sensor_msgs/msg/CompressedImage (H.264 / Standby Blue Screen CDR)
# ============================================================================
def serialize_camera_cdr(
    image_bytes: bytes,
    mode: str = "standby_blue",
    codec: str = "h264",
    sec: Optional[int] = None,
    nanosec: Optional[int] = None,
) -> bytes:
    w = CDRWriter()
    now = time.time()
    s = int(now) if sec is None else int(sec)
    ns = int((now - int(now)) * 1e9) if nanosec is None else int(nanosec)
    w.write_header(s, ns, "camera_optical_frame")
    fmt_str = f"{codec}; mode={mode}"
    w.write_string(fmt_str)
    w.write_bytes(image_bytes or get_standby_blue_screen_bytes())
    return w.to_bytes()


def deserialize_camera_cdr(data: bytes) -> Dict[str, Any]:
    if len(data) >= 4 and data[:4] in (CDR_LE_HEADER, CDR_BE_HEADER):
        r = CDRReader(data)
        header = r.read_header()
        fmt_str = r.read_string()
        img_bytes = r.read_bytes()
        mode = "live_h264" if "live" in fmt_str else ("standby_blue" if "standby" in fmt_str else "h264")
        return {
            "header": header,
            "format": fmt_str,
            "mode": mode,
            "size_bytes": len(img_bytes),
            "image_bytes": img_bytes,
        }
    # Geriye dönük ham JPEG / H264 verisi
    mode = "standby_blue" if len(data) <= 350 else "live_h264"
    return {
        "header": {"stamp": {"sec": int(time.time()), "nanosec": 0}, "frame_id": "camera_optical_frame"},
        "format": f"h264; mode={mode}",
        "mode": mode,
        "size_bytes": len(data),
        "image_bytes": data,
    }


# ============================================================================
# 4. pisim_msgs/msg/Channels (ArduPilot Joystick / Port Numaralı Komutlar)
#    Channel 1: Sürüş Tekerlekleri (İleri / Geri)
#    Channel 2: Direksiyon Servoları (Sağ / Sol)
#    Channel 3+: Pervane / Ek Eyleyiciler
# ============================================================================
def normalize_to_pwm(norm_val: float) -> int:
    """[-1.0 .. +1.0] normalize değeri [1000 .. 2000] us ArduPilot PWM sinyaline çevirir."""
    v = max(-1.0, min(1.0, float(norm_val)))
    return int(round(1500.0 + v * 500.0))


def pwm_to_normalized(pwm_val: float) -> float:
    """[1000 .. 2000] PWM sinyalini (veya zaten -1..+1 olan değeri) [-1.0 .. +1.0] aralığına çevirir."""
    val = float(pwm_val)
    if abs(val) <= 2.5:
        return max(-1.0, min(1.0, val))
    return max(-1.0, min(1.0, (val - 1500.0) / 500.0))


def serialize_channels_cdr(channels: Dict[Union[int, str], float]) -> bytes:
    """
    Kanal sözlüğünü ({1: throttle, 2: steering} veya {"1": 1750, "2": 1300})
    Binary CDR formatına paketler.
    """
    w = CDRWriter()
    now = time.time()
    w.write_header(int(now), int((now - int(now)) * 1e9), "rc_channels")

    items = []
    for k, v in (channels or {}).items():
        try:
            ch_num = int(str(k).strip().replace("ch", "").replace("CH", ""))
            f_val = float(v)
            if abs(f_val) <= 2.5:
                norm_val = max(-1.0, min(1.0, f_val))
                pwm_val = normalize_to_pwm(norm_val)
            else:
                pwm_val = max(800, min(2200, int(round(f_val))))
                norm_val = pwm_to_normalized(pwm_val)
            items.append((ch_num, norm_val, pwm_val))
        except Exception:
            continue

    w.write_uint32(len(items))
    for ch_num, norm_val, pwm_val in items:
        w.write_uint16(ch_num)
        w.write_float32(norm_val)
        w.write_uint16(pwm_val)
    return w.to_bytes()


def deserialize_channels_cdr(data: bytes) -> Dict[str, Any]:
    r = CDRReader(data)
    header = r.read_header()
    count = r.read_uint32()
    channels_norm: Dict[str, float] = {}
    channels_pwm: Dict[str, int] = {}
    for _ in range(min(count, 64)):
        ch_num = r.read_uint16()
        norm_val = r.read_float32()
        pwm_val = r.read_uint16()
        channels_norm[str(ch_num)] = round(norm_val, 4)
        channels_pwm[str(ch_num)] = int(pwm_val)
    return {
        "header": header,
        "channels": channels_pwm,
        "channels_normalized": channels_norm,
    }


# ============================================================================
# 5. pisim_msgs/msg/Actuators (Kemik/Motor Haritası Binary CDR)
# ============================================================================
def serialize_actuators_cdr(motors: Dict[str, float]) -> bytes:
    w = CDRWriter()
    now = time.time()
    w.write_header(int(now), int((now - int(now)) * 1e9), "actuators")
    items = list((motors or {}).items())
    w.write_uint32(len(items))
    for name, val in items:
        w.write_string(str(name))
        w.write_float32(float(val))
    return w.to_bytes()


def deserialize_actuators_cdr(data: bytes) -> Dict[str, float]:
    r = CDRReader(data)
    _ = r.read_header()
    count = r.read_uint32()
    motors: Dict[str, float] = {}
    for _ in range(min(count, 128)):
        name = r.read_string()
        val = r.read_float32()
        if name:
            motors[name] = round(val, 4)
    return motors


# ============================================================================
# 6. geometry_msgs/msg/Twist (cmd_vel Binary CDR)
# ============================================================================
def serialize_twist_cdr(msg: Dict[str, Any]) -> bytes:
    w = CDRWriter()
    lin = msg.get("linear", {}) if isinstance(msg, dict) else {}
    ang = msg.get("angular", {}) if isinstance(msg, dict) else {}
    lx = float(lin.get("x", msg.get("linear_x", 0.0)))
    ly = float(lin.get("y", 0.0))
    lz = float(lin.get("z", 0.0))
    ax = float(ang.get("x", 0.0))
    ay = float(ang.get("y", 0.0))
    az = float(ang.get("z", msg.get("angular_z", 0.0)))
    for v in (lx, ly, lz, ax, ay, az):
        w.write_float64(v)
    return w.to_bytes()


def deserialize_twist_cdr(data: bytes) -> Dict[str, Any]:
    r = CDRReader(data)
    lx, ly, lz = r.read_float64(), r.read_float64(), r.read_float64()
    ax, ay, az = r.read_float64(), r.read_float64(), r.read_float64()
    return {
        "linear": {"x": round(lx, 4), "y": round(ly, 4), "z": round(lz, 4)},
        "angular": {"x": round(ax, 4), "y": round(ay, 4), "z": round(az, 4)},
    }


# ============================================================================
# 7. pisim_msgs/msg/Ack (Çift Yönlü Acknowledgement ve Tetikleme Paketi)
#    Kodlar:
#      0 -> Topic yok / bulunamadı
#      1 -> Topic var ama veri istenilen yapıda değil
#      2 -> Topic güzel, sağlıklı
#      3 -> Trigger (Kamerayı tam çözünürlük H.264 modunda aç)
#      4 -> Standby (Kamerayı Mavi Ekran moduna geri al)
# ============================================================================
def serialize_ack_cdr(
    vehicle_code: str,
    topic_statuses: Dict[str, int],
    source: str = "pi5",
) -> bytes:
    w = CDRWriter()
    now = time.time()
    w.write_header(int(now), int((now - int(now)) * 1e9), source)
    w.write_string(vehicle_code)
    items = list((topic_statuses or {}).items())
    w.write_uint32(len(items))
    for topic_name, status_code in items:
        w.write_string(str(topic_name))
        w.write_int32(int(status_code))
    return w.to_bytes()


def deserialize_ack_cdr(data: bytes) -> Dict[str, Any]:
    r = CDRReader(data)
    header = r.read_header()
    vehicle_code = r.read_string()
    count = r.read_uint32()
    topics: Dict[str, int] = {}
    topics_labeled: Dict[str, str] = {}
    for _ in range(min(count, 128)):
        t_name = r.read_string()
        code = r.read_int32()
        if t_name:
            topics[t_name] = code
            topics_labeled[t_name] = get_ack_label(code)
    return {
        "header": header,
        "vehicle_code": vehicle_code,
        "source": header.get("frame_id", "unknown"),
        "topics": topics,
        "topics_labeled": topics_labeled,
    }


# ============================================================================
# EVRENSEL (UNIVERSAL) SERIALIZE / DESERIALIZE VE INSPECTOR FONKSİYONLARI
# ============================================================================
def is_cdr_binary(raw_bytes: bytes) -> bool:
    return isinstance(raw_bytes, (bytes, bytearray)) and len(raw_bytes) >= 4 and bytes(raw_bytes[:4]) in (CDR_LE_HEADER, CDR_BE_HEADER)


def serialize_for_topic(topic: str, payload: Any) -> bytes:
    """
    Topic adına göre veriyi otomatik olarak ROS 2 Binary CDR byte dizisine çevirir.
    """
    t_low = topic.lower().strip("/")

    if isinstance(payload, (bytes, bytearray)):
        if is_cdr_binary(payload):
            return bytes(payload)
        if "camera" in t_low or "h264" in t_low:
            mode = "standby_blue" if len(payload) <= 350 else "live_h264"
            return serialize_camera_cdr(bytes(payload), mode=mode, codec="h264")
        return bytes(payload)

    if isinstance(payload, str):
        try:
            payload = json.loads(payload)
        except Exception:
            w = CDRWriter()
            w.write_string(payload)
            return w.to_bytes()

    if not isinstance(payload, dict):
        w = CDRWriter()
        w.write_string( json.dumps(payload))
        return w.to_bytes()

    if t_low.endswith("/imu") or t_low == "imu":
        return serialize_imu_cdr(payload)
    elif t_low.endswith("/gps") or t_low == "gps":
        return serialize_gps_cdr(payload)
    elif "camera" in t_low or "h264" in t_low:
        b64_data = payload.get("data", "")
        raw_img = base64.b64decode(b64_data) if isinstance(b64_data, str) and b64_data else get_standby_blue_screen_bytes()
        mode = payload.get("mode", "standby_blue" if len(raw_img) <= 350 else "live_h264")
        return serialize_camera_cdr(raw_img, mode=mode, codec=payload.get("format", "h264"))
    elif t_low.endswith("/channels") or t_low == "channels" or t_low.endswith("/ardupilot/pwm") or t_low.endswith("/pwm"):
        ch_map = payload.get("channels", payload)
        return serialize_channels_cdr(ch_map)
    elif t_low.endswith("/actuators") or t_low == "actuators":
        m_map = payload.get("motors", payload)
        return serialize_actuators_cdr(m_map)
    elif t_low.endswith("/cmd_vel") or t_low == "cmd_vel":
        return serialize_twist_cdr(payload)
    elif t_low.endswith("/ack") or t_low == "ack":
        v_code = payload.get("vehicle_code", payload.get("vehicle", "rover"))
        t_map = payload.get("topics", payload)
        src = payload.get("source", "unreal")
        return serialize_ack_cdr(v_code, t_map, source=src)
    else:
        # Genel CDR zarflı JSON string
        w = CDRWriter()
        w.write_string(json.dumps(payload))
        return w.to_bytes()


def deserialize_from_topic(topic: str, raw_bytes: Union[bytes, str]) -> Tuple[Dict[str, Any], int]:
    """
    Zenoh ağından gelen Binary CDR (veya yedek JSON) paketini çözer.
    Dönüş: (decoded_dict, ack_status_code)
      ack_status_code = 2 -> Sağlıklı Binary CDR
      ack_status_code = 1 -> Yapı/Şema Hatası veya JSON fallback
    """
    if isinstance(raw_bytes, str):
        raw_bytes = raw_bytes.encode("utf-8")

    t_low = topic.lower().strip("/")

    if is_cdr_binary(raw_bytes):
        try:
            if t_low.endswith("/imu") or t_low == "imu":
                return deserialize_imu_cdr(raw_bytes), ACK_HEALTHY_OK
            elif t_low.endswith("/gps") or t_low == "gps":
                return deserialize_gps_cdr(raw_bytes), ACK_HEALTHY_OK
            elif "camera" in t_low or "h264" in t_low:
                return deserialize_camera_cdr(raw_bytes), ACK_HEALTHY_OK
            elif t_low.endswith("/channels") or t_low == "channels" or t_low.endswith("/ardupilot/pwm") or t_low.endswith("/pwm"):
                return deserialize_channels_cdr(raw_bytes), ACK_HEALTHY_OK
            elif t_low.endswith("/actuators") or t_low == "actuators":
                return deserialize_actuators_cdr(raw_bytes), ACK_HEALTHY_OK
            elif t_low.endswith("/cmd_vel") or t_low == "cmd_vel":
                return deserialize_twist_cdr(raw_bytes), ACK_HEALTHY_OK
            elif t_low.endswith("/ack") or t_low == "ack":
                return deserialize_ack_cdr(raw_bytes), ACK_HEALTHY_OK
            else:
                r = CDRReader(raw_bytes)
                txt = r.read_string()
                return json.loads(txt), ACK_HEALTHY_OK
        except Exception as e:
            return {"error": f"CDR decode error: {e}"}, ACK_INVALID_SCHEMA

    # Eğer ham kamera çerçevesi ise (JPEG magic bytes \xff\xd8)
    if ("camera" in t_low or "h264" in t_low) and len(raw_bytes) >= 2 and raw_bytes[:2] == b"\xff\xd8":
        return deserialize_camera_cdr(raw_bytes), ACK_HEALTHY_OK

    # JSON Fallback kontrolü
    try:
        decoded_json = json.loads(raw_bytes.decode("utf-8", errors="strict"))
        if isinstance(decoded_json, dict):
            return decoded_json, ACK_HEALTHY_OK
    except Exception:
        pass

    return {"raw_len": len(raw_bytes), "error": "Unrecognized schema"}, ACK_INVALID_SCHEMA


def format_hex_dump(raw_bytes: bytes, max_bytes: int = 64) -> str:
    """Binary CDR verisini okunaklı Hex + ASCII tablosu olarak biçimlendirir."""
    slice_b = raw_bytes[:max_bytes]
    hex_part = " ".join(f"{b:02X}" for b in slice_b)
    if len(raw_bytes) > max_bytes:
        hex_part += f" ... (+{len(raw_bytes) - max_bytes} byte)"
    return hex_part


def inspect_cdr_packet(topic: str, raw_bytes: bytes) -> Dict[str, Any]:
    """
    Hem Unreal Zenoh Bridge hem de Pi 5 Inspector için bir paketin:
      1) Binary CDR Hex / Byte detaylarını
      2) Çözümlenmiş JSON karşılığını
      3) ACK sağlık kodunu (0, 1, 2, 3, 4)
    tek seferde üretir.
    """
    if isinstance(raw_bytes, str):
        raw_bytes = raw_bytes.encode("utf-8")

    cdr_flag = is_cdr_binary(raw_bytes)
    decoded, ack_code = deserialize_from_topic(topic, raw_bytes)

    # JSON gösteriminde devasa image_bytes dizisini gizleyip özet yaz
    json_view = dict(decoded) if isinstance(decoded, dict) else {"value": decoded}
    if "image_bytes" in json_view:
        img_b = json_view.pop("image_bytes")
        json_view["image_bytes_summary"] = f"<{len(img_b)} bytes H.264/Frame payload>"

    return {
        "topic": topic,
        "is_binary_cdr": cdr_flag,
        "cdr_header": "00 01 00 00 (CDR_LE)" if cdr_flag else "RAW/JSON",
        "byte_size": len(raw_bytes),
        "hex_preview": format_hex_dump(raw_bytes, max_bytes=48),
        "ack_code": ack_code,
        "ack_label": get_ack_label(ack_code),
        "json": json_view,
    }

