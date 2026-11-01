#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
swarm_context.py - PiSim Merkeziyetsiz Sürü ve Sanal Merkez Soyutlama Katmanı
==============================================================================
Bu modül, araçlar arasındaki merkeziyetsiz (P2P Mesh / UDP Broadcast) iletişimi
soyutlayarak, geliştiricinin sanki tek bir merkezi sunucu varmış gibi rahatça
görev yazmasını sağlar.

Özellikler:
  1. Port 7405 (SWARM_MISSION_PORT) üzerinden görev planlarını ve adım durumlarını yayınlar/dinler.
  2. Arka plan dinleyici iş parçacığı (Background Worker) ile diğer sürü üyelerinin
     durumlarını (Hangi adımda? Kalan mesafe ne?) anlık RAM önbelleğinde tutar.
  3. `wait_for_vehicle("RV02", condition="at_step", step=2)` gibi yüksek seviyeli
     koşul bekleme fonksiyonları sunar.
  4. Görev olaylarını yerel SQLite veritabanına (pisim_telemetry.db) senkronize eder.
"""

import socket
import json
import time
import threading
from typing import Optional, Dict, Any, Callable
import telemetry_db


SWARM_MISSION_PORT = 7405


class SwarmContext:
    def __init__(self, my_vehicle_code: str = "RV01", broadcast_port: int = SWARM_MISSION_PORT):
        self.my_vehicle_code = str(my_vehicle_code).upper()
        self.port = int(broadcast_port)
        self.peers: Dict[str, Dict[str, Any]] = {}
        self.active_mission: Optional[Dict[str, Any]] = None
        self._lock = threading.Lock()
        self._abort_requested = False
        self._running = True

        # UDP Yayın (TX) Soketi
        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.tx_sock.settimeout(0.2)

        # UDP Dinleyici (RX) Soketi
        self.rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.rx_sock.bind(("0.0.0.0", self.port))
            print(f"📡 [SwarmContext] Sürü Görev Portu Dinleniyor: UDP {self.port} (Araç: {self.my_vehicle_code})")
        except Exception as e:
            print(f"⚠️ [SwarmContext] Port {self.port} dinleme uyarısı (Paylaşımlı mod): {e}")

        # Arka plan dinleyici iş parçacığı
        self.rx_thread = threading.Thread(target=self._listener_loop, daemon=True, name="SwarmListener")
        self.rx_thread.start()

    def _listener_loop(self):
        """Ağdan gelen Port 7405 sürü paketlerini kesintisiz dinler."""
        while self._running:
            try:
                data, addr = self.rx_sock.recvfrom(65535)
                payload = json.loads(data.decode("utf-8", errors="ignore").strip())
                self._handle_swarm_packet(payload, addr)
            except (socket.timeout, BlockingIOError):
                continue
            except Exception:
                time.sleep(0.05)

    def _handle_swarm_packet(self, packet: dict, addr: tuple):
        """Gelen sürü paketini türüne göre işler ve RAM önbelleğini günceller."""
        pkt_type = packet.get("type", "")
        sender_code = str(packet.get("vehicle", packet.get("vehicle_code", ""))).upper()

        now = time.time()
        with self._lock:
            # 1. Görev Planı Dağıtımı (Yeni bir görev ağa sunulduğunda)
            if pkt_type == "mission_plan":
                mission = packet.get("mission", {})
                m_veh = str(mission.get("vehicle_code", sender_code)).upper()
                if m_veh == self.my_vehicle_code:
                    self.active_mission = mission
                # Veritabanına da kaydet (Her araç diğerinin tüm planını yerel SQL'de bilir!)
                telemetry_db.save_mission_plan(mission)
                telemetry_db.record_swarm_event(m_veh, "MISSION_RECEIVED", f"Görev planı alındı: {mission.get('mission_id')}")

            # 2. Adım Ping'i (Bir araç adım değiştirdiğinde veya periyodik anons ettiğinde)
            elif pkt_type == "step_ping":
                if sender_code:
                    self.peers[sender_code] = {
                        "last_seen": now,
                        "mission_id": packet.get("mission_id", ""),
                        "step": int(packet.get("step", 0)),
                        "state": packet.get("state", "ACTIVE"),
                        "status": packet.get("status", ""),
                        "rem_dist_m": float(packet.get("rem_dist_m", 0.0)),
                        "lat": float(packet.get("lat", 0.0)),
                        "lon": float(packet.get("lon", 0.0)),
                        "ip": addr[0]
                    }
                    # Yerel veritabanında bu eşin durumunu güncelle
                    telemetry_db.update_mission_step_state(
                        packet.get("mission_id", ""),
                        int(packet.get("step", 0)),
                        packet.get("state", "ACTIVE"),
                        sender_code
                    )

            # 3. Acil Durdurma / Görev İptal Anonsu
            elif pkt_type == "mission_abort":
                target_veh = str(packet.get("target_vehicle", "ALL")).upper()
                if target_veh in (self.my_vehicle_code, "ALL"):
                    self._abort_requested = True
                    print(f"🛑 [SwarmContext] AĞDAN ACİL DURDURMA ALINDI! ({packet.get('reason', '')})")
                    telemetry_db.record_swarm_event(sender_code, "MISSION_ABORTED", packet.get("reason", ""))

    def broadcast_mission_plan(self, mission_dict: dict):
        """
        Tüm görev planını Port 7405 üzerinden 1 KEZ yayınlar ve yerel SQL'e kaydeder.
        Böylece ağ gereksiz yere boğulmaz; tüm sürü ve Dashboard planı tek seferde öğrenir.
        """
        mission_dict["vehicle_code"] = self.my_vehicle_code
        self.active_mission = mission_dict
        telemetry_db.save_mission_plan(mission_dict)

        payload = {
            "type": "mission_plan",
            "vehicle": self.my_vehicle_code,
            "mission": mission_dict,
            "ts": time.time()
        }
        self._send_broadcast(payload)
        print(f"📡 [SwarmContext] Görev Planı Ağda Yayınlandı (Port {self.port}): {mission_dict.get('mission_id')}")

    def report_step(self, step_index: int, state: str = "ACTIVE", status_msg: str = "", rem_dist_m: float = 0.0, lat: float = 0.0, lon: float = 0.0):
        """
        Aracın o anki adım durumunu hafif bir ping olarak ağa atar ve yerel SQL'i günceller.
        state: 'ACTIVE', 'PASSED', 'FAILED', 'COMPLETED'
        """
        m_id = self.active_mission.get("mission_id", "") if self.active_mission else "DEFAULT"
        telemetry_db.update_mission_step_state(m_id, step_index, state, self.my_vehicle_code)

        payload = {
            "type": "step_ping",
            "vehicle": self.my_vehicle_code,
            "mission_id": m_id,
            "step": int(step_index),
            "state": str(state),
            "status": str(status_msg),
            "rem_dist_m": round(float(rem_dist_m), 2),
            "lat": round(float(lat), 6),
            "lon": round(float(lon), 6),
            "ts": time.time()
        }
        self._send_broadcast(payload)

    def abort_mission(self, reason: str = "Kullanıcı İptali"):
        """Görevi iptal eder ve ağa acil durdurma anonsu yapar."""
        self._abort_requested = True
        m_id = self.active_mission.get("mission_id", "") if self.active_mission else ""
        if m_id:
            telemetry_db.update_mission_step_state(m_id, 0, "ABORTED", self.my_vehicle_code)

        payload = {
            "type": "mission_abort",
            "vehicle": self.my_vehicle_code,
            "target_vehicle": self.my_vehicle_code,
            "mission_id": m_id,
            "reason": reason,
            "ts": time.time()
        }
        self._send_broadcast(payload)

    def is_abort_requested(self) -> bool:
        """Kullanıcı veya ağ tarafından acil durdurma emri verildi mi?"""
        return self._abort_requested

    def wait_for_vehicle(
        self,
        target_vehicle_code: str,
        condition: str = "at_step",
        step: int = 1,
        timeout_sec: float = 60.0,
        status_callback: Optional[Callable[[str], None]] = None
    ) -> bool:
        """
        [SANAL MERKEZ SOYUTLAMA FONKSİYONU]
        Belirtilen araç belirtilen koşulu sağlayana kadar asenkron bekler.
        Koşul sağlandığında True döner; zaman aşımı veya Abort gelirse False döner.
        """
        target = str(target_vehicle_code).upper()
        t0 = time.time()
        last_log = 0.0

        if status_callback:
            status_callback(f"⏳ [SÜRÜ BEKLEMESİ] {target} aracının '{condition}' (Adım: {step}) durumuna geçmesi bekleniyor (Maks: {timeout_sec}s)...")

        while not self._abort_requested and (time.time() - t0) < timeout_sec:
            with self._lock:
                peer = self.peers.get(target)

            if peer:
                p_step = peer.get("step", -1)
                p_state = peer.get("state", "")

                # 1. Koşul: Belirli bir adıma ulaşmış veya o adımı geçmiş mi?
                if condition in ("at_step", "step_reached", "at_waypoint"):
                    if p_step > step or (p_step == step and p_state in ("ACTIVE", "PASSED", "COMPLETED")):
                        if status_callback:
                            status_callback(f"✅ [SÜRÜ ŞARTI SAĞLANDI] {target} aracı Adım {step}'e ulaştı! Göreve devam ediliyor.")
                        return True

                # 2. Koşul: Belirli adımı tamamlamış (PASSED) mı?
                elif condition in ("step_completed", "passed_step"):
                    if p_step > step or (p_step == step and p_state in ("PASSED", "COMPLETED")):
                        if status_callback:
                            status_callback(f"✅ [SÜRÜ ŞARTI SAĞLANDI] {target} aracı Adım {step}'i tamamladı!")
                        return True

            if status_callback and (time.time() - last_log >= 2.0):
                last_log = time.time()
                elapsed = int(time.time() - t0)
                status_callback(f"⏳ [SÜRÜ BEKLENİYOR] Hedef: {target} | Geçen Süre: {elapsed}s / {timeout_sec}s")

            time.sleep(0.2)

        if self._abort_requested:
            if status_callback:
                status_callback("🛑 [SÜRÜ BEKLEMESİ İPTAL] Görev durduruldu.")
            return False

        if status_callback:
            status_callback(f"⚠️ [SÜRÜ ZAMAN AŞIMI] {target} aracı beklenen süre ({timeout_sec}s) içinde yanıt vermedi.")
        return False

    def get_peer_state(self, vehicle_code: str) -> Optional[Dict[str, Any]]:
        """Ağdaki herhangi bir aracın en son bilinen durumunu döner."""
        with self._lock:
            return self.peers.get(str(vehicle_code).upper())

    def get_all_peers(self) -> Dict[str, Dict[str, Any]]:
        """Tüm filonun o anki durum sözlüğünü döner."""
        with self._lock:
            return dict(self.peers)

    def _send_broadcast(self, payload: dict):
        """Port 7405 üzerinden LAN Broadcast ve Localhost'a gönderir."""
        raw = (json.dumps(payload) + "\n").encode("utf-8")
        for dest_ip in ("255.255.255.255", "127.0.0.1"):
            try:
                self.tx_sock.sendto(raw, (dest_ip, self.port))
            except Exception:
                pass

    def close(self):
        """Bağlantıları kapatır."""
        self._running = False
        try:
            self.tx_sock.close()
            self.rx_sock.close()
        except Exception:
            pass
