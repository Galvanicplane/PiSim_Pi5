#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
agent_node.py - PiSim Docker / Dağıtık Bağımsız Araç Beyni (Autonomous Swarm Agent)
===================================================================================
Bu script, her biri bağımsız bir Docker konteynerinde veya fiziksel bir Raspberry Pi 5'te çalışan
otonom araç beynini temsil eder.

Roller:
- Hiçbir merkezi SQL veya merkezi koordinatöre bağımlı DEĞİLDİR.
- P2P Zenoh Mesh üzerinden kendi aracının sensörlerini (IMU, GPS, JointStates) dinler.
- VehicleHardware (Ackermann kinematiği) kullanarak ön tekerlek servolarını ve sürüş motorlarını hesaplar.
- Kararını doğrudan 'pisim/<VEHICLE_CODE>/actuators' topic'ine basar.
- Kendi durumunu P2P bellek ağına ('fleet/<VEHICLE_CODE>/state') anlık yazar.
"""

import json
import logging
import os
import signal
import sys
import time

logging.basicConfig(level=logging.INFO, format="[%(asctime)s][%(name)s] %(message)s")

try:
    import zenoh
    HAS_ZENOH = True
except ImportError:
    HAS_ZENOH = False

from vehicle_hardware import VehicleHardware

VEHICLE_CODE = os.environ.get("VEHICLE_CODE", "rover_01")
UE5_HOST = os.environ.get("UE5_HOST", "host.docker.internal")
ZENOH_ROUTER = os.environ.get("ZENOH_ROUTER", None)

logger = logging.getLogger(f"Brain-{VEHICLE_CODE}")


class AutonomousAgentNode:
    def __init__(self, vehicle_code: str):
        self.vehicle_code = vehicle_code
        self.running = False
        self.hardware = VehicleHardware(vehicle_code=vehicle_code, drive_mode="ACKERMANN")
        self.session = None

        # Sensör Telemetri Verileri (Zenoh üzerinden anlık güncellenir)
        self.last_gps = None
        self.last_imu = None
        self.last_joints = None
        self.last_sensor_time = 0.0

    def start(self):
        self.running = True
        logger.info(f"🚀 Otonom Araç Beyni Başlatılıyor: {self.vehicle_code}")

        if HAS_ZENOH:
            conf = zenoh.Config()
            if ZENOH_ROUTER:
                conf.insert_json5("mode", json.dumps("client"))
                conf.insert_json5("connect/endpoints", json.dumps([ZENOH_ROUTER]))
            else:
                conf.insert_json5("mode", json.dumps("peer"))

            self.session = zenoh.open(conf)
            logger.info("✅ Zenoh Oturumu Açıldı.")

            # Kendi aracımıza ait sensör topic'lerini dinle
            self.session.declare_subscriber(f"pisim/{self.vehicle_code}/gps", self._on_gps)
            self.session.declare_subscriber(f"pisim/{self.vehicle_code}/imu", self._on_imu)
            self.session.declare_subscriber(f"pisim/{self.vehicle_code}/joint_states", self._on_joints)
        else:
            logger.warning("Zenoh yüklü değil, bağımsız döngü modunda.")

        # Ana kontrol döngüsü
        self._control_loop()

    def _on_gps(self, sample):
        try:
            self.last_gps = json.loads(sample.payload.to_string())
            self.last_sensor_time = time.time()
        except Exception:
            pass

    def _on_imu(self, sample):
        try:
            self.last_imu = json.loads(sample.payload.to_string())
            self.last_sensor_time = time.time()
        except Exception:
            pass

    def _on_joints(self, sample):
        try:
            self.last_joints = json.loads(sample.payload.to_string())
            self.last_sensor_time = time.time()
        except Exception:
            pass

    def _control_loop(self):
        """Otonom sürüş ve kontrol döngüsü (20 Hz)."""
        logger.info(f"🧠 {self.vehicle_code} Karar Mekanizması Devrede.")
        step = 0

        while self.running:
            now = time.time()
            time_since_sensor = now - self.last_sensor_time

            # Sensör durumu kontrolü
            is_connected = time_since_sensor < 2.0

            # Örnek Ackermann sürüş kararı:
            # Düzenli hafif salınım / devriye turu testi
            if is_connected:
                # Hafif sinüs dalgası şeklinde direksiyon dönüşü (ön servolar döner)
                steer_input = 0.3 * (1.0 if (step // 40) % 2 == 0 else -0.3)
                throttle_input = 0.45
            else:
                # Güvenli duruş
                steer_input = 0.0
                throttle_input = 0.0

            # VehicleHardware üzerinden aktüatörleri hesapla (Ön servolar + Tahrik tekerlekleri)
            motors = self.hardware.compute_actuators(throttle=throttle_input, steering=steer_input)

            # Zenoh üzerinden aktüatörleri Unreal'a bas
            if self.session and is_connected:
                actuator_key = f"pisim/{self.vehicle_code}/actuators"
                self.session.put(actuator_key, json.dumps(motors))

                # Kendi durumumuzu dağıtık ağa (fleet/<code_code>/state) bildir
                state_key = f"fleet/{self.vehicle_code}/state"
                state_data = {
                    "code": self.vehicle_code,
                    "connected": is_connected,
                    "throttle": throttle_input,
                    "steering": steer_input,
                    "motors": motors,
                    "timestamp": now,
                }
                self.session.put(state_key, json.dumps(state_data))

            step += 1
            time.sleep(0.05)  # 20 Hz

    def stop(self):
        self.running = False
        if self.session:
            try:
                # Durdururken motorları sıfırla
                motors = self.hardware.compute_actuators(0.0, 0.0)
                self.session.put(f"pisim/{self.vehicle_code}/actuators", json.dumps(motors))
                self.session.close()
            except Exception:
                pass
        logger.info(f"🛑 {self.vehicle_code} kapatıldı.")


def main():
    agent = AutonomousAgentNode(VEHICLE_CODE)

    def handle_sig(sig, frame):
        agent.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sig)
    signal.signal(signal.SIGTERM, handle_sig)
    agent.start()


if __name__ == "__main__":
    main()

