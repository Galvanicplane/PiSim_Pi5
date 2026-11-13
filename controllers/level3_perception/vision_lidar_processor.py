"""
Level 3 — Perception: Vision (H.264 ACK=3/4) & LiDAR Processing
---------------------------------------------------------------
Handles active sensor ingestion from Zenoh topics:
- CameraPerceptionProcessor: Sends ACK=3 to wake camera from Blue Screen Standby, checks frame bytes/header,
  and extracts target offset / obstacle alert metrics. Sends ACK=4 when perception goes idle.
- LidarObstacleProcessor: Evaluates sector clearances (front/left/right) and computes avoidance steering.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass
from typing import Dict, List

_ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

from controllers.level1_atomic.atomic_motion import publish_channels_topic


@dataclass
class PerceptionReport:
    is_blue_standby: bool = True
    closest_front_m: float = 50.0
    closest_left_m: float = 50.0
    closest_right_m: float = 50.0
    avoidance_steer_norm: float = 0.0
    obstacle_blocked: bool = False
    summary: str = ""


class CameraPerceptionProcessor:
    """Controls on-demand camera H.264 wake-up (ACK=3) and evaluates incoming frame bytes."""

    def build_camera_ack_payload(self, vehicle_code: str, active: bool) -> Dict[str, int]:
        code = 3 if active else 4
        return {f"/{vehicle_code}/camera/h264": code}

    def analyze_frame_bytes(self, frame_bytes: bytes) -> Dict[str, object]:
        """Inspect JPEG/H.264 payload size and detect standby blue screen vs real scene frame."""
        size = len(frame_bytes)
        is_standby = size <= 1024
        return {
            "bytes": size,
            "is_blue_standby": is_standby,
            "stream_mode": "STANDBY_BLUE (4)" if is_standby else "LIVE_H264 (3)",
        }


class LidarObstacleProcessor:
    """Computes front/left/right sector clearances from 360-deg LiDAR distance arrays."""

    def __init__(self, safety_stop_m: float = 1.4, avoid_trigger_m: float = 3.5):
        self.safety_stop_m = safety_stop_m
        self.avoid_trigger_m = avoid_trigger_m

    def evaluate_scan(self, ranges_m: List[float]) -> PerceptionReport:
        if not ranges_m:
            return PerceptionReport(summary="No LiDAR rays")

        n = len(ranges_m)
        front_vals = [ranges_m[i] for i in range(n) if i < n * 0.12 or i > n * 0.88]
        left_vals = [ranges_m[i] for i in range(n) if n * 0.65 <= i <= n * 0.88]
        right_vals = [ranges_m[i] for i in range(n) if n * 0.12 <= i <= n * 0.35]

        min_front = min(front_vals) if front_vals else 50.0
        min_left = min(left_vals) if left_vals else 50.0
        min_right = min(right_vals) if right_vals else 50.0

        blocked = min_front < self.safety_stop_m
        avoid_steer = 0.0
        if min_front < self.avoid_trigger_m:
            avoid_steer = -0.65 if min_left > min_right else 0.65

        return PerceptionReport(
            is_blue_standby=False,
            closest_front_m=round(min_front, 2),
            closest_left_m=round(min_left, 2),
            closest_right_m=round(min_right, 2),
            avoidance_steer_norm=avoid_steer,
            obstacle_blocked=blocked,
            summary=f"F={min_front:.1f}m L={min_left:.1f}m R={min_right:.1f}m steer={avoid_steer:+.2f}",
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="PiSim Level 3 Perception ROS/Zenoh Publisher")
    parser.add_argument("--vehicle", default="RV01")
    parser.add_argument("--front-dist", type=float, default=1.2, help="Simulated front clearance (m)")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7402)
    args = parser.parse_args()

    proc = LidarObstacleProcessor()
    rays = [args.front_dist if (i < 4 or i > 31) else 12.0 for i in range(36)]
    rep = proc.evaluate_scan(rays)
    ch1 = 0.0 if rep.obstacle_blocked else 0.4
    pkt = publish_channels_topic(args.vehicle, {"1": ch1, "2": rep.avoidance_steer_norm}, source="level3_perception", host=args.host, port=args.port)
    print(json.dumps({"summary": rep.summary, "blocked": rep.obstacle_blocked, "packet": pkt}))


if __name__ == "__main__":
    main()
