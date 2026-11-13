"""
Level 4 — High-Level Mission & Swarm Coordinator (level4_mission_swarm/swarm_mission_planner.py)
------------------------------------------------------------------------------------------------
Top-tier orchestrator combining Level 3 Perception -> Level 2 Navigation -> Level 1 Atomic primitives
for single-vehicle missions and multi-vehicle Zenoh swarm formations.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional

_ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

from controllers.level1_atomic.atomic_motion import ChannelOutput, publish_channels_topic
from controllers.level2_navigation.waypoint_and_follow import GeoPose, WaypointAndFollowController
from controllers.level3_perception.vision_lidar_processor import LidarObstacleProcessor


@dataclass
class MissionTask:
    vehicle_code: str
    mission_type: str = "idle"  # "idle" | "patrol" | "convoy_follow" | "u_turn"
    waypoints: List[GeoPose] = field(default_factory=list)
    active_wp_idx: int = 0
    leader_code: Optional[str] = None
    follow_offset_forward_m: float = -3.0
    follow_offset_right_m: float = 0.0
    active: bool = False


class SwarmMissionPlanner:
    """High-level mission state machine coordinating fleet vehicles over Zenoh."""

    def __init__(self):
        self.nav = WaypointAndFollowController()
        self.lidar = LidarObstacleProcessor()
        self.tasks: Dict[str, MissionTask] = {}

    def assign_mission(self, task: MissionTask) -> None:
        self.tasks[task.vehicle_code] = task

    def step_vehicle(
        self,
        vehicle_code: str,
        poses: Dict[str, GeoPose],
        lidar_ranges: Optional[List[float]] = None,
    ) -> ChannelOutput:
        task = self.tasks.get(vehicle_code)
        if not task or not task.active or vehicle_code not in poses:
            return ChannelOutput(0.0, 0.0, done=True, summary="No active mission")

        # 1) Level 3 Perception obstacle override check
        if lidar_ranges:
            rep = self.lidar.evaluate_scan(lidar_ranges)
            if rep.obstacle_blocked:
                return ChannelOutput(
                    ch1_drive_or_roll=-0.25,
                    ch2_steer_or_pitch=rep.avoidance_steer_norm,
                    done=False,
                    summary=f"[L3 AVOID] {rep.summary}",
                )

        # 2) Level 2 Navigation -> Level 1 Atomic primitives
        curr = poses[vehicle_code]
        if task.mission_type == "convoy_follow" and task.leader_code and task.leader_code in poses:
            return self.nav.maintain_relative_position(
                curr,
                poses[task.leader_code],
                offset_forward_m=task.follow_offset_forward_m,
                offset_right_m=task.follow_offset_right_m,
            )

        if task.mission_type == "patrol" and task.waypoints:
            wp = task.waypoints[task.active_wp_idx % len(task.waypoints)]
            out = self.nav.navigate_to_coordinate(curr, wp)
            if out.done:
                task.active_wp_idx = (task.active_wp_idx + 1) % len(task.waypoints)
                out.done = False
                out.summary = f"WP advanced -> {task.active_wp_idx}"
            return out

        return ChannelOutput(0.0, 0.0, done=True, summary="Mission complete")


def main() -> None:
    parser = argparse.ArgumentParser(description="PiSim Level 4 Mission & Swarm ROS/Zenoh Publisher")
    parser.add_argument("--vehicle", default="RV01")
    parser.add_argument("--target-lat", type=float, default=39.9260)
    parser.add_argument("--target-lon", type=float, default=32.8668)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7402)
    args = parser.parse_args()

    planner = SwarmMissionPlanner()
    planner.assign_mission(MissionTask(
        vehicle_code=args.vehicle,
        mission_type="patrol",
        waypoints=[GeoPose(lat=args.target_lat, lon=args.target_lon)],
        active=True,
    ))
    poses = {args.vehicle: GeoPose(lat=39.9255, lon=32.8663, yaw_deg=0.0)}
    out = planner.step_vehicle(args.vehicle, poses)
    pkt = publish_channels_topic(args.vehicle, out.to_channels_dict(), source="level4_mission_swarm", host=args.host, port=args.port)
    print(json.dumps({"summary": out.summary, "packet": pkt}))


if __name__ == "__main__":
    main()

