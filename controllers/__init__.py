"""
PiSim 4-Level Hierarchical Control Architecture (controllers/)
=============================================================
Organizes autonomous vehicle control algorithms into 4 clean tiers:
- level1_atomic: Low-level atomic primitives (drive distance, constant-radius turn, roll hold, altitude change).
- level2_navigation: Mid-level navigation behaviors (go to coordinate, dynamic target following, U-turn).
- level3_perception: On-demand H.264 vision + LiDAR obstacle/target processing.
- level4_mission_swarm: High-level mission planning & multi-vehicle Zenoh swarm orchestration.
"""

from .level1_atomic.atomic_motion import AtomicMotionController
from .level2_navigation.waypoint_and_follow import WaypointAndFollowController
from .level3_perception.vision_lidar_processor import CameraPerceptionProcessor, LidarObstacleProcessor
from .level4_mission_swarm.swarm_mission_planner import SwarmMissionPlanner

__all__ = [
    "AtomicMotionController",
    "WaypointAndFollowController",
    "CameraPerceptionProcessor",
    "LidarObstacleProcessor",
    "SwarmMissionPlanner",
]
