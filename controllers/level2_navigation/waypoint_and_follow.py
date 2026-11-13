"""
Level 2 — Navigation & Target Following (level2_navigation/waypoint_and_follow.py)
-----------------------------------------------------------------------------------
Combines Level 1 atomic primitives into geometric navigation behaviors:
- navigate_to_coordinate: High-precision waypoint convergence.
- maintain_relative_position: Keeps a desired body-frame offset relative to a moving leader vehicle.
- execute_u_turn: Automated 180-degree turnaround maneuver.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

from controllers.level1_atomic.atomic_motion import AtomicMotionController, ChannelOutput


@dataclass
class GeoPose:
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    yaw_deg: float = 0.0
    speed_mps: float = 0.0


def _wrap_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def _delta_meters(origin: GeoPose, target: GeoPose) -> Tuple[float, float]:
    """Convert local or WGS84 coordinates cleanly (supports both (0,0) flat world & GPS)."""
    if abs(origin.lat) < 0.05 and abs(origin.lon) < 0.05 and abs(target.lat) < 0.05 and abs(target.lon) < 0.05:
        # Local metric coordinates encoded directly
        return (target.lat - origin.lat) * 111320.0, (target.lon - origin.lon) * 111320.0
    dx_north = (target.lat - origin.lat) * 111320.0
    dy_east = (target.lon - origin.lon) * 111320.0 * max(0.1, math.cos(math.radians(origin.lat)))
    return dx_north, dy_east


class WaypointAndFollowController:
    """Mid-level closed-loop path and formation controller."""

    def __init__(self, atomic: AtomicMotionController | None = None):
        self.atomic = atomic or AtomicMotionController()

    def navigate_to_coordinate(
        self,
        current: GeoPose,
        target: GeoPose,
        tolerance_m: float = 0.6,
        cruise_speed_mps: float = 1.8,
    ) -> ChannelOutput:
        dx, dy = _delta_meters(current, target)
        dist_m = math.hypot(dx, dy)
        if dist_m <= tolerance_m:
            return ChannelOutput(0.0, 0.0, done=True, summary=f"Waypoint reached (dist={dist_m:.2f}m)")

        desired_bearing_deg = math.degrees(math.atan2(dy, dx))
        heading_err_deg = _wrap_deg(desired_bearing_deg - current.yaw_deg)

        if abs(heading_err_deg) > 25.0:
            # Align heading via tight constant-radius turn first
            return self.atomic.turn_constant_radius(
                radius_m=1.2,
                turned_angle_deg=0.0,
                target_angle_deg=heading_err_deg,
                speed_mps=max(0.7, cruise_speed_mps * 0.6),
            )
        return self.atomic.drive_straight_distance(
            distance_traveled_m=0.0,
            target_distance_m=dist_m,
            speed_mps=cruise_speed_mps,
            heading_error_deg=heading_err_deg,
        )

    def maintain_relative_position(
        self,
        follower: GeoPose,
        leader: GeoPose,
        offset_forward_m: float = -3.0,
        offset_right_m: float = 0.0,
    ) -> ChannelOutput:
        """Hold dynamic standoff offset behind/beside a moving target vehicle."""
        yaw_rad = math.radians(leader.yaw_deg)
        north_off = offset_forward_m * math.cos(yaw_rad) - offset_right_m * math.sin(yaw_rad)
        east_off = offset_forward_m * math.sin(yaw_rad) + offset_right_m * math.cos(yaw_rad)

        dlat = north_off / 111320.0
        dlon = east_off / (111320.0 * max(0.1, math.cos(math.radians(leader.lat))))
        virtual_target = GeoPose(
            lat=leader.lat + dlat,
            lon=leader.lon + dlon,
            alt=leader.alt,
            yaw_deg=leader.yaw_deg,
        )
        speed = max(0.8, min(self.atomic.max_speed_mps, leader.speed_mps + 0.5))
        out = self.navigate_to_coordinate(follower, virtual_target, tolerance_m=0.45, cruise_speed_mps=speed)
        out.summary = f"Follow(off=({offset_forward_m:+.1f},{offset_right_m:+.1f})m) | {out.summary}"
        return out

    def execute_u_turn(
        self,
        turned_deg: float,
        turn_radius_m: float = 1.5,
        direction: int = 1,
    ) -> ChannelOutput:
        """Perform an automated 180-degree U-turn."""
        target = 180.0 * (1.0 if direction >= 0 else -1.0)
        return self.atomic.turn_constant_radius(
            radius_m=turn_radius_m,
            turned_angle_deg=turned_deg,
            target_angle_deg=target,
            speed_mps=1.1,
        )
