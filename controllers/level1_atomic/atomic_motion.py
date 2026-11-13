"""
Level 1 — Atomic Motion Primitives (level1_atomic/atomic_motion.py)
---------------------------------------------------------------------
Generates normalized ArduPilot channel commands (CH1..CH4 in [-1.0 .. +1.0]):
- Ground vehicles:
    * drive_straight_distance(distance_m, speed_mps)
    * turn_constant_radius(radius_m, angle_deg, speed_mps)
- Aerial vehicles:
    * hold_roll_bank_angle(target_roll_deg, current_roll_deg)
    * change_altitude(target_alt_m, current_alt_m, climb_rate_mps)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict


@dataclass
class ChannelOutput:
    """Normalized [-1.0..+1.0] channel frame sent over Zenoh / Unreal port 7402."""
    ch1_drive_or_roll: float = 0.0
    ch2_steer_or_pitch: float = 0.0
    ch3_throttle: float = 0.0
    ch4_yaw_rudder: float = 0.0
    done: bool = False
    summary: str = ""

    def to_channels_dict(self) -> Dict[str, float]:
        return {
            "1": round(max(-1.0, min(1.0, self.ch1_drive_or_roll)), 4),
            "2": round(max(-1.0, min(1.0, self.ch2_steer_or_pitch)), 4),
            "3": round(max(-1.0, min(1.0, self.ch3_throttle)), 4),
            "4": round(max(-1.0, min(1.0, self.ch4_yaw_rudder)), 4),
        }


class AtomicMotionController:
    """Stateless/lightweight Level-1 actuator primitive generator."""

    def __init__(self, wheelbase_m: float = 0.65, max_speed_mps: float = 3.0, max_steer_deg: float = 35.0):
        self.wheelbase_m = wheelbase_m
        self.max_speed_mps = max_speed_mps
        self.max_steer_deg = max_steer_deg

    def drive_straight_distance(
        self,
        distance_traveled_m: float,
        target_distance_m: float,
        speed_mps: float = 1.5,
        heading_error_deg: float = 0.0,
    ) -> ChannelOutput:
        """Drive straight for a specified distance while stabilizing heading via CH2."""
        remaining = target_distance_m - distance_traveled_m
        if abs(remaining) <= 0.08:
            return ChannelOutput(0.0, 0.0, done=True, summary=f"Arrived {target_distance_m:.2f}m")

        direction = 1.0 if remaining >= 0.0 else -1.0
        slowdown = min(1.0, abs(remaining) / 1.0)
        ch1 = max(-1.0, min(1.0, direction * (abs(speed_mps) / self.max_speed_mps) * slowdown))
        ch2 = max(-1.0, min(1.0, (heading_error_deg / self.max_steer_deg) * 0.8))
        return ChannelOutput(
            ch1_drive_or_roll=ch1,
            ch2_steer_or_pitch=ch2,
            done=False,
            summary=f"Straight remaining={remaining:.2f}m ch1={ch1:+.2f} ch2={ch2:+.2f}",
        )

    def turn_constant_radius(
        self,
        radius_m: float,
        turned_angle_deg: float,
        target_angle_deg: float,
        speed_mps: float = 1.2,
    ) -> ChannelOutput:
        """Execute a constant-radius Ackermann turn (negative target_angle_deg = left, positive = right)."""
        remaining_deg = target_angle_deg - turned_angle_deg
        if abs(remaining_deg) <= 1.5:
            return ChannelOutput(0.0, 0.0, done=True, summary=f"Turn complete {target_angle_deg:+.1f} deg")

        sign = 1.0 if remaining_deg >= 0.0 else -1.0
        steer_rad = math.atan(self.wheelbase_m / max(0.3, abs(radius_m)))
        steer_deg = math.degrees(steer_rad) * sign
        ch2 = max(-1.0, min(1.0, steer_deg / self.max_steer_deg))
        ch1 = max(-1.0, min(1.0, abs(speed_mps) / self.max_speed_mps))
        return ChannelOutput(
            ch1_drive_or_roll=ch1,
            ch2_steer_or_pitch=ch2,
            done=False,
            summary=f"Radius={radius_m:.2f}m rem={remaining_deg:+.1f}deg ch1={ch1:+.2f} ch2={ch2:+.2f}",
        )

    def hold_roll_bank_angle(
        self,
        target_roll_deg: float,
        current_roll_deg: float,
        kp: float = 0.035,
    ) -> ChannelOutput:
        """Hold desired aerial bank angle using ailerons / elevon differential."""
        err = target_roll_deg - current_roll_deg
        roll_cmd = max(-1.0, min(1.0, err * kp))
        return ChannelOutput(
            ch1_drive_or_roll=roll_cmd,
            done=abs(err) < 1.0,
            summary=f"Bank target={target_roll_deg:+.1f}deg curr={current_roll_deg:+.1f}deg roll_cmd={roll_cmd:+.2f}",
        )

    def change_altitude(
        self,
        target_alt_m: float,
        current_alt_m: float,
        cruise_throttle: float = 0.55,
    ) -> ChannelOutput:
        """Adjust aircraft pitch + throttle to climb or descend to target altitude."""
        alt_err = target_alt_m - current_alt_m
        if abs(alt_err) <= 0.5:
            return ChannelOutput(ch2_steer_or_pitch=0.0, ch3_throttle=cruise_throttle, done=True, summary=f"Alt hold {current_alt_m:.1f}m")

        pitch_cmd = max(-0.6, min(0.6, alt_err * 0.05))
        throttle = max(0.2, min(1.0, cruise_throttle + alt_err * 0.03))
        return ChannelOutput(
            ch2_steer_or_pitch=pitch_cmd,
            ch3_throttle=throttle,
            done=False,
            summary=f"Climb target={target_alt_m:.1f}m err={alt_err:+.1f}m pitch={pitch_cmd:+.2f} thr={throttle:.2f}",
        )
