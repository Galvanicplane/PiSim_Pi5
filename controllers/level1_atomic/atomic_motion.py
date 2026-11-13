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

import argparse
import json
import math
import os
import socket
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Union

_ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

import ros2_cdr


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


def publish_channels_topic(
    vehicle_code: str,
    channels: Dict[Union[int, str], float],
    source: str = "level1_atomic",
    host: str = "127.0.0.1",
    port: int = 7402,
    zenoh_session: Any = None,
) -> Dict[str, Any]:
    """Convert normalized [-1..+1] or raw PWM channel values to PWM and publish over Zenoh Binary CDR + UDP 7402."""
    pwm_channels: Dict[str, int] = {}
    for ch_key, val in (channels or {}).items():
        key_str = str(ch_key).strip()
        f_val = float(val)
        if abs(f_val) <= 2.5:
            pwm_channels[key_str] = ros2_cdr.normalize_to_pwm(f_val)
        else:
            pwm_channels[key_str] = max(800, min(2200, int(round(f_val))))

    if zenoh_session is not None:
        try:
            cdr_bytes = ros2_cdr.serialize_channels_cdr(pwm_channels)
            zenoh_session.put(f"pisim/{vehicle_code}/channels", cdr_bytes)
        except Exception:
            pass

    pkt = {
        "topic": f"/{vehicle_code}/channels",
        "cmd": "channels",
        "target": vehicle_code,
        "channels": pwm_channels,
        "source": source,
        "timestamp": time.time(),
    }
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto((json.dumps(pkt) + "\n").encode("utf-8"), (host, int(port)))
        sock.close()
    except Exception:
        pass
    return pkt


def _open_optional_zenoh(use_zenoh: bool) -> Any:
    if not use_zenoh:
        return None
    try:
        import zenoh  # type: ignore
        return zenoh.open(zenoh.Config())
    except Exception:
        return None


def execute_move_distance(
    vehicle_code: str,
    target_distance_m: float = 50.0,
    target_speed_mps: float = 2.0,
    dt: float = 0.1,
    max_steps: int = 0,
    host: str = "127.0.0.1",
    port: int = 7402,
    use_zenoh: bool = False,
    sleep_step: bool = True,
) -> List[Dict[str, Any]]:
    """Run closed/open-loop straight-distance motion and publish real /{vehicle}/channels topic frames."""
    ctrl = AtomicMotionController(max_speed_mps=max(0.5, abs(target_speed_mps)))
    z_sess = _open_optional_zenoh(use_zenoh)
    emitted: List[Dict[str, Any]] = []
    traveled = 0.0
    direction = 1.0 if target_distance_m >= 0.0 else -1.0
    step_idx = 0
    total_limit = max_steps if max_steps > 0 else max(1, int(abs(target_distance_m) / max(0.2, abs(target_speed_mps) * dt)) + 5)

    try:
        while step_idx < total_limit:
            out = ctrl.drive_straight_distance(traveled, abs(target_distance_m), abs(target_speed_mps))
            ch_dict = out.to_channels_dict()
            if direction < 0.0:
                ch_dict["1"] = -abs(ch_dict["1"])
            pkt = publish_channels_topic(
                vehicle_code=vehicle_code,
                channels=ch_dict,
                source="level1_atomic",
                host=host,
                port=port,
                zenoh_session=z_sess,
            )
            emitted.append(pkt)
            if out.done:
                break
            traveled += abs(target_speed_mps) * dt
            step_idx += 1
            if sleep_step:
                time.sleep(dt)

        stop_pkt = publish_channels_topic(
            vehicle_code=vehicle_code,
            channels={"1": 1500, "2": 1500, "3": 1000, "4": 1500},
            source="level1_atomic",
            host=host,
            port=port,
            zenoh_session=z_sess,
        )
        emitted.append(stop_pkt)
    finally:
        if z_sess is not None:
            try:
                z_sess.close()
            except Exception:
                pass
    return emitted


def main() -> None:
    parser = argparse.ArgumentParser(description="PiSim Level 1 Atomic Motion ROS/Zenoh Publisher")
    parser.add_argument("--vehicle", default="RV01", help="Target vehicle code (e.g. RV01)")
    parser.add_argument("--distance", type=float, default=50.0, help="Straight drive distance in meters")
    parser.add_argument("--speed", type=float, default=2.0, help="Target speed (m/s)")
    parser.add_argument("--turn-deg", type=float, default=0.0, help="Optional constant-radius turn angle (deg)")
    parser.add_argument("--steps", type=int, default=0, help="Max loop iterations (0 = until target)")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7402)
    parser.add_argument("--zenoh", action="store_true", help="Publish Binary CDR directly to Zenoh")
    parser.add_argument("--no-sleep", action="store_true", help="Emit frames without sleeping dt")
    args = parser.parse_args()

    if abs(args.turn_deg) > 1e-3:
        ctrl = AtomicMotionController(max_speed_mps=max(0.5, args.speed))
        out = ctrl.turn_constant_radius(radius_m=2.0, turned_angle_deg=0.0, target_angle_deg=args.turn_deg, speed_mps=args.speed)
        pkt = publish_channels_topic(args.vehicle, out.to_channels_dict(), source="level1_atomic", host=args.host, port=args.port)
        print(json.dumps({"mode": "turn", "summary": out.summary, "packet": pkt}))
    else:
        pkts = execute_move_distance(
            vehicle_code=args.vehicle,
            target_distance_m=args.distance,
            target_speed_mps=args.speed,
            max_steps=args.steps,
            host=args.host,
            port=args.port,
            use_zenoh=args.zenoh,
            sleep_step=not args.no_sleep,
        )
        print(json.dumps({"mode": "distance", "frames": len(pkts), "first_packet": pkts[0] if pkts else None, "last_packet": pkts[-1] if pkts else None}))


if __name__ == "__main__":
    main()

