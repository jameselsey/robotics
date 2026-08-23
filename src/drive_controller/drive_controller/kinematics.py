"""Pure differential-drive kinematics used by the hardware-facing node."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose2D:
    """Planar robot pose in metres and radians."""

    x: float
    y: float
    theta: float


def metres_per_tick(wheel_radius_m: float, counts_per_rev: float) -> float:
    """Return linear wheel travel represented by one encoder count."""
    if wheel_radius_m <= 0.0:
        raise ValueError("wheel_radius_m must be > 0")
    if counts_per_rev <= 0.0:
        raise ValueError("counts_per_rev must be > 0")
    return (2.0 * math.pi * wheel_radius_m) / counts_per_rev


def shortest_angular_delta(current: float, previous: float) -> float:
    """Return the signed change between wrapped angles."""
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


def calibrated_wheel_base(
    current_wheel_base_m: float,
    reported_rotation_rad: float,
    physical_rotation_rad: float = 2.0 * math.pi,
) -> float:
    """Calculate effective wheel separation from a measured physical rotation."""
    if current_wheel_base_m <= 0.0:
        raise ValueError("current_wheel_base_m must be > 0")
    if abs(reported_rotation_rad) <= 0.0:
        raise ValueError("reported_rotation_rad must be non-zero")
    if abs(physical_rotation_rad) <= 0.0:
        raise ValueError("physical_rotation_rad must be non-zero")
    return current_wheel_base_m * abs(reported_rotation_rad / physical_rotation_rad)


def encoder_delta_metres(
    delta_ticks: int,
    metres_per_encoder_tick: float,
    distance_scale: float,
    inverted: bool,
) -> float:
    """Convert a signed encoder delta to calibrated wheel travel."""
    signed_ticks = -delta_ticks if inverted else delta_ticks
    return signed_ticks * metres_per_encoder_tick * distance_scale


def integrate_differential_drive(
    pose: Pose2D,
    left_distance_m: float,
    right_distance_m: float,
    wheel_base_m: float,
) -> tuple[Pose2D, float, float]:
    """Integrate one differential-drive encoder update.

    The integration order intentionally matches the established robot behavior:
    update heading first, then project the centre distance using that heading.
    """
    if wheel_base_m <= 0.0:
        raise ValueError("wheel_base_m must be > 0")

    centre_distance = 0.5 * (left_distance_m + right_distance_m)
    heading_delta = (right_distance_m - left_distance_m) / wheel_base_m
    theta = math.atan2(
        math.sin(pose.theta + heading_delta),
        math.cos(pose.theta + heading_delta),
    )
    return (
        Pose2D(
            x=pose.x + centre_distance * math.cos(theta),
            y=pose.y + centre_distance * math.sin(theta),
            theta=theta,
        ),
        centre_distance,
        heading_delta,
    )
