"""Interactive, passive calibration of tracked-robot angular odometry."""

import argparse
import math
from pathlib import Path
import threading

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import yaml

from drive_controller.kinematics import (
    calibrated_wheel_base,
    shortest_angular_delta,
)


def _yaw_from_odometry(message: Odometry) -> float:
    orientation = message.pose.pose.orientation
    sin_yaw = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def _configured_wheel_base(config_path: Path) -> float:
    with config_path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    return float(config["drive_controller"]["ros__parameters"]["wheel_base_m"])


class AngularCalibration(Node):
    """Accumulate unwrapped encoder-odometry yaw without commanding movement."""

    def __init__(self) -> None:
        super().__init__("angular_odometry_calibration")
        self.latest_yaw = None
        self.previous_yaw = None
        self.accumulated_yaw = 0.0
        self.recording = False
        self.create_subscription(Odometry, "/odom", self._odom_callback, 20)

    def _odom_callback(self, message: Odometry) -> None:
        yaw = _yaw_from_odometry(message)
        self.latest_yaw = yaw
        if self.recording and self.previous_yaw is not None:
            self.accumulated_yaw += shortest_angular_delta(yaw, self.previous_yaw)
        self.previous_yaw = yaw

    def start_recording(self) -> None:
        self.accumulated_yaw = 0.0
        self.previous_yaw = self.latest_yaw
        self.recording = True


def _wait_for_enter(stop_event: threading.Event) -> None:
    try:
        input()
    finally:
        stop_event.set()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Observe one physical rotation and calculate effective wheel separation."
    )
    parser.add_argument(
        "--config",
        type=Path,
        required=True,
        help="Drive-controller YAML containing wheel_base_m.",
    )
    args = parser.parse_args()
    current_wheel_base = _configured_wheel_base(args.config)

    rclpy.init()
    node = AngularCalibration()
    try:
        print("Waiting for raw /odom data...")
        while rclpy.ok() and node.latest_yaw is None:
            rclpy.spin_once(node, timeout_sec=0.2)

        print("\nAngular odometry calibration")
        print("- Make sure autonomous navigation is idle.")
        print("- Mark the robot's starting direction on the floor.")
        print("- Hold controller button 6 and rotate slowly in either direction.")
        input("Press Enter immediately before beginning the turn...")
        node.start_recording()

        stop_event = threading.Event()
        print("Rotate exactly ONE physical revolution, release the controls, then press Enter.")
        threading.Thread(
            target=_wait_for_enter,
            args=(stop_event,),
            daemon=True,
        ).start()
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
        node.recording = False

        reported = abs(node.accumulated_yaw)
        physical = 2.0 * math.pi
        if reported < math.pi:
            raise RuntimeError(
                "Recorded less than half a revolution; repeat the calibration."
            )
        recommended = calibrated_wheel_base(
            current_wheel_base,
            reported_rotation_rad=reported,
            physical_rotation_rad=physical,
        )
        print(f"\nCurrent wheel_base_m:     {current_wheel_base:.5f}")
        print(f"Odometry-reported turn:  {math.degrees(reported):.2f} degrees")
        print("Physical turn entered:   360.00 degrees")
        print(f"Recommended wheel_base_m: {recommended:.5f}")
        print("\nDo not edit the configuration yet; send these results back for review.")
    except KeyboardInterrupt:
        print("\nCalibration canceled; no configuration was changed.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
