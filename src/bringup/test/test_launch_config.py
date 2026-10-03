"""Evaluate launch arguments and parameters without executing any processes."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

ROOT = Path(__file__).resolve().parents[2]


def description(package, filename):
    spec = importlib.util.spec_from_file_location(
        "launch_under_test", ROOT / package / "launch" / filename
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def defaults(ld, **overrides):
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in ld.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


def parameters(node, context):
    # Inspect normalized ROS parameters; never call Node.execute().
    return evaluate_parameters(context, node._Node__parameters)


def test_mapping_and_localization_are_mutually_exclusive_and_navigation_opt_in():
    ld = description("bringup", "all.launch.py")
    for saved in ("true", "false"):
        context = defaults(ld, use_saved_map=saved)
        conditional = [
            a
            for a in ld.entities
            if isinstance(a, IncludeLaunchDescription) and a.condition is not None
        ]
        assert len(conditional) == 2
        assert sum(a.condition.evaluate(context) for a in conditional) == 1
        assert context.launch_configurations["enable_navigation"] == "false"
    context = defaults(ld, foxglove_port="9876", drive_params_file="/deployment/drive.yaml")
    nodes = [a for a in ld.entities if isinstance(a, Node)]
    bridge = next(a for a in nodes if a.node_package == "foxglove_bridge")
    assert parameters(bridge, context)[0]["port"] == 9876
    assert parameters(bridge, context)[0]["max_qos_depth"] == 100
    drive = next(a for a in nodes if a.node_package == "drive_controller")
    assert str(parameters(drive, context)[0]) == "/deployment/drive.yaml"


def test_camera_forwarding_and_typed_lidar_and_voice_parameters():
    ld = description("senses", "senses.launch.py")
    context = defaults(
        ld,
        camera_device="/dev/video2",
        serial_port="/dev/serial/by-id/lidar",
        audio_output_device_name="USB speaker",
        oww_host="wakeword",
        oww_port="10500",
    )
    camera = next(a for a in ld.entities if isinstance(a, IncludeLaunchDescription))
    assert (
        perform_substitutions(
            context,
            normalize_to_list_of_substitutions(dict(camera.launch_arguments)["camera_device"]),
        )
        == "/dev/video2"
    )
    nodes = [a for a in ld.entities if isinstance(a, Node)]
    voice = next(a for a in nodes if a.node_executable == "voice_agent")
    values = parameters(voice, context)[1]
    assert values["output_device_name"] == "USB speaker"
    assert values["oww_host"] == "wakeword" and values["oww_port"] == 10500
    assert values["max_session_seconds"] == 240.0
    lidar = next(a for a in nodes if a.node_package == "sllidar_ros2")
    values = parameters(lidar, context)[0]
    assert values["serial_port"] == "/dev/serial/by-id/lidar"
    assert values["serial_baudrate"] == 460800
    assert values["inverted"] is False and values["angle_compensate"] is True
    camera_ld = description("senses", "eyes.launch.py")
    camera_context = defaults(camera_ld, camera_device="/dev/video2")
    node = next(
        a for a in camera_ld.entities if isinstance(a, Node) and a.node_package == "v4l2_camera"
    )
    assert parameters(node, camera_context)[0]["video_device"] == "/dev/video2"
