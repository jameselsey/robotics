"""Structural and orientation regression tests for the robot description."""

from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
XACRO_PATH = PACKAGE_ROOT / "urdf" / "robot.urdf.xacro"
GENERATED_URDF_PATH = PACKAGE_ROOT / "urdf" / "robot.urdf"


def _root(path: Path) -> ET.Element:
    return ET.parse(path).getroot()


def _joint_x(root: ET.Element, joint_name: str) -> float:
    joint = root.find(f"./joint[@name='{joint_name}']")
    assert joint is not None, f"missing joint {joint_name}"
    origin = joint.find("origin")
    assert origin is not None
    return float(origin.attrib["xyz"].split()[0])


def test_red_drive_wheels_are_front_and_green_idlers_are_rear():
    root = _root(XACRO_PATH)
    assert _joint_x(root, "left_wheel_joint") > 0.0
    assert _joint_x(root, "right_wheel_joint") > 0.0
    assert _joint_x(root, "left_idler_joint") < 0.0
    assert _joint_x(root, "right_idler_joint") < 0.0


def test_tall_enclosure_is_at_the_rear():
    root = _root(XACRO_PATH)
    base_link = root.find("./link[@name='base_link']")
    assert base_link is not None
    elevated_origins = [
        visual.find("origin")
        for visual in base_link.findall("visual")
        if visual.find("origin") is not None
        and float(visual.find("origin").attrib["xyz"].split()[2]) >= 0.10
    ]
    assert len(elevated_origins) == 2
    assert all(float(origin.attrib["xyz"].split()[0]) < 0.0 for origin in elevated_origins)


def test_generated_urdf_matches_xacro_and_parses_with_urdfdom():
    generated = subprocess.run(
        ["xacro", str(XACRO_PATH)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    assert ET.tostring(ET.fromstring(generated)) == ET.tostring(_root(GENERATED_URDF_PATH))

    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf") as output:
        output.write(generated)
        output.flush()
        result = subprocess.run(
            ["check_urdf", output.name],
            check=False,
            capture_output=True,
            text=True,
        )
    assert result.returncode == 0, result.stderr
