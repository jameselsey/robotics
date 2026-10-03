import os
from glob import glob

from setuptools import find_packages, setup

package_name = "drive_controller"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="James Elsey",
    maintainer_email="james.elsey@gmail.com",
    description="Tracked motor control, encoder odometry, diagnostics, and passive calibration",
    license="MIT",
    entry_points={
        "console_scripts": [
            "drive_controller = drive_controller.drive_controller_node:main",
            "calibrate_angular = drive_controller.angular_calibration:main",
        ],
    },
)
