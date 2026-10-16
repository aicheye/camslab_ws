import os
from glob import glob

from setuptools import setup

package_name = "camslab"


def tree(directory):
    """data_files entries that install directory/ under share/camslab, keeping subfolders."""
    entries = []
    for root, _, files in os.walk(directory):
        if files and "__pycache__" not in root:
            entries.append((os.path.join("share", package_name, root),
                            [os.path.join(root, f) for f in files]))
    return entries


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        *[entry for d in ("launch", "config", "models", "rviz", "urdf", "worlds")
          if os.path.isdir(d) for entry in tree(d)],
    ],
    install_requires=["setuptools", "numpy", "scipy"],
    zip_safe=True,
    maintainer="Sean Yang",
    maintainer_email="seanxxyang@gmail.com",
    description="Trajectory tracking and adaptive estimation for the lab's cars, ROS 2.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "controller = camslab.controller_node:main",
            "pose_tf = camslab.pose_tf:main",
            "estimator = camslab.estimator_node:main",
        ],
    },
)
