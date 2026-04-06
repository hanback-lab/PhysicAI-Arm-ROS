from glob import glob
from setuptools import setup
import os

package_name = "physicai_arm_gz"

def package_files(directory):
    paths = []
    for path, _, filenames in os.walk(directory):
        if filenames:
            paths.append((os.path.join("share", package_name, path), [os.path.join(path, f) for f in filenames]))
    return paths

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
]

for subdir in ["launch", "config", "urdf", "models"]:
    data_files.extend(package_files(subdir))

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenAI",
    maintainer_email="noreply@example.com",
    description="Gazebo Sim bridge package for the PhysicAI / SO101 arm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "joint_targets_to_cmd_pos = physicai_arm_gz.joint_targets_to_cmd_pos:main",
        ],
    },
)
