from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'physicai_arm'

def only_files(pattern: str):
    return [p for p in glob(pattern) if os.path.isfile(p)]

def package_files(directory: str):
    paths = []
    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            full_path = os.path.join(path, filename)
            install_dir = os.path.join("share", package_name, path)
            paths.append((install_dir, [full_path]))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', only_files('config/*')),
        ('share/' + package_name + '/launch', only_files('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', only_files('urdf/*')),
        *package_files("urdf/assets")
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soda',
    maintainer_email='soda@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'feetech_driver = physicai_arm.feetech_jointstate_driver:main',
            "camera_node = physicai_arm.camera_node:main",
            "fk_calc = physicai_arm.fk_calc:main",
            "ik_calc = physicai_arm.ik_calc:main",
            "simple_pose = physicai_arm.simple_pose_pub:main",
            "joy_to_target = physicai_arm.joy_to_target:main",
            "joint_targets_to_cmd_pos = physicai_arm_gz.joint_targets_to_cmd_pos:main",
        ],
    },
)