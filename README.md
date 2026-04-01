# PhysicAI Arm ROS2 Package

On details, please check below the link.

- https://github.com/hanback-lab/PhysicAI-Arm

## Requirement

- vassar-feetech-servo-sdk
- Opencv
- ROS2 Humble

## Installation

```sh
mkdir -p ~/<your_workspace_name\>/src/physicai_arm
cd ~/<your_workspace_name\>/src/physicai_arm
git clone https://github.com/hanback-lab/PhysicAI-Arm-ROS
chmod +x "launch/*.launch.py"
cd ../../
colcon build --symlink-install --packages-select PhysicAI-Arm-ROS
```

## Usage

### Common

```sh
source ~/<your_workspace_name\>/install/setup.bash  # or setup.zsh
```

### Activate manipulator + camera + TF2

```sh
ros2 launch physicai_arm bringup.launch.py
```

Topic list

- /front_cam, /top_cam : Publish sensor_msgs/Image from each camera on the device.
- /joint_states : Publish sensor_msgs/JointState from manipulator.
- /joint_targets : Subscribe sensor_msgs/JointState for set the manipulator pose.
- /tf : Publish TF2 message from the manipulator.
- /safety/torque_enable : Subscribe std_msgs/Bool for enable/disable torque value on manipulator's motor.

### FK Calculate

```sh
ros2 run physicai_arm fk_calc
```

Topic list

- /ee_position : Publish geometry_msgs/PoseStamped from ee's position.
- /ee_matrix : Publish std_msgs/Float64MultiArray from ee's rotation matrix.

### IK Calculate

```sh
ros2 run physicai_arm ik_calc
```

Topic list

- /target_pose : Subscribe geometry_msgs/PoseStamped for set the ee's position and orientation.

### Use joystick to move ee

Before you execute, please check the connectivitiy between your joystick and onboard computer, then also check the `/dev/input/js0` path is exist.

```sh
ros2 launch physicai_arm joystick.launch.py
```
