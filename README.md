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

When opening a new terminal to run the program, please make sure to enter the command below before proceeding. This is a prerequisite for all procedures.

```sh
source ~/<your_workspace_name\>/install/setup.bash  # or setup.zsh
```

### Activate manipulator + camera + TF2

The method for simultaneously activating the equipment manipulator's motors, capturing camera images, and publishing TF2 topics is as follows.

```sh
ros2 launch physicai_arm bringup.launch.py
```

The topic information for publishing and subscribing is as follows.

| Topic | Pub/Sub | Message type | Description | 
| --- | ------- | -------- | ---- |
| /front_cam, /top_cam | Pub | `sensor_msgs/Image` | Publish images from each of the two cameras attached to the equipment. |
| /joint_states | Pub | `sensor_msgs/JointState` | Publish the manipulator's current joint angles. |
| /joint_targets | Sub | `sensor_msgs/JointState` | Publish a message to this topic if you want to set the target joint angle of the manipulator. |
| /tf | Pub | `tf2_msgs/TFMessage` | Publish a TF2 message from the manipulator. |
| /safety/torque_enable | Sub | `std_msgs/Bool` | Enables/disables the motor torque setting of the manipulator. When disabled, the torque value assigned to the motor is released, allowing the manipulator's posture to be changed arbitrarily through external physical factors, such as using hands. |

### FK Calculate

```sh
ros2 run physicai_arm fk_calc
```

This is a package that infers the EE location. You must proceed after running `bringup.launch.py`.

The topic information for publishing and subscribing is as follows.

| Topic | Pub/Sub | Message type | Description | 
| --- | ------- | -------- | ---- |
| /ee_position | Pub | `geometry_msgs/PoseStamped` | Publish the position and azimuth (Quternion) values ​​of EE. |
| /ee_matrix | Pub | `geometry_msgs/Float64MultiArray` | Publish the rotation matrix value of EE. |

### IK Calculate

```sh
ros2 run physicai_arm ik_calc
```

Operates the robot by receiving arbitrary EE position and orientation values.

The topic information for publishing and subscribing is as follows.

| Topic | Pub/Sub | Message type | Description | 
| --- | ------- | -------- | ---- |
| /target_pose | Sub | `geometry_msgs/PoseStamped` | Receives arbitrary EE position and bearing values. |

### Use joystick to move ee

```sh
ros2 launch physicai_arm joystick.launch.py
```

This is a launch file that manipulates the position of the manipulator's EE using a joystick. Before proceeding, please verify that the joystick is connected and that it is properly displayed as connected in the `/dev/input/js0` path.

Perform the following actions based on the specified axis of the joystick.

- EE's x, y, z
- EE's Pitch
- Gripper Open/Close (Boolean)