import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class JoyPosePitchGripperNode(Node):
    def __init__(self):
        super().__init__('joy_pose_pitch_gripper_node')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('pose_topic', '/target_pose')
        self.declare_parameter('gripper_topic', '/gripper_open')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('rate', 25.0)
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('axis_x', 1)
        self.declare_parameter('axis_y', 0)
        self.declare_parameter('axis_z', 7)
        self.declare_parameter('axis_pitch', 3)
        self.declare_parameter('speed_x', 0.075)
        self.declare_parameter('speed_y', 0.075)
        self.declare_parameter('speed_z', 0.060)
        self.declare_parameter('speed_pitch', 0.90)
        self.declare_parameter('x0', 0.20)
        self.declare_parameter('y0', 0.00)
        self.declare_parameter('z0', 0.15)
        self.declare_parameter('pitch0', 1.00)
        self.declare_parameter('min_x', 0.05)
        self.declare_parameter('max_x', 0.35)
        self.declare_parameter('min_y', -0.20)
        self.declare_parameter('max_y', 0.20)
        self.declare_parameter('min_z', 0.00)
        self.declare_parameter('max_z', 0.1)
        self.declare_parameter('min_pitch', 0.30)
        self.declare_parameter('max_pitch', 1.50)
        self.declare_parameter('use_radius_clamp', True)
        self.declare_parameter('min_radius', 0.145)
        self.declare_parameter('max_radius', 0.42)
        self.declare_parameter('button_gripper', 0)
        self.declare_parameter('gripper_open_initial', False)
        self.x = float(self.get_parameter('x0').value)
        self.y = float(self.get_parameter('y0').value)
        self.z = float(self.get_parameter('z0').value)
        self.pitch = float(self.get_parameter('pitch0').value)
        self.gripper_open = bool(self.get_parameter('gripper_open_initial').value)
        self.axes = []
        self.buttons = []
        self.prev_buttons = []
        self.pose_pub = self.create_publisher(PoseStamped, str(self.get_parameter('pose_topic').value), 10)
        self.gripper_pub = self.create_publisher(Bool, str(self.get_parameter('gripper_topic').value), 10)
        self.create_subscription(Joy, str(self.get_parameter('joy_topic').value), self.cb, 10)
        self.timer = self.create_timer(1.0 / float(self.get_parameter('rate').value), self.tick)

    def get_axis(self, idx):
        if idx < 0 or idx >= len(self.axes):
            return 0.0
        v = float(self.axes[idx])
        dz = float(self.get_parameter('deadzone').value)
        return 0.0 if abs(v) < dz else v

    def get_button(self, idx):
        if idx < 0 or idx >= len(self.buttons):
            return 0
        return int(self.buttons[idx])

    def prev_button(self, idx):
        if idx < 0 or idx >= len(self.prev_buttons):
            return 0
        return int(self.prev_buttons[idx])

    def cb(self, msg):
        self.prev_buttons = list(self.buttons)
        self.axes = list(msg.axes)
        self.buttons = list(msg.buttons)
        b = int(self.get_parameter('button_gripper').value)
        if self.get_button(b) == 1 and self.prev_button(b) == 0:
            self.gripper_open = not self.gripper_open

    def tick(self):
        dt = 1.0 / float(self.get_parameter('rate').value)
        self.x += float(self.get_parameter('speed_x').value) * self.get_axis(int(self.get_parameter('axis_x').value)) * dt
        self.y += float(self.get_parameter('speed_y').value) * self.get_axis(int(self.get_parameter('axis_y').value)) * dt
        self.z += float(self.get_parameter('speed_z').value) * self.get_axis(int(self.get_parameter('axis_z').value)) * dt
        self.pitch += float(self.get_parameter('speed_pitch').value) * self.get_axis(int(self.get_parameter('axis_pitch').value)) * dt
        self.x = min(max(self.x, float(self.get_parameter('min_x').value)), float(self.get_parameter('max_x').value))
        self.y = min(max(self.y, float(self.get_parameter('min_y').value)), float(self.get_parameter('max_y').value))
        self.z = min(max(self.z, float(self.get_parameter('min_z').value)), float(self.get_parameter('max_z').value))
        self.pitch = min(max(self.pitch, float(self.get_parameter('min_pitch').value)), float(self.get_parameter('max_pitch').value))
        if bool(self.get_parameter('use_radius_clamp').value):
            r = math.hypot(self.x, self.y)
            rmin = float(self.get_parameter('min_radius').value)
            rmax = float(self.get_parameter('max_radius').value)
            if r > 1e-9 and r < rmin:
                s = rmin / r
                self.x *= s
                self.y *= s
            if r > rmax:
                s = rmax / r
                self.x *= s
                self.y *= s
        qy = math.sin(0.5 * self.pitch)
        qw = math.cos(0.5 * self.pitch)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = str(self.get_parameter('frame_id').value)
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = self.z
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = qw
        self.pose_pub.publish(pose)
        g = Bool()
        g.data = self.gripper_open
        self.gripper_pub.publish(g)


def main(args=None):
    rclpy.init(args=args)
    node = JoyPosePitchGripperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
