import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        
        self.declare_parameter("camera_name", "front")
        
        self.camera_name = self.get_parameter("camera_name").value
        
        if self.camera_name == "top":
            sensor_id = 0
        elif self.camera_name == "front":
            sensor_id = 1
        else:
            raise NameError("Please write correct camera name.")
        
        gst = (
            f"nvarguscamerasrc sensor-id={sensor_id} ! "
            "video/x-raw(memory:NVMM), width=1280, height=720, framerate=60/1, format=NV12 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw, width=640, height=480, format=BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! "
            "appsink max-buffers=1 drop=true sync=false"
        )
        self.cam = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)

        self.cam_publisher = self.create_publisher(
            Image,
            f"/arm/{self.camera_name}_cam",
            0
        )
        self.bridge = CvBridge()

        self.get_logger().info(
            f"Node started : {self.get_name()}"
        )
        
        self.timer = self.create_timer(
            0.05,
            self.loop
        )
        
    def destroy_node(self):
        self.get_logger().info("Stopping camera.")
        self.cam.release()
        return super().destroy_node()
        
    def loop(self):
        ret, img = self.cam.read()
        image = self.bridge.cv2_to_imgmsg(img, "bgr8")
        self.cam_publisher.publish(image)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()