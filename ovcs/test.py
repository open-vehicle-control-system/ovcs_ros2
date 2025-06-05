import cv2
print(cv2.cuda.getCudaEnabledDeviceCount())

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class StereoDisparityNode(Node):
    def __init__(self):
        super().__init__("stereo_disparity_node")
        self.bridge = CvBridge()

        self.sub_left = self.create_subscription(Image, "/stereo/left/image_raw", self.left_callback, 10)
        self.sub_right = self.create_subscription(Image, "/right/image_raw", self.right_callback, 10)

        self.pub_disp = self.create_publisher(Image, "/disparity/image", 10)

        self.left_img = None
        self.right_img = None

        # CUDA StereoBM (or use cv2.cuda.createStereoSGM if available)
        self.stereo = cv2.cuda.createStereoBM(numDisparities=64, blockSize=15)

    def left_callback(self, msg):
        self.left_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        self.try_compute_disparity()

    def right_callback(self, msg):
        self.right_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        self.try_compute_disparity()

    def try_compute_disparity(self):
        if self.left_img is None or self.right_img is None:
            return

        gpu_left = cv2.cuda_GpuMat()
        gpu_right = cv2.cuda_GpuMat()
        gpu_left.upload(self.left_img)
        gpu_right.upload(self.right_img)

        disparity_gpu = self.stereo.compute(gpu_left, gpu_right)
        disparity = disparity_gpu.download()

        disp_norm = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
        disp_uint8 = disp_norm.astype("uint8")

        disp_msg = self.bridge.cv2_to_imgmsg(disp_uint8, encoding="mono8")
        self.pub_disp.publish(disp_msg)

        self.left_img = None
        self.right_img = None

def main(args=None):
    rclpy.init(args=args)
    node = StereoDisparityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
