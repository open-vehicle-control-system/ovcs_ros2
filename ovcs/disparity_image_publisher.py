#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class DisparityImagePublisher(Node):

    def __init__(self):
        super().__init__("disparity_image_publisher")

        self.declare_parameter("disparity_topic", "/stereo/disparity")
        self.declare_parameter("image_topic", "/stereo/disparity/image_raw")

        disparity_topic = self.get_parameter("disparity_topic").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(DisparityImage, disparity_topic, self.disparity_callback, 10)

        self.publisher = self.create_publisher(Image,image_topic, 10)

        self.get_logger().info(f"Subscribed to disparity: {disparity_topic}")
        self.get_logger().info(f"Publishing visual image to: {image_topic}")

    def disparity_callback(self, msg):
        try:
            disparity = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='32FC1')
            disp_normalized = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
            disp_uint8 = np.uint8(disp_normalized)
            disp_color = cv2.applyColorMap(disp_uint8, cv2.COLORMAP_JET)

            image_msg = self.bridge.cv2_to_imgmsg(disp_color, encoding="bgr8")
            image_msg.header = msg.header
            self.publisher.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to convert disparity: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DisparityImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
