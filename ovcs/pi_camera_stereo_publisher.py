#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo, SetCameraInfo_Response
from picamera2 import Picamera2
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile
import socket
from libcamera import controls
import os
import yaml
from ovcs.fixed_camera_info_manager import CameraInfoManager
from std_srvs.srv import Trigger

class PiCameraStereoPublisher(Node):

  def __init__(self):
    super().__init__("pi_camera_stereo_publisher")
    self.declare_parameter("left_id", 1)
    self.declare_parameter("right_id", 0)
    self.declare_parameter("width", 1024)
    self.declare_parameter("height", 768)
    self.declare_parameter("frame_rate", 40)

    left_id = self.get_parameter("left_id").get_parameter_value().integer_value
    right_id = self.get_parameter("right_id").get_parameter_value().integer_value
    width = self.get_parameter("width").get_parameter_value().integer_value
    height = self.get_parameter("height").get_parameter_value().integer_value
    frame_rate = self.get_parameter("frame_rate").get_parameter_value().integer_value
    left_camera_calibration_url = f"file:///data/camera_left_{left_id}_{width}x{height}.yaml"
    right_camera_calibration_url = f"file:///data/camera_right_{right_id}_{width}x{height}.yaml"

    custom_qos = QoSProfile(
        depth=10
    )

    self.get_logger().info(f"Starting PiCameraStereoPublisher with {width}x{height}@{frame_rate}")

    left_raw_topic_name = f"/stereo/left/image_raw"
    left_camera_info_topic_name = f"/stereo/left/camera_info"

    self.left_raw_publisher = self.create_publisher(Image, left_raw_topic_name, custom_qos)
    self.left_camera_info_publisher = self.create_publisher(CameraInfo, left_camera_info_topic_name, custom_qos)
    self.left_camera_info_manager = CameraInfoManager(self, cname=f"camera_left", url=left_camera_calibration_url, namespace=f"/stereo/left")

    right_raw_topic_name = f"/stereo/right/image_raw"
    right_camera_info_topic_name = f"/stereo/right/camera_info"

    self.right_raw_publisher = self.create_publisher(Image, right_raw_topic_name, custom_qos)
    self.right_camera_info_publisher = self.create_publisher(CameraInfo, right_camera_info_topic_name, custom_qos)
    self.right_camera_info_manager = CameraInfoManager(self, cname=f"camera_right", url=right_camera_calibration_url, namespace=f"/stereo/right")

    self.cv_bridge = CvBridge()
    self.picam2_left = Picamera2(left_id)
    self.picam2_right = Picamera2(right_id)

    config_left = self.picam2_left.create_video_configuration(
        main={"size": (width, height), "format": "RGB888"},
        controls={"FrameRate": frame_rate, "SyncMode": controls.rpi.SyncModeEnum.Server},
        buffer_count=4
    )

    config_right = self.picam2_right.create_video_configuration(
        main={"size": (width, height), "format": "RGB888"},
        controls={"FrameRate": frame_rate, "SyncMode": controls.rpi.SyncModeEnum.Client},
        buffer_count=4
    )
    self.picam2_left.configure(config_left)
    self.picam2_left.start()

    self.picam2_right.configure(config_right)
    self.picam2_right.start()

    self.get_logger().info(f"Loading left camera info from: {left_camera_calibration_url}")
    self.left_camera_info_manager.loadCameraInfo()
    self.left_camera_info_msg = self.left_camera_info_manager.getCameraInfo()

    self.get_logger().info(f"Loading right camera info from: {right_camera_calibration_url}")
    self.right_camera_info_manager.loadCameraInfo()
    self.right_camera_info_msg = self.right_camera_info_manager.getCameraInfo()

    self.get_logger().info("Wait for synchronization...")
    req_right = self.picam2_right.capture_sync_request()
    req_left = self.picam2_left.capture_sync_request()

    self.get_logger().info(f"Left synchronization error: {req_left.get_metadata()['SyncTimer']}")
    self.get_logger().info(f"Right synchronization error: {req_right.get_metadata()['SyncTimer']}")

    req_right.release()
    req_left.release()

    timer_period = 1/frame_rate
    self.timer = self.create_timer(timer_period, self.timer_callback)

  def timer_callback(self):
    now = self.get_clock().now().to_msg()

    left_image = self.picam2_left.capture_array()
    left_image_msg = self.cv_bridge.cv2_to_imgmsg(left_image, "bgr8")

    right_image = self.picam2_right.capture_array()
    right_image_msg = self.cv_bridge.cv2_to_imgmsg(right_image, "bgr8")

    left_image_msg.header.stamp = now
    left_image_msg.header.frame_id = "camera"

    right_image_msg.header.stamp = now
    right_image_msg.header.frame_id = "camera"

    self.left_camera_info_msg = self.left_camera_info_manager.getCameraInfo()
    self.right_camera_info_msg = self.right_camera_info_manager.getCameraInfo()

    self.left_raw_publisher.publish(left_image_msg)
    self.right_raw_publisher.publish(right_image_msg)

    self.left_camera_info_msg.header = left_image_msg.header
    self.right_camera_info_msg.header = right_image_msg.header

    self.left_camera_info_publisher.publish(self.left_camera_info_msg)
    self.right_camera_info_publisher.publish(self.right_camera_info_msg)

def main(args=None):
  rclpy.init(args=args)
  node = PiCameraStereoPublisher()

  try:
      rclpy.spin(node)
  except KeyboardInterrupt:
      node.get_logger().info("Shutting down node gracefully.")
  finally:
      node.destroy_node()
      if rclpy.ok():
          rclpy.shutdown()

if __name__ == "__main__":
    main()
