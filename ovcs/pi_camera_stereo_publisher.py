#!/usr/bin/python3
import os
import socket
import time
from datetime import datetime
import cv2
import numpy as np
import yaml
from cv_bridge import CvBridge
from libcamera import controls
from picamera2 import Picamera2
from picamera2.devices.hailo import Hailo

# ROS 2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CameraInfo, Image
from sensor_msgs.srv import SetCameraInfo, SetCameraInfo_Response
from std_srvs.srv import Trigger
from stereo_msgs.msg import DisparityImage
from rcl_interfaces.msg import SetParametersResult

# Local packages
from ovcs.fixed_camera_info_manager import CameraInfoManager

class PiCameraStereoPublisher(Node):

  def __init__(self):
    super().__init__("pi_camera_stereo_publisher")
    self.declare_parameter("left_id", 1)
    self.declare_parameter("right_id", 0)
    self.declare_parameter("width", 640)
    self.declare_parameter("height", 480)
    self.declare_parameter("frame_rate", 30)
    self.declare_parameter("frame_id", "camera")
    self.declare_parameter("block_size", 27)
    self.declare_parameter("num_disparities", 5 * 16)

    self.add_on_set_parameters_callback(self.parameter_callback)

    custom_qos = QoSProfile(depth=10)

    left_raw_topic_name = f"/stereo/left/image_raw"
    left_camera_info_topic_name = f"/stereo/left/camera_info"

    self.left_raw_publisher = self.create_publisher(Image, left_raw_topic_name, custom_qos)
    self.left_rectified_publisher = self.create_publisher(Image, left_raw_topic_name + "/rect", custom_qos)
    self.left_camera_info_publisher = self.create_publisher(CameraInfo, left_camera_info_topic_name, custom_qos)
    self.left_camera_info_manager = CameraInfoManager(self, cname=f"camera_left",namespace=f"/stereo/left")

    right_raw_topic_name = f"/stereo/right/image_raw"
    right_camera_info_topic_name = f"/stereo/right/camera_info"

    self.right_raw_publisher = self.create_publisher(Image, right_raw_topic_name, custom_qos)
    self.right_rectified_publisher = self.create_publisher(Image, right_raw_topic_name + "/rect", custom_qos)
    self.right_camera_info_publisher = self.create_publisher(CameraInfo, right_camera_info_topic_name, custom_qos)
    self.right_camera_info_manager = CameraInfoManager(self, cname=f"camera_right", namespace=f"/stereo/right")
    self.disparity_publisher = self.create_publisher(DisparityImage, '/stereo/disparity', 10)
    self.cv_bridge = CvBridge()

    # self.hailo = Hailo("/opt/ros2_ws/src/ovcs_ros2/scdepthv3.hef")
    # self.hailo_model_height, self.hailo_model_width, _ = self.hailo.get_input_shape()

    self.load_parameters()
    self.load_cameras()
    self.load_stereo()

    timer_period = 1/self.frame_rate
    self.timer = self.create_timer(timer_period, self.timer_callback)

  def timer_callback(self):
    now = self.get_clock().now().to_msg()

    left_image = self.picam2_left.capture_array("main")
    # left_image_lores = self.picam2_left.capture_array("lores")
    # inference_results = self.hailo.run(left_image_lores)

    left_image_msg = self.build_image_message(left_image, now)

    right_image = self.picam2_right.capture_array()
    right_image_msg = self.build_image_message(right_image, now)

    self.left_camera_info_msg = self.left_camera_info_manager.getCameraInfo()
    self.left_camera_info_msg.header = left_image_msg.header
    self.right_camera_info_msg = self.right_camera_info_manager.getCameraInfo()
    self.right_camera_info_msg.header = right_image_msg.header

    self.left_raw_publisher.publish(left_image_msg)
    self.right_raw_publisher.publish(right_image_msg)

    self.left_camera_info_publisher.publish(self.left_camera_info_msg)
    self.right_camera_info_publisher.publish(self.right_camera_info_msg)

    if (self.stereoCalibrated()):
      disparity, left_rectified_image, right_rectified_image = self.rectify_and_compute_disparity(left_image, right_image)

      disparity_msg = self.build_disparity_message(disparity, now)
      self.disparity_publisher.publish(disparity_msg)

      left_rectified_image_msg = self.build_image_message(left_rectified_image, now)
      right_rectified_image_msg = self.build_image_message(right_rectified_image, now)

      self.left_rectified_publisher.publish(left_rectified_image_msg)
      self.right_rectified_publisher.publish(right_rectified_image_msg)

  def load_parameters(self, params={}):
    def get_parameter_value(name, expected_type):
      for param in params:
        if param.name == name:
          return param.value
      return self.get_parameter(name).value if expected_type != str else self.get_parameter(name).get_parameter_value().string_value

    self.left_id = get_parameter_value("left_id", int)
    self.right_id = get_parameter_value("right_id", int)
    self.width = get_parameter_value("width", int)
    self.height = get_parameter_value("height", int)
    self.frame_rate = get_parameter_value("frame_rate", int)
    self.frame_id = get_parameter_value("frame_id", str)
    self.block_size = get_parameter_value("block_size", int)
    self.num_disparities = get_parameter_value("num_disparities", int)

    current_directory = os.path.dirname(os.path.abspath(__file__))
    calibration_directory = os.path.join(current_directory, "../data/calibrations")
    left_camera_calibration_url = f"file://{os.path.join(calibration_directory, f'camera_left_{self.left_id}_{self.width}x{self.height}.yaml')}"
    right_camera_calibration_url = f"file://{os.path.join(calibration_directory, f'camera_right_{self.right_id}_{self.width}x{self.height}.yaml')}"

    self.left_camera_info_manager.setURL(left_camera_calibration_url)
    self.right_camera_info_manager.setURL(right_camera_calibration_url)

  def load_cameras(self):
    if hasattr(self, "picam2_left"):
      self.get_logger().info(f"Stopping left camera")
      self.picam2_left.stop()
    else:
      self.picam2_left = Picamera2(self.left_id)

    if hasattr(self, "picam2_right"):
      self.get_logger().info(f"Stopping right camera")
      self.picam2_right.stop()
    else:
      self.picam2_right = Picamera2(self.right_id)

    config_left = self.picam2_left.create_video_configuration(
        main={"size": (self.width, self.height), "format": "RGB888"},
        # lores={"size": (self.hailo_model_width, self.hailo_model_height), "format": "RGB888"},
        controls={
          "FrameRate": self.frame_rate,
          "SyncMode": controls.rpi.SyncModeEnum.Server,
          "AeEnable": False,
          "AwbEnable": False,
          "ExposureTime": 15000
        },
        buffer_count=2
    )
    self.picam2_left.configure(config_left)
    self.picam2_left.start()
    self.get_logger().info(f"Loading left camera info from: {self.left_camera_info_manager.getURL()}")
    self.left_camera_info_manager.loadCameraInfo()
    self.left_camera_info_msg = self.left_camera_info_manager.getCameraInfo()

    config_right = self.picam2_right.create_video_configuration(
        main={"size": (self.width, self.height), "format": "RGB888"},
        controls={
          "FrameRate": self.frame_rate,
          "SyncMode": controls.rpi.SyncModeEnum.Client,
          "AeEnable": False,
          "AwbEnable": False,
          "ExposureTime": 15000
        },
        buffer_count=2
    )
    self.picam2_right.configure(config_right)
    self.picam2_right.start()
    self.get_logger().info(f"Loading right camera info from: {self.right_camera_info_manager.getURL()}")
    self.right_camera_info_manager.loadCameraInfo()
    self.right_camera_info_msg = self.right_camera_info_manager.getCameraInfo()

    self.get_logger().info("Wait for synchronization...")
    right_request = self.picam2_right.capture_sync_request()
    left_request = self.picam2_left.capture_sync_request()

    self.get_logger().info(f"Left synchronization error: {left_request.get_metadata()['SyncTimer']}")
    self.get_logger().info(f"Right synchronization error: {right_request.get_metadata()['SyncTimer']}")

    right_request.release()
    left_request.release()

    self.get_logger().info(f"Starting cameras with {self.width}x{self.height}@{self.frame_rate}")

  def load_stereo(self):
    if (self.stereoCalibrated()):
      K_left = np.array(self.left_camera_info_msg.k).reshape((3, 3))
      D_left = np.array(self.left_camera_info_msg.d)
      K_right = np.array(self.right_camera_info_msg.k).reshape((3, 3))
      D_right = np.array(self.right_camera_info_msg.d)
      R_right = np.array(self.right_camera_info_msg.r).reshape((3, 3))
      P_left = np.array(self.left_camera_info_msg.p).reshape((3, 4))
      P_right = np.array(self.right_camera_info_msg.p).reshape((3, 4))

      self.focal_length_px = P_left[0, 0]
      self.baseline_m = abs(P_left[0, 3] - P_right[0, 3]) / self.focal_length_px

      self.get_logger().info("Focal Length (pixels): {:.3f}".format(self.focal_length_px))
      self.get_logger().info("Baseline (meters): {:.6f}".format(self.baseline_m))

      rectification_left, rectification_right, P_left_rect, P_right_rect, Q, _, _ = cv2.stereoRectify(
        K_left, D_left, K_right, D_right,
        (self.width, self.height), R_right, np.array([self.right_camera_info_msg.p[3], 0, 0]),
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0
      )

      self.map_left_x, self.map_right_y = cv2.initUndistortRectifyMap(K_left, D_left, rectification_left, P_left_rect, (self.width, self.height), cv2.CV_32FC1)
      self.map_right_x, self.map_right_y = cv2.initUndistortRectifyMap(K_right, D_right, rectification_right, P_right_rect, (self.width, self.height), cv2.CV_32FC1)

      self.stereo = cv2.StereoBM_create(blockSize=self.block_size, numDisparities=self.num_disparities)
      self.get_logger().info(f"Stereo loaded")
    else:
      self.get_logger().info(f"Stereo not loaded, cameras are no calibrated")

  def parameter_callback(self, params):
    reload_cameras_params = {"left_id", "right_id", "width", "height", "frame_rate"}
    reload_stereo = {"num_disparities", "block_size"}

    for param in params:
      self.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")
      # Validate block_size: must be odd and between 5 and 255
      if param.name == "block_size":
          if not (5 <= param.value <= 255 and param.value % 2 == 1):
              self.get_logger().error("Invalid block_size: must be odd and between 5 and 255.")
              return SetParametersResult(successful=False)

      # Validate num_disparities: must be divisible by 16
      if param.name == "num_disparities":
          if param.value % 16 != 0:
              self.get_logger().error("Invalid num_disparities: must be a multiple of 16.")
              return SetParametersResult(successful=False)

    self.load_parameters(params)
    if param.name in reload_cameras_params:
      self.load_cameras()
    if param.name in reload_stereo:
      self.load_stereo()

    return SetParametersResult(successful=True)

  def build_image_message(self, image, timestamp):
    image_msg = self.cv_bridge.cv2_to_imgmsg(image, "bgr8")
    image_msg.header.stamp = timestamp
    image_msg.header.frame_id = self.frame_id
    return image_msg

  def build_disparity_message(self, disparity, timestamp):
    disparity_image_msg = self.cv_bridge.cv2_to_imgmsg(disparity, encoding="32FC1")
    disparity_image_msg.header.stamp = timestamp
    disparity_image_msg.header.frame_id = self.frame_id

    disparity_msg = DisparityImage()
    disparity_msg.header = disparity_image_msg.header
    disparity_msg.image = disparity_image_msg
    disparity_msg.f = self.focal_length_px
    disparity_msg.t = self.baseline_m
    disparity_msg.min_disparity = float(0)
    disparity_msg.max_disparity = float(self.num_disparities)
    disparity_msg.delta_d = 1.0 / 16.0  # For OpenCV SGBM with subpixel resolution

    return disparity_msg

  def rectify_and_compute_disparity(self, left_image, right_image):
    left_rectified_image = cv2.remap(left_image, self.map_left_x, self.map_right_y, cv2.INTER_LINEAR)
    right_rectified_image = cv2.remap(right_image, self.map_right_x, self.map_right_y, cv2.INTER_LINEAR)
    gray_left = cv2.cvtColor(left_rectified_image, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(right_rectified_image, cv2.COLOR_BGR2GRAY)

    disparity = self.stereo.compute(gray_left, gray_right).astype(np.float32) / 16.0
    disparity[disparity < 0] = 0.0 # Mask invalid disparities

    return disparity, left_rectified_image, right_rectified_image

  def stereoCalibrated(self):
    return self.left_camera_info_manager.isCalibrated() and self.right_camera_info_manager.isCalibrated()

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
