#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from picamera2 import Picamera2
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile
import socket
from libcamera import controls
import os
import yaml
from camera_info_manager import CameraInfoManager

class PiCameraSynchronizedPublisher(Node):

  def __init__(self):
    super().__init__("pi_camera_synchronized_publisher")
    self.declare_parameter("camera_side", "left")
    self.declare_parameter("width", 1024)
    self.declare_parameter("height", 768)
    self.declare_parameter("frame_rate", 40)

    width = self.get_parameter("width").get_parameter_value().integer_value
    height = self.get_parameter("height").get_parameter_value().integer_value
    frame_rate = self.get_parameter("frame_rate").get_parameter_value().integer_value
    camera_side = self.get_parameter("camera_side").get_parameter_value().string_value
    camera_calibration_url = f"file:///data/camera_{camera_side}_{width}x{height}.yaml"

    custom_qos = QoSProfile(
        depth=10
    )

    self.get_logger().info(f"Starting PiCameraSynchronizer for {camera_side} camera with {width}x{height}@{frame_rate}")

    raw_topic_name = f"/stereo/{camera_side}/unsynced/image_raw"
    camera_info_topic_name = f"/stereo/{camera_side}/unsynced/camera_info"

    self.raw_publisher = self.create_publisher(Image, raw_topic_name, custom_qos)
    self.camera_info_publisher = self.create_publisher(CameraInfo, camera_info_topic_name, custom_qos)
    self.camera_info_manager = CameraInfoManager(self, url=camera_calibration_url)

    self.cv_bridge = CvBridge()

    self.picam2 = Picamera2(0)

    sync_mode = controls.rpi.SyncModeEnum.Server if camera_side == "right" else controls.rpi.SyncModeEnum.Client

    config0 = self.picam2.create_video_configuration(
        main={"size": (width, height), "format": "RGB888"},
        controls={"FrameRate": frame_rate, "SyncMode": sync_mode},
        buffer_count=4
    )
    self.picam2.configure(config0)
    self.picam2.start()

    self.get_logger().info(f"Loading {camera_side} camera info from: {camera_calibration_url}")
    self.camera_info_manager.loadCameraInfo()

    self.camera_info_msg = self.camera_info_manager.getCameraInfo()

    self.get_logger().info("Wait for synchronization...")
    req = self.picam2.capture_sync_request()
    self.get_logger().info(f"Synchronization error: {req.get_metadata()['SyncTimer']}")
    req.release()

    timer_period = 1/frame_rate
    self.timer = self.create_timer(timer_period, self.timer_callback)

  def timer_callback(self):
    image = self.picam2.capture_array()
    image_msg = self.cv_bridge.cv2_to_imgmsg(image, "bgr8")
    self.raw_publisher.publish(image_msg)
    self.camera_info_msg.header = image_msg.header
    self.camera_info_publisher.publish(self.camera_info_msg)

  def load_camera_info_from_yaml(self, yaml_file_path):
    cam_info_msg = CameraInfo()

    if not os.path.exists(yaml_file_path):
        print(f"[WARN] Calibration file not found: {yaml_file_path}. Using empty CameraInfo.")
        return cam_info_msg

    with open(yaml_file_path, "r") as file:
        calibration_data = yaml.safe_load(file)

    cam_info_msg.width = calibration_data.get("image_width", 0)
    cam_info_msg.height = calibration_data.get("image_height", 0)
    cam_info_msg.distortion_model = calibration_data.get("distortion_model", "")
    cam_info_msg.d = calibration_data.get("distortion_coefficients", {}).get("data", [])
    cam_info_msg.k = calibration_data.get("camera_matrix", {}).get("data", [0.0]*9)
    cam_info_msg.r = calibration_data.get("rectification_matrix", {}).get("data", [0.0]*9)
    cam_info_msg.p = calibration_data.get("projection_matrix", {}).get("data", [0.0]*12)

    return cam_info_msg

def main(args=None):
  rclpy.init(args=args)
  node = PiCameraSynchronizedPublisher()

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
