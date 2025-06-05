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
from camera_info_manager import CameraInfoManager
from std_srvs.srv import Trigger


class MyCameraInfoManager(CameraInfoManager):
    def setCameraInfo(self, req, third):
      self.node.get_logger().info("CALLLEEDD-----------------------------------------------------------")
      self.node.get_logger().debug('SetCameraInfo received for ' + self.cname)
      self.camera_info = req.camera_info
      rsp = SetCameraInfo.Response()
      rsp.success = super().saveCalibration(req.camera_info, self.url, self.cname)
      if not rsp.success:
          rsp.status_message = 'Error storing camera calibration.'
      return rsp

class PiCameraSynchronizedPublisher(Node):

  def __init__(self):
    super().__init__("pi_camera_synchronized_publisher")
    self.declare_parameter("id", 0)
    self.declare_parameter("side", "left")
    self.declare_parameter("sync_mode", "none")
    self.declare_parameter("width", 1024)
    self.declare_parameter("height", 768)
    self.declare_parameter("frame_rate", 40)

    id = self.get_parameter("id").get_parameter_value().integer_value
    sync_mode = self.get_parameter("sync_mode").get_parameter_value().string_value
    width = self.get_parameter("width").get_parameter_value().integer_value
    height = self.get_parameter("height").get_parameter_value().integer_value
    frame_rate = self.get_parameter("frame_rate").get_parameter_value().integer_value
    side = self.get_parameter("side").get_parameter_value().string_value
    camera_calibration_url = f"file:///data/camera_{side}_{width}x{height}.yaml"

    custom_qos = QoSProfile(
        depth=10
    )

    self.get_logger().info(f"Starting PiCameraSynchronizer for {side} camera with {width}x{height}@{frame_rate}")

    raw_topic_name = f"/stereo/{side}/unsynced/image_raw"
    camera_info_topic_name = f"/stereo/{side}/unsynced/camera_info"

    self.raw_publisher = self.create_publisher(Image, raw_topic_name, custom_qos)
    self.camera_info_publisher = self.create_publisher(CameraInfo, camera_info_topic_name, custom_qos)
    self.camera_info_manager = MyCameraInfoManager(self, cname=f"camera_{side}", url=camera_calibration_url, namespace=f"/stereo/{side}")

    self.cv_bridge = CvBridge()
    self.picam2 = Picamera2(id)

    sync_mode_ = controls.rpi.SyncModeEnum.Off
    if sync_mode == "server":
      sync_mode_ = controls.rpi.SyncModeEnum.Server
    elif sync_mode == "client":
      sync_mode_ = controls.rpi.SyncModeEnum.Client

    config0 = self.picam2.create_video_configuration(
        main={"size": (width, height), "format": "RGB888"},
        controls={"FrameRate": frame_rate, "SyncMode": sync_mode_},
        buffer_count=4
    )
    self.picam2.configure(config0)
    self.picam2.start()

    self.get_logger().info(f"Loading {side} camera info from: {camera_calibration_url}")
    self.camera_info_manager.loadCameraInfo()

    self.camera_info_msg = self.camera_info_manager.getCameraInfo()

    if sync_mode_ != controls.rpi.SyncModeEnum.Off:
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
