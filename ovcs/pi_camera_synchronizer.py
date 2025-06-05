from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image, CameraInfo
from message_filters import Subscriber, ApproximateTimeSynchronizer
import time

class PiCameraSynchronizer(Node):
  def __init__(self):
    super().__init__("pi_camera_synchronizer")
    self.get_logger().info("Starting PiCameraSynchronizer")

    self.sub_img_left = Subscriber(self, Image, "/stereo/left/unsynced/image_raw")
    self.sub_img_right = Subscriber(self, Image, "/stereo/right/unsynced/image_raw")

    self.sub_info_left = Subscriber(self, CameraInfo, "/stereo/left/unsynced/camera_info")
    self.sub_info_right = Subscriber(self, CameraInfo, "/stereo/right/unsynced/camera_info")

    self.ts = ApproximateTimeSynchronizer(
        [
            self.sub_img_left,
            self.sub_img_right,
            self.sub_info_left,
            self.sub_info_right
        ],
        queue_size=5, slop=0.000001)
    self.ts.registerCallback(self.callback)

    # Publishers for republishing synchronized messages
    self.raw_left_publisher = self.create_publisher(Image, "/stereo/left/image_raw", 10)
    self.raw_right_publisher = self.create_publisher(Image, "/stereo/right/image_raw", 10)
    self.camera_info_left_publisher = self.create_publisher(CameraInfo, "/stereo/left/camera_info", 10)
    self.camemra_info_right_publisher = self.create_publisher(CameraInfo, "/stereo/right/camera_info", 10)

    self.frame_count = 0
    self.last_fps_time = time.time()

  def callback(self, img_left, img_right, info_left, info_right):
    stamp = img_right.header.stamp
    img_left.header.stamp = stamp
    info_left.header.stamp = stamp
    info_right.header.stamp = stamp

    img_right.header.frame_id = "camera"
    img_left.header.frame_id = "camera"
    info_left.header.frame_id = "camera"
    info_right.header.frame_id = "camera"

    self.raw_left_publisher.publish(img_left)
    self.raw_right_publisher.publish(img_right)
    self.camera_info_left_publisher.publish(info_left)
    self.camemra_info_right_publisher.publish(info_right)

    # FPS calculation
    self.frame_count += 1
    current_time = time.time()
    if current_time - self.last_fps_time >= 1.0:
      fps = self.frame_count / (current_time - self.last_fps_time)
      self.get_logger().info(f"Republished synchronized FPS: {fps:.2f}")
      self.last_fps_time = current_time
      self.frame_count = 0

def main(args=None):
  rclpy.init(args=args)
  node = PiCameraSynchronizer()

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
