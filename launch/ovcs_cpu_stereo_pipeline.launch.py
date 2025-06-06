from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
      Node(
      package="ovcs_ros2",
      executable="pi_camera_synchronizer",
      name="pi_camera_synchronizer",
      output="screen",
      parameters=[],
    ),
    Node(
      package="ovcs_ros2",
      executable="disparity_image_publisher",
      name="disparity_image_publisher",
      output="screen",
      parameters=[],
    ),
    ComposableNodeContainer(
      name="ovcs_perception_container_right",
      namespace="stereo",
      package="rclcpp_components",
      executable="component_container_mt",
      composable_node_descriptions=[
      ComposableNode(
          package="image_proc",
          plugin="image_proc::RectifyNode",
          namespace="stereo/right",
          name="right_rectify_node",
          parameters=[{
          }],
          extra_arguments=[{"use_intra_process_comms": True}],
          remappings=[
            ("image", "image_raw")
          ]
        ),
        ComposableNode(
          package="image_proc",
          plugin="image_proc::DebayerNode",
          namespace="stereo/right",
          name="right_debayer_node",
          parameters=[{
          }],
          extra_arguments=[{"use_intra_process_comms": True}],
          remappings=[
            ("image", "image_rect"),
            ("image_color", "image_rect/color"),
            ("image_mono", "image_rect/mono")
          ]
        ),
        ComposableNode(
          package="image_proc",
          plugin="image_proc::RectifyNode",
          namespace="stereo/left",
          name="left_rectify_node",
          parameters=[{
          }],
          extra_arguments=[{"use_intra_process_comms": True}],
          remappings=[
            ("image", "image_raw")
          ]
        ),
        ComposableNode(
          package="image_proc",
          plugin="image_proc::DebayerNode",
          namespace="stereo/left",
          name="left_debayer_node",
          parameters=[{
          }],
          extra_arguments=[{"use_intra_process_comms": True}],
          remappings=[
            ("image", "image_rect"),
            ("image_color", "image_rect/color"),
            ("image_mono", "image_rect/mono")
          ]
        ),
        ComposableNode(
          package="stereo_image_proc",
          plugin="stereo_image_proc::DisparityNode",
          parameters=[{
          }],
          extra_arguments=[{"use_intra_process_comms": True}],
          namespace="stereo",
          name="disparity_node",
          remappings=[
            ("left/image_rect", "left/image_rect/mono"),
            ("right/image_rect", "right/image_rect/mono"),
            ("left/image_rect/camera_info", "left/camera_info"),
            ("right/image_rect/camera_info", "right/camera_info")
          ]
        ),
        ComposableNode(
          package="stereo_image_proc",
          plugin="stereo_image_proc::PointCloudNode",
          parameters=[{
            "use_color": True,
          }],
          namespace="stereo",
          name="point_cloud_node",
          extra_arguments=[{"use_intra_process_comms": True}],
          remappings=[
            ("left/image_rect_color", "left/image_rect/color"),
            ("left/image_rect/camera_info", "left/camera_info")
          ]
        )
      ],
      output="screen"
    )
])
