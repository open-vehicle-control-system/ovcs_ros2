from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
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
        )
      ],
      output="screen"
    )
  ])
