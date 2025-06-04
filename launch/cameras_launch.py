from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # RIGHT
        ComposableNodeContainer(
            name='ovcs_perception_container_right',
            namespace='stereo',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='camera_ros',
                    plugin='camera::CameraNode',
                    name='right_camera_node',
                    namespace='stereo',
                    parameters=[{
                        'width': 1920,
                        'height': 1080,
                        'format': 'BGR888',
                        'camera': 0
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                    remappings=[
                        ('right_camera_node/image_raw', 'right/image_raw'),
                        ('right_camera_node/camera_info', 'right/camera_info'),
                        ('right_camera_node/image_raw/compressed', 'right/image_raw/compressed'),
                    ]
                )
            ],
            output='screen'
        ),
        # LEFT
        ComposableNodeContainer(
            name='ovcs_perception_container_left',
            namespace='stereo',
            package='rclcpp_components',
            executable='component_container_mt',
            # arguments=['--ros-args', '--log-level', 'debug'],
            composable_node_descriptions=[
                ComposableNode(
                    package='camera_ros',
                    plugin='camera::CameraNode',
                    name='left_camera_node',
                    namespace='stereo',
                    parameters=[{
                        'width': 1920,
                        'height': 1080,
                        'format': 'BGR888',
                        'camera': 1
                        # 'camera_info_url' : 'file:///home/pi/Development/ros2/calibration/imx477__base_axi_pcie_120000_rp1_i2c_80000_imx477_1a_640x480.yaml'

                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                    remappings=[
                        ('left_camera_node/image_raw', 'left/image_raw'),
                        ('left_camera_node/camera_info', 'left/camera_info'),
                        ('left_camera_node/image_raw/compressed', 'left/image_raw/compressed'),
                    ]
                )
            ],
            output='screen'
        )
    ])

# ros2 run image_proc rectify_node --ros-args -r __ns:=/stereo/left --remap image:=/stereo/left/image_raw
# ros2 run image_proc debayer_node --ros-args -r __ns:=/stereo/right -r image_raw:=image_rect -r image_color:=image_rect/color -r image_mono:=image_rect/mono


# image_proc
#   image_proc::RectifyNode
#   image_proc::DebayerNode
#   image_proc::ResizeNode
#   image_proc::CropDecimateNode
#   image_proc::CropNonZeroNode
#   image_proc::TrackMarkerNode
