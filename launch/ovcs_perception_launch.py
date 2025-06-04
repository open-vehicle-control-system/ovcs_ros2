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
                        'width': 640,
                        'height': 480,
                        'format': 'BGR888',
                        'camera': 0
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                    remappings=[
                        ('right_camera_node/image_raw', 'right/image_raw'),
                        ('right_camera_node/camera_info', 'right/camera_info'),
                        ('right_camera_node/image_raw/compressed', 'right/image_raw/compressed'),
                    ]
                ),
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    namespace='stereo/right',
                    name='right_rectify_node',
                    parameters=[{
                        # 'approximate_sync': True,
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                    remappings=[
                        ('image', 'image_raw')
                    ]
                ),
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::DebayerNode',
                    namespace='stereo/right',
                    name='right_debayer_node',
                    parameters=[{
                        # 'approximate_sync': True,
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                    remappings=[
                        ('image', 'image_rect'),
                        ('image_color', 'image_rect/color'),
                        ('image_mono', 'image_rect/mono')
                    ]
                ),
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
                        'width': 640,
                        'height': 480,
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
                ),
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    namespace='stereo/left',
                    name='left_rectify_node',
                    parameters=[{
                        # 'approximate_sync': True,
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                    remappings=[
                        ('image', 'image_raw')
                    ]
                ),
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::DebayerNode',
                    namespace='stereo/left',
                    name='left_debayer_node',
                    parameters=[{
                        # 'approximate_sync': True,
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                    remappings=[
                        ('image', 'image_rect'),
                        ('image_color', 'image_rect/color'),
                        ('image_mono', 'image_rect/mono')
                    ]
                )
            ],
            output='screen'
        ),
        ComposableNodeContainer(
            name='ovcs_perception_container_shared',
            namespace='stereo',
            package='rclcpp_components',
            executable='component_container_mt',
            # arguments=['--ros-args', '--log-level', 'debug'],
            composable_node_descriptions=[
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::DisparityNode',
                    parameters=[{
                        'approximate_sync': True
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                    namespace='stereo',
                    name='disparity_node',
                    remappings=[
                        ('left/image_rect', 'left/image_rect/mono'),
                        ('right/image_rect', 'right/image_rect/mono'),
                        ('left/image_rect/camera_info', 'left/camera_info'),
                        ('right/image_rect/camera_info', 'right/camera_info')
                    ]
                ),
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::PointCloudNode',
                    parameters=[{
                        'approximate_sync': True,
                        'use_color': False
                    }],
                    namespace='stereo',
                    name='point_cloud_node',
                    extra_arguments=[{'use_intra_process_comms': True}],
                    remappings=[
                        ('left/image_rect_color', 'left/image_rect/color'),
                        ('left/image_rect/camera_info', 'left/camera_info')
                    ]
                ),
                # ComposableNode(
                #     # ros2 run point_cloud_transport republish --ros-args -p in_transport:=raw -p out_transport:=zstd --remap in:=/stereo/points2 --remap /out/zstd:=/stereo/points2/compressed
                #     package='point_cloud_transport',
                #     plugin='point_cloud_transport::Republisher',
                #     parameters=[{
                #         'approximate_sync': True,
                #         'in_transport': 'raw',
                #         'out_transport': 'zstd'
                #     }],
                #     namespace='stereo',
                #     name='point_cloud_republisher_node',
                #     extra_arguments=[{'use_intra_process_comms': True}],
                #     remappings=[
                #         ('in', 'points2'),
                #         ('out', 'points2/compressed'),
                #     ]
                # )
            ],
            output='screen'
        ),
        # Node(
        #     package='sllidar_ros2',
        #     executable='sllidar_node',
        #     name='sllidar_node',
        #     parameters=[{
        #         'channel_type': 'serial',
        #         'serial_port': '/dev/ttyUSB0',
        #         'serial_baudrate': 460800,
        #         'frame_id': 'laser',
        #         'inverted': False,
        #         'angle_compensate': True,
        #         'scan_mode': 'Standard'
        #     }],
        #     output='screen'
        # ),
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
