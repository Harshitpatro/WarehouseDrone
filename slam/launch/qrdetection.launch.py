# Requirements:
#   A realsense D435i camera
#   Flight controller with MAVROS connection
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
#   Install mavros (ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras)
#   QR code detection with YOLO model (best.pt)
#
# NOTE: Launch MAVROS separately first:
#   $ ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:921600
#
# Then launch this file:
#   $ ros2 launch slam qrdetection.launch.py
#
# Or with custom parameters:
#   $ ros2 launch slam qrdetection.launch.py camera_x:=0.1 camera_z:=0.05 model_path:=/path/to/best.pt

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import math

def generate_launch_description():
    parameters=[{
          'frame_id':'base_link',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':True,
          'approx_sync_max_interval': 0.04,
          'wait_imu_to_init':True,
          'always_check_imu_tf':False,
          'RGBD/Enabled': 'True',
          'RGBD/ImagesAlreadyRectified': 'True',
          'RGBD/LinearUpdate': '0.03',
          'RGBD/AngularUpdate': '0.03',
          'RGBD/LocalRadius': '30',
          'RGBD/OptimizeMaxError': '1.5',
          'RGBD/CreateOccupancyGrid': 'True',
          'RGBD/NeighborLinkRefining': 'True',
          'RGBD/ProximityBySpace': 'True',
          'RGBD/ProximityAngle': '120',
          'RGBD/ProximityMaxGraphDepth': '150',
          'RGBD/ProximityMaxPaths': '7',
          'RGBD/MaxOdomCacheSize': '20',
          # Visual Odometry Parameters
          'Vis/FeatureType': '8',
          'Vis/MaxFeatures': '1200',
          'Vis/MinInliers': '8',
          'Vis/Iterations': '250',
          'Vis/InlierDistance': '0.2',
          'Vis/CorNNDR': '0.8',
          'Vis/MaxDepth': '3.0',
          'Vis/MinDepth': '0.0',
          # Grid Parameters
          'Grid/RangeMax': '3.0',
          'Grid/CellSize': '0.05',
          'Grid/MaxObstacleHeight': '3.0',
          'Grid/MinClusterSize': '20',
          'Grid/NormalsSegmentation': 'True',
          'Grid/FlatObstacleDetected': 'True',
          'Grid/GroundIsObstacle': 'False',
          'Grid/NoiseFilteringRadius': '0.15',
          'Grid/NoiseFilteringMinNeighbors': '15',
          'Kp/DetectorStrategy': '8',
          # Odometry Parameters
          'Odometry/Strategy': '0',
          'Odometry/MaxFeatures': '1200',
          'Odometry/MinInliers': '8',
          'Odometry/Iterations': '250',
          'Odom/FilteringStrategy': '1',
          'Odom/GuessMotion': 'false',
          'RGBD/NeighborLinkRefining': 'True',
          # Loop Closure and Map Management
          'RGBD/LocalImmunizationRatio': '0.4',
          'Mem/LocalSpaceLinksKept': 'true',
          'RGBD/LocalLoopDetectionRadius': '10',
          'RGBD/LocalLoopDetectionTime': '10',
          'Rtabmap/DetectionRate': '2',
          'Rtabmap/LoopThr': '0.15',
          'Optimizer/Iterations': '100',
          'Optimizer/Strategy': '2',
          'g2o/PixelVariance': '0.3',
          'Vis/EstimationType': '1',
          # Memory and Performance
          'Mem/STMSize': '30',
          'Mem/RecentWmRatio': '0.4',
          'Rtabmap/TimeThr': '1000'}]

    remappings=[
          ('imu', '/mavros/imu/data'),
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/aligned_depth_to_color/image_raw')]

    return LaunchDescription([

        # Launch arguments for camera position relative to base_link
        DeclareLaunchArgument(
            'camera_x', default_value='0.0',
            description='Camera X position from base_link (forward)'),
        DeclareLaunchArgument(
            'camera_y', default_value='0.0',
            description='Camera Y position from base_link (left)'),
        DeclareLaunchArgument(
            'camera_z', default_value='0.0',
            description='Camera Z position from base_link (up)'),
        DeclareLaunchArgument(
            'camera_roll', default_value='0.0',
            description='Camera roll in radians'),
        DeclareLaunchArgument(
            'camera_pitch', default_value='0.0',
            description='Camera pitch in radians'),
        DeclareLaunchArgument(
            'camera_yaw', default_value='0.0',
            description='Camera yaw in radians'),
        
        # QR Code Detection Arguments
        DeclareLaunchArgument(
            'enable_qr_labeling', default_value='true',
            description='Enable QR code detection and labeling'),
        DeclareLaunchArgument(
            'front_serial', default_value='327122076542',
            description='Serial number of front D435i camera'),
        DeclareLaunchArgument(
            'model_path', default_value='/home/aero/best.pt',
            description='Path to YOLO model file for QR detection'),
        DeclareLaunchArgument(
            'qr_csv_file', default_value='qr_data.csv',
            description='Path to CSV file for storing QR data'),
        DeclareLaunchArgument(
            'qr_images_dir', default_value='qr_images',
            description='Directory to save QR detection images'),
        DeclareLaunchArgument(
            'qr_detection_interval', default_value='3.0',
            description='Minimum interval (seconds) between same QR detections'),

        # Make sure IR emitter is enabled
        SetParameter(name='depth_module.emitter_enabled', value=1),

        # Static transform from base_link to camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=[
                LaunchConfiguration('camera_x'),
                LaunchConfiguration('camera_y'),
                LaunchConfiguration('camera_z'),
                LaunchConfiguration('camera_roll'),
                LaunchConfiguration('camera_pitch'),
                LaunchConfiguration('camera_yaw'),
                'base_link',
                'camera_link'
            ]
        ),

        # Add proper TF tree structure
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=[
                '0', '0', '0',
                '0', '0', '0',
                'map',
                'odom'
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base',
            arguments=[
                '0', '0', '0',
                '0', '0', '0',
                'odom',
                'base_link'
            ]
        ),

        # Launch camera driver (without IMU)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py']),
            launch_arguments={'camera_namespace': '',
                              'enable_gyro': 'false',
                              'serial_no': ['_', LaunchConfiguration('front_serial')],
                              'enable_accel': 'false',
                              'align_depth.enable': 'true',
                              'enable_sync': 'true',
                              'depth_module.profile': '640x360x30',
                              'rgb_camera.profile': '640x360x30'}.items(),
        ),

        # RGBD Odometry
        Node(
            package='rtabmap_odom', 
            executable='rgbd_odometry', 
            output='screen',
            parameters=parameters,
            remappings=remappings
        ),

        # RTAB-Map SLAM
        Node(
            package='rtabmap_slam', 
            executable='rtabmap', 
            output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']
        ),

        # QR Code Detection and Labeling Node
        Node(
            package='slam',
            executable='qr_detection_node',
            name='qr_standalone_labeling_node',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('enable_qr_labeling')
            ),
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'camera_topic': '/camera/color/image_raw',
                'csv_file': LaunchConfiguration('qr_csv_file'),
                'images_dir': LaunchConfiguration('qr_images_dir'),
                'min_detection_interval': LaunchConfiguration('qr_detection_interval'),
                'odom_topic': '/odom',
            }]
        ),

    ])