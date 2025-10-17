# Requirements:
#   A realsense D435i camera
#   Flight controller with MAVROS connection
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
#   Install mavros (ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras)
#
# NOTE: Launch MAVROS separately first:
#   $ ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:921600
#
# Then launch this file:
#   $ ros2 launch slam rtabmap_infra.launch.py
#
# Or with custom camera position (example: camera 0.1m forward, 0.05m up from base_link):
#   $ ros2 launch slam rtabmap_infra.launch.py camera_x:=0.1 camera_z:=0.05

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import math

def generate_launch_description():
    parameters=[{
          'frame_id':'base_link',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':True,
          'approx_sync_max_interval': 0.1,  # Increased tolerance to 100ms
          'wait_imu_to_init':True,
          'wait_for_transform':0.5,  # Wait up to 500ms for transforms
          
          # CRITICAL: Publish TF from odometry node
          'publish_tf':True,
          'publish_null_when_lost':False,
          
          'RGBD/Enabled': 'True',
          
          # Visual Odometry Parameters
          'Vis/FeatureType': '6',
          'Vis/MaxFeatures': '3000',
          'Vis/MinInliers': '6',
          'Vis/Iterations': '300',
          'Vis/InlierDistance': '0.15',
          'Vis/CorNNDR': '0.7',
          'Vis/MaxDepth': '3.0',
          'Vis/MinDepth': '0.3',
          'Vis/CorGuessWinSize': '40',
          'Vis/CorFlowWinSize': '32',
          'Vis/CorFlowMaxLevel': '3',
          
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
          'Kp/DetectorStrategy': '6',
          
          # Odometry Parameters
          'Odometry/Strategy': '0',
          'Odometry/MaxFeatures': '3000',
          'Odometry/MinInliers': '6',
          'Odometry/Iterations': '300',
          'Odom/FilteringStrategy': '1',
          'Odom/GuessMotion': 'true',
          'Odom/KeyFrameThr': '0.6',
          'RGBD/NeighborLinkRefining': 'True',
          'RGBD/ProximityBySpace': 'True',
          'RGBD/ProximityAngle': '45',
          'RGBD/LinearUpdate': '0.0',
          'RGBD/AngularUpdate': '0.0',
          
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
          'Mem/STMSize': '50',
          'Mem/RecentWmRatio': '0.5',
          'Mem/IntermediateNodeDataKept': 'true',
          'Rtabmap/TimeThr': '0',
          'Rtabmap/MemoryThr': '0',
          }]

    remappings=[
          ('imu', '/mavros/imu/data'),
          ('rgb/image', '/camera/infra1/image_rect_raw'),
          ('rgb/camera_info', '/camera/infra1/camera_info'),
          ('depth/image', '/camera/depth/image_rect_raw')]

    return LaunchDescription([

        # Launch arguments for camera position
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

        # ONLY map to odom static transform
        # DO NOT publish odom to base_link - rgbd_odometry will do this
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

        # Launch camera driver with aligned depth to infra1
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py']),
            launch_arguments={
                              'camera_namespace': '',
                              'enable_gyro': 'false',
                              'enable_accel': 'false',
                              'enable_infra1': 'true',
                              'enable_infra2': 'false',  # Disabled, only need one infrared
                              'enable_sync': 'true',
                              'align_depth.enable': 'true',  # CRITICAL: Align depth to infra1
                              'depth_module.profile': '640x360x30',
                              'infra_rgb': 'false',
                              'depth_module.emitter_enabled': '0',  # Disable IR emitter
                              'initial_reset': 'true'}.items(),
        ),

        # RGBD Odometry node - will publish odom->base_link transform
        Node(
            package='rtabmap_odom', 
            executable='rgbd_odometry', 
            output='screen',
            parameters=parameters,
            remappings=remappings),

        # RTAB-Map SLAM node
        Node(
            package='rtabmap_slam', 
            executable='rtabmap', 
            output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

    ])