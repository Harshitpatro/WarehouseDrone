import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import math

def generate_launch_description():

    # Declare serial numbers
    declare_front_serial = DeclareLaunchArgument(
        'front_serial', default_value='310222076155',
        description='Serial number of front D435i camera')
    declare_right_serial = DeclareLaunchArgument(
        'right_serial', default_value='327322061348',
        description='Serial number of right D435i camera')
    declare_left_serial = DeclareLaunchArgument(
        'left_serial', default_value='319522067209',
        description='Serial number of left D415 camera')
        
   
 
    # Front D435i - MASTER
    front_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py']),
        launch_arguments={
            'camera_namespace': 'camera_front',
            'camera_name': 'camera_front',
            'serial_no': ['_', LaunchConfiguration('front_serial')],
            
        }.items()
    )

    # Right D435i - SLAVE
    right_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py']),
        launch_arguments={
            'camera_namespace': 'camera_right',
            'camera_name': 'camera_right',
            'serial_no': ['_', LaunchConfiguration('right_serial')],
            
        }.items()
    )

    # Left D415 - SLAVE
    left_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py']),
        launch_arguments={
            'camera_namespace': 'camera_left',
            'camera_name': 'camera_left',
            'serial_no': ['_', LaunchConfiguration('left_serial')],
            
        }.items()
    )


    # RGBD Sync nodes - OPTIMIZED
    front_rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        output='screen',
        name='rgbd_sync_front',
        namespace='camera_front',
        parameters=[{
            'approx_sync': False,
            }],
        remappings=[
            ('rgb/image', '/camera_front/camera_front/color/image_raw'),
            ('depth/image', '/camera_front/camera_front/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera_front/camera_front/color/camera_info'),
            ('rgbd_image', '/camera_front/rgbd_image')
        ]
    )

    right_rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        output='screen',
        name='rgbd_sync_right',
        namespace='camera_right',
        parameters=[{
            'approx_sync': False,
        }],
        remappings=[
            ('rgb/image', '/camera_right/camera_right/color/image_raw'),
            ('depth/image', '/camera_right/camera_right/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera_right/camera_right/color/camera_info'),
            ('rgbd_image', '/camera_right/rgbd_image')
        ]
    )

    left_rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        output='screen',
        name='rgbd_sync_left',
        namespace='camera_left',
        parameters=[{
            'approx_sync': False,
         
        }],
        remappings=[
            ('rgb/image', '/camera_left/camera_left/color/image_raw'),
            ('depth/image', '/camera_left/camera_left/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera_left/camera_left/color/camera_info'),
            ('rgbd_image', '/camera_left/rgbd_image')
        ]
    )

    # Static transforms for cameras
    front_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_front_camera_tf',
        arguments=['0.107', '0.0', '-0.44', '0', '0', '0', 'base_link', 'camera_front_link']
    )

    left_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_left_camera_tf',
        arguments=['0', '0.102', '-0.44', '1.5708', '0', '0', 'base_link', 'camera_left_link']
    )

    right_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_right_camera_tf',
        arguments=['0', '-0.102', '-0.44', '-1.5708', '0', '0', 'base_link', 'camera_right_link']
    )
    
    # RTAB-Map visual odometry - OPTIMIZED FOR HIGH CONFIDENCE
    rtabmap_odom = Node(
    package='rtabmap_odom',
    executable='rgbd_odometry',
    name='rtabmap_odom',
    output='screen',
    parameters=[{

        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'publish_tf': True,
        'subscribe_rgbd':True,
        'rgbd_cameras':3,
        
       
        'approx_sync': True,
  
        
        
    }],
    remappings=[
        ('rgbd_image0', '/camera_front/rgbd_image'),
        ('rgbd_image1', '/camera_right/rgbd_image'),
        ('rgbd_image2', '/camera_left/rgbd_image'),
    ]
)
    # RTAB-Map SLAM node - Optimized for loop closure
    
    


    return LaunchDescription([
        # Camera serial number arguments
        declare_front_serial,
        declare_right_serial,
        declare_left_serial,
        # QR detection arguments
      
        # Transform nodes
        front_tf,
        right_tf,
        left_tf,
        # Camera nodes
        front_camera_launch,
        right_camera_launch,
        left_camera_launch,
        # RGBD sync nodes
        front_rgbd_sync,
        right_rgbd_sync,
        left_rgbd_sync,
        # SLAM nodes
        rtabmap_odom,
        
        # rtabmap_node,
       
      
    ])