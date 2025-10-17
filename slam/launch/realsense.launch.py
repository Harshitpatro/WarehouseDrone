import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Declare serial numbers
    declare_front_serial = DeclareLaunchArgument(
        'front_serial', default_value='327122076542',
        description='Serial number of front D435i camera')
    declare_right_serial = DeclareLaunchArgument(
        'right_serial', default_value='310222076155',
        description='Serial number of right D435i camera')
    declare_left_serial = DeclareLaunchArgument(
        'left_serial', default_value='319522067209',
        description='Serial number of left D415 camera')
        
    declare_down_serial = DeclareLaunchArgument(
        'down_serial', default_value='323522061991',
        description='Serial number of down D415 camera')
    declare_back_serial = DeclareLaunchArgument(
        'back_serial', default_value='327322061348',
        description='Serial number of back D415 camera')

    front_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py']),
        launch_arguments={
            'camera_namespace': 'camera_front',
            'camera_name': 'camera_front',
            'serial_no': ['_', LaunchConfiguration('front_serial')],
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'depth_module.depth_profile': '640,360,30',
            'rgb_camera.color_profile': '640,360,30',
            'depth_module.emitter_enabled': '0',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'unite_imu_method': '0',
            'initial_reset': 'true'
        }.items()
    )

    # Right D435i
    right_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py']),
        launch_arguments={
            'camera_namespace': 'camera_right',
            'camera_name': 'camera_right',
            'serial_no': ['_', LaunchConfiguration('right_serial')],
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'depth_module.depth_profile': '640,360,30',
            'rgb_camera.color_profile': '640,360,30',
            'depth_module.emitter_enabled': '0',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'unite_imu_method': '0',
            'initial_reset': 'true'
        }.items()
    )

    # Left D415
    left_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py']),
        launch_arguments={
            'camera_namespace': 'camera_left',
            'camera_name': 'camera_left',
            'serial_no': ['_', LaunchConfiguration('left_serial')],
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'depth_module.depth_profile': '640,360,30',
            'rgb_camera.color_profile': '640,360,30',
            'depth_module.emitter_enabled': '0',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'initial_reset': 'true'
        }.items()
    )
    down_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([             os.path.join(get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py']),
        launch_arguments={
            'camera_namespace': 'camera_down',
            'camera_name': 'camera_down',
            'serial_no': ['_', LaunchConfiguration('down_serial')],
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'depth_module.depth_profile': '640,360,30',
            'rgb_camera.color_profile': '640,360,30',
            'depth_module.emitter_enabled': '0',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'initial_reset': 'true'
        }.items()
    )
    back_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([             os.path.join(get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py']),
        launch_arguments={
            'camera_namespace': 'camera_back',
            'camera_name': 'camera_back',
            'serial_no': ['_', LaunchConfiguration('back_serial')],
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'depth_module.depth_profile': '640,360,30',
            'rgb_camera.color_profile': '640,360,30',
            'depth_module.emitter_enabled': '0',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'initial_reset': 'true'
        }.items()
    )
    front_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_front_camera_tf',
        arguments=['0.07', '0.0', '0.05', '0', '0', '0', 'base_link', 'camera_front_link']
    )

    left_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_left_camera_tf',
        arguments=['-0.09', '0.1210', '0.05', '1.5708', '0', '0', 'base_link', 'camera_left_link']
    )

    right_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_right_camera_tf',
        arguments=['-0.09', '-0.1210', '0.05', '-1.5708', '0', '0', 'base_link', 'camera_right_link']
    )
    down_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_down_camera_tf',
        arguments=['0.07', '0.0', '-0.03', '3.1416', '0', '0', 'base_link', 'camera_down_link']
    )
    back_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_back_camera_tf',
        arguments=['0.07', '0.0', '-0.03', '3.1416', '0', '0', 'base_link', 'camera_back_link']
    )
    return LaunchDescription([
        declare_front_serial,
        declare_right_serial,
        declare_left_serial,
        declare_down_serial,
        declare_back_serial,
        front_tf,
        right_tf,
        left_tf,
        down_tf,
        back_tf,
        front_camera_launch,
        right_camera_launch,
        left_camera_launch,
        down_camera_launch,
        back_camera_launch,
    ])