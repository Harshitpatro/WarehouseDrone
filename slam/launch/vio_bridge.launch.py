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
    declare_down_serial = DeclareLaunchArgument(
        'down_serial', default_value='323522061991',
        description='Serial number of down D415 camera')
        
    # Front D435i - MASTER
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
            'depth_module.depth_profile': '424,240,30',
            'rgb_camera.color_profile': '424,240,30',
            'depth_module.emitter_enabled': '1',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'unite_imu_method': '0',
            'initial_reset': 'true',
            'inter_cam_sync_mode': '1',
            'depth_module.min_distance': '0.3',
            'depth_module.max_distance': '20.0',
            'depth_module.visual_preset': '4',
            'spatial_filter.enable': 'true',
            'spatial_filter.magnitude': '2',
            'spatial_filter.smooth_alpha': '0.5',
            'spatial_filter.smooth_delta': '20',
            'temporal_filter.enable': 'true',
            'temporal_filter.smooth_alpha': '0.4',
            'temporal_filter.smooth_delta': '20',
            'depth_module.enable_auto_exposure': 'true',
            'rgb_camera.enable_auto_exposure': 'true'
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
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'depth_module.depth_profile': '424,240,30',
            'rgb_camera.color_profile': '424,240,30',
            'depth_module.emitter_enabled': '1',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'unite_imu_method': '0',
            'initial_reset': 'true',
            'inter_cam_sync_mode': '2',
            'depth_module.min_distance': '0.3',
            'depth_module.max_distance': '20.0',
            'depth_module.visual_preset': '4',
            'spatial_filter.enable': 'true',
            'spatial_filter.magnitude': '2',
            'spatial_filter.smooth_alpha': '0.5',
            'spatial_filter.smooth_delta': '20',
            'temporal_filter.enable': 'true',
            'temporal_filter.smooth_alpha': '0.4',
            'temporal_filter.smooth_delta': '20',
            'depth_module.enable_auto_exposure': 'true',
            'rgb_camera.enable_auto_exposure': 'true'
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
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'depth_module.depth_profile': '424,240,30',
            'rgb_camera.color_profile': '424,240,30',
            'depth_module.emitter_enabled': '1',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'initial_reset': 'true',
            'inter_cam_sync_mode': '2',
            'depth_module.min_distance': '0.3',
            'depth_module.max_distance': '20.0',
            'depth_module.visual_preset': '4',
            'spatial_filter.enable': 'true',
            'spatial_filter.magnitude': '2',
            'spatial_filter.smooth_alpha': '0.5',
            'spatial_filter.smooth_delta': '20',
            'temporal_filter.enable': 'true',
            'temporal_filter.smooth_alpha': '0.4',
            'temporal_filter.smooth_delta': '20',
            'depth_module.enable_auto_exposure': 'true',
            'rgb_camera.enable_auto_exposure': 'true'
        }.items()
    )
    
    # Down D415 - SLAVE
    down_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py']),
        launch_arguments={
            'camera_namespace': 'camera_down',
            'camera_name': 'camera_down',
            'serial_no': ['_', LaunchConfiguration('down_serial')],
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'depth_module.depth_profile': '424,240,15',
            'rgb_camera.color_profile': '424,240,15',
            'depth_module.emitter_enabled': '1',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'initial_reset': 'true',
            'inter_cam_sync_mode': '2',
            'depth_module.min_distance': '0.1',
            'depth_module.max_distance': '10.0',
            'depth_module.visual_preset': '4',
            'spatial_filter.enable': 'true',
            'spatial_filter.magnitude': '2',
            'spatial_filter.smooth_alpha': '0.5',
            'spatial_filter.smooth_delta': '20',
            'temporal_filter.enable': 'true',
            'temporal_filter.smooth_alpha': '0.4',
            'temporal_filter.smooth_delta': '20',
            'depth_module.enable_auto_exposure': 'true',
            'rgb_camera.enable_auto_exposure': 'true'
        }.items()
    )

    # RGBD Sync nodes
    front_rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        output='screen',
        name='rgbd_sync_front',
        namespace='camera_front',
        parameters=[{
            'approx_sync': True,
            'approx_sync_max_interval': 0.1,
            'use_sim_time': False,
            'queue_size': 50,
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
            'approx_sync': True,
            'approx_sync_max_interval': 0.1,
            'use_sim_time': False,
            'queue_size': 50,
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
            'approx_sync': True,
            'approx_sync_max_interval': 0.1,
            'use_sim_time': False,
            'queue_size': 50,
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
        arguments=['0.107', '0.0', '-0.044', '0', '0', '0', 'base_link', 'camera_front_link']
    )

    left_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_left_camera_tf',
        arguments=['0', '0.102', '-0.044', '1.5708', '0', '0', 'base_link', 'camera_left_link']
    )

    right_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_right_camera_tf',
        arguments=['0', '-0.102', '-0.044', '-1.5708', '0', '0', 'base_link', 'camera_right_link']
    )
    down_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_down_camera_tf',
        arguments=['0.07', '0.0', '-0.077', '0', '-1.5708', '0', 'base_link', 'camera_down_link']
    )
    back_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_back_camera_tf',
        arguments=['-0.107', '0.0', '-0.044', '3.1416', '0', '0', 'base_link', 'camera_back_link']
    )

    # RTAB-Map visual odometry
    rtabmap_odom = Node(
    package='rtabmap_odom',
    executable='rgbd_odometry',
    name='rtabmap_odom',
    output='screen',
    parameters=[{
        # Frame Configuration
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'publish_tf': True,
        'wait_for_transform': 0.5,
        
        # Synchronization - Critical for multi-camera
        'approx_sync': True,
        'queue_size': 5,  # Increased for better sync
        'slop': 0.15,  # More timing slack for multi-camera
        'always_process_most_recent_frame': False,
        
        # Multi-camera setup
        'subscribe_rgbd': True,
        'subscribe_imu': False,
        'rgbd_cameras': 3,  # Using 3 cameras
        
        # ═══════════════════════════════════════════════════════════
        # REGISTRATION STRATEGY - Critical for low-texture
        # ═══════════════════════════════════════════════════════════
        'Reg/Strategy': '1',  # Changed to 1 (ICP) for better low-texture handling
        'Reg/Force3DoF': 'false',  # Allow full 6DOF
        
        # ═══════════════════════════════════════════════════════════
        # VISUAL ODOMETRY - Optimized for sparse features
        # ═══════════════════════════════════════════════════════════
        'Vis/FeatureType': '8',  # ORB features (good for low-texture)
        'Vis/MaxFeatures': '3000',  # Significantly increased to find more features
        'Vis/MinInliers': '5',  # REDUCED - more permissive for sparse features
        'Vis/Iterations': '300',  # More iterations for convergence
        'Vis/InlierDistance': '0.3',  # INCREASED - more permissive
        'Vis/CorNNDR': '0.85',  # INCREASED - more permissive matching ratio
        'Vis/MaxDepth': '5.0',  # Increased to use more of the scene
        'Vis/MinDepth': '0.1',  # Avoid very close features
        'Vis/EstimationType': '1',  # PnP estimation
        'Vis/PnPReprojError': '3.0',  # More permissive reprojection error
        'Vis/EpipolarGeometryVar': '0.1',  # More permissive epipolar check
        
        # Feature Detection Strategy
        'Kp/DetectorStrategy': '8',  # ORB detector
        'GFTT/Gpu': 'true',
        'GFTT/MinDistance': '3',  # Reduced to allow more features
        'GFTT/QualityLevel': '0.0001',  # REDUCED - detect weaker corners
        
        # ORB specific parameters
        'ORB/EdgeThreshold': '15',  # Reduced to detect more edge features
        'ORB/ScaleFactor': '1.2',
        'ORB/NLevels': '8',
        'ORB/Gpu': 'true',
        
        # ═══════════════════════════════════════════════════════════
        # ODOMETRY STRATEGY - Frame-to-Map with ICP fallback
        # ═══════════════════════════════════════════════════════════
        'Odometry/Strategy': '0',  # Frame-to-Map
        'Odometry/MaxFeatures': '3000',
        'Odometry/MinInliers': '5',  # REDUCED for sparse features
        'Odometry/Iterations': '300',
        'Odom/FilteringStrategy': '1',
        'Odom/GuessMotion': 'true',  # ENABLED - use motion model
        'Odom/ResetCountdown': '5',  # Reset after 5 failed frames
        'Odom/FillInfoData': 'true',
        'Odom/Holonomic': 'false',  # Assume non-holonomic motion
        
        # ICP parameters - Important for low-texture areas
        'Icp/MaxTranslation': '2.0',  # Allow larger translations
        'Icp/MaxRotation': '0.78',  # ~45 degrees
        'Icp/VoxelSize': '0.05',  # Point cloud downsampling
        'Icp/PointToPlane': 'true',  # Better for planar surfaces
        'Icp/Iterations': '50',
        'Icp/Epsilon': '0.001',
        'Icp/MaxCorrespondenceDistance': '0.3',  # Increased
        'Icp/CorrespondenceRatio': '0.2',  # REDUCED - more permissive
        'Icp/OutlierRatio': '0.85',  # Allow more outliers
        
        # ═══════════════════════════════════════════════════════════
        # RGBD PARAMETERS
        # ═══════════════════════════════════════════════════════════
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityPathMaxNeighbors': '5',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'RGBD/OptimizeMaxError': '10.0',  # Increased tolerance
        'RGBD/LocalRadius': '5',  # Reduced for faster processing
        'RGBD/LocalImmunizationRatio': '0.5',
        'RGBD/LocalLoopDetectionRadius': '15',  # Increased for warehouse
        'RGBD/LocalLoopDetectionTime': '15',
        'RGBD/LinearUpdate': '0.05',  # Update every 5cm
        'RGBD/AngularUpdate': '0.05',  # Update every ~3 degrees
        
        # ═══════════════════════════════════════════════════════════
        # GRID/OCCUPANCY PARAMETERS - For navigation
        # ═══════════════════════════════════════════════════════════
        'Grid/RangeMax': '5.0',  # Increased for warehouse
        'Grid/CellSize': '0.05',
        'Grid/MaxObstacleHeight': '3.0',
        'Grid/MinClusterSize': '15',  # Reduced
        'Grid/NormalsSegmentation': 'true',
        'Grid/FlatObstacleDetected': 'true',
        'Grid/GroundIsObstacle': 'false',
        'Grid/NoiseFilteringRadius': '0.1',  # Reduced
        'Grid/NoiseFilteringMinNeighbors': '10',  # Reduced
        'Grid/Sensor': '0',  # Depth camera
        
        # ═══════════════════════════════════════════════════════════
        # MEMORY MANAGEMENT
        # ═══════════════════════════════════════════════════════════
        'Mem/STMSize': '50',  # Increased short-term memory
        'Mem/RecentWmRatio': '0.3',  # Balance between recent and long-term
        'Mem/LocalSpaceLinksKept': 'true',
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',
        'Mem/ReduceGraph': 'false',  # Don't reduce graph for better accuracy
        
        # ═══════════════════════════════════════════════════════════
        # LOOP CLOSURE - Relaxed for warehouse
        # ═══════════════════════════════════════════════════════════
        'Rtabmap/DetectionRate': '1',  # Detect every frame
        'Rtabmap/LoopThr': '0.11',  # REDUCED - more permissive
        'Rtabmap/LoopRatio': '0.5',  # Less strict
        'Rtabmap/TimeThr': '2000',  # Increased time threshold
        'Rtabmap/MemoryThr': '0',  # Unlimited memory
        'Rtabmap/StartNewMapOnLoopClosure': 'false',
        
        # ═══════════════════════════════════════════════════════════
        # OPTIMIZER - g2o for better optimization
        # ═══════════════════════════════════════════════════════════
        'Optimizer/Strategy': '1',  # g2o optimizer (better than TORO)
        'Optimizer/Iterations': '150',  # More iterations
        'Optimizer/Epsilon': '0.0001',
        'Optimizer/Robust': 'true',
        'Optimizer/VarianceIgnored': 'false',
        'g2o/Solver': '0',  # PCG solver
        'g2o/Optimizer': '0',  # Levenberg-Marquardt
        'g2o/PixelVariance': '0.5',  # INCREASED - less confident in pixels
        'g2o/RobustKernelDelta': '8.0',
        
        # ═══════════════════════════════════════════════════════════
        # MULTI-CAMERA STEREO PARAMETERS
        # ═══════════════════════════════════════════════════════════
        'Stereo/MaxDisparity': '128.0',
        'Stereo/OpticalFlow': 'true',
        'Stereo/Iterations': '50',
        'Stereo/Eps': '0.01',
        'Stereo/MaxLevel': '3',
        'Stereo/WinSize': '15',
        
        # ═══════════════════════════════════════════════════════════
        # DIAGNOSTIC & OUTPUT
        # ═══════════════════════════════════════════════════════════
        'Rtabmap/PublishStats': 'true',
        'Rtabmap/PublishLastSignature': 'true',
        'Rtabmap/PublishLikelihood': 'true',
        'Rtabmap/PublishPdf': 'false',
        'Rtabmap/StatisticLogsBufferedInRAM': 'false',
        
    }],
    remappings=[
        ('rgbd_image0', '/camera_front/rgbd_image'),
        ('rgbd_image1', '/camera_right/rgbd_image'),
        ('rgbd_image2', '/camera_left/rgbd_image'),
    ]
)
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_depth': False,
            'subscribe_rgbd': True,
            'subscribe_rgb': False,
            'subscribe_stereo': False,
            'subscribe_scan': False,  # NO laser scan
            'subscribe_scan_cloud': False,  # NO point cloud scan
            'subscribe_user_data': False,
            'subscribe_odom_info': True,
            'wait_for_transform': 0.5,
            'rgbd_cameras': 3,
            'approx_sync': True,
            'wait_imu_to_init': False,
            
            'queue_size': 50,
            'publish_tf': True,
            'map_frame_id': 'map',
            'odom_frame_id': 'odom',
            
            # Detection and loop closure - Enhanced stability
            'Vis/FeatureType': '8',  # Using ORB features for Jetson
          'Vis/MaxFeatures': '2000',  # Increased features
          'Vis/MinInliers': '8',    # More permissive
          'Vis/Iterations': '250',   # More iterations
          'Vis/InlierDistance': '0.2', # More permissive distance
          'Vis/CorNNDR': '0.8',     # More permissive matching
          'Vis/MaxDepth': '3.0',    # Increased max depth
          'Vis/MinDepth': '0.0',    # Minimum depth threshold
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
          'Odometry/Strategy': '0',    # Frame-to-Map (more robust)
          'Odometry/MaxFeatures': '2000',   # Increased features
          'Odometry/MinInliers': '8',      # More permissive
          'Odometry/Iterations': '250',     # Balanced iterations
          'Odom/FilteringStrategy': '1',
          'Odom/GuessMotion': 'false',
          'RGBD/NeighborLinkRefining': 'True',
          # Loop Closure and Map Management
          'RGBD/LocalImmunizationRatio': '0.4',  # Increased from default 0.25
          'Mem/LocalSpaceLinksKept': 'true',    # Keep local space links
          'RGBD/LocalLoopDetectionRadius': '10', # Local loop closure radius
          'RGBD/LocalLoopDetectionTime': '10',   # Time window for local loop detection
          'Rtabmap/DetectionRate': '2',     # Increased detection rate
          'Rtabmap/LoopThr': '0.15',        # More permissive loop closure
          'Optimizer/Iterations': '100',     # More iterations for better optimization
          'Optimizer/Strategy': '0',
          'Vis/EstimationType': '1',         # Added: Use PnP estimation
          # Memory and Performance
          'Mem/STMSize': '30',
          'Mem/RecentWmRatio': '0.4',
          'Rtabmap/TimeThr': '1000'
        }],
        remappings=[
            ('odom', '/odom'),
            ('rgbd_image0', '/camera_front/rgbd_image'),
            ('rgbd_image1', '/camera_right/rgbd_image'),
            ('rgbd_image2', '/camera_left/rgbd_image')
        ],
        arguments=['-d']  # Delete old database
    )
    # Odometry to Vision Pose Bridge (for MAVROS VIO)
    odom_to_vision = Node(
        package='slam',  # Replace with your package name
        executable='odom_converter',
        name='odom_to_vision_pose',
        output='screen',
        parameters=[{
            'use_enu_to_ned': True,  # Set to True if using ArduPilot/PX4
            'send_twist': True
        }]
    )

    # Rangefinder Node (for terrain following/POSHOLD/LOITER)
    rangefinder_node = Node(
        package='slam',  # Replace with your package name
        executable='rangefinder',
        name='rangefinder_node',
        output='screen'
    )

    return LaunchDescription([
        # Camera serial number arguments
        declare_front_serial,
        declare_right_serial,
        declare_left_serial,
        declare_down_serial,
        # Transform nodes
        front_tf,
        right_tf,
        left_tf,
        down_tf,
        # Camera nodes
        front_camera_launch,
        right_camera_launch,
        left_camera_launch,
        down_camera_launch,
        # RGBD sync nodes
        front_rgbd_sync,
        right_rgbd_sync,
        left_rgbd_sync,
        # SLAM nodes
        rtabmap_odom,
        rtabmap_node,
        # MAVROS bridge nodes
        odom_to_vision,
        rangefinder_node,
        
    ])