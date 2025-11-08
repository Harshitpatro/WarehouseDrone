#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation
from collections import deque
import threading

class MultiCameraVIO(Node):
    def __init__(self):
        super().__init__('multi_camera_vio')
        
        # Parameters
        self.declare_parameter('use_cuda', True)
        self.declare_parameter('feature_type', 'ORB')  # ORB, SIFT, or AKAZE
        self.declare_parameter('max_features', 2000)
        self.declare_parameter('scale_factor', 1.2)
        self.declare_parameter('n_levels', 8)
        self.declare_parameter('single_camera_mode', False)  # Use only one camera if true
        
        self.use_cuda = self.get_parameter('use_cuda').value
        self.feature_type = self.get_parameter('feature_type').value
        self.single_camera_mode = self.get_parameter('single_camera_mode').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # RealSense pipelines
        self.pipelines = {}
        self.configs = {}
        self.setup_cameras()
        
        # Feature detector
        self.setup_feature_detector()
        
        # Visual Odometry state
        self.prev_frames = {'front': None, 'left': None, 'right': None}
        self.prev_kpts = {'front': None, 'left': None, 'right': None}
        self.prev_desc = {'front': None, 'left': None, 'right': None}
        
        # Pose estimation
        self.current_pose = np.eye(4)
        self.pose_covariance = np.eye(6) * 0.01
        
        # IMU integration
        self.imu_buffer = deque(maxlen=100)
        self.last_imu_time = None
        
        # Kalman filter for pose fusion
        self.setup_kalman_filter()
        
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10
        )
        
        # Timer for processing
        self.create_timer(0.033, self.process_vio)  # 30Hz
        
        self.get_logger().info('Multi-Camera VIO Node Initialized')
        
    def setup_cameras(self):
        """Initialize RealSense cameras"""
        ctx = rs.context()
        devices = ctx.query_devices()
        
        if len(devices) == 0:
            self.get_logger().error('No RealSense devices found!')
            return
        
        camera_serials = {
            'front': '310222076155',  # D435i
            'left': '319522067209',   # D415
            'right': '327322061348'   # D415
        }
        
        # Identify cameras by serial number
        d435_count = 0
        d415_count = 0
        
        for dev in devices:
            serial = dev.get_info(rs.camera_info.serial_number)
            name = dev.get_info(rs.camera_info.name)
            
            self.get_logger().info(f'Found camera: {name} - Serial: {serial}')
            
            if 'D435' in name:
                camera_serials['front'] = serial
                d435_count += 1
            elif 'D415' in name:
                if camera_serials['left'] is None:
                    camera_serials['left'] = serial
                else:
                    camera_serials['right'] = serial
                d415_count += 1
        
        self.get_logger().info(f'Camera count - D435: {d435_count}, D415: {d415_count}')
        
        # Configure each camera with proper context handling
        for cam_name, serial in camera_serials.items():
            if serial is None:
                self.get_logger().warn(f'No serial found for {cam_name} camera')
                continue
                
            try:
                # Create separate pipeline and config for each camera
                pipeline = rs.pipeline(ctx)
                config = rs.config()
                
                # Enable specific device by serial
                config.enable_device(serial)
                
                # Get device to check available streams
                dev = None
                for device in devices:
                    if device.get_info(rs.camera_info.serial_number) == serial:
                        dev = device
                        break
                
                if dev is None:
                    self.get_logger().error(f'Could not find device with serial {serial}')
                    continue
                
                # Configure streams based on camera type
                # Use lower resolution and frame rate for stability with multiple cameras
                if 'D435' in dev.get_info(rs.camera_info.name):
                    # D435i configuration
                    config.enable_stream(rs.stream.infrared, 1, 424, 240, rs.format.y8, 30)
                    config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 30)
                    
                    # Enable IMU for D435i
                    try:
                        config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
                        config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
                        self.get_logger().info(f'{cam_name}: IMU enabled')
                    except Exception as e:
                        self.get_logger().warn(f'{cam_name}: Could not enable IMU: {e}')
                else:
                    # D415 configuration
                    config.enable_stream(rs.stream.infrared, 1, 424, 240, rs.format.y8, 30)
                    config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 30)
                
                # Start pipeline with config
                self.get_logger().info(f'Starting {cam_name} camera...')
                profile = pipeline.start(config)
                
                # Get device and disable emitter for better feature tracking
                device = profile.get_device()
                depth_sensor = device.first_depth_sensor()
                
                # Disable emitter for infrared cameras (better for feature tracking)
                if depth_sensor.supports(rs.option.emitter_enabled):
                    depth_sensor.set_option(rs.option.emitter_enabled, 0)
                    self.get_logger().info(f'{cam_name}: IR emitter disabled')
                
                # Set exposure for better feature detection
                if depth_sensor.supports(rs.option.exposure):
                    depth_sensor.set_option(rs.option.exposure, 8500)
                    self.get_logger().info(f'{cam_name}: Exposure set to 8500')
                
                # Get intrinsics from infrared stream
                infrared_stream = profile.get_stream(rs.stream.infrared, 1)
                intrinsics = infrared_stream.as_video_stream_profile().get_intrinsics()
                
                self.pipelines[cam_name] = {
                    'pipeline': pipeline,
                    'config': config,
                    'profile': profile,
                    'intrinsics': intrinsics,
                    'K': np.array([[intrinsics.fx, 0, intrinsics.ppx],
                                   [0, intrinsics.fy, intrinsics.ppy],
                                   [0, 0, 1]]),
                    'serial': serial
                }
                
                self.get_logger().info(f'✓ Initialized {cam_name} camera: {serial}')
                
                # Small delay between camera initializations
                import time
                time.sleep(0.5)
                
            except Exception as e:
                self.get_logger().error(f'Failed to initialize {cam_name} camera ({serial}): {e}')
                continue
        
        if len(self.pipelines) == 0:
            self.get_logger().error('No cameras initialized successfully!')
        else:
            self.get_logger().info(f'Successfully initialized {len(self.pipelines)} camera(s)')
    
    def setup_feature_detector(self):
        """Setup feature detector with CUDA if available"""
        if self.use_cuda and cv2.cuda.getCudaEnabledDeviceCount() > 0:
            try:
                if self.feature_type == 'ORB':
                    self.detector = cv2.cuda.ORB_create(
                        nfeatures=self.get_parameter('max_features').value,
                        scaleFactor=self.get_parameter('scale_factor').value,
                        nlevels=self.get_parameter('n_levels').value
                    )
                self.use_cuda = True
                self.get_logger().info('Using CUDA-accelerated feature detection')
            except Exception as e:
                self.get_logger().warn(f'CUDA init failed: {e}, falling back to CPU')
                self.use_cuda = False
        
        if not self.use_cuda:
            if self.feature_type == 'ORB':
                self.detector = cv2.ORB_create(
                    nfeatures=self.get_parameter('max_features').value,
                    scaleFactor=self.get_parameter('scale_factor').value,
                    nlevels=self.get_parameter('n_levels').value
                )
            elif self.feature_type == 'SIFT':
                self.detector = cv2.SIFT_create(
                    nfeatures=self.get_parameter('max_features').value
                )
            elif self.feature_type == 'AKAZE':
                self.detector = cv2.AKAZE_create()
        
        # Matcher
        if self.feature_type == 'ORB':
            self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        else:
            self.matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
    
    def setup_kalman_filter(self):
        """Setup Extended Kalman Filter for pose estimation"""
        # State: [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]
        self.kf_state = np.zeros(12)
        self.kf_P = np.eye(12) * 0.1
        
        # Process noise
        self.kf_Q = np.eye(12) * 0.01
        self.kf_Q[6:9, 6:9] *= 0.1  # Higher velocity noise
        
        # Measurement noise
        self.kf_R_vision = np.eye(6) * 0.1
        self.kf_R_imu = np.eye(6) * 0.01
    
    def process_vio(self):
        """Main VIO processing loop"""
        try:
            # Capture frames from all cameras
            frames = {}
            depths = {}
            
            for cam_name, cam_data in self.pipelines.items():
                try:
                    # Use poll_for_frames instead of wait_for_frames for non-blocking
                    frameset = cam_data['pipeline'].poll_for_frames()
                    
                    if not frameset:
                        continue
                    
                    # Get infrared frame (better for feature tracking)
                    ir_frame = frameset.get_infrared_frame(1)
                    depth_frame = frameset.get_depth_frame()
                    
                    if ir_frame and depth_frame:
                        frames[cam_name] = np.asanyarray(ir_frame.get_data())
                        depths[cam_name] = np.asanyarray(depth_frame.get_data())
                    
                    # Get IMU data from front camera (D435i)
                    if cam_name == 'front':
                        self.process_imu_data(frameset)
                        
                except Exception as e:
                    self.get_logger().debug(f'Frame capture error for {cam_name}: {e}')
                    continue
            
            if len(frames) == 0:
                return
            
            # Process each camera
            delta_poses = []
            weights = []
            
            for cam_name in ['front', 'left', 'right']:
                if cam_name not in frames:
                    continue
                
                frame = frames[cam_name]
                depth = depths[cam_name]
                K = self.pipelines[cam_name]['K']
                
                # Track features and estimate motion
                delta_pose, confidence = self.estimate_camera_motion(
                    frame, depth, cam_name, K
                )
                
                if delta_pose is not None:
                    delta_poses.append(delta_pose)
                    weights.append(confidence)
            
            if len(delta_poses) == 0:
                return
            
            # Fuse multiple camera estimates
            weights = np.array(weights)
            weights = weights / weights.sum()
            
            # Weighted average of translations
            delta_t = np.zeros(3)
            for i, dp in enumerate(delta_poses):
                delta_t += weights[i] * dp[:3, 3]
            
            # Average rotations using quaternions
            delta_R = self.average_rotations([dp[:3, :3] for dp in delta_poses], weights)
            
            # Update pose
            delta_pose = np.eye(4)
            delta_pose[:3, :3] = delta_R
            delta_pose[:3, 3] = delta_t
            
            self.current_pose = self.current_pose @ delta_pose
            
            # Kalman filter update with vision measurement
            self.update_kalman_with_vision(delta_pose)
            
            # Publish pose
            self.publish_pose()
            
        except Exception as e:
            self.get_logger().error(f'VIO processing error: {e}')
    
    def estimate_camera_motion(self, frame, depth, cam_name, K):
        """Estimate motion from a single camera"""
        if self.prev_frames[cam_name] is None:
            # Initialize
            if self.use_cuda:
                gpu_frame = cv2.cuda_GpuMat()
                gpu_frame.upload(frame)
                kpts, desc = self.detector.detectAndComputeAsync(gpu_frame, None)
                kpts = self.detector.convert(kpts)
                desc = desc.download()
            else:
                kpts, desc = self.detector.detectAndCompute(frame, None)
            
            self.prev_frames[cam_name] = frame
            self.prev_kpts[cam_name] = kpts
            self.prev_desc[cam_name] = desc
            return None, 0.0
        
        # Detect features in current frame
        if self.use_cuda:
            gpu_frame = cv2.cuda_GpuMat()
            gpu_frame.upload(frame)
            curr_kpts, curr_desc = self.detector.detectAndComputeAsync(gpu_frame, None)
            curr_kpts = self.detector.convert(curr_kpts)
            curr_desc = curr_desc.download()
        else:
            curr_kpts, curr_desc = self.detector.detectAndCompute(frame, None)
        
        if len(curr_kpts) < 10:
            return None, 0.0
        
        # Match features
        matches = self.matcher.knnMatch(self.prev_desc[cam_name], curr_desc, k=2)
        
        # Apply ratio test
        good_matches = []
        for m_n in matches:
            if len(m_n) == 2:
                m, n = m_n
                if m.distance < 0.75 * n.distance:
                    good_matches.append(m)
        
        if len(good_matches) < 10:
            return None, 0.0
        
        # Get matched points with depth
        pts_prev_3d = []
        pts_curr_2d = []
        
        for m in good_matches:
            pt_prev = self.prev_kpts[cam_name][m.queryIdx].pt
            pt_curr = curr_kpts[m.trainIdx].pt
            
            # Get depth at previous point
            x, y = int(pt_prev[0]), int(pt_prev[1])
            if 0 <= x < depth.shape[1] and 0 <= y < depth.shape[0]:
                d = depth[y, x] * 0.001  # Convert to meters
                
                if 0.1 < d < 10.0:  # Valid depth range
                    # Back-project to 3D
                    pt_3d = np.array([
                        (pt_prev[0] - K[0, 2]) * d / K[0, 0],
                        (pt_prev[1] - K[1, 2]) * d / K[1, 1],
                        d
                    ])
                    pts_prev_3d.append(pt_3d)
                    pts_curr_2d.append(pt_curr)
        
        if len(pts_prev_3d) < 10:
            return None, 0.0
        
        pts_prev_3d = np.array(pts_prev_3d)
        pts_curr_2d = np.array(pts_curr_2d)
        
        # PnP RANSAC to estimate pose
        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            pts_prev_3d, pts_curr_2d, K, None,
            iterationsCount=500,
            reprojectionError=2.0,
            confidence=0.99
        )
        
        if not success or inliers is None or len(inliers) < 10:
            return None, 0.0
        
        # Convert to transformation matrix
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec.flatten()
        
        # Calculate confidence based on inlier ratio
        confidence = len(inliers) / len(pts_prev_3d)
        
        # Update for next iteration
        self.prev_frames[cam_name] = frame
        self.prev_kpts[cam_name] = curr_kpts
        self.prev_desc[cam_name] = curr_desc
        
        return T, confidence
    
    def process_imu_data(self, frameset):
        """Process IMU data from D435i"""
        try:
            accel = frameset.first_or_default(rs.stream.accel)
            gyro = frameset.first_or_default(rs.stream.gyro)
            
            if accel and gyro:
                accel_data = accel.as_motion_frame().get_motion_data()
                gyro_data = gyro.as_motion_frame().get_motion_data()
                
                timestamp = accel.get_timestamp()
                
                self.imu_buffer.append({
                    'timestamp': timestamp,
                    'accel': np.array([accel_data.x, accel_data.y, accel_data.z]),
                    'gyro': np.array([gyro_data.x, gyro_data.y, gyro_data.z])
                })
                
                # Update Kalman filter with IMU
                if self.last_imu_time is not None:
                    dt = (timestamp - self.last_imu_time) / 1000.0
                    self.predict_kalman_with_imu(
                        np.array([accel_data.x, accel_data.y, accel_data.z]),
                        np.array([gyro_data.x, gyro_data.y, gyro_data.z]),
                        dt
                    )
                
                self.last_imu_time = timestamp
        except Exception as e:
            pass
    
    def predict_kalman_with_imu(self, accel, gyro, dt):
        """Predict Kalman state using IMU"""
        # Simple integration
        # State: [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]
        
        # Predict velocity
        self.kf_state[6:9] += accel * dt
        
        # Predict position
        self.kf_state[0:3] += self.kf_state[6:9] * dt
        
        # Predict orientation
        self.kf_state[3:6] += gyro * dt
        
        # Predict covariance
        F = np.eye(12)
        F[0:3, 6:9] = np.eye(3) * dt
        
        self.kf_P = F @ self.kf_P @ F.T + self.kf_Q
    
    def update_kalman_with_vision(self, delta_pose):
        """Update Kalman filter with vision measurement"""
        # Extract position and orientation from delta pose
        delta_t = delta_pose[:3, 3]
        delta_R = delta_pose[:3, :3]
        delta_euler = Rotation.from_matrix(delta_R).as_euler('xyz')
        
        # Measurement
        z = np.hstack([delta_t, delta_euler])
        
        # Measurement matrix (we observe position and orientation)
        H = np.zeros((6, 12))
        H[0:3, 0:3] = np.eye(3)
        H[3:6, 3:6] = np.eye(3)
        
        # Innovation
        y = z - H @ self.kf_state
        
        # Innovation covariance
        S = H @ self.kf_P @ H.T + self.kf_R_vision
        
        # Kalman gain
        K = self.kf_P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.kf_state = self.kf_state + K @ y
        
        # Update covariance
        self.kf_P = (np.eye(12) - K @ H) @ self.kf_P
    
    def average_rotations(self, rotations, weights):
        """Average rotation matrices using quaternions"""
        quats = [Rotation.from_matrix(R).as_quat() for R in rotations]
        
        # Weighted average
        avg_quat = np.zeros(4)
        for i, q in enumerate(quats):
            avg_quat += weights[i] * q
        
        avg_quat = avg_quat / np.linalg.norm(avg_quat)
        
        return Rotation.from_quat(avg_quat).as_matrix()
    
    def publish_pose(self):
        """Publish VIO pose to MAVROS"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Position from Kalman filter (more stable)
        msg.pose.position.x = float(self.kf_state[0])
        msg.pose.position.y = float(self.kf_state[1])
        msg.pose.position.z = float(self.kf_state[2])
        
        # Orientation from Kalman filter
        euler = self.kf_state[3:6]
        quat = Rotation.from_euler('xyz', euler).as_quat()
        
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        
        self.pose_pub.publish(msg)
    
    def destroy_node(self):
        """Cleanup"""
        for cam_data in self.pipelines.values():
            cam_data['pipeline'].stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraVIO()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()