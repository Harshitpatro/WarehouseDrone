#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, FluidPressure
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import numpy as np
from collections import deque
import cv2

class HeightFusionNode(Node):
    def __init__(self):
        super().__init__('height_fusion_node')
        
        # Parameters
        self.declare_parameter('camera_depth_topic', '/camera_down/camera_down/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_down/camera_down/color/camera_info')
        self.declare_parameter('barometer_topic', '/mavros/imu/static_pressure')
        self.declare_parameter('fused_height_topic', '/fused_height')
        self.declare_parameter('max_camera_range', 6.0)  # Maximum reliable camera range
        self.declare_parameter('outlier_threshold', 0.8)  # Increased for more tolerance
        self.declare_parameter('use_median_filter', True)
        self.declare_parameter('median_window_size', 7)  # Increased for smoother filtering
        self.declare_parameter('center_region_size', 0.4)  # Use center 40% of image
        self.declare_parameter('sea_level_pressure', 101325.0)  # Pa, adjust based on location
        self.declare_parameter('temperature', 15.0)  # Celsius, adjust based on conditions
        
        # Get parameters
        self.camera_depth_topic = self.get_parameter('camera_depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.barometer_topic = self.get_parameter('barometer_topic').value
        self.fused_height_topic = self.get_parameter('fused_height_topic').value
        self.max_camera_range = self.get_parameter('max_camera_range').value
        self.outlier_threshold = self.get_parameter('outlier_threshold').value
        self.use_median_filter = self.get_parameter('use_median_filter').value
        self.median_window_size = self.get_parameter('median_window_size').value
        self.center_region_size = self.get_parameter('center_region_size').value
        self.sea_level_pressure = self.get_parameter('sea_level_pressure').value
        self.temperature = self.get_parameter('temperature').value
        
        # State variables
        self.bridge = CvBridge()
        self.camera_height = None
        self.baro_height = None
        self.baro_height_raw = None
        self.ground_pressure = None  # Will be set when drone is on ground
        self.last_valid_camera_height = None
        self.camera_info = None
        
        # Filtering
        if self.use_median_filter:
            self.camera_height_buffer = deque(maxlen=self.median_window_size)
            self.baro_height_buffer = deque(maxlen=self.median_window_size)
        
        # Kalman filter for smooth fusion
        self.kf_initialized = False
        self.kf_state = 0.0  # Estimated height
        self.kf_variance = 1.0  # Estimate variance
        self.process_noise = 0.01  # Process noise
        
        # Ground pressure calibration
        self.calibration_samples = []
        self.calibration_duration = 3.0  # seconds
        self.is_calibrated = False
        self.calibration_start_time = self.get_clock().now()
        
        # Subscribers
        self.depth_sub = self.create_subscription(
            Image, self.camera_depth_topic, self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)
        self.baro_sub = self.create_subscription(
            FluidPressure, self.barometer_topic, self.baro_callback, 10)
        
        # Publishers
        self.height_pub = self.create_publisher(Float32, self.fused_height_topic, 10)
        self.camera_weight_pub = self.create_publisher(Float32, '/height_fusion/camera_weight', 10)
        self.baro_weight_pub = self.create_publisher(Float32, '/height_fusion/baro_weight', 10)
        
        # Timer for fusion at 20Hz
        self.timer = self.create_timer(0.05, self.fusion_callback)
        
        self.get_logger().info('Height Fusion Node initialized')
        self.get_logger().info('Calibrating barometer... Keep drone stationary for 3 seconds')

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        try:
            # Convert depth image
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Get center region for robust ground detection
            h, w = depth_image.shape
            center_h = int(h * self.center_region_size)
            center_w = int(w * self.center_region_size)
            start_h = (h - center_h) // 2
            start_w = (w - center_w) // 2
            center_region = depth_image[start_h:start_h+center_h, start_w:start_w+center_w]
            
            # Filter out invalid depths (in mm from RealSense)
            valid_depths = center_region[(center_region > 100) & (center_region < self.max_camera_range * 1000)]
            
            if len(valid_depths) > 100:  # Need enough valid points
                # Use median for robustness against outliers
                median_depth = np.median(valid_depths) / 1000.0  # Convert mm to m
                
                # Additional filtering using percentiles to reject outliers
                p25 = np.percentile(valid_depths, 25) / 1000.0
                p75 = np.percentile(valid_depths, 75) / 1000.0
                iqr = p75 - p25
                
                # Check if measurement is reasonable (within IQR bounds)
                if abs(median_depth - np.median([p25, p75])) < iqr * 1.5:
                    if 0.1 < median_depth < self.max_camera_range:
                        if self.use_median_filter:
                            self.camera_height_buffer.append(median_depth)
                            if len(self.camera_height_buffer) >= 3:  # Need some history
                                self.camera_height = np.median(list(self.camera_height_buffer))
                        else:
                            self.camera_height = median_depth
                        
                        self.last_valid_camera_height = self.camera_height
                    else:
                        # Out of camera range
                        self.camera_height = None
                else:
                    # Outlier detected
                    if self.last_valid_camera_height is not None:
                        self.camera_height = None
            else:
                # Not enough valid points
                self.camera_height = None
                
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

    def pressure_to_altitude(self, pressure_pa, reference_pressure_pa):
        """
        Convert pressure to altitude using barometric formula
        h = (T0 / L) * (1 - (P / P0)^(R*L/g*M))
        Simplified: h ≈ 44330 * (1 - (P/P0)^0.1903)
        """
        if reference_pressure_pa <= 0 or pressure_pa <= 0:
            return 0.0
        
        # Barometric formula
        altitude = 44330.0 * (1.0 - (pressure_pa / reference_pressure_pa) ** 0.1903)
        return altitude

    def baro_callback(self, msg):
        """Process barometer data from FluidPressure message"""
        pressure_pa = msg.fluid_pressure
        
        # Calibration phase - determine ground pressure
        if not self.is_calibrated:
            elapsed = (self.get_clock().now() - self.calibration_start_time).nanoseconds / 1e9
            
            if elapsed < self.calibration_duration:
                self.calibration_samples.append(pressure_pa)
                if len(self.calibration_samples) % 10 == 0:
                    self.get_logger().info(f'Calibrating... {elapsed:.1f}s / {self.calibration_duration}s')
            else:
                # Calibration complete
                self.ground_pressure = np.median(self.calibration_samples)
                self.is_calibrated = True
                self.get_logger().info(f'Barometer calibrated! Ground pressure: {self.ground_pressure:.2f} Pa')
            return
        
        # Calculate altitude relative to ground
        baro_height = self.pressure_to_altitude(pressure_pa, self.ground_pressure)
        
        # Filter negative heights
        if baro_height < 0:
            baro_height = 0.0
        
        self.baro_height_raw = baro_height
        
        # Apply median filter
        if self.use_median_filter:
            self.baro_height_buffer.append(baro_height)
            if len(self.baro_height_buffer) >= 3:
                self.baro_height = np.median(list(self.baro_height_buffer))
        else:
            self.baro_height = baro_height

    def calculate_dynamic_weights(self, camera_height):
        """
        Calculate dynamic weights based on camera height:
        - 1m:  90% cam, 10% baro
        - 3m:  60% cam, 40% baro  
        - 6m:  20% cam, 80% baro
        - >6m: 0% cam, 100% baro
        """
        if camera_height >= self.max_camera_range:
            return 0.0, 1.0
        
        # Piecewise linear interpolation
        if camera_height <= 1.0:
            # 0-1m: 100% to 90% camera
            cam_weight = 1.0 - (camera_height * 0.1)
        elif camera_height <= 3.0:
            # 1-3m: 90% to 60% camera
            t = (camera_height - 1.0) / 2.0  # Normalize to [0, 1]
            cam_weight = 0.9 - (t * 0.3)
        elif camera_height <= 6.0:
            # 3-6m: 60% to 20% camera
            t = (camera_height - 3.0) / 3.0  # Normalize to [0, 1]
            cam_weight = 0.6 - (t * 0.4)
        else:
            cam_weight = 0.0
        
        baro_weight = 1.0 - cam_weight
        
        return cam_weight, baro_weight

    def kalman_predict(self):
        """Prediction step"""
        self.kf_variance += self.process_noise

    def kalman_update(self, measurement, measurement_variance):
        """Update step"""
        kalman_gain = self.kf_variance / (self.kf_variance + measurement_variance)
        self.kf_state = self.kf_state + kalman_gain * (measurement - self.kf_state)
        self.kf_variance = (1 - kalman_gain) * self.kf_variance

    def fusion_callback(self):
        if not self.is_calibrated:
            return
        
        if self.baro_height is None:
            return
        
        # Kalman predict
        self.kalman_predict()
        
        # Determine fusion strategy
        if self.camera_height is not None:
            # Camera has valid measurement
            
            if not self.kf_initialized:
                # Initialize Kalman filter with camera measurement
                self.kf_state = self.camera_height
                self.kf_initialized = True
            
            # Calculate dynamic weights based on camera height
            cam_weight, baro_weight = self.calculate_dynamic_weights(self.camera_height)
            
            # Check for gross outliers between sensors
            if self.baro_height > 0.1:  # Baro has valid reading
                height_diff = abs(self.camera_height - self.baro_height)
                relative_diff = height_diff / max(self.camera_height, self.baro_height)
                
                if relative_diff > 0.5 and self.camera_height > 3.0:
                    # Large disagreement at higher altitudes - trust baro more
                    self.get_logger().warn(f'Large sensor disagreement: Cam={self.camera_height:.2f}m, '
                                          f'Baro={self.baro_height:.2f}m. Trusting barometer.')
                    cam_weight = 0.2
                    baro_weight = 0.8
            
            # Weighted fusion
            fused_height = cam_weight * self.camera_height + baro_weight * self.baro_height
            
            # Calculate measurement variance based on weights
            # Lower variance when camera weight is high (more confident)
            measurement_variance = 0.05 + (baro_weight * 0.15)
            
            # Update Kalman filter
            self.kalman_update(fused_height, measurement_variance)
            
            # Publish weights for monitoring
            self.publish_weights(cam_weight, baro_weight)
            
            self.get_logger().debug(f'Fusion: Cam={self.camera_height:.2f}m ({cam_weight*100:.0f}%), '
                                   f'Baro={self.baro_height:.2f}m ({baro_weight*100:.0f}%), '
                                   f'Fused={self.kf_state:.2f}m', 
                                   throttle_duration_sec=1.0)
        
        else:
            # Camera out of range - use barometer only
            if not self.kf_initialized:
                self.kf_state = self.baro_height
                self.kf_initialized = True
            
            # Update Kalman with barometer (higher variance)
            measurement_variance = 0.25
            self.kalman_update(self.baro_height, measurement_variance)
            
            # Publish weights
            self.publish_weights(0.0, 1.0)
            
            self.get_logger().debug(f'Barometer only: {self.baro_height:.2f}m (Camera out of range)', 
                                   throttle_duration_sec=1.0)
        
        # Publish fused height
        if self.kf_initialized:
            msg = Float32()
            msg.data = float(max(0.0, self.kf_state))  # Ensure non-negative
            self.height_pub.publish(msg)

    def publish_weights(self, cam_weight, baro_weight):
        """Publish current fusion weights for monitoring"""
        cam_msg = Float32()
        cam_msg.data = float(cam_weight)
        self.camera_weight_pub.publish(cam_msg)
        
        baro_msg = Float32()
        baro_msg.data = float(baro_weight)
        self.baro_weight_pub.publish(baro_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HeightFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()