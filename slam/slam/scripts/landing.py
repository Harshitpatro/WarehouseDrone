#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge
import time

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from mavros_msgs.msg import LandingTarget, State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from std_srvs.srv import Trigger


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Parameters
        self.declare_parameter('marker_size', 0.15)
        self.declare_parameter('camera_topic', '/camera_down/camera_down/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_down/camera_down/color/camera_info')
        self.declare_parameter('aruco_dict_name', 'DICT_5X5_250')
        self.declare_parameter('show_rejected', False)
        self.declare_parameter('show_axes', True)
        self.declare_parameter('show_markers', True)
        self.declare_parameter('target_frame', 'BODY_FRAME_FRD')
        self.declare_parameter('landing_target_type', 2)
        self.declare_parameter('auto_set_home', True)
        self.declare_parameter('home_set_retry_delay', 2.0)
        self.declare_parameter('home_set_max_retries', 5)
        self.declare_parameter('vio_position_std_threshold', 0.01)  # 10mm total movement - adjusted for VIO noise
        
        # Get parameters
        self.marker_size = self.get_parameter('marker_size').value
        self.show_rejected = self.get_parameter('show_rejected').value
        self.show_axes = self.get_parameter('show_axes').value
        self.show_markers = self.get_parameter('show_markers').value
        self.target_frame = self.get_parameter('target_frame').value
        self.landing_target_type = self.get_parameter('landing_target_type').value
        self.auto_set_home = self.get_parameter('auto_set_home').value
        self.home_retry_delay = self.get_parameter('home_set_retry_delay').value
        self.home_max_retries = self.get_parameter('home_set_max_retries').value
        self.vio_std_threshold = self.get_parameter('vio_position_std_threshold').value
        
        # State tracking
        self.vehicle_state = None
        self.ekf_origin_set = False
        self.home_set = False
        self.current_marker_pose = None
        self.initialization_in_progress = False
        self.ekf_origin_attempts = 0
        self.home_set_attempts = 0
        self.last_init_attempt = 0.0
        
        # VIO stability tracking using position history
        self.vio_stable = False
        self.vio_stable_count = 0
        self.position_history = []
        self.last_pose_time = 0.0
        
        # ArUco setup
        dict_name = self.get_parameter('aruco_dict_name').value
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        self.aruco_params = cv2.aruco.DetectorParameters()

        # Camera calibration
        self.camera_matrix = None
        self.dist_coeffs = None

        # CV Bridge
        self.bridge = CvBridge()

        # QoS profile for camera subscription
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.get_parameter('camera_topic').value,
            self.image_callback,
            sensor_qos
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.camera_info_callback,
            sensor_qos
        )

        # Publishers
        self.image_pub = self.create_publisher(
            Image,
            '~/detected_markers',
            10
        )
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '~/marker_pose',
            10
        )
        self.marker_pub = self.create_publisher(
            Marker,
            '~/marker_visual',
            10
        )
        
        # MAVROS publishers
        self.landing_target_pub = self.create_publisher(
            LandingTarget,
            '/mavros/landing_target/raw',
            10
        )
        
        # MAVROS subscribers
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )
        
            # Subscribe to vision pose with reliable QoS
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        self.vision_pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/vision_pose/pose',
            self.vision_pose_callback,
            reliable_qos
        )        # MAVROS service clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')
        
        # Services
        self.set_home_srv = self.create_service(
            Trigger,
            '~/set_home',
            self.set_home_callback
        )

        self.get_logger().info('ArUco detector node initialized')
        self.get_logger().info(f'Watching for markers on {self.get_parameter("camera_topic").value}')
        self.get_logger().info(f'VIO stability threshold: {self.vio_std_threshold*1000:.1f}mm std dev')

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera calibration received')

    def state_callback(self, msg: State):
        """Callback for vehicle state updates"""
        self.vehicle_state = msg
    
    def vision_pose_callback(self, msg: PoseStamped):
        """Monitor vision pose for VIO stability"""
        current_time = time.time()
        
        # Check update rate
        if self.last_pose_time > 0:
            dt = current_time - self.last_pose_time
            if dt > 0.5:  # More than 500ms gap - adjusted for system characteristics
                self.vio_stable = False
                self.vio_stable_count = 0
                self.position_history.clear()
                self.get_logger().warn(f"⚠ Vision pose updates too slow ({dt:.3f}s gap)")
                self.last_pose_time = current_time
                return
        
        self.last_pose_time = current_time
        
        # Store position
        pos = msg.pose.position
        self.position_history.append([pos.x, pos.y, pos.z])
        
        # Keep only last 30 samples (~1 second at 30Hz)
        if len(self.position_history) > 30:
            self.position_history.pop(0)
        
        # Check stability once we have enough samples
        if len(self.position_history) >= 30:
            positions = np.array(self.position_history)
            
            # Calculate standard deviation per axis
            std_x = np.std(positions[:, 0])
            std_y = np.std(positions[:, 1])
            std_z = np.std(positions[:, 2])
            
            # Total std dev (simple sum)
            total_std = std_x + std_y + std_z
            
            # Check if stable
            is_currently_stable = total_std < self.vio_std_threshold
            
            if is_currently_stable:
                self.vio_stable_count += 1
            else:
                self.vio_stable_count = 0
            
            # Require 10 consecutive stable readings
            was_stable = self.vio_stable
            self.vio_stable = self.vio_stable_count >= 10
            
            # Log when stability changes
            if not was_stable and self.vio_stable:
                self.get_logger().info(
                    f"✓ VIO stabilized! Position std: {total_std*1000:.2f}mm "
                    f"(x:{std_x*1000:.2f} y:{std_y*1000:.2f} z:{std_z*1000:.2f})"
                )
            elif was_stable and not self.vio_stable:
                self.get_logger().warn(
                    f"⚠ VIO unstable. Position std: {total_std*1000:.2f}mm "
                    f"(threshold: {self.vio_std_threshold*1000:.1f}mm)"
                )

    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.aruco_params
            )

            output_image = cv_image.copy()
            if ids is not None and len(ids) > 0:
                if self.show_markers:
                    cv2.aruco.drawDetectedMarkers(output_image, corners, ids)

                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners,
                    self.marker_size,
                    self.camera_matrix,
                    self.dist_coeffs
                )

                for i, marker_id in enumerate(ids):
                    if self.show_axes:
                        cv2.drawFrameAxes(
                            output_image,
                            self.camera_matrix,
                            self.dist_coeffs,
                            rvecs[i],
                            tvecs[i],
                            self.marker_size * 0.5
                        )

                    distance = np.sqrt(
                        tvecs[i][0][2]**2 +
                        tvecs[i][0][0]**2 +
                        tvecs[i][0][1]**2
                    )

                    corner = corners[i][0]
                    top_right = corner[0].astype(int)
                    bottom_right = corner[2].astype(int)

                    cv2.putText(
                        output_image,
                        f"id: {marker_id[0]} Dist: {distance:.2f}m",
                        tuple(top_right),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )

                    cv2.putText(
                        output_image,
                        f"x:{tvecs[i][0][0]:.2f} y:{tvecs[i][0][1]:.2f} z:{tvecs[i][0][2]:.2f}",
                        tuple(bottom_right),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )

                    if i == 0:
                        self.publish_marker_pose(rvecs[i], tvecs[i], msg.header)
            else:
                self.current_marker_pose = None

            if self.show_rejected and len(rejected) > 0:
                cv2.aruco.drawDetectedMarkers(
                    output_image,
                    rejected,
                    None,
                    (100, 0, 200)
                )

            self.image_pub.publish(
                self.bridge.cv2_to_imgmsg(output_image, encoding='bgr8')
            )

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def publish_marker_pose(self, rvec, tvec, header):
        """Publish the pose of a detected marker"""
        pose_msg = PoseStamped()
        pose_msg.header = header

        pose_msg.pose.position.x = float(tvec[0][0])
        pose_msg.pose.position.y = float(tvec[0][1])
        pose_msg.pose.position.z = float(tvec[0][2])

        rot_matrix = np.zeros(shape=(3,3))
        cv2.Rodrigues(rvec, rot_matrix)
        
        trace = rot_matrix[0][0] + rot_matrix[1][1] + rot_matrix[2][2]
        
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2
            qw = 0.25 * S
            qx = (rot_matrix[2][1] - rot_matrix[1][2]) / S
            qy = (rot_matrix[0][2] - rot_matrix[2][0]) / S
            qz = (rot_matrix[1][0] - rot_matrix[0][1]) / S
        else:
            if rot_matrix[0][0] > rot_matrix[1][1] and rot_matrix[0][0] > rot_matrix[2][2]:
                S = np.sqrt(1.0 + rot_matrix[0][0] - rot_matrix[1][1] - rot_matrix[2][2]) * 2
                qw = (rot_matrix[2][1] - rot_matrix[1][2]) / S
                qx = 0.25 * S
                qy = (rot_matrix[0][1] + rot_matrix[1][0]) / S
                qz = (rot_matrix[0][2] + rot_matrix[2][0]) / S
            elif rot_matrix[1][1] > rot_matrix[2][2]:
                S = np.sqrt(1.0 + rot_matrix[1][1] - rot_matrix[0][0] - rot_matrix[2][2]) * 2
                qw = (rot_matrix[0][2] - rot_matrix[2][0]) / S
                qx = (rot_matrix[0][1] + rot_matrix[1][0]) / S
                qy = 0.25 * S
                qz = (rot_matrix[1][2] + rot_matrix[2][1]) / S
            else:
                S = np.sqrt(1.0 + rot_matrix[2][2] - rot_matrix[0][0] - rot_matrix[1][1]) * 2
                qw = (rot_matrix[1][0] - rot_matrix[0][1]) / S
                qx = (rot_matrix[0][2] + rot_matrix[2][0]) / S
                qy = (rot_matrix[1][2] + rot_matrix[2][1]) / S
                qz = 0.25 * S

        norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        pose_msg.pose.orientation.x = float(qx)
        pose_msg.pose.orientation.y = float(qy)
        pose_msg.pose.orientation.z = float(qz)
        pose_msg.pose.orientation.w = float(qw)

        self.current_marker_pose = (tvec, rvec)
        self.pose_pub.publish(pose_msg)
        self.publish_landing_target(tvec, rvec, header)

    def publish_landing_target(self, tvec, rvec, header):
        """Publish landing target message for precision landing"""
        msg = LandingTarget()
        msg.header = header
        msg.target_num = 0
        msg.frame = 12  # BODY_FRD
        
        distance = float(np.linalg.norm(tvec[0]))
        
        x = float(tvec[0][2])
        y = float(tvec[0][0])
        z = float(tvec[0][1])
        
        angle_x = float(np.arctan2(y, x))
        angle_y = float(np.arctan2(z, x))
        msg.angle = [angle_x, angle_y]
        msg.distance = distance
        
        size = float(np.arctan2(self.marker_size, distance))
        msg.size = [size, size]
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        
        msg.type = 2  # VISION_FIDUCIAL
        
        self.landing_target_pub.publish(msg)
        
        # Initialize local reference frame if auto-set is enabled
        if self.auto_set_home and not self.initialization_in_progress:
            current_time = time.time()
            if current_time - self.last_init_attempt >= self.home_retry_delay:
                if not self.home_set and self.ekf_origin_attempts < self.home_max_retries:
                    self.initialize_ekf_and_home()

    def initialize_ekf_and_home(self):
        """Initialize EKF origin first, then set HOME"""
        if self.initialization_in_progress:
            return
            
        if self.vehicle_state is None or not self.vehicle_state.connected:
            self.get_logger().warn("Vehicle not connected via MAVROS")
            return
            
        # Ensure we have recent vision pose data
        current_time = time.time()
        if current_time - self.last_pose_time > 1.0:
            self.get_logger().warn("No recent vision pose data")
            return
        
        # Check VIO stability
        if not self.vio_stable:
            self.get_logger().warn(
                "VIO not stable yet. Keep drone stationary and ensure camera sees textured environment."
            )
            return
            
        if not self.command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("MAVROS command service not available")
            return
        
        self.initialization_in_progress = True
        self.last_init_attempt = time.time()
        
        # Step 1: Set EKF origin
        self.set_ekf_origin()

    def set_ekf_origin(self):
        """Set EKF origin using current VIO position"""
        try:
            self.ekf_origin_attempts += 1
            
            # Check if we're armed or in flight
            if self.vehicle_state.armed:
                self.get_logger().warn("Cannot set home while armed. Please disarm first.")
                self.initialization_in_progress = False
                return
                
            # Check if we're already in GUIDED
            if self.vehicle_state.mode != 'GUIDED':
                self.get_logger().info("Setting GUIDED mode to establish local reference...")
                # First switch to GUIDED mode
                req = SetMode.Request()
                req.custom_mode = 'GUIDED'
                if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().error("Set mode service not available")
                    self.initialization_in_progress = False
                    return
                mode_future = self.set_mode_client.call_async(req)
                mode_future.add_done_callback(self.guided_mode_callback)
                return  # Wait for mode callback to continue
                
            self.get_logger().info(
                f"📍 Setting EKF origin at marker location (attempt {self.ekf_origin_attempts}/{self.home_max_retries})..."
            )
            
            # For VIO-only operation, we directly set HOME which will also establish the local frame
            req = CommandLong.Request()
            req.command = 179  # MAV_CMD_DO_SET_HOME
            req.param1 = 1.0   # Use current position
            req.param2 = 0.0   # Unused
            req.param3 = 0.0   # Unused
            req.param4 = 0.0   # Unused
            req.param5 = 0.0   # Latitude (not used with param1=1)
            req.param6 = 0.0   # Longitude (not used with param1=1)
            req.param7 = 0.0   # Altitude (not used with param1=1)
            
            # Set home position which will also establish local reference frame
            future = self.command_client.call_async(req)
            future.add_done_callback(self.ekf_origin_callback)
        except Exception as e:
            self.get_logger().error(f"Error setting EKF origin: {str(e)}")
            self.initialization_in_progress = False

    def guided_mode_callback(self, future):
        """Callback for GUIDED mode set command"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("✓ GUIDED mode set successfully")
                # Now we can set home position
                self.set_ekf_origin()
            else:
                self.get_logger().error(f"Failed to set GUIDED mode: {response.result_str}")
                self.initialization_in_progress = False
        except Exception as e:
            self.get_logger().error(f"Error in GUIDED mode callback: {str(e)}")
            self.initialization_in_progress = False
            
    def ekf_origin_callback(self, future):
        """Callback for origin set command"""
        try:
            response = future.result()
            if response.success:
                self.ekf_origin_set = True
                self.home_set = True  # Setting home automatically sets the origin in VIO mode
                self.initialization_in_progress = False
                self.get_logger().info("✓ Local reference frame established successfully")
                self.get_logger().info("✓ HOME position set at current location")
                self.get_logger().info("System ready for flight operations")
            else:
                result_code = int(response.result)
                self.get_logger().warn(
                    f"Failed to set local reference frame (attempt {self.ekf_origin_attempts}/{self.home_max_retries}): "
                    f"{self._get_error_message(result_code)} (code={result_code})"
                )
                self.initialization_in_progress = False
        except Exception as e:
            self.get_logger().error(f"Error in EKF origin callback: {str(e)}")
            self.initialization_in_progress = False

    def set_home(self):
        """Set HOME position using current local position"""
        if self.home_set:
            return
            
        if not self.ekf_origin_set:
            self.get_logger().warn("Cannot set HOME: EKF origin not set yet")
            self.initialization_in_progress = False
            return
        
        try:
            self.home_set_attempts += 1
            
            self.get_logger().info(
                f"🏠 Setting HOME position (attempt {self.home_set_attempts}/{self.home_max_retries})..."
            )
            
            # MAV_CMD_DO_SET_HOME (command 179)
            req = CommandLong.Request()
            req.command = 179  # MAV_CMD_DO_SET_HOME
            req.param1 = 1.0   # Use current position
            req.param2 = 0.0
            req.param3 = 0.0
            req.param4 = 0.0
            req.param5 = 0.0
            req.param6 = 0.0
            req.param7 = 0.0
            
            future = self.command_client.call_async(req)
            future.add_done_callback(self.home_set_callback_wrapper)
        except Exception as e:
            self.get_logger().error(f"Error setting HOME: {str(e)}")
            self.initialization_in_progress = False

    def home_set_callback_wrapper(self, future):
        """Callback for HOME position set command"""
        try:
            response = future.result()
            if response.success:
                self.home_set = True
                self.initialization_in_progress = False
                self.get_logger().info("✓✓✓ HOME position set successfully! ✓✓✓")
                self.get_logger().info("System ready for flight operations")
            else:
                result_code = int(response.result)
                error_msg = self._get_error_message(result_code)
                    
                self.get_logger().warn(
                    f"Failed to set HOME (attempt {self.home_set_attempts}/{self.home_max_retries}): "
                    f"{error_msg} (code={result_code})"
                )
                
                self.initialization_in_progress = False
                
                if self.home_set_attempts >= self.home_max_retries:
                    self.get_logger().error(
                        "Maximum HOME set retries reached. Use ~/set_home service to retry."
                    )
        except Exception as e:
            self.get_logger().error(f"Error in HOME callback: {str(e)}")
            self.initialization_in_progress = False
    
    def _get_error_message(self, result_code):
        """Get human-readable error message for result code"""
        error_messages = {
            0: "Command accepted",
            1: "Command temporarily rejected",
            2: "Command denied",
            3: "Command unsupported",
            4: "Command failed",
            5: "Command in progress",
            6: "Command cancelled"
        }
        return error_messages.get(result_code, f"Unknown error code {result_code}")

    def set_home_callback(self, request, response):
        """Service callback to manually trigger initialization"""
        if self.current_marker_pose is not None:
            self.ekf_origin_attempts = 0
            self.home_set_attempts = 0
            self.initialization_in_progress = False
            self.ekf_origin_set = False
            self.home_set = False
            
            self.get_logger().info("Manual initialization triggered")
            self.initialize_ekf_and_home()
            
            response.success = True
            response.message = "Initialization sequence started"
        else:
            response.success = False
            response.message = "No marker detected"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()