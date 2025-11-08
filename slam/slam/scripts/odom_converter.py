#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from rcl_interfaces.msg import ParameterDescriptor
import math

class OdomToVisionPose(Node):
    def __init__(self):
        super().__init__('odom_to_vision_pose')
        
        # Declare parameters with descriptions
        self.declare_parameter('use_enu_to_ned', False, 
            ParameterDescriptor(description='Convert from ENU to NED coordinate frame'))
        self.declare_parameter('send_twist', True,
            ParameterDescriptor(description='Publish velocity/twist messages'))
        self.declare_parameter('confidence_update_hz', 1.0,
            ParameterDescriptor(description='Frequency to send confidence status messages (Hz)'))
        
        self.use_enu_to_ned = self.get_parameter('use_enu_to_ned').value
        self.send_twist = self.get_parameter('send_twist').value
        confidence_hz = self.get_parameter('confidence_update_hz').value
        
        # Quality assessment thresholds based on covariance values
        # Adjusted for RTAB-Map output (more realistic thresholds)
        self.EXCELLENT_COV_THRESHOLD = 0.000005  # Very low covariance = high confidence
        self.GOOD_COV_THRESHOLD = 0.00002       # Low covariance = good confidence
        self.FAIR_COV_THRESHOLD = 0.0001        # Medium covariance = fair confidence
        self.POOR_COV_THRESHOLD = 0.001         # High covariance = poor confidence
        
        # For tracking confidence messages
        self.last_confidence_time = self.get_clock().now()
        self.confidence_interval = 1.0 / confidence_hz if confidence_hz > 0 else 1.0
        self.current_confidence_level = 100.0  # Default high confidence
        self.current_quality_text = "High"
        
        # For velocity-based quality assessment
        self.prev_position = None
        self.prev_time = None
        
        # Statistics
        self.msg_count = 0
        self.pose_msg_count = 0
        self.last_print_time = self.get_clock().now()
        
        # Subscribe to odometry (from RTAB-Map or any VIO)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publish to MAVROS vision pose
        self.vision_pose_pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10
        )
        
        # Optionally publish vision speed/twist
        if self.send_twist:
            self.vision_speed_pub = self.create_publisher(
                TwistStamped,
                '/mavros/vision_speed/speed_twist',
                10
            )
        
        # Create timer for periodic confidence updates
        self.confidence_timer = self.create_timer(
            self.confidence_interval,
            self.send_confidence_status
        )
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('VIO to MAVROS Bridge Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'ENU to NED conversion: {self.use_enu_to_ned}')
        self.get_logger().info(f'Send twist/velocity: {self.send_twist}')
        self.get_logger().info(f'Confidence update rate: {confidence_hz:.1f} Hz')
        self.get_logger().info('Subscribed to: /odom')
        self.get_logger().info('Publishing to: /mavros/vision_pose/pose')
        if self.send_twist:
            self.get_logger().info('Publishing to: /mavros/vision_speed/speed_twist')
        self.get_logger().info('=' * 60)
    
    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions q1 * q2"""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        
        return [
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2
        ]
    
    def enu_to_ned_quaternion(self, qx, qy, qz, qw):
        """
        Convert quaternion from ENU to NED frame
        ENU: X=East, Y=North, Z=Up
        NED: X=North, Y=East, Z=Down
        """
        sqrt2_2 = math.sqrt(2) / 2
        
        # 90° rotation around Z (yaw)
        q_z90 = [0, 0, sqrt2_2, sqrt2_2]
        
        # 180° rotation around X (pitch) 
        q_x180 = [1, 0, 0, 0]
        
        # Input quaternion
        q_in = [qx, qy, qz, qw]
        
        # Combined rotation
        q_temp = self.quaternion_multiply(q_z90, q_x180)
        q_out = self.quaternion_multiply(q_temp, q_in)
        
        return q_out
    
    def assess_tracking_quality(self, msg):
        """
        Assess tracking quality based on RTAB-Map specific covariance format
        RTAB-Map stores actual covariance in diagonal, NOT quality score
        Returns: (confidence_level, quality_text)
        """
        if len(msg.pose.covariance) != 36:
            return 50.0, "Unknown"
        
        # Extract ACTUAL covariance values (not RTAB-Map's quality score)
        # Position covariance (diagonal elements)
        cov_x = msg.pose.covariance[0]   # [0,0] - X position variance
        cov_y = msg.pose.covariance[7]   # [1,1] - Y position variance
        cov_z = msg.pose.covariance[14]  # [2,2] - Z position variance
        
        # Calculate average position standard deviation (sqrt of variance)
        avg_position_std = ((cov_x + cov_y + cov_z) / 3.0) ** 0.5
        
        # Extract orientation covariance
        cov_roll = msg.pose.covariance[21]   # [3,3]
        cov_pitch = msg.pose.covariance[28]  # [4,4]
        cov_yaw = msg.pose.covariance[35]    # [5,5]
        
        # Calculate average orientation standard deviation
        avg_orientation_std = ((cov_roll + cov_pitch + cov_yaw) / 3.0) ** 0.5
        
        # Quality thresholds based on standard deviation (in meters and radians)
        # These match typical RTAB-Map output: std dev ~0.001-0.003m is excellent
        EXCELLENT_STD_POS = 0.002    # < 2mm std dev
        GOOD_STD_POS = 0.005         # < 5mm std dev
        FAIR_STD_POS = 0.010         # < 10mm std dev
        POOR_STD_POS = 0.020         # < 20mm std dev
        
        EXCELLENT_STD_ORI = 0.020    # < ~1.1 degrees
        GOOD_STD_ORI = 0.035         # < ~2.0 degrees
        FAIR_STD_ORI = 0.070         # < ~4.0 degrees
        POOR_STD_ORI = 0.140         # < ~8.0 degrees
        
        # Determine quality based on BOTH position and orientation
        if avg_position_std < EXCELLENT_STD_POS and avg_orientation_std < EXCELLENT_STD_ORI:
            confidence = 100.0
            quality = "High"
        elif avg_position_std < GOOD_STD_POS and avg_orientation_std < GOOD_STD_ORI:
            confidence = 75.0
            quality = "Medium"
        elif avg_position_std < FAIR_STD_POS and avg_orientation_std < FAIR_STD_ORI:
            confidence = 50.0
            quality = "Low"
        elif avg_position_std < POOR_STD_POS and avg_orientation_std < POOR_STD_ORI:
            confidence = 25.0
            quality = "Poor"
        else:
            confidence = 0.0
            quality = "Failed"
        
        return confidence, quality
    
    def send_confidence_status(self):
        """
        Periodic callback to send confidence status message
        This mimics the T265 script's update_tracking_confidence_to_gcs function
        """
        if self.current_confidence_level is not None:
            status_msg = f'VIO Tracking confidence: {self.current_quality_text} ({self.current_confidence_level:.0f}%)'
            self.get_logger().info(status_msg)
            
            # Also log pose message rate
            if self.pose_msg_count > 0:
                hz = self.pose_msg_count / self.confidence_interval
                self.get_logger().info(f'Vision pose rate: {hz:.1f} Hz')
                self.pose_msg_count = 0
    
    def odom_callback(self, msg):
        """
        Main callback to process odometry and publish vision pose
        """
        try:
            current_time = self.get_clock().now()
            
            # Assess tracking quality from covariance
            confidence, quality_text = self.assess_tracking_quality(msg)
            self.current_confidence_level = confidence
            self.current_quality_text = quality_text
            
            # Create PoseStamped message for position
            pose_msg = PoseStamped()
            pose_msg.header.stamp = msg.header.stamp
            pose_msg.header.frame_id = "odom"
            
            if self.use_enu_to_ned:
                # ENU to NED transformation
                pose_msg.pose.position.x = msg.pose.pose.position.y   # North
                pose_msg.pose.position.y = msg.pose.pose.position.x   # East
                pose_msg.pose.position.z = -msg.pose.pose.position.z  # Down
                
                # Quaternion transformation
                qx = msg.pose.pose.orientation.x
                qy = msg.pose.pose.orientation.y
                qz = msg.pose.pose.orientation.z
                qw = msg.pose.pose.orientation.w
                
                q_ned = self.enu_to_ned_quaternion(qx, qy, qz, qw)
                
                pose_msg.pose.orientation.x = q_ned[0]
                pose_msg.pose.orientation.y = q_ned[1]
                pose_msg.pose.orientation.z = q_ned[2]
                pose_msg.pose.orientation.w = q_ned[3]
            else:
                # Direct copy
                pose_msg.pose.position = msg.pose.pose.position
                pose_msg.pose.orientation = msg.pose.pose.orientation
            
            # Publish vision pose
            self.vision_pose_pub.publish(pose_msg)
            self.pose_msg_count += 1
            
            # Optionally publish twist/velocity
            if self.send_twist:
                twist_msg = TwistStamped()
                twist_msg.header.stamp = pose_msg.header.stamp
                twist_msg.header.frame_id = "base_link"
                
                if self.use_enu_to_ned:
                    # Transform velocity to NED
                    twist_msg.twist.linear.x = msg.twist.twist.linear.y
                    twist_msg.twist.linear.y = msg.twist.twist.linear.x
                    twist_msg.twist.linear.z = -msg.twist.twist.linear.z
                    
                    twist_msg.twist.angular.x = msg.twist.twist.angular.y
                    twist_msg.twist.angular.y = msg.twist.twist.angular.x
                    twist_msg.twist.angular.z = -msg.twist.twist.angular.z
                else:
                    twist_msg.twist = msg.twist.twist
                
                self.vision_speed_pub.publish(twist_msg)
            
            # Detailed statistics every 5 seconds
            self.msg_count += 1
            elapsed = (current_time - self.last_print_time).nanoseconds / 1e9
            
            if elapsed >= 5.0:
                hz = self.msg_count / elapsed
                
                # Extract covariance info for logging
                cov_x = msg.pose.covariance[0] if len(msg.pose.covariance) == 36 else 0
                cov_y = msg.pose.covariance[7] if len(msg.pose.covariance) == 36 else 0
                cov_z = msg.pose.covariance[14] if len(msg.pose.covariance) == 36 else 0
                avg_cov = (cov_x + cov_y + cov_z) / 3.0
                
                self.get_logger().info('─' * 60)
                self.get_logger().info(f'Rate: {hz:.1f} Hz | Quality: {quality_text} ({confidence:.0f}%)')
                self.get_logger().info(
                    f'Position [m]: [{pose_msg.pose.position.x:.4f}, '
                    f'{pose_msg.pose.position.y:.4f}, '
                    f'{pose_msg.pose.position.z:.4f}]'
                )
                self.get_logger().info(f'Avg Position Covariance: {avg_cov:.8f}')
                self.get_logger().info('─' * 60)
                
                self.msg_count = 0
                self.last_print_time = current_time
                
        except Exception as e:
            self.get_logger().error(f'Error in odom_callback: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = OdomToVisionPose()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()