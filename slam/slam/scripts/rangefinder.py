#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Range
from cv_bridge import CvBridge
import numpy as np

class RangefinderNode(Node):
    def __init__(self):
        super().__init__('rangefinder_node')
        
        self.bridge = CvBridge()
        
        # Subscribe to downward camera depth
        self.depth_sub = self.create_subscription(
            Image,
            '/camera_down/camera_down/depth/image_rect_raw',
            self.depth_callback,
            10
        )
        
        # Publish to MAVROS distance sensor (using Range message)
        self.range_pub = self.create_publisher(
            Range,
            '/mavros/rangefinder_sub',
            10
        )
        
        self.get_logger().info('Rangefinder node started')
        
    def depth_callback(self, msg):
        try:
            # Convert depth image to numpy array
            if msg.encoding == '16UC1':
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            elif msg.encoding == '32FC1':
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                depth_image = (depth_image * 1000).astype(np.uint16)  # Convert m to mm
            else:
                self.get_logger().warn(f'Unexpected encoding: {msg.encoding}')
                return
            
            # Get center region (40x40 pixels)
            h, w = depth_image.shape
            center_h, center_w = h // 2, w // 2
            roi_size = 20
            center_region = depth_image[
                center_h - roi_size:center_h + roi_size,
                center_w - roi_size:center_w + roi_size
            ]
            
            # Filter valid depths (300mm to 10000mm = 0.3m to 10m)
            valid_mask = (center_region > 100) & (center_region < 10000)
            
            if np.any(valid_mask):
                valid_depths = center_region[valid_mask]
                distance_mm = float(np.median(valid_depths))
                distance_m = distance_mm / 1000.0
                
                # Create Range message
                range_msg = Range()
                range_msg.header.stamp = self.get_clock().now().to_msg()
                range_msg.header.frame_id = 'camera_down_link'
                range_msg.radiation_type = Range.INFRARED
                range_msg.field_of_view = 0.7854  # 45 degrees
                range_msg.min_range = 0.3
                range_msg.max_range = 10.0
                range_msg.range = distance_m
                
                self.range_pub.publish(range_msg)
                
                # Log occasionally (every ~1 second at 30fps)
                if self.get_clock().now().nanoseconds % 1000000000 < 33333333:
                    self.get_logger().info(f'Ground distance: {distance_m:.2f}m')
                    
        except Exception as e:
            self.get_logger().error(f'Error processing depth: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = RangefinderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()