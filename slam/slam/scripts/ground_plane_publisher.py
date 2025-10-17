#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Float32
from geometry_msgs.msg import TransformStamped
import numpy as np
from tf2_ros import TransformBroadcaster
import struct

class GroundPlanePublisher(Node):
    """
    Publishes a synthetic ground plane point cloud based on fused height.
    This helps RTAB-Map maintain a continuous ground surface in the occupancy grid.
    """
    
    def __init__(self):
        super().__init__('ground_plane_publisher')
        
        # Parameters
        self.declare_parameter('fused_height_topic', '/fused_height')
        self.declare_parameter('ground_cloud_topic', '/ground_plane_cloud')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('grid_size', 5.0)  # Size of ground plane grid (meters)
        self.declare_parameter('grid_resolution', 0.2)  # Point spacing (meters)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ground_frame', 'ground_plane')
        self.declare_parameter('min_height', 0.5)  # Min height to publish ground plane
        self.declare_parameter('max_height', 15.0)  # Max height to publish ground plane
        
        # Get parameters
        self.fused_height_topic = self.get_parameter('fused_height_topic').value
        self.ground_cloud_topic = self.get_parameter('ground_cloud_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.grid_size = self.get_parameter('grid_size').value
        self.grid_resolution = self.get_parameter('grid_resolution').value
        self.base_frame = self.get_parameter('base_frame').value
        self.ground_frame = self.get_parameter('ground_frame').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        
        # State
        self.current_height = None
        
        # Generate ground plane grid
        self.generate_ground_grid()
        
        # Subscriber
        self.height_sub = self.create_subscription(
            Float32, self.fused_height_topic, self.height_callback, 10)
        
        # Publisher
        self.cloud_pub = self.create_publisher(
            PointCloud2, self.ground_cloud_topic, 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_callback)
        
        self.get_logger().info('Ground Plane Publisher initialized')
    
    def generate_ground_grid(self):
        """Generate a grid of points for the ground plane"""
        half_size = self.grid_size / 2.0
        x = np.arange(-half_size, half_size, self.grid_resolution)
        y = np.arange(-half_size, half_size, self.grid_resolution)
        
        # Create meshgrid
        self.grid_x, self.grid_y = np.meshgrid(x, y)
        self.grid_x = self.grid_x.flatten()
        self.grid_y = self.grid_y.flatten()
        
        self.get_logger().info(f'Generated ground grid with {len(self.grid_x)} points')
    
    def height_callback(self, msg):
        """Receive fused height estimate"""
        self.current_height = msg.data
    
    def publish_callback(self):
        """Publish ground plane point cloud"""
        if self.current_height is None:
            return
        
        # Only publish if within reasonable height range
        if not (self.min_height <= self.current_height <= self.max_height):
            return
        
        # Create ground plane at current height below drone
        ground_z = -self.current_height
        
        # Create point cloud
        points = []
        for i in range(len(self.grid_x)):
            x = float(self.grid_x[i])
            y = float(self.grid_y[i])
            z = float(ground_z)
            
            # Pack as xyz (12 bytes per point)
            points.append(struct.pack('fff', x, y, z))
        
        # Create PointCloud2 message
        cloud_msg = PointCloud2()
        cloud_msg.header = Header()
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.header.frame_id = self.base_frame
        
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.is_dense = True
        cloud_msg.is_bigendian = False
        
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        cloud_msg.point_step = 12  # 3 floats * 4 bytes
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.data = b''.join(points)
        
        # Publish
        self.cloud_pub.publish(cloud_msg)
        
        # Also publish TF for visualization
        self.publish_ground_tf(ground_z)
    
    def publish_ground_tf(self, ground_z):
        """Publish transform to ground plane for visualization"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.ground_frame
        
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = ground_z
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = GroundPlanePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()