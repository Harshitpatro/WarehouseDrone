#!/usr/bin/env python3
"""
RGBD Images Synchronizer Node
Subscribes to multiple RGBD image topics and publishes them as a single RGBDImages message.
This is a workaround for RTAB-Map installations without RTABMAP_SYNC_MULTI_RGBD support.
"""

import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rtabmap_msgs.msg import RGBDImage, RGBDImages


class RGBDImagesSync(Node):
    def __init__(self):
        super().__init__('rgbd_images_sync')
        
        # Declare parameters
        self.declare_parameter('approx_sync', True)
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('slop', 0.04)  # 40ms tolerance for approximate sync
        
        # Get parameters
        approx_sync = self.get_parameter('approx_sync').get_parameter_value().bool_value
        queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
        slop = self.get_parameter('slop').get_parameter_value().double_value
        
        # Create subscribers for two RGBD images
        self.rgbd_sub0 = Subscriber(self, RGBDImage, 'rgbd_image0')
        self.rgbd_sub1 = Subscriber(self, RGBDImage, 'rgbd_image1')
        
        # Create publisher for synchronized RGBDImages
        self.rgbd_images_pub = self.create_publisher(RGBDImages, 'rgbd_images', 10)
        
        # Create approximate time synchronizer
        if approx_sync:
            self.sync = ApproximateTimeSynchronizer(
                [self.rgbd_sub0, self.rgbd_sub1],
                queue_size,
                slop
            )
            self.get_logger().info(f'Using ApproximateTimeSynchronizer with slop={slop}s')
        else:
            # For exact sync, you would use TimeSynchronizer
            from message_filters import TimeSynchronizer
            self.sync = TimeSynchronizer(
                [self.rgbd_sub0, self.rgbd_sub1],
                queue_size
            )
            self.get_logger().info('Using TimeSynchronizer (exact sync)')
        
        self.sync.registerCallback(self.sync_callback)
        
        self.get_logger().info('RGBD Images Synchronizer started')
        self.get_logger().info('Subscribing to:')
        self.get_logger().info('  - rgbd_image0')
        self.get_logger().info('  - rgbd_image1')
        self.get_logger().info('Publishing to: rgbd_images')
        
        self.msg_count = 0

    def sync_callback(self, rgbd0, rgbd1):
        """Callback when both RGBD images are synchronized"""
        # Create RGBDImages message
        rgbd_images = RGBDImages()
        
        # Use the header from the first image
        rgbd_images.header = rgbd0.header
        
        # Add both RGBD images to the array
        rgbd_images.rgbd_images.append(rgbd0)
        rgbd_images.rgbd_images.append(rgbd1)
        
        # Publish
        self.rgbd_images_pub.publish(rgbd_images)
        
        # Log periodically
        self.msg_count += 1
        if self.msg_count % 30 == 0:  # Every 30 messages (roughly 1-2 seconds)
            self.get_logger().info(
                f'Published {self.msg_count} synchronized RGBD image pairs'
            )


def main(args=None):
    rclpy.init(args=args)
    node = RGBDImagesSync()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()