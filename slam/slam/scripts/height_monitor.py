#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import FluidPressure, Image
from cv_bridge import CvBridge
import numpy as np

class HeightMonitor(Node):
    """Monitor and display height fusion performance"""
    
    def __init__(self):
        super().__init__('height_monitor')
        
        self.fused_height = None
        self.cam_weight = None
        self.baro_weight = None
        self.raw_pressure = None
        self.depth_received = False
        
        # Subscribers
        self.fused_sub = self.create_subscription(
            Float32, '/fused_height', self.fused_callback, 10)
        self.cam_weight_sub = self.create_subscription(
            Float32, '/height_fusion/camera_weight', self.cam_weight_callback, 10)
        self.baro_weight_sub = self.create_subscription(
            Float32, '/height_fusion/baro_weight', self.baro_weight_callback, 10)
        self.pressure_sub = self.create_subscription(
            FluidPressure, '/mavros/imu/static_pressure', self.pressure_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera_down/camera_down/aligned_depth_to_color/image_raw', 
            self.depth_callback, 10)
        
        # Timer for display
        self.timer = self.create_timer(0.5, self.display_callback)
        
        self.get_logger().info('Height Fusion Monitor started')
        self.get_logger().info('=' * 80)
    
    def fused_callback(self, msg):
        self.fused_height = msg.data
    
    def cam_weight_callback(self, msg):
        self.cam_weight = msg.data
    
    def baro_weight_callback(self, msg):
        self.baro_weight = msg.data
    
    def pressure_callback(self, msg):
        self.raw_pressure = msg.fluid_pressure
    
    def depth_callback(self, msg):
        self.depth_received = True
    
    def get_status_bar(self, value, max_value, width=30):
        """Create a visual progress bar"""
        filled = int((value / max_value) * width)
        bar = '█' * filled + '░' * (width - filled)
        return bar
    
    def display_callback(self):
        if self.fused_height is None:
            self.get_logger().info('Waiting for fused height data...', throttle_duration_sec=2.0)
            return
        
        # Clear screen (optional)
        print('\033[2J\033[H', end='')
        
        print('=' * 80)
        print('                    HEIGHT FUSION MONITOR')
        print('=' * 80)
        print()
        
        # Fused height
        print(f'📏 FUSED HEIGHT: {self.fused_height:.2f} m')
        print()
        
        # Sensor fusion weights
        if self.cam_weight is not None and self.baro_weight is not None:
            print('🎚️  SENSOR FUSION WEIGHTS:')
            cam_bar = self.get_status_bar(self.cam_weight, 1.0)
            baro_bar = self.get_status_bar(self.baro_weight, 1.0)
            print(f'   Camera:     [{cam_bar}] {self.cam_weight*100:5.1f}%')
            print(f'   Barometer:  [{baro_bar}] {self.baro_weight*100:5.1f}%')
            print()
            
            # Determine operating mode
            if self.cam_weight > 0.7:
                mode = '🟢 CAMERA DOMINANT (Low altitude)'
                mode_color = '\033[92m'  # Green
            elif self.cam_weight > 0.3:
                mode = '🟡 BLENDED MODE (Medium altitude)'
                mode_color = '\033[93m'  # Yellow
            elif self.cam_weight > 0.0:
                mode = '🟠 BAROMETER DOMINANT (High altitude)'
                mode_color = '\033[91m'  # Orange/Red
            else:
                mode = '🔴 BAROMETER ONLY (Camera out of range)'
                mode_color = '\033[91m'  # Red
            
            print(f'{mode_color}{mode}\033[0m')
            print()
        
        # Height zones visualization
        print('📊 HEIGHT ZONES:')
        height = self.fused_height
        
        zones = [
            (0, 1, '  0-1m:  Camera 90-100% '),
            (1, 3, '  1-3m:  Camera 60-90%  '),
            (3, 6, '  3-6m:  Camera 20-60%  '),
            (6, 100, ' >6m:   Barometer 100% ')
        ]
        
        for start, end, label in zones:
            if start <= height < end:
                marker = '→ '
                color = '\033[92m'  # Green highlight
            else:
                marker = '  '
                color = ''
            
            print(f'{color}{marker}{label}\033[0m')
        
        print()
        
        # Additional info
        if self.raw_pressure is not None:
            print(f'🌡️  Raw Pressure: {self.raw_pressure:.2f} Pa')
        
        if self.depth_received:
            print('📷 Downward camera: Active')
        else:
            print('📷 Downward camera: No data')
        
        print()
        print('=' * 80)
        print('Press Ctrl+C to exit')
        print()


def main(args=None):
    rclpy.init(args=args)
    node = HeightMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down monitor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()