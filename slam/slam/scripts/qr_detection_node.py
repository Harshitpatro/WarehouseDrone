#!/usr/bin/env python3
"""
Standalone QR Code Detection and RTAB-Map Labeling Node with Odometry Logging
Run this alongside your existing RTAB-Map launch file

Usage:
  ros2 run qr_rtabmap_labeling qr_standalone_node --ros-args -p model_path:=best.pt
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rtabmap_msgs.srv import SetLabel, AddLink
from rtabmap_msgs.msg import UserData
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
import pandas as pd
import os
from datetime import datetime
from pyzbar.pyzbar import decode
from ultralytics import YOLO
import threading

class StandaloneQRNode(Node):
    def __init__(self):
        super().__init__('qr_standalone_labeling_node')
        
        # Parameters
        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('csv_file', 'qr_labels.csv')
        self.declare_parameter('images_dir', 'qr_images')
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('odom_topic', '/rtabmap/odom')
        self.declare_parameter('min_detection_interval', 3.0)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('enable_visualization', True)
        
        self.model_path = self.get_parameter('model_path').value
        self.csv_file = self.get_parameter('csv_file').value
        self.images_dir = self.get_parameter('images_dir').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.min_interval = self.get_parameter('min_detection_interval').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.enable_viz = self.get_parameter('enable_visualization').value
        
        # Create images directory
        os.makedirs(self.images_dir, exist_ok=True)
        
        # Initialize GPU and YOLO model
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f'🚀 Using device: {self.device}')
        
        try:
            self.model = YOLO(self.model_path)
            self.model.to(self.device)
            self.get_logger().info(f'✅ Model loaded: {self.model_path}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load model: {str(e)}')
            raise
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize tracking data structures
        self.last_detection = {}  # Track last detection time for each QR code
        self.labeled_qr_codes = set()  # Track QR codes that have already been labeled
        self.lock = threading.Lock()  # Lock for thread-safe operations
        
        # Odometry data
        self.current_odom = None
        self.odom_lock = threading.Lock()
        
        # Initialize CSV (must be after labeled_qr_codes initialization)
        self.init_csv()
        
        # ROS2 Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        
        # ROS2 Publishers
        self.qr_data_pub = self.create_publisher(String, '/qr_detected', 10)
        if self.enable_viz:
            self.debug_image_pub = self.create_publisher(Image, '/qr_debug_image', 10)
        
        # RTAB-Map service client
        self.label_client = self.create_client(SetLabel, '/rtabmap/set_label')
        
        # Wait for RTAB-Map service
        self.check_rtabmap_service()
        
        self.get_logger().info('━' * 60)
        self.get_logger().info('🔍 QR Code RTAB-Map Labeling Node Started')
        self.get_logger().info(f'📷 Camera topic: {self.camera_topic}')
        self.get_logger().info(f'🧭 Odometry topic: {self.odom_topic}')
        self.get_logger().info(f'💾 CSV file: {self.csv_file}')
        self.get_logger().info(f'📁 Images dir: {self.images_dir}')
        self.get_logger().info('━' * 60)
    
    def check_rtabmap_service(self):
        """Check if RTAB-Map service is available"""
        if self.label_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('✅ RTAB-Map add_label service found')
        else:
            self.get_logger().warn('⚠️  RTAB-Map service not available yet')
            self.get_logger().warn('    Labels will be queued until service is ready')
    
    def odom_callback(self, msg):
        """Store the latest odometry data"""
        with self.odom_lock:
            self.current_odom = msg
    
    def get_current_pose(self):
        """Get current position and orientation from odometry"""
        with self.odom_lock:
            if self.current_odom is None:
                return None
            
            pose = self.current_odom.pose.pose
            return {
                'pos_x': pose.position.x,
                'pos_y': pose.position.y,
                'pos_z': pose.position.z,
                'orient_x': pose.orientation.x,
                'orient_y': pose.orientation.y,
                'orient_z': pose.orientation.z,
                'orient_w': pose.orientation.w
            }
    
    def init_csv(self):
        """Initialize or load CSV file"""
        try:
            self.df = pd.read_csv(self.csv_file)
            self.get_logger().info(f'📂 Loaded existing CSV with {len(self.df)} entries')
            # Initialize labeled QR codes from existing CSV
            successful_labels = self.df[self.df['label_status'] == 'success']['qr_data'].unique()
            self.labeled_qr_codes.update(successful_labels)
        except (FileNotFoundError, pd.errors.EmptyDataError):
            self.df = pd.DataFrame(columns=[
                "timestamp", "qr_data", "image_path", "label_status", "confidence",
                "pos_x", "pos_y", "pos_z", 
                "orient_x", "orient_y", "orient_z", "orient_w"
            ])
            self.df.to_csv(self.csv_file, index=False)
            self.get_logger().info(f'📝 Created new CSV: {self.csv_file}')
    
    def should_process_qr(self, qr_data):
        """Check if enough time has passed since last detection"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        with self.lock:
            if qr_data in self.last_detection:
                time_diff = current_time - self.last_detection[qr_data]
                if time_diff < self.min_interval:
                    return False
            
            self.last_detection[qr_data] = current_time
        return True
    
    def add_label_to_rtabmap(self, label_text):
        """Add label to current RTAB-Map location"""
        if not self.label_client.service_is_ready():
            self.get_logger().warn('⚠️  RTAB-Map service not ready, skipping label')
            return False
        
        try:
            request = SetLabel.Request()
            request.node_label = label_text
            
            future = self.label_client.call_async(request)
            
            # Don't block - just fire and forget
            self.get_logger().info(f'📤 Sent label to RTAB-Map: "{label_text}"')
            return True
        except Exception as e:
            self.get_logger().error(f'❌ Failed to send label: {str(e)}')
            return False
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS Image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO detection
            results = self.model.predict(
                frame, 
                device=self.device, 
                verbose=False,
                conf=self.conf_threshold
            )
            
            # Process detections
            qr_detected = False
            for result in results:
                boxes = result.boxes
                if boxes is None or len(boxes) == 0:
                    continue
                    
                for box in boxes.xyxy:
                    x1, y1, x2, y2 = map(int, box[:4])
                    
                    # Ensure valid ROI
                    if x2 <= x1 or y2 <= y1:
                        continue
                    
                    roi = frame[y1:y2, x1:x2]
                    
                    # Decode QR from cropped region
                    decoded_objects = decode(roi)
                    for qr in decoded_objects:
                        try:
                            qr_data = qr.data.decode("utf-8")
                            
                            # Check if we should process this QR
                            if not self.should_process_qr(qr_data):
                                continue
                            
                            qr_detected = True
                            
                            # Draw rectangle and text
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.putText(frame, qr_data, (x1, y1 - 10),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                            
                            # Get confidence if available
                            confidence = float(boxes.conf[0]) if hasattr(boxes, 'conf') else 1.0
                            
                            # Process QR detection
                            self.process_qr_detection(qr_data, frame, confidence)
                            
                        except Exception as e:
                            self.get_logger().error(f'Error processing QR: {str(e)}')
            
            # Publish debug image if enabled
            if qr_detected and self.enable_viz:
                try:
                    debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    self.debug_image_pub.publish(debug_msg)
                except Exception as e:
                    self.get_logger().error(f'Error publishing debug image: {str(e)}')
        
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')
    
    def process_qr_detection(self, qr_data, frame, confidence):
        """Process detected QR code and add to RTAB-Map"""
        # Skip if QR code has already been labeled successfully
        with self.lock:
            if qr_data in self.labeled_qr_codes:
                self.get_logger().debug(f'Skipping already labeled QR code: {qr_data}')
                return
        
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        image_filename = f"qr_{qr_data}_{timestamp}.jpg"
        image_path = os.path.join(self.images_dir, image_filename)
        
        # Save the frame
        cv2.imwrite(image_path, frame)
        
        # Get current pose
        pose = self.get_current_pose()
        
        if pose is None:
            self.get_logger().warn('⚠️  No odometry data available yet')
        
        # Add label to RTAB-Map
        label_status = "success" if self.add_label_to_rtabmap(qr_data) else "failed"
        
        # If labeling was successful, add to tracked set
        if label_status == "success":
            with self.lock:
                self.labeled_qr_codes.add(qr_data)
        
        # Save to CSV
        with self.lock:
            new_row = {
                "timestamp": timestamp.replace("_", " "),
                "qr_data": qr_data,
                "image_path": image_path,
                "label_status": label_status,
                "confidence": f"{confidence:.2f}",
                "pos_x": pose['pos_x'] if pose else None,
                "pos_y": pose['pos_y'] if pose else None,
                "pos_z": pose['pos_z'] if pose else None,
                "orient_x": pose['orient_x'] if pose else None,
                "orient_y": pose['orient_y'] if pose else None,
                "orient_z": pose['orient_z'] if pose else None,
                "orient_w": pose['orient_w'] if pose else None
            }
            self.df = pd.concat([self.df, pd.DataFrame([new_row])], ignore_index=True)
            self.df.to_csv(self.csv_file, index=False)
        
        # Publish QR data
        qr_msg = String()
        qr_msg.data = qr_data
        self.qr_data_pub.publish(qr_msg)
        
        # Log detection
        self.get_logger().info('═' * 60)
        self.get_logger().info(f'✅ QR CODE DETECTED: {qr_data}')
        self.get_logger().info(f'📸 Image: {image_filename}')
        self.get_logger().info(f'🎯 Confidence: {confidence:.2f}')
        self.get_logger().info(f'🏷️  Label status: {label_status}')
        if pose:
            self.get_logger().info(f'📍 Position: x={pose["pos_x"]:.3f}, y={pose["pos_y"]:.3f}, z={pose["pos_z"]:.3f}')
            self.get_logger().info(f'🧭 Orientation: x={pose["orient_x"]:.3f}, y={pose["orient_y"]:.3f}, z={pose["orient_z"]:.3f}, w={pose["orient_w"]:.3f}')
        self.get_logger().info('═' * 60)


def main(args=None):
    rclpy.init(args=args)
    
    node = StandaloneQRNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down QR labeling node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()