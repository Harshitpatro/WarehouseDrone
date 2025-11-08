#!/usr/bin/env python3
"""
RealSense Camera Debug and Test Script
Tests individual camera initialization and helps diagnose issues
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import sys
import time

def list_devices():
    """List all connected RealSense devices"""
    ctx = rs.context()
    devices = ctx.query_devices()
    
    print("\n" + "="*60)
    print("CONNECTED REALSENSE DEVICES")
    print("="*60)
    
    if len(devices) == 0:
        print("❌ No RealSense devices found!")
        return []
    
    device_info = []
    for i, dev in enumerate(devices):
        serial = dev.get_info(rs.camera_info.serial_number)
        name = dev.get_info(rs.camera_info.name)
        fw = dev.get_info(rs.camera_info.firmware_version)
        usb = dev.get_info(rs.camera_info.usb_type_descriptor)
        
        print(f"\nDevice {i+1}:")
        print(f"  Name: {name}")
        print(f"  Serial: {serial}")
        print(f"  Firmware: {fw}")
        print(f"  USB: {usb}")
        
        # List available sensors
        sensors = dev.query_sensors()
        print(f"  Sensors:")
        for sensor in sensors:
            sensor_name = sensor.get_info(rs.camera_info.name)
            print(f"    - {sensor_name}")
        
        device_info.append({
            'serial': serial,
            'name': name,
            'device': dev
        })
    
    print("\n" + "="*60)
    return device_info

def test_single_camera(serial, name):
    """Test initialization of a single camera"""
    print(f"\n{'='*60}")
    print(f"TESTING: {name} (Serial: {serial})")
    print("="*60)
    
    try:
        # Create pipeline and config
        ctx = rs.context()
        pipeline = rs.pipeline(ctx)
        config = rs.config()
        
        # Enable device
        config.enable_device(serial)
        
        # Try different stream configurations
        configs = [
            {
                'name': 'Config 1: 848x480 @ 30fps',
                'streams': [
                    (rs.stream.infrared, 1, 848, 480, rs.format.y8, 30),
                    (rs.stream.depth, 848, 480, rs.format.z16, 30)
                ]
            },
            {
                'name': 'Config 2: 640x480 @ 30fps',
                'streams': [
                    (rs.stream.infrared, 1, 640, 480, rs.format.y8, 30),
                    (rs.stream.depth, 640, 480, rs.format.z16, 30)
                ]
            },
            {
                'name': 'Config 3: 640x480 @ 15fps',
                'streams': [
                    (rs.stream.infrared, 1, 640, 480, rs.format.y8, 15),
                    (rs.stream.depth, 640, 480, rs.format.z16, 15)
                ]
            }
        ]
        
        successful_config = None
        
        for cfg in configs:
            print(f"\nTrying {cfg['name']}...")
            
            # Create fresh config
            config = rs.config()
            config.enable_device(serial)
            
            for stream_params in cfg['streams']:
                config.enable_stream(*stream_params)
            
            # Try to start
            try:
                profile = pipeline.start(config)
                print(f"  ✓ Successfully started!")
                
                # Try to get a few frames
                print(f"  Testing frame capture...")
                for i in range(5):
                    frames = pipeline.wait_for_frames(timeout_ms=1000)
                    ir_frame = frames.get_infrared_frame(1)
                    depth_frame = frames.get_depth_frame()
                    
                    if ir_frame and depth_frame:
                        print(f"    Frame {i+1}/5: ✓")
                    else:
                        print(f"    Frame {i+1}/5: ❌ Missing frames")
                
                print(f"  ✓ Frame capture successful!")
                successful_config = cfg['name']
                
                # Get intrinsics
                ir_stream = profile.get_stream(rs.stream.infrared, 1)
                intrinsics = ir_stream.as_video_stream_profile().get_intrinsics()
                print(f"\n  Camera Intrinsics:")
                print(f"    Resolution: {intrinsics.width}x{intrinsics.height}")
                print(f"    Focal Length: ({intrinsics.fx:.2f}, {intrinsics.fy:.2f})")
                print(f"    Principal Point: ({intrinsics.ppx:.2f}, {intrinsics.ppy:.2f})")
                
                # Check for IMU (D435i only)
                if 'D435' in name:
                    print(f"\n  Checking IMU...")
                    try:
                        for i in range(3):
                            frames = pipeline.wait_for_frames(timeout_ms=1000)
                            accel = frames.first_or_default(rs.stream.accel)
                            gyro = frames.first_or_default(rs.stream.gyro)
                            
                            if accel and gyro:
                                accel_data = accel.as_motion_frame().get_motion_data()
                                gyro_data = gyro.as_motion_frame().get_motion_data()
                                print(f"    IMU Sample {i+1}: ✓")
                                print(f"      Accel: ({accel_data.x:.3f}, {accel_data.y:.3f}, {accel_data.z:.3f})")
                                print(f"      Gyro: ({gyro_data.x:.3f}, {gyro_data.y:.3f}, {gyro_data.z:.3f})")
                            else:
                                print(f"    IMU Sample {i+1}: ❌ No IMU data")
                    except Exception as e:
                        print(f"    IMU test failed: {e}")
                
                pipeline.stop()
                print(f"\n  ✓ Camera test PASSED with {cfg['name']}")
                break
                
            except Exception as e:
                print(f"  ❌ Failed: {e}")
                try:
                    pipeline.stop()
                except:
                    pass
                continue
        
        if successful_config:
            return True, successful_config
        else:
            print(f"\n  ❌ All configurations failed for this camera")
            return False, None
            
    except Exception as e:
        print(f"❌ Fatal error testing camera: {e}")
        return False, None

def test_multi_camera_init(device_list):
    """Test initializing multiple cameras simultaneously"""
    print(f"\n{'='*60}")
    print("TESTING MULTI-CAMERA INITIALIZATION")
    print("="*60)
    
    if len(device_list) < 2:
        print("Need at least 2 cameras for multi-camera test")
        return
    
    ctx = rs.context()
    pipelines = []
    
    # Configuration to use
    width, height, fps = 640, 480, 15
    
    print(f"\nAttempting to initialize {len(device_list)} cameras simultaneously...")
    print(f"Configuration: {width}x{height} @ {fps}fps")
    
    for i, dev_info in enumerate(device_list):
        serial = dev_info['serial']
        name = dev_info['name']
        
        print(f"\n  Camera {i+1}: {name} ({serial})")
        
        try:
            pipeline = rs.pipeline(ctx)
            config = rs.config()
            config.enable_device(serial)
            
            # Use conservative settings
            config.enable_stream(rs.stream.infrared, 1, width, height, rs.format.y8, fps)
            config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
            
            print(f"    Starting pipeline...")
            profile = pipeline.start(config)
            
            pipelines.append({
                'name': name,
                'serial': serial,
                'pipeline': pipeline
            })
            
            print(f"    ✓ Started successfully")
            
            # Small delay between initializations
            time.sleep(0.3)
            
        except Exception as e:
            print(f"    ❌ Failed: {e}")
    
    if len(pipelines) == 0:
        print("\n❌ No cameras initialized successfully")
        return
    
    print(f"\n✓ Successfully initialized {len(pipelines)}/{len(device_list)} cameras")
    
    # Test frame capture from all cameras
    print(f"\nTesting simultaneous frame capture...")
    
    for test_round in range(5):
        print(f"  Round {test_round + 1}/5:")
        for pipe_info in pipelines:
            try:
                frames = pipe_info['pipeline'].poll_for_frames()
                if frames:
                    ir = frames.get_infrared_frame(1)
                    depth = frames.get_depth_frame()
                    if ir and depth:
                        print(f"    {pipe_info['name']}: ✓")
                    else:
                        print(f"    {pipe_info['name']}: ⚠ Incomplete frameset")
                else:
                    print(f"    {pipe_info['name']}: ⚠ No frames")
            except Exception as e:
                print(f"    {pipe_info['name']}: ❌ {e}")
        
        time.sleep(0.1)
    
    # Cleanup
    print(f"\nStopping all pipelines...")
    for pipe_info in pipelines:
        try:
            pipe_info['pipeline'].stop()
            print(f"  {pipe_info['name']}: ✓ Stopped")
        except Exception as e:
            print(f"  {pipe_info['name']}: ❌ Error stopping: {e}")
    
    print(f"\n✓ Multi-camera test complete")

def show_camera_streams(serial, name):
    """Show live camera streams"""
    print(f"\nShowing streams from {name} ({serial})")
    print("Press 'q' to quit")
    
    ctx = rs.context()
    pipeline = rs.pipeline(ctx)
    config = rs.config()
    config.enable_device(serial)
    
    config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    try:
        pipeline.start(config)
        
        while True:
            frames = pipeline.wait_for_frames()
            
            ir_frame = frames.get_infrared_frame(1)
            depth_frame = frames.get_depth_frame()
            
            if ir_frame and depth_frame:
                ir_image = np.asanyarray(ir_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                
                # Colorize depth
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03),
                    cv2.COLORMAP_JET
                )
                
                # Stack images
                combined = np.hstack((
                    cv2.cvtColor(ir_image, cv2.COLOR_GRAY2BGR),
                    depth_colormap
                ))
                
                cv2.imshow(f'{name} - IR (left) | Depth (right)', combined)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        pipeline.stop()
        cv2.destroyAllWindows()
        
    except Exception as e:
        print(f"Error showing streams: {e}")

def main():
    print("\n" + "="*60)
    print("REALSENSE CAMERA DEBUG TOOL")
    print("="*60)
    
    # List all devices
    devices = list_devices()
    
    if len(devices) == 0:
        print("\n❌ No devices found. Check USB connections.")
        return
    
    # Test each camera individually
    print("\n" + "="*60)
    print("INDIVIDUAL CAMERA TESTS")
    print("="*60)
    
    working_cameras = []
    for dev_info in devices:
        success, config = test_single_camera(dev_info['serial'], dev_info['name'])
        if success:
            working_cameras.append(dev_info)
            dev_info['working_config'] = config
    
    print(f"\n{'='*60}")
    print(f"SUMMARY: {len(working_cameras)}/{len(devices)} cameras working")
    print("="*60)
    
    if len(working_cameras) > 1:
        # Test multi-camera initialization
        test_multi_camera_init(working_cameras)
    
    # Interactive menu
    while True:
        print("\n" + "="*60)
        print("OPTIONS:")
        print("  1. Show live streams from a camera")
        print("  2. Re-test all cameras")
        print("  3. Test multi-camera again")
        print("  q. Quit")
        print("="*60)
        
        choice = input("\nSelect option: ").strip().lower()
        
        if choice == 'q':
            break
        elif choice == '1':
            print("\nAvailable cameras:")
            for i, dev in enumerate(working_cameras):
                print(f"  {i+1}. {dev['name']} ({dev['serial']})")
            
            cam_choice = input("Select camera (1-{}): ".format(len(working_cameras)))
            try:
                idx = int(cam_choice) - 1
                if 0 <= idx < len(working_cameras):
                    dev = working_cameras[idx]
                    show_camera_streams(dev['serial'], dev['name'])
            except:
                print("Invalid selection")
        elif choice == '2':
            devices = list_devices()
            working_cameras = []
            for dev_info in devices:
                success, config = test_single_camera(dev_info['serial'], dev_info['name'])
                if success:
                    working_cameras.append(dev_info)
        elif choice == '3':
            if len(working_cameras) > 1:
                test_multi_camera_init(working_cameras)
            else:
                print("Need at least 2 working cameras")

if __name__ == '__main__':
    main()