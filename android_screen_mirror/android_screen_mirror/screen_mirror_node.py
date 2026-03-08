# deploy시 로그, screenshot_pub 폴더에 저장 관련 코드 주석 처리 필요

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import subprocess
import threading
import time
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from android_screen_mirror.utils  import Rate

# 아래 두 주기 모두 이것보단 느리게 진행되도록 강제. 작동이 느리면 주기보다 늦을 수는 있음. 주기보다 빠르는 건 막음.
# TIMER_PERIOD 보다 조금 짧게 하면 될 듯
CAPTURE_PERIOD = 0.8  # seconds
# navigate.py의 rclpy.spin_once(node)가 다시 실행되는 데 걸리는 평균 주기의 절반 정도로 해서, 3개 이상 지나치게 많이 publish 해둬서 불필요하게 subscribe 여러 번 해서 시간 낭비하지 않게, 그 사이에 갱신이 아예 안 돼있을 일도 없게
TIMER_PERIOD = 1.0  # seconds

class AndroidScreenMirrorNode(Node):
    def __init__(self):
        super().__init__('android_screen_mirror_node')
        
        # Create publisher for image frames
        # 마지막 인자 publishing queue size
        self.image_publisher = self.create_publisher(
            Image, 
            '/android/screen_image', 
            3
        )
        
        # Timer for publishing screen captures
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        
        self.capture_thread = None

        self.current_frame = None
        self.frame_lock = threading.Lock()

        # CV Bridge for converting between OpenCV and ROS images
        self.bridge = CvBridge()
        
        # Start capture thread
        self.start_screen_capture()
        
        self.get_logger().info('Android Screen Mirror Node started')
    
    def start_screen_capture(self):
        """Start a thread to capture the Android screen using scrcpy"""
        self.capture_thread = threading.Thread(target=self._capture_screen)
        # daemon thread is a background thread that will automatically terminate when the main program exits
        self.capture_thread.daemon = True
        self.capture_thread.start()
    
    def _capture_screen(self):
        try:
            # Check if ADB is connected to any device with retries
            self.check_adb_connection()

            rate = Rate(1.0 / CAPTURE_PERIOD)  # Create a rate object for the capture period
            while rclpy.ok():
                # 아래 있는 캡처 간격 로그 작성용.
                # before_capture = time.time()

                capture_cmd = ['adb', 'exec-out', 'screencap', '-p']
                capture_process = subprocess.run(capture_cmd, check=True, timeout=5.0, stdout=subprocess.PIPE)
                
                # screenshot 폴더에 생성해서 확인해보기. deployment 시에는 주석처리 필요
                screenshots_dir = './screenshots_pub'
                os.makedirs(screenshots_dir, exist_ok=True)
                # Capture screenshot using adb with timestamp in filename
                timestamp = time.strftime("%m_%d_%H_%M_%S", time.localtime())  # Format: month_day_hour_min_sec
                screenshot_path = os.path.join(screenshots_dir, f'screen_{timestamp}.png')              
                with open(screenshot_path, 'wb') as f:
                    f.write(capture_process.stdout) 

                # Convert stdout bytes directly to numpy array
                image_array = np.frombuffer(capture_process.stdout, dtype=np.uint8)
                # OpenCV's imdecode() always returns BGR when using IMREAD_COLOR
                frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
                
                if frame is None:
                    self.get_logger().error('Failed to decode image from screencap')
                    continue

                with self.frame_lock:
                    self.current_frame = frame

                # Brief sleep to prevent excessive CPU usage
                rate.sleep()

                # after_capture = time.time()
                # self.get_logger().info("Frame captured after {:.1f} seconds".format(after_capture - before_capture))

            # Clean up subprocess when ending
            capture_process.terminate()
            capture_process.wait(timeout=1.0)

        except Exception as e:
            self.get_logger().error(f'Screen capture error: {e}')
            time.sleep(2.0)
    
    def check_adb_connection(self):
        """
        Check if ADB is connected to any device

        Returns:
            bool: True if connected to a device, False otherwise
        """

        try:
            result = subprocess.run(
                ['adb', 'devices'], 
                capture_output=True, 
                text=True, 
                check=True,
                timeout=5.0
            )
            
            lines = result.stdout.strip().split('\n')
            # Check if there are device lines (more than just the "List of devices attached" header)
            if len(lines) <= 1:
                self.get_logger().error('No Android devices found. Connect a device and try again.')
                return False
            
            # Look for actual device entries (not empty lines or "unauthorized" devices)
            devices = [line for line in lines[1:] if line.strip() and 'unauthorized' not in line.lower()]
            if not devices:
                self.get_logger().error('No authorized Android devices found.')
                return False
                
            self.get_logger().info(f'ADB connected to {len(devices)} device(s)')
            return True
            
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'ADB error: {e}')
            return False
        except subprocess.TimeoutExpired:
            self.get_logger().error('ADB command timed out')
            return False
        except Exception as e:
            self.get_logger().error(f'Unexpected error checking device connection: {e}')
            return False
    
    def timer_callback(self):
        """Timer callback to publish the current frame"""
        # before_capture = time.time()
        with self.frame_lock:
            if self.current_frame is not None:
                try:
                    # Convert the OpenCV image to a ROS image message
                    ros_image = self.bridge.cv2_to_imgmsg(self.current_frame, encoding="bgr8")
                    # Publish the image
                    self.image_publisher.publish(ros_image) 

                    # after_capture = time.time()
                    # self.get_logger().info("Frame published after {:.1f} seconds".format(after_capture - before_capture))
    
                except Exception as e:
                    self.get_logger().error(f'Error publishing image: {e}')
            else:
                self.get_logger().info("No frame available to publish")
    
    # 끝낼 때 main에서 사용
    def destroy_node(self):
        """Clean up when the node is shutting down"""
        if self.capture_thread:
            self.capture_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AndroidScreenMirrorNode()
    while rclpy.ok():
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
