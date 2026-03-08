# deploy시 screenshot_sub 폴더에 저장 관련 코드 주석 처리 필요


import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from android_screen_mirror.utils import msg_to_pil

class ScreenshotSubscriber(Node):
    def __init__(self):
        super().__init__('screenshot_subscriber')
        self.context_queue = []
        self.context_size = 5
        
        # Create screenshots directory
        self.screenshots_dir = './screenshots_sub'
        os.makedirs(self.screenshots_dir, exist_ok=True)

        self.subscription = self.create_subscription(
            Image,
            '/android/screen_image',
            self.callback_obs,
            1)
    
    def callback_obs(self, msg):
        obs_img = msg_to_pil(msg)

        # Handle context queue
        if self.context_size is not None:
            if len(self.context_queue) < self.context_size + 1:
                self.context_queue.append(obs_img)
            else:
                self.context_queue.pop(0)
                self.context_queue.append(obs_img)
        
        # 스크린샷 폴더에 저장
        timestamp = time.strftime("%m_%d_%H_%M_%S", time.localtime())
        screenshot_path = os.path.join(self.screenshots_dir, f'screen_{timestamp}.png')
        obs_img.save(screenshot_path)

def main(args=None):
    rclpy.init(args=args)
    node = ScreenshotSubscriber() 
    while rclpy.ok(): 
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
