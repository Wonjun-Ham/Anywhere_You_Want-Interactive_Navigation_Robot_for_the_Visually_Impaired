import argparse
import os
from utils import msg_to_pil 
import time

# ROS
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy

#=========추가=========
from sensor_msgs.msg import CompressedImage
from PIL import Image as PILImage
import io
import torchvision.transforms.functional as TF
IMAGE_SIZE = (160, 120)
IMAGE_ASPECT_RATIO = 4 / 3
# ===================

IMAGE_TOPIC = "/usb_cam/image_raw"
TOPOMAP_IMAGES_DIR = "../topomaps/images"
obs_img = None


def remove_files_in_dir(dir_path: str):
    for f in os.listdir(dir_path):
        file_path = os.path.join(dir_path, f)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print("Failed to delete %s. Reason: %s" % (file_path, e))

def callback_obs(msg: Image):
    global obs_img
    obs_img = msg_to_pil(msg)
    
    if obs_img.mode == 'RGBA':
        obs_img = obs_img.convert('RGB')

#=========추가=========
# def callback_obs(msg: CompressedImage):
#     global obs_img
#     img = PILImage.open(io.BytesIO(msg.data))
#     # center crop image to 4:3 aspect ratio
#     w, h = img.size
#     img = TF.center_crop(
#         img, (h, int(h * IMAGE_ASPECT_RATIO))
#     )  # crop to the right ratio
#     # resize image to IMAGE_SIZE
#     obs_img = img.resize(IMAGE_SIZE)

class Rate:
    def __init__(self, hz):
        self.period = 1.0 / hz
        self.last_time = time.time()

    def sleep(self):
        now = time.time()
        sleep_time = self.period - (now - self.last_time)
        if sleep_time > 0:
            time.sleep(sleep_time)
        self.last_time = time.time()
# ===================

def callback_joy(msg: Joy):
    if msg.buttons[0]:
        rospy.signal_shutdown("shutdown")


def main(args: argparse.Namespace):
    global obs_img
    rclpy.init(args=None)
    node = rclpy.create_node('CREATE_TOPOMAP')
    
    publisher_image = node.create_publisher(Image, "/subgoals",1)

    subscription = node.create_subscription(
        Image,
        IMAGE_TOPIC,
        callback_obs,
        1)
    subscription = node.create_subscription(
        Joy,
        "joy",
        callback_joy,
        1)

    topomap_name_dir = os.path.join(TOPOMAP_IMAGES_DIR, args.dir)
    if not os.path.isdir(topomap_name_dir):
        os.makedirs(topomap_name_dir)
    else:
        print(f"{topomap_name_dir} already exists. Removing previous images...")
        remove_files_in_dir(topomap_name_dir)
        

    assert args.dt > 0, "dt must be positive"
    rate = Rate(1 / args.dt)
    print("Registered with master node. Waiting for images...")
    i = 0
    start_time = float("inf")
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1) # timeout_sec=1 추가 
        if obs_img is not None:
            obs_img.save(os.path.join(topomap_name_dir, f"{i}.png"))
            print("published image", i)
            i += 1
            rate.sleep()
            start_time = time.time()
            obs_img = None
        if time.time() - start_time > 2 * args.dt:
            print(f"Topic {IMAGE_TOPIC} not publishing anymore. Shutting down...")
            rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser( 
        description=f"Code to generate topomaps from the {IMAGE_TOPIC} topic"
    )
    parser.add_argument(
        "--dir",
        "-d",
        default="topomap",
        type=str,
        help="path to topological map images in ../topomaps/images directory (default: topomap)",
    )
    parser.add_argument(
        "--dt",
        "-t",
        default=1.,
        type=float,
        help=f"time between images sampled from the {IMAGE_TOPIC} topic (default: 3.0)",
    )
    args = parser.parse_args()

    main(args)
