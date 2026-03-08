
import matplotlib.pyplot as plt
import os
from typing import Tuple, Sequence, Dict, Union, Optional, Callable
import numpy as np
import torch
import torch.nn as nn
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler

import matplotlib.pyplot as plt
import yaml
import copy
# ROS
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray
from utils import msg_to_pil, to_numpy, transform_images, load_model, Rate

from vint_train.training.train_utils import get_action
import torch
from PIL import Image as PILImage
import numpy as np
import argparse
import yaml
import time

# =======add=========
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from vint_train.visualizing.action_utils import plot_trajs_and_points_on_image, get_pos_pixels, gen_camera_matrix

# load data_config.yaml
with open(os.path.join(os.path.dirname(__file__), "../../train/vint_train/data/data_config.yaml"), "r") as f:
    data_config = yaml.safe_load(f)

# =================

# UTILS
from topic_names import (IMAGE_TOPIC,
                        WAYPOINT_TOPIC,
                        SAMPLED_ACTIONS_TOPIC)


# CONSTANTS
MODEL_WEIGHTS_PATH = "../model_weights"
ROBOT_CONFIG_PATH ="../config/robot.yaml"
MODEL_CONFIG_PATH = "../config/models.yaml"
with open(ROBOT_CONFIG_PATH, "r") as f:
    robot_config = yaml.safe_load(f)
MAX_V = robot_config["max_v"]
MAX_W = robot_config["max_w"]
RATE = robot_config["frame_rate"] 

# GLOBALS
context_queue = []
context_size = None  

# Load the model 
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("Using device:", device)

# visualization
def visualize_selected_path(data_2d, publisher):
    marker_array = MarkerArray()
    marker = Marker()
    marker.header.stamp = rclpy.time.Time().to_msg()
    marker.header.frame_id = "map"
    marker.ns = "path"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.1  # 경로 두께 설정
    marker.color.a = 1.0
    marker.color.b = 1.0

    for point in data_2d[:args.waypoint+1]:
        p = Point()
        p.x = float(point[0])
        p.y = float(point[1])
        marker.points.append(p)
    marker_array.markers.append(marker)
    publisher.publish(marker_array)

def visualize_other_path(data_3d, publisher):
    marker_array = MarkerArray()
    marker_id = 1
    for i in range(1, len(data_3d)):
        data = data_3d[i]
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rclpy.time.Time().to_msg()
        marker.ns = "paths"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.a = 1.0
        marker.color.r = 1.0  
        marker.color.g = 1.0

        for point in data[:args.waypoint+1]:
            p = Point()
            p.x = float(point[0])
            p.y = float(point[1])
            marker.points.append(p)
        marker_array.markers.append(marker)
        marker_id += 1  # 각 마커를 고유하게 식별하기 위함 
    publisher.publish(marker_array)

def visualize_maker(data_2d, publisher):
    marker = Marker()
    point = Point()

    marker.header.frame_id = "map"
    marker.header.stamp = rclpy.time.Time().to_msg()
    marker.ns = "target"
    marker.id = 999
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3
    marker.color.a = 1.0
    marker.color.b = 1.0

    point.x = float(data_2d[0])
    point.y = float(data_2d[1])

    marker.points.append(point)
    publisher.publish(marker)

def visualize_path_on_image(data_3d, publisher):
    if (
        "camera_metrics" in data_config[args.dataset_name]
        and "camera_height" in data_config[args.dataset_name]["camera_metrics"]
        and "camera_matrix" in data_config[args.dataset_name]["camera_metrics"]
        and "dist_coeffs" in data_config[args.dataset_name]["camera_metrics"]
    ):
        camera_height = data_config[args.dataset_name]["camera_metrics"]["camera_height"]
        camera_x_offset = data_config[args.dataset_name]["camera_metrics"]["camera_x_offset"]

        fx = data_config[args.dataset_name]["camera_metrics"]["camera_matrix"]["fx"]
        fy = data_config[args.dataset_name]["camera_metrics"]["camera_matrix"]["fy"]
        cx = data_config[args.dataset_name]["camera_metrics"]["camera_matrix"]["cx"]
        cy = data_config[args.dataset_name]["camera_metrics"]["camera_matrix"]["cy"]
        camera_matrix = gen_camera_matrix(fx, fy, cx, cy)

        k1 = data_config[args.dataset_name]["camera_metrics"]["dist_coeffs"]["k1"]
        k2 = data_config[args.dataset_name]["camera_metrics"]["dist_coeffs"]["k2"]
        p1 = data_config[args.dataset_name]["camera_metrics"]["dist_coeffs"]["p1"]
        p2 = data_config[args.dataset_name]["camera_metrics"]["dist_coeffs"]["p2"]
        k3 = data_config[args.dataset_name]["camera_metrics"]["dist_coeffs"]["k3"]
        dist_coeffs = np.array([k1, k2, p1, p2, k3, 0.0, 0.0, 0.0])

        open_cv_image = np.array(context_queue[-1]) # PIL 이미지를 넘파이 변환
        open_cv_image = open_cv_image[:, :, ::-1].copy() # 순서를 BGR로 변환한 뒤, 배열 복사
                
        # # 자를 범위 설정 (x, y 축의 범위)
        # VIZ_IMAGE_SIZE = (640, 480)
        # x_start, x_end = 0.5, VIZ_IMAGE_SIZE[0] - 0.5
        # y_start, y_end = VIZ_IMAGE_SIZE[1] - 0.5, 0.
        # # 범위를 정수로 변환
        # x_start, x_end = int(np.ceil(x_start)), int(np.floor(x_end))
        # y_start, y_end = int(np.ceil(y_start)), int(np.floor(y_end)
        # # 이미지 자르기
        # open_cv_image = open_cv_image[y_end:y_start, x_start:x_end
        
        # Red, Green, Blue, Yellow, Cyan, Magenta, Orange, Gray
        color = [(0, 0, 255), (0, 255, 0),(255, 0, 0), (0, 255, 255), (255, 255, 0), (255, 0, 255), (0, 140, 255), [128, 128, 128]]
             
        for i, point in enumerate(data_3d):
            if len(point.shape) == 1:
                point = point[None, :2]
            else:
                point = point[:, :2]
                pt_pixels = get_pos_pixels(
                point, camera_height, camera_x_offset, camera_matrix, dist_coeffs, clip=True
                )

            if i==0:
                chosen_points_array = np.array(pt_pixels[:250], dtype=np.int32)
            else:
                points_array = np.array(pt_pixels[:250], dtype=np.int32)
                cv2.polylines(open_cv_image, [points_array], isClosed=False, color=color[3], thickness=5)

        cv2.polylines(open_cv_image, [chosen_points_array], isClosed=False, color=color[2], thickness=10)
        bridge = CvBridge()
        image_msg = bridge.cv2_to_imgmsg(open_cv_image, "bgr8") # ros 이미지 메시지 형태로 변환
        publisher.publish(image_msg)

def callback_obs(msg):
    obs_img = msg_to_pil(msg)

    if obs_img.mode == 'RGBA':
        obs_img = obs_img.convert('RGB')

    if context_size is not None:
        if len(context_queue) < context_size + 1:
            context_queue.append(obs_img)
        else:
            context_queue.pop(0)
            context_queue.append(obs_img)

def main(args: argparse.Namespace):
    global context_size

    # load model parameters
    with open(MODEL_CONFIG_PATH, "r") as f:
        model_paths = yaml.safe_load(f)

    model_config_path = model_paths[args.model]["config_path"]
    with open(model_config_path, "r") as f:
        model_params = yaml.safe_load(f)

    context_size = model_params["context_size"]

    # load model weights
    ckpth_path = model_paths[args.model]["ckpt_path"]
    if os.path.exists(ckpth_path):
        print(f"Loading model from {ckpth_path}")
    else:
        raise FileNotFoundError(f"Model weights not found at {ckpth_path}")
    model = load_model(
        ckpth_path,
        model_params,
        device,
    )
    model = model.to(device)
    model.eval()

    num_diffusion_iters = model_params["num_diffusion_iters"]
    noise_scheduler = DDPMScheduler(
        num_train_timesteps=model_params["num_diffusion_iters"],
        beta_schedule='squaredcos_cap_v2',
        clip_sample=True,
        prediction_type='epsilon'
    )

    # ROS
    rclpy.init(args=None)
    node = rclpy.create_node('EXPLORATION')
    rate = Rate(RATE)

    publisher_waypoint = node.create_publisher(Float32MultiArray, WAYPOINT_TOPIC,1)
    publisher_smapled_action = node.create_publisher(Float32MultiArray, SAMPLED_ACTIONS_TOPIC,1)

    # for visualization
    publisher_selected_path_rviz2 = node.create_publisher(MarkerArray, "selected_path",1)
    publisher_path_rviz2 = node.create_publisher(MarkerArray, "other_path",1)
    publisher_target_point = node.create_publisher(Marker, "target_waypoint",1)
    publisher_image_path = node.create_publisher(Image, "/image_path" ,1)

    subscription = node.create_subscription(
        Image,
        IMAGE_TOPIC,
        callback_obs,
        1)

    print("Registered with master node. Waiting for image observations...")

    while rclpy.ok():
        # EXPLORATION MODE
        rclpy.spin_once(node)
        waypoint_msg = Float32MultiArray()
        if (
                len(context_queue) > model_params["context_size"]
            ):
            
            obs_images = transform_images(context_queue, model_params["image_size"], center_crop=False)
            obs_images = obs_images.to(device)
            fake_goal = torch.randn((1, 3, *model_params["image_size"])).to(device)
            mask = torch.ones(1).long().to(device)

            # infer action
            with torch.no_grad():
                # encoder vision features
                obs_cond = model('vision_encoder', obs_img=obs_images, goal_img=fake_goal, input_goal_mask=mask)
                
                # (B, obs_horizon * obs_dim)
                if len(obs_cond.shape) == 2:
                    obs_cond = obs_cond.repeat(args.num_samples, 1)
                else:
                    obs_cond = obs_cond.repeat(args.num_samples, 1, 1)
                
                # initialize action from Gaussian noise
                noisy_action = torch.randn(
                    (args.num_samples, model_params["len_traj_pred"], 2), device=device)
                naction = noisy_action

                # init scheduler
                noise_scheduler.set_timesteps(num_diffusion_iters)

                start_time = time.time()
                for k in noise_scheduler.timesteps[:]:
                    # predict noise
                    noise_pred = model(
                        'noise_pred_net',
                        sample=naction,
                        timestep=k,
                        global_cond=obs_cond
                    )

                    # inverse diffusion step (remove noise)
                    naction = noise_scheduler.step(
                        model_output=noise_pred,
                        timestep=k,
                        sample=naction
                    ).prev_sample
                print("time elapsed:", time.time() - start_time)

            naction = to_numpy(get_action(naction))
            visualize_other_path(naction, publisher_path_rviz2)

            sampled_actions_msg = Float32MultiArray()
            sampled_actions_msg.data = [float(x) for x in np.concatenate((np.array([0.0]), naction.flatten()))]
            print("published sampled actions")
            publisher_smapled_action.publish(sampled_actions_msg)
            visualize_path_on_image(naction, publisher_image_path)
            naction = naction[0] # change this based on heuristic
            chosen_waypoint = copy.deepcopy(naction[args.waypoint])

            if model_params["normalize"]:
                chosen_waypoint *= (MAX_V / RATE)
            waypoint_msg.data = [float(x) for x in chosen_waypoint]
            publisher_waypoint.publish(waypoint_msg)
            visualize_selected_path(naction, publisher_selected_path_rviz2)
            visualize_maker(chosen_waypoint, publisher_target_point)
            print("Published waypoint")
        print('-----------------')
        rate.sleep() 

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Code to run GNM DIFFUSION EXPLORATION on the locobot")
    parser.add_argument(
        "--model",
        "-m",
        default="nomad",
        type=str,
        help="model name (hint: check ../config/models.yaml) (default: nomad)",
    )
    parser.add_argument(
        "--waypoint",
        "-w",
        default=2, # close waypoints exihibit straight line motion (the middle waypoint is a good default)
        type=int,
        help=f"""index of the waypoint used for navigation (between 0 and 4 or 
        how many waypoints your model predicts) (default: 2)""",
    )
    parser.add_argument(
        "--num-samples",
        "-n",
        default=8,
        type=int,
        help=f"Number of actions sampled from the exploration model (default: 8)",
    )
    # add
    parser.add_argument(
        "--dataset_name",
        "-dn",
        default="ZED2",
        type=str,
        help=f"using dataset name (default: scand)",
    )
    # ==
    args = parser.parse_args()
    print(f"Using {device}")
    main(args)


