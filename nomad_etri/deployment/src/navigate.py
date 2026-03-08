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
from utils import msg_to_pil, to_numpy, transform_images, load_model

from vint_train.training.train_utils import get_action
import torch
from PIL import Image as PILImage
import numpy as np
import argparse  
import yaml
import time

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

# UTILS
from topic_names import (IMAGE_TOPIC,
                        WAYPOINT_TOPIC,
                        SAMPLED_ACTIONS_TOPIC)


# CONSTANTS
TOPOMAP_IMAGES_DIR = "../topomaps/images"
MODEL_WEIGHTS_PATH = "../model_weights"
ROBOT_CONFIG_PATH ="../config/robot.yaml"
MODEL_CONFIG_PATH = "../config/models.yaml"
with open(ROBOT_CONFIG_PATH, "r") as f:
    robot_config = yaml.safe_load(f)
MAX_V = robot_config["max_v"]  # 0.2
MAX_W = robot_config["max_w"]  # 0.4 
RATE = robot_config["frame_rate"] # 4

# GLOBALS
context_queue = []
context_size = None  
subgoal = []

# Load the model
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("Using device:", device)

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
        model_params = yaml.safe_load(f) # model_params : 선택한 모델의 yaml 파일 

    context_size = model_params["context_size"]

    # load model weights
    ckpth_path = model_paths[args.model]["ckpt_path"] #ckpth_path : 선택한 모델의 pth 파일 
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
    model.eval() # 평가모드 

    # load topomap
    topomap_filenames = sorted(os.listdir(os.path.join(
        TOPOMAP_IMAGES_DIR, args.dir)), key=lambda x: int(x.split(".")[0]))
    topomap_dir = f"{TOPOMAP_IMAGES_DIR}/{args.dir}" 
    num_nodes = len(os.listdir(topomap_dir))  
    topomap = [] 
    for i in range(num_nodes):
        image_path = os.path.join(topomap_dir, topomap_filenames[i])
        topomap.append(PILImage.open(image_path)) # 이미지 파일들을 pil 파일로 변환 후 리스트에 저장 

    closest_node = 0
    assert -1 <= args.goal_node < len(topomap), "Invalid goal index"
    if args.goal_node == -1: 
        goal_node = len(topomap) - 1 
    else:
        goal_node = args.goal_node    
    reached_goal = Bool() # add
    reached_goal.data = False

    # ROS
    rclpy.init(args=None)
    node = rclpy.create_node('EXPLORATION')

    rate = Rate(RATE)
    publisher_goal = node.create_publisher(Bool, "/topoplan/reached_goal",1)
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

    if model_params["model_type"] == "nomad":
        num_diffusion_iters = model_params["num_diffusion_iters"]

        noise_scheduler = DDPMScheduler(
            num_train_timesteps=model_params["num_diffusion_iters"],
            beta_schedule='squaredcos_cap_v2',
            clip_sample=True, #샘플값이 특정 범위를 넘지 않도록 클리핑
            prediction_type='epsilon'
        )

    first = True
    # navigation loop
    while rclpy.ok():
        # EXPLORATION MODE
        rclpy.spin_once(node)
        chosen_waypoint = np.zeros(4)
        if len(context_queue) > model_params["context_size"]:
            if model_params["model_type"] == "nomad":
            # 주어진 pil 이미지를 주어진 크기로 자른 후 텐서로 변환 및 결합 
                obs_images = transform_images(context_queue, model_params["image_size"], center_crop=False)
                obs_images = torch.split(obs_images, 3, dim=1)
                obs_images = torch.cat(obs_images, dim=1) 
                obs_images = obs_images.to(device)      
                mask = torch.zeros(1).long().to(device)  # ignore the goal

                start = max(closest_node - args.radius, 0)
                end = min(closest_node + args.radius + 1, goal_node)

                if first == True:
                    start = 0
                    end = goal_node
                    first = False
                    
                #topomap image를 가져와서 텐서로 변환 
                goal_image = [transform_images(g_img, model_params["image_size"], center_crop=False).to(device) for g_img in topomap[start:end + 1]]
                goal_image = torch.concat(goal_image, dim=0)

                pub_g_img = [x for x in topomap[start:end + 1]]
                publisher_subgoal_image = node.create_publisher(Image, "/subgoal_image" ,1)
                
                obsgoal_cond = model('vision_encoder', obs_img=obs_images.repeat(len(goal_image), 1, 1, 1), goal_img=goal_image, input_goal_mask=mask.repeat(len(goal_image)))
                dists = model("dist_pred_net", obsgoal_cond=obsgoal_cond)
                dists = to_numpy(dists.flatten())
                min_idx = np.argmin(dists) 
                closest_node = min_idx + start 
                print("closest node:", closest_node)
                sg_idx = min(min_idx + int(dists[min_idx] < args.close_threshold), len(obsgoal_cond) - 1)  
                obs_cond = obsgoal_cond[sg_idx].unsqueeze(0)

                subgoal_image = np.array(pub_g_img[sg_idx]) # PIL 이미지를 넘파이 변환
                subgoal_image = subgoal_image[:, :, ::-1].copy() # 순서를 BGR로 변환한 뒤, 배열 복사
                bridge = CvBridge()
                subimage_msg = bridge.cv2_to_imgmsg(subgoal_image, "bgr8") # ros 이미지 메시지 형태로 변환
                publisher_subgoal_image.publish(subimage_msg)

                # infer action
                with torch.no_grad():
                    # encoder vision features
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
                sampled_actions_msg = Float32MultiArray()
                sampled_actions_msg.data = [float(x) for x in (np.concatenate((np.array([0]), naction.flatten())))]
                print("published sampled actions")
                publisher_smapled_action.publish(sampled_actions_msg)
                visualize_other_path(naction, publisher_path_rviz2)
                visualize_path_on_image(naction, publisher_image_path)
                naction = naction[0]      
                chosen_waypoint = copy.deepcopy(naction[args.waypoint])
                visualize_selected_path(naction, publisher_selected_path_rviz2)
            elif (len(context_queue) > model_params["context_size"]): 
                start = max(closest_node - args.radius, 0)   
                end = min(closest_node + args.radius + 1, goal_node)
                distances = []
                waypoints = []
                batch_obs_imgs = []
                batch_goal_data = []
                for i, sg_img in enumerate(topomap[start: end + 1]):
                    transf_obs_img = transform_images(context_queue, model_params["image_size"])
                    goal_data = transform_images(sg_img, model_params["image_size"])
                    batch_obs_imgs.append(transf_obs_img)
                    batch_goal_data.append(goal_data)
                    
                # predict distances and waypoints
                batch_obs_imgs = torch.cat(batch_obs_imgs, dim=0).to(device)
                batch_goal_data = torch.cat(batch_goal_data, dim=0).to(device)

                distances, waypoints = model(batch_obs_imgs, batch_goal_data)
                distances = to_numpy(distances)
                waypoints = to_numpy(waypoints)
                # look for closest node
                closest_node = np.argmin(distances)
                # chose subgoal and output waypoints
                if distances[closest_node] > args.close_threshold:
                    chosen_waypoint = waypoints[closest_node][args.waypoint]
                    sg_img = topomap[start + closest_node]
                else:
                    chosen_waypoint = waypoints[min(
                        closest_node + 1, len(waypoints) - 1)][args.waypoint]
                    sg_img = topomap[start + min(closest_node + 1, len(waypoints) - 1)]     
        # RECOVERY MODE
        if model_params["normalize"]:
            chosen_waypoint[:2] *= (MAX_V / RATE)  
        waypoint_msg = Float32MultiArray()
        waypoint_msg.data = [float(x) for x in chosen_waypoint]
        publisher_waypoint.publish(waypoint_msg)
        print("Published waypoint")

        reached_goal.data = bool(closest_node == goal_node)
        publisher_goal.publish(reached_goal)
        print(f'goal data: {goal_node}')
        visualize_maker(chosen_waypoint, publisher_target_point)
        if reached_goal.data:
            print("Reached goal! Stopping...")
            rclpy.shutdown()
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
        help="model name (only nomad is supported) (hint: check ../config/models.yaml) (default: nomad)",
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
        "--dir",
        "-d",
        default="topomap",
        type=str,
        help="path to topomap images",
    )
    parser.add_argument(
        "--goal-node",
        "-g",
        default=-1,
        type=int,
        help="""goal node index in the topomap (if -1, then the goal node is 
        the last node in the topomap) (default: -1)""",
    )
    parser.add_argument(
        "--close-threshold",
        "-t",
        default=3,
        type=int,
        help="""temporal distance within the next node in the topomap before 
        localizing to it (default: 3)""",
    )
    parser.add_argument(
        "--radius",
        "-r",
        default=4,
        type=int,
        help="""temporal number of locobal nodes to look at in the topopmap for
        localization (default: 2)""",
    )
    parser.add_argument(
        "--num-samples",
        "-n",
        default=8,
        type=int,
        help=f"Number of actions sampled from the exploration model (default: 8)",
    )

    parser.add_argument(
        "--dataset_name",
        "-dn",
        default="ZED2",
        type=str,
        help=f"using dataset name (default: scand)",
    )

    args = parser.parse_args()
    print(f"Using {device}")
    main(args)

