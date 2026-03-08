import numpy as np
import yaml
from typing import Tuple

# ROS
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Bool

from topic_names import (WAYPOINT_TOPIC, 
						REACHED_GOAL_TOPIC)
from ros_data import ROSData
from utils import clip_angle
import time # 추가

# CONSTS
CONFIG_PATH = "../config/robot.yaml"
with open(CONFIG_PATH, "r") as f:
	robot_config = yaml.safe_load(f)
MAX_V = robot_config["max_v"]
MAX_W = robot_config["max_w"]
VEL_TOPIC = robot_config["vel_navi_topic"] # 최종 로봇 제어 토픽 
DT = 1/robot_config["frame_rate"]
RATE = 9
EPS = 1e-8
#제어 코드에서 EPS는 작은수 즉 목표값과 실제값 차이가 특정 임계값 보다 작을 때 EPS로 임계값을 설정 
WAYPOINT_TIMEOUT = 1 # seconds # TODO: tune this
FLIP_ANG_VEL = np.pi/4

# GLOBALS
vel_msg = Twist()
waypoint = ROSData(WAYPOINT_TIMEOUT, name="waypoint")
reached_goal = False
reverse_mode = False
current_yaw = None

# 추가
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
#

def clip_angle(theta) -> float:
	"""Clip angle to [-pi, pi]"""
	theta %= 2 * np.pi
	if -np.pi < theta < np.pi:
		return theta
	return theta - 2 * np.pi
	
# shape 배열의 크기 출력, [[[1,2] [3,4]], [[5,6] [7,8]]]  -> (2,2,2)  dim 은 차원의 수를 출력 
# len() 은 numpy 배열의 최상위 차원의 크기 반환. 즉, 첫번째 차원에 있는 요소의 개수를 알 수 있음 
def pd_controller(waypoint: np.ndarray) -> Tuple[float]: # numpy 형태로 매개변수를 받고, return은 float 형태임 
	"""PD controller for the robot"""
	assert len(waypoint) == 2 or len(waypoint) == 4, "waypoint must be a 2D or 4D vector" # 어떻게 waypoint의 최고 차원 요소가 4일까???
	if len(waypoint) == 2:
		dx, dy = waypoint
	else:
		dx, dy, hx, hy = waypoint
	# this controller only uses the predicted heading if dx and dy near zero
	if len(waypoint) == 4 and np.abs(dx) < EPS and np.abs(dy) < EPS:
		v = 0
		w = clip_angle(np.arctan2(hy, hx))/DT		
	elif np.abs(dx) < EPS:
		v =  0
		w = np.sign(dy) * np.pi/(2*DT) # np.sign (부호 판단): 양수인 경우 1, 음수인 경우 -1, 0인 경우 0 을 반환  
	else:
		v = dx / DT
		w = np.arctan(dy/dx) / DT
	v = np.clip(v, 0, MAX_V) # np.clip 0 보다 작은 값은 0으로, MAX_V 보다 큰 값을 MAX_V로 바꿔줌 
	w = np.clip(w, -MAX_W, MAX_W)
	return v, w #float 형태로 반환 


def callback_drive(waypoint_msg: Float32MultiArray):
	"""Callback function for the waypoint subscriber"""
	global vel_msg
	print("seting waypoint")
	waypoint.set(waypoint_msg.data)
	
	
def callback_reached_goal(reached_goal_msg: Bool):
	"""Callback function for the reached goal subscriber"""
	global reached_goal
	reached_goal = reached_goal_msg.data


def main():
	global vel_msg, reverse_mode
	rclpy.init(args=None)
	node = rclpy.create_node('PD_CONTROLLER')

	publisher_vel_out = node.create_publisher(Twist, VEL_TOPIC,1)

	subscription = node.create_subscription(
		Bool,
		REACHED_GOAL_TOPIC,
		callback_reached_goal,
		1)
	subscription = node.create_subscription(
		Float32MultiArray,
		WAYPOINT_TOPIC,
		callback_drive,
		1)
	
	rate = Rate(RATE)
	print("Registered with master node. Waiting for waypoints...")
	while rclpy.ok():
		vel_msg = Twist()
		rclpy.spin_once(node, timeout_sec=1) # 원본에선 이게 없는데, 그럼 callback은 어떻게 받는 건지
		if reached_goal:
			publisher_vel_out.publish(vel_msg)
			print("Reached goal! Stopping...")
			return
		elif waypoint.is_valid(verbose=True):
			v, w = pd_controller(waypoint.get())
			if reverse_mode:
				v *= -1
			vel_msg.linear.x = v
			vel_msg.angular.z = w
			print(f"publishing new vel: {v}, {w}")
		publisher_vel_out.publish(vel_msg)
		rate.sleep()
	

if __name__ == '__main__':
	main()
