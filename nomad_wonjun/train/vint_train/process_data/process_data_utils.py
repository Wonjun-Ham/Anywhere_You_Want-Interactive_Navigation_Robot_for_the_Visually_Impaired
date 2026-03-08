# ETRI : get_images_and_odom 변경. 이를 위해 process_odom_data 추가, 기존 코드 대신 check_topics_in_bag 만들어 대체

import numpy as np
import io
import os
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from PIL import Image
import cv2
from typing import Any, Tuple, List, Dict
import torchvision.transforms.functional as TF

import time
from nav_msgs.msg import Odometry
import rclpy
from rclpy.serialization import deserialize_message

IMAGE_SIZE = (160, 120)
IMAGE_ASPECT_RATIO = 4 / 3

# etri 추가. get_images_and_odom에서 사용
def process_odom_data(data: bytes) -> Odometry:
    odom_msg = deserialize_message(data, Odometry)
    return odom_msg


def process_images(im_list: List, img_process_func) -> List:
    """
    Process image data from a topic that publishes ros images into a list of PIL images
    """
    images = []
    for img_msg in im_list:
        img = img_process_func(img_msg)
        images.append(img)
    return images


def process_tartan_img(msg) -> Image:
    """
    Process image data from a topic that publishes sensor_msgs/Image to a PIL image for the tartan_drive dataset
    """
    img = ros_to_numpy(msg, output_resolution=IMAGE_SIZE) * 255
    img = img.astype(np.uint8)
    # reverse the axis order to get the image in the right orientation
    img = np.moveaxis(img, 0, -1)
    # convert rgb to bgr
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    img = Image.fromarray(img)
    return img

# deployment에서도 쓰고 있는 이 locobot은, camera_front.yaml에 적었듯
# 노드에서 publish시 rgb로 publish함.
# 그래서 img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) 필요없음
def process_locobot_img(msg) -> Image:
    """
    Process image data from a topic that publishes sensor_msgs/Image to a PIL image for the locobot dataset
    """
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
        msg.height, msg.width, -1)
    pil_image = Image.fromarray(img)
    return pil_image


def process_scand_img(msg) -> Image:
    """
    Process image data from a topic that publishes sensor_msgs/CompressedImage to a PIL image for the scand dataset
    """

    # convert sensor_msgs/CompressedImage to PIL image

    # jpeg면 rgb로, png면 rgb 또는 rgba로 변환됨. 
    # 이 데이터는 process_bags.py에서 에러없이 jpg로 잘 저장됐다는 건데, 
    # 이를 통해 jpeg였을 것임을 알 수 있음. png였다면 rgb로 바꿔줘야 함
    img = Image.open(io.BytesIO(msg))

    # center crop image to 4:3 aspect ratio
    w, h = img.size
    img = TF.center_crop(
        img, (h, int(h * IMAGE_ASPECT_RATIO))
    )  # crop to the right ratio
    # resize image to IMAGE_SIZE
    img = img.resize(IMAGE_SIZE)
    return img


############## Add custom image processing functions here #############

# 원래부터 있던 함수. sascon도 process_bags_config.yaml 보니까 CompressedImage 
# process_scand_img 방식으로 하는 게 더 간단. img = Image.open(io.BytesIO(msg))면 끝.
def process_sacson_img(msg) -> Image:
    # [수정] np.fromstring() is deprecated since NumPy 1.19. You should use np.frombuffer() instead.
    np_arr = np.frombuffer(msg.data, np.uint8)
    # OpenCV's imdecode() always returns BGR when using IMREAD_COLOR
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
    pil_image = Image.fromarray(image_np)
    return pil_image


# data가 sensor_msgs/Image, sensor_msgs/CompressedImage 중 
# 어떤 건지 따라 다르게 처리해야 함. 이 파일의 process_~_img 함수들 참고. 

# 1. sensor_msgs/Image일 때
# ros2 sensor_msgs/Image 인코딩 정보
# https://docs.ros2.org/foxy/api/sensor_msgs/msg/Image.html
# https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/include/sensor_msgs/image_encodings.hpp
# 아래 코드 현재는 "rgb8""rgba8""rgb16""rgba16""bgr8""bgra8""bgr16""bgra16"만 고려
'''
# bgr, rgb 얘기 있으면
if "bgr" in msg.encoding or "rgb" in msg.encoding:
    
    if "8" in msg.encoding:  # For 8-bit images
        dtype = np.uint8
    else: # For 16-bit images
        dtype = np.uint16  
    # convert to numpy array
    img = np.frombuffer(msg.data, dtype=dtype).reshape(
        msg.height, msg.width, -1)
        
    # For 16-bit images, scale down to 8-bit for PIL
    if dtype == np.uint16:
        img = (img / 256).astype(np.uint8)

    # (rgba, bgra) to (rgb, bgr)  
    if img.shape[-1] == 4: 
        img = img[:, :, :3]  # Keep only rgb or bgr channels

    # bgr to rgb
    if "bgr" in msg.encoding:
        img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # convert to PIL Image
    img = PILImage.fromarray(img)

else:
    raise ValueError(f"encoding format : {msg.encoding}. modify this function to handle this case")    

return img
'''

# 2. sensor_msgs/CompressedImage일 때
# 이것도 1번처럼 사이트 찾아서 더 고려할 경우 있나 확인
"""
# 디폴트로 bgr이 아닌 rgb 형태로 받아들임
img = Image.open(io.BytesIO(msg))
if img.mode in ['RGBA', 'L', 'I']:  # L=8-bit grayscale, I=16-bit grayscale
    img = img.convert('RGB') 
"""

#######################################################################


def process_odom(
    odom_list: List,
    odom_process_func: Any,
    ang_offset: float = 0.0,
) -> Dict[np.ndarray, np.ndarray]:
    """
    Process odom data from a topic that publishes nav_msgs/Odometry into position and yaw
    """
    xys = []
    yaws = []
    for odom_msg in odom_list:
        xy, yaw = odom_process_func(odom_msg, ang_offset)
        xys.append(xy)
        yaws.append(yaw)
    return {"position": np.array(xys), "yaw": np.array(yaws)}


def nav_to_xy_yaw(odom_msg, ang_offset: float) -> Tuple[List[float], float]:
    """
    Process odom data from a topic that publishes nav_msgs/Odometry into position
    """

    position = odom_msg.pose.pose.position
    orientation = odom_msg.pose.pose.orientation
    yaw = (
        quat_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
        + ang_offset
    )
    return [position.x, position.y], yaw


############ Add custom odometry processing functions here ############


#######################################################################

# etri 추가
def check_topics_in_bag(reader: SequentialReader, imtopics: List[str], odomtopics: List[str]) -> Tuple[str, str]:
    """
    Check if bag file contains both image and odom topics

    Args:
        reader (SequentialReader): ROS 2 bag reader
        imtopics (list[str]): list of image topics
        odomtopics (list[str]): list of odom topics

    Returns:
        tuple: (imtopic, odomtopic) if both topics exist, else (None, None)
    """
    # Get all topics and types from the bag
    topic_types = {info.name: info.type for info in reader.get_all_topics_and_types()}

    # Check for image topics
    imtopic = None
    for imt in imtopics:
        if imt in topic_types:
            imtopic = imt
            break

    # Check for odom topics
    odomtopic = None
    for ot in odomtopics:
        if ot in topic_types:
            odomtopic = ot
            break

    # Return topics if both exist
    if not (imtopic and odomtopic):
        return None, None

    return imtopic, odomtopic

# 이 함수 etri에서 많이 변경했음
def get_images_and_odom(
    reader: SequentialReader,
    imtopics: List[str] or str,
    odomtopics: List[str] or str,
    img_process_func: Any,
    odom_process_func: Any,
    rate: float = 4.0,
    ang_offset: float = 0.0,
):
    """
    Get image and odom data from a bag file

    Args:
        bag (rosbag.Bag): bag file
        imtopics (list[str] or str): topic name(s) for image data
        odomtopics (list[str] or str): topic name(s) for odom data
        img_process_func (Any): function to process image data
        odom_process_func (Any): function to process odom data
        rate (float, optional): rate to sample data. Defaults to 4.0.
        ang_offset (float, optional): angle offset to add to odom data. Defaults to 0.0.
    Returns:
        img_data (list): list of PIL images
        traj_data (list): list of odom data
    """
    # check if bag has both topics
    odomtopic = None
    imtopic = None

    # etri 추가 (여기 있던 코드들 대신 이 함수로 대체했음. 같은 기능)
    imtopic, odomtopic = check_topics_in_bag(reader, imtopics, odomtopics)

    if not (imtopic and odomtopic):
        return None, None

    synced_imdata = []
    synced_odomdata = []
    start_time = None

    curr_imdata = None
    curr_odomdata = None

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if start_time is None:
            start_time = timestamp

        if topic == imtopic:
            curr_imdata = data
        elif topic == odomtopic:
            curr_odomdata = data

        # timestamp를 이용해 sync된 image와 odometry(pos,yaw) 쌍을 저장
        # 처음에 일부러 if문 만족 안 되게 함. 어차피 image,odometry 데이터 모두 있는 상태로 실행돼야하는데 처음에 하나밖에 없을테니.
        # ros2에선 단위 nanoseconds여서 1e9 곱하는 걸로 변경한 걸로 추정. etri에서 변경
        # 1/rate 초(디폴트는 0.25초) 안의 새로운 데이터면 무시하는 것
        if (timestamp - start_time) >= (1.0 / rate) * 1e9:
            # image,odometry 데이터 모두 있는 상태로
            if curr_imdata is not None and curr_odomdata is not None:
                
                # 기존 노마드에선 이 세 줄이 이 블록에서 전부. 왜냐면,
                # ros2의 reader.read_next()는 rosbag.Bag.read_messages()와 달리 serialized 된 값을 리턴.
                # 기존 코드에선 Image(), CompressedImage(), Odometry()를 리턴했던 것.
                # synced_imdata.append(curr_imdata)
                # synced_odomdata.append(curr_odomdata)
                # currtime = t.to_sec()

                # curr_imdata를 sensor_msgs/CompressedImage로 가정하고 있음
                # sensor_msgs/Image에 대해선 에러가 날 것.
                # 원래 이게 이 데이터 형식의 처음과 끝이여서(아래 sensor_msgs/CompressedImage example 확인), 추출의 의미는 없음
                # 아래와 같이 jpeg 또는 png 아닌 거 들어와서 ValueError 뜨는지 확인하기 위해서 넣는다면, if문으로 둬야 함
                # 그런데 굳이 이러지 말고 아래 odometry process_odom_data()처럼 deserialize해서 넘기는 게 나을 듯
                # 그러면 process_images(synced_imdata, img_process_func)에서 img_process_func가 기존 노마드에서처럼 맞춰서 해결해줄테니
                # 어차피 process_scand_img를 제외하고는 msg.data를 사용하고 있기 때문에 deserialized 형태로 안 하면 에러
                # etri에서 process_bags.py 바꾼 것 중에(보고서 위치 언급) argument 입력 default scand로 바꾼 것 있었는데 scand만 test해본 걸로 보임
                # ---------------------------------------------------
                jpeg_start = curr_imdata.find(b'\xff\xd8')  # JPEG 시작
                jpeg_end = curr_imdata.find(b'\xff\xd9') + 2  # JPEG 종료

                # 엄밀히는 png여도 에러 안 남. sensor_msgs/CompressedImage 들어왔으면 에러 안 남.
                if jpeg_start == -1 or jpeg_end == -1:
                    raise ValueError("JPEG data not found in message")

                # JPEG 데이터 추출
                curr_imdata = curr_imdata[jpeg_start:jpeg_end]
                synced_imdata.append(curr_imdata)
                # ----------------------------------------------------
                
                """ # Example of sensor_msgs/Image message structure and data

                # Header information
                header:
                seq: 123
                stamp:
                    secs: 1621234567
                    nsecs: 890123456
                frame_id: "camera_frame"

                # Image dimensions
                height: 480    # pixels
                width: 640     # pixels

                # Image format
                encoding: "bgr8"   # common formats: bgr8, rgb8, mono8, etc.
                is_bigendian: 0    # byte order
                step: 1920         # = width * 3 (for BGR/RGB)

                # The actual image data as a byte array
                # Each pixel takes 3 bytes (for BGR/RGB)
                # Total size = height * step
                data: [
                    # First pixel (BGR format)
                    120, 64, 23,   # B=120, G=64, R=23
                    # Second pixel
                    98, 132, 45,
                    # ... continues for width*height pixels
                ] """

                """ sensor_msgs/CompressedImage
                string format  # "jpeg" or "png"
                uint8[] data   # Compressed image data

                Example of compressed JPEG data structure
                jpeg_data = [
                    # Header markers
                    0xFF, 0xD8,     # Start of Image (SOI)
                    
                    # Quantization tables (used for compression)
                    0xFF, 0xDB,     # Define Quantization Table marker
                    0x00, 0x84,     # Length
                    0x00,           # Table ID
                    # Luminance quantization values (8x8 matrix compressed)
                    0x10, 0x0B, 0x0C, 0x0E, 0x0C, 0x0A, 0x10, 0x0E,
                    0x0D, 0x0E, 0x12, 0x11, 0x10, 0x13, 0x18, 0x28,
                    # ...more quantization values...
                    
                    # Huffman tables
                    0xFF, 0xC4,     # Define Huffman Table marker
                    0x00, 0x1F,     # Length
                    # Huffman table data (compressed symbols)
                    0x00, 0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01,
                    
                    # Compressed image data (MCUs - Minimum Coded Units)
                    # Each MCU contains 8x8 blocks of pixels after DCT
                    0xF4, 0x7D, 0x3E, 0x21,    # DCT-compressed block
                    0x1A, 0x0B, 0x05, 0x02,    # Run-length encoded
                    0x01, 0x00, 0x00, 0x01,    # Huffman encoded
                    # ...more compressed image data...
                    
                    0xFF, 0xD9      # End of Image (EOI)
                ] """


                # curr_odomdata: deserialize
                curr_odomdata = process_odom_data(curr_odomdata)
                synced_odomdata.append(curr_odomdata)

                start_time = timestamp

    # 전처리
    img_data = process_images(synced_imdata, img_process_func)
    traj_data = process_odom(
        synced_odomdata,
        odom_process_func,
        ang_offset=ang_offset,
    )

    return img_data, traj_data


def is_backwards(
    pos1: np.ndarray, yaw1: float, pos2: np.ndarray, eps: float = 1e-5
) -> bool:
    """
    Check if the trajectory is going backwards given the position and yaw of two points
    Args:
        pos1: position of the first point

    """
    dx, dy = pos2 - pos1
    return dx * np.cos(yaw1) + dy * np.sin(yaw1) < eps


# cut out non-positive velocity segments of the trajectory
def filter_backwards(
    img_list: List[Image.Image],
    traj_data: Dict[str, np.ndarray],
    start_slack: int = 0,
    end_slack: int = 0,
) -> Tuple[List[np.ndarray], List[int]]:
    """
    Cut out non-positive velocity segments of the trajectory
    Args:
        traj_type: type of trajectory to cut
        img_list: list of images
        traj_data: dictionary of position and yaw data
        start_slack: number of points to ignore at the start of the trajectory
        end_slack: number of points to ignore at the end of the trajectory
    Returns:
        cut_trajs: list of cut trajectories
        start_times: list of start times of the cut trajectories
    """
    traj_pos = traj_data["position"]
    traj_yaws = traj_data["yaw"]
    cut_trajs = []
    start = True

    def process_pair(traj_pair: list) -> Tuple[List, Dict]:
        new_img_list, new_traj_data = zip(*traj_pair)
        new_traj_data = np.array(new_traj_data)
        new_traj_pos = new_traj_data[:, :2]
        new_traj_yaws = new_traj_data[:, 2]
        return (new_img_list, {"position": new_traj_pos, "yaw": new_traj_yaws})

    for i in range(max(start_slack, 1), len(traj_pos) - end_slack):
        pos1 = traj_pos[i - 1]
        yaw1 = traj_yaws[i - 1]
        pos2 = traj_pos[i]
        if not is_backwards(pos1, yaw1, pos2):
            if start:
                new_traj_pairs = [
                    (img_list[i - 1], [*traj_pos[i - 1], traj_yaws[i - 1]])
                ]
                start = False
            elif i == len(traj_pos) - end_slack - 1:
                cut_trajs.append(process_pair(new_traj_pairs))
            else:
                new_traj_pairs.append(
                    (img_list[i - 1], [*traj_pos[i - 1], traj_yaws[i - 1]])
                )
        elif not start:
            cut_trajs.append(process_pair(new_traj_pairs))
            start = True
    return cut_trajs


def quat_to_yaw(
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    w: np.ndarray,
) -> np.ndarray:
    """
    Convert a batch quaternion into a yaw angle
    yaw is rotation around z in radians (counterclockwise)
    """
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)
    return yaw

# aggregate를 진행하면 process_tartan_img에서 img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) 할 때 shape(1,H,W)여서 에러남
def ros_to_numpy(
    msg, nchannels=3, empty_value=None, output_resolution=None, aggregate="none"
):
    """
    Convert a ROS image message to a numpy array
    """
    if output_resolution is None:
        output_resolution = (msg.width, msg.height)

    is_rgb = "8" in msg.encoding
    if is_rgb:
        data = np.frombuffer(msg.data, dtype=np.uint8).copy()
    else:
        data = np.frombuffer(msg.data, dtype=np.float32).copy()

    data = data.reshape(msg.height, msg.width, nchannels)

    # data의 큰 값으로 빈 곳 채움
    if empty_value:
        mask = np.isclose(abs(data), empty_value)
        fill_value = np.percentile(data[~mask], 99)
        data[mask] = fill_value

    data = cv2.resize(
        data,
        dsize=(output_resolution[0], output_resolution[1]),
        interpolation=cv2.INTER_AREA,
    )

    # shape becomes (height, width). Each pixel : channel0 * 256⁰ + channel1 * 256¹ + channel2 * 256²
    if aggregate == "littleendian":
        data = sum([data[:, :, i] * (256**i) for i in range(nchannels)])
    elif aggregate == "bigendian":
        data = sum([data[:, :, -(i + 1)] * (256**i) for i in range(nchannels)])

    # aggregate 따라 위 과정 했는지 따라 if문 결정. 기능은 똑같이 채널을 앞으로.
    # 어차피 process_tartan_img()에서 원상복구할 건데 왜 하는지 모르겠음
    if len(data.shape) == 2:
        data = np.expand_dims(data, axis=0)
    else:
        data = np.moveaxis(data, 2, 0)  # Switch to channels-first

    # 0~1 사이로 정규화 
    if is_rgb:
        data = data.astype(np.float32) / (
            255.0 if aggregate == "none" else 255.0**nchannels
        )

    # 0~1 float32 aggregate 여부 따라 (C,H,W)에서 C= 3 or 1
    # 근데 이 함수 위에 주석으로 썼듯 어차피 현재 코드에선 C=3만 가능
    return data
