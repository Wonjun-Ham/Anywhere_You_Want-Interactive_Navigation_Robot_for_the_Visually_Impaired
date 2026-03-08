# Anywhere-You-Want: Interactive Navigation Robot for the Visually Impaired

## 개요
시각장애인 분들은 낯선 곳을 혼자 찾아가는 게 불가능합니다.

여태까지의 주행 보조도구들은, 장애물 회피를 돕는 용도가 전부입니다. 길안내 기능은 사실상 도움되지 않을 만큼 매우 제한적입니다.

이 프로젝트에서는 2024 ICRA best paper NoMaD를 변형해서, 길찾기 앱 화면과 전방 카메라 데이터를 이용해 실시간으로 주행을 결정하는 AI를 설계했습니다.  

이를 통해 사용자가 로봇을 잡고 따라가기만 하면 어떤 목적지든 도착할 수 있도록 로봇을 제작하고 있습니다.

NoMaD는 아래와 같은 논문입니다.

<img width="1239" height="437" alt="image" src="https://github.com/user-attachments/assets/41fdf2eb-07a0-4dd2-a1bc-cf2f11a3b3dc" />
<img width="1938" height="528" alt="image" src="https://github.com/user-attachments/assets/7db065a4-8352-4790-adf1-2770f1eb043a" />


이 프로젝트에선 goal 부분을 실시간 길찾기 앱 화면으로 대신합니다.

<img width="358" height="222" alt="image" src="https://github.com/user-attachments/assets/76fcdecd-8576-4c66-827c-4394c30942df" />

<br><br>


## 디렉토리 설명

nomad_original 디렉토리는, 논문에서 공개된 원본 코드입니다. <br>
nomad_etri 디렉토리는, 교수님께서 제공하신 코드로, ETRI에서 ROS2로 업데이트하면서 수정한 코드입니다.<br>
nomad_wonjun 디렉토리는, 제가 nomad_etri를 수정하여 만든 코드입니다.<br>

android_screen_mirror 디렉토리는, 스마트폰 미러링을 수행한 코드입니다. 따로 README.md 파일이 없어서, 이에 대한 설명을 아래에 작성하겠습니다. 

해당 디렉토리는 다음과 같이 구성돼있습니다.

<img width="733" height="605" alt="image" src="https://github.com/user-attachments/assets/054bbb1f-d4a4-4bfa-b262-03198ad8eef1" />

<img width="733" height="483" alt="image" src="https://github.com/user-attachments/assets/bccd6bd2-68e0-4ece-b486-af4a318b5717" />


github에는 src 폴더 부분만 upload 했습니다. 

각 부분을 설명하자면,
- Build, install, log는 프로젝트 빌드 후 자동적으로 생성된 폴더들입니다.
- screenshots_pub폴더는, publish하는 노드에서 안드로이드폰 스크린을 캡처한 직후, 캡처 이미지를 해당 폴더에 저장하도록 하여, 캡처가 잘 됐는지 확인해볼 때 사용한 폴더입니다.
- screenshots_sub폴더는, subscribe 노드에서 ros msg를 subscribe하고 pil image로 전환한 후 그 이미지를 해당 폴더에 저장하도록 하여, 잘 수신했는지 확인해볼 때 사용한 폴더입니다.

Src 폴더에 있는 파일들의 경우,
- screen_mirror_node.py : 실시간으로 캡처하는 스크린샷을 지속적으로 publish하는 노드
- subscribe_node.py : screen_mirror_node.py에서 publish하는 토픽을 subscribe 하는 노드
- utils.py : Rate 클래스, msg_to_pil 함수 작성돼있음. Rate 클래스는 screen_mirror_node.py에서 쓰여, 캡처 주기보다 빨리 캡처가 끝나면 주기가 될 때까지 time.sleep()하고 있도록 함.  Msg_to_pil은 subscribe_node.py에서 ros msg를 pil image로 전환할 때 사용.
- screenshots.sh : nomad에서 .sh 파일만 실행하면 관련된 태스크를 수행할 수 있도록 정리해둔 것처럼, 이 파일만 터미널에서 실행하면 이 프로젝트 실행 가능. tmux를 통해 screen_mirror_node.py와 subscribe_node.py가 각각의 패널에서 실행 시작
- resource 폴더는 빌드시 관련한 warning이 떠서 생성해둔 빈 폴더
- package.xml, setup.cfg, setup.py는 설정 파일들
<br>

## 본인 기여 사항

### 논문을 이해한 후 팀원들에게 overview를 진행했습니다.

### 프로젝트 세부사항 기획에 기여했습니다.
1. 로봇 형태 및 기능 
   - 시장조사를 통해, 스탠포드 대학교의 지팡이(옴니휠을 통해 사용자의 좌우 움직임을 필요로 하며 주행 정확도가 떨어지는 단점 존재)를 대신한, glide를 발견하여 해당 형태로의 제작 제안
2. AI 모델 구조 
   - 기존 모델에서 추가해야할 부분, 제거해야할 부분 파악
   - rgb 카메라 대신 depth 카메라 또는 lidar를 활용한 개선 방안 제안
3. 길찾기 앱 정보 추출 방식 기획
   - API보다 직관적인 스마트폰 미러링 방식 제안
<br>  <br>

### 논문의 코드를 심층적으로 파악했습니다. 
1.프로젝트 내 모든 파일들의 역할 파악 -> 2.수정 필요할 수 있는 파일 파악 -> 3.파일 내 관련 함수들은 라인 단위로 이해
위 과정을 통해 단계적으로 효율적인 이해를 하고자 노력했습니다.
  > 후에 참고하기 위해 주석을 달면서 진행했습니다. (nomad_etri 폴더와 비교해서 nomad_wonjun 폴더에 추가된 주석들이 이에 해당합니다.)
- 파악한 내용을 팀원들에게 전반적으로 설명해주고, 각 팀원이 어떤 파일들의 어떤 부분들을 수정하면 되는지 안내했습니다.

> 예를 들어, 아래와 같이 vint_dataset.py에 있는 action mask의 역할을 파악하고, 현 프로젝트에서는 제거해야할 대상임을 파악했습니다.  
```python
# action mask는 train_utils.py의 loss 계산에서 사용됨
# goal이 적당한 거리 떨어져 있을 때로 학습시키기 위해 그 외의 경우에는 학습 안 하도록 이 경우 loss 0으로 만들어버리는 mask
# 너무 먼 목표: 정확한 경로를 예측하는 건 비현실적
# 너무 가까운 목표: 바로 앞 목표는 학습 의미가 적음
# Negative goal: 현재 궤적과 무관한 목표에 대한 action은 학습하면 안 됨
action_mask = (
    (distance < self.max_action_distance) and
    (distance > self.min_action_distance) and
    (not goal_is_negative)
)
```

### 논문의 설명에 따라 상상되는 아키텍처와, 실제 코드 사이 괴리가 큰 부분들 찾아낼 수 있었습니다.<br>

- 실제 코드를 보니, 논문에서는 더 많은 기능을 구현한 것처럼 포장해서 설명된 부분이 있었습니다. 
논문에서는 topomap을 지점별 이미지들을 노드로 하는 그래프처럼 설명하지만,실제 코드상에서는 처음 해당 지역을 사람의 지도를 받아 explore 해볼 때 일정한 주기로 순서대로 찍은 사진들의 리스트에 불과했습니다.

- navigate시 거리 예측 모델에 모든 topomap 노드들을 입력해서, 현위치에서 가장 가까운 노드를 찾아낸 후에는, 이를 subgoal로 삼고 향하고, 거의 다다르면 topomap상 다음 노드를 subgoal로 삼아 진행하는 식으로, explore할 때 갔던 경로 그대로를 따라갈 뿐입니다. 따라서 목적지에 최단거리로 도착할 수 있는 기능은 없습니다.

- 한편, 논문에서 말하는 temporal distance는, 실제 시간적 개념이 아니라, '현재로부터 n번째만큼 떨어진 노드'의 개념으로 학습되고 있었습니다.
```python
# distance를 실제 시간적 개념이 아니라, 얼마나 떨어진 노드냐의 개념으로 학습
# Compute distances
if goal_is_negative:
    distance = self.max_dist_cat
else:
    distance = (goal_time - curr_time) // self.waypoint_spacing
    assert (goal_time - curr_time) % self.waypoint_spacing == 0, f"{goal_time} and {curr_time} should be separated by an integer multiple of {self.waypoint_spacing}"
```
  
<br><br>




### 논문 내 주요 개선점을 발견했습니다.
  
  논문에서는, 다양한 로봇의 주행 데이터를 수집해 ai를 학습시킵니다.<br>
  그런데, 로봇마다 카메라가 다르기 때문에 수집한 이미지들의 배율, 시야각이 제각각입니다.<br>
  전처리를 통해 통일해주지 않으면, 학습하는 동안 동일한 물체의 크기 및 형태가 계속 바뀌어보이는 것과 같습니다.<br>
  한쪽 눈만 보이는 사람이 돌아다니는 와중에 물체의 크기/형태가 계속 달라져 보이면 거리 감각을 가질 수 없는 것과 마찬가지입니다.<br>
  로봇의 주행 학습 능력은 크게 저하될 수밖에 없습니다.<br>
  논문의 코드를 확인한 결과, 이에 대한 처리가 전무했습니다.<br>

  여러 로봇에서 수집한 이미지를 train에 사용하고자 전처리하는 부분은 두 군데가 있습니다. 
<br>
  1. train\vint_train\process_data\process_data_utils.py <br><br>
    각 dataset별로 함수들이 존재하며, ros msg -> image로 형태를 변환하는 과정이 진행됩니다. 
    이때 각 함수들에서 이미지 크기 및 형태에 대한 처리는 나타나지 않습니다.
<br><br>
  1. train\vint_train\data\data_utils.py <br><br>
   이미지 크기 및 형태를 처리하는 ```resize_and_aspect_crop``` 함수가 존재합니다. 이 함수가 어떻게 사용되는지 살펴보기 위해 사용처를 파악해봤습니다. <br>
   이 함수는 같은 파일 내 ```img_path_to_data``` 함수에서만 사용되는 함수입니다.<br>
   ```img_path_to_data``` 함수는 train\vint_train\data\vint_dataset.py의 ```ViNT_Dataset``` 클래스 내 함수에서만 사용됩니다.<br>
   또한, ```ViNT_Dataset``` 클래스는 train.py에서만 사용됩니다. <br>
   train.py에서 ```ViNT_Dataset이``` 사용된 부분을 살펴보면, ```resize_and_aspect_crop``` 함수와 관련된 부분은 ```image_size``` 매개변수 뿐입니다. <br>
   이 매개변수에 dataset별로 다른 값을 입력받는다면, dataset들의 배율, 시야각을 통일했을 것으로 추측할 수 있습니다. <br>
   값은 config 폴더 내 nomad.yaml에서 불러오는 형태입니다. <br>
   nomad.yaml 파일을 살펴보면, 모든 dataset이 ```image_size: [96, 96]``` 으로 동일한 형태를 사용하고 있습니다. <br>
   <br>
   따라서, dataset별로 이미지 크기 및 형태에 대한 처리가 전혀 이루어지지 않고 있음을 알 수 있습니다.
  
<br><br>

### 스마트폰 미러링 노드를 제작했습니다. 
#### 캡처 주기 단축을 위해 멀티스레드를 사용했고 속도가 약 2배 증가했습니다.

 길찾기앱 화면을 실시간으로 추출하기 위해 제작했습니다.
 스크린샷 캡처와 publish를 주기적으로 수행하는 node(ROS2)와, publish하는 토픽을 subscribe 하는 node를 제작했습니다.
 publish 주기는 평균 2초 중반대로 나타났습니다.

 수행 속도를 높이기 위해 멀티스레드 방식을 도입했습니다.
 Capture thread에서는 화면 캡처, openCV2 포맷으로 변환, current_frame 변수를 해당 이미지로 업데이트하는 과정을 반복합니다.
 timer thread에서는, current_frame의 이미지를 ROS image message 포맷으로 변환하고 publish하는 과정을 반복합니다. 
 위 두 thread는, current_frame 변수에 동시에 접근할 수 있으므로, frame_lock 변수를 mutex로 사용하여 동기화했습니다.
 이후, publish 주기는 평균 1초 초반대로 감소했습니다.
 
<img width="940" height="107" alt="image" src="https://github.com/user-attachments/assets/c0900f3d-ddc1-4935-936e-fd9eceb25fba" />

<img width="940" height="497" alt="image" src="https://github.com/user-attachments/assets/2dfb1ede-d1b6-41cb-8a9f-8ed291f87d5c" />

<br><br>

### ETRI에서 수정했던 코드에서 버그를 찾아 수정했습니다.

ETRI에서는, ROS1이 사용된 기존의 코드를 ROS2가 사용되도록 업데이트하는 작업을 진행했습니다. 그 중 아래와 같은 버그들을 수정했습니다.
  
  #### 1. get_images_and_odom 함수
  sensor_msgs/CompressedImage만 에러가 나지 않을 형태로 만들어져 sensor_msgs/Image에 대해선 에러가 나는 형태였습니다. 따라서 두 형태 모두에 대해 에러가 나지 않도록 수정했습니다. 아래는 프로젝트 내 코드를 가져온 모습입니다.

  ```python
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
```

  
  #### 2. 카메라 pixel format
  https://github.com/ros-drivers/usb_cam/blob/develop/src/camera_driver.cpp  
  https://ros-drivers.github.io/usb_cam/main/ 
  두 문서를 참고하여 camera_front.yaml에서 ros2에 맞게 pixel_format: "yuyv2rgb"으로 수정했습니다.
   
 
  #### 3. 리팩토링
 ETRI에서는 Rate 클래스를 제작하여 create_topomap.py, explore.py, navigate.py, joy_teleop.py, pd_controller.py에 추가했습니다. 그런데, 똑같은 클래스 정의가 위 다섯 개 파일 각각에 작성된 상태였습니다. 중복 작성을 없애고자,
 utils.py에 해당 클래스를 정의해두고 각 파일들에서 import해서 쓰도록 수정했습니다.

<br>

### msg_to_pil 함수의 기능을 확장했습니다.
프로젝트 내에서는 다양한 로봇 데이터를 사용하기 때문에 rgb 8bit 뿐만 아니라, bgr, rgba, 16bit, 흑백 등 다양한 형식을 처리해 형식을 통일해야 합니다. 원 코드에서는 일부 형식만을 처리할 수 있는 형태로 작성돼있기 때문에, 모든 형식을 고려하도록 아래와 같은 형태로 함수의 기능을 확장했습니다.
  ```python
  def msg_to_pil(msg: Image) -> PILImage.Image:
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
    
```
