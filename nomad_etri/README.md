# General Navigation Models: GNM, ViNT and NoMaD

[Project Page](https://general-navigation-models.github.io) | [Citing](https://github.com/robodhruv/visualnav-transformer#citing) | [Pre-Trained Models](https://drive.google.com/drive/folders/1a9yWR2iooXFAqjQHetz263--4_2FFggg?usp=sharing)

---

General Navigation Models are general-purpose goal-conditioned visual navigation policies trained on diverse, cross-embodiment training data, and can control many different robots in zero-shot. They can also be efficiently fine-tuned, or adapted, to new robots and downstream tasks. Our family of models is described in the following research papers (and growing):
1. [GNM: A General Navigation Model to Drive Any Robot](https://sites.google.com/view/drive-any-robot) (_October 2022_, presented at ICRA 2023)
2. [ViNT: A Foundation Model for Visual Navigation](https://general-navigation-models.github.io/vint/index.html) (_June 2023_, presented at CoRL 2023)
3. [NoMaD: Goal Masking Diffusion Policies for Navigation and Exploration](https://general-navigation-models.github.io/nomad/index.html) (_October 2023_)

## Overview
This repository contains code for training our family of models with your own data, pre-trained model checkpoints, as well as example code to deploy it on a TurtleBot2/LoCoBot robot. The repository follows the organization from [GNM](https://github.com/PrieureDeSion/drive-any-robot).

- `./train/train.py`: training script to train or fine-tune the ViNT model on your custom data.
- `./train/vint_train/models/`: contains model files for GNM, ViNT, and some baselines.
- `./train/process_*.py`: scripts to process rosbags or other formats of robot trajectories into training data.
- `./deployment/src/record_bag.sh`: script to collect a demo trajectory as a ROS bag in the target environment on the robot. This trajectory is subsampled to generate a topological graph of the environment.
- `./deployment/src/create_topomap.sh`: script to convert a ROS bag of a demo trajectory into a topological graph that the robot can use to navigate.
- `./deployment/src/navigate.sh`: script that deploys a trained GNM/ViNT/NoMaD model on the robot to navigate to a desired goal in the generated topological graph. Please see relevant sections below for configuration settings.
- `./deployment/src/explore.sh`: script that deploys a trained NoMaD model on the robot to randomly explore its environment. Please see relevant sections below for configuration settings.
## Modifications

기존 패키지를 ROS2-humble 버전으로 변환하면서 생긴 수정사항과, 기존 코드에서 추가적으로 개선시킨 기술에대한 내용이다. 

### ./train/process_bags.py

ROS1과 ROS2 bag 파일의 확장자는 각각 '.bag'(ROS1), '.db3'(ROS2) 로 다르기 때문에 bag 파일을 검색하는 과정에서 파일 확장자를 ROS2 버전에 맞게 수정하였다. 

ROS1 버전에서는 'rosbag' 라이브러리를 사용하여 rosbag.Bag' 클래스를 통해 파일을 여는 반면, ROS2 버전에서의 bag 파일은 다양한 설정 옵션을 제공하기 때문에 [rosbag2](https://github.com/ros2/rosbag2) 라이브러리를 사용하여 보다 세밀하게 파일에 접근해야 한다.
기존 process_bags.py 코드에서는 'rosbag.Bag' 클래스를 통해 파일을 열 수 있었지만, 수정된 코드에서는 'StorageOptions'와 'ConverterOption' 클래스를 사용하여 읽을 bag 파일의 위치 및 저장소 플러그인, 데이터의 직렬화 및 역직렬화 형식을 지정하한다. 또한 'SequentialReader' 클래스를 사용하여 설정된 옵션을 통해 bag 파일을 연다.

### ./train/vint_train/process_data/process_data_utils.py

ROS 버전에 따라 bag 파일의 데이터를 처리하는 과정이 다르다. ROS2 bag 파일은 메세지나, 메타데이터 등과 같은 여러 데이터베이스 테이블로 구성되어있는 binary file로 저장된다. 그렇기 때문에 binary file에서 원하는 데이터(JPEG, Odom)만 추출하여 원래의 데이터 구조로 복원하는 역직렬화(Deserialization) 과정이 필요하다.
이를 위해 binary file 에서 JPEG 데이터에 해당하는 부분을 추출한 후 기존의 이미지 형태로 변환하여 image dataset을 구축하였으며, [rclpy.serialization](https://docs.ros.org/en/rolling/p/rclpy/rclpy.serialization.html) 라이브러리의 'deserialize_message' 함수를 사용하여 Odometry data 데이터를 추출한 후 trajectory dataset을 구출하였다.

### ./deployment/src/process_topomap.py

해당 패키지에서의 TOPOMAP 생성 코드(./deployment/src/create_topomap.py)는 임의로 정한 RATE에 따라 받아오는 current image를 해당 디렉토리(./deployment/topomaps/images/topomap)에 저장한다. 생성한 TOPOMAP을 확인해 본 결과, 직진 경로가 긴 환경(비슷한 환경)에서는 유사한 이미지가 연속해서 저장되는 상황이 발생하였다. 이는 navigation 과정에서 Current Observation과 유사한 subgoal image 후보군들만 탐색하여 temperal distance를 산출하는 문제점의 원인이 되었고, 이로 인해 선택한 subgoal image가 이전 경로의 image로 되돌아가는 결과를 초래하였다.

이를 해결하기 위해 생성한 TOPOPMAP image들 간에 유사도를 계산하여, 결과값이 일정 임계값 이하의 값을 갖는 이미지들만 재저장하는 후처리 코드를 작성하였다.(저장 디렉토리: ./deployment/topomaps/images/process_topomap) 유사도 측정은 이미지의 구조, 밝기, 대비 등을 고려하는 [SSIM](https://scikit-image.org/docs/stable/auto_examples/transform/plot_ssim.html)(구조적 유사도) 알고리즘을 사용하였다.

### ./deployment/src/navigate.py

기존 navigation은 로봇의 시작 위치가 TOPOMAP의 0번 index에 해당하는 image에 위치해 있다고 가정하였다. 0번 index image를 기준으로 TOPOMAP에서 주변 subgoal image들을 가져오는데, 이는 시작 위치가 약간이라도 0번 image에서 벗어나면 subgoal image를 찾지 못하는 문제의 원인이 되었다. 

이를 해결하기 위해 처음 subgoal image들에 대한 temporal distance를 계산할 때, TOPOMAP image 0번 부터 goal node image까지의 image들에 대한 temporal distance를 찾고 기존 navigation 기능을 이어가게 하였다.

## Train

This subfolder contains code for processing datasets and training models from your own data.

### Pre-requisites

The codebase assumes access to a workstation running Ubuntu (tested on 22.04), Python 3.10, and a GPU with CUDA 11.8. It also assumes access to conda, but you can modify it to work with other virtual environment packages, or a native setup.
(Ubuntu 22.04, ros2-humble, Python 3.10, GPU with CUDA 11.8, Pytorch 2.3.1, cuDNN 8.9.0)

### Setup
Run the commands below inside the `vint_release/` (topmost) directory:

1. Source the virtual environment:
    ```bash
    workon nomad
    ```
2. Set up the vtrtual environment:
    ```bash
    pip install torchvision matplotlib ipykernel numpy tqdm==4.64.0 git+https://github.com/ildoonet/pytorch-gradual-warmup-lr.git opencv-python==4.6.0.66 h5py==3.6.0 wandb==0.12.18 --extra-index-url https://rospypi.github.io/simple/  prettytable efficientnet-pytorch warmup-scheduler diffusers==0.11.1 lmdb vit-pytorch positional-encodings pyyaml scikit-image --upgrade wandb
    ```
3. Install the vint_train packages:
    ```bash
    pip install -e train/
    ```
4. Install the `diffusion_policy` package from this [repo](https://github.com/real-stanford/diffusion_policy):
    ```bash
    git clone https://github.com/real-stanford/diffusion_policy.git
    pip install -e diffusion_policy/
    ```


### Data-Wrangling
In the [papers](https://general-navigation-models.github.io), we train on a combination of publicly available and unreleased datasets. Below is a list of publicly available datasets used for training; please contact the respective authors for access to the unreleased data.
- [RECON](https://sites.google.com/view/recon-robot/dataset)
- [TartanDrive](https://github.com/castacks/tartan_drive)
- [SCAND](https://www.cs.utexas.edu/~xiao/SCAND/SCAND.html#Links)
- [GoStanford2 (Modified)](https://drive.google.com/drive/folders/1xrNvMl5q92oWed99noOt_UhqQnceJYV0?usp=share_link)
- [SACSoN/HuRoN](https://sites.google.com/view/sacson-review/huron-dataset)

We recommend you to download these (and any other datasets you may want to train on) and run the processing steps below.

#### Data Processing 

We provide some sample scripts to process these datasets, either directly from a rosbag or from a custom format like HDF5s:
1. Run `process_bags.py` with the relevant args, or `process_recon.py` for processing RECON HDF5s. You can also manually add your own dataset by following our structure below (if you are adding a custom dataset, please checkout the [Custom Datasets](#custom-datasets) section).
2. Run `data_split.py` on your dataset folder with the relevant args.

After step 1 of data processing, the processed dataset should have the following structure:

```
├── <dataset_name>
│   ├── <name_of_traj1>
│   │   ├── 0.jpg
│   │   ├── 1.jpg
│   │   ├── ...
│   │   ├── T_1.jpg
│   │   └── traj_data.pkl
│   ├── <name_of_traj2>
│   │   ├── 0.jpg
│   │   ├── 1.jpg
│   │   ├── ...
│   │   ├── T_2.jpg
│   │   └── traj_data.pkl
│   ...
└── └── <name_of_trajN>
    	├── 0.jpg
    	├── 1.jpg
    	├── ...
        ├── T_N.jpg
        └── traj_data.pkl
```  

Each `*.jpg` file contains an forward-facing RGB observation from the robot, and they are temporally labeled. The `traj_data.pkl` file is the odometry data for the trajectory. It’s a pickled dictionary with the keys:
- `"position"`: An np.ndarray [T, 2] of the xy-coordinates of the robot at each image observation.
- `"yaw"`: An np.ndarray [T,] of the yaws of the robot at each image observation.


After step 2 of data processing, the processed data-split should the following structure inside `vint_release/train/vint_train/data/data_splits/`:

```
├── <dataset_name>
│   ├── train
|   |   └── traj_names.txt
└── └── test
        └── traj_names.txt 
``` 

### Training your General Navigation Models
Run this inside the `vint_release/train` directory:
```bash
python train.py -c <path_of_train_config_file>
```
The premade config yaml files are in the `train/config` directory. 

#### Custom Config Files
You can use one of the premade yaml files as a starting point and change the values as you need. `config/vint.yaml` is good choice since it has commented arguments. `config/defaults.yaml` contains the default config values (don't directly train with this config file since it does not specify any datasets for training).

#### Custom Datasets
Make sure your dataset and data-split directory follows the structures provided in the [Data Processing](#data-processing) section. Locate `train/vint_train/data/data_config.yaml` and append the following:

```
<dataset_name>:
    metric_waypoints_distance: <average_distance_in_meters_between_waypoints_in_the_dataset>
```

Locate your training config file and add the following text under the `datasets` argument (feel free to change the values of `end_slack`, `goals_per_obs`, and `negative_mining`):
```
<dataset_name>:
    data_folder: <path_to_the_dataset>
    train: data/data_splits/<dataset_name>/train/ 
    test: data/data_splits/<dataset_name>/test/ 
    end_slack: 0 # how many timesteps to cut off from the end of each trajectory  (in case many trajectories end in collisions)
    goals_per_obs: 1 # how many goals are sampled per observation
    negative_mining: True # negative mining from the ViNG paper (Shah et al.)
```

#### Training your model from a checkpoint
Instead of training from scratch, you can also load an existing checkpoint from the published results.
Add `load_run: <project_name>/<log_run_name>`to your .yaml config file in `vint_release/train/config/`. The `*.pth` of the file you are loading to be saved in this file structure and renamed to “latest”: `vint_release/train/logs/<project_name>/<log_run_name>/latest.pth`. This makes it easy to train from the checkpoint of a previous run since logs are saved this way by default. Note: if you are loading a checkpoint from a previous run, check for the name the run in the `vint_release/train/logs/<project_name>/`, since the code appends a string of the date to each run_name specified in the config yaml file of the run to avoid duplicate run names. 


If you want to use our checkpoints, you can download the `*.pth` files from [this link](https://drive.google.com/drive/folders/1a9yWR2iooXFAqjQHetz263--4_2FFggg?usp=sharing).


## Deployment
This subfolder contains code to load a pre-trained ViNT and deploy it on the open-source [LoCoBot indoor robot platform](http://www.locobot.org/) with a [NVIDIA Jetson Orin Nano](https://www.amazon.com/NVIDIA-Jetson-Orin-Nano-Developer/dp/B0BZJTQ5YP/ref=asc_df_B0BZJTQ5YP/?tag=hyprod-20&linkCode=df0&hvadid=652427572954&hvpos=&hvnetw=g&hvrand=12520404772764575478&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=1013585&hvtargid=pla-2112361227514&psc=1&gclid=CjwKCAjw4P6oBhBsEiwAKYVkq7dqJEwEPz0K-H33oN7MzjO0hnGcAJDkx2RdT43XZHdSWLWHKDrODhoCmnoQAvD_BwE). It can be easily adapted to be run on alternate robots, and researchers have been able to independently deploy it on the following robots – Clearpath Jackal, DJI Tello, Unitree A1, TurtleBot2, Vizbot – and in simulated environments like CARLA.


#### Software Installation (in this order)
1. ROS: [ros-humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
2. (Recommended) Install [tmux](https://github.com/tmux/tmux/wiki/Installing) if not present.
    Many of the bash scripts rely on tmux to launch multiple screens with different commands. This will be useful for debugging because you can see the output of each screen.

#### Hardware Requirements
- LoCoBot: http://locobot.org (just the navigation stack)
- A wide-angle RGB camera: [Example](https://www.amazon.com/ELP-170degree-Fisheye-640x480-Resolution/dp/B00VTHD17W). The `vint_locobot.launch` file uses camera parameters that work with cameras like the ELP fisheye wide angle, feel free to modify to your own. Adjust the camera parameters in `vint_release/deployment/config/camera.yaml` your camera accordingly (used for visualization).
- [Joystick](https://www.amazon.com/Logitech-Wireless-Nano-Receiver-Controller-Vibration/dp/B0041RR0TW)/[keyboard teleop](http://wiki.ros.org/teleop_twist_keyboard) that works with Linux. Add the index mapping for the _deadman_switch_ on the joystick to the `vint_release/deployment/config/joystick.yaml`. You can find the mapping from buttons to indices for common joysticks in the [wiki](https://wiki.ros.org/joy). 


### Loading the model weights

Save the model weights *.pth file in `vint_release/deployment/model_weights` folder. Our model's weights are in [this link](https://drive.google.com/drive/folders/1a9yWR2iooXFAqjQHetz263--4_2FFggg?usp=sharing).

### Collecting a Topological Map

_Make sure to run these scripts inside the `vint_release/deployment/src/` directory._


This section discusses a simple way to create a topological map of the target environment for deployment. For simplicity, we will use the robot in “path-following” mode, i.e. given a single trajectory in an environment, the task is to follow the same trajectory to the goal. The environment may have new/dynamic obstacles, lighting variations etc.

#### Record the rosbag: 
```bash
./record_bag.sh <bag_name>
```

Run this command to teleoperate the robot with the joystick and camera. This command opens up three windows 
1. `ros2 launch vint_locobot.launch`: This launch file opens the `usb_cam` node for the camera, the joy node for the joystick, and nodes for the robot’s mobile base.
2. `python joy_teleop.py`: This python script starts a node that reads inputs from the joy topic and outputs them on topics that teleoperate the robot’s base.
3. `ros2 bag record /usb_cam/image_raw -o <bag_name>`: This command isn’t run immediately (you have to click Enter). It will be run in the vint_release/deployment/topomaps/bags directory, where we recommend you store your rosbags.

Once you are ready to record the bag, run the `rosbag record` script and teleoperate the robot on the map you want the robot to follow. When you are finished with recording the path, kill the `rosbag record` command, and then kill the tmux session.

#### Make the topological map: 
```bash
./create_topomap.sh <topomap_name> <bag_filename>
```

This command opens up 3 windows:
1. `python create_topomap.py —dt 1 —dir <topomap_dir>`: This command creates a directory in `/vint_release/deployment/topomaps/images` and saves an image as a node in the map every second the bag is played.
2. `ros2 bag play -r 1.5 <bag_filename>`: This command plays the rosbag at x5 speed, so the python script is actually recording nodes 1.5 seconds apart. The `<bag_filename>` should be the entire bag name with the .bag extension. You can change this value in the `make_topomap.sh` file. The command does not run until you hit Enter, which you should only do once the python script gives its waiting message. Once you play the bag, move to the screen where the python script is running so you can kill it when the rosbag stops playing.

When the bag stops playing, kill the tmux session.


### Running the model 
#### Navigation
_Make sure to run this script inside the `vint_release/deployment/src/` directory._

```bash
./navigate.sh “--model <model_name> --dir <topomap_dir>”
```

To deploy one of the models from the published results, we are releasing model checkpoints that you can download from [this link](https://drive.google.com/drive/folders/1a9yWR2iooXFAqjQHetz263--4_2FFggg?usp=sharing).


The `<model_name>` is the name of the model in the `vint_release/deployment/config/models.yaml` file. In this file, you specify these parameters of the model for each model (defaults used):
- `config_path` (str): path of the *.yaml file in `vint_release/train/config/` used to train the model
- `ckpt_path` (str): path of the *.pth file in `vint_release/deployment/model_weights/`


Make sure these configurations match what you used to train the model. The configurations for the models we provided the weights for are provided in yaml file for your reference.

The `<topomap_dir>` is the name of the directory in `vint_release/deployment/topomaps/images` that has the images corresponding to the nodes in the topological map. The images are ordered by name from 0 to N.

This command opens up 4 windows:

1. `ros2 launch vint_locobot.launch`: This launch file opens the usb_cam node for the camera, the joy node for the joystick, and several nodes for the robot’s mobile base).
2. `python navigate.py --model <model_name> -—dir <topomap_dir>`: This python script starts a node that reads in image observations from the `/usb_cam/image_raw` topic, inputs the observations and the map into the model, and publishes actions to the `/waypoint` topic.
3. `python joy_teleop.py`: This python script starts a node that reads inputs from the joy topic and outputs them on topics that teleoperate the robot’s base.
4. `python pd_controller.py`: This python script starts a node that reads messages from the `/waypoint` topic (waypoints from the model) and outputs velocities to navigate the robot’s base.

When the robot is finishing navigating, kill the `pd_controller.py` script, and then kill the tmux session. If you want to take control of the robot while it is navigating, the `joy_teleop.py` script allows you to do so with the joystick.

#### Exploration
_Make sure to run this script inside the `vint_release/deployment/src/` directory._

```bash
./exploration.sh “--model <model_name>”
```

To deploy one of the models from the published results, we are releasing model checkpoints that you can download from [this link](https://drive.google.com/drive/folders/1a9yWR2iooXFAqjQHetz263--4_2FFggg?usp=sharing).


The `<model_name>` is the name of the model in the `vint_release/deployment/config/models.yaml` file (note that only NoMaD works for exploration). In this file, you specify these parameters of the model for each model (defaults used):
- `config_path` (str): path of the *.yaml file in `vint_release/train/config/` used to train the model
- `ckpt_path` (str): path of the *.pth file in `vint_release/deployment/model_weights/`


Make sure these configurations match what you used to train the model. The configurations for the models we provided the weights for are provided in yaml file for your reference.

The `<topomap_dir>` is the name of the directory in `vint_release/deployment/topomaps/images` that has the images corresponding to the nodes in the topological map. The images are ordered by name from 0 to N.

This command opens up 4 windows:

1. `ros2 launch vint_locobot.launch`: This launch file opens the usb_cam node for the camera, the joy node for the joystick, and several nodes for the robot’s mobile base.
2. `python explore.py --model <model_name>`: This python script starts a node that reads in image observations from the `/usb_cam/image_raw` topic, inputs the observations and the map into the model, and publishes exploration actions to the `/waypoint` topic.
3. `python joy_teleop.py`: This python script starts a node that reads inputs from the joy topic and outputs them on topics that teleoperate the robot’s base.
4. `python pd_controller.py`: This python script starts a node that reads messages from the `/waypoint` topic (waypoints from the model) and outputs velocities to navigate the robot’s base.

When the robot is finishing navigating, kill the `pd_controller.py` script, and then kill the tmux session. If you want to take control of the robot while it is navigating, the `joy_teleop.py` script allows you to do so with the joystick.


### Adapting this code to different robots

We hope that this codebase is general enough to allow you to deploy it to your favorite ROS-based robots. You can change the robot configuration parameters in `vint_release/deployment/config/robot.yaml`, like the max angular and linear velocities of the robot and the topics to publish to teleop and control the robot. Please feel free to create a Github Issue or reach out to the authors at shah@cs.berkeley.edu.
