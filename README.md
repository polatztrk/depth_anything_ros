# Depth Anything ROS2 Wrapper

This repository contains a ROS (Robot Operating System) node for estimating depth from RGB images using the Depth Anything model. The Depth Anything model is based on [LiheYoung's depth_anything_model](https://github.com/LiheYoung/Depth-Anything) .

## Prerequisites

- Python 3.x
- ROS (Robot Operating System)
- OpenCV
- PyTorch
- torchvision
- Additionally, you need a robot or camera model that generates /image_raw, either from Gazebo or the real world.


## Installation

Clone the repository to your ROS2 workspace:

```bash
cd your_ws/src
git clone https://github.com/polatztrk/depth_anything_ros.git<img width="608" alt="Screen Shot 2024-03-03 at 15 45 15" src="https://github.com/polatztrk/depth_anything_ros/assets/66801019/b7409e3f-6500-4de2-b249-2e907c09ad04">

cd ..
colcon build
```
## Usage

Inside your workspace you can launch this package:

```bash
cd your_ws
source install/setup.bash
ros2 launch depth_anything_ros launch_depth_anything.launch.py
```







