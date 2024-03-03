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
git clone https://github.com/polatztrk/depth_anything_ros.git
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
Then you can see these as a result in Rviz and Gazebo:
![ROS Depth Anything](https://github.com/polatztrk/depth_anything_ros/blob/d504431a4b5cfdcda3e71adc1293f1da2dd28222/depth_anything_ros/images/1.png)
![ROS Depth Anything](https://github.com/polatztrk/depth_anything_ros/blob/d504431a4b5cfdcda3e71adc1293f1da2dd28222/depth_anything_ros/images/2.png)
![ROS Depth Anything](https://github.com/polatztrk/depth_anything_ros/blob/d504431a4b5cfdcda3e71adc1293f1da2dd28222/depth_anything_ros/images/3.png)
![ROS Depth Anything](https://github.com/polatztrk/depth_anything_ros/blob/d504431a4b5cfdcda3e71adc1293f1da2dd28222/depth_anything_ros/images/4.png)

## Depth map to point cloud converter for Rviz

Inside your workspace you can launch this package:
```bash
cd your_ws
source install/setup.bash
ros2 launch depth2point launch_depth_to_point.launch.py
```










