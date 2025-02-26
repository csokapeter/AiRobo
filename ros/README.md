# ROS 2 and Unity Integration Setup
The project was made using ROS2 foxy and Unity version 2022.3.20f1

## Prerequisites
First, install a ROS 2 version that matches your operating system.

## Steps to Setup

### 1. Install Unity
You can download Unity from the [official website](https://unity.com/download).

### 2. Install ROS TCP Endpoint for Unity

- Download Unity's ROS TCP endpoint by following the instructions in this [documentation](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md#-ros-environment).
  
- Create a new workspace:
  ```bash
  mkdir unity_ws; cd unity_ws
  mkdir src; cd src
  git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git --branch main-ros2
  ```

### 3. Build ROS 2 Packages
In a terminal navigate to the packages folder:
  ```bash
  source /opt/ros/foxy/setup.bash
  colcon build
  ```

## Running the project

- running the ROS TCP server endpoint
  ```bash
  source /opt/ros/foxy/setup.bash
  source install/setup.bash
  hostname -I # get IP address
  ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=<your IP address>
  ```

- running the navigation server in another terminal
  ```bash
  source /opt/ros/foxy/setup.bash
  source install/setup.bash
  ros2 run navigation_server navigation_server
  ```

## Configuring Unity
After opening the project
- navigate to the Robotics menu and go to ROS Settings
- Change the Protocol to ROS2
- Add the IP address seen in the terminal as the ROS IP Address
- Click Play in Unity