# xarm_ros2

## Introduction

This repository contains simulation models, and corresponding motion planning and controlling demos of the xArm series from UFACTORY. The development and test environment is as follows:

- Ubuntu 22.04 + ROS Humble + Gazebo fortress

## Preparation

### Install Dependencies

```bash
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget

sudo apt install ros-humble-ros-ign-bridge
sudo apt install ros-humble-ros-ign-gazebo
sudo apt install ros-humble-ign-ros2-control
sudo apt update
sudo apt dist-upgrade
rosdep update
```

### Install MoveIt2

MoveIt2 must be installed from source for Humble and Python support:

```bash
mkdir -p manip_ws/src
cd manip_ws/src
git clone https://github.com/moveit/moveit2.git -b $ROS_DISTRO
for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
cd ..
colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Build Repository

```bash
cd ~/manip_ws/src
git clone https://github.com/zetanschy/xarm_ros2.git --recursive
cd ..
colcon build
```

### Rviz and Moveit Setup Assistant Conflict
Moveit assistant does not work with default version of rviz in humble: 

```bash
wget http://snapshots.ros.org/humble/2025-06-18/ubuntu/pool/main/r/ros-humble-rviz-common/ros-humble-rviz-common_11.2.17-1jammy.20250617.234657_amd64.deb
```

```bash
sudo dpkg -i ros-humble-rviz-common_11.2.17-1jammy.20250617.234657_amd64.deb
```

## Examples

### Using MoveIt Python API

Launch simulation:

```bash
ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py
```

Launch example script:

```bash
ros2 launch xarm_scripts moveit_py_joint_goal.launch.py
```

or

```bash
ros2 launch xarm_scripts moveit_py_pose_goal.launch.py
```

