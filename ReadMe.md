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
sudo apt install ros-humble-tf-transformations
sudo apt install ros-humble-py-binding-tools
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

# If the build dies on its own or the
# machine freezes, it is RAM, not your setup: colcon builds several packages at once
# and each one spawns N compilers. Retry one at a time (slower, but it fits):
MAKEFLAGS="-j1" colcon build --executor sequential --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> The sequential build resumes whatever already compiled, so there is no need to
> delete `build/` and start over.

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

Launch example script. Replace EXAMPLE_NAME with the script filename (e.g. example_joint_goal):

```bash
ros2 launch xarm_scripts xarm_scripts.launch.py script:=EXAMPLE_NAME
```

### Pick and Place - Classic Example

Launch simulation. `world:=table_gz.world` is required: the default `table.world`
contains only the ground, the sun and the table, so there is no cube to pick and
no obstacle to avoid.

```bash
ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py \
  world:=table_gz.world add_gripper:=true
```

Launch pick and place example script.

```bash
ros2 launch xarm_scripts xarm_scripts.launch.py script:=pick_and_place
```

### Hybrid Planning

Arquitectura de hybrid planning de MoveIt 2 (global planner + local planner +
manager) sobre el xArm6, con un planner logic plugin propio
(`xarm_hybrid_planning/ReplanWhenIdle`) que cierra el ciclo reactivo: aparece un
obstaculo a mitad del movimiento, se replanifica, y el brazo llega a la meta. Ver
`xarm_scripts/HYBRID_PLANNING.md`.

Launch simulation:

```bash
ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py \
  world:=table_gz.world add_gripper:=true
```

Launch the three hybrid planning components plus the demo:

```bash
ros2 launch xarm_scripts hybrid_planning_xarm6.launch.py
```

### Pick and Place - MTC Example

Please refer to README_FINAL_PROJECT.md for instructions to install MTC package.

[EXTRA] Debug planning solutions:
```bash
ros2 launch xarm_scripts pickplace_mtc_planning.launch.py
```

Launch simulation with gripper and world with colored cubes:

```bash
ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py robot_x:=0.0 robot_y:=-0.4 robot_z:=0.45 robot_yaw:=1.5708 world:=table_color_objects.world add_gripper:=true
```

Launch example script, it places red cube in the trash bin.

```bash
ros2 launch xarm_scripts pickplace_mtc_gazebo_colored.launch.py pick_x:=0.606 pick_y:=0.203
```