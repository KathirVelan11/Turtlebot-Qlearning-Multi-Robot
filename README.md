# Multi-Robot Q-Learning Exploration
[![IEEE Paper](https://img.shields.io/badge/IEEE-Paper-blue)](https://ieeexplore.ieee.org/abstract/document/10969952?casa_token=l6-5WLFp6IYAAAAA:PF2OjNDB6tvW-Up4EojfFEiSzUtS5ljlr8dxVrF5ErdVm9d_-ZmeS8V4o9Xi-wxU4e8aTD_7_AQA)
[![ROS Version](https://img.shields.io/badge/ROS-Noetic-brightgreen)](http://wiki.ros.org/noetic)
[![Python Version](https://img.shields.io/badge/Python-3.8+-blue)](https://www.python.org/downloads/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04%20Focal%20Fossa-orange)](https://releases.ubuntu.com/20.04/)

## Overview

This project implements a multi-robot exploration system using Q-learning for autonomous navigation and area coverage in simulated environments. Four TurtleBot3 robots collaborate to explore an unknown environment without centralized control, using reinforcement learning to optimize their exploration strategies.

## 📄 Official IEEE Paper

_K. Velan et al., "Multi-Robot Exploration using Decentralized Q-Learning," IEEE Xplore, 2025._  
[🔗 Click here to view the paper on IEEE Xplore](https://ieeexplore.ieee.org/abstract/document/10969952)

### Key Features

- **Multi-Agent Q-Learning**: Decentralized learning approach with same Q-table for all 4 turtlebots.
- **Autonomous Exploration**: Grid-based environment discretization for systematic coverage
- **Collision Avoidance**: Laser scan-based obstacle detection and avoidance
- **Real-time Visualization**: RViz integration for exploration progress and Q-value visualization
- **Reward Shaping**: Custom defined reward system encouraging exploration while penalizing collisions and repeated exploration on same place.

## Project Structure
```
Turtlebot-Qlearning-Multi-Robot/
├── launch/
│   └── multi_robot_world.launch                                 # Launch file for Gazebo simulation
├── src/
│   ├── q_learning_node.py                                       # Main Q-learning implementation
│   └── visualization_node.py                                    # Evaluation metrics and visualization
├── world/
│   └── training.world                                           # Custom Gazebo world file
├── data/
│   └── q_table_data.csv                                         # Pre-trained Q-table data
├── config/
│   └── package.xml                                              # ROS package configuration
├── docs/
│   └── Swarm_Robotics_for_Area_Exploration_Using_Q-Learning     # PDF copy of the published IEEE paper for offline reference
└── requirements.txt                                             # Python dependencies
```

## 🛠️ Installation

> ⚠️ **Note**: This project is tested and designed to work only on **Ubuntu 20.04 (Focal Fossa)** with **ROS Noetic** and **Gazebo 11.11+**. Compatibility with other Ubuntu or ROS versions is **not guaranteed**.
> It is meant for **simulation only** and is **not tested on real hardware**.

### Step 1: Install ROS Noetic
```bash
# Add ROS repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
```bash
# Add keys
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
```bash
# Update package index
sudo apt update
```
```bash
# Install ROS Noetic Desktop Full
sudo apt install ros-noetic-desktop-full
```
```bash
# Initialize rosdep
sudo rosdep init
rosdep update
```
```bash
# Setup environment
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Install TurtleBot3 Packages
```bash
# Install TurtleBot3 packages
sudo apt install ros-noetic-turtlebot3-*
sudo apt install ros-noetic-gazebo-*
sudo apt install ros-noetic-navigation
```
```bash
# Set TurtleBot3 model environment variable
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

### Step 3: Create Workspace and Clone Repository
```bash
# Create catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```
```bash
# Clone this repository
git clone https://github.com/yourusername/multi-robot-qlearning-exploration.git
```
```bash
# Navigate to workspace and build
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```
```bash
# Source the workspace
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 4: Install Python Dependencies
> ✅ Make sure you are using **Python 3.8+**.
```bash
# Install required Python packages
pip3 install numpy pandas matplotlib
```

## ▶️ How to use
```bash
# Clean any existing ROS processes
killall -9 rosmaster roscore gazebo gzserver gzclient
```
```bash
# Start roscore
roscore
```
```bash
# Kill any existing nodes
rosnode kill -a
```
```bash
# Navigate to workspace and rebuild
cd ~/catkin_ws
catkin_make clean
catkin_make
```
```bash
# Source environments
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```
```bash
# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger
```
```bash
# Launch the simulation
roslaunch multi_robot_qlearning_exploration multi_robot_world.launch
```

### Open another terminal - To run the Q-Learning Algorithm :
```bash
# Source the workspace
source ~/catkin_ws/devel/setup.bash
```
```bash
# Run the Q-learning node
rosrun multi_robot_qlearning_exploration q_learning_node.py
```

### Open another terminal -To start RViz :
```bash
# Launch RViz
rosrun rviz rviz
# Add the following topics in RViz:
# - /map (for environment)
# - /robot1/scan, /robot2/scan, /robot3/scan, /robot4/scan (laser scans)
# - /explored_areas (visualization markers)
# - /q_values (Q-value heatmap)
```

### To see Evaluation metrics in RViz :
```bash
# Run visualization node for metrics
rosrun multi_robot_qlearning_exploration visualization_node.py
```

## 📚 Citation
If you use this work in your research, please cite:
```

@article{Velan2025MultiRobotQLearning,
author = {Kathir Velan and Others},
title = {Multi-Robot Exploration using Decentralized Q-Learning},
journal = {IEEE Xplore},
year = {2025},
doi = {10.1109/XXXXXX.2025.10969952}
}
```

## 🤝 Want to Help?
Fork the repo and send a pull request with any improvements!
