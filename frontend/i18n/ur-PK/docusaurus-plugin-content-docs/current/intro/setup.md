---
sidebar_position: 3
title: Environment Setup
description: Robotics development ke liye apna development environment set up karein
---

# Environment Setup

Ye guide aap ko Physical AI aur Humanoid Robotics development ke liye apna development environment set up karne men madad kare gi.

## ROS 2 Humble Install Karein

ROS 2 Humble Hawksbill is kurs ke liye tajveez karda LTS release hai.

### Ubuntu 22.04

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

### Installation Ki Tasdiq

```bash
# Source the setup script
source /opt/ros/humble/setup.bash

# Run the demo
ros2 run demo_nodes_cpp talker
```

Doosri terminal men:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

Aap ko messages publish aur receive hote dikhayi denge!

## Gazebo Install Karein

Gazebo ROS 2 ke liye default simulator hai.

```bash
# Install Gazebo Fortress (LTS)
sudo apt install ros-humble-gazebo-ros-pkgs
```

### Gazebo Ki Tasdiq

```bash
gazebo --version
```

## Development Tools Install Karein

### Colcon Build System

```bash
sudo apt install python3-colcon-common-extensions
```

### ROS 2 Development Tools

```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
```

### VS Code Extensions

Behtar development experience ke liye ye VS Code extensions install karein:

- **ROS** - Microsoft ki ROS extension
- **Python** - Python language support
- **C/C++** - C++ language support
- **YAML** - YAML file support

## Apna Workspace Banayein

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

Apni `~/.bashrc` men shamil karein:

```bash
# ROS 2 setup
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Useful aliases
alias cb='cd ~/ros2_ws && colcon build'
alias cs='cd ~/ros2_ws && source install/setup.bash'
```

## Mokammal Setup Ki Tasdiq

Ye verification script chalaein:

```bash
#!/bin/bash

echo "Checking ROS 2 installation..."
ros2 --version

echo ""
echo "Checking Gazebo installation..."
gazebo --version

echo ""
echo "Checking Python..."
python3 --version

echo ""
echo "Checking colcon..."
colcon version-check

echo ""
echo "Setup verification complete!"
```

## Masa'il Ka Hal

### Aam Masa'il

**"ros2: command not found"**
```bash
source /opt/ros/humble/setup.bash
```

**"Package not found"**
```bash
rosdep install --from-paths src --ignore-src -r -y
```

**Gazebo shuru men crash ho jaye**
```bash
# Check OpenGL support
glxinfo | grep "OpenGL version"
```

---

Ab aap ROS 2 seekhna shuru karne ke liye taiyar hain! Robot systems banana shuru karne ke liye [Module 1: ROS 2](/docs/module-1-ros2) par jaein.