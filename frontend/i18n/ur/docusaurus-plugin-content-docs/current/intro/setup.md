---
sidebar_position: 3
title: Environment Setup
description: robotics development کے لیے اپنا development environment سیٹ اپ کریں
---

# Environment Setup

یہ گائیڈ آپ کو Physical AI اور Humanoid Robotics development کے لیے اپنا development environment سیٹ اپ کرنے میں مدد کرے گی۔

## ROS 2 Humble انسٹال کریں

ROS 2 Humble Hawksbill اس کورس کے لیے تجویز کردہ LTS release ہے۔

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

### Installation کی تصدیق

```bash
# Source the setup script
source /opt/ros/humble/setup.bash

# Run the demo
ros2 run demo_nodes_cpp talker
```

دوسری terminal میں:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

آپ کو messages publish اور receive ہوتے دکھائی دیں گے!

## Gazebo انسٹال کریں

Gazebo ROS 2 کے لیے default simulator ہے۔

```bash
# Install Gazebo Fortress (LTS)
sudo apt install ros-humble-gazebo-ros-pkgs
```

### Gazebo کی تصدیق

```bash
gazebo --version
```

## Development Tools انسٹال کریں

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

بہتر development experience کے لیے یہ VS Code extensions انسٹال کریں:

- **ROS** - Microsoft کی ROS extension
- **Python** - Python language support
- **C/C++** - C++ language support
- **YAML** - YAML file support

## اپنا Workspace بنائیں

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

اپنی `~/.bashrc` میں شامل کریں:

```bash
# ROS 2 setup
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Useful aliases
alias cb='cd ~/ros2_ws && colcon build'
alias cs='cd ~/ros2_ws && source install/setup.bash'
```

## مکمل Setup کی تصدیق

یہ verification script چلائیں:

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

## مسائل کا حل

### عام مسائل

**"ros2: command not found"**
```bash
source /opt/ros/humble/setup.bash
```

**"Package not found"**
```bash
rosdep install --from-paths src --ignore-src -r -y
```

**Gazebo شروع میں crash ہو جائے**
```bash
# Check OpenGL support
glxinfo | grep "OpenGL version"
```

---

اب آپ ROS 2 سیکھنا شروع کرنے کے لیے تیار ہیں! روبوٹ سسٹمز بنانا شروع کرنے کے لیے [Module 1: ROS 2](/docs/module-1-ros2) پر جائیں۔
