---
sidebar_position: 3
title: Environment Setup
description: Set up your development environment for robotics development
---

# Environment Setup

This guide will help you set up your development environment for Physical AI and Humanoid Robotics development.

## Install ROS 2 Humble

ROS 2 Humble Hawksbill is the recommended LTS release for this course.

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

### Verify Installation

```bash
# Source the setup script
source /opt/ros/humble/setup.bash

# Run the demo
ros2 run demo_nodes_cpp talker
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

You should see messages being published and received!

## Install Gazebo

Gazebo is the default simulator for ROS 2.

```bash
# Install Gazebo Fortress (LTS)
sudo apt install ros-humble-gazebo-ros-pkgs
```

### Verify Gazebo

```bash
gazebo --version
```

## Install Development Tools

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

Install these VS Code extensions for a better development experience:

- **ROS** - Microsoft's ROS extension
- **Python** - Python language support
- **C/C++** - C++ language support
- **YAML** - YAML file support

## Create Your Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

Add to your `~/.bashrc`:

```bash
# ROS 2 setup
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Useful aliases
alias cb='cd ~/ros2_ws && colcon build'
alias cs='cd ~/ros2_ws && source install/setup.bash'
```

## Verify Complete Setup

Run this verification script:

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

## Troubleshooting

### Common Issues

**"ros2: command not found"**
```bash
source /opt/ros/humble/setup.bash
```

**"Package not found"**
```bash
rosdep install --from-paths src --ignore-src -r -y
```

**Gazebo crashes on startup**
```bash
# Check OpenGL support
glxinfo | grep "OpenGL version"
```

---

You're now ready to start learning ROS 2! Proceed to [Module 1: ROS 2](/docs/module-1-ros2) to begin building robot systems.
