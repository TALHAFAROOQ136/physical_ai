---
sidebar_position: 3
title: Perception Pipelines
description: GPU-accelerated perception with Isaac
---

# Perception Pipelines

NVIDIA Isaac provides GPU-accelerated perception capabilities for robotics.

## Learning Objectives

- Set up camera sensors in Isaac Sim
- Process sensor data with Isaac ROS
- Implement object detection and segmentation

## Isaac ROS Packages

| Package | Function |
|---------|----------|
| `isaac_ros_image_proc` | Image processing |
| `isaac_ros_stereo_image` | Stereo vision |
| `isaac_ros_dnn_inference` | Neural network inference |
| `isaac_ros_apriltag` | AprilTag detection |

## Installing Isaac ROS

```bash
# Create workspace
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src

# Clone Isaac ROS
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# Build
cd ~/isaac_ros_ws
colcon build --symlink-install
```

## Camera Pipeline

```python
# Launch Isaac ROS camera pipeline
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isaac_ros_image_proc',
            executable='image_proc',
            parameters=[{
                'target_fps': 30.0,
            }],
            remappings=[
                ('image_raw', '/camera/color/image_raw'),
            ],
        ),
    ])
```

## Object Detection

```python
# Using Isaac ROS DNN for object detection
Node(
    package='isaac_ros_dnn_inference',
    executable='dnn_image_encoder',
    parameters=[{
        'network_image_width': 640,
        'network_image_height': 480,
    }],
)
```

## Synthetic Data Generation

Isaac Sim can generate labeled training data:

- RGB images with ground truth labels
- Depth maps
- Semantic segmentation masks
- 3D bounding boxes

## Exercise

Create a perception pipeline that:
1. Captures RGB-D images
2. Detects objects using a pre-trained model
3. Publishes detections to ROS 2

---

Congratulations! Continue to [Module 4: VLA](/docs/module-4-vla).
