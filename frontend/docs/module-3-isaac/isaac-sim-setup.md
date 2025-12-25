---
sidebar_position: 2
title: Isaac Sim Setup
description: Install and configure NVIDIA Isaac Sim
---

# Isaac Sim Setup

NVIDIA Isaac Sim is a photorealistic, physically-accurate simulation platform for robotics.

## Learning Objectives

- Install Isaac Sim
- Navigate the interface
- Import robots and create environments
- Connect to ROS 2

## System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| GPU | RTX 2070 | RTX 3080+ |
| VRAM | 8 GB | 16+ GB |
| RAM | 32 GB | 64 GB |
| Storage | 50 GB SSD | 100+ GB NVMe |

## Installation

### Using Omniverse Launcher

1. Download [Omniverse Launcher](https://www.nvidia.com/en-us/omniverse/)
2. Install Isaac Sim from the Exchange tab
3. Launch Isaac Sim

### Docker (Alternative)

```bash
# Pull Isaac Sim container
docker pull nvcr.io/nvidia/isaac-sim:2023.1.0

# Run with GPU support
docker run --gpus all -it \
  -v ~/isaac_ws:/isaac_ws \
  nvcr.io/nvidia/isaac-sim:2023.1.0
```

## First Launch

```bash
# From Omniverse Launcher or
cd ~/.local/share/ov/pkg/isaac_sim-*/
./isaac-sim.sh
```

## ROS 2 Bridge

Enable the ROS 2 bridge in Isaac Sim:

```python
# In Isaac Sim script editor
from omni.isaac.ros2_bridge import ROS2Bridge

ros_bridge = ROS2Bridge()
ros_bridge.enable()
```

## Importing a Robot

```python
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.urdf")

from omni.isaac.urdf import _urdf
urdf_interface = _urdf.acquire_urdf_interface()

# Import URDF
import_config = _urdf.ImportConfig()
robot_path = urdf_interface.parse_urdf(
    "/path/to/robot.urdf",
    import_config
)
```

## Exercise

Import a simple robot into Isaac Sim and verify ROS 2 communication.

---

Continue to [Perception Pipelines](/docs/module-3-isaac/perception-pipelines).
