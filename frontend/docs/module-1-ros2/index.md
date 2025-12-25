---
sidebar_position: 1
title: Introduction to ROS 2
description: Learn the fundamentals of ROS 2 middleware architecture
---

# Module 1: ROS 2 - The Robotic Nervous System

ROS 2 (Robot Operating System 2) is the middleware that powers modern robotics. Think of it as the nervous system that connects all parts of a robot - sensors, actuators, and AI - allowing them to communicate and work together.

## Learning Objectives

By the end of this module, you will be able to:

- Understand the ROS 2 architecture and communication patterns
- Create ROS 2 nodes that publish and subscribe to topics
- Implement services for synchronous request-response communication
- Use actions for long-running tasks with feedback
- Define robot descriptions using URDF

## Why ROS 2?

ROS 2 improves upon ROS 1 with:

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Real-time support | No | Yes |
| Multi-robot support | Limited | Native |
| Security | None | DDS Security |
| Platforms | Linux only | Linux, Windows, macOS |
| Communication | Custom | Industry-standard DDS |

## Core Concepts

### The ROS 2 Graph

ROS 2 uses a distributed graph architecture:

```
┌─────────────┐    Topic    ┌─────────────┐
│   Camera    │ ──────────► │   Vision    │
│   Node      │   /image    │   Node      │
└─────────────┘             └─────────────┘
                                   │
                                   │ /detected_objects
                                   ▼
┌─────────────┐  Service    ┌─────────────┐
│   Motor     │ ◄────────── │   Planner   │
│   Node      │  /move_arm  │   Node      │
└─────────────┘             └─────────────┘
```

### Key Components

1. **Nodes** - Modular processes that perform computation
2. **Topics** - Named buses for asynchronous message passing
3. **Services** - Synchronous request-response communication
4. **Actions** - Long-running tasks with feedback and cancellation
5. **Parameters** - Node configuration values

## Module Structure

This module is organized into the following chapters:

1. **[Nodes and Topics](/docs/module-1-ros2/nodes-and-topics)** - Create your first ROS 2 nodes
2. **[Services and Actions](/docs/module-1-ros2/services-and-actions)** - Build interactive robot behaviors
3. **[URDF Basics](/docs/module-1-ros2/urdf-basics)** - Define robot structure and kinematics

## Hands-On Project

By the end of this module, you'll build a simple robot control system:

```python
# Example: A simple robot controller
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5  # Move forward
        self.publisher.publish(msg)
```

## Prerequisites

Before starting this module, ensure you have:

- Completed the [Environment Setup](/docs/intro/setup)
- ROS 2 Humble installed and working
- Basic Python programming skills

---

Ready to start? Let's begin with [Nodes and Topics](/docs/module-1-ros2/nodes-and-topics)!
