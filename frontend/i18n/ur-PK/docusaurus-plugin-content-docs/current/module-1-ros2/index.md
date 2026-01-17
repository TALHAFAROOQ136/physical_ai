---
sidebar_position: 1
title: ROS 2 Ka Taaruf
description: ROS 2 middleware architecture ke buniyadi usool seekhein
---

# Module 1: ROS 2 - Robotic A'asir System

ROS 2 (Robot Operating System 2) wo middleware hai jo jadid robotics ko power deti hai. Is ko a'asir system samjhein jo robot ke tamam hisson - sensors, actuators, aur AI - ko judta hai, inhe communicate aur mil kar kam karne ki ijazat deta hai.

## Seekhne Ke Maqasid

Is module ke ikhtetam tak, aap ye karne ke qabil honge:

- ROS 2 architecture aur communication patterns ko samjhein
- ROS 2 nodes banayein jo topics par publish aur subscribe karein
- Synchronous request-response communication ke liye services implement karein
- Feedback ke sath tooli kamon ke liye actions istemal karein
- URDF ka istemal karte hue robot descriptions define karein

## ROS 2 Kyun?

ROS 2 ROS 1 men in behtriyon ke sath aage berta hai:

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Real-time support | Nahin | Haan |
| Multi-robot support | Muhdud | Native |
| Security | Koi nahin | DDS Security |
| Platforms | Sirf Linux | Linux, Windows, macOS |
| Communication | Custom | Industry-standard DDS |

## Buniyadi Tasawwurat

### ROS 2 Graph

ROS 2 ek distributed graph architecture istemal karta hai:

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

### Ahem Ajza

1. **Nodes** - Modular processes jo computation karte hain
2. **Topics** - Asynchronous message passing ke liye named buses
3. **Services** - Synchronous request-response communication
4. **Actions** - Feedback aur cancellation ke sath tooli kam
5. **Parameters** - Node configuration values

## Module Ka Dhancha

Ye module darj zail chapters men munnazzam hai:

1. **[Nodes Aur Topics](/docs/module-1-ros2/nodes-and-topics)** - Apne pehle ROS 2 nodes banayein
2. **[Services Aur Actions](/docs/module-1-ros2/services-and-actions)** - Interactive robot behaviors banayein
3. **[URDF Basics](/docs/module-1-ros2/urdf-basics)** - Robot structure aur kinematics define karein

## Hands-On Project

Is module ke akhir men, aap ek sadah robot control system banayein ge:

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

## Shara'it

Is module ko shuru karne se pehle, yaqeeni banaein ke aap ne:

- [Environment Setup](/docs/intro/setup) Mokammal kar lia
- ROS 2 Humble install aur kaam kar raha hai
- Buniyadi Python programming ki maharatein hain

---

Shuru karne ke liye taiyar hain? Aie [Nodes Aur Topics](/docs/module-1-ros2/nodes-and-topics) se shuru karte hain!