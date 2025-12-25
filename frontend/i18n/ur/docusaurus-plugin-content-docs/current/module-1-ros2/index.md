---
sidebar_position: 1
title: ROS 2 کا تعارف
description: ROS 2 middleware architecture کے بنیادی اصول سیکھیں
---

# Module 1: ROS 2 - روبوٹک اعصابی نظام

ROS 2 (Robot Operating System 2) وہ middleware ہے جو جدید robotics کو power دیتا ہے۔ اسے اعصابی نظام سمجھیں جو روبوٹ کے تمام حصوں - sensors، actuators، اور AI - کو جوڑتا ہے، انہیں communicate اور مل کر کام کرنے کی اجازت دیتا ہے۔

## سیکھنے کے مقاصد

اس module کے اختتام تک، آپ یہ کرنے کے قابل ہوں گے:

- ROS 2 architecture اور communication patterns کو سمجھیں
- ROS 2 nodes بنائیں جو topics پر publish اور subscribe کریں
- synchronous request-response communication کے لیے services implement کریں
- feedback کے ساتھ طویل کاموں کے لیے actions استعمال کریں
- URDF کا استعمال کرتے ہوئے robot descriptions define کریں

## ROS 2 کیوں؟

ROS 2 ROS 1 میں ان بہتریوں کے ساتھ آگے بڑھتا ہے:

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Real-time support | نہیں | ہاں |
| Multi-robot support | محدود | Native |
| Security | کوئی نہیں | DDS Security |
| Platforms | صرف Linux | Linux، Windows، macOS |
| Communication | Custom | Industry-standard DDS |

## بنیادی تصورات

### ROS 2 Graph

ROS 2 ایک distributed graph architecture استعمال کرتا ہے:

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

### اہم اجزاء

1. **Nodes** - Modular processes جو computation کرتے ہیں
2. **Topics** - asynchronous message passing کے لیے named buses
3. **Services** - Synchronous request-response communication
4. **Actions** - feedback اور cancellation کے ساتھ طویل کام
5. **Parameters** - Node configuration values

## Module کا ڈھانچہ

یہ module درج ذیل chapters میں منظم ہے:

1. **[Nodes اور Topics](/docs/module-1-ros2/nodes-and-topics)** - اپنے پہلے ROS 2 nodes بنائیں
2. **[Services اور Actions](/docs/module-1-ros2/services-and-actions)** - interactive robot behaviors بنائیں
3. **[URDF Basics](/docs/module-1-ros2/urdf-basics)** - robot structure اور kinematics define کریں

## Hands-On Project

اس module کے آخر میں، آپ ایک سادہ robot control system بنائیں گے:

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

## شرائط

اس module کو شروع کرنے سے پہلے، یقینی بنائیں کہ آپ نے:

- [Environment Setup](/docs/intro/setup) مکمل کر لیا
- ROS 2 Humble انسٹال اور کام کر رہا ہے
- بنیادی Python programming کی مہارتیں ہیں

---

شروع کرنے کے لیے تیار ہیں؟ آئیے [Nodes اور Topics](/docs/module-1-ros2/nodes-and-topics) سے شروع کریں!
