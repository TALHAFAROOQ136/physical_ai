---
sidebar_position: 2
title: Nodes اور Topics
description: ROS 2 nodes بنائیں اور topics کے ذریعے communicate کریں
---

# Nodes اور Topics

اس chapter میں، آپ سیکھیں گے کہ ROS 2 nodes کیسے بنائیں اور ان کے درمیان communication کے لیے topics کیسے استعمال کریں۔

## سیکھنے کے مقاصد

- ROS 2 package بنائیں
- publisher اور subscriber nodes لکھیں
- message types کو سمجھیں
- debugging کے لیے command-line tools استعمال کریں

## Node کیا ہے؟

ایک **node** ایک process ہے جو ROS 2 میں computation کرتا ہے۔ Nodes ایک دوسرے سے ان طریقوں سے communicate کرتے ہیں:

- **Topics** - Publish/subscribe messaging
- **Services** - Request/response calls
- **Actions** - طویل goal-oriented tasks

## اپنا پہلا Package بنائیں

آئیے اپنے robot controller کے لیے ایک package بنائیں:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_pkg
```

یہ درج ذیل structure بناتا ہے:

```
my_robot_pkg/
├── my_robot_pkg/
│   └── __init__.py
├── resource/
│   └── my_robot_pkg
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
├── setup.cfg
└── setup.py
```

## Publisher Node بنانا

`my_robot_pkg/talker.py` بنائیں:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    """A node that publishes messages to a topic."""

    def __init__(self):
        super().__init__('talker')

        # Create publisher
        self.publisher = self.create_publisher(
            String,           # Message type
            'chatter',        # Topic name
            10                # Queue size
        )

        # Create timer to publish periodically
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        """Callback function called by the timer."""
        msg = String()
        msg.data = f'Hello, ROS 2! Count: {self.count}'

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = Talker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Subscriber Node بنانا

`my_robot_pkg/listener.py` بنائیں:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    """A node that subscribes to messages from a topic."""

    def __init__(self):
        super().__init__('listener')

        # Create subscription
        self.subscription = self.create_subscription(
            String,                    # Message type
            'chatter',                 # Topic name
            self.listener_callback,    # Callback function
            10                         # Queue size
        )

    def listener_callback(self, msg):
        """Callback function called when a message is received."""
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = Listener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## setup.py کو Update کرنا

اپنے nodes کے لیے entry points شامل کریں:

```python
entry_points={
    'console_scripts': [
        'talker = my_robot_pkg.talker:main',
        'listener = my_robot_pkg.listener:main',
    ],
},
```

## Build اور Run کریں

```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select my_robot_pkg

# Source the workspace
source install/setup.bash

# Run the talker (Terminal 1)
ros2 run my_robot_pkg talker

# Run the listener (Terminal 2)
ros2 run my_robot_pkg listener
```

## Command-Line Tools

ROS 2 debugging کے لیے طاقتور CLI tools فراہم کرتا ہے:

### Topics کی فہرست

```bash
ros2 topic list
```

### Topic کی معلومات دیکھیں

```bash
ros2 topic info /chatter
```

### Messages دیکھیں

```bash
ros2 topic echo /chatter
```

### Command Line سے Publish کریں

```bash
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from CLI'"
```

## عام Message Types

| Message Type | Package | تفصیل |
|-------------|---------|-------------|
| `String` | `std_msgs` | سادہ string |
| `Int32` | `std_msgs` | 32-bit integer |
| `Float64` | `std_msgs` | 64-bit float |
| `Twist` | `geometry_msgs` | Velocity commands |
| `Pose` | `geometry_msgs` | Position اور orientation |
| `Image` | `sensor_msgs` | Camera images |
| `LaserScan` | `sensor_msgs` | LIDAR data |

## Quality of Service (QoS)

QoS settings message delivery کو control کرتی ہیں:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

self.publisher = self.create_publisher(String, 'chatter', qos_profile)
```

## مشق

ایک temperature sensor node بنائیں جو:

1. `/temperature` پر random temperature values publish کرے
2. `Float64` message type استعمال کرے
3. ہر 2 سیکنڈ میں publish کرے

<details>
<summary>حل</summary>

```python
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class TemperatureSensor(Node):
    def __init__(self):
        super().__init__('temperature_sensor')
        self.publisher = self.create_publisher(Float64, 'temperature', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        msg = Float64()
        msg.data = 20.0 + random.random() * 10.0  # 20-30°C
        self.publisher.publish(msg)
        self.get_logger().info(f'Temperature: {msg.data:.1f}°C')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TemperatureSensor())
    rclpy.shutdown()
```

</details>

---

اگلا، request-response communication کے لیے [Services اور Actions](/docs/module-1-ros2/services-and-actions) سیکھیں۔
