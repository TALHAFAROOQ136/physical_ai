---
sidebar_position: 2
title: Nodes Aur Topics
description: ROS 2 nodes banayein aur topics ke zariye communicate karein
---

# Nodes Aur Topics

Is chapter men, aap seekhenge ke ROS 2 nodes kaise banayein aur in ke darmiyan communication ke liye topics kaise istemal karein.

## Seekhne Ke Maqasid

- ROS 2 package banayein
- Publisher aur subscriber nodes likhein
- Message types ko samjhein
- Debugging ke liye command-line tools istemal karein

## Node Kya Hai?

Ek **node** ek process hai jo ROS 2 men computation karta hai. Nodes ek dosre se in tariqon se communicate karte hain:

- **Topics** - Publish/subscribe messaging
- **Services** - Request/response calls
- **Actions** - Tooli goal-oriented tasks

## Apna Pehla Package Banayein

Aie apne robot controller ke liye ek package banayein:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_pkg
```

Ye darj zail structure banata hai:

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

## Publisher Node Banana

`my_robot_pkg/talker.py` banayein:

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

## Subscriber Node Banana

`my_robot_pkg/listener.py` banayein:

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

## setup.py Ko Update Karna

Apne nodes ke liye entry points shamil karein:

```python
entry_points={
    'console_scripts': [
        'talker = my_robot_pkg.talker:main',
        'listener = my_robot_pkg.listener:main',
    ],
},
```

## Build Aur Run Karna

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

ROS 2 debugging ke liye talwart CLI tools faraham karta hai:

### Topics Ki Fehrist

```bash
ros2 topic list
```

### Topic Ki Maloomat Dekhein

```bash
ros2 topic info /chatter
```

### Messages Dekhein

```bash
ros2 topic echo /chatter
```

### Command Line Se Publish Karna

```bash
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from CLI'"
```

## Aam Message Types

| Message Type | Package | Tafseel |
|-------------|---------|-------------|
| `String` | `std_msgs` | Sadah string |
| `Int32` | `std_msgs` | 32-bit integer |
| `Float64` | `std_msgs` | 64-bit float |
| `Twist` | `geometry_msgs` | Velocity commands |
| `Pose` | `geometry_msgs` | Position aur orientation |
| `Image` | `sensor_msgs` | Camera images |
| `LaserScan` | `sensor_msgs` | LIDAR data |

## Quality of Service (QoS)

QoS settings message delivery ko control karti hain:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

self.publisher = self.create_publisher(String, 'chatter', qos_profile)
```

## Mashq

Ek temperature sensor node banayein jo:

1. `/temperature` par random temperature values publish kare
2. `Float64` message type istemal kare
3. Har 2 seconds men publish kare

<details>
<summary>Hal</summary>

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

Agla, request-response communication ke liye [Services Aur Actions](/docs/module-1-ros2/services-and-actions) seekhein.