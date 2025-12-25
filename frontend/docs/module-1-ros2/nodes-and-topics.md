---
sidebar_position: 2
title: Nodes and Topics
description: Create ROS 2 nodes and communicate via topics
---

# Nodes and Topics

In this chapter, you'll learn how to create ROS 2 nodes and use topics for communication between them.

## Learning Objectives

- Create a ROS 2 package
- Write publisher and subscriber nodes
- Understand message types
- Use command-line tools for debugging

## What is a Node?

A **node** is a process that performs computation in ROS 2. Nodes communicate with each other through:

- **Topics** - Publish/subscribe messaging
- **Services** - Request/response calls
- **Actions** - Long-running goal-oriented tasks

## Creating Your First Package

Let's create a package for our robot controller:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_pkg
```

This creates the following structure:

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

## Creating a Publisher Node

Create `my_robot_pkg/talker.py`:

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

## Creating a Subscriber Node

Create `my_robot_pkg/listener.py`:

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

## Updating setup.py

Add entry points for your nodes:

```python
entry_points={
    'console_scripts': [
        'talker = my_robot_pkg.talker:main',
        'listener = my_robot_pkg.listener:main',
    ],
},
```

## Build and Run

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

ROS 2 provides powerful CLI tools for debugging:

### List Topics

```bash
ros2 topic list
```

### View Topic Info

```bash
ros2 topic info /chatter
```

### Echo Messages

```bash
ros2 topic echo /chatter
```

### Publish from Command Line

```bash
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from CLI'"
```

## Common Message Types

| Message Type | Package | Description |
|-------------|---------|-------------|
| `String` | `std_msgs` | Simple string |
| `Int32` | `std_msgs` | 32-bit integer |
| `Float64` | `std_msgs` | 64-bit float |
| `Twist` | `geometry_msgs` | Velocity commands |
| `Pose` | `geometry_msgs` | Position and orientation |
| `Image` | `sensor_msgs` | Camera images |
| `LaserScan` | `sensor_msgs` | LIDAR data |

## Quality of Service (QoS)

QoS settings control message delivery:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

self.publisher = self.create_publisher(String, 'chatter', qos_profile)
```

## Exercise

Create a temperature sensor node that:

1. Publishes random temperature values to `/temperature`
2. Uses `Float64` message type
3. Publishes every 2 seconds

<details>
<summary>Solution</summary>

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

Next, learn about [Services and Actions](/docs/module-1-ros2/services-and-actions) for request-response communication.
