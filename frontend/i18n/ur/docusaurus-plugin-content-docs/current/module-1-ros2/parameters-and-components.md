---
sidebar_position: 5
title: Parameters aur Components
description: Configurations ka elaj karen aur lightweight composable nodes ka istemal karen
---

# Parameters aur Components

Iss chapter mein, aap seekhenge ke ROS 2 mein parameters ke elaj kaise karte hain aur components ka istemal kaise karte hain.

## Seeking Objectives

- Nodes ke configuration ke liye parameters ka istemal karen
- Parameter callbacks laagoo karen
- Composable nodes banayein aur istemal karen
- Lifecycle nodes ko samjhein

## Parameters

**Parameters** named values hote hain jo runtime par configure kiye ja sakte hain. Ye configuration settings ke liye perfect hain jo deployments ke darmiyan badal sakte hain.

### Parameters Set Karna

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('param_node')

        # Default values ke saath parameters declare karen
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('wheel_diameter', 0.3)
        self.declare_parameter('robot_name', 'my_robot')

        # Parameter values hasil karen
        self.max_speed = self.get_parameter('max_speed').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.robot_name = self.get_parameter('robot_name').value

        self.get_logger().info(f'Robot: {self.robot_name}, Max speed: {self.max_speed}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()

    # Aap runtime par parameters change kar sakte hain
    node.set_parameters([rclpy.parameter.Parameter('max_speed', rclpy.Parameter.Type.DOUBLE, 2.0)])

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Parameter Descriptors

Aap parameters par constraints laga sakte hain:

```python
from rclpy.node import ParameterDescriptor
from rclpy.parameter import ParameterType

class ConstrainedParamNode(Node):
    def __init__(self):
        super().__init__('constrained_param_node')

        # Constraints ke saath descriptor
        desc = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Maximum robot speed in m/s',
            additional_constraints='Must be positive and less than 10.0',
            floating_point_range=[(0.0, 10.0, '')]  # min, max, name
        )

        self.declare_parameter('max_speed', 1.0, descriptor=desc)
```

### Parameter Callbacks

Parameter changes ko monitor karen:

```python
from rclpy.parameter import Parameter

class ParamCallbackNode(Node):
    def __init__(self):
        super().__init__('param_callback_node')
        self.declare_parameter('threshold', 0.5)

        # Parameter changes ke liye callback register karen
        self.add_on_set_parameters_callback(self.param_callback)

        self.threshold = self.get_parameter('threshold').value

    def param_callback(self, params):
        for param in params:
            if param.name == 'threshold':
                if param.value > 1.0 or param.value < 0.0:
                    return SetParametersResult(successful=False, reason='Threshold must be between 0.0 and 1.0')

                self.threshold = param.value
                self.get_logger().info(f'Threshold updated to {self.threshold}')

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ParamCallbackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Composable Nodes

Composable nodes multiple nodes ko single process ke andhar chalane ki ijazat dete hain, overhead ko kam karke aur performance ko behtar bana kar.

### Ek Component Banana

```python
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from std_msgs.msg import String

class MyComponent(Node):
    def __init__(self, name):
        super().__init__(name)

        # Parameters declare karen
        self.declare_parameter('frequency', 1.0)

        # Publisher aur timer banayein
        self.pub = self.create_publisher(String, 'component_topic', 10)
        self.timer = self.create_timer(
            1.0 / self.get_parameter('frequency').value,
            self.timer_callback
        )
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Component message #{self.counter}'
        self.pub.publish(msg)
        self.counter += 1

# Component register karen
from rclpy_components.register_node import register_node
register_node(MyComponent)
```

### Composition Container

Components ko manage karne ke liye container banayein:

```python
from rclpy_components.component_manager import ComponentManager
import rclpy

def main(args=None):
    rclpy.init(args=args)

    # Component manager banayein
    manager = ComponentManager('composition_container')

    # Components ko dynamically load karen
    try:
        # Component ko class type se load karen
        node_instance = manager.add_node_by_class('my_package', 'MyComponent')

        # Component ke liye parameters configure karen
        node_instance.set_parameters([
            Parameter('frequency', Parameter.Type.DOUBLE, 2.0)
        ])

        rclpy.spin(manager)
    except Exception as e:
        print(f'Failed to load component: {e}')

    manager.destroy_node()
    rclpy.shutdown()
```

## Lifecycle Nodes

Lifecycle nodes node initialization, activation, aur cleanup ko manage karne ke liye ek state machine provide karte hain.

```python
from rclpy.lifecycle import LifecycleNode, ManagedEntity, Publisher, Timer
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import String

class LifecycleTalker(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_talker')
        self.timer = None
        self.pub = None
        self.counter = 0

    # Lifecycle callbacks
    def on_configure(self, state):
        self.get_logger().info('Configuring')
        self.pub = self.create_publisher(String, 'lifecycle_chatter', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating')
        self.timer = self.create_timer(1.0, self.timer_callback)
        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating')
        self.timer.cancel()
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self.get_logger().info('Cleaning up')
        self.destroy_publisher(self.pub)
        self.pub = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.get_logger().info('Shutting down')
        if self.timer is not None:
            self.timer.cancel()
        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        if self.pub is not None:
            msg = String()
            msg.data = f'Lifecycle message #{self.counter}'
            self.pub.publish(msg)
            self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = LifecycleTalker()

    # Unconfigured state mein start karen
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
```

## Parameter Files ke Saath Kaam Karna

Parameters ko YAML files mein store karen:

```yaml
# config/robot_params.yaml
/**:
  ros__parameters:
    max_speed: 2.0
    wheel_diameter: 0.3
    robot_name: "turtlebot4"
    sensors:
      lidar_enabled: true
      camera_enabled: true
    navigation:
      planner_frequency: 1.0
      controller_frequency: 10.0
```

Launch files mein parameters load karen:

```python
# launch/params_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Parameter file path hasil karen
    param_file = os.path.join(
        get_package_share_directory('my_robot_pkg'),
        'config',
        'robot_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='parameter_node',
            parameters=[param_file],
            name='robot_controller'
        )
    ])
```

## Command-Line Parameter Tools

Parameter management ke liye useful CLI commands:

```bash
# Sabhi parameters list karen
ros2 param list

# Parameter value hasil karen
ros2 param get /node_name parameter_name

# Parameter value set karen
ros2 param set /node_name parameter_name new_value

# Sabhi parameters ko file mein dump karen
ros2 param dump /node_name --output params.yaml

# File se parameters load karen
ros2 param load /node_name params.yaml
```

## Best Practices

1. **Configuration ke liye parameters ka istemal karen**, na ke data streaming ke liye
2. **Related parameters ko meaningful prefixes ke neechay group karen**
3. **Parameter values ko callbacks mein validate karen**
4. **Meaningful descriptions ke saath parameters ko document karen**
5. **Performance zaroori honay pe composable nodes ka istemal karen**
6. **Complex initialization sequences ke liye lifecycle nodes ka soch samjh kar istemal karen**

---

Continue to [Advanced Communication Patterns](/docs/module-1-ros2/advanced-communication) to learn about custom message types and advanced communication architectures.