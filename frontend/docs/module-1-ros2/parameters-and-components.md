---
sidebar_position: 5
title: Parameters and Components
description: Manage configurations and create lightweight composable nodes
---

# Parameters and Components

In this chapter, you'll learn about parameter management and composable nodes in ROS 2.

## Learning Objectives

- Use parameters for node configuration
- Implement parameter callbacks
- Create and use composable nodes
- Understand lifecycle nodes

## Parameters

**Parameters** are named values that can be configured at runtime. They're perfect for configuration settings that might change between deployments.

### Setting Parameters

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('param_node')

        # Declare parameters with default values
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('wheel_diameter', 0.3)
        self.declare_parameter('robot_name', 'my_robot')

        # Get parameter values
        self.max_speed = self.get_parameter('max_speed').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.robot_name = self.get_parameter('robot_name').value

        self.get_logger().info(f'Robot: {self.robot_name}, Max speed: {self.max_speed}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()

    # You can change parameters at runtime
    node.set_parameters([rclpy.parameter.Parameter('max_speed', rclpy.Parameter.Type.DOUBLE, 2.0)])

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Parameter Descriptors

You can add constraints to parameters:

```python
from rclpy.node import ParameterDescriptor
from rclpy.parameter import ParameterType

class ConstrainedParamNode(Node):
    def __init__(self):
        super().__init__('constrained_param_node')

        # Descriptor with constraints
        desc = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Maximum robot speed in m/s',
            additional_constraints='Must be positive and less than 10.0',
            floating_point_range=[(0.0, 10.0, '')]  # min, max, name
        )

        self.declare_parameter('max_speed', 1.0, descriptor=desc)
```

### Parameter Callbacks

Monitor parameter changes:

```python
from rclpy.parameter import Parameter

class ParamCallbackNode(Node):
    def __init__(self):
        super().__init__('param_callback_node')
        self.declare_parameter('threshold', 0.5)

        # Register callback for parameter changes
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

Composable nodes allow multiple nodes to run within a single process, reducing overhead and improving performance.

### Creating a Component

```python
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from std_msgs.msg import String

class MyComponent(Node):
    def __init__(self, name):
        super().__init__(name)

        # Declare parameters
        self.declare_parameter('frequency', 1.0)

        # Create publisher and timer
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

# Register the component
from rclpy_components.register_node import register_node
register_node(MyComponent)
```

### Composition Container

Create a container to manage components:

```python
from rclpy_components.component_manager import ComponentManager
import rclpy

def main(args=None):
    rclpy.init(args=args)

    # Create a component manager
    manager = ComponentManager('composition_container')

    # Load components dynamically
    try:
        # Load component by class type
        node_instance = manager.add_node_by_class('my_package', 'MyComponent')

        # Configure parameters for the component
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

Lifecycle nodes provide a state machine for managing node initialization, activation, and cleanup.

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

    # Start in unconfigured state
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
```

## Working with Parameter Files

Store parameters in YAML files:

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

Load parameters in launch files:

```python
# launch/params_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get parameter file path
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

Useful CLI commands for parameter management:

```bash
# List all parameters
ros2 param list

# Get parameter value
ros2 param get /node_name parameter_name

# Set parameter value
ros2 param set /node_name parameter_name new_value

# Dump all parameters to a file
ros2 param dump /node_name --output params.yaml

# Load parameters from a file
ros2 param load /node_name params.yaml
```

## Best Practices

1. **Use parameters for configuration**, not for data streaming
2. **Group related parameters** under meaningful prefixes
3. **Validate parameter values** in callbacks
4. **Document parameters** with meaningful descriptions
5. **Use composable nodes** when performance is critical
6. **Consider lifecycle nodes** for complex initialization sequences

---

Continue to [Advanced Communication Patterns](/docs/module-1-ros2/advanced-communication) to learn about custom message types and advanced communication architectures.