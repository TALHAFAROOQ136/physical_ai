---
sidebar_position: 6
title: Advanced Communication Patterns
description: Custom message types and advanced communication architectures
---

# Advanced Communication Patterns

In this chapter, you'll learn about custom message types and advanced communication patterns in ROS 2.

## Learning Objectives

- Create custom message, service, and action definitions
- Implement advanced communication patterns
- Use shared memory and zero-copy transport
- Understand advanced QoS configurations

## Custom Message Types

Custom messages allow you to define your own data structures for communication.

### Creating Custom Messages

Create a custom message file `msg/RobotState.msg`:

```
# Custom robot state message
Header header
float64 x
float64 y
float64 theta
float64 linear_velocity
float64 angular_velocity
uint8 battery_level
bool emergency_stop
geometry_msgs/Pose pose
sensor_msgs/BatteryState battery_state
```

### Using Custom Messages

```python
# Publisher with custom message
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from my_robot_msgs.msg import RobotState  # Your custom message
from geometry_msgs.msg import Pose
from sensor_msgs.msg import BatteryState

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        self.publisher = self.create_publisher(RobotState, 'robot_state', 10)
        self.timer = self.create_timer(0.1, self.publish_state)

    def publish_state(self):
        msg = RobotState()

        # Set header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Set robot state
        msg.x = 1.0
        msg.y = 2.0
        msg.theta = 0.5
        msg.linear_velocity = 0.5
        msg.angular_velocity = 0.2
        msg.battery_level = 85
        msg.emergency_stop = False

        # Set nested messages
        msg.pose.position.x = msg.x
        msg.pose.position.y = msg.y
        msg.pose.orientation.z = msg.theta

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Custom Services

Create custom services for specific request-response patterns:

Create `srv/MoveToPosition.srv`:
```
# Request
float64 x
float64 y
float64 theta
float64 max_velocity
---
# Response
bool success
string message
float64 actual_distance
```

Implementation:
```python
# Service server
from my_robot_msgs.srv import MoveToPosition

class MoveToPositionServer(Node):
    def __init__(self):
        super().__init__('move_to_position_server')
        self.srv = self.create_service(
            MoveToPosition,
            'move_to_position',
            self.move_callback
        )

    def move_callback(self, request, response):
        # Calculate distance to target
        current_pos = self.get_current_position()
        distance = ((request.x - current_pos.x)**2 +
                   (request.y - current_pos.y)**2)**0.5

        # Execute movement (simplified)
        success = self.execute_movement(request)

        response.success = success
        response.actual_distance = distance
        response.message = f'Moved to ({request.x}, {request.y})' if success else 'Movement failed'

        return response
```

## Custom Actions

Create complex actions with detailed feedback:

Create `action/Navigate.action`:
```
# Goal
float64 target_x
float64 target_y
float64 tolerance
---
# Result
bool success
float64 final_x
float64 final_y
int32 path_length
string message
---
# Feedback
float64 current_x
float64 current_y
float64 distance_remaining
float64 progress_percentage
bool obstacle_detected
```

Action server implementation:
```python
from my_robot_msgs.action import Navigate
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')

        # Use reentrant callback group to handle multiple goals
        callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            Navigate,
            'navigate_to_pose',
            self.execute_callback,
            callback_group=callback_group
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing navigation goal...')

        target_x = goal_handle.request.target_x
        target_y = goal_handle.request.target_y
        tolerance = goal_handle.request.tolerance

        feedback_msg = Navigate.Feedback()
        result = Navigate.Result()

        # Navigation loop
        while not goal_handle.is_cancel_requested:
            current_pos = self.get_current_position()

            # Calculate distance to goal
            dist = ((target_x - current_pos.x)**2 +
                   (target_y - current_pos.y)**2)**0.5

            # Check if reached
            if dist <= tolerance:
                result.success = True
                result.final_x = current_pos.x
                result.final_y = current_pos.y
                result.message = 'Successfully reached destination'
                goal_handle.succeed()
                return result

            # Publish feedback
            feedback_msg.current_x = current_pos.x
            feedback_msg.current_y = current_pos.y
            feedback_msg.distance_remaining = dist
            feedback_msg.progress_percentage = (1.0 - dist/max_start_dist) * 100.0
            feedback_msg.obstacle_detected = self.detect_obstacle()

            goal_handle.publish_feedback(feedback_msg)

            # Move towards goal
            self.move_towards(target_x, target_y)

            # Sleep briefly
            self.get_clock().sleep_for(Duration(seconds=0.1))

        # Handle cancellation
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = 'Navigation canceled'
            return result

def main(args=None):
    rclpy.init(args=args)
    node = NavigationActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Advanced QoS Configurations

Fine-tune communication behavior with advanced QoS settings:

```python
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
    Lifespan,
    Deadline,
    LivelinessPolicy
)

class AdvancedQoSPublisher(Node):
    def __init__(self):
        super().__init__('advanced_qos_publisher')

        # High-reliability profile for critical data
        reliable_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_ALL,
            depth=1
        )

        # Low-latency profile for real-time data
        real_time_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            deadline=Duration(seconds=0.1),  # Must arrive within 100ms
            lifespan=Duration(seconds=0.5),  # Expire after 500ms
            liveliness=LivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration(seconds=1.0)
        )

        # Critical safety messages
        self.safety_pub = self.create_publisher(
            String, 'safety_critical', reliable_profile
        )

        # Real-time sensor data
        self.sensor_pub = self.create_publisher(
            String, 'sensor_data', real_time_profile
        )

class AdvancedQoSSubscriber(Node):
    def __init__(self):
        super().__init__('advanced_qos_subscriber')

        # Match the publisher's QoS for safety messages
        reliable_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_ALL,
            depth=1
        )

        self.safety_sub = self.create_subscription(
            String, 'safety_critical', self.safety_callback, reliable_profile
        )

    def safety_callback(self, msg):
        self.get_logger().info(f'Safety message: {msg.data}')
```

## Shared Memory Transport

For high-performance applications, use shared memory transport:

```python
from rclpy.qos import QoSProfile, HistoryPolicy
from rclpy.memory_strategies import PYBIND_MEMORY_STRATEGY

class SharedMemoryNode(Node):
    def __init__(self):
        super().__init__('shared_memory_node')

        # Configure for shared memory
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            # Note: Shared memory is handled at the DDS level
            # Configure your RMW to use shared memory transport
        )

        self.image_pub = self.create_publisher(Image, 'camera/image_raw', qos_profile)
        self.pointcloud_pub = self.create_publisher(PointCloud2, 'lidar/points', qos_profile)
```

## Communication Patterns

### Publisher-Subscriber with Filters

```python
class FilteredSubscriber(Node):
    def __init__(self):
        super().__init__('filtered_subscriber')

        # Subscribe to raw data
        self.raw_sub = self.create_subscription(
            LaserScan, 'scan_raw', self.raw_scan_callback, 10
        )

        # Publish filtered data
        self.filtered_pub = self.create_publisher(
            LaserScan, 'scan_filtered', 10
        )

        # Filter parameters
        self.min_range = 0.3
        self.max_range = 10.0

    def raw_scan_callback(self, msg):
        # Filter out invalid ranges
        filtered_ranges = []
        for r in msg.ranges:
            if self.min_range <= r <= self.max_range:
                filtered_ranges.append(r)
            else:
                filtered_ranges.append(float('inf'))  # Invalid range

        # Create and publish filtered message
        filtered_msg = msg
        filtered_msg.ranges = filtered_ranges
        self.filtered_pub.publish(filtered_msg)
```

### Request-Reply with Timeout

```python
import asyncio
from rclpy.action import ActionClient

class TimeoutClient(Node):
    def __init__(self):
        super().__init__('timeout_client')
        self.action_client = ActionClient(self, Navigate, 'navigate_to_pose')

    async def send_goal_with_timeout(self, goal, timeout_seconds=30.0):
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            raise RuntimeError('Action server not available')

        # Send goal
        goal_future = await self.action_client.send_goal_async(goal)

        # Wait with timeout
        try:
            result = await asyncio.wait_for(
                goal_future,
                timeout=timeout_seconds
            )
            return result.result()
        except asyncio.TimeoutError:
            # Cancel the goal
            if goal_future.done():
                goal_handle = goal_future.result()
                await goal_handle.cancel_goal_async()
            raise TimeoutError(f'Action timed out after {timeout_seconds}s')
```

## Best Practices for Advanced Communication

1. **Use custom messages** for domain-specific data
2. **Match QoS profiles** between publishers and subscribers
3. **Consider message size** - large messages impact performance
4. **Use TRANSIENT_LOCAL** for static data that late-joining nodes need
5. **Implement proper error handling** for all communication patterns
6. **Use actions for complex, cancellable operations**
7. **Consider zero-copy transport** for high-frequency data
8. **Profile your communication patterns** to identify bottlenecks

## Exercises

1. Create a custom message for robot joint states with position, velocity, and effort for each joint
2. Implement a service that calculates the shortest path between two points
3. Design an action that performs a complete mapping operation with feedback

---

Next, learn about [Real-Time Considerations](/docs/module-1-ros2/realtime-considerations) for time-critical robotic applications.