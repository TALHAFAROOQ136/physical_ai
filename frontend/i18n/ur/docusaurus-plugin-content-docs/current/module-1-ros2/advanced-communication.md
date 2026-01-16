---
sidebar_position: 6
title: Advanced Communication Patterns
description: Custom message types aur advanced communication architectures
---

# Advanced Communication Patterns

Iss chapter mein, aap seekhenge ke ROS 2 mein custom message types aur advanced communication patterns ke bare mein.

## Seeking Objectives

- Apne message types banayein aur istemal karen
- Advanced communication patterns laagoo karen
- Shared memory aur zero-copy transport ka istemal karen
- Advanced QoS configurations ko samjhein

## Custom Message Types

Custom messages aap ko apne data structures ko define karne ki ijazat dete hain communication ke liye.

### Custom Messages Banana

Ek custom message file banayein `msg/RobotState.msg`:

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

### Custom Messages Ka Istemal

```python
# Publisher with custom message
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from my_robot_msgs.msg import RobotState  # Aap ka custom message
from geometry_msgs.msg import Pose
from sensor_msgs.msg import BatteryState

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        self.publisher = self.create_publisher(RobotState, 'robot_state', 10)
        self.timer = self.create_timer(0.1, self.publish_state)

    def publish_state(self):
        msg = RobotState()

        # Header set karen
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Robot state set karen
        msg.x = 1.0
        msg.y = 2.0
        msg.theta = 0.5
        msg.linear_velocity = 0.5
        msg.angular_velocity = 0.2
        msg.battery_level = 85
        msg.emergency_stop = False

        # Nested messages set karen
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

Specific request-response patterns ke liye custom services banana:

`srv/MoveToPosition.srv` banana:
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
        # Target tak distance calculate karen
        current_pos = self.get_current_position()
        distance = ((request.x - current_pos.x)**2 +
                   (request.y - current_pos.y)**2)**0.5

        # Movement execute karen (simplified)
        success = self.execute_movement(request)

        response.success = success
        response.actual_distance = distance
        response.message = f'Moved to ({request.x}, {request.y})' if success else 'Movement failed'

        return response
```

## Custom Actions

Detailed feedback ke saath complex actions banana:

`action/Navigate.action` banana:
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

        # Multiple goals handle karne ke liye reentrant callback group ka istemal karen
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

            # Goal tak distance calculate karen
            dist = ((target_x - current_pos.x)**2 +
                   (target_y - current_pos.y)**2)**0.5

            # Check karen ke pohunch gaye kya
            if dist <= tolerance:
                result.success = True
                result.final_x = current_pos.x
                result.final_y = current_pos.y
                result.message = 'Successfully reached destination'
                goal_handle.succeed()
                return result

            # Feedback publish karen
            feedback_msg.current_x = current_pos.x
            feedback_msg.current_y = current_pos.y
            feedback_msg.distance_remaining = dist
            feedback_msg.progress_percentage = (1.0 - dist/max_start_dist) * 100.0
            feedback_msg.obstacle_detected = self.detect_obstacle()

            goal_handle.publish_feedback(feedback_msg)

            # Goal ki taraf move karen
            self.move_towards(target_x, target_y)

            # Thora sleep karen
            self.get_clock().sleep_for(Duration(seconds=0.1))

        # Cancellation handle karen
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

Communication behavior ko fine-tune karen advanced QoS settings ke saath:

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

        # Critical data ke liye high-reliability profile
        reliable_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_ALL,
            depth=1
        )

        # Real-time data ke liye low-latency profile
        real_time_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            deadline=Duration(seconds=0.1),  # 100ms ke andhar aana chahiye
            lifespan=Duration(seconds=0.5),  # 500ms ke baad expire ho jaye
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

        # Safety messages ke liye publisher ke QoS ko match karen
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

High-performance applications ke liye, shared memory transport ka istemal karen:

```python
from rclpy.qos import QoSProfile, HistoryPolicy
from rclpy.memory_strategies import PYBIND_MEMORY_STRATEGY

class SharedMemoryNode(Node):
    def __init__(self):
        super().__init__('shared_memory_node')

        # Shared memory ke liye configure karen
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            # Note: Shared memory DDS level par handled hota hai
            # Apna RMW ko shared memory transport use karne ke liye configure karen
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

        # Raw data ke liye subscribe karen
        self.raw_sub = self.create_subscription(
            LaserScan, 'scan_raw', self.raw_scan_callback, 10
        )

        # Filtered data publish karen
        self.filtered_pub = self.create_publisher(
            LaserScan, 'scan_filtered', 10
        )

        # Filter parameters
        self.min_range = 0.3
        self.max_range = 10.0

    def raw_scan_callback(self, msg):
        # Invalid ranges ko filter karen
        filtered_ranges = []
        for r in msg.ranges:
            if self.min_range <= r <= self.max_range:
                filtered_ranges.append(r)
            else:
                filtered_ranges.append(float('inf'))  # Invalid range

        # Filtered message banayein aur publish karen
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

        # Goal bhejein
        goal_future = await self.action_client.send_goal_async(goal)

        # Timeout ke saath wait karen
        try:
            result = await asyncio.wait_for(
                goal_future,
                timeout=timeout_seconds
            )
            return result.result()
        except asyncio.TimeoutError:
            # Goal cancel karen
            if goal_future.done():
                goal_handle = goal_future.result()
                await goal_handle.cancel_goal_async()
            raise TimeoutError(f'Action timed out after {timeout_seconds}s')
```

## Best Practices for Advanced Communication

1. **Domain-specific data ke liye custom messages ka istemal karen**
2. **Publishers aur subscribers ke beech QoS profiles match karen**
3. **Message size ko dhyan rakhen** - bade messages performance par asar daalte hain
4. **TRANSIENT_LOCAL ko static data ke liye istemal karen jo late-joining nodes ko zaroorat hoti hai**
5. **Communication patterns ke liye proper error handling laagoo karen**
6. **Complex, cancellable operations ke liye actions ka istemal karen**
7. **High-frequency data ke liye zero-copy transport ka soch samjh kar istemal karen**
8. **Bottlenecks ko identify karne ke liye apni communication patterns profile karen**

## Exercises

1. Har joint ke liye position, velocity, aur effort ke saath robot joint states ke liye custom message banana
2. Do points ke beech shortest path calculate karne wala service laagoo karen
3. Complete mapping operation perform karne wala action design karen feedback ke saath

---

Next, learn about [Real-Time Considerations](/docs/module-1-ros2/realtime-considerations) for time-critical robotic applications.