---
sidebar_position: 3
title: Services and Actions
description: Implement synchronous and asynchronous robot behaviors
---

# Services and Actions

While topics are great for streaming data, sometimes you need request-response communication. That's where services and actions come in.

## Learning Objectives

- Create and call ROS 2 services
- Understand when to use services vs topics
- Implement action servers and clients
- Handle long-running tasks with feedback

## Services

A **service** is a synchronous request-response mechanism. The client sends a request and waits for a response.

### When to Use Services

| Use Case | Topics | Services |
|----------|--------|----------|
| Sensor data streaming | Yes | No |
| Robot state queries | No | Yes |
| Configuration changes | No | Yes |
| Continuous commands | Yes | No |
| One-time operations | No | Yes |

### Defining a Service

Create a custom service interface `srv/AddTwoInts.srv`:

```
int64 a
int64 b
---
int64 sum
```

### Service Server

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class AdditionServer(Node):
    def __init__(self):
        super().__init__('addition_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback
        )
        self.get_logger().info('Service ready')

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response

def main():
    rclpy.init()
    node = AdditionServer()
    rclpy.spin(node)
    rclpy.shutdown()
```

### Service Client

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class AdditionClient(Node):
    def __init__(self):
        super().__init__('addition_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result()

def main():
    rclpy.init()
    client = AdditionClient()

    result = client.send_request(2, 3)
    print(f'Result: {result.sum}')

    rclpy.shutdown()
```

### Call Service from CLI

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"
```

## Actions

**Actions** are for long-running tasks that need feedback and can be cancelled.

### Action Structure

```
# Goal (request)
int32 order
---
# Result (response)
int32[] sequence
---
# Feedback (during execution)
int32[] partial_sequence
```

### Action Server

```python
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciServer(Node):
    def __init__(self):
        super().__init__('fibonacci_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        sequence = [0, 1]
        feedback_msg = Fibonacci.Feedback()

        for i in range(1, goal_handle.request.order):
            # Check if cancelled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()

            # Compute next number
            sequence.append(sequence[i] + sequence[i-1])

            # Send feedback
            feedback_msg.partial_sequence = sequence
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(1)  # Simulate work

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = sequence
        return result

def main():
    rclpy.init()
    node = FibonacciServer()
    rclpy.spin(node)
    rclpy.shutdown()
```

### Action Client

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciClient(Node):
    def __init__(self):
        super().__init__('fibonacci_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        sequence = feedback_msg.feedback.partial_sequence
        self.get_logger().info(f'Feedback: {sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

def main():
    rclpy.init()
    client = FibonacciClient()
    client.send_goal(10)
    rclpy.spin(client)
```

## Comparison: Topics vs Services vs Actions

| Feature | Topics | Services | Actions |
|---------|--------|----------|---------|
| Communication | Pub/Sub | Request/Response | Goal/Feedback/Result |
| Blocking | No | Yes | No |
| Cancellation | N/A | No | Yes |
| Feedback | N/A | No | Yes |
| Use Case | Streaming | Quick operations | Long tasks |

## Practical Example: Robot Arm Control

```python
# Action definition for moving a robot arm
# MoveArm.action

# Goal
geometry_msgs/Pose target_pose
float64 max_velocity
---
# Result
bool success
string message
---
# Feedback
float64 progress_percent
geometry_msgs/Pose current_pose
```

## Exercise

Create a timer service that:

1. Takes a duration in seconds
2. Waits for that duration
3. Returns "Timer complete!"

<details>
<summary>Solution</summary>

```python
import time
from example_interfaces.srv import SetBool
import rclpy
from rclpy.node import Node

class TimerService(Node):
    def __init__(self):
        super().__init__('timer_service')
        self.srv = self.create_service(
            SetBool,  # Using SetBool as simple example
            'wait_timer',
            self.timer_callback
        )

    def timer_callback(self, request, response):
        # request.data is the duration in this case
        duration = 5 if request.data else 2
        self.get_logger().info(f'Waiting {duration} seconds...')
        time.sleep(duration)
        response.success = True
        response.message = 'Timer complete!'
        return response
```

</details>

---

Next, learn about [URDF Basics](/docs/module-1-ros2/urdf-basics) to define your robot's structure.
