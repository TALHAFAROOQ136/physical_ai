---
sidebar_position: 3
title: Services اور Actions
description: synchronous اور asynchronous robot behaviors implement کریں
---

# Services اور Actions

جبکہ topics streaming data کے لیے بہترین ہیں، کبھی کبھی آپ کو request-response communication کی ضرورت ہوتی ہے۔ یہی جگہ services اور actions کام آتے ہیں۔

## سیکھنے کے مقاصد

- ROS 2 services بنائیں اور call کریں
- سمجھیں کہ services کب استعمال کریں بمقابلہ topics
- action servers اور clients implement کریں
- feedback کے ساتھ طویل کاموں کو handle کریں

## Services

ایک **service** ایک synchronous request-response mechanism ہے۔ Client ایک request بھیجتا ہے اور response کا انتظار کرتا ہے۔

### Services کب استعمال کریں

| Use Case | Topics | Services |
|----------|--------|----------|
| Sensor data streaming | ہاں | نہیں |
| Robot state queries | نہیں | ہاں |
| Configuration changes | نہیں | ہاں |
| Continuous commands | ہاں | نہیں |
| One-time operations | نہیں | ہاں |

### Service Define کرنا

ایک custom service interface `srv/AddTwoInts.srv` بنائیں:

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

### CLI سے Service Call کریں

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"
```

## Actions

**Actions** طویل کاموں کے لیے ہیں جنہیں feedback کی ضرورت ہو اور cancel کیا جا سکے۔

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

## موازنہ: Topics بمقابلہ Services بمقابلہ Actions

| Feature | Topics | Services | Actions |
|---------|--------|----------|---------|
| Communication | Pub/Sub | Request/Response | Goal/Feedback/Result |
| Blocking | نہیں | ہاں | نہیں |
| Cancellation | N/A | نہیں | ہاں |
| Feedback | N/A | نہیں | ہاں |
| Use Case | Streaming | Quick operations | طویل کام |

## عملی مثال: Robot Arm Control

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

## مشق

ایک timer service بنائیں جو:

1. سیکنڈز میں duration لے
2. اس duration کا انتظار کرے
3. "Timer complete!" واپس کرے

<details>
<summary>حل</summary>

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

اگلا، اپنے robot کی structure define کرنے کے لیے [URDF Basics](/docs/module-1-ros2/urdf-basics) سیکھیں۔
