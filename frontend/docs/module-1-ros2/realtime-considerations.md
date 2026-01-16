---
sidebar_position: 7
title: Real-Time Considerations
description: Time-critical robotic applications and real-time performance
---

# Real-Time Considerations

In this chapter, you'll learn about real-time performance considerations for time-critical robotic applications in ROS 2.

## Learning Objectives

- Understand real-time constraints in robotics
- Configure ROS 2 for real-time performance
- Optimize communication for low latency
- Implement real-time safe code patterns

## Real-Time Concepts in Robotics

Real-time systems must respond to events within strict time constraints. In robotics, this is crucial for:

- **Control loops**: Robot controllers often need to run at fixed frequencies (100Hz, 1kHz)
- **Safety systems**: Emergency stops and collision avoidance must respond immediately
- **Sensor fusion**: Combining data from multiple sensors with precise timing
- **Motion planning**: Trajectory execution requires precise timing

### Hard vs Soft Real-Time

- **Hard real-time**: Missing deadlines causes system failure (safety-critical systems)
- **Soft real-time**: Missing deadlines degrades performance but doesn't cause failure (most robotic applications)

## Real-Time Kernel Configuration

For hard real-time applications, configure your system:

### Installing RT Kernel (Ubuntu)

```bash
# Install real-time kernel
sudo apt update
sudo apt install linux-image-rt-generic

# Reboot and select RT kernel from GRUB menu
sudo reboot
```

### Real-Time Privileges

Add your user to real-time groups:

```bash
sudo usermod -a -G realtime $USER
# Log out and back in for changes to take effect
```

## Real-Time Node Implementation

### Real-Time Safe Timers

```python
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
import threading
import time

class RealTimeController(Node):
    def __init__(self):
        super().__init__('realtime_controller')

        # Set real-time priority for this node
        self.set_realtime_priority()

        # Create publisher for control commands
        self.cmd_pub = self.create_publisher(Float64MultiArray, 'joint_commands', 1)

        # Create high-frequency timer (1kHz)
        self.control_timer = self.create_timer(
            0.001,  # 1ms = 1000Hz
            self.control_loop,
            clock=self.get_clock()
        )

        self.joint_positions = [0.0] * 6  # 6-DOF robot
        self.control_counter = 0

    def set_realtime_priority(self):
        """Set real-time scheduling priority for the node"""
        try:
            import os
            import ctypes
            from ctypes import util

            # Try to set real-time priority using sched_setscheduler
            libc = ctypes.CDLL(util.find_library("c"))

            # SCHED_FIFO with priority 80 (out of 99)
            policy = 1  # SCHED_FIFO
            priority = 80

            sched_param = ctypes.c_int(priority)
            result = libc.sched_setscheduler(
                os.getpid(),
                policy,
                ctypes.byref(sched_param)
            )

            if result != 0:
                self.get_logger().warn('Could not set real-time priority')
            else:
                self.get_logger().info(f'Set real-time priority to {priority}')

        except Exception as e:
            self.get_logger().warn(f'Could not set real-time priority: {e}')

    def control_loop(self):
        """Real-time control loop"""
        start_time = self.get_clock().now()

        # Minimal computation in real-time loop
        self.update_control_commands()

        # Publish control commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = self.joint_positions
        self.cmd_pub.publish(cmd_msg)

        self.control_counter += 1

        # Monitor timing
        elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e6  # ms
        if elapsed > 1.0:  # More than 1ms taken
            self.get_logger().warn(f'Control loop exceeded deadline: {elapsed:.2f}ms')

    def update_control_commands(self):
        """Minimal control algorithm - keep this function fast!"""
        # Simple sinusoidal trajectory
        t = self.get_clock().now().nanoseconds / 1e9  # Convert to seconds
        for i in range(len(self.joint_positions)):
            self.joint_positions[i] = 0.5 * math.sin(t * 2 * math.pi * 0.5 + i)

def main(args=None):
    rclpy.init(args=args)

    # Use real-time safe executor
    executor = SingleThreadedExecutor()

    node = RealTimeController()
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Zero-Copy Message Passing

Reduce memory allocation overhead for high-frequency data:

```python
from rclpy.qos import QoSProfile, HistoryPolicy
from rclpy.publisher import Publisher
import numpy as np

class ZeroCopyPublisher(Node):
    def __init__(self):
        super().__init__('zero_copy_publisher')

        # Configure for zero-copy transport
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.publisher = self.create_publisher(SensorMsgs, 'sensor_data', qos_profile)

        # Pre-allocate message buffer
        self.msg_buffer = SensorMsgs()
        self.sensor_data = np.zeros(1000, dtype=np.float32)  # Pre-allocated data

    def publish_sensor_data(self):
        # Fill pre-allocated message
        self.msg_buffer.header.stamp = self.get_clock().now().to_msg()
        self.msg_buffer.header.frame_id = 'sensor_frame'

        # Update sensor data in-place
        self.update_sensor_data()

        # Publish without copying if possible
        self.publisher.publish(self.msg_buffer)

    def update_sensor_data(self):
        """Fill sensor data efficiently"""
        # Use in-place operations
        np.sin(np.arange(1000) * 0.01, out=self.sensor_data)
        self.msg_buffer.data = self.sensor_data.tolist()
```

## Lock-Free Data Structures

Use thread-safe, lock-free data structures for inter-node communication:

```python
import queue
import threading
from collections import deque
from rclpy.node import Node

class LockFreeBufferNode(Node):
    def __init__(self):
        super().__init__('lockfree_buffer_node')

        # Use lock-free queues for inter-thread communication
        self.data_queue = queue.Queue(maxsize=100)
        self.result_queue = queue.Queue(maxsize=100)

        # Subscriber in separate thread
        self.sub_thread = threading.Thread(target=self.subscribe_thread)
        self.sub_thread.daemon = True
        self.sub_thread.start()

        # Processing in main thread
        self.process_timer = self.create_timer(0.01, self.process_data)

    def subscribe_thread(self):
        """Dedicated thread for subscription"""
        while rclpy.ok():
            try:
                # This would normally be a subscription callback
                # Simulating incoming data
                data = self.simulate_sensor_data()

                try:
                    self.data_queue.put_nowait(data)
                except queue.Full:
                    self.get_logger().warn('Data queue full, dropping packet')

            except Exception as e:
                self.get_logger().error(f'Subscription thread error: {e}')

    def process_data(self):
        """Process data in main thread"""
        try:
            data = self.data_queue.get_nowait()

            # Process data
            result = self.process_sensor_data(data)

            # Put result in output queue
            try:
                self.result_queue.put_nowait(result)
            except queue.Full:
                self.get_logger().warn('Result queue full')

        except queue.Empty:
            pass  # No data to process

    def simulate_sensor_data(self):
        """Simulate incoming sensor data"""
        return {'timestamp': time.time(), 'values': [1.0, 2.0, 3.0]}

    def process_sensor_data(self, data):
        """Process sensor data"""
        # Real-time safe processing
        return {'processed': True, 'avg': sum(data['values']) / len(data['values'])}
```

## Memory Management

Minimize dynamic memory allocation in real-time paths:

```python
from rclpy.node import Node
import numpy as np

class MemoryEfficientNode(Node):
    def __init__(self):
        super().__init__('memory_efficient_node')

        # Pre-allocate all buffers
        self.input_buffer = np.zeros(1000, dtype=np.float32)
        self.output_buffer = np.zeros(1000, dtype=np.float32)
        self.temp_buffer = np.zeros(1000, dtype=np.float32)

        self.publisher = self.create_publisher(Float32MultiArray, 'filtered_data', 1)
        self.subscriber = self.create_subscription(
            Float32MultiArray, 'raw_data', self.data_callback, 1
        )

        # Pre-allocate message objects
        self.output_msg = Float32MultiArray()

    def data_callback(self, msg):
        """Real-time safe callback"""
        # Copy data to pre-allocated buffer
        if len(msg.data) <= len(self.input_buffer):
            self.input_buffer[:len(msg.data)] = msg.data
            self.input_buffer[len(msg.data):] = 0  # Zero-pad remaining

            # Process in real-time safe manner
            self.filter_data()

            # Publish result
            self.output_msg.data = self.output_buffer[:len(msg.data)].tolist()
            self.publisher.publish(self.output_msg)

    def filter_data(self):
        """Real-time safe filtering operation"""
        # Use pre-allocated arrays only
        np.copyto(self.temp_buffer, self.input_buffer)
        np.multiply(self.temp_buffer, 0.9, out=self.output_buffer)  # IIR filter
```

## Real-Time Synchronization

Coordinate between real-time and non-real-time threads:

```python
import threading
from threading import Condition
import time

class SynchronizedRealTimeNode(Node):
    def __init__(self):
        super().__init__('sync_rt_node')

        # Thread synchronization
        self.rt_condition = Condition()
        self.non_rt_condition = Condition()

        self.rt_ready = False
        self.non_rt_ready = False
        self.cycle_count = 0

        # Real-time thread
        self.rt_thread = threading.Thread(target=self.realtime_thread)
        self.rt_thread.daemon = True
        self.rt_thread.start()

        # Non-real-time thread for logging/debugging
        self.non_rt_thread = threading.Thread(target=self.non_rt_thread)
        self.non_rt_thread.daemon = True
        self.non_rt_thread.start()

    def realtime_thread(self):
        """High-priority real-time thread"""
        # Set real-time priority
        self.set_realtime_priority()

        while rclpy.ok():
            with self.rt_condition:
                # Do real-time work
                self.do_realtime_work()

                # Signal non-RT thread
                self.rt_ready = True
                self.rt_condition.notify()

                # Wait for non-RT to finish
                while self.non_rt_ready:
                    self.rt_condition.wait()

    def non_rt_thread(self):
        """Non-real-time thread for logging, etc."""
        while rclpy.ok():
            with self.non_rt_condition:
                # Wait for RT thread to signal
                while not self.rt_ready:
                    self.non_rt_condition.wait()

                # Do non-RT work (logging, etc.)
                self.do_non_rt_work()

                # Signal RT thread
                self.non_rt_ready = False
                self.non_rt_condition.notify()

    def do_realtime_work(self):
        """Real-time critical work"""
        # Minimal, deterministic work here
        self.cycle_count += 1
        time.sleep(0.001)  # Simulate control cycle

    def do_non_rt_work(self):
        """Non-real-time work"""
        # Logging, diagnostics, etc. can go here
        if self.cycle_count % 100 == 0:
            self.get_logger().info(f'Processed {self.cycle_count} cycles')
```

## Performance Monitoring

Monitor real-time performance:

```python
from rclpy.node import Node
import statistics

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('perf_monitor')

        self.cycle_times = []
        self.deadline_misses = 0
        self.total_cycles = 0

        # Monitor timer performance
        self.period = 0.001  # 1ms
        self.timer = self.create_timer(self.period, self.timed_callback)

        # Performance monitoring timer
        self.monitor_timer = self.create_timer(5.0, self.report_performance)

    def timed_callback(self):
        start_time = time.perf_counter()

        # Do work
        self.do_work()

        end_time = time.perf_counter()
        cycle_time = (end_time - start_time) * 1000  # ms

        self.cycle_times.append(cycle_time)
        self.total_cycles += 1

        # Check for deadline miss (cycle took longer than period)
        if cycle_time > (self.period * 1000):  # Convert to ms
            self.deadline_misses += 1

    def do_work(self):
        """Work to be monitored"""
        # Simulate some processing
        time.sleep(0.0005)  # 0.5ms of "work"

    def report_performance(self):
        if self.cycle_times:
            avg_time = statistics.mean(self.cycle_times)
            max_time = max(self.cycle_times)
            min_time = min(self.cycle_times)

            if len(self.cycle_times) > 1:
                std_dev = statistics.stdev(self.cycle_times)
            else:
                std_dev = 0

            miss_rate = (self.deadline_misses / self.total_cycles) * 100 if self.total_cycles > 0 else 0

            self.get_logger().info(
                f'Performance Report:\n'
                f'  Avg cycle time: {avg_time:.3f}ms\n'
                f'  Min/Max: {min_time:.3f}/{max_time:.3f}ms\n'
                f'  Std Dev: {std_dev:.3f}ms\n'
                f'  Deadline misses: {self.deadline_misses}/{self.total_cycles} ({miss_rate:.2f}%)\n'
            )

            # Keep only recent measurements
            if len(self.cycle_times) > 1000:
                self.cycle_times = self.cycle_times[-500:]
```

## Best Practices for Real-Time ROS 2

1. **Keep real-time paths minimal** - avoid complex computations in critical loops
2. **Pre-allocate memory** - avoid dynamic allocation in real-time paths
3. **Use appropriate QoS** - configure for your timing requirements
4. **Separate RT and non-RT work** - use threads for non-critical tasks
5. **Monitor performance** - track timing metrics continuously
6. **Test under load** - verify performance with realistic workloads
7. **Consider hardware** - use deterministic hardware when possible
8. **Profile your code** - identify bottlenecks in your implementation

## Troubleshooting Real-Time Issues

Common real-time problems and solutions:

- **Deadline misses**: Reduce computational load or increase period
- **Jitter**: Check for sources of non-determinism (dynamic allocation, locks)
- **System interference**: Use CPU isolation, disable power management
- **Memory allocation**: Pre-allocate all needed memory

---

Congratulations! You've completed all advanced topics in Module 1. Continue to [Module 2: Simulation](/docs/module-2-simulation) to learn how to test your ROS 2 systems in virtual environments.