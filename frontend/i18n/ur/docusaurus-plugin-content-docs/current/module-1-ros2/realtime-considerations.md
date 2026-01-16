---
sidebar_position: 7
title: Real-Time Considerations
description: Time-critical robotic applications aur real-time performance
---

# Real-Time Considerations

Iss chapter mein, aap seekhenge ke time-critical robotic applications aur ROS 2 mein real-time performance ke bare mein.

## Seeking Objectives

- Robotics mein real-time constraints ko samjhein
- Real-time performance ke liye ROS 2 ko configure karen
- Low latency ke liye communication optimize karen
- Real-time safe code patterns laagoo karen

## Robotics Mein Real-Time Concepts

Real-time systems ko events par strict time constraints ke andhar response dena chahiye. Robotics mein, ye zaroori hai:

- **Control loops**: Robot controllers ko often fixed frequencies par chalna chahiye (100Hz, 1kHz)
- **Safety systems**: Emergency stops aur collision avoidance turant response dena chahiye
- **Sensor fusion**: Multiple sensors se data ko precise timing ke saath combine karna
- **Motion planning**: Trajectory execution ko precise timing ki zaroorat hoti hai

### Hard vs Soft Real-Time

- **Hard real-time**: Deadlines miss karne se system failure ho jata hai (safety-critical systems)
- **Soft real-time**: Deadlines miss karne se performance degrade hota hai lekin failure nahi hota (most robotic applications)

## Real-Time Kernel Configuration

Hard real-time applications ke liye, apna system configure karen:

### Installing RT Kernel (Ubuntu)

```bash
# Real-time kernel install karen
sudo apt update
sudo apt install linux-image-rt-generic

# Reboot karen aur GRUB menu se RT kernel select karen
sudo reboot
```

### Real-Time Privileges

Apne user ko real-time groups mein add karen:

```bash
sudo usermod -a -G realtime $USER
# Changes effect mein lane ke liye log out aur phir wapas log in karen
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

        # Is node ke liye real-time priority set karen
        self.set_realtime_priority()

        # Control commands ke liye publisher banayein
        self.cmd_pub = self.create_publisher(Float64MultiArray, 'joint_commands', 1)

        # High-frequency timer banayein (1kHz)
        self.control_timer = self.create_timer(
            0.001,  # 1ms = 1000Hz
            self.control_loop,
            clock=self.get_clock()
        )

        self.joint_positions = [0.0] * 6  # 6-DOF robot
        self.control_counter = 0

    def set_realtime_priority(self):
        """Node ke liye real-time scheduling priority set karen"""
        try:
            import os
            import ctypes
            from ctypes import util

            # Sched_setscheduler ka istemal karke real-time priority set karen
            libc = ctypes.CDLL(util.find_library("c"))

            # SCHED_FIFO priority 80 (99 mein se)
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

        # Real-time loop mein minimal computation
        self.update_control_commands()

        # Control commands publish karen
        cmd_msg = Float64MultiArray()
        cmd_msg.data = self.joint_positions
        self.cmd_pub.publish(cmd_msg)

        self.control_counter += 1

        # Timing monitor karen
        elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e6  # ms
        if elapsed > 1.0:  # Ziyada se ziyada 1ms liya
            self.get_logger().warn(f'Control loop exceeded deadline: {elapsed:.2f}ms')

    def update_control_commands(self):
        """Minimal control algorithm - is function ko tezi mein rakhein!"""
        # Simple sinusoidal trajectory
        t = self.get_clock().now().nanoseconds / 1e9  # Seconds mein convert karen
        for i in range(len(self.joint_positions)):
            self.joint_positions[i] = 0.5 * math.sin(t * 2 * math.pi * 0.5 + i)

def main(args=None):
    rclpy.init(args=args)

    # Real-time safe executor ka istemal karen
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

High-frequency data ke liye memory allocation overhead ko km karen:

```python
from rclpy.qos import QoSProfile, HistoryPolicy
from rclpy.publisher import Publisher
import numpy as np

class ZeroCopyPublisher(Node):
    def __init__(self):
        super().__init__('zero_copy_publisher')

        # Zero-copy transport ke liye configure karen
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.publisher = self.create_publisher(SensorMsgs, 'sensor_data', qos_profile)

        # Message buffer pre-allocate karen
        self.msg_buffer = SensorMsgs()
        self.sensor_data = np.zeros(1000, dtype=np.float32)  # Pre-allocated data

    def publish_sensor_data(self):
        # Pre-allocated message fill karen
        self.msg_buffer.header.stamp = self.get_clock().now().to_msg()
        self.msg_buffer.header.frame_id = 'sensor_frame'

        # Sensor data inplace update karen
        self.update_sensor_data()

        # Copy karne ke bina publish karen agar possible ho
        self.publisher.publish(self.msg_buffer)

    def update_sensor_data(self):
        """Sensor data efficiently fill karen"""
        # In-place operations ka istemal karen
        np.sin(np.arange(1000) * 0.01, out=self.sensor_data)
        self.msg_buffer.data = self.sensor_data.tolist()
```

## Lock-Free Data Structures

Inter-node communication ke liye thread-safe, lock-free data structures ka istemal karen:

```python
import queue
import threading
from collections import deque
from rclpy.node import Node

class LockFreeBufferNode(Node):
    def __init__(self):
        super().__init__('lockfree_buffer_node')

        # Inter-thread communication ke liye lock-free queues ka istemal karen
        self.data_queue = queue.Queue(maxsize=100)
        self.result_queue = queue.Queue(maxsize=100)

        # Separate thread mein subscriber
        self.sub_thread = threading.Thread(target=self.subscribe_thread)
        self.sub_thread.daemon = True
        self.sub_thread.start()

        # Main thread mein processing
        self.process_timer = self.create_timer(0.01, self.process_data)

    def subscribe_thread(self):
        """Subscription ke liye dedicated thread"""
        while rclpy.ok():
            try:
                # Isse normally subscription callback hota
                # Incoming data simulate karte hain
                data = self.simulate_sensor_data()

                try:
                    self.data_queue.put_nowait(data)
                except queue.Full:
                    self.get_logger().warn('Data queue full, dropping packet')

            except Exception as e:
                self.get_logger().error(f'Subscription thread error: {e}')

    def process_data(self):
        """Main thread mein data process karen"""
        try:
            data = self.data_queue.get_nowait()

            # Data process karen
            result = self.process_sensor_data(data)

            # Result output queue mein daalen
            try:
                self.result_queue.put_nowait(result)
            except queue.Full:
                self.get_logger().warn('Result queue full')

        except queue.Empty:
            pass  # Process karne ke liye koi data nahi hai

    def simulate_sensor_data(self):
        """Incoming sensor data simulate karen"""
        return {'timestamp': time.time(), 'values': [1.0, 2.0, 3.0]}

    def process_sensor_data(self, data):
        """Sensor data process karen"""
        # Real-time safe processing
        return {'processed': True, 'avg': sum(data['values']) / len(data['values'])}
```

## Memory Management

Real-time paths mein dynamic memory allocation ko km karen:

```python
from rclpy.node import Node
import numpy as np

class MemoryEfficientNode(Node):
    def __init__(self):
        super().__init__('memory_efficient_node')

        # Sab buffers pre-allocate karen
        self.input_buffer = np.zeros(1000, dtype=np.float32)
        self.output_buffer = np.zeros(1000, dtype=np.float32)
        self.temp_buffer = np.zeros(1000, dtype=np.float32)

        self.publisher = self.create_publisher(Float32MultiArray, 'filtered_data', 1)
        self.subscriber = self.create_subscription(
            Float32MultiArray, 'raw_data', self.data_callback, 1
        )

        # Message objects pre-allocate karen
        self.output_msg = Float32MultiArray()

    def data_callback(self, msg):
        """Real-time safe callback"""
        # Data ko pre-allocated buffer mein copy karen
        if len(msg.data) <= len(self.input_buffer):
            self.input_buffer[:len(msg.data)] = msg.data
            self.input_buffer[len(msg.data):] = 0  # Remaining zero-pad karen

            # Real-time safe manner mein process karen
            self.filter_data()

            # Result publish karen
            self.output_msg.data = self.output_buffer[:len(msg.data)].tolist()
            self.publisher.publish(self.output_msg)

    def filter_data(self):
        """Real-time safe filtering operation"""
        # Sirf pre-allocated arrays ka istemal karen
        np.copyto(self.temp_buffer, self.input_buffer)
        np.multiply(self.temp_buffer, 0.9, out=self.output_buffer)  # IIR filter
```

## Real-Time Synchronization

Real-time aur non-real-time threads ke beech coordinate karen:

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
        # Real-time priority set karen
        self.set_realtime_priority()

        while rclpy.ok():
            with self.rt_condition:
                # Real-time work karen
                self.do_realtime_work()

                # Non-RT thread ko signal karen
                self.rt_ready = True
                self.rt_condition.notify()

                # Non-RT khalm tamam hone tak wait karen
                while self.non_rt_ready:
                    self.rt_condition.wait()

    def non_rt_thread(self):
        """Non-real-time thread for logging, etc."""
        while rclpy.ok():
            with self.non_rt_condition:
                # RT thread ke signal karne tak wait karen
                while not self.rt_ready:
                    self.non_rt_condition.wait()

                # Non-RT work karen (logging, etc.)
                self.do_non_rt_work()

                # RT thread ko signal karen
                self.non_rt_ready = False
                self.non_rt_condition.notify()

    def do_realtime_work(self):
        """Real-time critical work"""
        # Minimal, deterministic work yahan
        self.cycle_count += 1
        time.sleep(0.001)  # Control cycle simulate karen

    def do_non_rt_work(self):
        """Non-real-time work"""
        # Logging, diagnostics, etc. yahan ja sakti hai
        if self.cycle_count % 100 == 0:
            self.get_logger().info(f'Processed {self.cycle_count} cycles')
```

## Performance Monitoring

Real-time performance ko monitor karen:

```python
from rclpy.node import Node
import statistics

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('perf_monitor')

        self.cycle_times = []
        self.deadline_misses = 0
        self.total_cycles = 0

        # Timer performance ko monitor karen
        self.period = 0.001  # 1ms
        self.timer = self.create_timer(self.period, self.timed_callback)

        # Performance monitoring timer
        self.monitor_timer = self.create_timer(5.0, self.report_performance)

    def timed_callback(self):
        start_time = time.perf_counter()

        # Work karen
        self.do_work()

        end_time = time.perf_counter()
        cycle_time = (end_time - start_time) * 1000  # ms

        self.cycle_times.append(cycle_time)
        self.total_cycles += 1

        # Deadline miss check karen (cycle ko period se ziada time laga)
        if cycle_time > (self.period * 1000):  # Ms mein convert karen
            self.deadline_misses += 1

    def do_work(self):
        """Monitor karne ke liye work"""
        # Thoda processing simulate karte hain
        time.sleep(0.0005)  # 0.5ms ka "work"

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

            # Sirf recent measurements rakhne
            if len(self.cycle_times) > 1000:
                self.cycle_times = self.cycle_times[-500:]
```

## Best Practices for Real-Time ROS 2

1. **Real-time paths minimal rakhein** - critical loops mein complex computations se bachen
2. **Memory pre-allocate karen** - real-time paths mein dynamic allocation se bachen
3. **Appropriate QoS ka istemal karen** - apne timing requirements ke liye configure karen
4. **RT aur non-RT work ko alag karen** - non-critical tasks ke liye threads ka istemal karen
5. **Performance monitor karen** - timing metrics ko continuously track karen
6. **Load ke saath test karen** - realistic workloads ke saath performance verify karen
7. **Hardware ko dhyan mein rakhen** - deterministic hardware ka istemal karen jab possible ho
8. **Code ko profile karen** - apne implementation mein bottlenecks identify karen

## Troubleshooting Real-Time Issues

Common real-time problems aur solutions:

- **Deadline misses**: Computational load ko km karen ya period bdhayein
- **Jitter**: Non-determinism ke sources check karen (dynamic allocation, locks)
- **System interference**: CPU isolation ka istemal karen, power management disable karen
- **Memory allocation**: Zaroori sab memory pre-allocate karen

---

Congratulations! Aapne Module 1 ke sabhi advanced topics complete kar liye hain. [Module 2: Simulation](/docs/module-2-simulation) par jane ke liye aur seekhein ke apne ROS 2 systems ko virtual environments mein kaise test karen.