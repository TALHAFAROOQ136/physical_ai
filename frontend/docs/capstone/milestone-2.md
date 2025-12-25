---
sidebar_position: 3
title: "Milestone 2: ROS 2 Foundation"
description: Implement core ROS 2 infrastructure
---

# Milestone 2: ROS 2 Foundation

Build the core ROS 2 infrastructure for your robot.

## Objectives

- Create ROS 2 package structure
- Implement core nodes
- Define message interfaces
- Set up launch files

## Deliverables

### 1. Package Structure

```
my_humanoid/
├── my_humanoid/
│   ├── __init__.py
│   ├── voice_node.py
│   ├── planner_node.py
│   ├── controller_node.py
│   └── perception_node.py
├── msg/
│   └── RobotState.msg
├── srv/
│   └── PlanTask.srv
├── action/
│   └── ExecuteTask.action
├── launch/
│   ├── robot.launch.py
│   └── simulation.launch.py
├── config/
│   └── params.yaml
├── urdf/
│   └── robot.urdf.xacro
├── package.xml
├── setup.py
└── setup.cfg
```

### 2. Core Nodes

**Voice Node:**
```python
class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')
        self.command_pub = self.create_publisher(
            String, '/voice_command', 10
        )
        # Implementation...
```

**Planner Node:**
```python
class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.command_sub = self.create_subscription(
            String, '/voice_command',
            self.command_callback, 10
        )
        self.action_pub = self.create_publisher(
            ActionSequence, '/planned_actions', 10
        )
```

### 3. Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_humanoid',
            executable='voice_node',
            name='voice',
            parameters=['config/params.yaml']
        ),
        Node(
            package='my_humanoid',
            executable='planner_node',
            name='planner'
        ),
        Node(
            package='my_humanoid',
            executable='controller_node',
            name='controller'
        ),
    ])
```

## Review Checklist

- [ ] All nodes launch without errors
- [ ] Topics are correctly connected
- [ ] Messages flow between nodes
- [ ] Parameters are configurable

---

Continue to [Milestone 3: Simulation](/docs/capstone/milestone-3).
