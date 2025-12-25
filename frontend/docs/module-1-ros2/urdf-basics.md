---
sidebar_position: 4
title: URDF Basics
description: Define robot structure using URDF
---

# URDF Basics

URDF (Unified Robot Description Format) is an XML format for describing a robot's physical structure. It defines links, joints, and visual/collision properties.

## Learning Objectives

- Understand URDF structure and syntax
- Create robot links and joints
- Add visual and collision geometries
- Visualize robots in RViz

## URDF Structure

A URDF file describes a robot as a tree of **links** connected by **joints**:

```
          base_link
              │
              │ (joint: shoulder)
              ▼
          upper_arm
              │
              │ (joint: elbow)
              ▼
          lower_arm
              │
              │ (joint: wrist)
              ▼
          gripper
```

## Basic URDF Example

```xml
<?xml version="1.0"?>
<robot name="my_robot">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Arm Link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>

  <!-- Joint connecting base to arm -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

</robot>
```

## Link Elements

### Visual

Defines how the link appears:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1 1 1"/>        <!-- or -->
    <cylinder radius="0.5" length="1"/>  <!-- or -->
    <sphere radius="0.5"/>     <!-- or -->
    <mesh filename="package://my_pkg/meshes/part.stl"/>
  </geometry>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
</visual>
```

### Collision

Defines collision boundaries (often simplified):

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1 1 1"/>
  </geometry>
</collision>
```

### Inertial

Defines mass and inertia for dynamics:

```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="1.0"/>
  <inertia ixx="0.01" ixy="0" ixz="0"
           iyy="0.01" iyz="0"
           izz="0.01"/>
</inertial>
```

## Joint Types

| Type | Description | DOF |
|------|-------------|-----|
| `fixed` | No motion | 0 |
| `revolute` | Rotation with limits | 1 |
| `continuous` | Unlimited rotation | 1 |
| `prismatic` | Linear motion | 1 |
| `floating` | Free 6-DOF motion | 6 |
| `planar` | Motion in a plane | 3 |

### Joint Example

```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="body"/>
  <child link="upper_arm"/>
  <origin xyz="0.2 0 0.5" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Rotation around Y axis -->
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  <dynamics damping="0.5" friction="0.1"/>
</joint>
```

## Using Xacro

Xacro adds macros and variables to URDF:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

  <!-- Properties -->
  <xacro:property name="arm_length" value="0.5"/>
  <xacro:property name="arm_radius" value="0.05"/>

  <!-- Macro for arm segments -->
  <xacro:macro name="arm_segment" params="name length parent">
    <link name="${name}">
      <visual>
        <geometry>
          <cylinder radius="${arm_radius}" length="${length}"/>
        </geometry>
        <origin xyz="0 0 ${length/2}"/>
      </visual>
    </link>

    <joint name="${name}_joint" type="revolute">
      <parent link="${parent}"/>
      <child link="${name}"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:arm_segment name="upper_arm" length="${arm_length}" parent="base_link"/>
  <xacro:arm_segment name="lower_arm" length="${arm_length}" parent="upper_arm"/>

</robot>
```

Process Xacro to URDF:

```bash
xacro robot.urdf.xacro > robot.urdf
```

## Visualizing in RViz

Create a launch file `display.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_pkg')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2'
        ),
    ])
```

Run:

```bash
ros2 launch my_robot_pkg display.launch.py
```

## Exercise

Create a URDF for a simple 2-link robot arm with:

1. A fixed base
2. Two revolute joints
3. Visual cylinders for each link

<details>
<summary>Solution</summary>

```xml
<?xml version="1.0"?>
<robot name="two_link_arm">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>

  <link name="link1">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
      <origin xyz="0 0 0.2"/>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
  </link>

  <link name="link2">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
      <origin xyz="0 0 0.15"/>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.4"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="50" velocity="1.0"/>
  </joint>

</robot>
```

</details>

---

Congratulations! You've completed Module 1. Next, learn about [Simulation with Gazebo](/docs/module-2-simulation) to test your robots in a virtual world.
