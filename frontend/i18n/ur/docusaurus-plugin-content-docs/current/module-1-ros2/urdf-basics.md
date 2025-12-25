---
sidebar_position: 4
title: URDF Basics
description: URDF کا استعمال کرتے ہوئے robot structure define کریں
---

# URDF Basics

URDF (Unified Robot Description Format) روبوٹ کی physical structure کو بیان کرنے کے لیے ایک XML format ہے۔ یہ links، joints، اور visual/collision properties define کرتا ہے۔

## سیکھنے کے مقاصد

- URDF structure اور syntax کو سمجھیں
- robot links اور joints بنائیں
- visual اور collision geometries شامل کریں
- RViz میں robots visualize کریں

## URDF Structure

ایک URDF file روبوٹ کو **links** کے درخت کے طور پر بیان کرتی ہے جو **joints** سے جڑے ہوں:

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

## بنیادی URDF مثال

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

یہ define کرتا ہے کہ link کیسا دکھائی دیتا ہے:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1 1 1"/>        <!-- یا -->
    <cylinder radius="0.5" length="1"/>  <!-- یا -->
    <sphere radius="0.5"/>     <!-- یا -->
    <mesh filename="package://my_pkg/meshes/part.stl"/>
  </geometry>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
</visual>
```

### Collision

یہ collision boundaries define کرتا ہے (اکثر simplified):

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1 1 1"/>
  </geometry>
</collision>
```

### Inertial

یہ dynamics کے لیے mass اور inertia define کرتا ہے:

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

| Type | تفصیل | DOF |
|------|-------------|-----|
| `fixed` | کوئی حرکت نہیں | 0 |
| `revolute` | حدود کے ساتھ گردش | 1 |
| `continuous` | لامحدود گردش | 1 |
| `prismatic` | خطی حرکت | 1 |
| `floating` | آزاد 6-DOF حرکت | 6 |
| `planar` | ایک plane میں حرکت | 3 |

### Joint مثال

```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="body"/>
  <child link="upper_arm"/>
  <origin xyz="0.2 0 0.5" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Y axis کے گرد گردش -->
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  <dynamics damping="0.5" friction="0.1"/>
</joint>
```

## Xacro کا استعمال

Xacro URDF میں macros اور variables شامل کرتا ہے:

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

Xacro کو URDF میں process کریں:

```bash
xacro robot.urdf.xacro > robot.urdf
```

## RViz میں Visualize کرنا

ایک launch file `display.launch.py` بنائیں:

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

چلائیں:

```bash
ros2 launch my_robot_pkg display.launch.py
```

## مشق

ایک سادہ 2-link robot arm کے لیے URDF بنائیں جس میں:

1. ایک fixed base ہو
2. دو revolute joints ہوں
3. ہر link کے لیے visual cylinders ہوں

<details>
<summary>حل</summary>

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

مبارک ہو! آپ نے Module 1 مکمل کر لیا۔ اگلا، اپنے robots کو virtual world میں test کرنے کے لیے [Gazebo کے ساتھ Simulation](/docs/module-2-simulation) سیکھیں۔
