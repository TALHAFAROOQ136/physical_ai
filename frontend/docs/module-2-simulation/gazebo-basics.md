---
sidebar_position: 2
title: Gazebo Basics
description: Physics simulation with Gazebo
---

# Gazebo Basics

Gazebo is a powerful open-source robotics simulator that integrates seamlessly with ROS 2.

## Learning Objectives

- Launch and navigate Gazebo
- Create simulation worlds
- Spawn robots in simulation
- Control robots from ROS 2

## Installing Gazebo

```bash
# Install Gazebo for ROS 2 Humble
sudo apt install ros-humble-gazebo-ros-pkgs
```

## Launching Gazebo

```bash
# Launch empty world
ros2 launch gazebo_ros gazebo.launch.py

# Launch with a world file
ros2 launch gazebo_ros gazebo.launch.py world:=my_world.world
```

## Creating a World File

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="my_world">
    <!-- Sun light -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- A simple box obstacle -->
    <model name="box">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Spawning Robots

```bash
ros2 run gazebo_ros spawn_entity.py \
  -entity my_robot \
  -file /path/to/robot.urdf
```

## ROS 2 Gazebo Plugins

Add plugins to your URDF for ROS 2 integration:

```xml
<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <ros>
      <namespace>/my_robot</namespace>
    </ros>
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.4</wheel_separation>
    <wheel_diameter>0.1</wheel_diameter>
    <command_topic>cmd_vel</command_topic>
    <odometry_topic>odom</odometry_topic>
  </plugin>
</gazebo>
```

## Exercise

Create a simple world with:
1. A ground plane
2. Three box obstacles
3. A robot spawn point

---

Continue to [Unity Integration](/docs/module-2-simulation/unity-integration) for advanced visualization.
