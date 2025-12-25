---
sidebar_position: 4
title: "Milestone 3: Simulation"
description: Build the simulation environment
---

# Milestone 3: Simulation Setup

Create a simulation environment for testing your robot.

## Objectives

- Build simulation world
- Import robot model
- Configure sensors
- Test basic behaviors

## Deliverables

### 1. World File

```xml
<!-- worlds/apartment.world -->
<sdf version="1.6">
  <world name="apartment">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Furniture -->
    <include>
      <uri>model://table</uri>
      <pose>2 0 0 0 0 0</pose>
    </include>

    <!-- Objects to manipulate -->
    <model name="cup">
      <pose>2 0.3 0.75 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.04</radius>
              <length>0.1</length>
            </cylinder>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### 2. Robot URDF with Sensors

```xml
<!-- Add camera -->
<gazebo reference="head_link">
  <sensor type="camera" name="head_camera">
    <update_rate>30.0</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
      </ros>
      <camera_name>head_camera</camera_name>
    </plugin>
  </sensor>
</gazebo>
```

### 3. Simulation Launch

```python
def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',
                 'worlds/apartment.world'],
        ),
        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot',
                      '-file', 'urdf/robot.urdf'],
        ),
    ])
```

## Testing Checklist

- [ ] World loads correctly
- [ ] Robot spawns in correct position
- [ ] Sensors publish data
- [ ] Robot responds to commands

---

Continue to [Milestone 4: AI Integration](/docs/capstone/milestone-4).
