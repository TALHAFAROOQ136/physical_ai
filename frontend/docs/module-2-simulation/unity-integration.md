---
sidebar_position: 3
title: Unity Integration
description: High-fidelity visualization with Unity
---

# Unity Integration

Unity provides photorealistic rendering and advanced physics for robotics simulation.

## Learning Objectives

- Install Unity Robotics Hub
- Connect Unity to ROS 2
- Create realistic robot visualizations

## Why Unity?

| Feature | Gazebo | Unity |
|---------|--------|-------|
| Physics | ODE/Bullet | PhysX/Custom |
| Graphics | Good | Photorealistic |
| VR Support | Limited | Native |
| Asset Store | Limited | Extensive |

## Installing Unity Robotics Hub

1. Download Unity Hub
2. Install Unity 2021.3 LTS or later
3. Add the Robotics packages:
   - `com.unity.robotics.ros-tcp-connector`
   - `com.unity.robotics.urdf-importer`

## ROS-TCP Connection

```csharp
// Unity C# script
using Unity.Robotics.ROSTCPConnector;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>("/cmd_vel", OnVelocityReceived);
    }

    void OnVelocityReceived(TwistMsg msg)
    {
        // Apply velocity to robot
    }
}
```

## Importing URDF

```csharp
using Unity.Robotics.UrdfImporter;

// Import robot from URDF file
UrdfRobotExtensions.Create(urdfPath);
```

## Exercise

Import a simple robot URDF into Unity and control it via ROS 2 topics.

---

Congratulations! You've completed Module 2. Continue to [Module 3: NVIDIA Isaac](/docs/module-3-isaac).
