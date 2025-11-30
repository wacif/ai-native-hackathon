---
sidebar_position: 3
---

# Module 2: The Digital Twin (Gazebo & Unity)

> **Focus: Physics simulation and environment building**

Before deploying AI to real robots (expensive, fragile, dangerous), we simulate everything in a **digital twin**. This module teaches you to build realistic virtual environments with accurate physics, sensors, and visualizations.

## Why Simulation Matters

Real robots are:
- 💰 **Expensive** - A single crash can cost thousands
- ⏱️ **Slow to iterate** - Physical testing takes time
- ⚠️ **Dangerous** - Bugs can cause real harm
- 🔁 **Hard to reproduce** - Real-world conditions vary

Simulation lets you:
- Train AI with millions of iterations
- Test edge cases safely
- Reproduce bugs exactly
- Iterate in seconds, not hours

## The Simulation Stack

```
┌─────────────────────────────────────────┐
│           Your ROS 2 Code               │
├─────────────────────────────────────────┤
│      Gazebo/Isaac Sim (Physics)         │
├─────────────────────────────────────────┤
│        Unity/Unreal (Visuals)           │
└─────────────────────────────────────────┘
```

| Tool | Strengths | Best For |
|------|-----------|----------|
| **Gazebo** | Open-source, ROS integration | Research, prototyping |
| **Isaac Sim** | RTX rendering, AI training | Production, sim-to-real |
| **Unity** | Game-quality visuals, easy | Demos, human interaction |
| **Unreal** | Photorealism | Film-quality renders |

## Gazebo Simulation

### Installing Gazebo (Ignition)

```bash
# Install Gazebo Fortress (LTS)
sudo apt install ros-humble-gazebo-ros-pkgs

# Install ROS 2 - Gazebo integration
sudo apt install ros-humble-ros-gz

# Verify installation
gz sim --version
```

### Your First Simulation

Create a simple world file `my_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="robot_world">
    
    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Sun Light -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Physics -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
  </world>
</sdf>
```

Launch:
```bash
gz sim my_world.sdf
```

### Adding a Robot

Create `robot.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="wheeled_robot">
  
  <!-- Properties -->
  <xacro:property name="wheel_radius" value="0.05"/>
  <xacro:property name="wheel_width" value="0.02"/>
  <xacro:property name="body_length" value="0.3"/>
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${body_length} 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${body_length} 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" 
               iyy="0.02" iyz="0" izz="0.04"/>
    </inertial>
  </link>
  
  <!-- Wheel Macro -->
  <xacro:macro name="wheel" params="prefix x_reflect y_reflect">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black">
          <color rgba="0.1 0.1 0.1 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.001" ixy="0" ixz="0" 
                 iyy="0.001" iyz="0" izz="0.001"/>
      </inertial>
    </link>
    
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${x_reflect*0.1} ${y_reflect*0.12} -0.03" 
              rpy="${pi/2} 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>
  
  <!-- Create 4 wheels -->
  <xacro:wheel prefix="front_left" x_reflect="1" y_reflect="1"/>
  <xacro:wheel prefix="front_right" x_reflect="1" y_reflect="-1"/>
  <xacro:wheel prefix="rear_left" x_reflect="-1" y_reflect="1"/>
  <xacro:wheel prefix="rear_right" x_reflect="-1" y_reflect="-1"/>
  
  <!-- Gazebo Plugins -->
  <gazebo>
    <plugin filename="libgazebo_ros_diff_drive.so" name="diff_drive">
      <ros>
        <namespace>/robot</namespace>
      </ros>
      <left_joint>front_left_wheel_joint</left_joint>
      <right_joint>front_right_wheel_joint</right_joint>
      <wheel_separation>0.24</wheel_separation>
      <wheel_diameter>0.1</wheel_diameter>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
    </plugin>
  </gazebo>
  
</robot>
```

### Simulating Sensors

#### LiDAR Sensor

```xml
<!-- LiDAR Link -->
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.03" length="0.05"/>
    </geometry>
  </visual>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.1"/>
</joint>

<!-- Gazebo LiDAR Plugin -->
<gazebo reference="lidar_link">
  <sensor type="gpu_lidar" name="lidar">
    <pose>0 0 0 0 0 0</pose>
    <topic>/scan</topic>
    <update_rate>10</update_rate>
    <lidar>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </lidar>
    <always_on>true</always_on>
    <visualize>true</visualize>
  </sensor>
</gazebo>
```

#### Depth Camera (RealSense-like)

```xml
<!-- Camera Link -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.02 0.08 0.02"/>
    </geometry>
  </visual>
</link>

<gazebo reference="camera_link">
  <sensor type="depth_camera" name="depth_camera">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <depth_camera>
        <output>depths</output>
      </depth_camera>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
    </camera>
    <plugin name="depth_camera_plugin" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/camera</namespace>
      </ros>
      <camera_name>depth</camera_name>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

#### IMU (Inertial Measurement Unit)

```xml
<gazebo reference="base_link">
  <sensor type="imu" name="imu_sensor">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0001</stddev>
          </noise>
        </x>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.001</stddev>
          </noise>
        </x>
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/imu</namespace>
      </ros>
      <frame_name>base_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Physics Simulation

Gazebo simulates real physics:

### Gravity and Collisions

```xml
<physics type="ode">
  <gravity>0 0 -9.81</gravity>
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
    </constraints>
  </ode>
  <real_time_factor>1.0</real_time_factor>
  <max_step_size>0.001</max_step_size>
</physics>
```

### Friction and Contact

```xml
<collision name="wheel_collision">
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>
        <mu2>1.0</mu2>
      </ode>
    </friction>
    <contact>
      <ode>
        <kp>1000000.0</kp>
        <kd>100.0</kd>
      </ode>
    </contact>
  </surface>
</collision>
```

## Unity for High-Fidelity Rendering

While Gazebo handles physics, Unity provides:
- Photorealistic graphics
- Human character animation
- VR/AR integration
- Game-quality environments

### Unity Robotics Hub Setup

```bash
# Install Unity Hub
# Download from https://unity.com/download

# Create a new 3D (URP) project

# Install packages via Package Manager:
# - ROS TCP Connector
# - URDF Importer
```

### Connecting Unity to ROS 2

1. **In Unity**: Add `ROS TCP Connector` component
2. **Set ROS IP**: `127.0.0.1` (or robot IP)
3. **Set Port**: `10000`

```csharp
// Unity C# - Subscribing to ROS topics
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RobotController : MonoBehaviour
{
    void Start()
    {
        ROSConnection.GetOrCreateInstance()
            .Subscribe<TwistMsg>("/cmd_vel", OnVelocityReceived);
    }
    
    void OnVelocityReceived(TwistMsg msg)
    {
        float linear = (float)msg.linear.x;
        float angular = (float)msg.angular.z;
        
        // Apply to robot transform
        transform.Translate(Vector3.forward * linear * Time.deltaTime);
        transform.Rotate(Vector3.up * angular * Time.deltaTime);
    }
}
```

### Human-Robot Interaction in Unity

Unity excels at simulating humans:

```csharp
// Animate a human walking toward the robot
public class HumanAgent : MonoBehaviour
{
    public Transform robot;
    public Animator animator;
    public float walkSpeed = 1.5f;
    
    void Update()
    {
        // Walk toward robot
        Vector3 direction = (robot.position - transform.position).normalized;
        transform.position += direction * walkSpeed * Time.deltaTime;
        transform.LookAt(robot);
        
        // Trigger walk animation
        animator.SetBool("isWalking", true);
    }
}
```

## ROS 2 Launch with Simulation

Complete launch file for simulation:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_pkg')
    
    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 
                        'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': os.path.join(pkg_dir, 'worlds', 'my_world.sdf')}.items()
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-file', os.path.join(pkg_dir, 'urdf', 'robot.urdf'),
            '-x', '0', '-y', '0', '-z', '0.1'
        ]
    )
    
    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': open(
                os.path.join(pkg_dir, 'urdf', 'robot.urdf')
            ).read()
        }]
    )
    
    # RViz for visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'config.rviz')]
    )
    
    return LaunchDescription([
        gazebo,
        spawn_robot,
        robot_state_pub,
        rviz
    ])
```

## Practical Exercise: Build a Test Environment

Create a simulation world with:

1. **Ground plane** with checkerboard texture
2. **Obstacles** (boxes, cylinders)
3. **Your robot** with LiDAR
4. **ROS 2 teleop** for manual control

```bash
# Run simulation
ros2 launch my_robot_pkg simulation.launch.py

# Control robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Visualize LiDAR in RViz
ros2 run rviz2 rviz2
```

## Sim-to-Real Considerations

Simulation is never perfect. Watch out for:

| Issue | Problem | Mitigation |
|-------|---------|------------|
| **Reality Gap** | Physics don't match real world | Domain randomization |
| **Sensor Noise** | Real sensors are noisy | Add noise in sim |
| **Latency** | Real systems have delays | Simulate delays |
| **Dynamics** | Motor response differs | System identification |

### Domain Randomization

Randomly vary simulation parameters during training:

```python
import random

def randomize_world():
    # Randomize lighting
    sun_intensity = random.uniform(0.5, 1.5)
    
    # Randomize friction
    floor_friction = random.uniform(0.5, 1.2)
    
    # Randomize object positions
    obstacle_x = random.uniform(-2, 2)
    obstacle_y = random.uniform(-2, 2)
    
    # Randomize sensor noise
    lidar_noise = random.uniform(0.01, 0.05)
    
    return {
        'sun_intensity': sun_intensity,
        'friction': floor_friction,
        'obstacle_pos': (obstacle_x, obstacle_y),
        'sensor_noise': lidar_noise
    }
```

## Key Takeaways

1. **Simulate before deploying** - Save time, money, and hardware
2. **Gazebo for physics** - Accurate dynamics and sensor simulation
3. **Unity for visuals** - Photorealistic human-robot interaction
4. **URDF describes robots** - Portable across simulators
5. **Add sensor noise** - Make simulation realistic
6. **Domain randomization** - Bridge the sim-to-real gap

## What's Next?

In **Module 3**, you'll use **NVIDIA Isaac** for photorealistic simulation, synthetic data generation, and hardware-accelerated perception pipelines.

---

**Next**: [Module 3: The AI-Robot Brain (NVIDIA Isaac) →](./module3-isaac)
