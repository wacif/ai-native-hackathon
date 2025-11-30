---
sidebar_position: 6
---

# Weekly Schedule & Labs

> **Detailed breakdown of the 13-week course**

This document provides the complete schedule with lab assignments, readings, and deliverables for each week.

## Course Calendar Overview

```
Week 1-2   │ Introduction & Foundations
Week 3-5   │ Module 1: ROS 2 Fundamentals  
Week 6-7   │ Module 2: Simulation (Gazebo/Unity)
Week 8-10  │ Module 3: NVIDIA Isaac Platform
Week 11-12 │ Humanoid Development
Week 13    │ Module 4: VLA + Capstone Presentations
```

---

## Weeks 1-2: Introduction to Physical AI

### Week 1: What is Physical AI?

**Learning Objectives:**
- Understand the paradigm shift from digital AI to embodied AI
- Survey the humanoid robotics landscape
- Setup development environment

**Topics:**
- From ChatGPT to robots that can act
- Why humanoids? The human-centered design argument
- The sensor-to-action loop
- Introduction to the Physical AI stack

**Lab 1: Environment Setup**
```bash
# Install Ubuntu 22.04 (if not already)
# Install ROS 2 Humble
sudo apt install ros-humble-desktop

# Install Isaac dependencies
sudo apt install nvidia-cuda-toolkit

# Verify setup
ros2 doctor
nvidia-smi
```

**Reading:**
- NVIDIA's "Physical AI" whitepaper
- [Why Humanoids Will Win](https://www.figure.ai/news)

---

### Week 2: Sensors & Perception Fundamentals

**Learning Objectives:**
- Understand robot sensing modalities
- Set up Intel RealSense camera
- Process basic sensor data

**Topics:**
- Sensor types: cameras, LiDAR, IMU, force/torque
- Camera intrinsics and extrinsics
- Point clouds and depth perception
- Sensor fusion concepts

**Lab 2: RealSense Depth Camera**
```python
import pyrealsense2 as rs
import numpy as np
import cv2

# Create pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        # Convert to numpy
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # Apply colormap
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), 
            cv2.COLORMAP_JET
        )
        
        # Display
        cv2.imshow('Depth', depth_colormap)
        cv2.imshow('Color', color_image)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
```

**Deliverable:** Sensor data visualization screenshot

---

## Weeks 3-5: ROS 2 Fundamentals

### Week 3: ROS 2 Architecture

**Learning Objectives:**
- Master ROS 2 core concepts
- Create and run ROS 2 nodes
- Understand the publish/subscribe pattern

**Topics:**
- ROS 2 vs ROS 1
- DDS (Data Distribution Service)
- Nodes, Topics, Services, Actions
- Quality of Service (QoS)

**Lab 3: Your First ROS 2 Package**
```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create package
ros2 pkg create --build-type ament_python --node-name my_node my_robot_pkg

# Build
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Run
ros2 run my_robot_pkg my_node
```

**Deliverable:** Working package that publishes sensor data

---

### Week 4: Messages & Communication

**Learning Objectives:**
- Use standard message types
- Create custom messages
- Implement service servers and clients

**Topics:**
- Standard messages: `std_msgs`, `sensor_msgs`, `geometry_msgs`
- Custom message definitions
- Service communication patterns
- Debugging with `ros2 topic` and `ros2 service`

**Lab 4: Sensor Fusion Node**
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        
        # Synchronized subscribers
        self.image_sub = Subscriber(self, Image, '/camera/image')
        self.imu_sub = Subscriber(self, Imu, '/imu/data')
        
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.imu_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)
        
        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/fused_pose', 10)
        
    def sync_callback(self, image_msg, imu_msg):
        self.get_logger().info('Synchronized data received!')
        # Fusion logic here
```

**Deliverable:** Node synchronizing camera + IMU data

---

### Week 5: URDF & Transforms

**Learning Objectives:**
- Create URDF robot descriptions
- Understand TF2 coordinate frames
- Visualize robots in RViz

**Topics:**
- URDF XML structure
- Links, joints, and visual/collision geometry
- TF2 transform tree
- Robot state publisher

**Lab 5: Create a Mobile Robot URDF**
```xml
<?xml version="1.0"?>
<robot name="mobile_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <xacro:property name="wheel_radius" value="0.05"/>
  <xacro:property name="wheel_width" value="0.03"/>
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.2"/>
    </inertial>
  </link>
  
  <!-- Camera Link -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.02 0.08 0.02"/>
      </geometry>
    </visual>
  </link>
  
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.05"/>
  </joint>
  
  <!-- Include wheel macro -->
  <xacro:include filename="wheel.urdf.xacro"/>
  <xacro:wheel prefix="left" y_offset="0.15"/>
  <xacro:wheel prefix="right" y_offset="-0.15"/>
  
</robot>
```

**Deliverable:** Complete URDF viewable in RViz

---

## Weeks 6-7: Robot Simulation

### Week 6: Gazebo Simulation

**Learning Objectives:**
- Set up Gazebo simulation
- Spawn robots from URDF
- Simulate sensors

**Topics:**
- Gazebo architecture (Ignition)
- SDF world files
- Gazebo plugins for sensors
- ROS 2-Gazebo bridge

**Lab 6: Simulate Your Robot**
```python
# launch/simulation.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_pkg')
    
    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 
                        'launch', 'gazebo.launch.py')
        ])
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', os.path.join(pkg_share, 'urdf', 'robot.urdf'),
            '-entity', 'my_robot',
            '-x', '0', '-y', '0', '-z', '0.1'
        ]
    )
    
    return LaunchDescription([gazebo, spawn_robot])
```

**Deliverable:** Robot simulating with working sensors

---

### Week 7: Unity Integration

**Learning Objectives:**
- Use Unity for visualization
- Connect Unity to ROS 2
- Build interactive environments

**Topics:**
- Unity Robotics Hub
- ROS TCP Connector
- URDF Importer
- Realistic rendering for training data

**Lab 7: Unity-ROS Bridge**
```csharp
// Unity C# Script
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class ROSController : MonoBehaviour
{
    ROSConnection ros;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>("/cmd_vel");
        ros.Subscribe<PoseStampedMsg>("/robot_pose", PoseCallback);
    }
    
    void PoseCallback(PoseStampedMsg msg)
    {
        // Update Unity robot position from ROS
        transform.position = new Vector3(
            (float)msg.pose.position.x,
            (float)msg.pose.position.z,  // Y-up in Unity
            (float)msg.pose.position.y
        );
    }
}
```

**Deliverable:** Unity scene receiving ROS data

---

## Weeks 8-10: NVIDIA Isaac Platform

### Week 8: Isaac Sim Introduction

**Learning Objectives:**
- Install and navigate Isaac Sim
- Import robots into Omniverse
- Create simulation worlds

**Topics:**
- Omniverse platform
- Isaac Sim interface
- USD format for assets
- Replicator for synthetic data

**Lab 8: Isaac Sim First Steps**
```python
# Isaac Sim Python script
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.nucleus import get_assets_root_path

# Create world
world = World(stage_units_in_meters=1.0)

# Add ground plane
world.scene.add_default_ground_plane()

# Add robot
assets_root = get_assets_root_path()
robot_path = assets_root + "/Isaac/Robots/Jetbot/jetbot.usd"
robot = world.scene.add(Robot(prim_path="/World/Robot", usd_path=robot_path))

# Initialize
world.reset()

# Simulation loop
while True:
    world.step(render=True)
```

**Deliverable:** Robot running in Isaac Sim

---

### Week 9: Isaac ROS Perception

**Learning Objectives:**
- Use GPU-accelerated perception
- Implement Visual SLAM
- Build 3D maps with NVBlox

**Topics:**
- Isaac ROS packages
- VSLAM configuration
- NVBlox for mapping
- TensorRT optimization

**Lab 9: Visual SLAM Pipeline**
```yaml
# isaac_ros_vslam.yaml
visual_slam_node:
  ros__parameters:
    enable_imu_fusion: true
    imu_frame: "imu_link"
    
    # Performance
    enable_ground_constraint_in_odometry: true
    enable_localization_n_mapping: true
    
    # Quality
    enable_verbosity: false
    path_max_size: 1024
```

**Deliverable:** Working VSLAM with 3D reconstruction

---

### Week 10: Navigation with Nav2

**Learning Objectives:**
- Configure Nav2 navigation stack
- Implement autonomous navigation
- Handle dynamic obstacles

**Topics:**
- Nav2 architecture
- Costmaps and planners
- Behavior trees
- Recovery behaviors

**Lab 10: Full Navigation System**
```python
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

class AutonomousNavigator:
    def __init__(self):
        self.navigator = BasicNavigator()
        
    def go_to_location(self, location_name):
        """Navigate to named location"""
        locations = {
            'kitchen': (5.0, 2.0, 1.57),
            'living_room': (0.0, 0.0, 0.0),
            'bedroom': (-3.0, 4.0, 3.14),
        }
        
        if location_name not in locations:
            return False
            
        x, y, yaw = locations[location_name]
        
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        
        # Quaternion from yaw
        import math
        goal.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.orientation.w = math.cos(yaw / 2)
        
        self.navigator.goToPose(goal)
        
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            print(f"Distance remaining: {feedback.distance_remaining:.2f}m")
            
        return self.navigator.getResult()
```

**Deliverable:** Robot navigating between waypoints

---

## Weeks 11-12: Humanoid Development

### Week 11: Humanoid Kinematics

**Learning Objectives:**
- Understand humanoid robot structure
- Implement inverse kinematics
- Control humanoid limbs

**Topics:**
- Forward and inverse kinematics
- Denavit-Hartenberg parameters
- MoveIt 2 for motion planning
- Humanoid arm control

**Lab 11: Humanoid Arm Control**
```python
from moveit2 import MoveGroupInterface
import rclpy

class HumanoidArmController:
    def __init__(self):
        self.move_group = MoveGroupInterface("arm")
        
    def reach_position(self, x, y, z):
        """Move end effector to position"""
        pose_goal = Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        pose_goal.orientation.w = 1.0
        
        self.move_group.set_pose_target(pose_goal)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        
        return success
        
    def pick_object(self, object_pose):
        """Pick up an object"""
        # Move above object
        self.reach_position(
            object_pose.x,
            object_pose.y,
            object_pose.z + 0.1
        )
        
        # Move down
        self.reach_position(
            object_pose.x,
            object_pose.y,
            object_pose.z
        )
        
        # Close gripper
        self.close_gripper()
        
        # Lift
        self.reach_position(
            object_pose.x,
            object_pose.y,
            object_pose.z + 0.2
        )
```

**Deliverable:** Humanoid arm executing pick task

---

### Week 12: Bipedal Locomotion

**Learning Objectives:**
- Understand walking dynamics
- Implement balance control
- Simulate bipedal walking

**Topics:**
- Zero Moment Point (ZMP)
- Walking pattern generation
- Balance controllers
- Whole-body control

**Lab 12: Walking Controller**
```python
class BipedalWalkingController:
    def __init__(self):
        self.step_length = 0.3  # meters
        self.step_height = 0.05
        self.step_duration = 0.5  # seconds
        
    def generate_footstep_plan(self, target_position):
        """Generate sequence of footsteps to target"""
        footsteps = []
        current_pos = [0, 0]
        
        dx = target_position[0] - current_pos[0]
        dy = target_position[1] - current_pos[1]
        
        distance = math.sqrt(dx**2 + dy**2)
        num_steps = int(distance / self.step_length) + 1
        
        for i in range(num_steps):
            footsteps.append({
                'foot': 'left' if i % 2 == 0 else 'right',
                'position': [
                    current_pos[0] + (dx/num_steps) * (i+1),
                    current_pos[1] + (dy/num_steps) * (i+1)
                ]
            })
            
        return footsteps
        
    def execute_step(self, step):
        """Execute single step with ZMP control"""
        # Shift weight to stance foot
        self.shift_zmp(step['foot'])
        
        # Lift swing foot
        self.lift_foot(step['foot'], self.step_height)
        
        # Move swing foot forward
        self.move_foot(step['foot'], step['position'])
        
        # Lower swing foot
        self.lower_foot(step['foot'])
```

**Deliverable:** Simulated humanoid walking

---

## Week 13: VLA & Capstone

### Week 13: Voice-Language-Action

**Learning Objectives:**
- Integrate speech recognition
- Connect LLMs for planning
- Build complete VLA pipeline

**Topics:**
- Whisper for speech-to-text
- LLM task planning
- Vision-language models
- End-to-end VLA systems

**Lab 13: Complete VLA System**
```python
class VLARobot:
    def __init__(self):
        self.voice = WhisperInterface()
        self.planner = GeminiPlanner()
        self.vision = VisionLanguageModel()
        self.navigator = AutonomousNavigator()
        
    def listen_and_act(self):
        # Voice to text
        command = self.voice.listen()
        print(f"Heard: {command}")
        
        # Get scene understanding
        scene = self.vision.describe_scene()
        
        # LLM planning
        plan = self.planner.create_plan(command, scene)
        print(f"Plan: {plan}")
        
        # Execute plan
        for action in plan:
            self.execute_action(action)
            
    def execute_action(self, action):
        if action['type'] == 'navigate':
            self.navigator.go_to_location(action['target'])
        elif action['type'] == 'pick':
            self.arm.pick_object(action['object'])
        elif action['type'] == 'say':
            self.speak(action['message'])
```

**Deliverable:** Complete VLA demonstration

---

## Capstone Presentations

### Week 13 (Final Day): Demo Day

**Format:**
- 10-minute live demonstration
- 5-minute Q&A
- 5-minute feedback

**Evaluation Criteria:**

| Criteria | Points |
|----------|--------|
| Voice Command Recognition | 20 |
| LLM Task Planning | 20 |
| Autonomous Navigation | 20 |
| Object Detection/Manipulation | 20 |
| Robot Communication | 10 |
| Code Quality & Documentation | 10 |

**Capstone Requirements:**
1. Robot responds to at least 5 different voice commands
2. Navigates autonomously through environment
3. Detects and interacts with at least 3 object types
4. Provides verbal feedback to user
5. Complete README with setup instructions
6. 3-minute demo video

---

## Resources

### Documentation
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [NVIDIA Isaac ROS](https://nvidia-isaac-ros.github.io/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [MoveIt 2](https://moveit.picknik.ai/)

### Community
- ROS Discourse: discourse.ros.org
- NVIDIA Isaac Discord
- GitHub Discussions

### Hardware
- [Intel RealSense SDK](https://www.intelrealsense.com/sdk-2/)
- [NVIDIA Jetson Developer Zone](https://developer.nvidia.com/embedded/jetson)

---

**Back to**: [Course Introduction →](./intro)
