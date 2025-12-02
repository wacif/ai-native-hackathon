---
sidebar_position: 6
---

# ہفتہ وار شیڈول اور لیبز

> **13 ہفتوں کے کورس کی تفصیلی breakdown**

یہ دستاویز ہر ہفتے کے لیے lab assignments، پڑھائی، اور deliverables کے ساتھ مکمل شیڈول فراہم کرتی ہے۔

## کورس کیلنڈر کا جائزہ

```
ہفتہ 1-2   │ تعارف اور بنیادیں
ہفتہ 3-5   │ ماڈیول 1: ROS 2 بنیادیات  
ہفتہ 6-7   │ ماڈیول 2: Simulation (Gazebo/Unity)
ہفتہ 8-10  │ ماڈیول 3: NVIDIA Isaac پلیٹ فارم
ہفتہ 11-12 │ ہیومنائڈ ڈویلپمنٹ
ہفتہ 13    │ ماڈیول 4: VLA + Capstone پریزنٹیشنز
```

---

## ہفتے 1-2: Physical AI کا تعارف

### ہفتہ 1: Physical AI کیا ہے؟

**سیکھنے کے مقاصد:**
- ڈیجیٹل AI سے embodied AI کی طرف paradigm shift سمجھیں
- ہیومنائڈ روبوٹکس کے منظر نامے کا جائزہ لیں
- ڈویلپمنٹ ماحول سیٹ اپ کریں

**موضوعات:**
- ChatGPT سے روبوٹس تک جو عمل کر سکتے ہیں
- ہیومنائڈز کیوں؟ انسان مرکوز ڈیزائن کی دلیل
- سینسر سے عمل تک کا loop
- Physical AI stack کا تعارف

**Lab 1: ماحول سیٹ اپ**
```bash
# Ubuntu 22.04 انسٹال کریں (اگر پہلے سے نہیں)
# ROS 2 Humble انسٹال کریں
sudo apt install ros-humble-desktop

# Isaac dependencies انسٹال کریں
sudo apt install nvidia-cuda-toolkit

# سیٹ اپ تصدیق کریں
ros2 doctor
nvidia-smi
```

**پڑھائی:**
- NVIDIA کا "Physical AI" وائٹ پیپر
- [Why Humanoids Will Win](https://www.figure.ai/news)

---

### ہفتہ 2: سینسرز اور Perception کی بنیادیں

**سیکھنے کے مقاصد:**
- روبوٹ sensing modalities سمجھیں
- Intel RealSense کیمرا سیٹ اپ کریں
- بنیادی sensor data process کریں

**موضوعات:**
- سینسر کی اقسام: کیمرے، LiDAR، IMU، force/torque
- کیمرا intrinsics اور extrinsics
- Point clouds اور depth perception
- Sensor fusion کے تصورات

**Lab 2: RealSense Depth Camera**
```python
import pyrealsense2 as rs
import numpy as np
import cv2

# Pipeline بنائیں
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Streaming شروع کریں
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        # Numpy میں بدلیں
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # Colormap apply کریں
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), 
            cv2.COLORMAP_JET
        )
        
        # دکھائیں
        cv2.imshow('Depth', depth_colormap)
        cv2.imshow('Color', color_image)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
```

**Deliverable:** Sensor data visualization کا screenshot

---

## ہفتے 3-5: ROS 2 بنیادیات

### ہفتہ 3: ROS 2 Architecture

**سیکھنے کے مقاصد:**
- ROS 2 کے بنیادی تصورات میں مہارت حاصل کریں
- ROS 2 nodes بنائیں اور چلائیں
- Publish/subscribe pattern سمجھیں

**موضوعات:**
- ROS 2 بمقابلہ ROS 1
- DDS (Data Distribution Service)
- Nodes، Topics، Services، Actions
- Quality of Service (QoS)

**Lab 3: آپ کی پہلی ROS 2 Package**
```bash
# Workspace بنائیں
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Package بنائیں
ros2 pkg create --build-type ament_python --node-name my_node my_robot_pkg

# بلڈ کریں
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# چلائیں
ros2 run my_robot_pkg my_node
```

**Deliverable:** Sensor data publish کرنے والی working package

---

### ہفتہ 4: Messages اور Communication

**سیکھنے کے مقاصد:**
- Standard message types استعمال کریں
- Custom messages بنائیں
- Service servers اور clients implement کریں

**موضوعات:**
- Standard messages: `std_msgs`، `sensor_msgs`، `geometry_msgs`
- Custom message definitions
- Service communication patterns
- `ros2 topic` اور `ros2 service` سے debugging

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
        self.get_logger().info('Synchronized data موصول ہوا!')
        # Fusion logic یہاں
```

**Deliverable:** Camera + IMU data synchronize کرنے والی Node

---

### ہفتہ 5: URDF اور Transforms

**سیکھنے کے مقاصد:**
- URDF robot descriptions بنائیں
- TF2 coordinate frames سمجھیں
- RViz میں روبوٹس visualize کریں

**موضوعات:**
- URDF XML ڈھانچہ
- Links، joints، اور visual/collision geometry
- TF2 transform tree
- Robot state publisher

**Lab 5: Mobile Robot URDF بنائیں**
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
  
  <!-- Wheel macro شامل کریں -->
  <xacro:include filename="wheel.urdf.xacro"/>
  <xacro:wheel prefix="left" y_offset="0.15"/>
  <xacro:wheel prefix="right" y_offset="-0.15"/>
  
</robot>
```

**Deliverable:** RViz میں دیکھنے کے قابل مکمل URDF

---

## ہفتے 6-7: Robot Simulation

### ہفتہ 6: Gazebo Simulation

**سیکھنے کے مقاصد:**
- Gazebo simulation سیٹ اپ کریں
- URDF سے روبوٹس spawn کریں
- سینسرز simulate کریں

**موضوعات:**
- Gazebo architecture (Ignition)
- SDF world files
- سینسرز کے لیے Gazebo plugins
- ROS 2-Gazebo bridge

**Lab 6: اپنے روبوٹ کی Simulation**
```python
# launch/simulation.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_pkg')
    
    # Gazebo شروع کریں
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 
                        'launch', 'gazebo.launch.py')
        ])
    )
    
    # روبوٹ spawn کریں
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

**Deliverable:** کام کرنے والے سینسرز کے ساتھ Robot simulation

---

### ہفتہ 7: Unity Integration

**سیکھنے کے مقاصد:**
- Visualization کے لیے Unity استعمال کریں
- Unity کو ROS 2 سے جوڑیں
- Interactive environments بنائیں

**موضوعات:**
- Unity Robotics Hub
- ROS TCP Connector
- URDF Importer
- Training data کے لیے Realistic rendering

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
        // ROS سے Unity روبوٹ position update کریں
        transform.position = new Vector3(
            (float)msg.pose.position.x,
            (float)msg.pose.position.z,  // Unity میں Y-up
            (float)msg.pose.position.y
        );
    }
}
```

**Deliverable:** ROS data وصول کرنے والا Unity scene

---

## ہفتے 8-10: NVIDIA Isaac پلیٹ فارم

### ہفتہ 8: Isaac Sim کا تعارف

**سیکھنے کے مقاصد:**
- Isaac Sim انسٹال کریں اور navigate کریں
- Omniverse میں روبوٹس import کریں
- Simulation worlds بنائیں

**موضوعات:**
- Omniverse پلیٹ فارم
- Isaac Sim interface
- Assets کے لیے USD format
- Synthetic data کے لیے Replicator

**Lab 8: Isaac Sim پہلے اقدامات**
```python
# Isaac Sim Python script
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.nucleus import get_assets_root_path

# World بنائیں
world = World(stage_units_in_meters=1.0)

# Ground plane شامل کریں
world.scene.add_default_ground_plane()

# روبوٹ شامل کریں
assets_root = get_assets_root_path()
robot_path = assets_root + "/Isaac/Robots/Jetbot/jetbot.usd"
robot = world.scene.add(Robot(prim_path="/World/Robot", usd_path=robot_path))

# Initialize کریں
world.reset()

# Simulation loop
while True:
    world.step(render=True)
```

**Deliverable:** Isaac Sim میں چلتا ہوا روبوٹ

---

### ہفتہ 9: Isaac ROS Perception

**سیکھنے کے مقاصد:**
- GPU-accelerated perception استعمال کریں
- Visual SLAM implement کریں
- NVBlox کے ساتھ 3D maps بنائیں

**موضوعات:**
- Isaac ROS packages
- VSLAM configuration
- Mapping کے لیے NVBlox
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

**Deliverable:** 3D reconstruction کے ساتھ کام کرتا VSLAM

---

### ہفتہ 10: Nav2 کے ساتھ Navigation

**سیکھنے کے مقاصد:**
- Nav2 navigation stack configure کریں
- Autonomous navigation implement کریں
- Dynamic obstacles handle کریں

**موضوعات:**
- Nav2 architecture
- Costmaps اور planners
- Behavior trees
- Recovery behaviors

**Lab 10: مکمل Navigation System**
```python
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

class AutonomousNavigator:
    def __init__(self):
        self.navigator = BasicNavigator()
        
    def go_to_location(self, location_name):
        """نامزد جگہ پر navigate کریں"""
        locations = {
            'کچن': (5.0, 2.0, 1.57),
            'لونگ_روم': (0.0, 0.0, 0.0),
            'بیڈ_روم': (-3.0, 4.0, 3.14),
        }
        
        if location_name not in locations:
            return False
            
        x, y, yaw = locations[location_name]
        
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        
        # Yaw سے Quaternion
        import math
        goal.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.orientation.w = math.cos(yaw / 2)
        
        self.navigator.goToPose(goal)
        
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            print(f"باقی فاصلہ: {feedback.distance_remaining:.2f}m")
            
        return self.navigator.getResult()
```

**Deliverable:** Waypoints کے درمیان navigate کرتا روبوٹ

---

## ہفتے 11-12: ہیومنائڈ ڈویلپمنٹ

### ہفتہ 11: ہیومنائڈ Kinematics

**سیکھنے کے مقاصد:**
- ہیومنائڈ روبوٹ کا ڈھانچہ سمجھیں
- Inverse kinematics implement کریں
- ہیومنائڈ اعضاء کنٹرول کریں

**موضوعات:**
- Forward اور inverse kinematics
- Denavit-Hartenberg parameters
- Motion planning کے لیے MoveIt 2
- ہیومنائڈ arm control

**Lab 11: ہیومنائڈ Arm Control**
```python
from moveit2 import MoveGroupInterface
import rclpy

class HumanoidArmController:
    def __init__(self):
        self.move_group = MoveGroupInterface("arm")
        
    def reach_position(self, x, y, z):
        """End effector کو پوزیشن پر لے جائیں"""
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
        """کوئی چیز اٹھائیں"""
        # چیز کے اوپر جائیں
        self.reach_position(
            object_pose.x,
            object_pose.y,
            object_pose.z + 0.1
        )
        
        # نیچے جائیں
        self.reach_position(
            object_pose.x,
            object_pose.y,
            object_pose.z
        )
        
        # Gripper بند کریں
        self.close_gripper()
        
        # اٹھائیں
        self.reach_position(
            object_pose.x,
            object_pose.y,
            object_pose.z + 0.2
        )
```

**Deliverable:** Pick task انجام دینے والی ہیومنائڈ arm

---

### ہفتہ 12: دو پاؤں پر چلنا

**سیکھنے کے مقاصد:**
- چلنے کی dynamics سمجھیں
- Balance control implement کریں
- دو پاؤں پر چلنا simulate کریں

**موضوعات:**
- Zero Moment Point (ZMP)
- Walking pattern generation
- Balance controllers
- Whole-body control

**Lab 12: Walking Controller**
```python
class BipedalWalkingController:
    def __init__(self):
        self.step_length = 0.3  # میٹرز
        self.step_height = 0.05
        self.step_duration = 0.5  # سیکنڈز
        
    def generate_footstep_plan(self, target_position):
        """ہدف تک footsteps کی ترتیب generate کریں"""
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
        """ZMP control کے ساتھ ایک قدم اٹھائیں"""
        # وزن کھڑے پاؤں پر منتقل کریں
        self.shift_zmp(step['foot'])
        
        # جھولتا پاؤں اٹھائیں
        self.lift_foot(step['foot'], self.step_height)
        
        # جھولتا پاؤں آگے لے جائیں
        self.move_foot(step['foot'], step['position'])
        
        # جھولتا پاؤں نیچے رکھیں
        self.lower_foot(step['foot'])
```

**Deliverable:** چلتا ہوا Simulated ہیومنائڈ

---

## ہفتہ 13: VLA اور Capstone

### ہفتہ 13: Voice-Language-Action

**سیکھنے کے مقاصد:**
- Speech recognition integrate کریں
- Planning کے لیے LLMs جوڑیں
- مکمل VLA pipeline بنائیں

**موضوعات:**
- Speech-to-text کے لیے Whisper
- LLM task planning
- Vision-language models
- End-to-end VLA systems

**Lab 13: مکمل VLA System**
```python
class VLARobot:
    def __init__(self):
        self.voice = WhisperInterface()
        self.planner = GeminiPlanner()
        self.vision = VisionLanguageModel()
        self.navigator = AutonomousNavigator()
        
    def listen_and_act(self):
        # آواز سے متن
        command = self.voice.listen()
        print(f"سنا: {command}")
        
        # Scene understanding حاصل کریں
        scene = self.vision.describe_scene()
        
        # LLM planning
        plan = self.planner.create_plan(command, scene)
        print(f"منصوبہ: {plan}")
        
        # منصوبہ چلائیں
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

**Deliverable:** مکمل VLA demonstration

---

## Capstone پریزنٹیشنز

### ہفتہ 13 (آخری دن): Demo Day

**فارمیٹ:**
- 10 منٹ live demonstration
- 5 منٹ سوال جواب
- 5 منٹ feedback

**جائزہ معیار:**

| معیار | پوائنٹس |
|-------|---------|
| Voice Command Recognition | 20 |
| LLM Task Planning | 20 |
| Autonomous Navigation | 20 |
| Object Detection/Manipulation | 20 |
| Robot Communication | 10 |
| Code Quality اور Documentation | 10 |

**Capstone ضروریات:**
1. روبوٹ کم از کم 5 مختلف voice commands پر جواب دے
2. ماحول میں خود مختار navigate کرے
3. کم از کم 3 قسم کی چیزیں detect کرے اور ان کے ساتھ interact کرے
4. صارف کو verbal feedback فراہم کرے
5. Setup instructions کے ساتھ مکمل README
6. 3 منٹ کی demo video

---

## وسائل

### Documentation
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [NVIDIA Isaac ROS](https://nvidia-isaac-ros.github.io/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [MoveIt 2](https://moveit.picknik.ai/)

### کمیونٹی
- ROS Discourse: discourse.ros.org
- NVIDIA Isaac Discord
- GitHub Discussions

### ہارڈویئر
- [Intel RealSense SDK](https://www.intelrealsense.com/sdk-2/)
- [NVIDIA Jetson Developer Zone](https://developer.nvidia.com/embedded/jetson)

---

**واپس**: [کورس تعارف →](./intro)
