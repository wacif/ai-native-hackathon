---
sidebar_position: 4
---

# ماڈیول 3: AI-Robot دماغ (NVIDIA Isaac)

> **توجہ: Perception، navigation، اور GPU-accelerated روبوٹکس**

NVIDIA Isaac Physical AI کے لیے **industrial-grade** پلیٹ فارم ہے۔ جبکہ Gazebo تحقیق کے لیے بہترین ہے، Isaac photorealistic simulation، synthetic data generation، اور production روبوٹکس کے لیے ضروری optimized perception pipelines فراہم کرتا ہے۔

## NVIDIA Isaac ایکو سسٹم

```
┌──────────────────────────────────────────────────────────┐
│                    NVIDIA Isaac                          │
├──────────────┬──────────────┬──────────────┬────────────┤
│  Isaac Sim   │  Isaac ROS   │  Isaac Lab   │  Jetson    │
│  (Omniverse) │  (Perception)│  (RL/AI)     │  (Deploy)  │
├──────────────┼──────────────┼──────────────┼────────────┤
│ Photorealism │ CUDA-accel   │ Gym envs     │ Edge AI    │
│ Synthetic    │ Nav2 stack   │ Isaac Gym    │ Nano/Orin  │
│ data         │ VSLAM        │ Dex manip    │ TensorRT   │
└──────────────┴──────────────┴──────────────┴────────────┘
```

## Isaac کیوں؟

| فیچر | Gazebo | Isaac Sim |
|------|--------|-----------|
| Rendering | بنیادی | RTX Raytracing |
| Physics | ODE/Bullet | PhysX 5 |
| AI Training | محدود | Native Isaac Lab |
| Synthetic Data | دستی | خودکار SDG |
| ROS 2 Integration | اچھا | عمدہ |
| Sim-to-Real | Gap موجود ہے | کم سے کم |

## Isaac Sim سیٹ اپ

### ہارڈویئر کی ضروریات

```
کم از کم:
- NVIDIA RTX 3070 (8GB VRAM)
- 32GB RAM
- 100GB SSD

تجویز کردہ (یہ کورس استعمال کرتا ہے):
- NVIDIA RTX 4070 Ti یا اس سے زیادہ (12GB+ VRAM)
- 64GB RAM
- 500GB NVMe SSD
```

### انسٹالیشن

```bash
# Omniverse Launcher انسٹال کریں
# https://www.nvidia.com/omniverse سے ڈاؤن لوڈ کریں

# Omniverse Launcher کے ذریعے:
# 1. سائن ان / اکاؤنٹ بنائیں
# 2. Exchange پر جائیں
# 3. "Isaac Sim" انسٹال کریں
# 4. Isaac Sim لانچ کریں

# Isaac ROS انسٹال کریں (ROS 2 integration کے لیے)
cd ~/workspaces/isaac_ros-dev
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common

# Docker container بلڈ کریں
./scripts/run_dev.sh
```

## Isaac ROS: GPU-Accelerated Perception

Isaac ROS عام روبوٹکس الگورتھمز کے **CUDA-accelerated** ورژن فراہم کرتا ہے:

### Package کا جائزہ

| Package | فنکشن | CPU سے Speedup |
|---------|-------|----------------|
| `isaac_ros_visual_slam` | Visual SLAM | 10x |
| `isaac_ros_nvblox` | 3D Reconstruction | 20x |
| `isaac_ros_dnn_inference` | Neural networks | 100x |
| `isaac_ros_apriltag` | Fiducial detection | 5x |
| `isaac_ros_image_pipeline` | Image processing | 15x |

### Visual SLAM (VSLAM)

صرف کیمروں کا استعمال کرتے ہوئے روبوٹ کی پوزیشن track کریں:

```bash
# Isaac ROS Visual SLAM لانچ کریں
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

```python
# VSLAM pose استعمال کرنے کے لیے Python node
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class VSLAMPoseTracker(Node):
    def __init__(self):
        super().__init__('vslam_pose_tracker')
        
        self.subscription = self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self.pose_callback,
            10
        )
        
    def pose_callback(self, msg):
        # پوزیشن نکالیں
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # Orientation نکالیں (quaternion)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        self.get_logger().info(f'روبوٹ پر: ({x:.2f}, {y:.2f}, {z:.2f})')

def main():
    rclpy.init()
    node = VSLAMPoseTracker()
    rclpy.spin(node)
```

### NVBlox: 3D Reconstruction

Navigation کے لیے real-time 3D maps بنائیں:

```bash
# RealSense کے ساتھ NVBlox لانچ کریں
ros2 launch nvblox_examples_bringup isaac_sim_example.launch.py
```

```yaml
# nvblox_params.yaml
nvblox_node:
  ros__parameters:
    # Voxel resolution
    voxel_size: 0.05  # 5cm voxels
    
    # Integration
    max_integration_distance: 7.0
    
    # Mesh generation
    mesh_update_rate_hz: 5.0
    
    # Planning کے لیے ESDF (Euclidean Signed Distance Field)
    esdf_update_rate_hz: 5.0
    
    # GPU میموری مینجمنٹ
    max_tsdf_update_hz: 30.0
```

### CUDA کے ساتھ Object Detection

```python
# Object detection کے لیے Isaac ROS DNN استعمال کرنا
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        self.bridge = CvBridge()
        
        # Isaac ROS DNN سے detections subscribe کریں
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detectnet/detections',
            self.detection_callback,
            10
        )
        
        # Annotated image subscribe کریں
        self.image_sub = self.create_subscription(
            Image,
            '/detectnet/image_out',
            self.image_callback,
            10
        )
        
    def detection_callback(self, msg):
        for detection in msg.detections:
            # Bounding box حاصل کریں
            bbox = detection.bbox
            center_x = bbox.center.position.x
            center_y = bbox.center.position.y
            width = bbox.size_x
            height = bbox.size_y
            
            # Class حاصل کریں (person، car، وغیرہ)
            if detection.results:
                class_id = detection.results[0].hypothesis.class_id
                score = detection.results[0].hypothesis.score
                
                self.get_logger().info(
                    f'{class_id} ({score:.2f}) پر ({center_x}, {center_y}) پایا'
                )
```

## Nav2: Robot Navigation

Nav2 ROS 2 کے لیے **standard navigation stack** ہے۔ Isaac ROS اسے GPU کے ساتھ accelerate کرتا ہے:

### Navigation Architecture

```
                    ┌─────────────┐
                    │  Goal Pose  │
                    └──────┬──────┘
                           │
                    ┌──────▼──────┐
                    │   Planner   │ (Global path)
                    │ Server      │
                    └──────┬──────┘
                           │
                    ┌──────▼──────┐
                    │ Controller  │ (Local control)
                    │ Server      │
                    └──────┬──────┘
                           │
                    ┌──────▼──────┐
                    │ Recovery    │ (Stuck handling)
                    │ Server      │
                    └──────┬──────┘
                           │
                    ┌──────▼──────┐
                    │  /cmd_vel   │
                    └─────────────┘
```

### Nav2 Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    
    # Map file
    map_yaml = LaunchConfiguration('map')
    
    # Nav2 parameters
    nav2_params = os.path.join(
        get_package_share_directory('my_robot_nav'),
        'config',
        'nav2_params.yaml'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='map.yaml'),
        
        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': map_yaml}]
        ),
        
        # AMCL localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[nav2_params]
        ),
        
        # Planner server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[nav2_params]
        ),
        
        # Controller server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[nav2_params]
        ),
        
        # Behavior tree navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[nav2_params]
        ),
        
        # Lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'map_server', 'amcl', 'planner_server',
                    'controller_server', 'bt_navigator'
                ]
            }]
        )
    ])
```

### Nav2 Parameters

```yaml
# nav2_params.yaml
amcl:
  ros__parameters:
    robot_model_type: "differential"
    min_particles: 500
    max_particles: 2000
    
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: true

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.5
      max_vel_theta: 1.0
      min_vel_x: 0.0
      acc_lim_x: 2.5
      acc_lim_theta: 3.2
```

### Navigation Goals بھیجنا

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class NavigationCommander(Node):
    def __init__(self):
        super().__init__('nav_commander')
        self.navigator = BasicNavigator()
        
    def go_to_pose(self, x, y, yaw):
        """روبوٹ کو مخصوص pose پر بھیجیں"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        
        # Yaw کو quaternion میں بدلیں
        import math
        goal_pose.pose.orientation.z = math.sin(yaw / 2)
        goal_pose.pose.orientation.w = math.cos(yaw / 2)
        
        self.navigator.goToPose(goal_pose)
        
        # مکمل ہونے کا انتظار کریں
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(
                    f'باقی فاصلہ: {feedback.distance_remaining:.2f}m'
                )
        
        result = self.navigator.getResult()
        self.get_logger().info(f'Navigation نتیجہ: {result}')
        return result

def main():
    rclpy.init()
    commander = NavigationCommander()
    
    # کچن میں navigate کریں
    commander.go_to_pose(5.0, 3.0, 1.57)  # x، y، yaw
    
    # لونگ روم میں navigate کریں
    commander.go_to_pose(0.0, 0.0, 0.0)
```

## Synthetic Data Generation (SDG)

Isaac Sim **لامحدود training data** generate کر سکتا ہے:

### خودکار Labeling

Isaac خود بخود generate کرتا ہے:
- 2D bounding boxes
- 3D bounding boxes
- Semantic segmentation
- Instance segmentation
- Depth maps
- Surface normals
- Optical flow

### Dataset بنانے کے لیے Replicator

```python
# Isaac Sim Replicator script
import omni.replicator.core as rep

# کیمرا سیٹ اپ کریں
camera = rep.create.camera(position=(0, 5, 5), look_at=(0, 0, 0))

# Randomized scene بنائیں
with rep.trigger.on_frame(num_frames=1000):
    # روشنی randomize کریں
    rep.randomizer.light(
        light_type="dome",
        intensity=rep.distribution.uniform(0.5, 1.5),
        temperature=rep.distribution.uniform(4500, 6500)
    )
    
    # Object positions randomize کریں
    with rep.get.prims(semantics=[("class", "robot")]):
        rep.modify.pose(
            position=rep.distribution.uniform((-2, 0, 0), (2, 0, 0)),
            rotation=rep.distribution.uniform((0, 0, 0), (0, 360, 0))
        )
    
    # Textures randomize کریں
    with rep.get.prims(semantics=[("class", "floor")]):
        rep.randomizer.texture(
            textures=["wood_floor", "concrete", "carpet"],
            project_uvw=True
        )

# Output سیٹ اپ کریں
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="_output",
    rgb=True,
    bounding_box_2d_tight=True,
    semantic_segmentation=True,
    distance_to_camera=True
)
writer.attach([camera])

rep.orchestrator.run()
```

## Reinforcement Learning کے لیے Isaac Lab

لاکھوں parallel simulations کے ساتھ روبوٹس کو train کریں:

```python
# Isaac Lab environment سیٹ اپ
from omni.isaac.lab.envs import ManagerBasedRLEnv
from omni.isaac.lab.utils import configclass

@configclass
class RobotEnvCfg:
    """Robot RL environment کے لیے Configuration"""
    
    # Scene
    scene: SceneCfg = SceneCfg(num_envs=4096, env_spacing=2.0)
    
    # Observations
    observations: ObservationsCfg = ObservationsCfg()
    
    # Actions
    actions: ActionsCfg = ActionsCfg()
    
    # Rewards
    rewards: RewardsCfg = RewardsCfg()
    
    # Terminations
    terminations: TerminationsCfg = TerminationsCfg()

class RobotEnv(ManagerBasedRLEnv):
    cfg: RobotEnvCfg
    
    def __init__(self, cfg: RobotEnvCfg):
        super().__init__(cfg)
        
    def _setup_scene(self):
        # روبوٹ شامل کریں
        self.robot = Articulation(self.cfg.robot_cfg)
        self.scene.articulations["robot"] = self.robot
        
        # زمین شامل کریں
        self.scene.ground = GroundPlaneCfg()
        
    def _get_observations(self):
        # Joint positions اور velocities
        return {
            "policy": torch.cat([
                self.robot.data.joint_pos,
                self.robot.data.joint_vel,
                self.robot.data.root_pos_w,
            ], dim=-1)
        }
```

### Stable-Baselines3 کے ساتھ Training

```python
from stable_baselines3 import PPO
from omni.isaac.lab_tasks.utils.wrappers.sb3 import Sb3VecEnvWrapper

# Environment بنائیں
env = gym.make("Isaac-Navigation-v0", num_envs=4096)
env = Sb3VecEnvWrapper(env)

# Train کریں
model = PPO(
    "MlpPolicy",
    env,
    verbose=1,
    n_steps=24,
    batch_size=24 * 4096,
    learning_rate=3e-4,
    device="cuda"
)

model.learn(total_timesteps=100_000_000)
model.save("navigation_policy")
```

## Jetson پر Deploy کرنا

Simulation سے حقیقی ہارڈویئر پر منتقلی:

### TensorRT Optimization

```python
# PyTorch ماڈل کو TensorRT میں بدلیں
import torch
import tensorrt as trt

def optimize_for_jetson(model_path, output_path):
    """Jetson deployment کے لیے ماڈل convert کریں"""
    
    # PyTorch ماڈل لوڈ کریں
    model = torch.load(model_path)
    model.eval()
    
    # ONNX میں export کریں
    dummy_input = torch.randn(1, 3, 224, 224).cuda()
    torch.onnx.export(
        model,
        dummy_input,
        "model.onnx",
        opset_version=11,
        input_names=["input"],
        output_names=["output"]
    )
    
    # ONNX کو TensorRT میں convert کریں
    # (عام طور پر Jetson ڈیوائس پر کیا جاتا ہے)
    import subprocess
    subprocess.run([
        "/usr/src/tensorrt/bin/trtexec",
        "--onnx=model.onnx",
        f"--saveEngine={output_path}",
        "--fp16"  # رفتار کے لیے FP16 استعمال کریں
    ])
```

### Jetson ROS 2 Node

```python
# Jetson Orin پر inference چلانا
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import tensorrt as trt
import pycuda.driver as cuda
import numpy as np

class JetsonNavigator(Node):
    def __init__(self):
        super().__init__('jetson_navigator')
        
        # TensorRT engine لوڈ کریں
        self.engine = self._load_engine("navigation_policy.trt")
        self.context = self.engine.create_execution_context()
        
        # GPU buffers allocate کریں
        self._allocate_buffers()
        
        # ROS 2 interfaces
        self.image_sub = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def _load_engine(self, path):
        with open(path, "rb") as f:
            engine_data = f.read()
        runtime = trt.Runtime(trt.Logger(trt.Logger.WARNING))
        return runtime.deserialize_cuda_engine(engine_data)
        
    def image_callback(self, msg):
        # Image preprocess کریں
        image = self._preprocess(msg)
        
        # Inference چلائیں
        output = self._infer(image)
        
        # Velocity command میں بدلیں
        cmd = Twist()
        cmd.linear.x = float(output[0])
        cmd.angular.z = float(output[1])
        
        self.cmd_pub.publish(cmd)
```

## عملی مشق: مکمل Navigation Stack

ایک روبوٹ بنائیں جو:
1. Localization کے لیے VSLAM استعمال کرے (GPS کی ضرورت نہیں)
2. NVBlox کے ساتھ 3D map بنائے
3. Nav2 کے ساتھ navigate کرے
4. رکاوٹوں کو detect اور avoid کرے

```bash
# مکمل stack لانچ کریں
ros2 launch my_robot_isaac complete_navigation.launch.py

# RViz میں visualize کریں
ros2 run rviz2 rviz2 -d navigation.rviz

# Navigation goal بھیجیں
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0, z: 0.0}}}}"
```

## اہم نکات

1. **Photorealism کے لیے Isaac Sim** - RTX rendering sim-to-real gap کم کرتی ہے
2. **رفتار کے لیے Isaac ROS** - Real-time perception کے لیے GPU acceleration
3. **Navigation کے لیے Nav2** - Autonomous movement کے لیے مکمل stack
4. **بڑے پیمانے پر Synthetic data** - لاکھوں labeled images generate کریں
5. **RL کے لیے Isaac Lab** - 4096+ parallel environments کے ساتھ policies train کریں
6. **Deployment کے لیے Jetson** - TensorRT optimization کے ساتھ Edge AI

## آگے کیا ہے؟

**ماڈیول 4** میں، آپ آخری ٹکڑا شامل کریں گے: **Voice-Language-Action (VLA)** ماڈلز جو انسانوں کو قدرتی زبان سے روبوٹس کنٹرول کرنے دیتے ہیں۔

---

**اگلا**: [ماڈیول 4: Voice-to-Action (VLA) →](./module4-vla)
