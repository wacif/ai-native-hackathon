---
sidebar_position: 3
---

# ماڈیول 2: ڈیجیٹل ٹوئن (Gazebo اور Unity)

> **توجہ: Physics simulation اور ماحول کی تعمیر**

حقیقی روبوٹس (مہنگے، نازک، خطرناک) پر AI deploy کرنے سے پہلے، ہم سب کچھ ایک **digital twin** میں simulate کرتے ہیں۔ یہ ماڈیول آپ کو سکھاتا ہے کہ درست physics، سینسرز، اور visualizations کے ساتھ حقیقت پسندانہ virtual environments کیسے بنائیں۔

## Simulation کیوں اہم ہے

حقیقی روبوٹس:
- 💰 **مہنگے** ہیں - ایک crash ہزاروں کا نقصان کر سکتا ہے
- ⏱️ **iterate کرنے میں سست** - جسمانی testing میں وقت لگتا ہے
- ⚠️ **خطرناک** - bugs حقیقی نقصان پہنچا سکتے ہیں
- 🔁 **دوبارہ پیدا کرنا مشکل** - حقیقی دنیا کے حالات مختلف ہوتے ہیں

Simulation آپ کو دیتی ہے:
- لاکھوں iterations کے ساتھ AI کو train کریں
- محفوظ طریقے سے edge cases کی جانچ کریں
- bugs کو بالکل دوبارہ پیدا کریں
- گھنٹوں کی بجائے سیکنڈوں میں iterate کریں

## Simulation Stack

```
┌─────────────────────────────────────────┐
│           آپ کا ROS 2 Code               │
├─────────────────────────────────────────┤
│      Gazebo/Isaac Sim (Physics)         │
├─────────────────────────────────────────┤
│        Unity/Unreal (Visuals)           │
└─────────────────────────────────────────┘
```

| Tool | طاقتیں | بہترین |
|------|--------|--------|
| **Gazebo** | Open-source، ROS integration | تحقیق، prototyping |
| **Isaac Sim** | RTX rendering، AI training | Production، sim-to-real |
| **Unity** | Game-quality visuals، آسان | Demos، human interaction |
| **Unreal** | Photorealism | Film-quality renders |

## Gazebo Simulation

### Gazebo (Ignition) انسٹال کرنا

```bash
# Gazebo Fortress (LTS) انسٹال کریں
sudo apt install ros-humble-gazebo-ros-pkgs

# ROS 2 - Gazebo integration انسٹال کریں
sudo apt install ros-humble-ros-gz

# انسٹالیشن تصدیق کریں
gz sim --version
```

### آپ کی پہلی Simulation

ایک سادہ world file `my_world.sdf` بنائیں:

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="robot_world">
    
    <!-- زمین کا Plane -->
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
    
    <!-- سورج کی روشنی -->
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

لانچ کریں:
```bash
gz sim my_world.sdf
```

### روبوٹ شامل کرنا

`robot.urdf.xacro` بنائیں:

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
  
  <!-- 4 پہیے بنائیں -->
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

### سینسرز کی Simulation

#### LiDAR سینسر

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

#### Depth Camera (RealSense جیسا)

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

Gazebo حقیقی physics simulate کرتا ہے:

### Gravity اور Collisions

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

### رگڑ اور رابطہ

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

## High-Fidelity Rendering کے لیے Unity

جبکہ Gazebo physics سنبھالتا ہے، Unity فراہم کرتا ہے:
- Photorealistic گرافکس
- Human character animation
- VR/AR integration
- Game-quality environments

### Unity Robotics Hub سیٹ اپ

```bash
# Unity Hub انسٹال کریں
# https://unity.com/download سے ڈاؤن لوڈ کریں

# نیا 3D (URP) پروجیکٹ بنائیں

# Package Manager کے ذریعے packages انسٹال کریں:
# - ROS TCP Connector
# - URDF Importer
```

### Unity کو ROS 2 سے جوڑنا

1. **Unity میں**: `ROS TCP Connector` component شامل کریں
2. **ROS IP سیٹ کریں**: `127.0.0.1` (یا روبوٹ IP)
3. **Port سیٹ کریں**: `10000`

```csharp
// Unity C# - ROS topics subscribe کرنا
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
        
        // روبوٹ transform پر apply کریں
        transform.Translate(Vector3.forward * linear * Time.deltaTime);
        transform.Rotate(Vector3.up * angular * Time.deltaTime);
    }
}
```

### Unity میں Human-Robot Interaction

Unity انسانوں کو simulate کرنے میں ماہر ہے:

```csharp
// ایک انسان کو روبوٹ کی طرف چلتے ہوئے animate کریں
public class HumanAgent : MonoBehaviour
{
    public Transform robot;
    public Animator animator;
    public float walkSpeed = 1.5f;
    
    void Update()
    {
        // روبوٹ کی طرف چلیں
        Vector3 direction = (robot.position - transform.position).normalized;
        transform.position += direction * walkSpeed * Time.deltaTime;
        transform.LookAt(robot);
        
        // چلنے کا animation شروع کریں
        animator.SetBool("isWalking", true);
    }
}
```

## Simulation کے ساتھ ROS 2 Launch

Simulation کے لیے مکمل launch file:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_pkg')
    
    # Gazebo شروع کریں
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 
                        'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': os.path.join(pkg_dir, 'worlds', 'my_world.sdf')}.items()
    )
    
    # روبوٹ spawn کریں
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
    
    # Visualization کے لیے RViz
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

## عملی مشق: Test Environment بنائیں

ایک simulation world بنائیں جس میں:

1. Checkerboard texture والا **زمینی plane**
2. **رکاوٹیں** (boxes، cylinders)
3. LiDAR والا **آپ کا روبوٹ**
4. Manual control کے لیے **ROS 2 teleop**

```bash
# Simulation چلائیں
ros2 launch my_robot_pkg simulation.launch.py

# روبوٹ کنٹرول کریں
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# RViz میں LiDAR visualize کریں
ros2 run rviz2 rviz2
```

## Sim-to-Real کے لیے غور و فکر

Simulation کبھی بھی کامل نہیں ہوتی۔ ان باتوں پر دھیان دیں:

| مسئلہ | پریشانی | حل |
|-------|---------|-----|
| **Reality Gap** | Physics حقیقی دنیا سے نہیں ملتی | Domain randomization |
| **Sensor Noise** | حقیقی سینسرز میں noise ہوتی ہے | Sim میں noise شامل کریں |
| **Latency** | حقیقی سسٹمز میں تاخیر ہوتی ہے | تاخیر simulate کریں |
| **Dynamics** | موٹر response مختلف ہوتا ہے | System identification |

### Domain Randomization

Training کے دوران simulation parameters کو randomly بدلیں:

```python
import random

def randomize_world():
    # روشنی randomize کریں
    sun_intensity = random.uniform(0.5, 1.5)
    
    # رگڑ randomize کریں
    floor_friction = random.uniform(0.5, 1.2)
    
    # آبجیکٹ پوزیشنز randomize کریں
    obstacle_x = random.uniform(-2, 2)
    obstacle_y = random.uniform(-2, 2)
    
    # سینسر noise randomize کریں
    lidar_noise = random.uniform(0.01, 0.05)
    
    return {
        'sun_intensity': sun_intensity,
        'friction': floor_friction,
        'obstacle_pos': (obstacle_x, obstacle_y),
        'sensor_noise': lidar_noise
    }
```

## اہم نکات

1. **Deploy کرنے سے پہلے simulate کریں** - وقت، پیسے، اور ہارڈویئر بچائیں
2. **Physics کے لیے Gazebo** - درست dynamics اور sensor simulation
3. **Visuals کے لیے Unity** - Photorealistic human-robot interaction
4. **URDF روبوٹس بیان کرتا ہے** - Simulators میں portable
5. **Sensor noise شامل کریں** - Simulation کو حقیقت پسندانہ بنائیں
6. **Domain randomization** - Sim-to-real gap پل کریں

## آگے کیا ہے؟

**ماڈیول 3** میں، آپ photorealistic simulation، synthetic data generation، اور hardware-accelerated perception pipelines کے لیے **NVIDIA Isaac** استعمال کریں گے۔

---

**اگلا**: [ماڈیول 3: AI-Robot دماغ (NVIDIA Isaac) →](./module3-isaac)
