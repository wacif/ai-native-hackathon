---
sidebar_position: 2
---

# ماڈیول 1: روبوٹک اعصابی نظام (ROS 2)

> **توجہ: روبوٹ کنٹرول کے لیے Middleware**

ROS 2 (Robot Operating System 2) جدید روبوٹکس کا اعصابی نظام ہے۔ یہ communication infrastructure فراہم کرتا ہے جو روبوٹ کے مختلف حصوں — سینسرز، ایکچویٹرز، AI ماڈیولز — کو آپس میں بات کرنے کی اجازت دیتا ہے۔ اس ماڈیول میں، آپ ROS 2 کی بنیادی باتوں میں مہارت حاصل کریں گے اور Python AI ایجنٹس کو روبوٹ کنٹرولرز سے جوڑنا سیکھیں گے۔

## ROS 2 کیا ہے؟

ROS 2 ایک آپریٹنگ سسٹم نہیں ہے — یہ **middleware** ہے۔ اسے ایک communication protocol کے طور پر سمجھیں جو اجازت دیتا ہے:

- ایک کیمرا تصاویر publish کرے
- ایک perception ماڈل objects detect کرے
- ایک planner فیصلہ کرے کہ کہاں جانا ہے
- موٹرز movement commands حاصل کریں

یہ تمام اجزاء الگ الگ **nodes** کے طور پر چلتے ہیں جو **topics**، **services**، اور **actions** کے ذریعے communicate کرتے ہیں۔

### ROS 1 کی بجائے ROS 2 کیوں؟

| فیچر | ROS 1 | ROS 2 |
|------|-------|-------|
| Real-time | ❌ نہیں | ✅ ہاں (DDS) |
| Multi-robot | ❌ محدود | ✅ Native سپورٹ |
| Security | ❌ کوئی نہیں | ✅ Built-in |
| پلیٹ فارم | صرف Linux | Linux, Windows, macOS |
| Python | Python 2 | Python 3 |

ROS 2 communication کے لیے **DDS (Data Distribution Service)** استعمال کرتا ہے، جو فراہم کرتا ہے:
- Quality of Service (QoS) policies
- Real-time guarantees
- Distributed discovery (کوئی rosmaster نہیں!)

## بنیادی تصورات

### 1. Nodes

ایک **node** ایک واحد process ہے جو ایک مخصوص کام انجام دیتی ہے۔ ایک روبوٹ عام طور پر بہت سی nodes چلاتا ہے:

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Camera    │    │  Detector   │    │   Planner   │
│    Node     │ -> │    Node     │ -> │    Node     │
└─────────────┘    └─────────────┘    └─────────────┘
```

ہر node:
- **واحد مقصد**: ایک کام اچھی طرح کرتی ہے
- **دوبارہ قابل استعمال**: روبوٹس میں شیئر کی جا سکتی ہے
- **بدلنے کے قابل**: دوسروں کو تبدیل کیے بغیر implementations swap کریں

### 2. Topics

**Topics** streaming data کے لیے نامزد buses ہیں۔ Nodes topics پر **publish** کرتی ہیں اور topics سے **subscribe** کرتی ہیں۔

```python
# Publisher: Camera node تصاویر publish کرتی ہے
publisher = node.create_publisher(Image, '/camera/image_raw', 10)

# Subscriber: Detector node تصاویر وصول کرتی ہے
subscription = node.create_subscription(
    Image, '/camera/image_raw', self.image_callback, 10
)
```

عام topic patterns:
- `/camera/image_raw` - خام کیمرا تصاویر
- `/scan` - LiDAR scan data
- `/cmd_vel` - Velocity commands
- `/odom` - Odometry (position/velocity)

### 3. Services

**Services** synchronous request/response کالز ہیں۔ جب آپ کو جواب چاہیے تو انہیں استعمال کریں۔

```python
# Service: "مجھے موجودہ روبوٹ pose دو"
from geometry_msgs.srv import GetPose

client = node.create_client(GetPose, '/get_robot_pose')
request = GetPose.Request()
future = client.call_async(request)
```

### 4. Actions

**Actions** طویل چلنے والے کاموں کے لیے ہیں جن میں feedback ہو۔ Navigation کے لیے بہترین!

```python
# Action: "پوزیشن (x=5, y=3) پر navigate کرو"
from nav2_msgs.action import NavigateToPose

action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
goal = NavigateToPose.Goal()
goal.pose.pose.position.x = 5.0
goal.pose.pose.position.y = 3.0
action_client.send_goal_async(goal)
```

Actions فراہم کرتے ہیں:
- **Goal**: کیا حاصل کرنا ہے
- **Feedback**: پیش رفت کی تازہ کاریاں
- **Result**: حتمی نتیجہ

## Python (rclpy) کے ساتھ آپ کی پہلی ROS 2 Node

آئیں ایک سادہ node بنائیں جو "Hello, Robot!" پیغامات publish کرے۔

### انسٹالیشن (Ubuntu 22.04)

```bash
# ROS 2 Humble انسٹال کریں
sudo apt update
sudo apt install ros-humble-desktop

# سیٹ اپ source کریں
source /opt/ros/humble/setup.bash

# اپنے .bashrc میں شامل کریں
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Workspace بنانا

```bash
# workspace بنائیں
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# ایک package بنائیں
ros2 pkg create --build-type ament_python my_robot_pkg

# بلڈ کریں
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Hello Robot Node

`~/ros2_ws/src/my_robot_pkg/my_robot_pkg/hello_robot.py` بنائیں:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloRobotNode(Node):
    def __init__(self):
        super().__init__('hello_robot')
        
        # ایک publisher بنائیں
        self.publisher = self.create_publisher(
            String, 
            'robot_greeting', 
            10  # Queue size
        )
        
        # ایک timer بنائیں (1 Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0
        
        self.get_logger().info('Hello Robot node شروع ہو گئی!')
    
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello, Robot! Count: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = HelloRobotNode()
    
    try:
        rclpy.spin(node)  # Node کو چلتا رکھیں
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node چلانا

```bash
# Terminal 1: Node چلائیں
ros2 run my_robot_pkg hello_robot

# Terminal 2: Topic سنیں
ros2 topic echo /robot_greeting
```

آپ کو یہ نظر آنا چاہیے:
```
data: 'Hello, Robot! Count: 0'
---
data: 'Hello, Robot! Count: 1'
---
```

## URDF: اپنے روبوٹ کی تفصیل

**URDF (Unified Robot Description Format)** ایک XML فارمیٹ ہے جو آپ کے روبوٹ کی تفصیل بیان کرتا ہے:
- Links (rigid bodies)
- Joints (connections)
- Visual appearance
- Collision geometry
- Physical properties

### بنیادی URDF ڈھانچہ

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  
  <!-- Base Link (Torso) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" 
               iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  
  <!-- سر -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
  </link>
  
  <!-- گردن کا جوڑ -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
  
</robot>
```

### Joint کی اقسام

| قسم | DOF | تفصیل |
|-----|-----|-------|
| `fixed` | 0 | کوئی حرکت نہیں |
| `revolute` | 1 | حدود کے ساتھ گھومنا |
| `continuous` | 1 | لامحدود گھومنا |
| `prismatic` | 1 | لکیری سلائیڈنگ |
| `floating` | 6 | آزاد حرکت |
| `planar` | 3 | 2D plane حرکت |

### URDF دیکھنا

```bash
# visualization tools انسٹال کریں
sudo apt install ros-humble-urdf-tutorial

# اپنا روبوٹ دیکھیں
ros2 launch urdf_tutorial display.launch.py model:=/path/to/robot.urdf
```

## Python Agents کو ROS 2 سے جوڑنا

اصل طاقت تب آتی ہے جب آپ AI ماڈلز کو ROS 2 سے جوڑتے ہیں۔ یہاں perception ماڈل integrate کرنے کا ایک pattern ہے:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from cv_bridge import CvBridge
import torch

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        # اپنا AI ماڈل لوڈ کریں
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        self.bridge = CvBridge()
        
        # کیمرا تصاویر subscribe کریں
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Detections publish کریں
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )
        
        self.get_logger().info('Object Detector تیار ہے!')
    
    def image_callback(self, msg):
        # ROS Image کو OpenCV میں بدلیں
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Inference چلائیں
        results = self.model(cv_image)
        
        # ROS message میں بدلیں
        detections = Detection2DArray()
        detections.header = msg.header
        
        for *box, conf, cls in results.xyxy[0]:
            det = Detection2D()
            det.bbox.center.position.x = float((box[0] + box[2]) / 2)
            det.bbox.center.position.y = float((box[1] + box[3]) / 2)
            det.bbox.size_x = float(box[2] - box[0])
            det.bbox.size_y = float(box[3] - box[1])
            detections.detections.append(det)
        
        self.detection_pub.publish(detections)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Launch Files

Launch files ایک ساتھ متعدد nodes شروع کرتی ہیں۔ `launch/robot_launch.py` بنائیں:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='camera_node',
            name='camera',
            parameters=[{'resolution': '640x480'}]
        ),
        Node(
            package='my_robot_pkg',
            executable='detector_node',
            name='detector',
            remappings=[('/camera/image_raw', '/camera/rgb')]
        ),
        Node(
            package='my_robot_pkg',
            executable='planner_node',
            name='planner'
        ),
    ])
```

اس کے ساتھ چلائیں:
```bash
ros2 launch my_robot_pkg robot_launch.py
```

## عملی مشق: Velocity Commander

ایک node بنائیں جو keyboard input subscribe کرے اور velocity commands publish کرے:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('کنٹرول کے لیے WASD استعمال کریں، چھوڑنے کے لیے Q')
        
    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def run(self):
        twist = Twist()
        while True:
            key = self.get_key()
            
            if key == 'w':
                twist.linear.x = 0.5
            elif key == 's':
                twist.linear.x = -0.5
            elif key == 'a':
                twist.angular.z = 0.5
            elif key == 'd':
                twist.angular.z = -0.5
            elif key == 'q':
                break
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            
            self.publisher.publish(twist)
```

## اہم نکات

1. **ROS 2 middleware ہے** - یہ روبوٹ کے اجزاء جوڑتا ہے
2. **Nodes واحد مقصد والی ہیں** - ایک کام فی node
3. **Topics data stream کرتے ہیں** - مسلسل sensor data کے لیے
4. **Services request/reply کرتی ہیں** - ایک بار کی queries کے لیے
5. **Actions طویل کام سنبھالتے ہیں** - feedback کے ساتھ
6. **URDF روبوٹس بیان کرتا ہے** - Links, joints, physics
7. **rclpy Python جوڑتا ہے** - AI ماڈلز → روبوٹ کنٹرول

## آگے کیا ہے؟

**ماڈیول 2** میں، آپ سیکھیں گے کہ اپنے ROS 2 روبوٹ کو **Gazebo** اور **Unity** میں کیسے simulate کریں — حقیقی ہارڈویئر پر deploy کرنے سے پہلے ایک مکمل digital twin بنانا۔

---

**اگلا**: [ماڈیول 2: ڈیجیٹل ٹوئن (Gazebo اور Unity) →](./module2-simulation)
