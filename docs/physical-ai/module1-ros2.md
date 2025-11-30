---
sidebar_position: 2
---

# Module 1: The Robotic Nervous System (ROS 2)

> **Focus: Middleware for robot control**

ROS 2 (Robot Operating System 2) is the nervous system of modern robotics. It provides the communication infrastructure that allows different parts of a robot—sensors, actuators, AI modules—to talk to each other. In this module, you'll master the fundamentals of ROS 2 and learn to bridge Python AI agents to robot controllers.

## What is ROS 2?

ROS 2 is not an operating system—it's **middleware**. Think of it as the communication protocol that lets:

- A camera publish images
- A perception model detect objects
- A planner decide where to move
- Motors receive movement commands

All these components run as separate **nodes** that communicate through **topics**, **services**, and **actions**.

### Why ROS 2 over ROS 1?

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Real-time | ❌ No | ✅ Yes (DDS) |
| Multi-robot | ❌ Limited | ✅ Native support |
| Security | ❌ None | ✅ Built-in |
| Platform | Linux only | Linux, Windows, macOS |
| Python | Python 2 | Python 3 |

ROS 2 uses **DDS (Data Distribution Service)** for communication, which provides:
- Quality of Service (QoS) policies
- Real-time guarantees
- Distributed discovery (no rosmaster!)

## Core Concepts

### 1. Nodes

A **node** is a single process that performs a specific task. A robot typically runs many nodes:

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Camera    │    │  Detector   │    │   Planner   │
│    Node     │ -> │    Node     │ -> │    Node     │
└─────────────┘    └─────────────┘    └─────────────┘
```

Each node is:
- **Single-purpose**: Does one thing well
- **Reusable**: Can be shared across robots
- **Replaceable**: Swap implementations without changing others

### 2. Topics

**Topics** are named buses for streaming data. Nodes **publish** to topics and **subscribe** from topics.

```python
# Publisher: Camera node publishes images
publisher = node.create_publisher(Image, '/camera/image_raw', 10)

# Subscriber: Detector node receives images
subscription = node.create_subscription(
    Image, '/camera/image_raw', self.image_callback, 10
)
```

Common topic patterns:
- `/camera/image_raw` - Raw camera images
- `/scan` - LiDAR scan data
- `/cmd_vel` - Velocity commands
- `/odom` - Odometry (position/velocity)

### 3. Services

**Services** are synchronous request/response calls. Use them when you need a reply.

```python
# Service: "Give me the current robot pose"
from geometry_msgs.srv import GetPose

client = node.create_client(GetPose, '/get_robot_pose')
request = GetPose.Request()
future = client.call_async(request)
```

### 4. Actions

**Actions** are for long-running tasks with feedback. Perfect for navigation!

```python
# Action: "Navigate to position (x=5, y=3)"
from nav2_msgs.action import NavigateToPose

action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
goal = NavigateToPose.Goal()
goal.pose.pose.position.x = 5.0
goal.pose.pose.position.y = 3.0
action_client.send_goal_async(goal)
```

Actions provide:
- **Goal**: What to achieve
- **Feedback**: Progress updates
- **Result**: Final outcome

## Your First ROS 2 Node with Python (rclpy)

Let's build a simple node that publishes "Hello, Robot!" messages.

### Installation (Ubuntu 22.04)

```bash
# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Source the setup
source /opt/ros/humble/setup.bash

# Add to your .bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Creating a Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create a package
ros2 pkg create --build-type ament_python my_robot_pkg

# Build
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Hello Robot Node

Create `~/ros2_ws/src/my_robot_pkg/my_robot_pkg/hello_robot.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloRobotNode(Node):
    def __init__(self):
        super().__init__('hello_robot')
        
        # Create a publisher
        self.publisher = self.create_publisher(
            String, 
            'robot_greeting', 
            10  # Queue size
        )
        
        # Create a timer (1 Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0
        
        self.get_logger().info('Hello Robot node started!')
    
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
        rclpy.spin(node)  # Keep node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running the Node

```bash
# Terminal 1: Run the node
ros2 run my_robot_pkg hello_robot

# Terminal 2: Listen to the topic
ros2 topic echo /robot_greeting
```

You should see:
```
data: 'Hello, Robot! Count: 0'
---
data: 'Hello, Robot! Count: 1'
---
```

## URDF: Describing Your Robot

**URDF (Unified Robot Description Format)** is an XML format that describes your robot's:
- Links (rigid bodies)
- Joints (connections)
- Visual appearance
- Collision geometry
- Physical properties

### Basic URDF Structure

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
  
  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
  </link>
  
  <!-- Neck Joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
  
</robot>
```

### Joint Types

| Type | DOF | Description |
|------|-----|-------------|
| `fixed` | 0 | No movement |
| `revolute` | 1 | Rotation with limits |
| `continuous` | 1 | Unlimited rotation |
| `prismatic` | 1 | Linear sliding |
| `floating` | 6 | Free movement |
| `planar` | 3 | 2D plane movement |

### Visualizing URDF

```bash
# Install visualization tools
sudo apt install ros-humble-urdf-tutorial

# View your robot
ros2 launch urdf_tutorial display.launch.py model:=/path/to/robot.urdf
```

## Bridging Python Agents to ROS 2

The real power comes when you connect AI models to ROS 2. Here's a pattern for integrating a perception model:

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
        
        # Load your AI model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        self.bridge = CvBridge()
        
        # Subscribe to camera images
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publish detections
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )
        
        self.get_logger().info('Object Detector ready!')
    
    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Run inference
        results = self.model(cv_image)
        
        # Convert to ROS message
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

Launch files start multiple nodes at once. Create `launch/robot_launch.py`:

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

Run with:
```bash
ros2 launch my_robot_pkg robot_launch.py
```

## Practical Exercise: Velocity Commander

Build a node that subscribes to keyboard input and publishes velocity commands:

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
        self.get_logger().info('Use WASD to control, Q to quit')
        
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

## Key Takeaways

1. **ROS 2 is middleware** - It connects robot components
2. **Nodes are single-purpose** - One task per node
3. **Topics stream data** - For continuous sensor data
4. **Services request/reply** - For one-time queries
5. **Actions handle long tasks** - With feedback
6. **URDF describes robots** - Links, joints, physics
7. **rclpy bridges Python** - AI models → Robot control

## What's Next?

In **Module 2**, you'll learn to simulate your ROS 2 robot in **Gazebo** and **Unity**—creating a complete digital twin before deploying to real hardware.

---

**Next**: [Module 2: The Digital Twin (Gazebo & Unity) →](./module2-simulation)
