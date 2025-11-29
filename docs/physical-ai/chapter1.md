# Chapter 1: Robot Sensors and Perception

Understanding the environment is the first step for any intelligent robot. In this chapter, we explore the sensory systems that enable robots to perceive and interpret the physical world around them.

## Overview

Robot perception is the process of acquiring, processing, and interpreting sensory data to build a representation of the environment. For humanoid robots, perception systems must be:

- **Fast**: Real-time processing for dynamic environments
- **Robust**: Reliable under varying conditions (lighting, weather, occlusion)
- **Multimodal**: Combining multiple sensor types for comprehensive understanding
- **Efficient**: Operating within computational and power constraints

## Visual Perception Systems

Vision is arguably the most important sense for humanoid robots, providing rich information about the environment.

### Camera Technologies

**Monocular Cameras**
- Single camera providing 2D images
- Lightweight and inexpensive
- Depth information requires learning or motion

**Stereo Cameras**
- Two cameras mimicking human binocular vision
- Direct depth estimation through triangulation
- Limited range and requires calibration

**RGB-D Cameras**
- Combines color and depth information
- Active sensing (structured light or time-of-flight)
- Examples: Microsoft Kinect, Intel RealSense, Apple LiDAR

**Event Cameras**
- Neuromorphic sensors detecting pixel-level brightness changes
- Extremely low latency (less than 1ms)
- Excellent for high-speed motion and low-light conditions

### Computer Vision Algorithms

Modern robot vision leverages deep learning extensively:

**Object Detection**
```python
# Example: Using YOLO for real-time object detection
import cv2
from ultralytics import YOLO

# Load pre-trained model
model = YOLO('yolov8n.pt')

# Process camera frame
def detect_objects(frame):
    results = model(frame)
    
    for result in results:
        boxes = result.boxes
        for box in boxes:
            # Extract bounding box coordinates
            x1, y1, x2, y2 = box.xyxy[0]
            confidence = box.conf[0]
            class_id = box.cls[0]
            
            # Draw detection
            cv2.rectangle(frame, (int(x1), int(y1)), 
                         (int(x2), int(y2)), (0, 255, 0), 2)
    
    return frame
```

**Semantic Segmentation**
- Pixel-wise classification of scene elements
- Essential for navigation and manipulation
- Modern architectures: DeepLab, Mask R-CNN, Segment Anything Model (SAM)

**Pose Estimation**
- Detecting and tracking human body keypoints
- Critical for human-robot interaction
- Applications: gesture recognition, activity understanding, safe collaboration

## Depth Sensing and 3D Perception

Understanding 3D structure is crucial for navigation and manipulation.

### LiDAR (Light Detection and Ranging)

LiDAR sensors emit laser pulses and measure return time to calculate distance:

- **2D LiDAR**: Scanning in a single plane, common for navigation
- **3D LiDAR**: Full 360° point clouds, used in autonomous vehicles
- **Solid-State LiDAR**: No moving parts, more compact and reliable

**Point Cloud Processing**
```python
import open3d as o3d
import numpy as np

# Load point cloud
pcd = o3d.io.read_point_cloud("environment.pcd")

# Downsample for efficiency
pcd_downsampled = pcd.voxel_down_sample(voxel_size=0.05)

# Remove outliers
pcd_clean, _ = pcd_downsampled.remove_statistical_outlier(
    nb_neighbors=20, std_ratio=2.0
)

# Plane segmentation (ground detection)
plane_model, inliers = pcd_clean.segment_plane(
    distance_threshold=0.01,
    ransac_n=3,
    num_iterations=1000
)

# Extract ground plane and obstacles
ground = pcd_clean.select_by_index(inliers)
obstacles = pcd_clean.select_by_index(inliers, invert=True)
```

### Depth Estimation from Vision

**Structure from Motion (SfM)**
- Triangulating 3D points from multiple camera views
- Requires camera motion and feature tracking
- Used in visual SLAM systems

**Monocular Depth Estimation**
- Deep learning models predict depth from single images
- Recent models: MiDaS, DPT (Dense Prediction Transformer)
- Enables depth perception without specialized hardware

## Inertial Measurement Units (IMUs)

IMUs provide critical information about the robot's motion and orientation.

### Components

1. **Accelerometers**: Measure linear acceleration in 3 axes
2. **Gyroscopes**: Measure angular velocity in 3 axes
3. **Magnetometers**: Measure magnetic field for absolute heading (optional)

### Sensor Fusion

Combining IMU data with other sensors for robust state estimation:

**Complementary Filter**
```python
import numpy as np

class ComplementaryFilter:
    def __init__(self, alpha=0.98):
        self.alpha = alpha  # Weight for gyro vs accelerometer
        self.angle = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw
    
    def update(self, accel, gyro, dt):
        # Integrate gyroscope for short-term accuracy
        gyro_angle = self.angle + gyro * dt
        
        # Calculate angle from accelerometer (long-term accuracy)
        accel_angle = np.array([
            np.arctan2(accel[1], accel[2]),  # roll
            np.arctan2(-accel[0], np.sqrt(accel[1]**2 + accel[2]**2)),  # pitch
            self.angle[2]  # yaw unchanged (no magnetometer)
        ])
        
        # Complementary filter fusion
        self.angle = self.alpha * gyro_angle + (1 - self.alpha) * accel_angle
        
        return self.angle
```

## Tactile and Force Sensing

Physical interaction requires sensing contact forces and textures.

### Force/Torque Sensors

- **6-axis F/T sensors**: Measure forces (Fx, Fy, Fz) and torques (Tx, Ty, Tz)
- **Joint torque sensors**: Monitor motor currents to estimate joint torques
- **Applications**: Compliant control, safe interaction, object weight estimation

### Tactile Sensors

- **Resistive sensors**: Change resistance under pressure
- **Capacitive sensors**: Detect proximity and touch
- **Optical tactile sensors**: Use cameras to observe surface deformation (GelSight)

**Example: Force-Controlled Grasping**
```python
def adaptive_grasp(force_sensor, target_force=5.0, max_iterations=100):
    """
    Adjust gripper force to maintain target contact force
    """
    kp = 0.1  # Proportional gain
    gripper_position = 0.0
    
    for i in range(max_iterations):
        # Read current force
        current_force = force_sensor.read()
        
        # Calculate error
        error = target_force - current_force
        
        # Proportional control
        gripper_position += kp * error
        gripper_position = np.clip(gripper_position, 0, 1)
        
        # Apply gripper command
        gripper.set_position(gripper_position)
        
        # Check convergence
        if abs(error) < 0.1:
            break
    
    return gripper_position
```

## Proprioception: Body Awareness

Proprioception refers to the robot's sense of its own body configuration.

### Joint Encoders

- **Absolute encoders**: Maintain position information even when powered off
- **Incremental encoders**: Count pulses from a reference position
- **Resolution**: Typically 12-bit to 17-bit (4096 to 131,072 counts per revolution)

### State Estimation

**Forward Kinematics**: Computing end-effector position from joint angles

```python
import numpy as np

def forward_kinematics_2dof(theta1, theta2, L1=1.0, L2=1.0):
    """
    Forward kinematics for a 2-DOF planar arm
    
    Args:
        theta1: Joint 1 angle (radians)
        theta2: Joint 2 angle (radians)
        L1: Link 1 length
        L2: Link 2 length
    
    Returns:
        (x, y): End-effector position
    """
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    
    return x, y
```

## Sensor Fusion and Multi-Modal Perception

Real-world robots combine multiple sensor modalities for robust perception.

### Extended Kalman Filter (EKF)

The EKF is widely used for fusing noisy sensor measurements:

1. **Prediction**: Use motion model to predict next state
2. **Update**: Correct prediction using sensor measurements
3. **Fusion**: Optimally combine multiple sensor sources

### Modern Approaches: Deep Learning Fusion

- **Multi-modal networks**: Combine RGB, depth, and IMU in neural networks
- **Cross-attention mechanisms**: Learn to weight different modalities
- **End-to-end learning**: Direct mapping from sensors to actions

## Perception Challenges

Real-world perception faces numerous challenges:

1. **Lighting Variations**: Shadows, glare, day/night transitions
2. **Occlusions**: Objects blocking view of important features
3. **Dynamic Environments**: Moving people, vehicles, and objects
4. **Sensor Noise**: All sensors have measurement uncertainty
5. **Computational Constraints**: Limited onboard processing power
6. **Latency**: Perception must be fast enough for real-time control

## Case Study: Tesla Optimus Perception System

Tesla's Optimus humanoid robot leverages technology from their autonomous vehicles:

- **Vision-Based**: 8 cameras providing 360° coverage
- **Neural Network Processing**: Custom FSD chip running vision transformers
- **No LiDAR**: Relies entirely on vision (cost and simplicity)
- **Occupancy Networks**: 3D voxel representation of environment
- **Language Grounding**: Connecting vision to natural language commands

## Summary

Robot perception is the foundation for intelligent behavior in physical AI systems. Key takeaways:

- Vision is the primary sense, enhanced by depth sensors and IMUs
- Multiple sensors provide redundancy and complementary information
- Deep learning has revolutionized perception capabilities
- Sensor fusion is essential for robust state estimation
- Real-time processing and efficiency are critical constraints

In the next chapter, we'll explore how robots act on their perception through actuators and movement control.

---

**Previous**: [Introduction ←](./intro.md) | **Next**: [Actuators and Movement Control →](./chapter2.md)

## Exercises

1. Implement a simple object tracking system using OpenCV
2. Compare monocular vs stereo depth estimation accuracy
3. Design a sensor suite for a humanoid robot operating indoors
4. Implement a complementary filter for IMU orientation estimation
5. Analyze the trade-offs between different camera types for robot vision

