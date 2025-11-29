---
sidebar_position: 3
---

# Chapter 2: Actuators and Movement Control

After perceiving the environment, robots must be able to act. This chapter explores the actuators that enable physical movement and the control systems that coordinate them.

## Overview

Actuators are the "muscles" of a robot, converting electrical, hydraulic, or pneumatic energy into mechanical motion. For humanoid robots, actuators must provide:

- **High torque-to-weight ratio**: Strong yet lightweight for dynamic movement
- **Precise control**: Accurate position and velocity tracking
- **Compliance**: Safe interaction with humans and environment
- **Efficiency**: Long operation time on limited battery power

## Types of Actuators

### Electric Motors

Electric motors are the most common actuators in humanoid robotics due to their precision and ease of control.

**DC Brushed Motors**
- Simple, inexpensive, and easy to control
- Brushes wear out over time
- Good for low-cost applications

**Brushless DC (BLDC) Motors**
- More efficient and longer-lasting than brushed motors
- Require electronic speed controllers (ESCs)
- Common in drones and modern robots

**Servo Motors**
- Integrated feedback control for position accuracy
- Available in various sizes and torque ratings
- Standard hobby servos: limited range (±90°)
- Industrial servos: high precision, multiple rotations

**Example: Servo Control**
```python
import time
from adafruit_servokit import ServoKit

# Initialize servo controller (PCA9685)
kit = ServoKit(channels=16)

def move_servo_smooth(servo_channel, target_angle, duration=1.0, steps=50):
    """
    Smoothly move servo from current to target angle
    
    Args:
        servo_channel: Servo channel number (0-15)
        target_angle: Target angle in degrees (0-180)
        duration: Time to complete movement (seconds)
        steps: Number of interpolation steps
    """
    current_angle = kit.servo[servo_channel].angle
    if current_angle is None:
        current_angle = 90  # Default starting position
    
    # Generate smooth trajectory
    angles = [current_angle + (target_angle - current_angle) * i / steps 
              for i in range(steps + 1)]
    
    delay = duration / steps
    
    for angle in angles:
        kit.servo[servo_channel].angle = angle
        time.sleep(delay)

# Example usage
move_servo_smooth(servo_channel=0, target_angle=120, duration=2.0)
```

### Hydraulic Actuators

Hydraulic systems use pressurized fluid to generate motion and force.

**Advantages:**
- Very high power-to-weight ratio
- Can generate enormous forces
- Naturally compliant and back-drivable

**Disadvantages:**
- Require pumps, reservoirs, and complex plumbing
- Potential for leaks
- Noise and maintenance requirements

**Applications:**
- Boston Dynamics' Atlas robot
- Large industrial manipulators
- Heavy-duty construction robots

### Pneumatic Actuators

Pneumatic systems use compressed air for actuation.

**Advantages:**
- Inherently safe (compressible medium)
- Compliant and soft interaction
- Simple and lightweight

**Disadvantages:**
- Lower force density than hydraulics
- Difficult to control precisely
- Requires air compressor

**Applications:**
- Soft robotics
- Grippers and end effectors
- Human-safe collaborative robots

### Series Elastic Actuators (SEAs)

SEAs include a compliant spring between the motor and load, providing:

- **Force control**: Measure spring deflection to estimate force
- **Shock absorption**: Spring protects gears from impacts
- **Energy storage**: Elastic element can store and release energy
- **Safe interaction**: Compliance prevents injury during contact

**Example Calculation:**
```python
def sea_force_estimation(motor_position, load_position, spring_stiffness):
    """
    Estimate force in a Series Elastic Actuator
    
    Args:
        motor_position: Motor shaft position (radians)
        load_position: Load/joint position (radians)
        spring_stiffness: Spring constant (Nm/rad)
    
    Returns:
        Estimated torque (Nm)
    """
    spring_deflection = motor_position - load_position
    torque = spring_stiffness * spring_deflection
    return torque

# Example
motor_pos = 1.5  # radians
joint_pos = 1.3  # radians
k_spring = 100  # Nm/rad

force = sea_force_estimation(motor_pos, joint_pos, k_spring)
print(f"Estimated torque: {force:.2f} Nm")
```

## Motor Control Techniques

### Position Control

Position control moves the actuator to a desired angle or position.

**PID Control** (Proportional-Integral-Derivative):

```python
class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(-1, 1)):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.output_limits = output_limits
        
        self.prev_error = 0
        self.integral = 0
    
    def update(self, setpoint, measured_value, dt):
        """
        Calculate control output
        
        Args:
            setpoint: Desired value
            measured_value: Current measured value
            dt: Time step (seconds)
        
        Returns:
            Control signal
        """
        # Calculate error
        error = setpoint - measured_value
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term (with anti-windup)
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative
        
        # Total output
        output = p_term + i_term + d_term
        
        # Clamp output
        output = max(min(output, self.output_limits[1]), 
                    self.output_limits[0])
        
        # Store for next iteration
        self.prev_error = error
        
        return output

# Example usage for joint control
pid = PIDController(kp=5.0, ki=0.1, kd=0.5)

target_angle = 90  # degrees
current_angle = 45  # degrees
dt = 0.01  # 100 Hz control loop

control_signal = pid.update(target_angle, current_angle, dt)
```

### Velocity Control

Velocity control maintains a desired angular or linear velocity.

**Applications:**
- Smooth trajectories
- Coordinated multi-joint movements
- Avoiding jerky motion

### Torque/Force Control

Torque control directly commands the force output of the actuator.

**Advantages:**
- Compliant interaction with environment
- Precise force application
- Energy-efficient movement

**Impedance Control:**
```python
import numpy as np

def impedance_controller(position, velocity, target_position, 
                        target_velocity, stiffness, damping):
    """
    Impedance control for compliant interaction
    
    Behaves like a virtual spring-damper system
    
    Args:
        position: Current position
        velocity: Current velocity
        target_position: Desired position
        target_velocity: Desired velocity
        stiffness: Virtual spring stiffness
        damping: Virtual damping coefficient
    
    Returns:
        Desired torque/force
    """
    position_error = target_position - position
    velocity_error = target_velocity - velocity
    
    # Virtual spring-damper system
    force = stiffness * position_error + damping * velocity_error
    
    return force

# Example: Compliant reaching
current_pos = np.array([0.5, 0.3, 0.8])
current_vel = np.array([0.0, 0.0, 0.0])
target_pos = np.array([0.7, 0.4, 0.9])
target_vel = np.array([0.0, 0.0, 0.0])

K = 50  # Stiffness
B = 10  # Damping

force_command = impedance_controller(current_pos, current_vel, 
                                    target_pos, target_vel, K, B)
print(f"Force command: {force_command}")
```

## Trajectory Generation

Smooth trajectories prevent jerky motion and reduce wear on mechanical components.

### Point-to-Point Trajectories

**Trapezoidal Velocity Profile:**
- Constant acceleration phase
- Constant velocity cruise phase
- Constant deceleration phase

**S-Curve (Jerk-Limited) Profile:**
- Limits rate of change of acceleration
- Smoother motion, less vibration

```python
import numpy as np
import matplotlib.pyplot as plt

def trapezoidal_trajectory(start, end, v_max, a_max, dt=0.01):
    """
    Generate trapezoidal velocity trajectory
    
    Args:
        start: Starting position
        end: Ending position
        v_max: Maximum velocity
        a_max: Maximum acceleration
        dt: Time step
    
    Returns:
        time, position, velocity, acceleration arrays
    """
    distance = abs(end - start)
    direction = np.sign(end - start)
    
    # Time to reach max velocity
    t_accel = v_max / a_max
    
    # Distance during acceleration and deceleration
    d_accel = 0.5 * a_max * t_accel**2
    
    # Check if we reach max velocity
    if 2 * d_accel > distance:
        # Triangular profile (never reach v_max)
        t_accel = np.sqrt(distance / a_max)
        v_max = a_max * t_accel
        t_cruise = 0
    else:
        # Trapezoidal profile
        t_cruise = (distance - 2 * d_accel) / v_max
    
    t_total = 2 * t_accel + t_cruise
    
    # Generate trajectory
    time = np.arange(0, t_total, dt)
    position = np.zeros_like(time)
    velocity = np.zeros_like(time)
    acceleration = np.zeros_like(time)
    
    for i, t in enumerate(time):
        if t < t_accel:
            # Acceleration phase
            position[i] = start + direction * 0.5 * a_max * t**2
            velocity[i] = direction * a_max * t
            acceleration[i] = direction * a_max
        elif t < t_accel + t_cruise:
            # Cruise phase
            t_cruise_elapsed = t - t_accel
            position[i] = start + direction * (d_accel + v_max * t_cruise_elapsed)
            velocity[i] = direction * v_max
            acceleration[i] = 0
        else:
            # Deceleration phase
            t_decel = t - t_accel - t_cruise
            position[i] = end - direction * 0.5 * a_max * (t_accel - t_decel)**2
            velocity[i] = direction * a_max * (t_accel - t_decel)
            acceleration[i] = -direction * a_max
    
    return time, position, velocity, acceleration

# Example usage
time, pos, vel, acc = trapezoidal_trajectory(
    start=0, end=100, v_max=50, a_max=100
)
```

### Multi-Joint Coordination

For humanoid robots, multiple joints must move in coordination.

**Joint Space Planning:**
- Plan each joint independently
- Simple but may cause inefficient Cartesian paths

**Cartesian Space Planning:**
- Plan end-effector path in 3D space
- Use inverse kinematics to compute joint angles
- More intuitive but computationally expensive

## Power Transmission

Getting power from motors to joints requires mechanical transmission systems.

### Gears

**Spur Gears:**
- Simple, efficient, parallel shafts
- Can be noisy

**Planetary Gears:**
- High reduction ratios in compact package
- Commonly used in robot joints

**Harmonic Drives (Strain Wave Gears):**
- Very high reduction ratios (50:1 to 200:1)
- Zero backlash
- Compact and lightweight
- Expensive but industry standard for precision robots

### Belts and Pulleys

- Lighter than gears
- Can transmit power over longer distances
- Used in lightweight robotic arms

### Direct Drive

- Motor directly coupled to joint (no gearbox)
- Zero backlash, highly backdrivable
- Requires high-torque motors
- Used in advanced research robots

## Heat Management

Motors generate heat during operation, which must be managed:

**Cooling Strategies:**
- Passive: Heat sinks, thermal conduction
- Active: Fans, liquid cooling systems
- Thermal design: Heat pipes, phase-change materials

**Thermal Modeling:**
```python
def motor_temperature_simple(ambient_temp, power_loss, thermal_resistance, 
                             thermal_capacitance, dt, current_temp):
    """
    Simple first-order thermal model for motor temperature
    
    Args:
        ambient_temp: Ambient temperature (°C)
        power_loss: Motor power loss/heat generation (W)
        thermal_resistance: Thermal resistance to ambient (°C/W)
        thermal_capacitance: Thermal capacitance (J/°C)
        dt: Time step (seconds)
        current_temp: Current motor temperature (°C)
    
    Returns:
        New motor temperature (°C)
    """
    # Heat flow to ambient
    heat_flow = (current_temp - ambient_temp) / thermal_resistance
    
    # Net heat change
    heat_net = power_loss - heat_flow
    
    # Temperature change
    temp_change = (heat_net / thermal_capacitance) * dt
    
    new_temp = current_temp + temp_change
    
    return new_temp
```

## Case Study: Boston Dynamics Atlas

Atlas is one of the most advanced humanoid robots, showcasing state-of-the-art actuation:

- **Hydraulic Actuation**: 28 hydraulically actuated joints
- **Custom Valves**: High-bandwidth servo valves for responsive control
- **Onboard Power**: Battery and hydraulic pump carried on robot
- **Force Control**: Whole-body torque control for dynamic balance
- **Performance**: Can run, jump, do backflips, and navigate rough terrain

## Summary

Actuators and control systems are the bridge between robot intelligence and physical action:

- Electric motors are most common, with various trade-offs in cost, precision, and power
- Control strategies range from simple position control to sophisticated torque control
- Smooth trajectory generation is essential for efficiency and longevity
- Power transmission systems amplify motor torque for high-force applications
- Thermal management is critical for continuous operation

In the next chapter, we'll explore how these actuators and sensors come together in AI algorithms for robotic intelligence.

---

**Previous**: [Robot Sensors and Perception ←](./chapter1.md) | **Next**: [AI Algorithms for Robotics →](./chapter3.md)

## Exercises

1. Implement a PID controller and tune gains for position control
2. Compare step response of P, PI, and PID controllers
3. Design a gear reduction system for a humanoid robot arm joint
4. Implement and visualize trapezoidal and S-curve trajectories
5. Calculate the thermal time constant of a motor from datasheet

