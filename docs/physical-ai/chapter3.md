# Chapter 3: AI Algorithms for Robotics

Artificial Intelligence provides the "brain" that transforms sensor data into intelligent decisions and coordinated actions. This chapter explores the fundamental AI algorithms that enable robots to navigate, plan, and interact with the world.

## Overview

AI for robotics encompasses multiple layers of intelligence:

1. **Perception AI**: Understanding sensor data
2. **Planning AI**: Deciding what to do
3. **Control AI**: Executing actions precisely
4. **Learning AI**: Improving through experience

Modern Physical AI systems integrate all these layers, often using end-to-end learned models.

## Localization and Mapping

Before a robot can navigate, it must know where it is and what's around it.

### SLAM (Simultaneous Localization and Mapping)

SLAM solves the chicken-and-egg problem: how do you build a map without knowing your location, and how do you localize without a map?

**Types of SLAM:**

1. **Visual SLAM**: Uses cameras (ORB-SLAM, RTAB-Map)
2. **LiDAR SLAM**: Uses laser scanners (Cartographer, LOAM)
3. **RGB-D SLAM**: Uses depth cameras (KinectFusion, ElasticFusion)

**Example: Simple 2D Grid-Based SLAM**
```python
import numpy as np
from collections import defaultdict

class GridSLAM:
    def __init__(self, map_size=100, resolution=0.1):
        """
        Simple occupancy grid SLAM
        
        Args:
            map_size: Size of map in meters
            resolution: Grid cell size in meters
        """
        self.resolution = resolution
        self.grid_size = int(map_size / resolution)
        
        # Occupancy grid: 0 = unknown, 1 = free, 2 = occupied
        self.map = np.zeros((self.grid_size, self.grid_size))
        
        # Robot pose: [x, y, theta]
        self.pose = np.array([map_size/2, map_size/2, 0.0])
    
    def update_pose(self, dx, dy, dtheta):
        """Update robot pose based on odometry"""
        self.pose[0] += dx * np.cos(self.pose[2]) - dy * np.sin(self.pose[2])
        self.pose[1] += dx * np.sin(self.pose[2]) + dy * np.cos(self.pose[2])
        self.pose[2] += dtheta
    
    def update_map(self, lidar_ranges, lidar_angles):
        """
        Update occupancy grid with LiDAR scan
        
        Args:
            lidar_ranges: Array of range measurements (meters)
            lidar_angles: Array of scan angles (radians)
        """
        robot_x, robot_y, robot_theta = self.pose
        
        for range_val, angle in zip(lidar_ranges, lidar_angles):
            if range_val > 0 and range_val < 10:  # Valid reading
                # Calculate obstacle position in world frame
                abs_angle = robot_theta + angle
                obs_x = robot_x + range_val * np.cos(abs_angle)
                obs_y = robot_y + range_val * np.sin(abs_angle)
                
                # Convert to grid coordinates
                grid_x = int(obs_x / self.resolution)
                grid_y = int(obs_y / self.resolution)
                
                # Mark as occupied
                if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                    self.map[grid_y, grid_x] = 2
                
                # Ray tracing: mark cells along ray as free
                self.ray_trace(robot_x, robot_y, obs_x, obs_y)
    
    def ray_trace(self, x0, y0, x1, y1):
        """Bresenham's line algorithm for ray tracing"""
        gx0 = int(x0 / self.resolution)
        gy0 = int(y0 / self.resolution)
        gx1 = int(x1 / self.resolution)
        gy1 = int(y1 / self.resolution)
        
        dx = abs(gx1 - gx0)
        dy = abs(gy1 - gy0)
        sx = 1 if gx0 < gx1 else -1
        sy = 1 if gy0 < gy1 else -1
        err = dx - dy
        
        while True:
            # Mark as free (if not already occupied)
            if 0 <= gx0 < self.grid_size and 0 <= gy0 < self.grid_size:
                if self.map[gy0, gx0] != 2:
                    self.map[gy0, gx0] = 1
            
            if gx0 == gx1 and gy0 == gy1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                gx0 += sx
            if e2 < dx:
                err += dx
                gy0 += sy
```

### Particle Filters for Localization

Monte Carlo Localization (MCL) uses particles to represent pose uncertainty:

```python
import numpy as np

class ParticleFilter:
    def __init__(self, num_particles=1000):
        self.num_particles = num_particles
        # Each particle: [x, y, theta, weight]
        self.particles = np.random.rand(num_particles, 4)
        self.particles[:, 3] = 1.0 / num_particles  # Equal weights
    
    def predict(self, dx, dy, dtheta, noise_std):
        """
        Prediction step: move particles based on motion model
        
        Args:
            dx, dy, dtheta: Odometry measurements
            noise_std: Motion noise standard deviation
        """
        # Add motion with noise
        self.particles[:, 0] += dx + np.random.normal(0, noise_std, self.num_particles)
        self.particles[:, 1] += dy + np.random.normal(0, noise_std, self.num_particles)
        self.particles[:, 2] += dtheta + np.random.normal(0, noise_std/10, self.num_particles)
    
    def update(self, measurement, map_data):
        """
        Update step: reweight particles based on measurement likelihood
        
        Args:
            measurement: Sensor measurement
            map_data: Environment map
        """
        for i in range(self.num_particles):
            # Calculate likelihood of measurement given particle pose
            expected = self.expected_measurement(self.particles[i, :3], map_data)
            likelihood = self.measurement_likelihood(measurement, expected)
            self.particles[i, 3] *= likelihood
        
        # Normalize weights
        total_weight = np.sum(self.particles[:, 3])
        if total_weight > 0:
            self.particles[:, 3] /= total_weight
    
    def resample(self):
        """Resample particles based on weights (low variance resampling)"""
        new_particles = np.zeros_like(self.particles)
        
        # Cumulative sum of weights
        cumsum = np.cumsum(self.particles[:, 3])
        
        # Low variance resampling
        step = 1.0 / self.num_particles
        start = np.random.uniform(0, step)
        
        i = 0
        for j in range(self.num_particles):
            u = start + j * step
            while u > cumsum[i]:
                i += 1
            new_particles[j] = self.particles[i]
        
        self.particles = new_particles
        self.particles[:, 3] = 1.0 / self.num_particles
    
    def estimate_pose(self):
        """Estimate pose as weighted mean of particles"""
        return np.average(self.particles[:, :3], 
                         weights=self.particles[:, 3], axis=0)
    
    def measurement_likelihood(self, measurement, expected):
        """Gaussian likelihood model"""
        sigma = 0.5
        return np.exp(-0.5 * ((measurement - expected) / sigma) ** 2)
    
    def expected_measurement(self, pose, map_data):
        """Predict sensor measurement from pose (placeholder)"""
        # In reality, ray-cast into map from pose
        return 0.0
```

## Path Planning

Once the robot knows where it is and where it wants to go, it needs to plan a collision-free path.

### A* Algorithm

A* is a graph search algorithm that finds the optimal path using a heuristic:

```python
import heapq
import numpy as np

class AStarPlanner:
    def __init__(self, grid_map):
        """
        A* path planner on occupancy grid
        
        Args:
            grid_map: 2D numpy array (0=free, 1=occupied)
        """
        self.grid = grid_map
        self.height, self.width = grid_map.shape
    
    def heuristic(self, a, b):
        """Euclidean distance heuristic"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def get_neighbors(self, node):
        """Get valid 8-connected neighbors"""
        x, y = node
        neighbors = []
        
        # 8-connected grid
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                
                nx, ny = x + dx, y + dy
                
                # Check bounds and obstacles
                if (0 <= nx < self.width and 
                    0 <= ny < self.height and 
                    self.grid[ny, nx] == 0):
                    
                    # Diagonal movement cost
                    cost = np.sqrt(dx**2 + dy**2)
                    neighbors.append(((nx, ny), cost))
        
        return neighbors
    
    def plan(self, start, goal):
        """
        Find optimal path from start to goal
        
        Args:
            start: (x, y) start position
            goal: (x, y) goal position
        
        Returns:
            List of (x, y) positions forming path, or None if no path
        """
        # Priority queue: (f_score, counter, node)
        counter = 0
        open_set = [(0, counter, start)]
        came_from = {}
        
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        while open_set:
            _, _, current = heapq.heappop(open_set)
            
            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]
            
            for neighbor, move_cost in self.get_neighbors(current):
                tentative_g = g_score[current] + move_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    
                    counter += 1
                    heapq.heappush(open_set, (f_score[neighbor], counter, neighbor))
        
        return None  # No path found

# Example usage
grid = np.zeros((100, 100))
grid[30:70, 50] = 1  # Vertical wall

planner = AStarPlanner(grid)
path = planner.plan(start=(10, 50), goal=(90, 50))
print(f"Path found with {len(path)} waypoints" if path else "No path found")
```

### RRT (Rapidly-Exploring Random Tree)

RRT is excellent for high-dimensional planning (like robot arms):

```python
import numpy as np

class RRTPlanner:
    def __init__(self, start, goal, obstacles, bounds, step_size=0.5, max_iter=5000):
        """
        RRT path planner
        
        Args:
            start: Start configuration (numpy array)
            goal: Goal configuration (numpy array)
            obstacles: List of obstacle objects
            bounds: [(min, max), ...] for each dimension
            step_size: Maximum step size for tree expansion
            max_iter: Maximum iterations
        """
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacles = obstacles
        self.bounds = bounds
        self.step_size = step_size
        self.max_iter = max_iter
        
        # Tree structure: node_id -> parent_id
        self.tree = {0: None}
        self.nodes = {0: self.start}
    
    def sample_random_config(self):
        """Sample random configuration in space"""
        config = []
        for min_val, max_val in self.bounds:
            config.append(np.random.uniform(min_val, max_val))
        return np.array(config)
    
    def nearest_node(self, config):
        """Find nearest node in tree to given config"""
        min_dist = float('inf')
        nearest_id = 0
        
        for node_id, node_config in self.nodes.items():
            dist = np.linalg.norm(config - node_config)
            if dist < min_dist:
                min_dist = dist
                nearest_id = node_id
        
        return nearest_id, self.nodes[nearest_id]
    
    def steer(self, from_config, to_config):
        """Steer from one config toward another with step_size limit"""
        direction = to_config - from_config
        distance = np.linalg.norm(direction)
        
        if distance <= self.step_size:
            return to_config
        else:
            return from_config + (direction / distance) * self.step_size
    
    def is_collision_free(self, config):
        """Check if configuration collides with obstacles"""
        for obstacle in self.obstacles:
            if obstacle.contains(config):
                return False
        return True
    
    def plan(self):
        """Execute RRT planning"""
        for i in range(self.max_iter):
            # Goal biasing: 10% chance to sample goal
            if np.random.rand() < 0.1:
                rand_config = self.goal
            else:
                rand_config = self.sample_random_config()
            
            # Find nearest node and steer toward random config
            nearest_id, nearest_config = self.nearest_node(rand_config)
            new_config = self.steer(nearest_config, rand_config)
            
            # Check collision
            if self.is_collision_free(new_config):
                new_id = len(self.nodes)
                self.nodes[new_id] = new_config
                self.tree[new_id] = nearest_id
                
                # Check if goal reached
                if np.linalg.norm(new_config - self.goal) < self.step_size:
                    # Add goal and reconstruct path
                    goal_id = len(self.nodes)
                    self.nodes[goal_id] = self.goal
                    self.tree[goal_id] = new_id
                    
                    return self.reconstruct_path(goal_id)
        
        return None  # Failed to find path
    
    def reconstruct_path(self, goal_id):
        """Reconstruct path from start to goal"""
        path = [self.nodes[goal_id]]
        current_id = goal_id
        
        while self.tree[current_id] is not None:
            current_id = self.tree[current_id]
            path.append(self.nodes[current_id])
        
        return path[::-1]
```

## Motion Planning for Manipulation

Manipulation requires planning in the configuration space of the robot arm.

### Inverse Kinematics

Given a desired end-effector position, compute joint angles:

```python
import numpy as np

def inverse_kinematics_2dof(target_x, target_y, L1=1.0, L2=1.0):
    """
    Analytical IK for 2-DOF planar arm
    
    Args:
        target_x, target_y: Desired end-effector position
        L1, L2: Link lengths
    
    Returns:
        (theta1, theta2): Joint angles or None if unreachable
    """
    # Check if target is reachable
    distance = np.sqrt(target_x**2 + target_y**2)
    if distance > L1 + L2 or distance < abs(L1 - L2):
        return None
    
    # Use law of cosines
    cos_theta2 = (target_x**2 + target_y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    
    # Clamp to avoid numerical errors
    cos_theta2 = np.clip(cos_theta2, -1, 1)
    
    # Elbow-down solution
    theta2 = np.arccos(cos_theta2)
    
    # Calculate theta1
    k1 = L1 + L2 * np.cos(theta2)
    k2 = L2 * np.sin(theta2)
    theta1 = np.arctan2(target_y, target_x) - np.arctan2(k2, k1)
    
    return theta1, theta2

# Example
target = (1.5, 0.5)
solution = inverse_kinematics_2dof(*target)
if solution:
    print(f"Joint angles: θ1={np.degrees(solution[0]):.1f}°, θ2={np.degrees(solution[1]):.1f}°")
```

For more complex robots, numerical methods like Jacobian-based IK are used.

## Behavior Trees

Behavior trees provide a hierarchical structure for decision-making:

```python
from enum import Enum

class NodeStatus(Enum):
    SUCCESS = 1
    FAILURE = 2
    RUNNING = 3

class BehaviorNode:
    def tick(self):
        """Execute this node"""
        raise NotImplementedError

class SequenceNode(BehaviorNode):
    """Execute children in sequence; fail if any fails"""
    def __init__(self, children):
        self.children = children
        self.current_child = 0
    
    def tick(self):
        while self.current_child < len(self.children):
            status = self.children[self.current_child].tick()
            
            if status == NodeStatus.FAILURE:
                self.current_child = 0
                return NodeStatus.FAILURE
            elif status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
            else:  # SUCCESS
                self.current_child += 1
        
        self.current_child = 0
        return NodeStatus.SUCCESS

class SelectorNode(BehaviorNode):
    """Try children in order; succeed if any succeeds"""
    def __init__(self, children):
        self.children = children
        self.current_child = 0
    
    def tick(self):
        while self.current_child < len(self.children):
            status = self.children[self.current_child].tick()
            
            if status == NodeStatus.SUCCESS:
                self.current_child = 0
                return NodeStatus.SUCCESS
            elif status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
            else:  # FAILURE
                self.current_child += 1
        
        self.current_child = 0
        return NodeStatus.FAILURE

# Example: Pick and place behavior
class ApproachObject(BehaviorNode):
    def tick(self):
        # Move toward object
        if distance_to_object() < 0.05:
            return NodeStatus.SUCCESS
        return NodeStatus.RUNNING

class GraspObject(BehaviorNode):
    def tick(self):
        # Close gripper
        if gripper_has_object():
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE

# Build behavior tree
pick_and_place = SequenceNode([
    ApproachObject(),
    GraspObject(),
    # ... more nodes
])
```

## Machine Learning for Control

Modern robots increasingly use learned policies.

### Reinforcement Learning

RL trains policies through trial-and-error:

**Policy Gradient Example (REINFORCE):**
```python
import torch
import torch.nn as nn
import torch.optim as optim

class PolicyNetwork(nn.Module):
    def __init__(self, state_dim, action_dim):
        super().__init__()
        self.fc = nn.Sequential(
            nn.Linear(state_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, action_dim),
            nn.Tanh()  # Actions in [-1, 1]
        )
    
    def forward(self, state):
        return self.fc(state)

def train_policy(env, policy, num_episodes=1000):
    optimizer = optim.Adam(policy.parameters(), lr=0.001)
    
    for episode in range(num_episodes):
        states, actions, rewards = [], [], []
        state = env.reset()
        done = False
        
        # Collect trajectory
        while not done:
            state_tensor = torch.FloatTensor(state).unsqueeze(0)
            action = policy(state_tensor)
            
            states.append(state_tensor)
            actions.append(action)
            
            next_state, reward, done = env.step(action.detach().numpy()[0])
            rewards.append(reward)
            state = next_state
        
        # Compute returns
        returns = []
        G = 0
        for r in reversed(rewards):
            G = r + 0.99 * G  # Discount factor
            returns.insert(0, G)
        
        returns = torch.FloatTensor(returns)
        returns = (returns - returns.mean()) / (returns.std() + 1e-8)
        
        # Update policy
        policy_loss = []
        for state, action, G in zip(states, actions, returns):
            log_prob = -((action - policy(state))**2).sum()
            policy_loss.append(-log_prob * G)
        
        optimizer.zero_grad()
        loss = torch.stack(policy_loss).sum()
        loss.backward()
        optimizer.step()
```

## Summary

AI algorithms provide the intelligence layer for physical robots:

- **Localization and Mapping**: Knowing where you are and building representations
- **Path Planning**: Computing collision-free paths to goals
- **Motion Planning**: Coordinating multiple joints for manipulation
- **Behavior Trees**: Structuring complex decision-making
- **Reinforcement Learning**: Learning control policies from experience

These algorithms work together to create truly intelligent physical systems that can operate autonomously in the real world.

---

**Previous**: [Actuators and Movement Control ←](./chapter2.md)

## Exercises

1. Implement and visualize A* pathfinding on a 2D grid
2. Build a simple behavior tree for a home service robot
3. Solve inverse kinematics for a 3-DOF robot arm
4. Compare RRT and A* performance on the same planning problem
5. Train a simple RL agent to balance a pole (CartPole environment)

