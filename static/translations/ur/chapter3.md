---
sidebar_position: 4
---

# باب 3: روبوٹکس کے لیے AI الگورتھمز

مصنوعی ذہانت "دماغ" فراہم کرتی ہے جو سینسر ڈیٹا کو ذہین فیصلوں اور ہم آہنگ اعمال میں بدل دیتی ہے۔ یہ باب بنیادی AI الگورتھمز کی تلاش کرتا ہے جو روبوٹس کو نیویگیٹ، پلان، اور دنیا کے ساتھ تعامل کرنے کے قابل بناتے ہیں۔

## جائزہ

روبوٹکس کے لیے AI ذہانت کی متعدد تہوں پر مشتمل ہے:

1. **پرسیپشن AI**: سینسر ڈیٹا کو سمجھنا
2. **پلاننگ AI**: کیا کرنا ہے یہ فیصلہ کرنا
3. **کنٹرول AI**: اعمال کو درست طریقے سے انجام دینا
4. **لرننگ AI**: تجربے سے بہتر ہونا

جدید فزیکل AI سسٹمز ان تمام تہوں کو یکجا کرتے ہیں، اکثر اینڈ ٹو اینڈ سیکھے ہوئے ماڈلز استعمال کرتے ہوئے۔

## لوکلائزیشن اور میپنگ

روبوٹ کے نیویگیٹ کرنے سے پہلے، اسے جاننا ضروری ہے کہ وہ کہاں ہے اور اس کے ارد گرد کیا ہے۔

### SLAM (Simultaneous Localization and Mapping)

SLAM مرغی اور انڈے کا مسئلہ حل کرتا ہے: آپ اپنی لوکیشن جانے بغیر نقشہ کیسے بناتے ہیں، اور نقشے کے بغیر لوکلائز کیسے کرتے ہیں؟

**SLAM کی اقسام:**

1. **ویژول SLAM**: کیمرے استعمال کرتا ہے (ORB-SLAM, RTAB-Map)
2. **LiDAR SLAM**: لیزر سکینرز استعمال کرتا ہے (Cartographer, LOAM)
3. **RGB-D SLAM**: ڈیپتھ کیمرے استعمال کرتا ہے (KinectFusion, ElasticFusion)

**مثال: سادہ 2D گرڈ بیسڈ SLAM**
```python
import numpy as np
from collections import defaultdict

class GridSLAM:
    def __init__(self, map_size=100, resolution=0.1):
        """
        سادہ آکیوپینسی گرڈ SLAM
        
        Args:
            map_size: میٹرز میں نقشے کا سائز
            resolution: گرڈ سیل سائز میٹرز میں
        """
        self.resolution = resolution
        self.grid_size = int(map_size / resolution)
        
        # آکیوپینسی گرڈ: 0 = نامعلوم، 1 = خالی، 2 = قبضہ شدہ
        self.map = np.zeros((self.grid_size, self.grid_size))
        
        # روبوٹ پوز: [x, y, theta]
        self.pose = np.array([map_size/2, map_size/2, 0.0])
    
    def update_pose(self, dx, dy, dtheta):
        """اوڈومیٹری کی بنیاد پر روبوٹ پوز اپڈیٹ کریں"""
        self.pose[0] += dx * np.cos(self.pose[2]) - dy * np.sin(self.pose[2])
        self.pose[1] += dx * np.sin(self.pose[2]) + dy * np.cos(self.pose[2])
        self.pose[2] += dtheta
    
    def update_map(self, lidar_ranges, lidar_angles):
        """
        LiDAR سکین کے ساتھ آکیوپینسی گرڈ اپڈیٹ کریں
        
        Args:
            lidar_ranges: رینج پیمائشوں کی اری (میٹرز)
            lidar_angles: سکین اینگلز کی اری (ریڈینز)
        """
        robot_x, robot_y, robot_theta = self.pose
        
        for range_val, angle in zip(lidar_ranges, lidar_angles):
            if range_val > 0 and range_val < 10:  # درست ریڈنگ
                # ورلڈ فریم میں رکاوٹ کی پوزیشن کا حساب
                abs_angle = robot_theta + angle
                obs_x = robot_x + range_val * np.cos(abs_angle)
                obs_y = robot_y + range_val * np.sin(abs_angle)
                
                # گرڈ کوآرڈینیٹس میں تبدیل کریں
                grid_x = int(obs_x / self.resolution)
                grid_y = int(obs_y / self.resolution)
                
                # قبضہ شدہ کے طور پر نشان زد کریں
                if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                    self.map[grid_y, grid_x] = 2
                
                # رے ٹریسنگ: رے کے ساتھ سیلز کو خالی کے طور پر نشان زد کریں
                self.ray_trace(robot_x, robot_y, obs_x, obs_y)
    
    def ray_trace(self, x0, y0, x1, y1):
        """رے ٹریسنگ کے لیے بریسنہیم کا لائن الگورتھم"""
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
            # خالی کے طور پر نشان زد کریں (اگر پہلے سے قبضہ نہیں)
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

### لوکلائزیشن کے لیے پارٹیکل فلٹرز

Monte Carlo Localization (MCL) پوز کی غیر یقینیت کی نمائندگی کے لیے پارٹیکلز استعمال کرتا ہے:

```python
import numpy as np

class ParticleFilter:
    def __init__(self, num_particles=1000):
        self.num_particles = num_particles
        # ہر پارٹیکل: [x, y, theta, weight]
        self.particles = np.random.rand(num_particles, 4)
        self.particles[:, 3] = 1.0 / num_particles  # برابر وزن
    
    def predict(self, dx, dy, dtheta, noise_std):
        """
        پیشگوئی کا مرحلہ: موشن ماڈل کی بنیاد پر پارٹیکلز منتقل کریں
        
        Args:
            dx, dy, dtheta: اوڈومیٹری پیمائشیں
            noise_std: موشن شور کا معیاری انحراف
        """
        # شور کے ساتھ موشن شامل کریں
        self.particles[:, 0] += dx + np.random.normal(0, noise_std, self.num_particles)
        self.particles[:, 1] += dy + np.random.normal(0, noise_std, self.num_particles)
        self.particles[:, 2] += dtheta + np.random.normal(0, noise_std/10, self.num_particles)
    
    def update(self, measurement, map_data):
        """
        اپڈیٹ کا مرحلہ: پیمائش کے امکان کی بنیاد پر پارٹیکلز کا دوبارہ وزن کریں
        
        Args:
            measurement: سینسر پیمائش
            map_data: ماحول کا نقشہ
        """
        for i in range(self.num_particles):
            # پارٹیکل پوز دیے گئے پیمائش کے امکان کا حساب
            expected = self.expected_measurement(self.particles[i, :3], map_data)
            likelihood = self.measurement_likelihood(measurement, expected)
            self.particles[i, 3] *= likelihood
        
        # وزن نارملائز کریں
        total_weight = np.sum(self.particles[:, 3])
        if total_weight > 0:
            self.particles[:, 3] /= total_weight
    
    def resample(self):
        """وزن کی بنیاد پر پارٹیکلز ری سیمپل کریں (لو وریئنس ری سیمپلنگ)"""
        new_particles = np.zeros_like(self.particles)
        
        # وزن کا مجموعی جمع
        cumsum = np.cumsum(self.particles[:, 3])
        
        # لو وریئنس ری سیمپلنگ
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
        """پارٹیکلز کے وزنی اوسط کے طور پر پوز کا تخمینہ"""
        return np.average(self.particles[:, :3], 
                         weights=self.particles[:, 3], axis=0)
    
    def measurement_likelihood(self, measurement, expected):
        """گاؤسین امکان ماڈل"""
        sigma = 0.5
        return np.exp(-0.5 * ((measurement - expected) / sigma) ** 2)
    
    def expected_measurement(self, pose, map_data):
        """پوز سے سینسر پیمائش کی پیشگوئی (پلیس ہولڈر)"""
        # حقیقت میں، پوز سے نقشے میں رے کاسٹ کریں
        return 0.0
```

## پاتھ پلاننگ

جب روبوٹ جان لے کہ وہ کہاں ہے اور کہاں جانا چاہتا ہے، تو اسے تصادم سے پاک راستے کی پلاننگ کرنی ہوگی۔

### A* الگورتھم

A* ایک گراف سرچ الگورتھم ہے جو ہیورسٹک استعمال کرتے ہوئے بہترین راستہ تلاش کرتا ہے:

```python
import heapq
import numpy as np

class AStarPlanner:
    def __init__(self, grid_map):
        """
        آکیوپینسی گرڈ پر A* پاتھ پلانر
        
        Args:
            grid_map: 2D numpy array (0=خالی، 1=قبضہ شدہ)
        """
        self.grid = grid_map
        self.height, self.width = grid_map.shape
    
    def heuristic(self, a, b):
        """یوکلیڈین فاصلے کا ہیورسٹک"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def get_neighbors(self, node):
        """درست 8-کنیکٹڈ ہمسائے حاصل کریں"""
        x, y = node
        neighbors = []
        
        # 8-کنیکٹڈ گرڈ
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                
                nx, ny = x + dx, y + dy
                
                # حدود اور رکاوٹیں چیک کریں
                if (0 <= nx < self.width and 
                    0 <= ny < self.height and 
                    self.grid[ny, nx] == 0):
                    
                    # ترچھی حرکت کی لاگت
                    cost = np.sqrt(dx**2 + dy**2)
                    neighbors.append(((nx, ny), cost))
        
        return neighbors
    
    def plan(self, start, goal):
        """
        شروع سے ہدف تک بہترین راستہ تلاش کریں
        
        Args:
            start: (x, y) شروعاتی پوزیشن
            goal: (x, y) ہدف کی پوزیشن
        
        Returns:
            راستہ بنانے والی (x, y) پوزیشنوں کی فہرست، یا None اگر کوئی راستہ نہیں
        """
        # ترجیحی قطار: (f_score, counter, node)
        counter = 0
        open_set = [(0, counter, start)]
        came_from = {}
        
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        while open_set:
            _, _, current = heapq.heappop(open_set)
            
            if current == goal:
                # راستہ دوبارہ بنائیں
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
        
        return None  # کوئی راستہ نہیں ملا

# استعمال کی مثال
grid = np.zeros((100, 100))
grid[30:70, 50] = 1  # عمودی دیوار

planner = AStarPlanner(grid)
path = planner.plan(start=(10, 50), goal=(90, 50))
print(f"{len(path)} ویپوائنٹس کے ساتھ راستہ ملا" if path else "کوئی راستہ نہیں ملا")
```

### RRT (Rapidly-Exploring Random Tree)

RRT اعلیٰ جہتی پلاننگ (جیسے روبوٹ آرمز) کے لیے بہترین ہے:

```python
import numpy as np

class RRTPlanner:
    def __init__(self, start, goal, obstacles, bounds, step_size=0.5, max_iter=5000):
        """
        RRT پاتھ پلانر
        
        Args:
            start: شروعاتی کنفیگریشن (numpy array)
            goal: ہدف کی کنفیگریشن (numpy array)
            obstacles: رکاوٹ آبجیکٹس کی فہرست
            bounds: ہر ڈائمینشن کے لیے [(min, max), ...]
            step_size: ٹری ایکسپینشن کے لیے زیادہ سے زیادہ اسٹیپ سائز
            max_iter: زیادہ سے زیادہ تکرار
        """
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacles = obstacles
        self.bounds = bounds
        self.step_size = step_size
        self.max_iter = max_iter
        
        # ٹری ڈھانچہ: node_id -> parent_id
        self.tree = {0: None}
        self.nodes = {0: self.start}
    
    def sample_random_config(self):
        """اسپیس میں رینڈم کنفیگریشن سیمپل کریں"""
        config = []
        for min_val, max_val in self.bounds:
            config.append(np.random.uniform(min_val, max_val))
        return np.array(config)
    
    def nearest_node(self, config):
        """دی گئی کنفیگ سے ٹری میں قریب ترین نوڈ تلاش کریں"""
        min_dist = float('inf')
        nearest_id = 0
        
        for node_id, node_config in self.nodes.items():
            dist = np.linalg.norm(config - node_config)
            if dist < min_dist:
                min_dist = dist
                nearest_id = node_id
        
        return nearest_id, self.nodes[nearest_id]
    
    def steer(self, from_config, to_config):
        """ایک کنفیگ سے دوسری کی طرف step_size حد کے ساتھ اسٹیئر کریں"""
        direction = to_config - from_config
        distance = np.linalg.norm(direction)
        
        if distance <= self.step_size:
            return to_config
        else:
            return from_config + (direction / distance) * self.step_size
    
    def is_collision_free(self, config):
        """چیک کریں کہ کنفیگریشن رکاوٹوں سے ٹکراتی ہے یا نہیں"""
        for obstacle in self.obstacles:
            if obstacle.contains(config):
                return False
        return True
    
    def plan(self):
        """RRT پلاننگ انجام دیں"""
        for i in range(self.max_iter):
            # گول بائیسنگ: ہدف سیمپل کرنے کا 10% امکان
            if np.random.rand() < 0.1:
                rand_config = self.goal
            else:
                rand_config = self.sample_random_config()
            
            # قریب ترین نوڈ تلاش کریں اور رینڈم کنفیگ کی طرف اسٹیئر کریں
            nearest_id, nearest_config = self.nearest_node(rand_config)
            new_config = self.steer(nearest_config, rand_config)
            
            # تصادم چیک کریں
            if self.is_collision_free(new_config):
                new_id = len(self.nodes)
                self.nodes[new_id] = new_config
                self.tree[new_id] = nearest_id
                
                # چیک کریں کہ ہدف تک پہنچ گئے
                if np.linalg.norm(new_config - self.goal) < self.step_size:
                    # ہدف شامل کریں اور راستہ دوبارہ بنائیں
                    goal_id = len(self.nodes)
                    self.nodes[goal_id] = self.goal
                    self.tree[goal_id] = new_id
                    
                    return self.reconstruct_path(goal_id)
        
        return None  # راستہ تلاش کرنے میں ناکام
    
    def reconstruct_path(self, goal_id):
        """شروع سے ہدف تک راستہ دوبارہ بنائیں"""
        path = [self.nodes[goal_id]]
        current_id = goal_id
        
        while self.tree[current_id] is not None:
            current_id = self.tree[current_id]
            path.append(self.nodes[current_id])
        
        return path[::-1]
```

## ہیرا پھیری کے لیے موشن پلاننگ

ہیرا پھیری کے لیے روبوٹ آرم کی کنفیگریشن اسپیس میں پلاننگ کی ضرورت ہے۔

### انورس کائنیمیٹکس

مطلوبہ اینڈ ایفیکٹر پوزیشن دی گئی ہو تو، جوائنٹ اینگلز کمپیوٹ کریں:

```python
import numpy as np

def inverse_kinematics_2dof(target_x, target_y, L1=1.0, L2=1.0):
    """
    2-DOF پلینر آرم کے لیے تجزیاتی IK
    
    Args:
        target_x, target_y: مطلوبہ اینڈ ایفیکٹر پوزیشن
        L1, L2: لنک کی لمبائیاں
    
    Returns:
        (theta1, theta2): جوائنٹ اینگلز یا None اگر ناقابل رسائی
    """
    # چیک کریں کہ ہدف قابل رسائی ہے
    distance = np.sqrt(target_x**2 + target_y**2)
    if distance > L1 + L2 or distance < abs(L1 - L2):
        return None
    
    # کوسائن کا قانون استعمال کریں
    cos_theta2 = (target_x**2 + target_y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    
    # عددی غلطیوں سے بچنے کے لیے کلیمپ کریں
    cos_theta2 = np.clip(cos_theta2, -1, 1)
    
    # کہنی نیچے حل
    theta2 = np.arccos(cos_theta2)
    
    # theta1 کا حساب لگائیں
    k1 = L1 + L2 * np.cos(theta2)
    k2 = L2 * np.sin(theta2)
    theta1 = np.arctan2(target_y, target_x) - np.arctan2(k2, k1)
    
    return theta1, theta2

# مثال
target = (1.5, 0.5)
solution = inverse_kinematics_2dof(*target)
if solution:
    print(f"جوائنٹ اینگلز: θ1={np.degrees(solution[0]):.1f}°, θ2={np.degrees(solution[1]):.1f}°")
```

زیادہ پیچیدہ روبوٹس کے لیے، جیکوبین بیسڈ IK جیسے عددی طریقے استعمال ہوتے ہیں۔

## بیہیویئر ٹریز

بیہیویئر ٹریز فیصلہ سازی کے لیے درجہ بندی کا ڈھانچہ فراہم کرتے ہیں:

```python
from enum import Enum

class NodeStatus(Enum):
    SUCCESS = 1
    FAILURE = 2
    RUNNING = 3

class BehaviorNode:
    def tick(self):
        """اس نوڈ کو انجام دیں"""
        raise NotImplementedError

class SequenceNode(BehaviorNode):
    """بچوں کو ترتیب سے انجام دیں؛ کوئی بھی ناکام ہو تو ناکام"""
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
            else:  # کامیابی
                self.current_child += 1
        
        self.current_child = 0
        return NodeStatus.SUCCESS

class SelectorNode(BehaviorNode):
    """بچوں کو ترتیب سے آزمائیں؛ کوئی بھی کامیاب ہو تو کامیاب"""
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
            else:  # ناکامی
                self.current_child += 1
        
        self.current_child = 0
        return NodeStatus.FAILURE

# مثال: پک اینڈ پلیس بیہیویئر
class ApproachObject(BehaviorNode):
    def tick(self):
        # آبجیکٹ کی طرف جائیں
        if distance_to_object() < 0.05:
            return NodeStatus.SUCCESS
        return NodeStatus.RUNNING

class GraspObject(BehaviorNode):
    def tick(self):
        # گرپر بند کریں
        if gripper_has_object():
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE

# بیہیویئر ٹری بنائیں
pick_and_place = SequenceNode([
    ApproachObject(),
    GraspObject(),
    # ... مزید نوڈز
])
```

## کنٹرول کے لیے مشین لرننگ

جدید روبوٹس تیزی سے سیکھی ہوئی پالیسیاں استعمال کرتے ہیں۔

### ریانفورسمنٹ لرننگ

RL آزمائش اور غلطی کے ذریعے پالیسیوں کو ٹرین کرتا ہے:

**پالیسی گریڈینٹ مثال (REINFORCE):**
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
            nn.Tanh()  # [-1, 1] میں ایکشنز
        )
    
    def forward(self, state):
        return self.fc(state)

def train_policy(env, policy, num_episodes=1000):
    optimizer = optim.Adam(policy.parameters(), lr=0.001)
    
    for episode in range(num_episodes):
        states, actions, rewards = [], [], []
        state = env.reset()
        done = False
        
        # ٹریجیکٹری جمع کریں
        while not done:
            state_tensor = torch.FloatTensor(state).unsqueeze(0)
            action = policy(state_tensor)
            
            states.append(state_tensor)
            actions.append(action)
            
            next_state, reward, done = env.step(action.detach().numpy()[0])
            rewards.append(reward)
            state = next_state
        
        # ریٹرنز کمپیوٹ کریں
        returns = []
        G = 0
        for r in reversed(rewards):
            G = r + 0.99 * G  # ڈسکاؤنٹ فیکٹر
            returns.insert(0, G)
        
        returns = torch.FloatTensor(returns)
        returns = (returns - returns.mean()) / (returns.std() + 1e-8)
        
        # پالیسی اپڈیٹ کریں
        policy_loss = []
        for state, action, G in zip(states, actions, returns):
            log_prob = -((action - policy(state))**2).sum()
            policy_loss.append(-log_prob * G)
        
        optimizer.zero_grad()
        loss = torch.stack(policy_loss).sum()
        loss.backward()
        optimizer.step()
```

## خلاصہ

AI الگورتھمز جسمانی روبوٹس کے لیے ذہانت کی تہہ فراہم کرتے ہیں:

- **لوکلائزیشن اور میپنگ**: جاننا کہ آپ کہاں ہیں اور نمائندگی بنانا
- **پاتھ پلاننگ**: اہداف تک تصادم سے پاک راستوں کا حساب
- **موشن پلاننگ**: ہیرا پھیری کے لیے متعدد جوائنٹس کی ہم آہنگی
- **بیہیویئر ٹریز**: پیچیدہ فیصلہ سازی کی ساخت
- **ریانفورسمنٹ لرننگ**: تجربے سے کنٹرول پالیسیاں سیکھنا

یہ الگورتھمز مل کر حقیقی ذہین جسمانی نظام بنانے کے لیے کام کرتے ہیں جو حقیقی دنیا میں خودمختاری سے کام کر سکتے ہیں۔

---

**پچھلا**: [ایکچویٹرز اور موومنٹ کنٹرول ←](./chapter2.md)

## مشقیں

1. 2D گرڈ پر A* پاتھ فائنڈنگ نافذ اور ویژولائز کریں
2. ہوم سروس روبوٹ کے لیے سادہ بیہیویئر ٹری بنائیں
3. 3-DOF روبوٹ آرم کے لیے انورس کائنیمیٹکس حل کریں
4. ایک ہی پلاننگ مسئلے پر RRT اور A* کارکردگی کا موازنہ کریں
5. ڈنڈا متوازن کرنے کے لیے سادہ RL ایجنٹ ٹرین کریں (CartPole ماحول)
