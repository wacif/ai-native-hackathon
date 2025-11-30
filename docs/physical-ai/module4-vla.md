---
sidebar_position: 5
---

# Module 4: Voice-to-Action (VLA)

> **Focus: Natural language control and Vision-Language-Action models**

This is where the magic happens. In this final module, you'll give your robot the ability to understand human speech, reason about tasks, and execute complex actions—all through natural language.

## The VLA Revolution

Traditional robot programming:
```
if speech == "get water":
    move_to(kitchen)
    find(cup)
    grasp(cup)
    move_to(sink)
    ...
```

VLA approach:
```
Human: "I'm thirsty"
Robot: *understands context, plans, executes*
```

VLA models combine:
- **Vision**: Seeing the world through cameras
- **Language**: Understanding natural commands
- **Action**: Generating motor commands

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    VLA Pipeline                              │
├─────────────┬─────────────┬─────────────┬──────────────────┤
│  🎤 Voice   │  👁️ Vision   │  🧠 LLM     │  🦾 Action       │
│  Input      │  Input      │  Planner    │  Generation      │
├─────────────┼─────────────┼─────────────┼──────────────────┤
│  Whisper    │  Camera     │  Gemini/    │  Policy          │
│  STT        │  RGB-D      │  GPT-4/etc  │  Network         │
├─────────────┴─────────────┴─────────────┴──────────────────┤
│                    Robot Hardware                            │
└─────────────────────────────────────────────────────────────┘
```

## Voice Input with Whisper

OpenAI's Whisper is a state-of-the-art speech recognition model:

### Installation

```bash
# Install Whisper
pip install openai-whisper

# For faster inference
pip install faster-whisper

# Install audio dependencies
sudo apt install portaudio19-dev python3-pyaudio
pip install pyaudio sounddevice
```

### Real-time Speech Recognition

```python
import whisper
import sounddevice as sd
import numpy as np
from scipy.io.wavfile import write
import tempfile

class VoiceInterface:
    def __init__(self, model_size="base"):
        """Initialize Whisper model
        
        Model sizes: tiny, base, small, medium, large
        Larger = more accurate, slower
        """
        self.model = whisper.load_model(model_size)
        self.sample_rate = 16000
        
    def listen(self, duration=5):
        """Record audio for specified duration"""
        print("Listening...")
        
        # Record audio
        audio = sd.rec(
            int(duration * self.sample_rate),
            samplerate=self.sample_rate,
            channels=1,
            dtype='float32'
        )
        sd.wait()
        
        return audio.flatten()
        
    def transcribe(self, audio):
        """Convert audio to text"""
        # Whisper expects float32 numpy array
        result = self.model.transcribe(
            audio,
            language="en",
            fp16=False  # Use fp16=True on GPU
        )
        return result["text"].strip()
        
    def listen_and_transcribe(self, duration=5):
        """One-shot voice to text"""
        audio = self.listen(duration)
        text = self.transcribe(audio)
        print(f"You said: {text}")
        return text

# Usage
voice = VoiceInterface(model_size="base")
command = voice.listen_and_transcribe(duration=5)
```

### Continuous Listening with Wake Word

```python
import threading
from queue import Queue

class ContinuousListener:
    def __init__(self, wake_word="hey robot"):
        self.voice = VoiceInterface(model_size="small")
        self.wake_word = wake_word.lower()
        self.command_queue = Queue()
        self.running = False
        
    def start(self):
        """Start listening in background thread"""
        self.running = True
        self.thread = threading.Thread(target=self._listen_loop)
        self.thread.start()
        
    def stop(self):
        self.running = False
        self.thread.join()
        
    def _listen_loop(self):
        while self.running:
            # Listen for wake word (short clips)
            audio = self.voice.listen(duration=2)
            text = self.voice.transcribe(audio)
            
            if self.wake_word in text.lower():
                print("Wake word detected! Listening for command...")
                
                # Listen for actual command (longer)
                audio = self.voice.listen(duration=5)
                command = self.voice.transcribe(audio)
                
                if command:
                    self.command_queue.put(command)
                    
    def get_command(self, timeout=None):
        """Get next command from queue"""
        try:
            return self.command_queue.get(timeout=timeout)
        except:
            return None

# Usage
listener = ContinuousListener(wake_word="hey robot")
listener.start()

while True:
    command = listener.get_command(timeout=1)
    if command:
        print(f"Processing command: {command}")
        # Send to LLM planner
```

## LLM Task Planning

Use an LLM to convert natural language to robot actions:

### Task Planner with Google Gemini

```python
import google.generativeai as genai
import json

class TaskPlanner:
    def __init__(self):
        genai.configure(api_key="YOUR_GEMINI_API_KEY")
        self.model = genai.GenerativeModel('gemini-1.5-flash')
        
        # Define available robot actions
        self.actions = """
        Available robot actions:
        - move_to(location: str) - Navigate to a location
        - pick_up(object: str) - Grasp and lift an object
        - put_down(surface: str) - Place held object on surface
        - look_at(target: str) - Turn camera toward target
        - say(message: str) - Speak a message
        - wait(seconds: float) - Pause execution
        - find(object: str) -> bool - Search for object
        - is_holding() -> bool - Check if holding something
        """
        
    def plan(self, command: str, context: dict = None) -> list:
        """Convert natural language to action sequence"""
        
        prompt = f"""You are a robot task planner. Convert the user's request 
into a sequence of robot actions.

{self.actions}

Current context:
- Robot location: {context.get('location', 'living room')}
- Objects visible: {context.get('visible_objects', [])}
- Currently holding: {context.get('holding', None)}

User command: "{command}"

Respond with a JSON array of actions. Each action should have:
- "action": the function name
- "params": dictionary of parameters
- "reasoning": why this action is needed

Example response:
[
  {{"action": "move_to", "params": {{"location": "kitchen"}}, "reasoning": "Need to go to kitchen first"}},
  {{"action": "find", "params": {{"object": "cup"}}, "reasoning": "Looking for a cup"}}
]
"""
        
        response = self.model.generate_content(prompt)
        
        # Parse JSON from response
        text = response.text
        # Extract JSON array from response
        start = text.find('[')
        end = text.rfind(']') + 1
        json_str = text[start:end]
        
        return json.loads(json_str)
        
    def execute_plan(self, plan: list, robot):
        """Execute a plan on the robot"""
        for step in plan:
            action = step['action']
            params = step['params']
            
            print(f"Executing: {action}({params})")
            
            # Call the appropriate robot method
            method = getattr(robot, action)
            result = method(**params)
            
            if not result:
                print(f"Action {action} failed!")
                return False
                
        return True

# Usage
planner = TaskPlanner()
context = {
    'location': 'living room',
    'visible_objects': ['couch', 'table', 'remote'],
    'holding': None
}

plan = planner.plan("Please bring me a glass of water", context)
print(json.dumps(plan, indent=2))
```

### Chain-of-Thought Reasoning

For complex tasks, use chain-of-thought prompting:

```python
class AdvancedPlanner:
    def __init__(self):
        self.model = genai.GenerativeModel('gemini-1.5-pro')
        
    def plan_with_reasoning(self, command: str, context: dict) -> dict:
        prompt = f"""You are an intelligent robot assistant. Think through this 
task step by step.

TASK: {command}

CONTEXT:
- Current location: {context.get('location')}
- Known locations: {context.get('known_locations', [])}
- Visible objects: {context.get('visible_objects', [])}
- Holding: {context.get('holding')}

Think through this step by step:
1. What is the user really asking for?
2. What information do I need?
3. What could go wrong?
4. What's the optimal sequence of actions?

Provide your response in this format:
{{
  "understanding": "What I understand the user wants",
  "reasoning": "Step by step thinking",
  "preconditions": ["What must be true before starting"],
  "plan": [
    {{"action": "...", "params": {{}}, "fallback": "..."}}
  ],
  "success_criteria": "How to know if task succeeded"
}}
"""
        response = self.model.generate_content(prompt)
        return json.loads(response.text)
```

## Vision-Language Models

Combine vision and language for understanding:

### Scene Understanding

```python
import google.generativeai as genai
from PIL import Image
import base64
import io

class VisionLanguageModel:
    def __init__(self):
        genai.configure(api_key="YOUR_API_KEY")
        self.model = genai.GenerativeModel('gemini-1.5-flash')
        
    def describe_scene(self, image_path: str) -> str:
        """Describe what the robot sees"""
        image = Image.open(image_path)
        
        response = self.model.generate_content([
            "Describe this scene from a robot's perspective. Include:",
            "1. Objects visible and their approximate positions",
            "2. Any people and what they're doing",
            "3. Potential navigation paths",
            "4. Any hazards or obstacles",
            image
        ])
        
        return response.text
        
    def find_object(self, image_path: str, target: str) -> dict:
        """Locate a specific object in the scene"""
        image = Image.open(image_path)
        
        response = self.model.generate_content([
            f"Find the {target} in this image.",
            "If found, describe its location (left/center/right, near/far)",
            "If not found, say 'not visible'",
            "Respond in JSON: {\"found\": bool, \"location\": str, \"confidence\": float}",
            image
        ])
        
        return json.loads(response.text)
        
    def answer_question(self, image_path: str, question: str) -> str:
        """Answer questions about what the robot sees"""
        image = Image.open(image_path)
        
        response = self.model.generate_content([
            question,
            image
        ])
        
        return response.text

# Usage with ROS 2
class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.vlm = VisionLanguageModel()
        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10
        )
        
        self.query_srv = self.create_service(
            StringQuery, '/vision/query', self.query_callback
        )
        
    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        
    def query_callback(self, request, response):
        # Save current frame
        cv2.imwrite('/tmp/current_frame.jpg', self.latest_image)
        
        # Query VLM
        answer = self.vlm.answer_question(
            '/tmp/current_frame.jpg',
            request.query
        )
        
        response.answer = answer
        return response
```

## Complete VLA System

Putting it all together:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
import threading

class VLARobot(Node):
    def __init__(self):
        super().__init__('vla_robot')
        
        # Initialize components
        self.voice = VoiceInterface(model_size="base")
        self.planner = TaskPlanner()
        self.vision = VisionLanguageModel()
        
        # ROS 2 publishers/subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speech_pub = self.create_publisher(String, '/speech', 10)
        self.nav_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        self.image_sub = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10
        )
        
        # State
        self.current_image = None
        self.context = {
            'location': 'home',
            'holding': None,
            'visible_objects': []
        }
        
        # Start voice listener
        self.voice_thread = threading.Thread(target=self.voice_loop)
        self.voice_thread.start()
        
    def voice_loop(self):
        """Continuously listen for commands"""
        while rclpy.ok():
            try:
                command = self.voice.listen_and_transcribe(duration=5)
                if command and len(command) > 3:
                    self.process_command(command)
            except Exception as e:
                self.get_logger().error(f"Voice error: {e}")
                
    def process_command(self, command: str):
        """Process a voice command"""
        self.get_logger().info(f"Processing: {command}")
        
        # Update context with current vision
        if self.current_image is not None:
            scene_description = self.vision.describe_scene(self.current_image)
            self.context['visible_objects'] = self._extract_objects(scene_description)
        
        # Get plan from LLM
        plan = self.planner.plan(command, self.context)
        
        # Confirm with user
        self.say(f"I'll {self._summarize_plan(plan)}. Is that okay?")
        
        # Wait for confirmation
        response = self.voice.listen_and_transcribe(duration=3)
        if "yes" in response.lower() or "okay" in response.lower():
            self.execute_plan(plan)
        else:
            self.say("Okay, I won't do that.")
            
    def execute_plan(self, plan: list):
        """Execute action sequence"""
        for step in plan:
            action = step['action']
            params = step['params']
            
            self.get_logger().info(f"Executing: {action}")
            
            if action == 'move_to':
                self.move_to(params['location'])
            elif action == 'pick_up':
                self.pick_up(params['object'])
            elif action == 'put_down':
                self.put_down(params['surface'])
            elif action == 'say':
                self.say(params['message'])
            elif action == 'find':
                found = self.find(params['object'])
                if not found:
                    self.say(f"I couldn't find the {params['object']}")
                    return
            elif action == 'wait':
                time.sleep(params['seconds'])
                
    # Robot actions
    def move_to(self, location: str):
        """Navigate to a location"""
        # This would use Nav2 in practice
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = self._get_location_coords(location)[0]
        pose.pose.position.y = self._get_location_coords(location)[1]
        self.nav_pub.publish(pose)
        
    def say(self, message: str):
        """Speak a message (TTS)"""
        msg = String()
        msg.data = message
        self.speech_pub.publish(msg)
        
    def find(self, object_name: str) -> bool:
        """Search for an object"""
        if self.current_image is None:
            return False
            
        result = self.vision.find_object(self.current_image, object_name)
        return result.get('found', False)
        
    def pick_up(self, object_name: str):
        """Pick up an object (would control arm)"""
        self.get_logger().info(f"Picking up {object_name}")
        self.context['holding'] = object_name
        
    def put_down(self, surface: str):
        """Put down held object"""
        self.get_logger().info(f"Putting down on {surface}")
        self.context['holding'] = None
        
    def image_callback(self, msg):
        """Store latest camera frame"""
        self.current_image = msg
        
    def _get_location_coords(self, location: str) -> tuple:
        """Map location names to coordinates"""
        locations = {
            'kitchen': (5.0, 2.0),
            'living room': (0.0, 0.0),
            'bedroom': (-3.0, 4.0),
            'bathroom': (-3.0, 0.0)
        }
        return locations.get(location, (0.0, 0.0))
        
    def _summarize_plan(self, plan: list) -> str:
        """Create human-readable summary"""
        actions = [f"{s['action']} {list(s['params'].values())[0] if s['params'] else ''}" 
                   for s in plan[:3]]
        return ", then ".join(actions)

def main():
    rclpy.init()
    robot = VLARobot()
    rclpy.spin(robot)

if __name__ == '__main__':
    main()
```

## Text-to-Speech for Robot Response

Make the robot speak:

```python
import pyttsx3
# Or use cloud TTS for better quality:
from gtts import gTTS
import pygame

class SpeechOutput:
    def __init__(self, engine='local'):
        if engine == 'local':
            self.engine = pyttsx3.init()
            self.engine.setProperty('rate', 150)
        else:
            pygame.mixer.init()
            
        self.engine_type = engine
        
    def speak(self, text: str):
        """Convert text to speech"""
        if self.engine_type == 'local':
            self.engine.say(text)
            self.engine.runAndWait()
        else:
            # Cloud TTS (better quality)
            tts = gTTS(text=text, lang='en')
            tts.save('/tmp/speech.mp3')
            pygame.mixer.music.load('/tmp/speech.mp3')
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)

# ROS 2 TTS Node
class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.tts = SpeechOutput(engine='cloud')
        
        self.subscription = self.create_subscription(
            String,
            '/speech',
            self.speech_callback,
            10
        )
        
    def speech_callback(self, msg):
        self.tts.speak(msg.data)
```

## Capstone Project: Home Assistant Robot

Build a complete home assistant robot that can:

1. **Listen** for natural language commands
2. **See** and understand the environment
3. **Plan** complex multi-step tasks
4. **Navigate** autonomously
5. **Speak** to communicate with humans

### Project Structure

```
home_assistant_robot/
├── launch/
│   └── full_system.launch.py
├── config/
│   ├── nav2_params.yaml
│   └── locations.yaml
├── scripts/
│   ├── voice_interface.py
│   ├── task_planner.py
│   ├── vision_system.py
│   └── robot_controller.py
└── package.xml
```

### Integration Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('nav2_bringup'),
                '/launch/navigation_launch.py'
            ])
        ),
        
        # Isaac ROS VSLAM
        Node(
            package='isaac_ros_visual_slam',
            executable='visual_slam_node',
            name='vslam'
        ),
        
        # Voice interface
        Node(
            package='home_assistant_robot',
            executable='voice_interface.py',
            name='voice_interface'
        ),
        
        # Task planner
        Node(
            package='home_assistant_robot',
            executable='task_planner.py',
            name='task_planner'
        ),
        
        # Vision system
        Node(
            package='home_assistant_robot',
            executable='vision_system.py',
            name='vision_system'
        ),
        
        # Text-to-speech
        Node(
            package='home_assistant_robot',
            executable='tts_node.py',
            name='tts'
        ),
        
        # Main robot controller
        Node(
            package='home_assistant_robot',
            executable='robot_controller.py',
            name='robot_controller'
        )
    ])
```

## Evaluation Criteria

Your capstone project will be evaluated on:

| Criteria | Points | Description |
|----------|--------|-------------|
| Voice Recognition | 20 | Accurate speech-to-text |
| Task Planning | 25 | Sensible action sequences |
| Vision Integration | 20 | Scene understanding |
| Navigation | 20 | Autonomous movement |
| Communication | 15 | Natural speech output |

## Key Takeaways

1. **Whisper for voice** - Best-in-class speech recognition
2. **LLMs for planning** - Convert language to action sequences
3. **VLMs for vision** - Understand what the robot sees
4. **Integration is key** - Combine all modalities seamlessly
5. **Feedback loops** - Let the robot communicate back

## Course Complete! 🎉

You've learned to build:
- ✅ ROS 2 robotic systems
- ✅ Simulated environments in Gazebo
- ✅ GPU-accelerated perception with Isaac
- ✅ Voice-controlled robots with VLA

**What's next?**
- Join the [ROS 2 Discord](https://discord.gg/ros)
- Contribute to open-source robotics
- Build your own Physical AI projects!

---

**Back to**: [Course Introduction →](./intro)
