---
sidebar_position: 5
---

# ماڈیول 4: Voice-to-Action (VLA)

> **توجہ: قدرتی زبان کا کنٹرول اور Vision-Language-Action ماڈلز**

یہاں جادو ہوتا ہے۔ اس آخری ماڈیول میں، آپ اپنے روبوٹ کو انسانی تقریر سمجھنے، کاموں کے بارے میں سوچنے، اور پیچیدہ عمل انجام دینے کی صلاحیت دیں گے — سب کچھ قدرتی زبان کے ذریعے۔

## VLA انقلاب

روایتی روبوٹ پروگرامنگ:
```
if speech == "get water":
    move_to(kitchen)
    find(cup)
    grasp(cup)
    move_to(sink)
    ...
```

VLA اپروچ:
```
انسان: "مجھے پیاس لگی ہے"
روبوٹ: *سیاق سمجھتا ہے، منصوبہ بناتا ہے، انجام دیتا ہے*
```

VLA ماڈلز یکجا کرتے ہیں:
- **Vision**: کیمروں کے ذریعے دنیا دیکھنا
- **Language**: قدرتی commands سمجھنا
- **Action**: موٹر commands generate کرنا

## Architecture کا جائزہ

```
┌─────────────────────────────────────────────────────────────┐
│                    VLA Pipeline                              │
├─────────────┬─────────────┬─────────────┬──────────────────┤
│  🎤 آواز    │  👁️ Vision   │  🧠 LLM     │  🦾 Action       │
│  Input      │  Input      │  Planner    │  Generation      │
├─────────────┼─────────────┼─────────────┼──────────────────┤
│  Whisper    │  Camera     │  Gemini/    │  Policy          │
│  STT        │  RGB-D      │  GPT-4/etc  │  Network         │
├─────────────┴─────────────┴─────────────┴──────────────────┤
│                    Robot Hardware                            │
└─────────────────────────────────────────────────────────────┘
```

## Whisper کے ساتھ آواز کی Input

OpenAI کا Whisper ایک جدید ترین speech recognition ماڈل ہے:

### انسٹالیشن

```bash
# Whisper انسٹال کریں
pip install openai-whisper

# تیز inference کے لیے
pip install faster-whisper

# آڈیو dependencies انسٹال کریں
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
        """Whisper ماڈل initialize کریں
        
        ماڈل سائز: tiny، base، small، medium، large
        بڑا = زیادہ درست، سست
        """
        self.model = whisper.load_model(model_size)
        self.sample_rate = 16000
        
    def listen(self, duration=5):
        """مخصوص مدت کے لیے آڈیو ریکارڈ کریں"""
        print("سن رہا ہوں...")
        
        # آڈیو ریکارڈ کریں
        audio = sd.rec(
            int(duration * self.sample_rate),
            samplerate=self.sample_rate,
            channels=1,
            dtype='float32'
        )
        sd.wait()
        
        return audio.flatten()
        
    def transcribe(self, audio):
        """آڈیو کو متن میں بدلیں"""
        # Whisper float32 numpy array چاہتا ہے
        result = self.model.transcribe(
            audio,
            language="en",
            fp16=False  # GPU پر fp16=True استعمال کریں
        )
        return result["text"].strip()
        
    def listen_and_transcribe(self, duration=5):
        """ایک بار میں آواز سے متن"""
        audio = self.listen(duration)
        text = self.transcribe(audio)
        print(f"آپ نے کہا: {text}")
        return text

# استعمال
voice = VoiceInterface(model_size="base")
command = voice.listen_and_transcribe(duration=5)
```

### Wake Word کے ساتھ مسلسل سننا

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
        """Background thread میں سننا شروع کریں"""
        self.running = True
        self.thread = threading.Thread(target=self._listen_loop)
        self.thread.start()
        
    def stop(self):
        self.running = False
        self.thread.join()
        
    def _listen_loop(self):
        while self.running:
            # Wake word کے لیے سنیں (چھوٹے clips)
            audio = self.voice.listen(duration=2)
            text = self.voice.transcribe(audio)
            
            if self.wake_word in text.lower():
                print("Wake word ملا! Command کے لیے سن رہا ہوں...")
                
                # اصل command کے لیے سنیں (لمبا)
                audio = self.voice.listen(duration=5)
                command = self.voice.transcribe(audio)
                
                if command:
                    self.command_queue.put(command)
                    
    def get_command(self, timeout=None):
        """Queue سے اگلا command حاصل کریں"""
        try:
            return self.command_queue.get(timeout=timeout)
        except:
            return None

# استعمال
listener = ContinuousListener(wake_word="hey robot")
listener.start()

while True:
    command = listener.get_command(timeout=1)
    if command:
        print(f"Command process کر رہا ہوں: {command}")
        # LLM planner کو بھیجیں
```

## LLM Task Planning

قدرتی زبان کو روبوٹ اعمال میں بدلنے کے لیے LLM استعمال کریں:

### Google Gemini کے ساتھ Task Planner

```python
import google.generativeai as genai
import json

class TaskPlanner:
    def __init__(self):
        genai.configure(api_key="YOUR_GEMINI_API_KEY")
        self.model = genai.GenerativeModel('gemini-1.5-flash')
        
        # دستیاب روبوٹ اعمال بیان کریں
        self.actions = """
        دستیاب روبوٹ اعمال:
        - move_to(location: str) - کسی جگہ navigate کریں
        - pick_up(object: str) - کوئی چیز پکڑیں اور اٹھائیں
        - put_down(surface: str) - پکڑی ہوئی چیز کہیں رکھیں
        - look_at(target: str) - کیمرا target کی طرف موڑیں
        - say(message: str) - پیغام بولیں
        - wait(seconds: float) - عمل روکیں
        - find(object: str) -> bool - چیز تلاش کریں
        - is_holding() -> bool - چیک کریں کہ کچھ پکڑا ہوا ہے
        """
        
    def plan(self, command: str, context: dict = None) -> list:
        """قدرتی زبان کو عمل کی ترتیب میں بدلیں"""
        
        prompt = f"""آپ ایک روبوٹ ٹاسک پلانر ہیں۔ صارف کی درخواست کو 
روبوٹ اعمال کی ترتیب میں بدلیں۔

{self.actions}

موجودہ سیاق:
- روبوٹ کی جگہ: {context.get('location', 'لونگ روم')}
- نظر آنے والی چیزیں: {context.get('visible_objects', [])}
- فی الحال پکڑا ہوا: {context.get('holding', None)}

صارف کا حکم: "{command}"

اعمال کی JSON array واپس کریں۔ ہر عمل میں:
- "action": فنکشن کا نام
- "params": پیرامیٹرز کی dictionary
- "reasoning": یہ عمل کیوں ضروری ہے

مثال جواب:
[
  {{"action": "move_to", "params": {{"location": "kitchen"}}, "reasoning": "پہلے کچن جانا ہے"}},
  {{"action": "find", "params": {{"object": "cup"}}, "reasoning": "کپ ڈھونڈ رہا ہوں"}}
]
"""
        
        response = self.model.generate_content(prompt)
        
        # جواب سے JSON parse کریں
        text = response.text
        # جواب سے JSON array نکالیں
        start = text.find('[')
        end = text.rfind(']') + 1
        json_str = text[start:end]
        
        return json.loads(json_str)
        
    def execute_plan(self, plan: list, robot):
        """روبوٹ پر منصوبہ چلائیں"""
        for step in plan:
            action = step['action']
            params = step['params']
            
            print(f"چلا رہا ہوں: {action}({params})")
            
            # مناسب روبوٹ method call کریں
            method = getattr(robot, action)
            result = method(**params)
            
            if not result:
                print(f"عمل {action} ناکام ہوا!")
                return False
                
        return True

# استعمال
planner = TaskPlanner()
context = {
    'location': 'لونگ روم',
    'visible_objects': ['صوفہ', 'میز', 'ریموٹ'],
    'holding': None
}

plan = planner.plan("براہ کرم مجھے پانی کا گلاس لا دیں", context)
print(json.dumps(plan, indent=2))
```

### Chain-of-Thought Reasoning

پیچیدہ کاموں کے لیے، chain-of-thought prompting استعمال کریں:

```python
class AdvancedPlanner:
    def __init__(self):
        self.model = genai.GenerativeModel('gemini-1.5-pro')
        
    def plan_with_reasoning(self, command: str, context: dict) -> dict:
        prompt = f"""آپ ایک ذہین روبوٹ اسسٹنٹ ہیں۔ اس کام کے بارے میں 
قدم بہ قدم سوچیں۔

کام: {command}

سیاق:
- موجودہ جگہ: {context.get('location')}
- معلوم جگہیں: {context.get('known_locations', [])}
- نظر آنے والی چیزیں: {context.get('visible_objects', [])}
- پکڑا ہوا: {context.get('holding')}

قدم بہ قدم سوچیں:
1. صارف اصل میں کیا چاہتا ہے؟
2. مجھے کون سی معلومات چاہیے؟
3. کیا غلط ہو سکتا ہے؟
4. اعمال کی بہترین ترتیب کیا ہے؟

اپنا جواب اس فارمیٹ میں دیں:
{{
  "understanding": "میں سمجھتا ہوں صارف کیا چاہتا ہے",
  "reasoning": "قدم بہ قدم سوچ",
  "preconditions": ["شروع کرنے سے پہلے کیا سچ ہونا چاہیے"],
  "plan": [
    {{"action": "...", "params": {{}}, "fallback": "..."}}
  ],
  "success_criteria": "کیسے پتہ چلے کہ کام کامیاب ہوا"
}}
"""
        response = self.model.generate_content(prompt)
        return json.loads(response.text)
```

## Vision-Language ماڈلز

سمجھنے کے لیے vision اور language یکجا کریں:

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
        """روبوٹ کیا دیکھتا ہے اسے بیان کریں"""
        image = Image.open(image_path)
        
        response = self.model.generate_content([
            "روبوٹ کے نقطہ نظر سے اس منظر کو بیان کریں۔ شامل کریں:",
            "1. نظر آنے والی چیزیں اور ان کی تقریباً پوزیشنز",
            "2. کوئی لوگ اور وہ کیا کر رہے ہیں",
            "3. ممکنہ navigation راستے",
            "4. کوئی خطرات یا رکاوٹیں",
            image
        ])
        
        return response.text
        
    def find_object(self, image_path: str, target: str) -> dict:
        """منظر میں مخصوص چیز تلاش کریں"""
        image = Image.open(image_path)
        
        response = self.model.generate_content([
            f"اس تصویر میں {target} ڈھونڈیں۔",
            "اگر ملا تو، اس کی جگہ بیان کریں (بائیں/درمیان/دائیں، قریب/دور)",
            "اگر نہیں ملا تو، 'نظر نہیں آتا' کہیں",
            'JSON میں جواب دیں: {"found": bool, "location": str, "confidence": float}',
            image
        ])
        
        return json.loads(response.text)
        
    def answer_question(self, image_path: str, question: str) -> str:
        """روبوٹ کیا دیکھتا ہے اس کے بارے میں سوالات کے جواب دیں"""
        image = Image.open(image_path)
        
        response = self.model.generate_content([
            question,
            image
        ])
        
        return response.text

# ROS 2 کے ساتھ استعمال
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
        # موجودہ frame محفوظ کریں
        cv2.imwrite('/tmp/current_frame.jpg', self.latest_image)
        
        # VLM سے پوچھیں
        answer = self.vlm.answer_question(
            '/tmp/current_frame.jpg',
            request.query
        )
        
        response.answer = answer
        return response
```

## مکمل VLA سسٹم

سب کچھ ایک ساتھ:

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
        
        # اجزاء initialize کریں
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
            'location': 'گھر',
            'holding': None,
            'visible_objects': []
        }
        
        # Voice listener شروع کریں
        self.voice_thread = threading.Thread(target=self.voice_loop)
        self.voice_thread.start()
        
    def voice_loop(self):
        """مسلسل commands کے لیے سنیں"""
        while rclpy.ok():
            try:
                command = self.voice.listen_and_transcribe(duration=5)
                if command and len(command) > 3:
                    self.process_command(command)
            except Exception as e:
                self.get_logger().error(f"آواز میں خرابی: {e}")
                
    def process_command(self, command: str):
        """آواز کا حکم process کریں"""
        self.get_logger().info(f"Process کر رہا ہوں: {command}")
        
        # موجودہ vision کے ساتھ context update کریں
        if self.current_image is not None:
            scene_description = self.vision.describe_scene(self.current_image)
            self.context['visible_objects'] = self._extract_objects(scene_description)
        
        # LLM سے منصوبہ حاصل کریں
        plan = self.planner.plan(command, self.context)
        
        # صارف سے تصدیق کریں
        self.say(f"میں {self._summarize_plan(plan)} کروں گا۔ ٹھیک ہے؟")
        
        # تصدیق کا انتظار کریں
        response = self.voice.listen_and_transcribe(duration=3)
        if "ہاں" in response.lower() or "ٹھیک" in response.lower():
            self.execute_plan(plan)
        else:
            self.say("ٹھیک ہے، میں یہ نہیں کروں گا۔")
            
    def execute_plan(self, plan: list):
        """عمل کی ترتیب چلائیں"""
        for step in plan:
            action = step['action']
            params = step['params']
            
            self.get_logger().info(f"چلا رہا ہوں: {action}")
            
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
                    self.say(f"مجھے {params['object']} نہیں ملا")
                    return
            elif action == 'wait':
                time.sleep(params['seconds'])
                
    # روبوٹ اعمال
    def move_to(self, location: str):
        """کسی جگہ navigate کریں"""
        # عملی طور پر یہ Nav2 استعمال کرے گا
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = self._get_location_coords(location)[0]
        pose.pose.position.y = self._get_location_coords(location)[1]
        self.nav_pub.publish(pose)
        
    def say(self, message: str):
        """پیغام بولیں (TTS)"""
        msg = String()
        msg.data = message
        self.speech_pub.publish(msg)
        
    def find(self, object_name: str) -> bool:
        """کوئی چیز تلاش کریں"""
        if self.current_image is None:
            return False
            
        result = self.vision.find_object(self.current_image, object_name)
        return result.get('found', False)
        
    def pick_up(self, object_name: str):
        """کوئی چیز اٹھائیں (arm کنٹرول کرے گا)"""
        self.get_logger().info(f"{object_name} اٹھا رہا ہوں")
        self.context['holding'] = object_name
        
    def put_down(self, surface: str):
        """پکڑی ہوئی چیز رکھیں"""
        self.get_logger().info(f"{surface} پر رکھ رہا ہوں")
        self.context['holding'] = None
        
    def image_callback(self, msg):
        """تازہ ترین camera frame محفوظ کریں"""
        self.current_image = msg
        
    def _get_location_coords(self, location: str) -> tuple:
        """جگہ کے ناموں کو coordinates میں map کریں"""
        locations = {
            'کچن': (5.0, 2.0),
            'لونگ روم': (0.0, 0.0),
            'بیڈ روم': (-3.0, 4.0),
            'باتھ روم': (-3.0, 0.0)
        }
        return locations.get(location, (0.0, 0.0))
        
    def _summarize_plan(self, plan: list) -> str:
        """انسان کے پڑھنے کے قابل خلاصہ بنائیں"""
        actions = [f"{s['action']} {list(s['params'].values())[0] if s['params'] else ''}" 
                   for s in plan[:3]]
        return "، پھر ".join(actions)

def main():
    rclpy.init()
    robot = VLARobot()
    rclpy.spin(robot)

if __name__ == '__main__':
    main()
```

## روبوٹ جواب کے لیے Text-to-Speech

روبوٹ کو بولنے دیں:

```python
import pyttsx3
# یا بہتر معیار کے لیے cloud TTS استعمال کریں:
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
        """متن کو آواز میں بدلیں"""
        if self.engine_type == 'local':
            self.engine.say(text)
            self.engine.runAndWait()
        else:
            # Cloud TTS (بہتر معیار)
            tts = gTTS(text=text, lang='ur')  # اردو کے لیے
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

## Capstone پروجیکٹ: Home Assistant روبوٹ

ایک مکمل home assistant روبوٹ بنائیں جو:

1. قدرتی زبان کے commands کے لیے **سنے**
2. ماحول کو **دیکھے** اور سمجھے
3. پیچیدہ multi-step کام **منصوبہ بندی** کرے
4. خود مختار طریقے سے **navigate** کرے
5. انسانوں سے communicate کرنے کے لیے **بولے**

### پروجیکٹ کا ڈھانچہ

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

## جائزہ معیار

آپ کے capstone پروجیکٹ کا جائزہ ہوگا:

| معیار | پوائنٹس | تفصیل |
|-------|---------|-------|
| Voice Recognition | 20 | درست speech-to-text |
| Task Planning | 25 | معقول عمل کی ترتیب |
| Vision Integration | 20 | Scene understanding |
| Navigation | 20 | خود مختار حرکت |
| Communication | 15 | قدرتی آواز کی output |

## اہم نکات

1. **آواز کے لیے Whisper** - بہترین درجے کی speech recognition
2. **Planning کے لیے LLMs** - زبان کو عمل کی ترتیب میں بدلیں
3. **Vision کے لیے VLMs** - روبوٹ کیا دیکھتا ہے سمجھیں
4. **Integration کلیدی ہے** - تمام modalities کو بے تکلف یکجا کریں
5. **Feedback loops** - روبوٹ کو واپس communicate کرنے دیں

## کورس مکمل! 🎉

آپ نے سیکھا:
- ✅ ROS 2 روبوٹک سسٹمز
- ✅ Gazebo میں simulated environments
- ✅ Isaac کے ساتھ GPU-accelerated perception
- ✅ VLA کے ساتھ voice-controlled روبوٹس

**آگے کیا؟**
- [ROS 2 Discord](https://discord.gg/ros) میں شامل ہوں
- Open-source روبوٹکس میں حصہ ڈالیں
- اپنے Physical AI پروجیکٹس بنائیں!

---

**واپس**: [کورس تعارف →](./intro)
