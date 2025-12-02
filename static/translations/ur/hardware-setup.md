---
sidebar_position: 7
---

# ہارڈویئر سیٹ اپ گائیڈ

> **اپنا Physical AI ڈیولپمنٹ ماحول ترتیب دینا**

یہ گائیڈ Physical AI کورس کے لیے ہارڈویئر کی ضروریات اور سیٹ اپ کے طریقہ کار کا احاطہ کرتی ہے۔ آپ کے بجٹ اور ضروریات کی بنیاد پر تین اہم آپشنز ہیں۔

## آپشن 1: ہائی پرفارمنس ورک سٹیشن

بہترین: مکمل Isaac Sim تجربہ، لوکل ٹریننگ، پروفیشنل ڈیولپمنٹ کے لیے

### تجویز کردہ وضاحتیں

| کمپوننٹ | وضاحت | کیوں |
|---------|--------|------|
| **GPU** | NVIDIA RTX 4070 Ti Super (16GB) | Isaac Sim کو RTX، 12GB+ VRAM چاہیے |
| **CPU** | Intel i7-14700K / AMD Ryzen 9 7900X | Physics simulation CPU-intensive ہے |
| **RAM** | 64GB DDR5 | متعدد simulators + AI ماڈلز |
| **Storage** | 1TB NVMe SSD | بڑے datasets، تیز لوڈنگ |
| **OS** | Ubuntu 22.04 LTS | Native ROS 2 سپورٹ |

### تخمینی لاگت: $2,500-3,500

### GPU سلیکشن گائیڈ

| GPU | VRAM | Isaac Sim | ٹریننگ | قیمت |
|-----|------|-----------|--------|------|
| RTX 3060 | 12GB | ⚠️ کم از کم | محدود | $300 |
| RTX 4070 | 12GB | ✅ اچھا | اچھا | $550 |
| RTX 4070 Ti Super | 16GB | ✅ بہترین | بہترین | $800 |
| RTX 4080 | 16GB | ✅ عمدہ | عمدہ | $1,000 |
| RTX 4090 | 24GB | ✅ سب سے بہتر | سب سے بہتر | $1,600 |

### ورک سٹیشن سیٹ اپ

```bash
# 1. Ubuntu 22.04 LTS انسٹال کریں
# ڈاؤن لوڈ کریں: https://ubuntu.com/download/desktop

# 2. NVIDIA Drivers انسٹال کریں
sudo apt update
sudo ubuntu-drivers autoinstall
sudo reboot

# تصدیق کریں
nvidia-smi

# 3. CUDA Toolkit انسٹال کریں
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.0-1_all.deb
sudo dpkg -i cuda-keyring_1.0-1_all.deb
sudo apt update
sudo apt install cuda

# 4. ROS 2 Humble انسٹال کریں
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools

# bashrc میں شامل کریں
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 5. Omniverse Launcher کے ذریعے Isaac Sim انسٹال کریں
# ڈاؤن لوڈ کریں: https://www.nvidia.com/omniverse
```

---

## آپشن 2: Jetson Edge Kit

بہترین: Edge deployment، embedded systems، robot integration کے لیے

### Jetson سٹوڈنٹ کٹ (~$700)

| کمپوننٹ | ماڈل | قیمت | مقصد |
|---------|------|------|------|
| **دماغ** | NVIDIA Jetson Orin Nano Super (8GB) | $249 | Edge AI compute |
| **آنکھیں** | Intel RealSense D435i | $349 | RGB-D perception |
| **کان** | ReSpeaker USB Mic Array v2.0 | $69 | آواز کی ان پٹ |
| **Storage** | 128GB High-endurance SD Card | $30 | OS + ایپلیکیشنز |

### اختیاری اضافے

| کمپوننٹ | ماڈل | قیمت | مقصد |
|---------|------|------|------|
| Fan/Heatsink | Noctua 40mm | $20 | ایکٹو کولنگ |
| Power Supply | 65W USB-C PD | $30 | مستحکم پاور |
| USB Hub | Powered USB 3.0 | $25 | متعدد ڈیوائسز |
| Enclosure | 3D پرنٹڈ کیس | $15 | حفاظت |

### Jetson Orin سیٹ اپ

```bash
# 1. JetPack 6.0 فلیش کریں
# ہوسٹ Ubuntu PC پر NVIDIA SDK Manager استعمال کریں
# یا پہلے سے فلیش شدہ SD کارڈ امیج ڈاؤن لوڈ کریں

# 2. پہلی بوٹ
# مانیٹر، کی بورڈ، ماؤس کنیکٹ کریں
# Ubuntu سیٹ اپ وزرڈ مکمل کریں

# 3. Jetson کے لیے ROS 2 Humble انسٹال کریں
sudo apt update && sudo apt upgrade -y
sudo apt install software-properties-common
sudo add-apt-repository universe

# ROS 2 repository شامل کریں
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-ros-base ros-dev-tools

# 4. Isaac ROS انسٹال کریں (Jetson optimized)
mkdir -p ~/workspaces/isaac_ros-dev/src
cd ~/workspaces/isaac_ros-dev/src

# Isaac ROS Common کلون کریں
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common

# Docker کے ساتھ بلڈ کریں
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common
./scripts/run_dev.sh
```

### RealSense D435i سیٹ اپ

```bash
# librealsense انسٹال کریں
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
sudo apt update
sudo apt install librealsense2-dkms librealsense2-utils

# کیمرا ٹیسٹ کریں
realsense-viewer

# ROS 2 wrapper انسٹال کریں
sudo apt install ros-humble-realsense2-camera
```

### ReSpeaker مائیکروفون سیٹ اپ

```bash
# آڈیو ڈرائیورز انسٹال کریں
sudo apt install portaudio19-dev python3-pyaudio

# ReSpeaker USB کے لیے
pip install sounddevice numpy

# مائیکروفون ٹیسٹ کریں
python3 -c "
import sounddevice as sd
duration = 3  # سیکنڈز
fs = 16000
print('ریکارڈنگ...')
audio = sd.rec(int(duration * fs), samplerate=fs, channels=1)
sd.wait()
print('ہو گیا! Shape:', audio.shape)
"
```

---

## آپشن 3: Cloud Development

بہترین: محدود ہارڈویئر، سیکھنے کا مرحلہ، کبھی کبھار استعمال کے لیے

### AWS g5.2xlarge Instance

| وضاحت | قیمت |
|--------|------|
| GPU | NVIDIA A10G (24GB) |
| vCPU | 8 |
| RAM | 32GB |
| لاگت | ~$1.50/گھنٹہ |

### ماہانہ لاگت کا تخمینہ

| استعمال | گھنٹے | لاگت |
|---------|-------|------|
| ہلکا (10 گھنٹے/ہفتہ) | 40 گھنٹے | ~$60 |
| درمیانہ (20 گھنٹے/ہفتہ) | 80 گھنٹے | ~$120 |
| زیادہ (30 گھنٹے/ہفتہ) | 120 گھنٹے | ~$180 |

### AWS سیٹ اپ

```bash
# 1. AWS اکاؤنٹ بنائیں اور IAM کنفیگر کریں

# 2. g5.2xlarge instance لانچ کریں
# AMI: Deep Learning AMI GPU PyTorch (Ubuntu 22.04)
# Region: اپنے قریب ترین منتخب کریں

# 3. SSH کے ذریعے کنیکٹ کریں
ssh -i your-key.pem ubuntu@your-instance-ip

# 4. GPU تصدیق کریں
nvidia-smi

# 5. ROS 2 اور Isaac Sim انسٹال کریں
# اوپر ورک سٹیشن ہدایات کی پیروی کریں
```

### Google Cloud متبادل

```bash
# NVIDIA T4 یا A100 کے ساتھ ملتے جلتے specs
# GPU کے ساتھ Compute Engine استعمال کریں
# لاگت region اور GPU قسم کے مطابق مختلف ہوتی ہے
```

---

## ہائبرڈ اپروچ (تجویز کردہ)

سب سے لاگت موثر سیٹ اپ:

```
┌─────────────────────────────────────────────────────────┐
│              HYBRID DEVELOPMENT SETUP                    │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  ┌─────────────────┐        ┌─────────────────┐        │
│  │  Cloud (AWS)    │   →    │  Jetson Orin    │        │
│  │                 │        │                 │        │
│  │ • ٹریننگ       │        │ • Deployment    │        │
│  │ • Isaac Sim     │        │ • حقیقی سینسرز │        │
│  │ • بھاری compute │        │ • روبوٹ کنٹرول │        │
│  │                 │        │                 │        │
│  │ ~$100/ماہ       │        │ $700 ایک بار   │        │
│  └─────────────────┘        └─────────────────┘        │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

**ورک فلو:**
1. اپنے لیپ ٹاپ/ڈیسک ٹاپ پر **ڈیولپ** کریں
2. کلاؤڈ میں Isaac Sim کے ساتھ **سمیولیٹ** کریں
3. کلاؤڈ GPUs پر AI ماڈلز **ٹرین** کریں
4. Jetson پر بہتر بنائے گئے ماڈلز **ڈیپلائے** کریں
5. Jetson + سینسرز پر حقیقی دنیا کے ٹیسٹ **چلائیں**

---

## سافٹ ویئر انسٹالیشن چیک لسٹ

### ضروری سافٹ ویئر

```bash
# ✅ Python Environment
sudo apt install python3-pip python3-venv
pip install numpy scipy matplotlib

# ✅ ROS 2 Humble
# اوپر انسٹالیشن ہدایات دیکھیں

# ✅ عام ROS 2 Packages
sudo apt install \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-rviz2

# ✅ Development Tools
sudo apt install \
  git \
  cmake \
  build-essential \
  python3-colcon-common-extensions

# ✅ AI/ML Libraries
pip install \
  torch torchvision \
  transformers \
  openai-whisper \
  google-generativeai \
  opencv-python

# ✅ Isaac ROS (Jetson/RTX کے لیے)
# NVIDIA Isaac ROS documentation کی پیروی کریں
```

### انسٹالیشن تصدیق

```bash
# ROS 2 چیک کریں
ros2 --version

# GPU چیک کریں
nvidia-smi

# Python packages چیک کریں
python3 -c "import torch; print(f'PyTorch: {torch.__version__}, CUDA: {torch.cuda.is_available()}')"

# RealSense چیک کریں (اگر انسٹال ہے)
rs-enumerate-devices

# ROS 2 demo چلائیں
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_py listener
```

---

## ٹربل شوٹنگ

### عام مسائل

**مسئلہ: CUDA نہیں ملا**
```bash
# ~/.bashrc میں شامل کریں
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```

**مسئلہ: ROS 2 commands نہیں مل رہے**
```bash
# ROS 2 سیٹ اپ source کریں
source /opt/ros/humble/setup.bash
# مستقل بنانے کے لیے ~/.bashrc میں شامل کریں
```

**مسئلہ: RealSense permission denied**
```bash
# udev rules شامل کریں
sudo cp /lib/udev/rules.d/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**مسئلہ: Jetson میموری ختم**
```bash
# swap file بنائیں
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile swap swap defaults 0 0' | sudo tee -a /etc/fstab
```

**مسئلہ: Isaac Sim شروع نہیں ہو رہا**
```bash
# GPU میموری چیک کریں
nvidia-smi

# دوسری GPU ایپلیکیشنز بند کریں
# ورچوئل میموری بڑھائیں
# compatibility کے لیے --vulkan فلیگ استعمال کریں
```

---

## اگلے اقدامات

ایک بار آپ کا ہارڈویئر سیٹ اپ ہو جائے:

1. 📚 [ماڈیول 1: ROS 2 بنیادیات](./module1-ros2) سے شروع کریں
2. 🤖 اپنا پہلا robot URDF بنائیں
3. 🎮 Gazebo میں simulation لانچ کریں
4. 🧠 Isaac ROS کے ساتھ AI perception شامل کریں

---

**واپس**: [کورس تعارف →](./intro)
