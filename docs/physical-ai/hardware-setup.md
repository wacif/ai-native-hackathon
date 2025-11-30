---
sidebar_position: 7
---

# Hardware Setup Guide

> **Setting up your Physical AI development environment**

This guide covers the hardware requirements and setup procedures for the Physical AI course. You have three main options based on your budget and needs.

## Option 1: High-Performance Workstation

Best for: Full Isaac Sim experience, local training, professional development

### Recommended Specifications

| Component | Specification | Why |
|-----------|--------------|-----|
| **GPU** | NVIDIA RTX 4070 Ti Super (16GB) | Isaac Sim requires RTX, 12GB+ VRAM |
| **CPU** | Intel i7-14700K / AMD Ryzen 9 7900X | Physics simulation is CPU-intensive |
| **RAM** | 64GB DDR5 | Multiple simulators + AI models |
| **Storage** | 1TB NVMe SSD | Large datasets, fast loading |
| **OS** | Ubuntu 22.04 LTS | Native ROS 2 support |

### Estimated Cost: $2,500-3,500

### GPU Selection Guide

| GPU | VRAM | Isaac Sim | Training | Price |
|-----|------|-----------|----------|-------|
| RTX 3060 | 12GB | ⚠️ Minimum | Limited | $300 |
| RTX 4070 | 12GB | ✅ Good | Good | $550 |
| RTX 4070 Ti Super | 16GB | ✅ Great | Great | $800 |
| RTX 4080 | 16GB | ✅ Excellent | Excellent | $1,000 |
| RTX 4090 | 24GB | ✅ Best | Best | $1,600 |

### Workstation Setup

```bash
# 1. Install Ubuntu 22.04 LTS
# Download from: https://ubuntu.com/download/desktop

# 2. Install NVIDIA Drivers
sudo apt update
sudo ubuntu-drivers autoinstall
sudo reboot

# Verify
nvidia-smi

# 3. Install CUDA Toolkit
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.0-1_all.deb
sudo dpkg -i cuda-keyring_1.0-1_all.deb
sudo apt update
sudo apt install cuda

# 4. Install ROS 2 Humble
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools

# Add to bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 5. Install Isaac Sim via Omniverse Launcher
# Download from: https://www.nvidia.com/omniverse
```

---

## Option 2: Jetson Edge Kit

Best for: Edge deployment, embedded systems, robot integration

### The Jetson Student Kit (~$700)

| Component | Model | Price | Purpose |
|-----------|-------|-------|---------|
| **Brain** | NVIDIA Jetson Orin Nano Super (8GB) | $249 | Edge AI compute |
| **Eyes** | Intel RealSense D435i | $349 | RGB-D perception |
| **Ears** | ReSpeaker USB Mic Array v2.0 | $69 | Voice input |
| **Storage** | 128GB High-endurance SD Card | $30 | OS + applications |

### Optional Additions

| Component | Model | Price | Purpose |
|-----------|-------|-------|---------|
| Fan/Heatsink | Noctua 40mm | $20 | Active cooling |
| Power Supply | 65W USB-C PD | $30 | Stable power |
| USB Hub | Powered USB 3.0 | $25 | Multiple devices |
| Enclosure | 3D printed case | $15 | Protection |

### Jetson Orin Setup

```bash
# 1. Flash JetPack 6.0
# Use NVIDIA SDK Manager on a host Ubuntu PC
# Or download pre-flashed SD card image

# 2. Initial Boot
# Connect monitor, keyboard, mouse
# Complete Ubuntu setup wizard

# 3. Install ROS 2 Humble for Jetson
sudo apt update && sudo apt upgrade -y
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-ros-base ros-dev-tools

# 4. Install Isaac ROS (Jetson optimized)
mkdir -p ~/workspaces/isaac_ros-dev/src
cd ~/workspaces/isaac_ros-dev/src

# Clone Isaac ROS Common
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common

# Build with Docker
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common
./scripts/run_dev.sh
```

### RealSense D435i Setup

```bash
# Install librealsense
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
sudo apt update
sudo apt install librealsense2-dkms librealsense2-utils

# Test camera
realsense-viewer

# Install ROS 2 wrapper
sudo apt install ros-humble-realsense2-camera
```

### ReSpeaker Microphone Setup

```bash
# Install audio drivers
sudo apt install portaudio19-dev python3-pyaudio

# For ReSpeaker USB
pip install sounddevice numpy

# Test microphone
python3 -c "
import sounddevice as sd
duration = 3  # seconds
fs = 16000
print('Recording...')
audio = sd.rec(int(duration * fs), samplerate=fs, channels=1)
sd.wait()
print('Done! Shape:', audio.shape)
"
```

---

## Option 3: Cloud Development

Best for: Limited hardware, learning phase, occasional use

### AWS g5.2xlarge Instance

| Spec | Value |
|------|-------|
| GPU | NVIDIA A10G (24GB) |
| vCPU | 8 |
| RAM | 32GB |
| Cost | ~$1.50/hour |

### Monthly Cost Estimate

| Usage | Hours | Cost |
|-------|-------|------|
| Light (10 hrs/week) | 40 hrs | ~$60 |
| Medium (20 hrs/week) | 80 hrs | ~$120 |
| Heavy (30 hrs/week) | 120 hrs | ~$180 |

### AWS Setup

```bash
# 1. Create AWS Account and configure IAM

# 2. Launch g5.2xlarge instance
# AMI: Deep Learning AMI GPU PyTorch (Ubuntu 22.04)
# Region: Choose closest to you

# 3. Connect via SSH
ssh -i your-key.pem ubuntu@your-instance-ip

# 4. Verify GPU
nvidia-smi

# 5. Install ROS 2 and Isaac Sim
# Follow workstation instructions above
```

### Google Cloud Alternative

```bash
# Similar specs with NVIDIA T4 or A100
# Use Compute Engine with GPU
# Cost varies by region and GPU type
```

---

## Hybrid Approach (Recommended)

The most cost-effective setup:

```
┌─────────────────────────────────────────────────────────┐
│              HYBRID DEVELOPMENT SETUP                    │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  ┌─────────────────┐        ┌─────────────────┐        │
│  │  Cloud (AWS)    │   →    │  Jetson Orin    │        │
│  │                 │        │                 │        │
│  │ • Training      │        │ • Deployment    │        │
│  │ • Isaac Sim     │        │ • Real sensors  │        │
│  │ • Heavy compute │        │ • Robot control │        │
│  │                 │        │                 │        │
│  │ ~$100/month     │        │ $700 one-time   │        │
│  └─────────────────┘        └─────────────────┘        │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

**Workflow:**
1. **Develop** code on your laptop/desktop
2. **Simulate** in cloud with Isaac Sim
3. **Train** AI models on cloud GPUs
4. **Deploy** optimized models to Jetson
5. **Run** real-world tests on Jetson + sensors

---

## Software Installation Checklist

### Essential Software

```bash
# ✅ Python Environment
sudo apt install python3-pip python3-venv
pip install numpy scipy matplotlib

# ✅ ROS 2 Humble
# See installation instructions above

# ✅ Common ROS 2 Packages
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

# ✅ Isaac ROS (for Jetson/RTX)
# Follow NVIDIA Isaac ROS documentation
```

### Verify Installation

```bash
# Check ROS 2
ros2 --version

# Check GPU
nvidia-smi

# Check Python packages
python3 -c "import torch; print(f'PyTorch: {torch.__version__}, CUDA: {torch.cuda.is_available()}')"

# Check RealSense (if installed)
rs-enumerate-devices

# Run ROS 2 demo
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_py listener
```

---

## Troubleshooting

### Common Issues

**Issue: CUDA not found**
```bash
# Add to ~/.bashrc
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```

**Issue: ROS 2 commands not found**
```bash
# Source ROS 2 setup
source /opt/ros/humble/setup.bash
# Add to ~/.bashrc to make permanent
```

**Issue: RealSense permission denied**
```bash
# Add udev rules
sudo cp /lib/udev/rules.d/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**Issue: Jetson out of memory**
```bash
# Create swap file
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile swap swap defaults 0 0' | sudo tee -a /etc/fstab
```

**Issue: Isaac Sim won't start**
```bash
# Check GPU memory
nvidia-smi

# Close other GPU applications
# Increase virtual memory
# Use --vulkan flag for compatibility
```

---

## Next Steps

Once your hardware is set up:

1. 📚 Start with [Module 1: ROS 2 Fundamentals](./module1-ros2)
2. 🤖 Build your first robot URDF
3. 🎮 Launch simulation in Gazebo
4. 🧠 Add AI perception with Isaac ROS

---

**Back to**: [Course Introduction →](./intro)
