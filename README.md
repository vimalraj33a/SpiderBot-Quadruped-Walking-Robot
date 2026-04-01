# 🕷️ SpiderBot — Quadruped Walking Robot

<div align="center">

![SpiderBot](https://img.shields.io/badge/Robot-Quadruped-blue?style=for-the-badge)
![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?style=for-the-badge&logo=ros)
![ESP32](https://img.shields.io/badge/ESP32-Firmware-E7352C?style=for-the-badge&logo=espressif)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)

**A low-cost 4-legged walking robot that mirrors Gazebo simulation movements to real hardware in real time.**

*ROS2 Humble · Gazebo Classic · MoveIt2 · ESP32 · PCA9685 · LM2596 · 8× Servo*

</div>

---

## 📸 Demo

> Gazebo simulation and real robot moving together simultaneously

| Gazebo Simulation | Real Robot |
|:---:|:---:|
| ![Gazebo](docs/gazebo_demo.gif) | ![Real](docs/real_robot.gif) |

---

## 🧭 Table of Contents

- [Overview](#-overview)
- [System Architecture](#-system-architecture)
- [Hardware Components](#-hardware-components)
- [Pin Connection Table](#-pin-connection-table)
- [Software Stack](#-software-stack)
- [Workspace Structure](#-workspace-structure)
- [Setup & Installation](#-setup--installation)
- [Usage](#-usage)
- [Control Modes](#-control-modes)
- [Gait & Motion](#-gait--motion)
- [Troubleshooting](#-troubleshooting)
- [Future Work](#-future-work)
- [License](#-license)

---

## 🤖 Overview

SpiderBot is a fully open-source quadruped (4-legged) walking robot built with:

- **3D-printed body** — 110mm × 110mm × 28mm chassis, 4 legs each with coxa (60mm) + tibia (110mm)
- **8 servo motors** — 2 DOF per leg (shoulder yaw + knee pitch)
- **ROS2 Humble** — full robot middleware stack
- **Gazebo Classic** — physics simulation with `gazebo_ros2_control`
- **MoveIt2** — motion planning and named pose execution
- **ESP32 firmware** — receives joint angles over USB serial, drives servos via PCA9685

The key feature is the **hardware bridge** — a ROS2 node that subscribes to `/joint_states` published by Gazebo and forwards all 8 joint angles to the real ESP32 hardware at 20Hz over USB serial. Whatever moves in simulation moves on the real robot simultaneously.

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────┐
│              Ubuntu 22.04 PC                │
│                                             │
│  Gazebo  ──►  /joint_states  ──►  hardware  │
│  MoveIt       (ROS2 topic)       _bridge.py │
│  moveit_                              │      │
│  control.py                          │      │
│  walk.py                             │      │
│  keyboard_                           │      │
│  teleop.py                           │      │
└──────────────────────────────────────┼──────┘
                                       │ USB Serial
                                       │ 115200 baud
                                       │ CSV: fr1,fr2,fl1,fl2,
                                       │      rr1,rr2,rl1,rl2
                                       ▼
                              ┌────────────────┐
                              │  ESP32 DevKit  │
                              │  spider_robot  │
                              │  _esp32.ino    │
                              └───────┬────────┘
                                      │ I²C (GPIO21=SDA, GPIO22=SCL)
                                      ▼
                              ┌────────────────┐
                              │   PCA9685      │
                              │  PWM Driver    │
                              │  Addr: 0x40    │
                              └───────┬────────┘
                                      │ PWM 50Hz  CH0–CH7
                          ┌───────────┼───────────┐
                          ▼           ▼           ▼
                      FR Leg      FL Leg      RR+RL Leg
                    (CH0,CH1)  (CH2,CH3)   (CH4–CH7)
```

---

## 🔧 Hardware Components

| Component | Model | Purpose |
|---|---|---|
| Microcontroller | ESP32 DOIT DevKit V1 | Main controller, serial bridge |
| PWM Driver | PCA9685 16-Ch | Drives 8 servos over I²C |
| Servo Motors | SG90 / MG996R × 8 | Leg joints (2 per leg) |
| Voltage Regulator | LM2596 Step-Down | 7.4V → 5V for servos |
| Battery | LiPo 2S 7.4V 1500mAh | Main power source |
| Body | 3D Printed (PLA) | Chassis + leg segments |
| PC | Ubuntu 22.04 | ROS2 + Gazebo host |

---

## 📌 Pin Connection Table

| From | Pin | To | Pin | Wire | Purpose |
|---|---|---|---|---|---|
| LiPo | + (7.4V) | LM2596 | IN+ | Red | Raw power |
| LiPo | − (GND) | LM2596 | IN− | Black | GND |
| LM2596 | OUT+ (5V) | ESP32 | VIN | Red | ESP32 power |
| LM2596 | OUT+ (5V) | PCA9685 | V+ | Orange | Servo rail |
| LM2596 | OUT− | Common GND | — | Black | GND bus |
| ESP32 | GPIO21 | PCA9685 | SDA | Blue | I²C data |
| ESP32 | GPIO22 | PCA9685 | SCL | Blue | I²C clock |
| ESP32 | 3.3V | PCA9685 | VCC | Red | Logic power |
| ESP32 | GND | PCA9685 | GND | Black | GND |
| ESP32 | USB | Ubuntu PC | USB | Purple | ROS2 serial |
| PCA9685 | CH0 | fr1 Servo | Signal | Yellow | FR shoulder yaw |
| PCA9685 | CH1 | fr2 Servo | Signal | Yellow | FR knee pitch |
| PCA9685 | CH2 | fl1 Servo | Signal | Yellow | FL shoulder yaw |
| PCA9685 | CH3 | fl2 Servo | Signal | Yellow | FL knee pitch |
| PCA9685 | CH4 | rr1 Servo | Signal | Yellow | RR shoulder yaw |
| PCA9685 | CH5 | rr2 Servo | Signal | Yellow | RR knee pitch |
| PCA9685 | CH6 | rl1 Servo | Signal | Yellow | RL shoulder yaw |
| PCA9685 | CH7 | rl2 Servo | Signal | Yellow | RL knee pitch |
| PCA9685 | V+ rail | All servos | VCC red | Red | Servo 5V |
| Common GND | — | All servos | GND brown | Black | Servo GND |
| PCA9685 | A0–A5 | GND | — | Black | I²C addr 0x40 |

> ⚠️ **Critical:** All GNDs (LiPo, LM2596, ESP32, PCA9685, all 8 servos) MUST share one common ground.

> ⚠️ **Set LM2596 to exactly 5.0V** with a multimeter before connecting any components.

---

## 💻 Software Stack

| Layer | Technology |
|---|---|
| OS | Ubuntu 22.04 LTS |
| Middleware | ROS2 Humble |
| Simulation | Gazebo Classic 11 |
| Motion Planning | MoveIt2 |
| Robot Description | URDF / xacro |
| Control | ros2_control · joint_trajectory_controller |
| Hardware Bridge | Python 3 · pyserial |
| ESP32 Firmware | Arduino IDE 2 · Adafruit PWM Servo Driver |
| Build System | colcon |

---

## 📁 Workspace Structure

```
spiderrobot_ws/
├── src/
│   ├── spiderbot/                    ← Robot description package
│   │   ├── description/
│   │   │   ├── robot.urdf.xacro
│   │   │   └── robot_core.xacro     ← URDF with 8 joints
│   │   ├── config/
│   │   │   ├── controllers.yaml     ← PID gains, joint list
│   │   │   └── moveit/
│   │   │       ├── spiderbot.srdf   ← Planning groups
│   │   │       ├── kinematics.yaml
│   │   │       └── joint_limits.yaml
│   │   ├── launch/
│   │   │   ├── launch_sim.launch.py ← Gazebo simulation
│   │   │   └── moveit.launch.py     ← MoveIt + RViz
│   │   ├── src/
│   │   │   ├── moveit_control.py    ← Named pose controller
│   │   │   ├── walk.py              ← Trot gait loop
│   │   │   ├── keyboard_teleop.py   ← WASD keyboard control
│   │   │   ├── manual_control.py    ← GUI sliders / CLI
│   │   │   └── stabilize.py        ← Startup stabilizer
│   │   └── worlds/
│   │       └── spiderbot.world
│   │
│   └── spider_robot/                ← Hardware bridge package
│       ├── src/
│       │   └── hardware_bridge.py   ← /joint_states → USB serial
│       ├── firmware/
│       │   └── spider_robot_esp32.ino ← Flash to ESP32
│       ├── launch/
│       │   ├── hardware.launch.py   ← Gazebo + real robot
│       │   ├── real_robot.launch.py ← Real robot only
│       │   └── moveit_hardware.launch.py
│       └── config/
│           └── hardware_bridge.yaml
└── setup.sh                         ← One-shot setup script
```

---

## ⚙️ Setup & Installation

### 1. Prerequisites

```bash
# ROS2 Humble
sudo apt install ros-humble-desktop ros-dev-tools -y

# Dependencies
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-moveit \
  ros-humble-xacro

pip3 install pyserial --user
```

### 2. Clone and build

```bash
mkdir -p ~/spiderrobot_ws/src
cd ~/spiderrobot_ws/src

# Clone this repo
git clone https://github.com/yourusername/spiderbot.git

cd ~/spiderrobot_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Permanent source (add to ~/.bashrc)

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/spiderrobot_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4. Flash ESP32 firmware

1. Open `src/spider_robot/firmware/spider_robot_esp32.ino` in Arduino IDE 2
2. Install library: **Adafruit PWM Servo Driver** (Tools → Manage Libraries)
3. Board: `DOIT ESP32 DEVKIT V1` · Port: `/dev/ttyUSB0`
4. Click **Upload**
5. Open Serial Monitor at 115200 — should show:
   ```
   SpiderBot ESP32 ready. Waiting for joint commands...
   Standing. Ready.
   ```

### 5. USB permission

```bash
sudo usermod -aG dialout $USER
newgrp dialout
```

---

## 🚀 Usage

### Simulation only (no hardware)

```bash
ros2 launch spiderbot launch_sim.launch.py
```

### Simulation + Real robot (both move together)

```bash
# Terminal 1
ros2 launch spider_robot hardware.launch.py port:=/dev/ttyUSB0

# Terminal 2
cd ~/spiderrobot_ws/src/spiderbot/src
python3 moveit_control.py
```

### Real robot only (no Gazebo)

```bash
# Terminal 1
ros2 launch spider_robot real_robot.launch.py port:=/dev/ttyUSB0

# Terminal 2
python3 moveit_control.py
```

### Dry run (test without hardware)

```bash
ros2 launch spider_robot hardware.launch.py dry_run:=true
```

---

## 🎮 Control Modes

### moveit_control.py — Named poses

```bash
python3 moveit_control.py
```

| Command | Action |
|---|---|
| `stand` | All legs at standing pose |
| `sit` | Robot crouches low |
| `stretch` | Legs spread outward |
| `wave_fr` | Front-right leg raised |
| `wave_fl` | Front-left leg raised |
| `wave_rr` | Rear-right leg raised |
| `wave_rl` | Rear-left leg raised |
| `demo` | Full auto demo sequence |
| `j 0 17 0 17 0 17 0 17` | Raw joint angles in degrees |
| `q` | Quit |

### keyboard_teleop.py — Real-time keyboard

```bash
python3 keyboard_teleop.py
```

| Key | Action |
|---|---|
| `W` | Walk forward |
| `S` | Walk backward |
| `A` | Turn left |
| `D` | Turn right |
| `Space` | Stand |
| `Z` | Sit |
| `X` | Stretch |
| `1/2/3/4` | Select leg FR/FL/RR/RL |
| `↑ ↓` | Tibia pitch (selected leg) |
| `← →` | Coxa yaw (selected leg) |
| `Q` | Quit |

### manual_control.py — GUI sliders

```bash
python3 manual_control.py          # GUI sliders
python3 manual_control.py keyboard # keyboard mode
python3 manual_control.py cli      # command line mode
```

### walk.py — Auto trot gait loop

```bash
python3 walk.py
```
Runs continuous diagonal trot gait (FR+RL swing, FL+RR swing alternating).

---

## 🦿 Gait & Motion

The robot uses a **diagonal trot gait** — two diagonal legs swing simultaneously:

```
SWING_A: FR + RL lift and swing forward
PUSH_A : FR + RL push back (propulsion)
SWING_B: FL + RR lift and swing forward
PUSH_B : FL + RR push back (propulsion)
```

Each step cycle: `SWING_A (0.5s) → PUSH_A (0.5s) → SWING_B (0.5s) → PUSH_B (0.5s)`

Joint convention:
- `*1_joint` = coxa (shoulder yaw, Z-axis) range ±90°
- `*2_joint` = tibia (knee pitch, X-axis) range ±90°
- Standing pose: `*1 = 0.0 rad`, `*2 = 0.3 rad (≈17°)`

---

## 🔍 Troubleshooting

| Problem | Fix |
|---|---|
| `Package 'spiderbot' not found` | `source ~/spiderrobot_ws/install/setup.bash` |
| `Cannot open serial /dev/ttyUSB0` | `sudo chmod 666 /dev/ttyUSB0` |
| `Device or resource busy` | `sudo pkill minicom; sudo fuser -k /dev/ttyUSB0` |
| `No module named serial` | `pip3 install pyserial --user` |
| `Goal REJECTED` | `ros2 control list_controllers` — both must show `active` |
| Servos not moving | Check LM2596 output = 5.0V; check common GND |
| Garbled serial output | ESP32 not flashed — upload `spider_robot_esp32.ino` |
| ESP32 not found | Try `/dev/ttyACM0` instead of `/dev/ttyUSB0` |
| Robot moves wrong direction | Set `invert_fr1:=true` etc. in launch args |
| I²C scan finds nothing | Check GPIO21=SDA, GPIO22=SCL; ESP32 3.3V → PCA9685 VCC |

---

## 🔮 Future Work

- [ ] IMU (MPU6050) for real-time balance and stability control
- [ ] Camera module for visual obstacle detection
- [ ] ROS2 Nav2 integration for autonomous navigation
- [ ] Reinforcement learning for adaptive gait
- [ ] 3-DOF legs (12 joints total) for smoother motion
- [ ] Wireless control via WiFi (micro-ROS UDP)
- [ ] Battery level monitoring and low-power mode

---

## 👨‍💻 Author

**Vimalraj**
Robotics Engineering Student

[![LinkedIn](https://img.shields.io/badge/LinkedIn-Connect-0077B5?style=flat&logo=linkedin)](https://linkedin.com/in/yourprofile)
[![GitHub](https://img.shields.io/badge/GitHub-Follow-181717?style=flat&logo=github)](https://github.com/yourusername)

---

## 📄 License

This project is licensed under the MIT License — see [LICENSE](LICENSE) for details.

---

<div align="center">

**If this project helped you, please ⭐ star the repository!**

*Built with ❤️ using ROS2, Gazebo, MoveIt2, and ESP32*

</div>
