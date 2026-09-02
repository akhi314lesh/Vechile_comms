# END-OF-LINE ERROR — V2V Vehicle Communication & Autonomous Driving Simulation

[![JavaScript](https://img.shields.io/badge/JavaScript-ES6+-F7DF1E?logo=javascript&logoColor=black)](https://developer.mozilla.org/en-US/docs/Web/JavaScript)
[![Three.js](https://img.shields.io/badge/Three.js-r128-black?logo=three.js)](https://threejs.org/)
[![Spline 3D](https://img.shields.io/badge/Spline-3D%20Hero-FF2D55?logo=spline&logoColor=white)](https://spline.design/)
[![Python](https://img.shields.io/badge/Python-3.9+-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![PPO RL](https://img.shields.io/badge/RL-PPO%20PyTorch-EE4C2C?logo=pytorch&logoColor=white)](https://pytorch.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-00E5FF.svg)](LICENSE)

An advanced real-time Vehicle-to-Vehicle (V2V) cooperative perception and autonomous safety driving simulation platform. The system synthesizes local line-of-sight sensor raycasting with omnidirectional V2V Basic Safety Message (BSM) radio exchange to build a unified perception model, evaluate collision risk via authoritative Time-To-Collision (TTC) dynamics, and execute autonomous safety arbitration (AEB, curve governing, cooperative lane changing).

---

## 👥 Project Researchers — Team End-of-Line Error

| Researcher | Registration No. | Role / Focus |
| :--- | :--- | :--- |
| **Akhilesh** | `25BCE5300` | Autonomous Control Architecture & Risk Arbitration |
| **Vedant** | `25BCE5232` | Sensor Raycasting, Perception Fusion & RL Bridge |
| **Rajarshi** | `25BCE1931` | 3D Graphics Simulation, Tactical Radar & UI/UX |

---

## 🌟 Key Features

### 1. Cooperative Perception & Multi-Source Fusion
- **Local Sensor Suite**: 250 forward and peripheral lidar/radar raycasts performing real-time obstacle boundary detection.
- **V2V Radio Communication**: Omnidirectional Basic Safety Message (BSM) telemetry broadcast (position, velocity, acceleration, heading, hazard status) overcoming occlusion and visual blind spots.
- **Track Clustering & Deduplication**: Spatial clustering engine fusing local detections and V2V telemetry into a single authoritative track list, eliminating ghosting and scatter artifacts.

### 2. Risk-Triggered Safety & Control Arbitration
- **Authoritative TTC Calculation**: Exact closing-rate physics evaluating deterministic Time-To-Collision ($\text{TTC} = \frac{d_{\text{rel}}}{-\dot{d}_{\text{rel}}}$).
- **Hysteresis Risk State Machine**: Three-tiered risk controller (`NORMAL` $\to$ `CAUTION` $\to$ `CRITICAL`) with cooldown timers to prevent control hunting and chatter.
- **Autonomous Emergency Braking (AEB)**: Dynamic deceleration profile matching required braking distance to maintain safe stopping margins.
- **Curve Speed Governor & Lane Adherence**: Predictive yaw-rate limiting and lane-centering control for curved multi-lane highway geometries.

### 3. Modular Tactical Radar & Multi-View Rendering
- **2D Tactical Perception Radar**: Left-docked 80m ego-centered radar displaying source-classified targets:
  - Cyan Square (`□`): **LOCAL** line-of-sight ray perception
  - Orange Circle (`○`): **V2V** radio BSM perception
  - Emerald Diamond (`◇`): **FUSED** verified multi-source perception
- **3D Top-Down Minimap**: Orthographic global overhead view displaying dynamic traffic flows.
- **Atmospheric Time-of-Day System**: Real-time lighting engine supporting `NIGHT`, `DAWN`, and `DAY` atmospheric presets with horizon-matched fog density and automated street/building emissives.

### 4. Reinforcement Learning Integration (PPO)
- **Gym-Compatible Environment Wrapper** (`rl/gym_env.py` & `js/rl-env.js`): Continuous observation and action space representations for lane-keeping and collision avoidance policies.
- **Asynchronous WebSocket Bridge** (`rl/ws_bridge.py`): High-throughput communication between Python PyTorch training scripts and browser physics runtime.
- **Proximal Policy Optimization** (`rl/ppo.py`): Actor-Critic network training for autonomous driving policies.

---

## 📁 Repository Structure

```text
.
├── index.html              # 3D Spline landing portal & project overview
├── sim.html                # Full 3D/2D V2V Scenario Lab autonomous simulation
├── style.css               # Cyber-glassmorphism design system & styles
├── assets/                 # Textures, models, and team photography
│   └── team/               # Researcher portraits (akhilesh.jpg, vedant.jpg, rajarshi.jpg)
├── js/                     # Core simulation logic modules
│   ├── aeb.js              # Autonomous emergency braking algorithms
│   ├── behavior.js         # Traffic NPC vehicle behavior trees
│   ├── perception.js       # Sensor raycasting & BSM radio fusion
│   ├── physics.js          # Vehicle dynamics & collision mathematics
│   ├── rl-env.js           # Browser-side RL observation & step handler
│   ├── utils.js            # Shared math, seeded RNG & data structures
│   └── v2v.js              # V2V protocol & message broadcast system
├── rl/                     # Reinforcement learning pipeline
│   ├── ppo.py              # PyTorch PPO Actor-Critic implementation
│   ├── gym_env.py          # OpenAI Gym environment interface
│   ├── ws_bridge.py        # Async WebSocket simulation bridge
│   └── lka_test.py         # Lane-keeping assistance policy testing
└── styles/                 # Theme and component stylesheets
    └── style.css           # Modular stylesheet mirror
```

---

## 🎮 Controls & Simulation Interaction

| Key / Control | Function | Description |
| :---: | :---: | :--- |
| <kbd>W</kbd> / <kbd>S</kbd> | **Throttle / Brake** | Manual longitudinal acceleration and braking (active in all modes including V2V Assistant) |
| <kbd>A</kbd> / <kbd>D</kbd> | **Steering** | Responsive lateral steering with auto-centering and co-pilot lane keeping |
| <kbd>C</kbd> | **Camera POV** | Cycle through Chase, Cockpit, Bumper, and Top-Down cameras |
| <kbd>V</kbd> | **Drive Mode** | Toggle between Driving, Perception, V2V Assistant, and Debug modes |
| <kbd>B</kbd> | **Hazard Beacon** | Toggle emergency hazard lights and broadcast V2V warning |
| <kbd>H</kbd> | **Headlights** | Toggle high/low-beam dynamic headlights |
| <kbd>R</kbd> | **Reset Scenario** | Reset ego vehicle position and clear traffic state |
| <kbd>E</kbd> | **Track Editor** | Open procedural roadway and scenario customizer |

---

## 🚀 Quick Start Guide

### Running the Web Simulation

1. Clone this repository:
   ```bash
   git clone https://github.com/akhi314lesh/Vechile_comms.git
   cd Vechile_comms
   ```

2. Open the project using any local HTTP server (such as VS Code Live Server, Node `http-server`, or Python's built-in server):
   ```bash
   # Using Python
   python -m http.server 8000
   ```

3. Open your browser and navigate to:
   - **Landing Page**: `http://localhost:8000/index.html`
   - **Simulation**: `http://localhost:8000/sim.html`

---

### Running the Reinforcement Learning Environment

1. Install Python dependencies:
   ```bash
   pip install torch numpy websockets gymnasium
   ```

2. Start the WebSocket bridge server:
   ```bash
   python rl/ws_bridge.py
   ```

3. Launch the simulation in your browser and connect to WebSocket mode from the Mission Console.

4. Train or evaluate the PPO agent:
   ```bash
   python rl/ppo.py --train
   # or test evaluation
   python rl/lka_test.py
   ```

---

## 📜 License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.
