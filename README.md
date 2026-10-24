# Autonomous Voice Drive

A ROS2 prototype for controlling an autonomous vehicle with **Turkish voice commands**, running **fully offline**.
Speech recognition is done with Whisper (faster-whisper); simulation uses Gazebo + TurtleBot3.

## Architecture

```
Microphone ──► voice_node ──/voice/command──► motion_node ──/cmd_vel──► TurtleBot3 (Gazebo)
              (Whisper + command matching)    (motion + safety)
```

- **voice_node**: Records with push-to-talk (Enter), transcribes with Whisper, and maps the text to a command.
  If no command matches, **nothing is sent to the vehicle**.
- **motion_node**: Converts commands into velocity messages. Publishes continuously at 10 Hz, runs a watchdog, and stops the vehicle on shutdown.

Splitting the system into two nodes keeps speech recognition (~1 s per utterance) from blocking the motion control loop.

## Commands

| Command | Example utterance (Turkish) | Behavior |
|---|---|---|
| FORWARD | "ileri git", "ilerle" (go forward) | Move forward at 0.2 m/s |
| BACKWARD | "geri git", "geriye gidin" (go back) | Move backward at 0.2 m/s |
| LEFT / RIGHT | "sola dön", "sağa dön" (turn left / right) | Turn ~90° in place |
| STOP | "dur", "stop" | Stop immediately |
| GOTO | "atölyeye git", "kırmızıya git", "merkeze dön" (go to the workshop / red / back to base) | Navigate to a named location |

Named locations are defined per world in `worlds/demo_locations.yaml` (name → coordinates + spoken aliases).
Turkish suffixes are handled automatically ("depo" → "depoya", "atölye" → "atölyesine").
A movement verb is required, so "atölye nerede?" (*where is the workshop?*) does not move the vehicle.

## Safety rules

- **STOP has priority**: If "dur" appears anywhere in the sentence, the vehicle stops.
- **Negation**: "Sakın ileri gitme" (*don't go forward*) does not move the vehicle.
- **Ambiguity rejection**: A sentence containing more than one different command is ignored.
- **Watchdog**: The vehicle stops if no new command arrives for 15 s (GOTO uses a distance-based timeout instead).
- **Lidar obstacle stop**: The vehicle stops (and refuses new commands) if an obstacle is within 0.5 m in its direction of travel.
- **Fail-safe**: No lidar data for 1 s, or no odometry during GOTO, means no motion.
- **Offline**: No internet connection required.

## Installation (Ubuntu 22.04 + ROS2 Humble)

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-turtlebot3 ros-humble-turtlebot3-simulations libportaudio2
pip install -r requirements.txt
export TURTLEBOT3_MODEL=burger
```

## Usage

```bash
# Terminal 1: Gazebo demo scene + TurtleBot3 + motion_node
ros2 launch launch/demo.launch.py
# Terminal 2: voice control (push-to-talk needs keyboard input, so it runs separately)
python3 voice_node.py
```

The demo scene (`worlds/demo.world`) places a colored marker 4 m away in each direction
(front: red = workshop, left: blue = depot, right: green = hangar, back: yellow = gate) plus orange barrels as obstacles.
A barrel deliberately blocks the straight path to the hangar: the straight-line GOTO controller stops in front of it,
which demonstrates the safety layer and motivates map-based navigation (Nav2) as the next step.

## Evaluation

```bash
python3 collect_test.py --speaker <name> --noise <quiet|music|...>   # guided recording session
python3 report.py                                                     # accuracy, false actions, missed STOPs, latency
```

`whisper_test.py`: Tests speech recognition and command matching without ROS.

## Crane simulation (`crane/`)

A truck-mounted telescopic boom crane (MPG style) on a work site with four areas
(stock, depot, workshop, shipping), four loads and a workshop building.

- **Joints**: slew (360°), luff (0–75°), telescope (7.6–13.6 m), hoist (0–12 m). The sheave counter-rotates
  with the boom so the rope always hangs vertically.
- **Kinematics** (`crane_geometry.py`): closed-form IK puts the boom tip above any target; joints move
  together with per-joint speed limits.
- **Single source of truth**: `gen_world.py` builds `worlds/crane_site.world` from `crane_geometry.py` + `sites.yaml`,
  and the controller uses the same files.

```bash
ros2 launch crane/launch/crane.launch.py
ros2 topic pub --once /crane/command std_msgs/msg/String "{data: 'MOVE kirmizi_kasa depo'}"
ros2 topic echo /crane/status
```

Commands: `MOVE <load> <area>`, `PICK <load>`, `PLACE <area>`, `HOME`, `STOP`.
Unknown loads/areas, full areas, a busy crane and unreachable targets are rejected with a reason.

## Roadmap

- Fine-tune Whisper on vehicle/engine noise
- Speaker verification so only authorized operators are accepted
- ASR-independent keyword spotting for STOP
- Map-based navigation around obstacles with Nav2 (SLAM + path planning)
- Closed-loop turn control using `/odom`
- Port to embedded hardware such as Jetson
