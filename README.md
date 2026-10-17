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

## Safety rules

- **STOP has priority**: If "dur" appears anywhere in the sentence, the vehicle stops.
- **Negation**: "Sakın ileri gitme" (*don't go forward*) does not move the vehicle.
- **Ambiguity rejection**: A sentence containing more than one different command is ignored.
- **Watchdog**: The vehicle stops if no new command arrives for 15 s.
- **Offline**: No internet connection required.

## Installation (Ubuntu 22.04 + ROS2 Humble)

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-turtlebot3 ros-humble-turtlebot3-simulations libportaudio2
pip install -r requirements.txt
export TURTLEBOT3_MODEL=burger
```

## Usage

```bash
# Terminal 1
ros2 launch turtlebot3_gazebo empty_world.launch.py
# Terminal 2
python3 motion_node.py
# Terminal 3
python3 voice_node.py
```

`whisper_test.py`: Tests speech recognition and command matching without ROS.

## Roadmap

- Fine-tune Whisper on vehicle/engine noise
- Speaker verification so only authorized operators are accepted
- ASR-independent keyword spotting for STOP
- Closed-loop turn control using `/odom`
- Port to embedded hardware such as Jetson
