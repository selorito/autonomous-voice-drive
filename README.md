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

## Listening modes

Both remotes (`voice_node.py` for the TurtleBot, `crane/voice_crane.py` for the crane) share `audio_listener.py`:

- **Continuous (default)**: the microphone streams in 32 ms chunks into Silero VAD (the ONNX model bundled with
  faster-whisper, run statefully chunk by chunk, ~0.3 ms per chunk). Each detected utterance goes to Whisper.
- **Wake word**: a command must start with the wake word: "**Robot**, atölyeye git" / "**Toros**, kırmızı kasayı depoya taşı" (the crane is named Toros).
  Saying only the wake word arms the next sentence for 6 s. Talking about the system ("mesela kırmızı kasayı
  depoya taşı dersem…") does not trigger it.
- **STOP bypasses the wake word**: "dur" is always accepted.
- **Spoken confirmation (crane)**: load movements are confirmed by voice via offline Piper TTS:
  "Kırmızı kasa depo alanına taşınacak. Onaylıyor musun?" → "evet" runs it; "hayır" or 5 s of silence cancels;
  "dur" triggers an emergency stop. The microphone is paused while the system speaks so it never hears itself.
  Voice: all ~66 phrases the crane can say are pre-rendered once with a natural neural voice
  (`pip install edge-tts && python3 crane/gen_phrases.py`, needs internet once) and played offline at runtime;
  Piper TTS (`tr_TR-dfki-medium`) is the offline fallback for anything not pre-rendered.
- `--ptt`: push-to-talk fallback (Enter → speak → Enter), no wake word. `--text` (crane): type sentences.

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

An MPG-style **truck-mounted telescopic crane** on a 110 × 80 m industrial yard: a central road, four work
areas (stock, workshop, depot, shipping) about 60 m apart, four loads, buildings, an arched hangar,
container stacks, light poles, trees and a fence.

- **Mobile truck**: the crane drives along the road between areas. Before driving it raises the hook, stows the
  boom and retracts the outriggers; on arrival it deploys the outriggers. The wheels roll with the distance driven.
- **Truck-bed transfer**: a crane never drives with a load hanging from the hook. If the target area is out of
  reach, the load is set on the truck bed, carried there, and lifted off again.
- **Joints**: slew (360°), luff (0–75°), telescope (7.6–13.6 m), hoist (0.8–12 m). The sheave counter-rotates
  with the boom so the rope always hangs vertically.
- **Kinematics** (`crane_geometry.py`): closed-form IK puts the boom tip above any target; joints move
  together with per-joint speed limits and smooth start/stop. `CRANE_SPEED` (default 2) scales all speeds.
- **Single source of truth**: `gen_world.py` builds `worlds/crane_site.world` from `crane_geometry.py` + `sites.yaml`,
  and the controller and vision node use the same files.

### Hook camera and vision (`vision_node.py`, `vision_core.py`)

A camera on the boom tip looks straight down, the way real crane operators get a "hook view".
Before every pick the controller asks the camera to confirm the load:

1. **Segmentation**: loads are bright, the ground (asphalt/concrete) is dark; yellow paint, the yellow crane and
   the hook are excluded by hue.
2. **Filtering**: contour area must match the expected size of a load at the current camera height; shape must be
   rectangular (aspect ratio, fill ratio); blobs touching the image border are ignored.
3. **Classification**: saturated → hue (red / blue); unsaturated → brightness *relative to the surrounding ground*
   (white crate vs. grey concrete block), so a shadow from the boom does not change the answer.
4. **Localization**: pixel → yard coordinates from camera height, slew angle and field of view (pinhole model),
   accounting for whether the load sits on the ground or on the truck bed.

Result: "camera: kirmizi_kasa confirmed (offset 0.02 m)". If a **different** load is under the hook, the task is
stopped. If vision is not running, the check is skipped with a warning. Synthetic test: 72/72 scenarios
(every area, every load, sun / full / partial shadow, truck bed), mean position error 1.3 cm.

```bash
ros2 launch crane/launch/crane.launch.py              # Gazebo + controller + vision ("Kanca kamerasi" window)
ros2 launch crane/launch/crane.launch.py vision:=false
ros2 topic pub --once /crane/command std_msgs/msg/String "{data: 'MOVE kirmizi_kasa depo'}"
ros2 topic echo /crane/status
ros2 topic echo /crane/vision                          # detections as JSON
```

Commands: `MOVE <load> <area>`, `PICK <load>`, `PLACE <area>`, `HOME`, `STOP`.
Unknown loads/areas, full areas, a busy crane and unreachable targets are rejected with a reason.

### Voice control over CAN bus

The crane is commanded the way real cranes are: over a CAN bus. The voice module is a "Voice ECU"
on the bus; `can_bridge.py` is the "Crane ECU" that forwards commands to the controller.

```
voice_crane.py (Voice ECU) ──vcan0──► can_bridge.py (Crane ECU) ──ROS──► crane_controller ──► Gazebo
```

Messages are defined in `crane/can/crane.dbc` (standard DBC format):

| ID | Message | From | Purpose |
|---|---|---|---|
| 0x010 | ESTOP | Voice | Emergency stop. Lowest ID = highest bus priority (CAN arbitration). |
| 0x100 | CRANE_CMD | Voice | MOVE / PICK / PLACE / HOME + load id + area id + rolling counter |
| 0x200 | CRANE_STATUS | Crane | IDLE / BUSY / STOPPED / FAULT, load on hook (10 Hz) |
| 0x201 | CRANE_ACK | Crane | Result for every command (OK, BUSY, AREA_FULL, ...) |
| 0x300 | CRANE_JOINTS | Crane | Slew, luff, telescope, hoist telemetry (10 Hz) |
| 0x710 / 0x720 | HEARTBEAT | both | Voice ECU 10 Hz, Crane ECU 1 Hz |

Safety on the bus: a repeated counter is ignored (DUPLICATE); if the Voice ECU heartbeat stops for 1 s
while a CAN-started task is running, the crane stops (dead-man).

Spoken commands (Turkish): "kırmızı kasayı depoya taşı" (MOVE), "beton bloğu al" (PICK),
"atölyeye bırak" (PLACE), "vinci topla" (HOME), "dur" (ESTOP). Suffixes such as "bloğu" and
source/target forms ("stoktan … depoya") are handled; a verb is required.

```bash
python3 crane/gen_world.py                                        # after changing sites.yaml / geometry
./crane/start_demo.sh                                             # clean start: vcan0 + Gazebo + controller + vision + Crane ECU
python3 crane/voice_crane.py                                      # "Toros, ..." (or --ptt / --text)
candump vcan0 | python3 -m cantools decode crane/can/crane.dbc    # live decoded bus traffic
```

## Roadmap

- Boom/load collision checking against buildings and other loads (e.g. distance fields for slender links)
- Learned detector (YOLO) instead of color rules, for real camera images
- Fine-tune Whisper on vehicle/engine noise
- Speaker verification so only authorized operators are accepted
- ASR-independent keyword spotting for STOP
- Map-based navigation around obstacles with Nav2 (SLAM + path planning)
- Closed-loop turn control using `/odom`
- Port to embedded hardware such as Jetson
