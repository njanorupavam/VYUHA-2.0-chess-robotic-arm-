# V3 — ROS 2 Final Pipeline

## What this is
The final, clean version. ROS 2 brought back as the communication backbone, replacing the flaky WebSocket layer entirely.

## Files
| File | Role |
|---|---|
| `chess_vision.py` | absdiff vision + ROS publisher on `/white_move` |
| `chess_ros_pkg/stockfish_node.py` | Unchanged from V1 — was always correct |
| `chess_ros_pkg/robot_arm_node.py` | Reads JSON + slews servos + drives Pico via serial |
| `chess_arm_calib.json` | 64-square calibration (same as V2) |
| `pico_servo_listener.py` | Pico firmware (same as V2) |

## How to run
```bash
# Terminal 1
source ~/ros2_ws/install/setup.bash
ros2 run chess_ros_pkg stockfish_node

# Terminal 2
source ~/ros2_ws/install/setup.bash
ros2 run chess_ros_pkg robot_arm_node

# Terminal 3
source ~/ros2_ws/install/setup.bash
python3 chess_vision.py

# Manual test (no camera needed)
ros2 topic pub --once /white_move std_msgs/msg/String "{data: 'e2e4'}"
```

## What improved over V2
- No browser required
- No WebSocket reliability issues
- Clean pub/sub between independent nodes
- Arm node reads directly from JSON — no HTML page needed
