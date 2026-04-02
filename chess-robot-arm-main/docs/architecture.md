# System Architecture

## ROS 2 Topic Map

```
chess_vision_node
    └── publishes → /white_move (std_msgs/String)
                    e.g. "e2e4"

stockfish_node
    ├── subscribes → /white_move
    │   validates move against python-chess board state
    │   computes best response via Stockfish UCI
    └── publishes  → /black_move (std_msgs/String)
                    e.g. "c7c5"

robot_arm_node
    └── subscribes → /black_move
        expands move into action sequence:
            PICK(c7) → LIFT → MOVE(c5) → PLACE → HOME
        loads angles from chess_arm_calib.json
        slews each servo degree-by-degree
        sends S<pin>:<pulse_us> over USB serial → Pico
```

## Servo Pin Map

| Pin | Servo | Type |
|---|---|---|
| GP0 | Base | MG995 |
| GP1 | Shoulder | MG995 |
| GP2 | Elbow | MG995 |
| GP3 | Wrist Pitch | SG90 |
| GP4 | Wrist Roll | SG90 |
| GP5 | Gripper | SG90 |

## Serial Protocol

Commands sent from PC to Pico over USB at 115200 baud:

```
S<pin>:<pulse_us>    # Move servo
# e.g. S0:1500       # Base to 90°

R                    # Reset all servos to 90°
P                    # Print current pulse widths (debug)
```

Pulse range: 500µs (0°) to 2500µs (180°)
Formula: `pulse = 500 + (angle / 180.0) * 2000`

## Board Coordinate System

```
Camera view (overhead, fixed mount):

TOP-LEFT  = H8    TOP-RIGHT  = H1
BOT-LEFT  = A8    BOT-RIGHT  = A1

Column index 0→7 maps to files a→h
Row    index 0→7 maps to ranks 8→1
```
