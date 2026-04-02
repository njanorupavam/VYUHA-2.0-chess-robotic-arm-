# V2 — Working System (absdiff + WebSocket)

## What this was
The first fully working version. Replaced YOLO with OpenCV `absdiff` for vision, and used a WebSocket bridge + HTML calibration page for arm control.

## Files
| File | Role |
|---|---|
| `chess_vision.py` | Camera capture, absdiff detection, Stockfish, WebSocket sender |
| `pc_serial_bridge.py` | WebSocket hub — routes commands between vision and HTML page |
| `pico_servo_listener.py` | Pico firmware — receives `S<pin>:<pulse>` over USB serial |
| `chess_arm_calib.json` | All 64 square angles + zones + slew speeds |

## Why we moved on
The WebSocket → HTML page pathway was unreliable. The browser page had to be open and connected at all times. If anything disconnected, the move was lost.

## Key achievement
- First time the arm physically moved a chess piece
- All 64 squares manually calibrated
- Capture zone, rest position, promotion zone all defined
