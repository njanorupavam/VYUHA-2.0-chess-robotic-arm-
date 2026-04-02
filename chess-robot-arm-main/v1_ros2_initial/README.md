# V1 — ROS 2 Initial Architecture

## What this was
The first version of the chess robot system. Built a full ROS 2 architecture with:
- `stockfish_node.py` — subscribes to `/white_move`, publishes to `/black_move`
- `robot_arm_node.py` — mock state machine (PICK→LIFT→MOVE→PLACE→HOME)
- `bridge_node.py` — stdin bridge for testing

## Why we moved on
YOLO-based piece detection was too sensitive to lighting. The ROS architecture itself was solid — just the vision layer needed replacing with something more deterministic.

## What we kept
The `stockfish_node.py` from this version is **unchanged** in V3. It was right from the start.
The state machine concept from `robot_arm_node.py` was carried forward and extended with real hardware integration.

## How to run (for reference)
```bash
# Terminal 1
ros2 run chess_ros_pkg stockfish_node

# Terminal 2
ros2 run chess_ros_pkg robot_arm_node

# Terminal 3 — manual test
ros2 topic pub --once /white_move std_msgs/msg/String "{data: 'e2e4'}"
```
