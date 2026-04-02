#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum
import time


class State(Enum):
    IDLE = "IDLE"
    PICKING = "PICKING"
    LIFTING = "LIFTING"
    MOVING = "MOVING"
    PLACING = "PLACING"
    RETURNING_HOME = "RETURNING_HOME"
    DONE = "DONE"
    ERROR = "ERROR"


class RobotArmNode(Node):
    def __init__(self):
        super().__init__("robot_arm_node")

        self.subscription = self.create_subscription(
            String,
            "black_move",
            self.on_move_received,
            10
        )

        self.state = State.IDLE
        self.get_logger().info("Robot arm mock node ready")

    # ---------- CALLBACK ----------

    def on_move_received(self, msg: String):
        if self.state != State.IDLE:
            self.get_logger().error("Received move while not IDLE")
            self.transition_to(State.ERROR)
            return

        move = msg.data.strip()
        self.get_logger().info(f"Received move: {move}")

        try:
            sequences = self.expand_move(move)
            for idx, seq in enumerate(sequences, start=1):
                self.get_logger().info(f"Executing sequence {idx}")
                self.execute_sequence(seq)
        except Exception as e:
            self.get_logger().error(f"Move processing failed: {e}")
            self.transition_to(State.ERROR)
            return

        self.transition_to(State.IDLE)

    # ---------- MOVE EXPANSION ----------

    def expand_move(self, move: str):
        sequences = []

        # Promotion detection
        promotion_piece = None
        if len(move) == 5:
            promotion_piece = move[4]
            move = move[:4]

        from_sq = move[:2]
        to_sq = move[2:4]

        # Castling detection
        castling_moves = {
            "e1g1": [("e1", "g1"), ("h1", "f1")],
            "e1c1": [("e1", "c1"), ("a1", "d1")],
            "e8g8": [("e8", "g8"), ("h8", "f8")],
            "e8c8": [("e8", "c8"), ("a8", "d8")],
        }

        if move in castling_moves:
            for frm, to in castling_moves[move]:
                sequences.append([
                    f"PICK({frm})",
                    "LIFT",
                    f"MOVE({to})",
                    "PLACE",
                    "HOME"
                ])
            return sequences

        # Normal move (capture handled later)
        sequences.append([
            f"PICK({from_sq})",
            "LIFT",
            f"MOVE({to_sq})",
            "PLACE",
            "HOME"
        ])

        if promotion_piece:
            self.get_logger().info(
                f"PROMOTION_EVENT: promote to {promotion_piece}"
            )

        return sequences

    # ---------- EXECUTION ----------

    def execute_sequence(self, actions):
        for action in actions:
            if self.state == State.ERROR:
                return

            if action.startswith("PICK"):
                self.transition_to(State.PICKING, action)
            elif action == "LIFT":
                self.transition_to(State.LIFTING)
            elif action.startswith("MOVE"):
                self.transition_to(State.MOVING, action)
            elif action == "PLACE":
                self.transition_to(State.PLACING)
            elif action == "HOME":
                self.transition_to(State.RETURNING_HOME)
            else:
                self.get_logger().error(f"Unknown action: {action}")
                self.transition_to(State.ERROR)
                return

            time.sleep(0.3)

        self.transition_to(State.DONE)

    # ---------- STATE TRANSITION ----------

    def transition_to(self, new_state: State, detail: str = ""):
        self.get_logger().info(
            f"STATE {self.state.value} → {new_state.value} {detail}"
        )
        self.state = new_state


def main():
    rclpy.init()
    node = RobotArmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
