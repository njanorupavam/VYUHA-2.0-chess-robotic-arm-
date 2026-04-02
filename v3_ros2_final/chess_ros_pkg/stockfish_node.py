#!/usr/bin/env python3
# stockfish_node.py — V1 (ROS 2 + YOLO era)
# Subscribes to /white_move, computes best response, publishes to /black_move

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import chess
import chess.engine
import shutil


class StockfishNode(Node):
    def __init__(self):
        super().__init__('stockfish_node')
        self.subscription = self.create_subscription(
            String, 'white_move', self.white_move_callback, 10)
        self.publisher = self.create_publisher(String, 'black_move', 10)
        self.board = chess.Board()

        stockfish_path = shutil.which("stockfish")
        if stockfish_path is None:
            self.get_logger().fatal("Stockfish binary not found in PATH")
            raise RuntimeError("Stockfish not found")

        self.engine = chess.engine.SimpleEngine.popen_uci(stockfish_path)
        self.get_logger().info("Stockfish ROS node ready")

    def white_move_callback(self, msg: String):
        uci_move = msg.data.strip()
        try:
            move = chess.Move.from_uci(uci_move)
            if move not in self.board.legal_moves:
                self.get_logger().warn(f"Illegal white move received: {uci_move}")
                return
            self.board.push(move)
            self.get_logger().info(f"White played: {uci_move}")
            result = self.engine.play(self.board, chess.engine.Limit(time=0.1))
            black_move = result.move
            self.board.push(black_move)
            out = String()
            out.data = black_move.uci()
            self.publisher.publish(out)
            self.get_logger().info(f"Black played: {out.data}")
        except Exception as e:
            self.get_logger().error(f"Error processing move: {e}")

    def destroy_node(self):
        self.engine.quit()
        super().destroy_node()


def main():
    rclpy.init()
    node = StockfishNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
