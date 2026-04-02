#!/usr/bin/env python3
# bridge_node.py — V1
# Simple bridge: reads white move from stdin, publishes to ROS, listens for black move

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BridgeNode(Node):
    def __init__(self, white_move: str):
        super().__init__('bridge_node')
        self.publisher = self.create_publisher(String, 'white_move', 10)
        self.subscription = self.create_subscription(
            String, 'black_move', self.black_move_callback, 10)
        self.white_move = white_move
        self.sent = False
        self.timer = self.create_timer(0.5, self.publish_once)
        self.get_logger().info(f"Bridge ready with white move: {white_move}")

    def publish_once(self):
        if self.sent:
            return
        msg = String()
        msg.data = self.white_move
        self.publisher.publish(msg)
        self.get_logger().info(f"Published white move: {self.white_move}")
        self.sent = True

    def black_move_callback(self, msg: String):
        self.get_logger().info(f"Received black move: {msg.data}")
        rclpy.shutdown()


def main():
    white_move = sys.stdin.read().strip()
    if not white_move:
        print("ERROR: No white move provided on stdin", file=sys.stderr)
        return
    rclpy.init()
    node = BridgeNode(white_move)
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
