#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class StatusMonitor(Node):
    def __init__(self):
        super().__init__("pico_body_real_status_monitor")
        self.create_subscription(
            String,
            "/pico_body_real/status",
            self._print_status,
            10,
        )

    @staticmethod
    def _print_status(message):
        print(message.data, flush=True)


def main():
    rclpy.init()
    node = StatusMonitor()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
