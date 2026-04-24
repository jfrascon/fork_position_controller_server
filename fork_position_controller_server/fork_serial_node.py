#!/usr/bin/env python3

import rclpy

from fork_position_controller_server.fork_serial_ros import ForkSerialNode


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: ForkSerialNode | None = None

    try:
        node = ForkSerialNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
