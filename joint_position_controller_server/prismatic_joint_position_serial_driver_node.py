#!/usr/bin/env python3

import rclpy

from joint_position_controller_server.prismatic_joint_position_serial_driver import (
    PrismaticJointPositionSerialDriver,
)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: PrismaticJointPositionSerialDriver | None = None

    try:
        node = PrismaticJointPositionSerialDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
