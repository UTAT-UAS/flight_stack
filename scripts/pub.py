#!/usr/bin/env python3

import rclpy

from flight_stack.flight_stack import FlightPlanner


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = FlightPlanner()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
