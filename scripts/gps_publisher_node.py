#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tuw_object_map.gps_publisher import GPSPublisher

def main(args=None):
    rclpy.init(args=args)

    gps_publisher = GPSPublisher()

    rclpy.spin(gps_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()