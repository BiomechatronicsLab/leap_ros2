#!/usr/bin/env python3

import rclpy
from leap_ros2.leap_node import LeapHandNode

def main(args=None):
    rclpy.init(args=args)

    leap_hand = LeapHandNode()

    try:
        rclpy.spin(leap_hand)
    except Exception as e:
        print(e)
    finally:
        leap_hand.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()