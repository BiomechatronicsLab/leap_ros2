#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time

class JointOscillatorNode(Node):
    def __init__(self):
        super().__init__('joint_oscillator_node')

        self.publisher = self.create_publisher(JointState, '/dom/command_joint_states', 10)

        # Initialize joint angles
        self.joint_angles = [0.0] * 16

        # Define angles for grabbing
        self.grab_angles = np.array([0.0] * 16)

        # Define angles for releasing
        self.release_angles = np.array([0.0] * 16)

        # # Create a timer to publish joint angles at 60 Hz
        # self.timer = self.create_timer(1.0 / 60.0, self.publish_joint_angles)
        
        # self.grab()  # Call grab to set angles for grabbing
        # self.release()
        for _ in range (12):
            input("Press Enter to Grab")
            self.grab()  # Call grab to set angles for grabbing
            # time.sleep(2)  # Simulate grabbing for 5 seconds
            input("Press Enter to Release")
            self.release()  # Call release to set angles for releasing
            # time.sleep(2)  # Simulate releasing for 5 seconds
            # input("Press Enter to Grab")


    def grab(self):
        # # self.grab_angles[[0, 4, 8]] = 0.0
        # self.grab_angles[0] = 5.0
        # self.grab_angles[4] = -3.0
        # self.grab_angles[8] = -15.0
        # self.grab_angles[[1, 5, 9]] = 80.0   #Fingers grip
        # self.grab_angles[[2, 6, 10]] = 30.0
        # self.grab_angles[[3, 7, 11]] = 15.0     #15
        # # self.grab_angles[[3, 7, 11]] = 30.0     #15


        # self.grab_angles[12] = 100.0
        # self.grab_angles[13] = 3.0
        # self.grab_angles[14] = -10.0     #Thumb grip
        # self.grab_angles[15] = 60.0     #60
        # # self.grab_angles[15] = 85.0     
        
        self.grab_angles[0] = 5.0
        self.grab_angles[1] = 80.0   #Fingers grip
        self.grab_angles[2] = 30.0
        self.grab_angles[3] = 15.0

        self.grab_angles[12] = 90.0
        self.grab_angles[13] = 3.0
        self.grab_angles[14] = -10.0     #Thumb grip
        self.grab_angles[15] = 60.0

        # self.grab_angles = [-19.36, 21.472, -24.376, 22.088, 20.152, 21.56, -17.072, -10.384, 3.256, 95.744, 48.4, 35.904, 83.688, -27.72, 69.256, 53.328]
        self.grab_angles = [-10.56, 104.632, 39.072, 35.024, -4.664, 106.656, 17.6, 38.72, 3.256, 99.968, 48.224, 35.904, 83.688, -27.192, 69.256, 53.064]
        # self.grab_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.256, 95.0, 48.4, 35.904, 0.0, 0.0, 0.0, 0.0]

        self.grab_angles[0] = -self.grab_angles[0]  # Index 0
        self.grab_angles[4] = -self.grab_angles[4]  # Index 4
        self.grab_angles[8] = -self.grab_angles[8]  # Index 4
        self.grab_angles[12] = -self.grab_angles[12]  # Index 12
        self.grab_angles[13] = -self.grab_angles[13]  # Index 13


        self.joint_angles = list(self.grab_angles)
        self.publish_joint_angles()


    def release(self):
        # self.release_angles[[0, 4, 8]] = 0.0
        # self.release_angles[[1, 5, 9]] = 30.0
        # self.release_angles[[2, 6, 10]] = 30.0
        # self.release_angles[[3, 7, 11]] = 0.0

        # self.release_angles[12] = 100.0
        # self.release_angles[13] = 0.0
        # self.release_angles[14] = -50.0
        # self.release_angles[15] = 30.0

        # Pinch
        self.release_angles[0] = 0.0
        self.release_angles[1] = 30.0
        self.release_angles[2] = 30.0
        self.release_angles[3] = 0.0

        self.release_angles[12] = 90.0
        self.release_angles[13] = 0.0
        self.release_angles[14] = -50.0
        self.release_angles[15] = 30.0

        self.release_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.joint_angles = list(self.release_angles)
        self.publish_joint_angles()


    def publish_joint_angles(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.position = self.joint_angles
        self.publisher.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointOscillatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
