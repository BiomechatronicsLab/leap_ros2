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

            input("Press Enter to Release")
            self.release()  # Call release to set angles for releasing


    def grab(self):
        ## Entire hand
        # Fingers grip
        self.grab_angles[[0, 4, 8]] = 80.0  
        self.grab_angles[[1, 5, 9]] = 0.0
        self.grab_angles[[2, 6, 10]] = 30.0
        self.grab_angles[[3, 7, 11]] = 15.0    

        # Thumb grip
        self.grab_angles[12] = 100.0
        self.grab_angles[13] = -85.0
        self.grab_angles[14] = -10.0     
        self.grab_angles[15] = 60.0        
        
        # ## Pinch (2 Fingers)
        # # Fingers grasp
        # self.grab_angles[8] = 80.0
        # self.grab_angles[9] = 5.0   
        # self.grab_angles[10] = 30.0
        # self.grab_angles[11] = 15.0

        # # Thumb grasp
        # self.grab_angles[12] = 90.0
        # self.grab_angles[13] = -90.0
        # self.grab_angles[14] = -10.0     
        # self.grab_angles[15] = 60.0

        self.joint_angles = list(self.grab_angles)
        self.publish_joint_angles()


    def release(self):
        ## Entire hand
        # Fingers grasp 
        self.release_angles[[0, 4, 8]] = 30.0
        self.release_angles[[1, 5, 9]] = 0.0
        self.release_angles[[2, 6, 10]] = 30.0
        self.release_angles[[3, 7, 11]] = 0.0

        # Thumb grasp
        self.release_angles[12] = 100.0
        self.release_angles[13] = -85.0
        self.release_angles[14] = -50.0
        self.release_angles[15] = 30.0

        # ## Pinch (2 Fingers)
        # # Fingers grasp
        # self.release_angles[8] = 30.0
        # self.release_angles[9] = 0.0
        # self.release_angles[10] = 30.0
        # self.release_angles[11] = 0.0

        # # Thumb grasp
        # self.release_angles[12] = 90.0
        # self.release_angles[13] = -90.0
        # self.release_angles[14] = -50.0
        # self.release_angles[15] = 30.0

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
