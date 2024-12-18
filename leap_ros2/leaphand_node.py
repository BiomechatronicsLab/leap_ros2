#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from dynamixel_driver import dynamixel_manager
from dynamixel_driver.XC330_M288_manager import XC330M288Manager
from dynamixel_driver.XL330_M288_manager import XL330M288Manager

import time

# TODO: I don't love this because you have to set your position enums in the leaphand node, when thats a driver specific implementation. TBD what to do.
POSITION_MODE_ENUM = 3

class LeapHandNode(Node):
    def __init__(self, test_flag=False):
        super().__init__("leap_ros2_node")

        # Declare parameters
        self.declare_parameter("joint_command_topic", "/leap/end_eff/command_joint_states" )
        self.declare_parameter("joint_feedback_topic", "/leap/end_eff/dynamixel_joint_states")
        self.declare_parameter("baud_rate", 3000000)
        self.declare_parameter("device_name","")
        self.declare_parameter("dynamixel_type", "")
        self.declare_parameter("pub_pos", True)
        self.declare_parameter("pub_vel", False)
        self.declare_parameter("pub_current", False)
        self.declare_parameter("kP", 500)
        self.declare_parameter("kI", 0)
        self.declare_parameter("kD", 200)
        self.declare_parameter("start_pos_deg", [0.0] * 16)

        if not test_flag:
            self.setup()

    def setup(self):
        # Obtain parameter values from the parameter server
        self.joint_command_topic = self.get_parameter("joint_command_topic").get_parameter_value().string_value
        self.joint_feedback_topic = self.get_parameter("joint_feedback_topic").get_parameter_value().string_value
        self.baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self.device_name = self.get_parameter("device_name").get_parameter_value().string_value
        if self.device_name == "":
            raise ValueError(
                "Please state the device_name in the configuration file."
            )

        self.dynamixel_type = self.get_parameter("dynamixel_type").get_parameter_value().string_value
        if self.dynamixel_type == "":
            raise ValueError(
                "Please state the dynamixel_type in the configuration file."
            )

        self.pub_pos = self.get_parameter("pub_pos").get_parameter_value().bool_value
        self.pub_vel = self.get_parameter("pub_vel").get_parameter_value().bool_value
        self.pub_current = self.get_parameter("pub_current").get_parameter_value().bool_value
        self.kP = self.get_parameter("kP").get_parameter_value().integer_value
        self.kI = self.get_parameter("kI").get_parameter_value().integer_value
        self.kD = self.get_parameter("kD").get_parameter_value().integer_value
        self.start_pos_deg = self.get_parameter("start_pos_deg").get_parameter_value().double_array_value

        motor_ids = list(range(16))
        if self.dynamixel_type == "XC330-M288":
            self.dynamixel_mgr = XC330M288Manager(motor_ids, self.baud_rate, self.device_name)
            
            
        elif self.dynamixel_type == "XL330-M288":
            self.dynamixel_mgr = XL330M288Manager(motor_ids, self.baud_rate, self.device_name)


        # Ensure the start_pos_deg array has the correct length. If not, just force it to home.
        if len(self.start_pos_deg) != 16:
            self.start_pos_deg = [0] * 16

        # Create publisher
        self.publisher = self.create_publisher(
            JointState, self.joint_feedback_topic, 10
        )

        # Create subscriber
        self.subscription = self.create_subscription(
            JointState, self.joint_command_topic, self.command_callback, 10
        )

        # Initialize gains and operating mode
        # Currently operating mode is only set to position and cannot be changed (TBD!)
        self.dynamixel_mgr.set_torque_enable(self.dynamixel_mgr.motor_ids, np.zeros(len(self.dynamixel_mgr.motor_ids))) # Disable torques prior to changing settings
        self.dynamixel_mgr.set_operating_mode(self.dynamixel_mgr.motor_ids, np.ones(len(self.dynamixel_mgr.motor_ids)) * POSITION_MODE_ENUM)
        self.dynamixel_mgr.set_torque_enable(self.dynamixel_mgr.motor_ids, np.ones(len(self.dynamixel_mgr.motor_ids)))
        self.initialize_gains()

        # Set initial position
        self.dynamixel_mgr.set_goal_position_deg(self.dynamixel_mgr.motor_ids, self.start_pos_deg)

        # # Create timer to publish data
        timer_period = 1.0 / 60.0
        self.timer = self.create_timer(timer_period, self.read_and_publish_data)

    def initialize_gains(self):
            try:
                kP = np.ones(len(self.dynamixel_mgr.motor_ids)) * self.kP
                kP[[0, 4, 8]] = np.ones(3) * (self.kP * 0.75) 
                
                kI = np.ones(len(self.dynamixel_mgr.motor_ids)) * self.kI
                kI[[0, 4, 8]] = np.ones(3) * (self.kI * 0.75)

                kD = np.ones(len(self.dynamixel_mgr.motor_ids)) * self.kD
                kD[[0, 4, 8]] = np.ones(3) * (self.kD * 0.75)

                self.dynamixel_mgr.initialize_gains(self.dynamixel_mgr.motor_ids, kP, kI, kD)

            except Exception as e:
                self.get_logger().error(f"Error initializing gains: {str(e)}")

    def __del__(self):
        del self.dynamixel_mgr # Kill dynamixel manager object        
 
    def read_and_publish_data(self):
        try:
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()

            if self.pub_pos:
                joint_state_msg.position = self.dynamixel_mgr.get_position_deg(self.dynamixel_mgr.motor_ids)
            if self.pub_vel:
                velocity_readings = self.dynamixel_mgr.get_velocity(self.dynamixel_mgr.motor_ids)
                # because of the two's compliments, have to convert from a list of ints to a list of floats!
                joint_state_msg.velocity = list(map(float, velocity_readings))
            if self.pub_current:
                current_readings = self.dynamixel_mgr.get_current(self.dynamixel_mgr.motor_ids)
                # because of the two's compliments, have to convert from a list of ints to a list of floats!
                joint_state_msg.effort = list(map(float, current_readings))
            self.publisher.publish(joint_state_msg)

        except Exception as e:
            self.get_logger().error(f"Error reading and publishing data: {str(e)}")

    def command_callback(self, msg):
        print(msg.position)
        try:
            self.dynamixel_mgr.set_goal_position_deg(self.dynamixel_mgr.motor_ids, msg.position)
        except Exception as e:
            self.get_logger().error(f"Error in command callback: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = LeapHandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
