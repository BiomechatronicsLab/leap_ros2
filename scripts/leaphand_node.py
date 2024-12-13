#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from dynamixel_driver import dynamixel_manager
from dynamixel_driver.XC330_M288_manager import XC330M288Manager
from dynamixel_driver.XL330_M288_manager import XL330M288Manager

POSITION_MODE_ENUM = 3


class LeapHandNode(Node):
    def __init__(self, test_flag=False):
        super().__init__("dynamixel_reader_node")

        # Declare and get parameters from the parameter server
        self.joint_command_topic = (
            self.declare_parameter(
                "joint_command_topic", "/leap/end_eff/command_joint_states"
            )
            .get_parameter_value()
            .string_value
        )

        self.joint_feedback_topic = (
            self.declare_parameter("joint_feedback_topic", "/leap/end_eff/dynamixel_joint_states")
            .get_parameter_value()
            .string_value
        )

        self.baud_rate = (
            self.declare_parameter("baud_rate", 3000000)
            .get_parameter_value()
            .integer_value
        )

        # TODO: If not specified - error out! 
        self.device_name = (
            self.declare_parameter(
                "device_name",
                "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISZ8G-if00-port0",
            )
            .get_parameter_value()
            .string_value
        )

        # TODO: If not specified - error out!
        self.dynamixel_type = (
            self.declare_parameter(
                "dynamixel_type", "None"
            )
            .get_parameter_value()
            .string_value
        )

        self.pub_pos = (
            self.declare_parameter("pub_pos", True).get_parameter_value().bool_value
        )

        self.pub_vel = (
            self.declare_parameter("pub_vel", False).get_parameter_value().bool_value
        )

        self.pub_current = (
            self.declare_parameter("pub_current", False)
            .get_parameter_value()
            .bool_value
        )

        self.kP = self.declare_parameter("kP", 500).get_parameter_value().integer_value
        self.kI = self.declare_parameter("kI", 0).get_parameter_value().integer_value
        self.kD = self.declare_parameter("kD", 200).get_parameter_value().integer_value

        self.start_pos = (
            self.declare_parameter("start_pos", [0.0] * 16)
            .get_parameter_value()
            .double_array_value
        )

        if not test_flag:
            self.setup()


    def setup(self):
        motors_ids = list(range(16))
        if self.dynamixel_type == "XC330-M288":
            self.dynamixel_mgr = XC330M288Manager(motors_ids, self.baud_rate, self.device_name)
            
            
        elif self.dynamixel_type == "XL330-M288":
            self.dynamixel_mgr = XL330M288Manager(motors_ids, self.baud_rate, self.device_name)

        self.curr_pos = self.start_pos

        # Ensure the curr_pos array has the correct length
        if len(self.curr_pos) != 16:
            self.curr_pos = [0] * 16

        # # Create publisher
        # self.publisher = self.create_publisher(
        #     JointState, self.joint_feedback_topic, 10
        # )

        # # Create subscriber
        # self.subscription = self.create_subscription(
        #     JointState, self.joint_command_topic, self.command_callback, 10
        # )

        # Initialize gains and operating mode
        self.dynamixel_mgr.set_operating_mode(self, np.ones(len(self.dynamixel_mgr.motors_ids) * POSITION_MODE_ENUM))
        self.initialize_gains()
        self.dynamixel_mgr.set_torque_enable(self, np.ones(len(self.dynamixel_mgr.motor_ids)))

        # Set initial position
        self.dynamixel_mgr.set_goal_position_deg(self.curr_pos)

        # Create timer to publish data
        timer_period = 1.0 / 60.0
        self.timer = self.create_timer(timer_period, self.read_and_publish_data)

    def initialize_gains(self):
            try:
                kP = np.ones(len(self.motor_ids)) * self.kP
                kP[[0, 4, 8]] = np.ones(3) * (self.kP * 0.75) 
                
                kI = np.ones(len(self.motor_ids)) * self.kI
                kI[[0, 4, 8]] = np.ones(3) * (self.kI * 0.75)

                kD = np.ones(len(self.motor_ids)) * self.kD
                kD[[0, 4, 8]] = np.ones(3) * (self.kD * 0.75)

                self.dynamixel_mgr.initialize_gains(self.dynamixel_mgr.motor_ids, kP, kI, kD)

            except Exception as e:
                self.get_logger().error(f"Error initializing gains: {str(e)}")

    def __del__(self):
        del self.dynamixel_mgr # Kill dynamixel manager object        
 
    # def read_and_publish_data(self):
    #     try:
    #         joint_state_msg = JointState()
    #         joint_state_msg.header.stamp = self.get_clock().now().to_msg()

    #         if self.pub_pos:
    #             dxl_comm_result_position = self.group_sync_read_position.txRxPacket()
    #             if dxl_comm_result_position != dxl.COMM_SUCCESS:
    #                 self.get_logger().warn(
    #                     f"GroupSyncRead txRxPacket failed for position: {self.packet_handler.getTxRxResult(dxl_comm_result_position)}"
    #                 )

    #         if self.pub_vel:
    #             dxl_comm_result_velocity = self.group_sync_read_velocity.txRxPacket()
    #             if dxl_comm_result_velocity != dxl.COMM_SUCCESS:
    #                 self.get_logger().warn(
    #                     f"GroupSyncRead txRxPacket failed for velocity: {self.packet_handler.getTxRxResult(dxl_comm_result_velocity)}"
    #                 )

    #         if self.pub_current:
    #             dxl_comm_result_current = self.group_sync_read_current.txRxPacket()
    #             if dxl_comm_result_current != dxl.COMM_SUCCESS:
    #                 self.get_logger().warn(
    #                     f"GroupSyncRead txRxPacket failed for current: {self.packet_handler.getTxRxResult(dxl_comm_result_current)}"
    #                 )

    #         for dxl_id in self.motors:
    #             if self.pub_pos:
    #                 pos_ticks = self.group_sync_read_position.getData(
    #                     dxl_id, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION
    #                 )
    #                 pos_deg = self.ticks_to_degrees(pos_ticks)
    #                 joint_state_msg.position.append(pos_deg)

    #             if self.pub_vel:
    #                 vel_ticks = self.group_sync_read_velocity.getData(
    #                     dxl_id, self.ADDR_PRESENT_VELOCITY, self.LEN_PRESENT_VELOCITY
    #                 )
    #                 vel_deg = self.ticks_to_degrees(vel_ticks)
    #                 joint_state_msg.velocity.append(vel_deg)

    #             if self.pub_current:
    #                 curr_ticks = self.group_sync_read_current.getData(
    #                     dxl_id, self.ADDR_PRESENT_CURRENT, self.LEN_PRESENT_CURRENT
    #                 )
    #                 joint_state_msg.effort.append(curr_ticks)

    #         self.publisher.publish(joint_state_msg)

    #     except Exception as e:
    #         self.get_logger().error(f"Error reading and publishing data: {str(e)}")

    # def command_callback(self, msg):
    #     try:
    #         for i, pos_deg in enumerate(msg.position):
    #             pos_ticks = self.degrees_to_ticks(pos_deg)
    #             param_goal_position = [
    #                 dxl.DXL_LOBYTE(dxl.DXL_LOWORD(int(pos_ticks))),
    #                 dxl.DXL_HIBYTE(dxl.DXL_LOWORD(int(pos_ticks))),
    #                 dxl.DXL_LOBYTE(dxl.DXL_HIWORD(int(pos_ticks))),
    #                 dxl.DXL_HIBYTE(dxl.DXL_HIWORD(int(pos_ticks))),
    #             ]
    #             dxl_id = self.motors[i]
    #             if not self.group_sync_write_position.addParam(
    #                 dxl_id, param_goal_position
    #             ):
    #                 self.get_logger().error(
    #                     f"Failed to add param for Dynamixel ID: {dxl_id} (goal position)"
    #                 )

    #         dxl_comm_result = self.group_sync_write_position.txPacket()
    #         if dxl_comm_result != dxl.COMM_SUCCESS:
    #             self.get_logger().warn(
    #                 f"GroupSyncWrite txPacket failed for goal position: {self.packet_handler.getTxRxResult(dxl_comm_result)}"
    #             )
    #         self.group_sync_write_position.clearParam()

    #     except Exception as e:
    #         self.get_logger().error(f"Error in command callback: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = LeapHandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
