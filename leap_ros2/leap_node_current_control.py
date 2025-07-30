#!/usr/bin/env python3

from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from dynamixel_driver.XC330_M288_manager import XC330M288Manager
from dynamixel_driver.XL330_M288_manager import XL330M288Manager
import time

class LeapHandCurrentNode(Node):
    def __init__(self, test_flag=False, node_name="leap_ros2_node"):
        super().__init__(node_name)

        # Declare parameters
        self.declare_parameter("joint_command_topic", "/leap/end_eff/command_joint_states" )
        self.declare_parameter("joint_feedback_topic", "/leap/end_eff/feedback_joint_states")
        self.declare_parameter("baud_rate", 3000000)
        self.declare_parameter("device_name","")
        self.declare_parameter("dynamixel_type", "")
        self.declare_parameter("percent_current_limit", 100)
        self.declare_parameter("pub_pos", True)
        self.declare_parameter("pub_vel", False)
        self.declare_parameter("pub_current", False)
        self.declare_parameter("kP", [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        self.declare_parameter("kI", [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        self.declare_parameter("kD", [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        self.declare_parameter("start_pos_deg", [0.0] * 16)
        self.declare_parameter('min_position_deg', [0.0, -30.0, 0.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0, -60.0, 0.0, 0.0])
        self.declare_parameter('max_position_deg', [90.0, 30.0, 90.0, 90.0, 90.0, 30.0, 90.0, 90.0, 90.0, 30.0, 90.0, 90.0, 90.0, 60.0, 90.0, 90.0])
        self.declare_parameter("return_delay_time", 250) # default parameter

        self.dynamixel_mgr = None

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
        
        self.percent_current_limit = self.get_parameter("percent_current_limit").get_parameter_value().integer_value
        self.pub_pos = self.get_parameter("pub_pos").get_parameter_value().bool_value
        self.pub_vel = self.get_parameter("pub_vel").get_parameter_value().bool_value
        self.pub_current = self.get_parameter("pub_current").get_parameter_value().bool_value
        self.kP = self.get_parameter("kP").get_parameter_value().double_array_value
        self.kI = self.get_parameter("kI").get_parameter_value().double_array_value
        self.kD = self.get_parameter("kD").get_parameter_value().double_array_value
        self.start_pos_deg = self.get_parameter("start_pos_deg").get_parameter_value().double_array_value
        self.min_position_deg = self.get_parameter('min_position_deg').get_parameter_value().double_array_value
        self.max_position_deg = self.get_parameter('max_position_deg').get_parameter_value().double_array_value
        self.return_delay_time = self.get_parameter("return_delay_time").get_parameter_value().integer_value

        # Current limit that is set based on what type of dynamixel motor
        motor_ids = list(range(16))

        # TODO: need to fix this somehow so that it can just automatically detect the model number, to avoid user incorrectly typing motor type...
        if self.dynamixel_type == "XC330-M288": 
            self.dynamixel_mgr = XC330M288Manager(motor_ids, self.baud_rate, self.device_name)
            
        elif self.dynamixel_type == "XL330-M288":
            self.dynamixel_mgr = XL330M288Manager(motor_ids, self.baud_rate, self.device_name)
        else:
            raise ValueError("No valid dynamixel type was specified!")

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

        errored_motors = self.get_errored_motors()
        if errored_motors:
            self.motor_reset(errored_motors)

        print("Begining Start-up Sequence")  # using current-based position mode
        self.dynamixel_mgr.set_torque_disable(self.dynamixel_mgr.motor_ids) # Disable torques prior to changing settings
        self.initialize_position_gains(600, 0, 0)
        self.dynamixel_mgr.set_velocity_limit(self.dynamixel_mgr.motor_ids, 100)
        self.dynamixel_mgr.set_profile_acceleration(self.dynamixel_mgr.motor_ids, 20)
        self.dynamixel_mgr.set_profile_velocity(self.dynamixel_mgr.motor_ids, 100)
        self.dynamixel_mgr.set_current_based_position_mode(self.dynamixel_mgr.motor_ids)
        self.dynamixel_mgr.set_torque_enable(self.dynamixel_mgr.motor_ids)
        self.curr_pos_deg = self.dynamixel_mgr.get_position_deg(self.dynamixel_mgr.motor_ids)
        trajectories = self.make_trajectory(100, motor_ids, self.curr_pos_deg, self.start_pos_deg)
        self.execute_trajectory(0.01, motor_ids, trajectories)

        print("Current Control Begin!")
        # Initialize gains and operating mode
        # Currently operating mode is only set to current based position control!
        self.dynamixel_mgr.set_torque_disable(self.dynamixel_mgr.motor_ids) # Disable torques prior to changing settings
        self.dynamixel_mgr.set_current_mode(self.dynamixel_mgr.motor_ids)

        # Configuration
        self.dynamixel_mgr.set_return_delay_time(self.dynamixel_mgr.motor_ids, self.return_delay_time)
        self.dynamixel_mgr.set_min_position_deg(self.dynamixel_mgr.motor_ids, self.min_position_deg)
        self.dynamixel_mgr.set_max_position_deg(self.dynamixel_mgr.motor_ids, self.max_position_deg)
        self.dynamixel_mgr.set_current_limit(self.dynamixel_mgr.motor_ids, np.ones(len(motor_ids))
                                              * self.dynamixel_mgr.max_curr_limit * (self.percent_current_limit/100))
        self.dynamixel_mgr.set_torque_enable(self.dynamixel_mgr.motor_ids)

        # PID Terms
        self.desired_pos_deg = self.start_pos_deg
        self.integral = 0
        self.previous_error = self.desired_pos_deg - np.array(self.dynamixel_mgr.get_position_deg(self.dynamixel_mgr.motor_ids))

        # TODO: This frequency could be an exposed config parameter, arbitrarily assigned currently
        self.timer_period = 1.0 / 100.0
        self.timer = self.create_timer(self.timer_period, self.read_and_publish_data)

    def __del__(self):
        del self.dynamixel_mgr # Kill dynamixel manager object        
 
    def initialize_position_gains(self, kP, kI, kD):
            try:
                kP = np.ones(len(self.dynamixel_mgr.motor_ids)) * self.kP                
                kI = np.ones(len(self.dynamixel_mgr.motor_ids)) * self.kI
                kD = np.ones(len(self.dynamixel_mgr.motor_ids)) * self.kD

                self.dynamixel_mgr.initialize_gains(self.dynamixel_mgr.motor_ids, kP, kI, kD)

            except Exception as e:
                self.get_logger().error(f"Error initializing gains: {str(e)}")

    def make_trajectory(self, num_points, motor_ids, current_position, goal_position):
       
        current_position = np.array(current_position)
        goal_position = np.array(goal_position)
        
        if len(current_position) != len(motor_ids) or len(goal_position) != len(motor_ids):
            raise ValueError("Position array length must match number of motor_ids")
        
        num_motors = len(motor_ids)
        trajectories = np.zeros((num_points, num_motors))
        
        for i in range(num_motors):
            start_pos = current_position[i]
            end_pos = goal_position[i]
            
            # Generate linear trajectory (constant velocity)
            trajectory = np.linspace(start_pos, end_pos, num_points)
            trajectories[:, i] = trajectory
        
        return trajectories

    def execute_trajectory(self, delay, id, trajectories):
        for i in range(len(trajectories)):
            self.dynamixel_mgr.set_goal_position_deg(id, trajectories[i,:])
            time.sleep(delay)

    def get_errored_motors(self):
        hardware_error_status = self.dynamixel_mgr.get_hardware_error_status(self.dynamixel_mgr.motor_ids)
        errored_motors = []
        errored_motors = [i for i, val in enumerate(hardware_error_status) if val != 0]
        return errored_motors

    def motor_reset(self, errored_motors):
        print("These motors are errored, resetting!", errored_motors)
        error_present = True
        while error_present:
            self.dynamixel_mgr.set_goal_current_mA(errored_motors, np.zeros(len(errored_motors)))
            self.dynamixel_mgr.set_torque_disable(errored_motors) # Disable torques prior to changing settings
            time.sleep(0.25) # give some time to have the torques disabled

            success_arr = self.dynamixel_mgr.reboot_motors(errored_motors)
            time.sleep(0.75) # give some time for the motors to be rebooted
            errored_motors = self.get_errored_motors()

            if not errored_motors:
                print("Motors should be reset")
                error_present = False
                self.dynamixel_mgr.set_torque_enable(errored_motors)

    def control_action(self, curr_pos_deg):
        try:

            # Calculate per dt errors
            pos_error_deg = (self.desired_pos_deg - np.array(curr_pos_deg))

            # TODO: redo with actual timer period ? 
            self.integral += pos_error_deg * self.timer_period
            derivate = (pos_error_deg - self.previous_error) / self.timer_period

            # Add Ki, Kd
            control_action = np.array(self.kP) * pos_error_deg + np.array(self.kI) * self.integral + np.array(self.kD) * derivate 
            goal_curr_mA_arr = control_action * np.ones(len(self.dynamixel_mgr.motor_ids))

            for ind, goal_curr in enumerate(goal_curr_mA_arr):
                if abs(goal_curr) > self.percent_current_limit/100 * self.dynamixel_mgr.max_curr_limit:
                    goal_curr_mA_arr[ind] = np.sign(self.percent_current_limit/100 * self.dynamixel_mgr.max_curr_limit)
            
            self.dynamixel_mgr.set_goal_current_mA(self.dynamixel_mgr.motor_ids, goal_curr_mA_arr)
            self.previous_error = pos_error_deg
        except Exception as e:
            self.get_logger().error(f"Error in control action: {str(e)}")

    def read_and_publish_data(self):
        try:
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()

            # By constantly setting the torque to be re-enabled, it will avoid the case where when rebooting the motor,
            # the command is somehow missed (likely due to time taken when rebooting)
            # Potentially a jank work around, but TBD if this is the policy we should use for the hand
            self.dynamixel_mgr.set_torque_enable(self.dynamixel_mgr.motor_ids) 

            # Check for errored motors
            errored_motors = self.get_errored_motors()

            if errored_motors:
                self.motor_reset(errored_motors)

            curr_pos_deg = self.dynamixel_mgr.get_position_deg(self.dynamixel_mgr.motor_ids)

            if self.pub_pos:
                joint_state_msg.position = curr_pos_deg
            if self.pub_vel:
                velocity_readings = self.dynamixel_mgr.get_velocity(self.dynamixel_mgr.motor_ids)
                # because of the two's compliments, have to convert from a list of ints to a list of floats!
                joint_state_msg.velocity = list(map(float, velocity_readings))
            if self.pub_current:
                current_readings = self.dynamixel_mgr.get_current(self.dynamixel_mgr.motor_ids)
                # because of the two's compliments, have to convert from a list of ints to a list of floats!
                joint_state_msg.effort = list(map(float, current_readings))
            self.publisher.publish(joint_state_msg)

            if not errored_motors:
                self.control_action(curr_pos_deg)

        except Exception as e:
            self.get_logger().error(f"Error reading and publishing data: {str(e)}")

    def command_callback(self, msg):
        
        # Update desired_pos_deg 
        self.desired_pos_deg = msg.position 