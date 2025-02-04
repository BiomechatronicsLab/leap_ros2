# test_example.py
import pytest
import launch_testing
import launch_testing.markers

from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType

import os
import launch_ros
import rclpy

from launch import LaunchDescription
from rclpy.node import Node
import launch_pytest
import time
from sensor_msgs.msg import JointState

# GLOBAL VARIABLES
node_name = "test_leap_node"

@launch_pytest.fixture
@launch_testing.markers.keep_alive
def launch_leap_ros2_node(config_params):

    # Set an arbitrary ROS_DOMAIN_ID so that the test is performed without inteference
    os.environ['ROS_DOMAIN_ID'] = '42'

    return LaunchDescription([
        launch_ros.actions.Node(
            package='leap_ros2',  #
            executable='leap_driver.py',  
            name=node_name,
            output='screen',
            parameters=[config_params]
        ),
        launch_testing.actions.ReadyToTest()
    ])

class LeapPositionChecker(Node):
    def __init__(self, config_params):
        super().__init__('leap_position_checker')

        # Create publisher for joint states
        self.pub_joint_states = self.create_publisher(JointState, config_params["joint_command_topic"], 10)

        self.sub_joint_states = self.create_subscription(
            JointState, config_params["joint_feedback_topic"], self.command_callback, 10
        )
        self.feedback_position_deg = []

    def publish_position(self, positions_to_command):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [f"joint_{i}" for i in range(16)]
        joint_state_msg.position = positions_to_command
        self.pub_joint_states.publish(joint_state_msg)

    def command_callback(self, msg):
        self.feedback_position_deg = msg.position 

    def spin_for_duration(self, duration_sec):
        """Spin the given node for the specified duration in seconds."""
        start_time = time.time()
        while time.time() - start_time < duration_sec:
            rclpy.spin_once(self, timeout_sec=0.1)  # Adjust timeout_sec for finer granularity if needed

@pytest.fixture
def leap_position_checker(config_params):
    test_node = LeapPositionChecker(config_params)
    yield test_node
    test_node.destroy_node()

@pytest.mark.launch(fixture=launch_leap_ros2_node)
def test_leap_end_to_end_position(leap_position_checker):
    tolerance_deg = 10.0 # currently arbitrary 10degree tolerance - if too low, the motors may not actually ever reach the position, due to their kP, kD, kI gains

    positions_to_command_deg = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [-4.928, 4.136, -26.664, 17.776, -1.936, 0.0, -22.352, -1.408, 0.616, 1.144, -12.584, -4.664, 81.312, -27.368, 65.736, 55.0],
            [99.528, 3.168, 48.4, 35.904, -2.376, 0.0, -22.352, -1.408, 0.792, 1.144, -12.584, -4.664, 82.984, -27.368, 68.992, 55.176],
            [100.056, 3.344, 48.136, 35.904, 106.744, -4.576, 17.6, 38.72, 3.432, 1.232, -12.584, -4.664, 83.688, -27.192, 69.344, 53.064],
            [99.968, 3.256, 48.224, 35.904, 106.656, -4.664, 17.6, 38.72, 104.632, -10.56, 39.072, 35.024, 83.688, -27.192, 69.256, 53.064],
            [95.744, 3.256, 48.4, 35.904, 21.56, 20.152, -17.072, -10.384, 21.472, -19.36, -24.376, 22.088, 83.688, -27.72, 69.256, 53.328],
            [8.184, 3.696, -18.568, 3.168, 95.04, -6.512, -3.344, -20.592, 15.136, -18.128, -24.464, 22.088, 98.384, -78.496, 38.632, -14.96],
            [70.752, 0.0, 24.728, 3.432, 12.144, 0.0, -4.048, -20.592, 14.608, 0.0, -24.464, 22.0, 111.672, -83.776, 40.216, -22.792],
            [4.752, 6.336, -26.664, 3.168, 13.816, -5.72, -26.84, -20.592, 83.952, -13.552, 5.368, 10.12, 82.456, -102.08, 44.616, -30.096],
            [80.0, 0.0, 30.0, 15.0, 80.0, 0.0, 30.0, 15.0, 80.0, 0.0, 30.0, 15.0, 100.0, -85.0, -10.0, 60.0], # GRASP
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]
    
    for positions_deg in positions_to_command_deg:
        leap_position_checker.publish_position(positions_deg)
        leap_position_checker.spin_for_duration(duration_sec=1.0) # Time for motor to reach position, and to subscribe from leap_node publisher
        position_comparison = [abs(a - b) for a, b in zip(positions_deg, leap_position_checker.feedback_position_deg)]
        print(position_comparison)
        comparison_result = [comparison < tolerance_deg for comparison in position_comparison]
        # comparison_result = [abs(a - b) for a, b in zip(truth_goal_position_deg, test_position_deg)]
        print(comparison_result)
        print("--------------------------")
        assert all(comparison_result)
