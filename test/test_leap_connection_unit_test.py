# test_example.py
import pytest
import launch_testing
import launch_testing.markers
import launch_ros
import rclpy
import os

import unittest

from launch import LaunchDescription
# from launch_ros.actions import LaunchNode
from rclpy.node import Node
from sensor_msgs.msg import JointState

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    os.environ['ROS_DOMAIN_ID'] = '42'

    return LaunchDescription([
        launch_ros.actions.Node(
            package='leap_ros2',  # Replace with your package name
            executable='leaphand_node.py',  # Replace with your node executable
            name='test_leap_node',
            output='screen'
        ),
        launch_testing.actions.ReadyToTest()
    ])
class TestSubscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber_node')
        self.subscription = self.create_subscription(
            JointState,
            '/leap/command_joint_states',
            self.listener_callback,
            10
        )
        self.received_message = None

    def listener_callback(self, msg):
        self.received_message = msg
class TestLeapHand(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = TestSubscriber()

    def tearDown(self):
        self.node.destroy_node()

    def test_publishes_pose(self):
        """Check whether pose messages published"""
        print("IN UNIT TEST!")
        rclpy.spin_once(self.node, timeout_sec=3.0)
        assert self.node.received_message is not None, "Cannot hear subscription from Leap Hand" 

        

# @launch_testing.post_shutdown_test()
# class TestShutdown(unittest.TestCase):
#     def test_exit_codes(self, proc_info):

#         processes = print(proc_info.processes())
#         """Check if the processes exited normally."""
#         # launch_testing.asserts.assertExitCodes(proc_info)
#         assert True
#         # I cant get away from this because the node will be killed everytime (in a not graceful way, so this assertion will always fail)
