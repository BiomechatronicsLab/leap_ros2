import pytest
import launch_pytest
# import launch_testing
# import launch_testing.markers

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState

import launch_ros
import rclpy

import unittest
from launch import LaunchDescription
import time

# from launch_ros.actions import LaunchNode
from rclpy.node import Node

# GLOBAL VARIABLES
config_directory = os.path.join(get_package_share_directory("leap_ros2"), "config")
config_file_path = os.path.join(config_directory, "test_params.yaml")

class LeapMsgSubscriber(Node):
    def __init__(self, config_params):
        super().__init__("leap_msg_subscriber")
        self.subscription = self.create_subscription(
            JointState, config_params["joint_feedback_topic"], self.command_callback, 10
        )
        self.joint_state_msg = []

    def command_callback(self, msg):
        self.joint_state_msg = msg    

@pytest.fixture
def leap_msg_subscriber(config_params):
    leap_msg_node = LeapMsgSubscriber(config_params)
    yield leap_msg_node
    leap_msg_node.destroy_node()
    time.sleep(1.0)

@pytest.fixture(autouse=True, scope="session")
def initialize_rclpy():
    # Set an arbitrary ROS_DOMAIN_ID so that the test is performed without inteference
    os.environ["ROS_DOMAIN_ID"] = "42"

    rclpy.init()
    yield
    rclpy.shutdown()

@launch_pytest.fixture
def launch_leap_ros2_node_no_pub_current(config_params):
    config_params["pub_current"] = False
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="leap_ros2",
                executable="leaphand_node.py",
                name="test_leap_node",
                output="screen",
                parameters=[config_params],
            ),
            launch_pytest.actions.ReadyToTest(),
        ]
    )

@launch_pytest.fixture
def launch_leap_ros2_node_pub_current(config_params):
    config_params["pub_current"] = True
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="leap_ros2",
                executable="leaphand_node.py",
                name="test_leap_node",
                output="screen",
                parameters=[config_params],
            ),
            launch_pytest.actions.ReadyToTest(),
        ]
    )

@launch_pytest.fixture
def launch_leap_ros2_node_no_pub_pos(config_params):
    config_params["pub_pos"] = False
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="leap_ros2",
                executable="leaphand_node.py",
                name="test_leap_node",
                output="screen",
                parameters=[config_params],
            ),
            launch_pytest.actions.ReadyToTest(),
        ]
    )

@launch_pytest.fixture
def launch_leap_ros2_node_pub_pos(config_params):
    config_params["pub_pos"] = True
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="leap_ros2",
                executable="leaphand_node.py",
                name="test_leap_node",
                output="screen",
                parameters=[config_params],
            ),
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@launch_pytest.fixture
def launch_leap_ros2_node_pub_velocity(config_params):
    config_params["pub_vel"] = True
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="leap_ros2",
                executable="leaphand_node.py",
                name="test_leap_node",
                output="screen",
                parameters=[config_params],
            ),
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@launch_pytest.fixture
def launch_leap_ros2_node_no_pub_velocity(config_params):
    config_params["pub_vel"] = False
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="leap_ros2",
                executable="leaphand_node.py",
                name="test_leap_node",
                output="screen",
                parameters=[config_params],
            ),
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=launch_leap_ros2_node_no_pub_current)
def test_no_pub_current(leap_msg_subscriber):
    """Checks if a topic is being published or subscribed to"""
    rclpy.spin_once(leap_msg_subscriber, timeout_sec=3.0)
    print("JOINT STATE EFFORT MSG!: " + str(leap_msg_subscriber.joint_state_msg.effort.tolist()))

    assert not leap_msg_subscriber.joint_state_msg.effort, (
        f'Joint State Message is Publishing when it shouldnt be'
    )

@pytest.mark.launch(fixture=launch_leap_ros2_node_pub_current)
def test_pub_current(leap_msg_subscriber):
    """Checks if a topic is being published or subscribed to"""
    rclpy.spin_once(leap_msg_subscriber, timeout_sec=3.0)
    print("JOINT STATE EFFORT MSG!: " + str(leap_msg_subscriber.joint_state_msg.effort.tolist()))

    assert leap_msg_subscriber.joint_state_msg.effort, (
        f'Joint State Message is Publishing when it shouldnt be'
    )

@pytest.mark.launch(fixture=launch_leap_ros2_node_pub_pos)
def test_pub_pos(leap_msg_subscriber):
    """Checks if a topic is being published or subscribed to"""
    rclpy.spin_once(leap_msg_subscriber, timeout_sec=3.0)
    print("JOINT STATE Position MSG!: " + str(leap_msg_subscriber.joint_state_msg.position.tolist()))

    assert leap_msg_subscriber.joint_state_msg.position, (
        f'Joint State Message is Publishing when it shouldnt be'
    )

@pytest.mark.launch(fixture=launch_leap_ros2_node_no_pub_pos)
def test_no_pub_pos(leap_msg_subscriber):
    """Checks if a topic is being published or subscribed to"""
    rclpy.spin_once(leap_msg_subscriber, timeout_sec=3.0)
    print("JOINT STATE Position MSG!: " + str(leap_msg_subscriber.joint_state_msg.position.tolist()))

    assert not leap_msg_subscriber.joint_state_msg.position, (
        f'Joint State Message is Publishing when it shouldnt be'
    )

@pytest.mark.launch(fixture=launch_leap_ros2_node_pub_velocity)
def test_pub_velocity(leap_msg_subscriber):
    """Checks if a topic is being published or subscribed to"""
    rclpy.spin_once(leap_msg_subscriber, timeout_sec=3.0)
    print("JOINT STATE Position MSG!: " + str(leap_msg_subscriber.joint_state_msg.velocity.tolist()))

    assert leap_msg_subscriber.joint_state_msg.velocity, (
        f'Joint State Message is Publishing when it shouldnt be'
    )

@pytest.mark.launch(fixture=launch_leap_ros2_node_no_pub_velocity)
def test_no_pub_velocity(leap_msg_subscriber):
    """Checks if a topic is being published or subscribed to"""
    rclpy.spin_once(leap_msg_subscriber, timeout_sec=3.0)
    print("JOINT STATE Position MSG!: " + str(leap_msg_subscriber.joint_state_msg.velocity.tolist()))

    assert not leap_msg_subscriber.joint_state_msg.velocity, (
        f'Joint State Message is Publishing when it shouldnt be'
    )

