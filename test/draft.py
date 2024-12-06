# test_example.py
import pytest
import launch_testing
import launch_testing.markers

import os
import yaml
from ament_index_python.packages import get_package_share_directory

import launch_ros
import rclpy

import unittest
from launch import LaunchDescription
# from launch_ros.actions import LaunchNode
from rclpy.node import Node

# GLOBAL VARIABLES
config_directory = os.path.join(get_package_share_directory('leap_ros2'), 'config')
config_file_path = os.path.join(config_directory, "test_params.yaml")

def load_yaml_file(file_path):
    # Print the path to the YAML file
    print(f"Loading configuration from: {file_path}")

    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():

    # Set an arbitrary ROS_DOMAIN_ID so that the test is performed without inteference
    os.environ['ROS_DOMAIN_ID'] = '42'
    # Load parameters from the YAML file
    config_params = load_yaml_file(config_file_path)

    return LaunchDescription([
        launch_ros.actions.Node(
            package='leap_ros2',  # Replace with your package name
            executable='leaphand_node.py',  # Replace with your node executable
            name='test_leap_node',
            output='screen',
            parameters=[config_params]
        ),
        launch_testing.actions.ReadyToTest()
    ])

class TopicChecker(Node):
    def __init__(self):
        super().__init__('topic_checker')
    
    def topic_exists(self, topic_name):
        topic_list = self.get_topic_names_and_types()
        print(topic_list)
        for topic, _ in topic_list:
            if topic == topic_name:
                return True
        return False

@pytest.fixture(scope="module", autouse=True)
def test_node():
    """Fixture to create and manage the TopicChecker node."""
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture(scope='function')
def create_node():
    """Fixture to create and manage the TopicChecker node."""
    node = TopicChecker()
    yield node
    node.destroy_node()
    rclpy.shutdown()

def test_publishes_pose(create_node):
    """Check whether leap hand positions are being published to"""
    print("TEST 1")
    node = create_node
    config_params = load_yaml_file(config_file_path)
    topic_to_check = config_params["joint_command_topic"]
    rclpy.spin_once(node, timeout_sec=3.0)
    assert node.topic_exists(topic_to_check), "Cannot hear Topic:" + topic_to_check 

def test_subscribes_pose(create_node):
    """Check whether leap hand requested positions are being subscribed to"""
    print("TEST 2")
    node = create_node
    config_params = load_yaml_file(config_file_path)
    topic_to_check = config_params["joint_feedback_topic"]
    rclpy.spin_once(node, timeout_sec=3.0)
    assert node.topic_exists(topic_to_check), "Cannot hear Topic:" + topic_to_check 