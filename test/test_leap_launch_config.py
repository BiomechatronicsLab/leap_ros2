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
from ros2param.api import call_get_parameters


# GLOBAL VARIABLES
config_directory = os.path.join(get_package_share_directory('leap_ros2'), 'config')
config_file_path = os.path.join(config_directory, "test_params.yaml")
node_name = 'test_leap_node'

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
            name=node_name,
            output='screen',
            parameters=[config_params]
        ),
        launch_testing.actions.ReadyToTest()
    ])

# class ParameterChecker(Node):
#     def __init__(self):
#         super().__init__('parameter_checker')
#         node_name = "test_leap_node"
#         client = self.create_client(
#             rcl_interfaces.srv.GetParameters,
#             f'/{node_name}/get_parameters'
#         )

#         if not client.wait_for_service(timeout_sec=5.0):
#             self.get_logger().error(f"Service for {node_name} not available.")
#             return None

#     def topic_exists(self, topic_name):
#         topic_list = self.get_topic_names_and_types()
#         print(topic_list)
#         for topic, _ in topic_list:
#             if topic == topic_name:
#                 return True
#         return False

class TestLeapHand(unittest.TestCase):
    # @classmethod
    # def setUpClass(cls):
    #     rclpy.init()

    # @classmethod
    # def tearDownClass(cls):
    #     # Can hold onto nodes prior to shutdown if you want... 
    #     # input("Press Enter to shut down the node...")
    #     rclpy.shutdown()

    # def setUp(self):
    #     self.node = ParameterChecker()

    # def tearDown(self):
    #     self.node.destroy_node()

    def test_parameters(self):
        """Check if all parameters in the ros node are what is set in the config file"""
        config_params = load_yaml_file(config_file_path)
        print(config_params)
        for param_name in config_params:
            param_value = call_get_parameters(node_name, parameter_names=param_name)
            self.assertEquals(param_value, config_params[param_name], "Parameter in the config file does not equal that of the node")
            # topic_to_check = config_params["joint_command_topic"]
            # rclpy.spin_once(self.node, timeout_sec=3.0)
            # assert self.node.topic_exists(topic_to_check), "Cannot hear Topic:" + topic_to_check 

