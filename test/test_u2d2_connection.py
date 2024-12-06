#!/usr/bin/env python3
import unittest
import os
import yaml
import dynamixel_sdk as dxl
from ament_index_python.packages import get_package_share_directory


# GLOBAL VARIABLES
config_directory = os.path.join(get_package_share_directory('leap_ros2'), 'config')
config_file_path = os.path.join(config_directory, "test_params.yaml")


def load_yaml_file(file_path):
    # Print the path to the YAML file
    print(f"Loading configuration from: {file_path}")
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)


class TestDynamixelConnection(unittest.TestCase):
    def setUp(self):
        # Load configuration parameters
        self.config_params = load_yaml_file(config_file_path)

        # Setup device name and protocol version
        self.device_name = self.config_params["device_name"]
        self.PROTOCOL_VERSION = 2.0

        # Initialize PortHandler and PacketHandler instances
        self.port_handler = dxl.PortHandler(self.device_name)
        self.packet_handler = dxl.PacketHandler(self.PROTOCOL_VERSION)

    def tearDown(self):
        # Close the port if it's open
        if self.port_handler:
            self.port_handler.closePort()

    def test_connection_success(self):
        # Attempt to open the port
        port_open = self.port_handler.openPort()

        print("TESTING!!!!")
        # Test if the port is open
        self.assertTrue(port_open, "Failed to open port")

        # Uncomment and implement ping testing if needed:
        # motor_id = 1  # Use the appropriate motor ID
        # comm_result, error, model_number = self.packet_handler.ping(self.port_handler, motor_id)
        # print(f"Ping Response: comm_result={comm_result}, error={error}, model_number={model_number}")
        # self.assertEqual(comm_result, 0, "Ping failed: Communication error")
        # self.assertEqual(error, 0, f"Ping failed: Motor error {error}")
        # self.assertGreater(model_number, 0, "Ping failed: Invalid model number")


if __name__ == '__main__':
    unittest.main()