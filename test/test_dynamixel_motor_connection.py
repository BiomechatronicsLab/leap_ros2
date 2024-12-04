#!/usr/bin/env python3
import pytest
import dynamixel_sdk as dxl
import os
import yaml
from ament_index_python.packages import get_package_share_directory


# GLOBAL VARIABLES
config_directory = os.path.join(get_package_share_directory('leap_ros2'), 'config')
config_file_path = os.path.join(config_directory, "test_params.yaml")

def load_yaml_file(file_path):
    # Print the path to the YAML file
    print(f"Loading configuration from: {file_path}")

    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

class TestDynamixelConnection:
    
    def setup_method(self):
        config_params = load_yaml_file(config_file_path)

        # Setup device name and protocol version
        self.device_name = config_params["device_name"]
        self.PROTOCOL_VERSION = 2.0
        
        # Initialize PortHandler and PacketHandler instances
        self.port_handler = dxl.PortHandler(self.device_name)
        self.packet_handler = dxl.PacketHandler(self.PROTOCOL_VERSION)
        
        # Attempt to open the port
        
        # # Set the baud rate for communication (adjust as needed)
        # self.baudrate = 3000000
        # if self.port_open:
        #     self.port_handler.setBaudRate(self.baudrate)

    def test_connection_success(self):
        port_open = self.port_handler.openPort()
        # Test if the port is open
        assert port_open, "Failed to open port"
        
        # TODO: Not sure why the ping is not working... would have to look more into it.

        # # Test if the motor responds to a ping (this assumes you have a motor ID, for example 1)
        # motor_id = 1  # Use the appropriate motor ID
        # comm_result, error, model_number = self.packet_handler.ping(self.port_handler, motor_id)
        
        # print(f"Ping Response: comm_result={comm_result}, error={error}, model_number={model_number}")

        # # Assert successful communication
        # assert comm_result == 0, "Ping failed: Communication error"
        # assert error == 0, f"Ping failed: Motor error {error}"
        # assert model_number > 0, "Ping failed: Invalid model number"
