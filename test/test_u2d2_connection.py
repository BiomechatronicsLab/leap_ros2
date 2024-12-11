#!/usr/bin/env python3

import pytest
import os
import yaml
import dynamixel_sdk as dxl
from ament_index_python.packages import get_package_share_directory


# GLOBAL VARIABLES
config_directory = os.path.join(get_package_share_directory('leap_ros2'), 'config')
config_file_path = os.path.join(config_directory, "test_params.yaml")

@pytest.fixture
def config_params():
    print(f"Loading configuration from: {config_file_path}")

    with open(config_file_path, "r") as file:
        return yaml.safe_load(file)


def test_port_connection(config_params):
    """Test to ensure successful connection to Dynamixel device."""
    try:
        # Initialize PortHandler and PacketHandler instances
        device_name = config_params['device_name']
        print(device_name)
        port_handler = dxl.PortHandler(device_name)

        # Attempt to open the port
        port_open = port_handler.openPort()

        # Verify that the port is open
        assert port_open, f"Failed to open port on device {device_name}"
                
        if port_handler:
            port_handler.closePort()

    except Exception as e:
        # Fail the test with the exception message
        pytest.fail(f"An exception occurred while trying to connect to the device: {e}", pytrace=True)

