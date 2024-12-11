import pytest
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from dynamixel_driver.XC330_M288_manager import XC330M288Manager
from dynamixel_driver.XL330_M288_manager import XL330M288Manager

config_directory = os.path.join(get_package_share_directory("leap_ros2"), "config")
config_file_path = os.path.join(config_directory, "test_params.yaml")


@pytest.fixture(scope="module")
def config_params():
    print(f"Loading configuration from: {config_file_path}")

    with open(config_file_path, "r") as file:
        return yaml.safe_load(file)
    
@pytest.fixture(scope="module")
def dynamixel_manager(config_params):
    baud_rate = config_params["baud_rate"]
    device_name = config_params['device_name']
    kP = config_params["kP"]
    kI = config_params["kI"]
    kD = config_params["kD"]

    # TODO: do we parameterize this value? 
    motors_ids = list(range(16))
    if config_params["dynamixel_type"] == "XC330-M288":
        yield XC330M288Manager(motors_ids, baud_rate,
                                            device_name,
                                            kP,
                                            kI,
                                            kD,
                                            config_params["curr_lim"],
                                            )
    elif config_params["dynamixel_type"] == "XL330-M288":
        yield XL330M288Manager(motors_ids, baud_rate,
                                            device_name,
                                            kP,
                                            kI,
                                            kD,
                                            config_params["curr_lim"],
                                            )
        
