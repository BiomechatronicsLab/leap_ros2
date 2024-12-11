#!/usr/bin/env python3
import unittest
import os
import yaml
from ament_index_python.packages import get_package_share_directory
import numpy as np
import time
import pytest


@pytest.mark.run(order=1)
def test_kP(dynamixel_manager, config_params):
    test_kP = dynamixel_manager.get_kP(dynamixel_manager.motor_ids)
    truth_kP = np.ones(len(dynamixel_manager.motor_ids)) * config_params["kP"]
    # truth_kP[[0, 4, 8]] = np.ones(3) * (self.config_params["kP"] * 0.75) 
    assert truth_kP.tolist() == test_kP, "kP values not set correctly"

@pytest.mark.run(order=2)
def test_kI(dynamixel_manager, config_params):
    test_kI = dynamixel_manager.get_kI(dynamixel_manager.motor_ids)
    truth_kI = np.ones(len(dynamixel_manager.motor_ids)) * config_params["kI"]
    # truth_kI[[0, 4, 8]] = np.ones(3) * (self.config_params["kI"] * 0.75) 
    assert truth_kI.tolist() == test_kI, "kI values not set correctly"

@pytest.mark.run(order=3)
def test_kD(dynamixel_manager, config_params):
    test_kD = dynamixel_manager.get_kD(dynamixel_manager.motor_ids)
    truth_kD = np.ones(len(dynamixel_manager.motor_ids)) * config_params["kD"]
    # truth_kD[[0, 4, 8]] = np.ones(3) * (self.config_params["kD"] * 0.75) 
    assert truth_kD.tolist() == test_kD, "kD values not set correctly"

@pytest.mark.run(order=4)
def test_baud_rate(dynamixel_manager):
    # truth_baud_rate = np.ones(len(.motor_ids)) * self.config_params["baud_rate"]

    # TODO: make this parameterized to the ENUM (as presented in the SDK)
    truth_baud_rate = np.ones(len(dynamixel_manager.motor_ids)) * 5
    test_baud_rate = dynamixel_manager.get_baud_rate(dynamixel_manager.motor_ids)
    assert truth_baud_rate.tolist() == test_baud_rate, "baud_rate not set correctly"

# TODO: need to create a test for making this torque mode too... 
@pytest.mark.run(order=5)
def test_current_limit(dynamixel_manager, config_params):
    truth_current_limit = np.ones(len(dynamixel_manager.motor_ids)) * config_params["curr_lim"]
    test_current_limit = dynamixel_manager.get_current_limit(dynamixel_manager.motor_ids)
    assert truth_current_limit.tolist() == test_current_limit, "current_limit is not set correctly"

# TODO: need to create a test for making this test position mode rigorously
@pytest.mark.run(order=6)
def test_position_mode(dynamixel_manager):
    truth_operating_mode = np.ones(len(dynamixel_manager.motor_ids)) * 3 # Position Mode
    test_operating_mode = dynamixel_manager.get_operating_mode(dynamixel_manager.motor_ids)
    assert truth_operating_mode.tolist() == test_operating_mode

# TODO: need to set torque enable on and off
@pytest.mark.run(order=7)
def test_torque_enable(dynamixel_manager):
    truth_torque_enable = np.ones(len(dynamixel_manager.motor_ids))
    test_torque_enable = dynamixel_manager.get_torque_enable(dynamixel_manager.motor_ids)
    assert truth_torque_enable.tolist() == test_torque_enable

@pytest.mark.run(order=8)
def test_position(dynamixel_manager):
    # Command Position!
    truth_goal_position_deg = np.ones(len(dynamixel_manager.motor_ids)) * 15.0
    print(truth_goal_position_deg)

    truth_goal_position_ticks = dynamixel_manager.degrees_to_ticks_list(truth_goal_position_deg.tolist())
    dynamixel_manager.set_goal_position(dynamixel_manager.motor_ids, truth_goal_position_ticks)

    time.sleep(3.0)
    # Read Position!
    test_position_ticks = dynamixel_manager.get_position(dynamixel_manager.motor_ids)
    test_position_deg = dynamixel_manager.ticks_to_degrees_list(test_position_ticks)
    print(test_position_deg)

    # Tolerance value
    tolerance = 5 # 
    # Compare element-wise and create boolean list based on the tolerance
    position_comparison = [abs(a - b) for a, b in zip(truth_goal_position_deg, test_position_deg)]
    print(position_comparison)
    comparison_result = [comparison < tolerance for comparison in position_comparison]
    # comparison_result = [abs(a - b) for a, b in zip(truth_goal_position_deg, test_position_deg)]
    print(comparison_result)
    # Assert that all elements in comparison_result are True
    assert all(comparison_result) # This will pass only if all values are True

@pytest.mark.run(order=9)
def test_multiple_positions(dynamixel_manager):
    # Define positions to command
    positions_to_command = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [-4.928, 4.136, -26.664, 17.776, -1.936, 0.0, -22.352, -1.408, 0.616, 1.144, -12.584, -4.664, 81.312, -27.368, 65.736, 55.0],
        [99.528, 3.168, 48.4, 35.904, -2.376, 0.0, -22.352, -1.408, 0.792, 1.144, -12.584, -4.664, 82.984, -27.368, 68.992, 55.176],
        [100.056, 3.344, 48.136, 35.904, 106.744, -4.576, 17.6, 38.72, 3.432, 1.232, -12.584, -4.664, 83.688, -27.192, 69.344, 53.064],
        [99.968, 3.256, 48.224, 35.904, 106.656, -4.664, 17.6, 38.72, 104.632, -10.56, 39.072, 35.024, 83.688, -27.192, 69.256, 53.064],
        [95.744, 3.256, 48.4, 35.904, 21.56, 20.152, -17.072, -10.384, 21.472, -19.36, -24.376, 22.088, 83.688, -27.72, 69.256, 53.328],
        [8.184, 3.696, -18.568, 3.168, 95.04, -6.512, -3.344, -20.592, 15.136, -18.128, -24.464, 22.088, 98.384, -78.496, 38.632, -14.96],
        [70.752, -20.328, 24.728, 3.432, 12.144, -6.424, -4.048, -20.592, 14.608, -18.128, -24.464, 22.0, 111.672, -83.776, 40.216, -22.792],
        [4.752, 6.336, -26.664, 3.168, 13.816, -5.72, -26.84, -20.592, 83.952, -13.552, 5.368, 10.12, 82.456, -102.08, 44.616, -30.096],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ]

    for truth_goal_position_deg in positions_to_command:
        truth_goal_position_ticks = dynamixel_manager.degrees_to_ticks_list(truth_goal_position_deg)
        dynamixel_manager.set_goal_position(dynamixel_manager.motor_ids, truth_goal_position_ticks)
        time.sleep(1.0)
        test_position_ticks = dynamixel_manager.get_position(dynamixel_manager.motor_ids)
        test_position_deg = dynamixel_manager.ticks_to_degrees_list(test_position_ticks)

        # Tolerance value
        tolerance = 10  
        # Compare element-wise and create boolean list based on the tolerance
        position_comparison = [abs(a - b) for a, b in zip(truth_goal_position_deg, test_position_deg)]
        comparison_result = [comparison < tolerance for comparison in position_comparison]
        
        # hardware_status = self.dynamixel_manager.get_hardware_error_status(self.motor_ids)

        print(truth_goal_position_deg)
        print(test_position_deg)
        print(position_comparison)
        print(comparison_result)
        # print(hardware_status)
        print("------------------------------------")
        # Assert that all elements in comparison_result are True
        assert(all(comparison_result))  # This will pass only if all values are True