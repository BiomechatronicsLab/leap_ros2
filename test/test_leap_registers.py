#!/usr/bin/env python3
import numpy as np
import time
import pytest

# Test Parameters
position_mode_enum = 3
current_based_position_enum = 5
baud_rate_3M_enum = 5 # Equivalent to 3M [bps]
baud_rate_2M_enum = 4 # Equivalent to 2M [bps]
baud_rate_1M_enum = 3 # Equivalent to 1M [bps]

# Helper Function
def command_and_check_position(dynamixel_manager, truth_goal_position_deg, tolerance_deg):
    dynamixel_manager.set_goal_position_deg(dynamixel_manager.motor_ids, truth_goal_position_deg)
    time.sleep(0.5)
    test_position_deg = dynamixel_manager.get_position_deg(dynamixel_manager.motor_ids)
    print(truth_goal_position_deg)
    print(test_position_deg)
    position_comparison = [abs(a - b) for a, b in zip(truth_goal_position_deg, test_position_deg)]
    print(position_comparison)
    comparison_result = [comparison < tolerance_deg for comparison in position_comparison]
    # comparison_result = [abs(a - b) for a, b in zip(truth_goal_position_deg, test_position_deg)]
    print(comparison_result)
    print("--------------------------")
    return all(comparison_result)

def test_kP(dynamixel_manager):
    kP = 600
    truth_kP = np.ones(len(dynamixel_manager.motor_ids)) * kP
    dynamixel_manager.set_operating_mode(dynamixel_manager.motor_ids, np.ones(len(dynamixel_manager.motor_ids)) * position_mode_enum)
    dynamixel_manager.set_kP(dynamixel_manager.motor_ids, truth_kP)
    test_kP = dynamixel_manager.get_kP(dynamixel_manager.motor_ids)
    assert truth_kP.tolist() == test_kP, "kP values not set correctly"
    time.sleep(0.5)

def test_kI(dynamixel_manager):
    kI = 0
    truth_kI = np.ones(len(dynamixel_manager.motor_ids)) * kI
    dynamixel_manager.set_operating_mode(dynamixel_manager.motor_ids, np.ones(len(dynamixel_manager.motor_ids)) * position_mode_enum)
    dynamixel_manager.set_kI(dynamixel_manager.motor_ids, truth_kI)
    test_kI = dynamixel_manager.get_kI(dynamixel_manager.motor_ids)
    assert truth_kI.tolist() == test_kI, "kI values not set correctly"
    time.sleep(0.5)

def test_kD(dynamixel_manager):
    kD = 200
    truth_kD = np.ones(len(dynamixel_manager.motor_ids)) * kD
    dynamixel_manager.set_operating_mode(dynamixel_manager.motor_ids, np.ones(len(dynamixel_manager.motor_ids)) * position_mode_enum)
    dynamixel_manager.set_kD(dynamixel_manager.motor_ids, truth_kD)
    test_kD = dynamixel_manager.get_kD(dynamixel_manager.motor_ids)
    assert truth_kD.tolist() == test_kD, "kD values not set correctly"
    time.sleep(0.5)

# TODO: this test is a little weird because if i change the baudrate (which is taken from the config right now, it will no longer connect)
def test_baud_rate(dynamixel_manager):
    truth_baud_rate = np.ones(len(dynamixel_manager.motor_ids)) * baud_rate_3M_enum
    dynamixel_manager.set_baud_rate(dynamixel_manager.motor_ids, truth_baud_rate)
    test_baud_rate = dynamixel_manager.get_baud_rate(dynamixel_manager.motor_ids)
    assert truth_baud_rate.tolist() == test_baud_rate, "baud_rate not set correctly"
    time.sleep(0.5)

def test_position_mode(dynamixel_manager):
    truth_operating_mode = np.ones(len(dynamixel_manager.motor_ids)) * position_mode_enum # Position Mode
    dynamixel_manager.set_operating_mode(dynamixel_manager.motor_ids, truth_operating_mode)
    test_operating_mode = dynamixel_manager.get_operating_mode(dynamixel_manager.motor_ids)
    assert truth_operating_mode.tolist() == test_operating_mode
    time.sleep(0.5)

def test_current_based_position_mode(dynamixel_manager):
    truth_operating_mode = np.ones(len(dynamixel_manager.motor_ids)) * current_based_position_enum # current-based position control mode
    dynamixel_manager.set_operating_mode(dynamixel_manager.motor_ids, truth_operating_mode)
    dynamixel_manager.set_torque_enable(dynamixel_manager.motor_ids, np.ones(len(dynamixel_manager.motor_ids)))
    test_operating_mode = dynamixel_manager.get_operating_mode(dynamixel_manager.motor_ids)
    assert truth_operating_mode.tolist() == test_operating_mode
    time.sleep(0.5)

def test_current_limit(dynamixel_manager):
    curr_lim = 1500 # arbitrary
    dynamixel_manager.set_operating_mode(dynamixel_manager.motor_ids, current_based_position_enum)
    truth_current_limit = np.ones(len(dynamixel_manager.motor_ids)) * curr_lim
    dynamixel_manager.set_current_limit(dynamixel_manager.motor_ids, truth_current_limit) # Set above! 
    dynamixel_manager.set_torque_enable(dynamixel_manager.motor_ids, np.ones(len(dynamixel_manager.motor_ids)))
    test_current_limit = dynamixel_manager.get_current_limit(dynamixel_manager.motor_ids)
    assert truth_current_limit.tolist() == test_current_limit, "current_limit is not set correctly!"
    time.sleep(0.5)

def test_max_current_limit(dynamixel_manager):
    max_curr_limit = dynamixel_manager.max_curr_limit
    dynamixel_manager.set_operating_mode(dynamixel_manager.motor_ids, current_based_position_enum)
    truth_current_limit = np.ones(len(dynamixel_manager.motor_ids)) * max_curr_limit
    dynamixel_manager.set_current_limit(dynamixel_manager.motor_ids, truth_current_limit) # Set above limit!
    dynamixel_manager.set_torque_enable(dynamixel_manager.motor_ids, np.ones(len(dynamixel_manager.motor_ids)))
    test_current_limit = dynamixel_manager.get_current_limit(dynamixel_manager.motor_ids)
    assert truth_current_limit.tolist() == test_current_limit, "current_limit did not sature correctly"
    time.sleep(0.5)

def test_current(dynamixel_manager):

    # Test no current to start!
    truth_current = np.zeros(len(dynamixel_manager.motor_ids))
    test_current = dynamixel_manager.get_current(dynamixel_manager.motor_ids)
    print(test_current)
    assert all([abs(val) <= 10.0 for val in test_current]), "current is greater than resting tolerance" # arbitrary tolerance, based on resting output of motors

    # Test that current reads less than motors rated maximum limit!
    # TODO: this could be optimized by actually testing with torque control, because unless I set the goal current in 
    # current_based_position_control, then it actually does not put a hard limit on the current. But it will be less than the maximum...

    dynamixel_manager.set_operating_mode(dynamixel_manager.motor_ids, np.ones(len(dynamixel_manager.motor_ids)) * current_based_position_enum)
    dynamixel_manager.set_torque_enable(dynamixel_manager.motor_ids, np.ones(len(dynamixel_manager.motor_ids)))
    max_curr_limit = dynamixel_manager.max_curr_limit
    goal_position_deg = np.ones(len(dynamixel_manager.motor_ids)) * 10.0
    dynamixel_manager.set_goal_position_deg(dynamixel_manager.motor_ids, goal_position_deg)
    time.sleep(2) # let the dynamixel move to the position
    test_current = dynamixel_manager.get_current(dynamixel_manager.motor_ids)
    print(test_current)
    assert all([abs(val) <= max_curr_limit for val in test_current]), "current is less than +- motors maximum current limit"
    time.sleep(0.5)

def test_velocity(dynamixel_manager):
    test_velocity = dynamixel_manager.get_velocity(dynamixel_manager.motor_ids)
    print(test_velocity)
    assert all([abs(val) <= 2.0 for val in test_velocity]), "velocity is practically 0"
    # dynamixel_manager.set_operating_mode(dynamixel_manager.motor_ids, np.ones(len(dynamixel_manager.motor_ids)) * position_mode_enum)
    # dynamixel_manager.set_torque_enable(dynamixel_manager.motor_ids, np.ones(len(dynamixel_manager.motor_ids)))

    # TODO: could do more work to see if values are changing when moving around...

def test_torque_enable(dynamixel_manager):
    truth_torque_enable = np.zeros(len(dynamixel_manager.motor_ids))
    dynamixel_manager.set_torque_enable(dynamixel_manager.motor_ids, truth_torque_enable)
    test_torque_enable = dynamixel_manager.get_torque_enable(dynamixel_manager.motor_ids)
    assert truth_torque_enable.tolist() == test_torque_enable # TORQUE OFF

    truth_torque_enable = np.ones(len(dynamixel_manager.motor_ids))
    dynamixel_manager.set_torque_enable(dynamixel_manager.motor_ids, truth_torque_enable)
    test_torque_enable = dynamixel_manager.get_torque_enable(dynamixel_manager.motor_ids)
    assert truth_torque_enable.tolist() == test_torque_enable # TORQUE ON
    time.sleep(0.5)

@pytest.mark.parametrize(
    "truth_goal_position_deg", 
    [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [-4.928, 4.136, -26.664, 17.776, -1.936, 0.0, -22.352, -1.408, 0.616, 1.144, -12.584, -4.664, 81.312, -27.368, 65.736, 55.0],
        [99.528, 3.168, 48.4, 35.904, -2.376, 0.0, -22.352, -1.408, 0.792, 1.144, -12.584, -4.664, 82.984, -27.368, 68.992, 55.176],
        [100.056, 3.344, 48.136, 35.904, 106.744, -4.576, 17.6, 38.72, 3.432, 1.232, -12.584, -4.664, 83.688, -27.192, 69.344, 53.064],
        [99.968, 3.256, 48.224, 35.904, 106.656, -4.664, 17.6, 38.72, 104.632, -10.56, 39.072, 35.024, 83.688, -27.192, 69.256, 53.064],
        [95.744, 3.256, 48.4, 35.904, 21.56, 20.152, -17.072, -10.384, 21.472, -19.36, -24.376, 22.088, 83.688, -27.72, 69.256, 53.328],
        [8.184, 3.696, -18.568, 3.168, 95.04, -6.512, -3.344, -20.592, 15.136, -18.128, -24.464, 22.088, 98.384, -78.496, 38.632, -14.96],
        [70.752, -20.328, 24.728, 3.432, 12.144, -6.424, -4.048, -20.592, 14.608, -18.128, -24.464, 22.0, 111.672, -83.776, 40.216, -22.792],
        [4.752, 6.336, -26.664, 3.168, 13.816, -5.72, -26.84, -20.592, 83.952, -13.552, 5.368, 10.12, 82.456, -102.08, 44.616, -30.096],
        [80.0, 0.0, 30.0, 15.0, 80.0, 0.0, 30.0, 15.0, 80.0, 0.0, 30.0, 15.0, 100, -85, -10, 60.0], # GRASP
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]
)

def test_position(dynamixel_manager, truth_goal_position_deg):
    # Test Parameters
    kP_gains = np.ones(len(dynamixel_manager.motor_ids)) * 600
    kI_gains = np.ones(len(dynamixel_manager.motor_ids)) * 0
    kD_gains = np.ones(len(dynamixel_manager.motor_ids)) * 200
    tolerance_deg = 10.0 # how much allowable error there can be (was never given any requirements...)

    dynamixel_manager.set_operating_mode(dynamixel_manager.motor_ids, np.ones(len(dynamixel_manager.motor_ids)) * position_mode_enum)
    dynamixel_manager.set_torque_enable(dynamixel_manager.motor_ids, np.ones(len(dynamixel_manager.motor_ids)))
    dynamixel_manager.initialize_gains(dynamixel_manager.motor_ids, kP_gains, kI_gains, kD_gains)

    # Command Position!
    assert command_and_check_position(dynamixel_manager, truth_goal_position_deg, tolerance_deg)
    time.sleep(0.5)

