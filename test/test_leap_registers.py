#!/usr/bin/env python3
import numpy as np
import time
import pytest

# Test Parameters
position_mode_enum = 3
current_mode_enum = 0
current_based_position_enum = 5
baud_rate_3M_enum = 5 # Equivalent to 3M [bps]
baud_rate_2M_enum = 4 # Equivalent to 2M [bps]
baud_rate_1M_enum = 3 # Equivalent to 1M [bps]

test_delay = 2.0

# Helper Function
def command_and_check_position(dynamixel_manager, truth_goal_position_deg, tolerance_deg):
    dynamixel_manager.set_goal_position_deg(dynamixel_manager.motor_ids, truth_goal_position_deg)
    time.sleep(2.0)
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

def test_reboot(dynamixel_manager):
    truth_reboot = np.ones(len(dynamixel_manager.motor_ids)) # Will return 0 
    test_reboot = dynamixel_manager.reboot_motors(dynamixel_manager.motor_ids) # Will return all working motors

    errored_motors = [i for i, val in enumerate(test_reboot) if val is not True]
    if errored_motors:
        test_reboot = dynamixel_manager.reboot_motors(dynamixel_manager.motor_ids) # If there are any errors, check again if errors exist

    assert truth_reboot.tolist() == test_reboot
    time.sleep(test_delay)


def test_hardware_errors(dynamixel_manager):
    truth_errors = np.zeros(len(dynamixel_manager.motor_ids))
    test_errors = dynamixel_manager.get_hardware_error_status(dynamixel_manager.motor_ids)
    print(test_errors)
    assert truth_errors.tolist() == test_errors, "Hardware status not reading correctly!"
    time.sleep(test_delay)

# TODO: need to add setting dynamixel position min / max

def test_kP(dynamixel_manager):
    dynamixel_manager.set_torque_disable(dynamixel_manager.motor_ids)
    kP = 600
    truth_kP = np.ones(len(dynamixel_manager.motor_ids)) * kP
    dynamixel_manager.set_position_mode(dynamixel_manager.motor_ids)
    dynamixel_manager.set_kP(dynamixel_manager.motor_ids, truth_kP)
    test_kP = dynamixel_manager.get_kP(dynamixel_manager.motor_ids)
    assert truth_kP.tolist() == test_kP, "kP values not set correctly"
    time.sleep(test_delay)

def test_kI(dynamixel_manager):
    dynamixel_manager.set_torque_disable(dynamixel_manager.motor_ids)
    kI = 0
    truth_kI = np.ones(len(dynamixel_manager.motor_ids)) * kI
    dynamixel_manager.set_position_mode(dynamixel_manager.motor_ids)
    dynamixel_manager.set_kI(dynamixel_manager.motor_ids, truth_kI)
    test_kI = dynamixel_manager.get_kI(dynamixel_manager.motor_ids)
    assert truth_kI.tolist() == test_kI, "kI values not set correctly"
    time.sleep(test_delay)

def test_kD(dynamixel_manager):
    dynamixel_manager.set_torque_disable(dynamixel_manager.motor_ids)
    kD = 200
    truth_kD = np.ones(len(dynamixel_manager.motor_ids)) * kD
    dynamixel_manager.set_position_mode(dynamixel_manager.motor_ids)
    dynamixel_manager.set_kD(dynamixel_manager.motor_ids, truth_kD)
    test_kD = dynamixel_manager.get_kD(dynamixel_manager.motor_ids)
    assert truth_kD.tolist() == test_kD, "kD values not set correctly"
    time.sleep(test_delay)

# TODO: this test is a little weird because if I change the baudrate in dynamixel wizard (which is taken from the config right now, it will no longer connect)
def test_baud_rate(dynamixel_manager):
    dynamixel_manager.set_torque_disable(dynamixel_manager.motor_ids)
    truth_baud_rate = np.ones(len(dynamixel_manager.motor_ids)) * baud_rate_3M_enum
    dynamixel_manager.set_baud_rate(dynamixel_manager.motor_ids, truth_baud_rate)
    test_baud_rate = dynamixel_manager.get_baud_rate(dynamixel_manager.motor_ids)
    assert truth_baud_rate.tolist() == test_baud_rate, "baud_rate not set correctly"
    time.sleep(test_delay)

def test_position_mode_enable(dynamixel_manager):
    dynamixel_manager.set_torque_disable(dynamixel_manager.motor_ids)
    truth_operating_mode = np.ones(len(dynamixel_manager.motor_ids)) * position_mode_enum # Position Mode
    dynamixel_manager.set_position_mode(dynamixel_manager.motor_ids)
    test_operating_mode = dynamixel_manager.get_operating_mode(dynamixel_manager.motor_ids)
    assert truth_operating_mode.tolist() == test_operating_mode
    time.sleep(test_delay)

def test_current_mode_enable(dynamixel_manager):
    dynamixel_manager.set_torque_disable(dynamixel_manager.motor_ids)
    truth_operating_mode = np.ones(len(dynamixel_manager.motor_ids)) * current_mode_enum # current-based position control mode
    dynamixel_manager.set_current_mode(dynamixel_manager.motor_ids)
    dynamixel_manager.set_torque_enable(dynamixel_manager.motor_ids)
    test_operating_mode = dynamixel_manager.get_operating_mode(dynamixel_manager.motor_ids)
    assert truth_operating_mode.tolist() == test_operating_mode
    time.sleep(test_delay)

def test_current_based_position_mode_enable(dynamixel_manager):
    dynamixel_manager.set_torque_disable(dynamixel_manager.motor_ids)
    truth_operating_mode = np.ones(len(dynamixel_manager.motor_ids)) * current_based_position_enum # current-based position control mode
    dynamixel_manager.set_current_based_position_mode(dynamixel_manager.motor_ids)
    dynamixel_manager.set_torque_enable(dynamixel_manager.motor_ids)
    test_operating_mode = dynamixel_manager.get_operating_mode(dynamixel_manager.motor_ids)
    assert truth_operating_mode.tolist() == test_operating_mode
    time.sleep(test_delay)

def test_current_limit(dynamixel_manager):
    dynamixel_manager.set_torque_disable(dynamixel_manager.motor_ids)
    curr_lim = 100 # somewhat arbitrary
    dynamixel_manager.set_current_based_position_mode(dynamixel_manager.motor_ids)
    truth_current_limit = np.ones(len(dynamixel_manager.motor_ids)) * curr_lim
    dynamixel_manager.set_current_limit(dynamixel_manager.motor_ids, truth_current_limit) # Set above! 
    dynamixel_manager.set_torque_enable(dynamixel_manager.motor_ids)
    test_current_limit = dynamixel_manager.get_current_limit(dynamixel_manager.motor_ids)
    assert truth_current_limit.tolist() == test_current_limit, "current_limit is not set correctly!"
    time.sleep(test_delay)

def test_max_current_limit(dynamixel_manager):
    dynamixel_manager.set_torque_disable(dynamixel_manager.motor_ids)
    max_curr_limit = dynamixel_manager.max_curr_limit
    dynamixel_manager.set_current_based_position_mode(dynamixel_manager.motor_ids)
    truth_current_limit = np.ones(len(dynamixel_manager.motor_ids)) * max_curr_limit
    dynamixel_manager.set_current_limit(dynamixel_manager.motor_ids, truth_current_limit) # Set above limit!
    dynamixel_manager.set_torque_enable(dynamixel_manager.motor_ids)
    test_current_limit = dynamixel_manager.get_current_limit(dynamixel_manager.motor_ids)
    assert truth_current_limit.tolist() == test_current_limit, "current_limit did not sature correctly"
    time.sleep(test_delay)

def test_current(dynamixel_manager):
    dynamixel_manager.set_torque_disable(dynamixel_manager.motor_ids)
    dynamixel_manager.set_current_based_position_mode(dynamixel_manager.motor_ids)

    # Test no current to start!
    test_current = dynamixel_manager.get_current(dynamixel_manager.motor_ids)
    print(test_current)
    assert all([abs(val) <= 10.0 for val in test_current]), "current is greater than resting tolerance" # arbitrary tolerance, based on resting output of motors

    # Test that current reads less than motors rated maximum limit!
    
    # TODO: this could be optimized by actually testing with torque control, because unless I set the goal current in 
    # current_based_position_control, then it actually does not put a hard limit on the current. But it will be less than the maximum...
    max_curr_limit = dynamixel_manager.max_curr_limit
    dynamixel_manager.set_torque_enable(dynamixel_manager.motor_ids)
    goal_position_deg = [44.704, 3.784, 32.912, 21.384, 47.168, 4.4, 32.736, 14.432, 50.776, 3.52, 30.536, 18.216, 38.544, 1.056, 19.976, 17.776] # REST
    dynamixel_manager.set_goal_position_deg(dynamixel_manager.motor_ids, goal_position_deg)
    time.sleep(2) # let the dynamixel move to the position
    test_current = dynamixel_manager.get_current(dynamixel_manager.motor_ids)
    print(test_current)
    assert all([abs(val) <= max_curr_limit for val in test_current]), "current is less than +- motors maximum current limit"
    time.sleep(test_delay)

def test_goal_current(dynamixel_manager):
    dynamixel_manager.set_torque_disable(dynamixel_manager.motor_ids)
    dynamixel_manager.set_current_mode(dynamixel_manager.motor_ids)
    dynamixel_manager.set_torque_enable(dynamixel_manager.motor_ids)
    truth_goal_current_mA = [50, -50, 50, -50, 30, 20, 10, 15, 32, -35, -15, 28, 14, 12, 1, 0]
    dynamixel_manager.set_goal_current_mA(dynamixel_manager.motor_ids, truth_goal_current_mA)
    time.sleep(2.0)
    test_goal_current_mA = dynamixel_manager.get_goal_current_mA(dynamixel_manager.motor_ids)
    print(test_goal_current_mA)
    assert truth_goal_current_mA == test_goal_current_mA

def test_velocity(dynamixel_manager):
    dynamixel_manager.set_torque_disable(dynamixel_manager.motor_ids)
    time.sleep(5.0) # wait for motors to stop moving
    # TODO: could do more work to see if values are changing when moving around...
    test_velocity = dynamixel_manager.get_velocity(dynamixel_manager.motor_ids)
    print(test_velocity)
    assert all([abs(val) <= 2.0 for val in test_velocity]), "velocity is practically 0"

def test_torque_enable(dynamixel_manager):
    truth_torque_enable = np.zeros(len(dynamixel_manager.motor_ids))
    dynamixel_manager.set_torque_disable(dynamixel_manager.motor_ids)
    test_torque_enable = dynamixel_manager.get_torque_enable(dynamixel_manager.motor_ids)
    assert truth_torque_enable.tolist() == test_torque_enable # TORQUE OFF

    truth_torque_enable = np.ones(len(dynamixel_manager.motor_ids))
    dynamixel_manager.set_torque_enable(dynamixel_manager.motor_ids)
    test_torque_enable = dynamixel_manager.get_torque_enable(dynamixel_manager.motor_ids)
    assert truth_torque_enable.tolist() == test_torque_enable # TORQUE ON
    time.sleep(test_delay)

@pytest.mark.parametrize(
    "truth_goal_position_deg", 
    [
        [44.704, 3.784, 32.912, 21.384, 47.168, 4.4, 32.736, 14.432, 50.776, 3.52, 30.536, 18.216, 38.544, 1.056, 19.976, 17.776], # REST
        [10.0, 3.784, 10.0, 10.0, 47.168, 4.4, 32.736, 14.432, 50.776, 3.52, 30.536, 18.216, 38.544, 1.056, 19.976, 17.776], # PINKY UP
        [44.704, 3.784, 32.912, 21.384, 47.168, 4.4, 32.736, 14.432, 50.776, 3.52, 30.536, 18.216, 38.544, 1.056, 19.976, 17.776], # REST
        [44.704, 3.784, 32.912, 21.384, 10.0, 4.4, 10.0, 10.0, 50.776, 3.52, 30.536, 18.216, 38.544, 1.056, 19.976, 17.776], # MIDDLE UP
        [44.704, 3.784, 32.912, 21.384, 47.168, 4.4, 32.736, 14.432, 50.776, 3.52, 30.536, 18.216, 38.544, 1.056, 19.976, 17.776], # REST
        [44.704, 3.784, 32.912, 21.384, 47.168, 4.4, 32.736, 14.432, 10.0, 3.52, 10.0, 10.0, 38.544, 1.056, 19.976, 17.776], # INDEX UP
        [44.704, 3.784, 32.912, 21.384, 47.168, 4.4, 32.736, 14.432, 50.776, 3.52, 30.536, 18.216, 38.544, 1.056, 19.976, 17.776], # REST
        [44.704, 3.784, 32.912, 21.384, 47.168, 4.4, 32.736, 14.432, 50.776, 3.52, 30.536, 18.216, 10.0, 1.056, 10.0, 10.0], # THUMB UP
        [44.704, 3.784, 32.912, 21.384, 47.168, 4.4, 32.736, 14.432, 50.776, 3.52, 30.536, 18.216, 38.544, 1.056, 19.976, 17.776], # REST
    ]
)

def test_position(dynamixel_manager, truth_goal_position_deg):
    # Test Parameters
    kP_gains = np.ones(len(dynamixel_manager.motor_ids)) * 400
    kI_gains = np.ones(len(dynamixel_manager.motor_ids)) * 0
    kD_gains = np.ones(len(dynamixel_manager.motor_ids)) * 100
    tolerance_deg = 10.0 # how much allowable error there can be (was never given any requirements...)

    dynamixel_manager.set_torque_disable(dynamixel_manager.motor_ids)
    dynamixel_manager.set_position_mode(dynamixel_manager.motor_ids)
    dynamixel_manager.set_torque_enable(dynamixel_manager.motor_ids)
    dynamixel_manager.initialize_gains(dynamixel_manager.motor_ids, kP_gains, kI_gains, kD_gains)

    # Command Position!
    assert command_and_check_position(dynamixel_manager, truth_goal_position_deg, tolerance_deg)
    time.sleep(0.5)

@pytest.mark.parametrize(
    "truth_goal_position_deg", 
    [
        [44.704, 3.784, 32.912, 21.384, 47.168, 4.4, 32.736, 14.432, 50.776, 3.52, 30.536, 18.216, 38.544, 1.056, 19.976, 17.776], # REST
        [10.0, 3.784, 10.0, 10.0, 47.168, 4.4, 32.736, 14.432, 50.776, 3.52, 30.536, 18.216, 38.544, 1.056, 19.976, 17.776], # PINKY UP
        [44.704, 3.784, 32.912, 21.384, 47.168, 4.4, 32.736, 14.432, 50.776, 3.52, 30.536, 18.216, 38.544, 1.056, 19.976, 17.776], # REST
        [44.704, 3.784, 32.912, 21.384, 10.0, 4.4, 10.0, 10.0, 50.776, 3.52, 30.536, 18.216, 38.544, 1.056, 19.976, 17.776], # MIDDLE UP
        [44.704, 3.784, 32.912, 21.384, 47.168, 4.4, 32.736, 14.432, 50.776, 3.52, 30.536, 18.216, 38.544, 1.056, 19.976, 17.776], # REST
        [44.704, 3.784, 32.912, 21.384, 47.168, 4.4, 32.736, 14.432, 10.0, 3.52, 10.0, 10.0, 38.544, 1.056, 19.976, 17.776], # INDEX UP
        [44.704, 3.784, 32.912, 21.384, 47.168, 4.4, 32.736, 14.432, 50.776, 3.52, 30.536, 18.216, 38.544, 1.056, 19.976, 17.776], # REST
        [44.704, 3.784, 32.912, 21.384, 47.168, 4.4, 32.736, 14.432, 50.776, 3.52, 30.536, 18.216, 10.0, 1.056, 10.0, 10.0], # THUMB UP
        [44.704, 3.784, 32.912, 21.384, 47.168, 4.4, 32.736, 14.432, 50.776, 3.52, 30.536, 18.216, 38.544, 1.056, 19.976, 17.776], # REST
    ]
)

def test_current_based_position(dynamixel_manager, truth_goal_position_deg):
    # Test Parameters
    kP_gains = np.ones(len(dynamixel_manager.motor_ids)) * 600
    kI_gains = np.ones(len(dynamixel_manager.motor_ids)) * 0
    kD_gains = np.ones(len(dynamixel_manager.motor_ids)) * 200
    tolerance_deg = 10.0 # how much allowable error there can be (was never given any requirements...)

    dynamixel_manager.set_torque_disable(dynamixel_manager.motor_ids)
    dynamixel_manager.set_current_based_position_mode(dynamixel_manager.motor_ids)
    dynamixel_manager.set_torque_enable(dynamixel_manager.motor_ids)
    dynamixel_manager.initialize_gains(dynamixel_manager.motor_ids, kP_gains, kI_gains, kD_gains)

    # Command Position!
    assert command_and_check_position(dynamixel_manager, truth_goal_position_deg, tolerance_deg)
    time.sleep(0.5)

