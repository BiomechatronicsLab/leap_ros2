#!/usr/bin/env python3
import pytest
from sensor_msgs.msg import JointState
import time
from rclpy.node import Node
from leap_ros2.leaphand_node import LeapHandNode 
from test_utilities.suspendable_thread import SuspendableThread 
from test_utilities.parameter_utility import ParameterSetter

from rcl_interfaces.msg import ParameterType
from rclpy.executors import SingleThreadedExecutor
import numpy as np

node_name = "test_leap_node"

# GLOBAL VARIABLES
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
def parameter_setter():
    parameter_setter = ParameterSetter(node_name)
    yield parameter_setter
    parameter_setter.destroy_node()

def spin_for_duration(executor, duration_sec):
    """Spin the given node for the specified duration in seconds."""
    start_time = time.time()
    while time.time() - start_time < duration_sec:
        spin_node(executor)  # Adjust timeout_sec for finer granularity if needed

def spin_node(executor):
    executor.spin_once(timeout_sec=0.01)

@pytest.fixture
def leap_msg_subscriber(config_params):
    leap_msg_node = LeapMsgSubscriber(config_params)
    yield leap_msg_node
    leap_msg_node.destroy_node()

@pytest.fixture
def leap_ros2_node_and_thread():
    leap_ros2_node = LeapHandNode(test_flag=True, node_name=node_name)
    executor = SingleThreadedExecutor()
    executor.add_node(leap_ros2_node)

    suspendable_thread = SuspendableThread(target=spin_node, args=(executor, ))
    suspendable_thread.start()

    yield leap_ros2_node, suspendable_thread
    
    executor.shutdown()
    leap_ros2_node.destroy_node()
    suspendable_thread.kill()

def test_leap_node_no_publisher_joint_state_msg(leap_msg_subscriber, leap_ros2_node_and_thread, parameter_setter, config_params):

    executor_test = SingleThreadedExecutor() # Will run two nodes sequentially (mutually exclusive)
    executor_test.add_node(leap_msg_subscriber)
    executor_test.add_node(parameter_setter)

    leap_ros2_node, _ = leap_ros2_node_and_thread

    # set the params you want
    config_params["pub_current"] = False
    config_params["pub_pos"] = False
    config_params["pub_vel"] = False

    config_param_keys = list(config_params.keys())
    parameter_setter.set_params(config_params, config_param_keys)

    # Spin subscriber and service to actually execute the service call
    spin_for_duration(executor_test, 1.0)

    # Setup the node so that it starts publishing with updated parameters
    leap_ros2_node.setup()
    print("LEAP NODE SETUP FINISHED!")

    # Spin subscriber and service to receive callbacks
    spin_for_duration(executor_test, 5.0)
    print("JOINT STATE EFFORT MSG!: " + str(leap_msg_subscriber.joint_state_msg.effort.tolist()))
    print("JOINT STATE POSITION MSG!: " + str(leap_msg_subscriber.joint_state_msg.position.tolist()))
    print("JOINT STATE VELOCITY MSG!: " + str(leap_msg_subscriber.joint_state_msg.velocity.tolist()))

    assert not leap_msg_subscriber.joint_state_msg.effort, (
        f'Joint State Message is Publishing Effort when it shouldnt be'
    )

    assert not leap_msg_subscriber.joint_state_msg.position, (
        f'Joint State Message is Publishing Position when it shouldnt be'
    )

    assert not leap_msg_subscriber.joint_state_msg.velocity, (
        f'Joint State Message is Publishing Velocity when it shouldnt be'
    )

    executor_test.shutdown()
    
def test_leap_node_publisher_joint_state_msg(leap_msg_subscriber, leap_ros2_node_and_thread, parameter_setter, config_params):

    leap_ros2_node, _ = leap_ros2_node_and_thread

    executor_test = SingleThreadedExecutor()
    executor_test.add_node(leap_msg_subscriber)
    executor_test.add_node(parameter_setter)

    # set the params you want
    config_params["pub_current"] = True
    config_params["pub_pos"] = True
    config_params["pub_vel"] = True

    config_param_keys = list(config_params.keys())
    parameter_setter.set_params(config_params, config_param_keys)

    # Spin subscriber and service to actually execute the service call
    spin_for_duration(executor_test, 1.0)

    # Setup the node so that it starts publishing with updated parameters
    leap_ros2_node.setup()
    print("LEAP NODE SETUP FINISHED!")

    # Spin subscriber and service to receive callbacks
    spin_for_duration(executor_test, 1.0)
    print("JOINT STATE EFFORT MSG!: " + str(leap_msg_subscriber.joint_state_msg.effort.tolist()))
    print("JOINT STATE POSITION MSG!: " + str(leap_msg_subscriber.joint_state_msg.position.tolist()))
    print("JOINT STATE VELOCITY MSG!: " + str(leap_msg_subscriber.joint_state_msg.velocity.tolist()))

    assert leap_msg_subscriber.joint_state_msg.effort, (
        f'Joint State Message is not Publishing Effort when it should be'
    )

    assert leap_msg_subscriber.joint_state_msg.position, (
        f'Joint State Message is not Publishing Position when it should be'
    )

    assert leap_msg_subscriber.joint_state_msg.velocity, (
        f'Joint State Message is not Publishing Velocity when it should be'
    )

    executor_test.shutdown()

def test_leap_node_gain_values(leap_ros2_node_and_thread, parameter_setter, config_params):

    leap_ros2_node, suspendable_thread = leap_ros2_node_and_thread

    executor_test = SingleThreadedExecutor()
    executor_test.add_node(parameter_setter)

    # set the params you want
    kP = 300
    kI = 10
    kD = 100 

    config_params["kP"] = kP
    config_params["kI"] = kI
    config_params["kD"] = kD

    config_param_keys = list(config_params.keys())
    parameter_setter.set_params(config_params, config_param_keys)

    # Spin subscriber and service to actually execute the service call
    spin_for_duration(executor_test, 1.0)

    # Setup the node so that it starts publishing with updated parameters
    leap_ros2_node.setup()
    print("LEAP NODE SETUP FINISHED!")

    # TODO: this probably could change depending on actual implementation
    truth_kP = np.ones(len(leap_ros2_node.dynamixel_mgr.motor_ids)) * kP
    truth_kP[[0, 4, 8]] = np.ones(3) * (kP * 0.75) 
    truth_kI = np.ones(len(leap_ros2_node.dynamixel_mgr.motor_ids)) * kI
    truth_kI[[0, 4, 8]] = np.ones(3) * (kI * 0.75) 
    truth_kD = np.ones(len(leap_ros2_node.dynamixel_mgr.motor_ids)) * kD
    truth_kD[[0, 4, 8]] = np.ones(3) * (kD * 0.75) 

    # Pause the leap_node from running so that I can check the actual driver information
    suspendable_thread.suspend()
    time.sleep(1.0)

    # Check the driver
    test_kP = leap_ros2_node.dynamixel_mgr.get_kP(leap_ros2_node.dynamixel_mgr.motor_ids)
    test_kI = leap_ros2_node.dynamixel_mgr.get_kI(leap_ros2_node.dynamixel_mgr.motor_ids)
    test_kD = leap_ros2_node.dynamixel_mgr.get_kD(leap_ros2_node.dynamixel_mgr.motor_ids)

    # have to convert to ints because thats how it is actually sent to the dynamixels
    assert truth_kP.astype(int).tolist() == test_kP
    assert truth_kI.astype(int).tolist() == test_kI
    assert truth_kD.astype(int).tolist() == test_kD
    executor_test.shutdown()


def test_leap_node_start_position(leap_ros2_node_and_thread, parameter_setter, config_params):
    leap_ros2_node, suspendable_thread = leap_ros2_node_and_thread

    executor_test = SingleThreadedExecutor()
    executor_test.add_node(parameter_setter)

    # set the initial conditions that you want (parameters much be a DOUBLE array)
    truth_start_pos_deg = [20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.00, 20.0, 20.0] # grasp
    config_params["start_pos_deg"] = truth_start_pos_deg

    config_param_keys = list(config_params.keys())
    parameter_setter.set_params(config_params, config_param_keys)

    # Spin subscriber and service to actually execute the service call
    spin_for_duration(executor_test, 1.0)

    # Setup the node so that it starts publishing with updated parameters
    leap_ros2_node.setup()
    print("LEAP NODE SETUP FINISHED!")
    
    # Pause the leap_node from running so that I can check the actual driver information
    suspendable_thread.suspend()
    time.sleep(1.0)
    test_start_pos_deg = leap_ros2_node.dynamixel_mgr.get_position_deg(leap_ros2_node.dynamixel_mgr.motor_ids)
    leap_ros2_node.dynamixel_mgr.set_goal_position_deg(leap_ros2_node.dynamixel_mgr.motor_ids, np.zeros(len(leap_ros2_node.dynamixel_mgr.motor_ids))) # just reset it back to home, not 100% necessary...

    # Now actually check without the node
    position_comparison = [abs(a - b) for a, b in zip(truth_start_pos_deg, test_start_pos_deg)]
    print(position_comparison)

    # tolerance of 15 degrees, etc (could be adjusted)
    comparison_result = [comparison < 15.0 for comparison in position_comparison]

    # have to convert to ints because thats how it is actually sent to the dynamixels
    assert all(comparison_result)
    executor_test.shutdown()

def test_leap_node_no_device_name(leap_ros2_node_and_thread, parameter_setter, config_params):
    leap_ros2_node, _ = leap_ros2_node_and_thread

    executor_test = SingleThreadedExecutor()
    executor_test.add_node(parameter_setter)

    # remove the device_name parameter to cause the exception to be raised!
    del config_params["device_name"] 

    config_param_keys = list(config_params.keys())
    parameter_setter.set_params(config_params, config_param_keys)

    # Spin subscriber and service to actually execute the service call
    spin_for_duration(executor_test, 1.0)

    # Assert that a ValueError is raised
    with pytest.raises(ValueError, match="Please state the device_name in the configuration file."):
        # Setup the node so that it starts publishing with updated parameters
        leap_ros2_node.setup()

    assert True
    executor_test.shutdown()

def test_leap_node_no_dynamixel_type(leap_ros2_node_and_thread, parameter_setter, config_params):
    leap_ros2_node, _ = leap_ros2_node_and_thread
    executor_test = SingleThreadedExecutor()
    executor_test.add_node(parameter_setter)

    # remove the device_name parameter to cause the exception to be raised!
    del config_params["dynamixel_type"] 

    config_param_keys = list(config_params.keys())
    parameter_setter.set_params(config_params, config_param_keys)

    # Spin subscriber and service to actually execute the service call
    spin_for_duration(executor_test, 1.0)

    # Assert that a ValueError is raised
    with pytest.raises(ValueError, match="Please state the dynamixel_type in the configuration file."):
        # Setup the node so that it starts publishing with updated parameters
        leap_ros2_node.setup()
        
    assert True
    executor_test.shutdown()
