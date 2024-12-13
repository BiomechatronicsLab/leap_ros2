import pytest
import launch_pytest
import os
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
import launch_ros
import rclpy
from launch import LaunchDescription
import time
from rclpy.node import Node

# GLOBAL VARIABLES
config_directory = os.path.join(get_package_share_directory("leap_ros2"), "config")
config_file_path = os.path.join(config_directory, "test_params.yaml")

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
def leap_msg_subscriber(config_params):
    leap_msg_node = LeapMsgSubscriber(config_params)
    yield leap_msg_node
    leap_msg_node.destroy_node()
    time.sleep(1.0)

@pytest.fixture(autouse=True, scope="session")
def initialize_rclpy():
    # Set an arbitrary ROS_DOMAIN_ID so that the test is performed without inteference
    os.environ["ROS_DOMAIN_ID"] = "42"

    rclpy.init()
    yield
    rclpy.shutdown()

@launch_pytest.fixture
def launch_leap_ros2_node_no_publishers(config_params):
    config_params["pub_current"] = False
    config_params["pub_pos"] = False
    config_params["pub_vel"] = False

    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="leap_ros2",
                executable="leaphand_node.py",
                name="test_leap_node",
                output="screen",
                parameters=[config_params],
            ),
            launch_pytest.actions.ReadyToTest(),
        ]
    )

@pytest.mark.launch(fixture=launch_leap_ros2_node_no_publishers)
def test_config_no_publishers(leap_msg_subscriber):

    """Checks if a topic is being published or subscribed to"""
    rclpy.spin_once(leap_msg_subscriber, timeout_sec=3.0)
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


@launch_pytest.fixture
def launch_leap_ros2_node_with_publishers(config_params):
    config_params["pub_current"] = True
    config_params["pub_pos"] = True
    config_params["pub_vel"] = True
    
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="leap_ros2",
                executable="leaphand_node.py",
                name="test_leap_node",
                output="screen",
                parameters=[config_params],
            ),
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=launch_leap_ros2_node_with_publishers)
def test_config_with_publishers(leap_msg_subscriber):

    """Checks if a topic is being published or subscribed to"""
    rclpy.spin_once(leap_msg_subscriber, timeout_sec=3.0)
    print("JOINT STATE EFFORT MSG!: " + str(leap_msg_subscriber.joint_state_msg.effort.tolist()))
    print("JOINT STATE POSITION MSG!: " + str(leap_msg_subscriber.joint_state_msg.position.tolist()))
    print("JOINT STATE VELOCITY MSG!: " + str(leap_msg_subscriber.joint_state_msg.velocity.tolist()))

    assert leap_msg_subscriber.joint_state_msg.effort, (
        f'Joint State Message is not Publishing Effort when should be'
    )

    assert leap_msg_subscriber.joint_state_msg.position, (
        f'Joint State Message is not Publishing Position when it should be'
    )

    assert leap_msg_subscriber.joint_state_msg.velocity, (
        f'Joint State Message is not Publishing Velocity when it should be'
    )



@launch_pytest.fixture
def launch_leap_ros2_node_PID_gains(config_params):
    config_params["kP"] = True
    config_params["kI"] = True
    config_params["kD"] = True
    
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="leap_ros2",
                executable="leaphand_node.py",
                name="test_leap_node",
                output="screen",
                parameters=[config_params],
            ),
            launch_pytest.actions.ReadyToTest(),
        ]
    )

@pytest.mark.launch(fixture=launch_leap_ros2_node_PID_gains)
def test_config_PID(leap_msg_subscriber):