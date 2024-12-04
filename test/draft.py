# test_example.py
import pytest
import launch_testing
import launch_pytest
from launch_pytest.tools import process as process_tools

import launch
import launch_ros
import rclpy


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
# from launch_ros.actions import LaunchNode
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState


    
@launch_pytest.fixture
def launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='leap_ros2',  # Replace with your package name
            executable='leaphand_node.py',  # Replace with your node executable
            name='test_leap_node',
            output='screen'
        ),
        launch_testing.actions.ReadyToTest()
    ])

class TestSubscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber_node')
        self.subscription = self.create_subscription(
            JointState,
            '/leap/command_joint_states',
            self.listener_callback,
            10
        )
        self.received_message = None

    def listener_callback(self, msg):
        self.received_message = msg.data

@pytest.mark.launch(fixture=launch_description)
def test_publisher_output():
    with rclpy.init():
        node = TestSubscriber()
        node.start_subscriber()
        rclpy.spin_once(node, timeout_sec=3.0)
        assert node.received_message is not None, "Cannot hear subscription from Leap Hand" 
        # msgs_received_flag = node.msg_event_object.wait(timeout=5.0)
        # assert msgs_received_flag, 'Did not receive msgs !'

    # with rclpy.init():
    #     # Start the node and subscribe to the topic
    #     test_subscriber = TestSubscriber()
    #     # launch_service.add_action(test_subscriber)

    #     # Give time for the node to publish a message
    #     rclpy.spin_once(test_subscriber, timeout_sec=3.0)

    #     # Check if the message was received
    #     assert test_subscriber.received_message is not None, "Cannot hear subscription from Leap Hand" 