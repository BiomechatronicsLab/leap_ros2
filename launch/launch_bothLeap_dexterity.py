import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

def load_yaml_file(file_path):
    # Print the path to the YAML file
    print(f"Loading configuration from: {file_path}")

    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def launch_setup(context, *args, **kwargs):
    # Retrieve the path to the YAML files for left and right hands
    config_file_1_name = LaunchConfiguration('config_file_1').perform(context)
    config_file_2_name = LaunchConfiguration('config_file_2').perform(context)
    
    # Prepend the path to the config directory
    config_directory = os.path.join(get_package_share_directory('leap_ros2'), 'config')
    config_file_1_path = os.path.join(config_directory, config_file_1_name)
    config_file_2_path = os.path.join(config_directory, config_file_2_name)

    # Load parameters from the YAML files
    left_config_params = load_yaml_file(config_file_1_path)
    right_config_params = load_yaml_file(config_file_2_path)

    # Add hand_side information to each set of parameters
    left_config_params['hand_side'] = 'left'
    right_config_params['hand_side'] = 'right'

    return [
        # Node for the left hand
        Node(
            package='leap_ros2',
            executable='leaphand_node.py',
            name='leap_left_hand_node',
            parameters=[left_config_params]
        ),
        Node(
            package='leap_ros2',
            executable='demo_repeat_joint_data.py',
            name='demo_repeat_joint_data_left',
            parameters=[left_config_params]
        ),
        # Node for the right hand
        Node(
            package='leap_ros2',
            executable='leaphand_node.py',
            name='leap_right_hand_node',
            parameters=[right_config_params]
        ),
        Node(
            package='leap_ros2',
            executable='demo_repeat_joint_data.py',
            name='demo_repeat_joint_data_right',
            parameters=[right_config_params]
        ),
    ]

def generate_launch_description():
    # Declare the YAML files for the left and right hands as launch arguments
    config_file_1_arg = DeclareLaunchArgument(
        'config_file_1',
        default_value='leap_left_1.yaml',  # Default file for left hand
        description='Name of the YAML config file for the left hand (located in the leap_ros2/config directory)'
    )

    config_file_2_arg = DeclareLaunchArgument(
        'config_file_2',
        default_value='leap_right_3.yaml',  # Default file for right hand
        description='Name of the YAML config file for the right hand (located in the leap_ros2/config directory)'
    )

    return LaunchDescription([
        config_file_1_arg,
        config_file_2_arg,
        OpaqueFunction(function=launch_setup)  # Ensure the config_file arguments are processed at runtime
    ])

if __name__ == '__main__':
    generate_launch_description()
