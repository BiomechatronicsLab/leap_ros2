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

def extract_hand_number(config_file_name):
    # Extract hand number from the config file name
    # Assuming the config file format is leap_left_X.yaml or leap_right_X.yaml
    parts = config_file_name.split('_')
    if len(parts) > 2 and parts[2].isdigit():
        return parts[2]
    return "unknown"

def launch_setup(context, *args, **kwargs):
    # Initialize a list to hold node configurations
    nodes = []
    
    # Prepend the path to the config directory
    config_directory = os.path.join(get_package_share_directory('leap_ros2'), 'config')
    
    # Check for up to 5 config files
    for i in range(1, 6):  # Supports up to 5 config files
        config_file_name = LaunchConfiguration(f'config_file_{i}').perform(context)

        # Proceed only if the config file name is provided
        if config_file_name:
            config_file_path = os.path.join(config_directory, config_file_name)
            if os.path.isfile(config_file_path):  # Check if the file exists
                config_params = load_yaml_file(config_file_path)

                # Determine if the hand is left or right and extract the hand number
                if 'left' in config_file_name.lower():
                    config_params['hand_side'] = 'left'
                    hand_number = extract_hand_number(config_file_name)
                    node_name_suffix = hand_number
                else:
                    config_params['hand_side'] = 'right'
                    hand_number = extract_hand_number(config_file_name)
                    node_name_suffix = hand_number

                # Create the nodes for the current configuration
                nodes.append(
                    Node(
                        package='leap_ros2',
                        executable='leaphand_node.py',
                        name=f'leap_{config_params["hand_side"]}_hand_node_{node_name_suffix}',
                        parameters=[config_params]
                    )
                )

                nodes.append(
                    Node(
                        package='leap_ros2',
                        executable='demo_repeat_joint_data.py',
                        name=f'demo_repeat_joint_data_{config_params["hand_side"]}_{node_name_suffix}',
                        parameters=[config_params]
                    )
                )
            else:
                print(f"Warning: Configuration file {config_file_name} does not exist.")

    return nodes

def generate_launch_description():
    # Declare the YAML files as launch arguments
    launch_arguments = []
    
    for i in range(1, 6):  # Supports up to 5 config files
        launch_arguments.append(
            DeclareLaunchArgument(
                f'config_file_{i}',
                default_value='',
                description=f'Name of the YAML config file for hand {i} (located in the leap_ros2/config directory)'
            )
        )

    return LaunchDescription(
        launch_arguments + [OpaqueFunction(function=launch_setup)]
    )

if __name__ == '__main__':
    generate_launch_description()
