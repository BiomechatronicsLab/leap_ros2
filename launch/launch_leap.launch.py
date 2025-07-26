import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import time

def load_yaml_file(file_path):
    # Print the path to the YAML file
    print(f"Loading configuration from: {file_path}")

    with open(file_path, 'r') as file:
        return yaml.safe_load(file)
    
def launch_setup(context, *args, **kwargs):
    # Retrieve the path to the YAML file from the config_file argument
    config_file_name = LaunchConfiguration('config_file').perform(context)
    
    # Prepend the path to the config directory
    config_directory = os.path.join(get_package_share_directory('leap_ros2'), 'config')
    config_file_path = os.path.join(config_directory, config_file_name)

    # Load parameters from the YAML file
    config_params = load_yaml_file(config_file_path)
    # print(config_params)

    # TODO: parse which control mode it is, and then based on the control mode, load all the parameters into parameters as expected and send over to node
    general_params = config_params["general_params"]
    control_mode = general_params["control_mode"]

    if control_mode == "current_control":
        control_params = config_params["current_control_params"]
        leap_node_exec_name = "leap_current_control_driver.py"
        
    elif control_mode == "current_based_position_control": # Default to current_based_position_control_mode? 
        control_params = config_params["current_based_position_control_params"]
        leap_node_exec_name = "leap_current_position_control_driver.py"
    else:
        raise KeyError("Incorrect control mode specified!")
    general_params.update(control_params)

    return [
        Node(
            package='leap_ros2',
            executable=leap_node_exec_name,
            output="screen",
            emulate_tty=True,
            name='leap_node',
            parameters=[general_params]  # Pass the loaded parameters here
            
        ),
    ]

def generate_launch_description():
    # Declare the YAML file name as a launch argument
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='default_params.yaml',  # Default to this if not specified
        description='Name of the YAML config file (located in the leap_ros2/config directory)'
    )

    return LaunchDescription([
        config_file_arg,
        OpaqueFunction(function=launch_setup)  # Ensure the config_file argument is processed at runtime
    ])

if __name__ == '__main__':
    generate_launch_description()
