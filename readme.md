
# Leap ROS2 Interface
- This repository contains code for running the [LEAP Hand](http://leaphand.com/).


## Table of Contents

- [Dependencies](#dependencies)
- [Setup Instructions](#setup-instructions)
- [Running the Leap Hand](#running-the-leap-hand)
- [Running Unit Tests](#running-unit-tests)
- [Debugging](#debugging)

## Dependencies
The table below lists the direct dependencies needed for this repository.

| **Dependency Name**                                                          | **Description**                                                                   |
|------------------------------------------------------------------------------|-----------------------------------------------------------------------------------|
| [dynamixel-sdk](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)                          | DYNAMIXEL SDK is a software development kit that provides DYNAMIXEL control functions using packet communication.       |
| [pytest](https://github.com/pytest-dev/pytest) | Python based unit test framework. |
| [launch_pytest](https://github.com/ros2/launch/tree/rolling/launch_pytest)       | Framework for launch integration testing with Pytest.     |
| [dynamixel_driver](https://github.com/BiomechatronicsLab/dynamixel_driver) | Python library which helps to manage, interface, and control multiple dynamixel motors across various packages. | 

## Setup Instructions

### Hardware Setup
1. Make sure that the power cable coming from the hand is connected to a power supply and that the power supply is turned on.
2. Plug Microusb into U2D2 and connect to computer.

### Software Setup
Follow these steps to set up the Leap Hand:

1. **Clone the Repository**:
   ```bash
   git clone git@github.com:BiomechatronicsLab/leap_ros2.git
   cd your-repo-directory
   ```

2. **Install Dependencies**:
   Ensure you have Python packages required for the project:
   ```bash
   pip install -r requirements.txt
   ```
   
3. **Compile the Package**:
   Build the package using `colcon`:
   ```bash
   colcon build --packages-select leap_ros2 --symlink-install
   ```

## Running the Leap Hand
1. **Create and Edit Configuration File**:
   Copy the default parameters:
   ```bash
   cp path/to/default_params.yaml path/to/your/config.yaml
   ```
   Modify `config.yaml` as needed to set your desired parameters.

2. **Launch the Driver**:
   Execute the following command, specifying the path to your configuration file:
   ```bash
   ros2 launch leap_ros2 launch_leap.py config_file:=leaphandName.yaml
   ```

## Running Unit Tests
This package utilizes `pytest` and [launch_pytest](https://github.com/ros2/launch/tree/rolling/launch_pytest) in order to run a suite of unit tests.
Make sure that `pytest` and `launch_pytest` is installed prior to running unit tests. List of unit tests can be found [here](https://docs.google.com/spreadsheets/d/1xNQxsC7j65EC7boTqYAhkELJ_jmE7E_Zx7XleKEM8Rc/edit?gid=1418432590#gid=1418432590)

1. **Remove old test results (if necessary)**:
   `rm -rf /build /install`
2. **Modify `config/test_params.yaml` (if needed)**:
   - The unit tests all draw from configurations set in `config/test_params.yaml`
3. **Build workspace**:
   `colcon build`
4. **Source workspace**:
   `source install/setup.bash`
5. **Run individual unit tests (if needed)**:
   - `pytest-3 -s src/leap_ros2/test/<test_name.py>`
6. **Run test suite from workspace**:
   `colcon test --event-handlers console_direct+ --packages-select leap_ros2`
7. **Process Tests**:
   `colcon test-result --all`
8. **Review Results**:
   `xunit-viewer -r build/leap_ros2/test_results -c`

## Debugging
- After plugging in the USB from the Leap into your computer, use the following if necessary to determine the serial ID associated with the appropriate hand: ```cd /dev/serial/by-id/ ```
- If your motor is 90/180/270 Degrees off, the horn is mounted incorrectly on the motor.  Remount it.
- If no motors show up, check that your serial port permissions are correct. Try this command: `sudo usermod -aG dialout $USER`
- If some motors are missing, make sure they are IDed corrrectly and are connected to the U2D2.
- If you get "overload error" and the motors are flashing red, then they have overloaded (self-collision etc). It should clear on a power cycle.  If it happens often, lower the current limits in the control code so that it does not happen as often.
- If you get "jittery" motors, try lowering the P and D values, either in the roslaunch file or the python file.
- If you feel the motors are too inaccurate, you can also try raising the P and D values.
- To improve latency on Ubuntu try these tips.   Configure [USB Latency Settings in Ubuntu](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) and the [Dynamixel Python SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK/issues/288)