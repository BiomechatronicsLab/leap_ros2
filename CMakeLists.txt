cmake_minimum_required(VERSION 3.5)
project(leap_ros2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#############
## Install ##
#############

# Install Python scripts in the scripts directory
file(GLOB SCRIPT_FILES scripts/*.py)
install(PROGRAMS ${SCRIPT_FILES}
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts in the tests directory
file(GLOB SCRIPT_FILES test/*.py)
install(PROGRAMS ${SCRIPT_FILES}
  DESTINATION lib/${PROJECT_NAME}
)

# Install all Python scripts in the launch directory (if any)
file(GLOB LAUNCH_SCRIPTS launch/*.py)
install(PROGRAMS ${LAUNCH_SCRIPTS}
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch" PATTERN "*.launch.py"
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)


###################################
## ament specific configuration  ##
###################################
ament_package()


########
# test #
########

if(BUILD_TESTING)
  # Python unit tests
  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests
    test/test_dynamixel_connection.py
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 20
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()

endif()
