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

# # Install Python scripts in the tests directory
# file(GLOB SCRIPT_FILES test/*.py)
# install(PROGRAMS ${SCRIPT_FILES}
#   DESTINATION lib/${PROJECT_NAME}
# )

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
    test/test_u2d2_connection.py
    test/test_leap_motor_connection.py
    test/test_leap_registers.py
    test/test_leap_connection_topics.py
    test/test_leap_launch_params.py
    test/test_leap_end_to_end.py
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 20
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()

  # find_package(ament_cmake_ros REQUIRED)
  # find_package(launch_testing_ament_cmake REQUIRED)
  # add_launch_test(test/test_leap_connection_unit_test.py)
  # add_launch_test(test/test_leap_connection_topics.py)


  # function(add_ros_isolated_launch_test path)
  #   set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
  #   add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  # endfunction()
  # add_ros_isolated_launch_test(test/test_leap_connection_unit_test.py)

endif()
