cmake_minimum_required(VERSION 3.8)
project(fr3_lab_stack)

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch rviz docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE NOTICE DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/check_rgb_rates.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(launch_contract test/test_launch_contract.py
    APPEND_ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs)
endif()

ament_package()
