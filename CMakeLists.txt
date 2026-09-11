cmake_minimum_required(VERSION 3.8)
project(fr3_lab_stack)

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch rviz docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE NOTICE DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/check_rgb_rates.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/fr3_joint_target.py DESTINATION lib/${PROJECT_NAME}
  RENAME fr3_joint_target)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(launch_contract test/test_launch_contract.py
    APPEND_ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs)
  ament_add_pytest_test(joint_target test/test_joint_target.py)
endif()

ament_package()
