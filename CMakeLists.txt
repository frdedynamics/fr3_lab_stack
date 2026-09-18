cmake_minimum_required(VERSION 3.8)
project(fr3_lab_stack)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(controller_interface REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

ament_python_install_package(fr3_lab_stack_runtime)

add_library(fr3_lab_stack_controllers SHARED
  src/streaming_joint_impedance_controller.cpp
)
target_compile_features(fr3_lab_stack_controllers PUBLIC cxx_std_17)
target_include_directories(fr3_lab_stack_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(fr3_lab_stack_controllers
  controller_interface
  franka_semantic_components
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_msgs
)
pluginlib_export_plugin_description_file(controller_interface fr3_lab_stack_controllers.xml)

install(TARGETS fr3_lab_stack_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch rviz docs config DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE NOTICE DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/check_rgb_rates.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/fr3_joint_target.py DESTINATION lib/${PROJECT_NAME}
  RENAME fr3_joint_target)
install(PROGRAMS scripts/fr3_joint_target_server.py DESTINATION lib/${PROJECT_NAME}
  RENAME fr3_joint_target_server)
install(PROGRAMS scripts/fr3_streaming_joint_target_forwarder.py DESTINATION lib/${PROJECT_NAME}
  RENAME fr3_streaming_joint_target_forwarder)

install(PROGRAMS scripts/fr3_streaming_timing_collector.py DESTINATION lib/${PROJECT_NAME}
  RENAME fr3_streaming_timing_collector)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(timing_evidence test/test_timing_evidence.cpp)
  target_compile_features(timing_evidence PRIVATE cxx_std_17)
  target_include_directories(timing_evidence PRIVATE include)
  ament_target_dependencies(timing_evidence realtime_tools)
  ament_add_pytest_test(timing_analysis test/test_timing_evidence.py)

  ament_add_gtest(streaming_joint_impedance_core
    test/test_streaming_joint_impedance_core.cpp
  )
  target_include_directories(streaming_joint_impedance_core PRIVATE include)

  ament_add_gtest(hybrid_impedance_math
    test/test_hybrid_impedance_math.cpp
  )
  target_include_directories(hybrid_impedance_math PRIVATE
    include
    ${EIGEN3_INCLUDE_DIRS}
  )

  ament_add_pytest_test(launch_contract test/test_launch_contract.py
    APPEND_ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs)
  ament_add_pytest_test(franka_hand test/test_franka_hand.py)
  ament_add_pytest_test(joint_target test/test_joint_target.py)
  ament_add_pytest_test(joint_target_server test/test_joint_target_server.py)
  ament_add_pytest_test(streaming_joint_target_forwarder test/test_streaming_joint_target_forwarder.py
    APPEND_ENV ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1
      ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs)
endif()

ament_export_include_directories(include)
ament_export_libraries(fr3_lab_stack_controllers)
ament_export_dependencies(
  controller_interface
  franka_semantic_components
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_msgs
)

ament_package()
