cmake_minimum_required(VERSION 3.8)
project(rbf_ntrip_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

include_directories(include)

# Add rbf_ntrip_driver.cpp as a rclcpp_component target
add_library(rbf_ntrip_driver_component SHARED
  src/rbf_ntrip_driver.cpp
  src/serial_port.cpp
  src/ntrip/ntrip_client.cpp
  src/ntrip/ntrip_util.cpp
)

ament_target_dependencies(rbf_ntrip_driver_component
  rclcpp
  rclcpp_components
  sensor_msgs
  mavros_msgs
  diagnostic_updater
)

rclcpp_components_register_node(
  rbf_ntrip_driver_component
  PLUGIN "rbf_ntrip_driver::NtripDriver"
  EXECUTABLE rbf_ntrip_driver
)

ament_export_targets(export_rbf_ntrip_driver_component)
install(TARGETS rbf_ntrip_driver_component
        EXPORT export_rbf_ntrip_driver_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)


install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
