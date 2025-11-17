cmake_minimum_required(VERSION 3.16)
project(spider_ros_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_export_dependencies(rosidl_default_runtime)

include_directories(include
../../libs/lib/twai_proto)

add_library(twai_proto ../../libs/lib/twai_proto/twai_proto.cpp)
link_libraries(twai_proto)
ament_auto_add_library(
  spider_ros_control SHARED
  # ../odrive_base/src/epoll_event_loop.cpp
  # ../odrive_base/src/socket_can.cpp
  src/spider_hardware_interface.cpp
)

pluginlib_export_plugin_description_file(hardware_interface spider_hardware_interface.xml)

target_compile_features(spider_ros_control PRIVATE cxx_std_20)

install(TARGETS spider_ros_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY description/urdf config launch
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
