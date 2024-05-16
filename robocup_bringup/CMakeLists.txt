cmake_minimum_required(VERSION 3.8)
project(robocup_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)

find_package(arm REQUIRED)
find_package(configuration REQUIRED)
find_package(hri REQUIRED)
find_package(motion REQUIRED)
find_package(perception REQUIRED)


set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  rclcpp_cascade_lifecycle
  behaviortree_cpp_v3
  arm
  configuration
  hri
  motion
  perception
)

include_directories(include)

add_executable(behaviors_main src/behaviors_main.cpp)
ament_target_dependencies(behaviors_main ${dependencies})


install(DIRECTORY bt_xml params
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch params maps DESTINATION share/${PROJECT_NAME})

install(TARGETS
  behaviors_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
