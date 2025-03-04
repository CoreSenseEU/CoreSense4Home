cmake_minimum_required(VERSION 3.8)
project(robocup_hri)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(whisper_msgs REQUIRED)
find_package(llama_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gpsr_msgs REQUIRED)
find_package(audio_common_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  rclcpp_cascade_lifecycle
  rclcpp_action
  behaviortree_cpp_v3
  audio_common_msgs
  whisper_msgs
  llama_msgs
  moveit_msgs
  shape_msgs
  std_msgs
  sensor_msgs
  gpsr_msgs
  std_srvs
  audio_common_msgs
)

include_directories(include)

add_library(speak_bt_node SHARED src/robocup_hri/dialog/Speak.cpp)
list(APPEND plugin_libs speak_bt_node)

add_library(dialogConfirmation_bt_node SHARED src/robocup_hri/dialog/DialogConfirmation.cpp)
list(APPEND plugin_libs dialogConfirmation_bt_node)

add_library(query_bt_node SHARED src/robocup_hri/dialog/Query.cpp)
list(APPEND plugin_libs query_bt_node)

add_library(listen_bt_node SHARED src/robocup_hri/dialog/Listen.cpp)
list(APPEND plugin_libs listen_bt_node)

add_library(generate_text_from_objects_bt_node SHARED src/robocup_hri/GenerateTextFromObjects.cpp)
list(APPEND plugin_libs generate_text_from_objects_bt_node)

add_library(store_detection_bt_node SHARED src/robocup_hri/dialog/store_detection.cpp)
list(APPEND plugin_libs store_detection_bt_node)

add_library(choose_from_classes_bt_node SHARED src/robocup_hri/dialog/choose_from_classes.cpp)
list(APPEND plugin_libs choose_from_classes_bt_node)

add_library(check_policy_bt_node SHARED src/robocup_hri/check_policy.cpp)
list(APPEND plugin_libs check_policy_bt_node)

add_library(command_planning_bt_node SHARED src/robocup_hri/CommandPlanning.cpp)
list(APPEND plugin_libs command_planning_bt_node)

add_library(start_music_bt_node SHARED src/robocup_hri/start_music.cpp)
list(APPEND plugin_libs start_music_bt_node)

add_library(stop_music_bt_node SHARED src/robocup_hri/stop_music.cpp)
list(APPEND plugin_libs stop_music_bt_node)

foreach(bt_plugin ${plugin_libs})
ament_target_dependencies(${bt_plugin} ${dependencies})
target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS
${plugin_libs}
EXPORT ${PROJECT_NAME}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
