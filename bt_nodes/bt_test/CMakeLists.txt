cmake_minimum_required(VERSION 3.8)
project(bt_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(whisper_msgs REQUIRED)
find_package(llama_msgs REQUIRED)
find_package(gpsr_msgs REQUIRED)
find_package(configuration REQUIRED)
find_package(motion REQUIRED)
find_package(perception REQUIRED)
find_package(arm REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robocup_hri REQUIRED)
find_package(perception_system_interfaces REQUIRED)
# find_package(backward_ros REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  rclcpp_cascade_lifecycle
  rclcpp_action
  behaviortree_cpp_v3
  audio_common_msgs
  whisper_msgs
  llama_msgs
  gpsr_msgs
  configuration
  motion
  perception
  tf2_ros
  robocup_hri
  arm
  geometry_msgs
  perception_system_interfaces
  # backward_ros
)

# Set WP test
add_executable(set_wp_test src/set_wp_test.cpp)
ament_target_dependencies(set_wp_test ${dependencies})
target_link_libraries(set_wp_test configuration::set_wp_bt_node)

# Look At test
add_executable(look_at_test src/look_at_test.cpp)
ament_target_dependencies(look_at_test ${dependencies})
target_link_libraries(look_at_test motion::look_at_bt_node)

# Move To test Real
add_executable(move_to_test src/move_to_test.cpp)
ament_target_dependencies(move_to_test ${dependencies})
target_link_libraries(move_to_test motion::move_to_bt_node)

# Move To test Simulated
add_executable(move_to_test_sim src/move_to_test_sim.cpp)
ament_target_dependencies(move_to_test_sim ${dependencies})
target_link_libraries(move_to_test_sim motion::move_to_bt_node)

# Set WP and MoveTo test
add_executable(setwp_and_moveto_test src/setwp_and_moveto_test.cpp)
ament_target_dependencies(setwp_and_moveto_test ${dependencies})
target_link_libraries(setwp_and_moveto_test configuration::set_wp_bt_node motion::move_to_bt_node)

# Is Detected test
add_executable(is_detected_test src/is_detected_test.cpp)
ament_target_dependencies(is_detected_test ${dependencies})
target_link_libraries(is_detected_test perception::is_detected_bt_node)

# Is Detected Real test
add_executable(is_detected_real_test src/is_detected_real_test.cpp)
ament_target_dependencies(is_detected_real_test ${dependencies})
target_link_libraries(is_detected_real_test perception::is_detected_bt_node)

# Is Detected Move To test
add_executable(isdetected_moveto_test src/isdetected_moveto_test.cpp)
ament_target_dependencies(isdetected_moveto_test ${dependencies})
target_link_libraries(isdetected_moveto_test perception::is_detected_bt_node motion::move_to_bt_node)

# Is Detected Move To Loop test
add_executable(is_detected_moveto_loop_test src/is_detected_moveto_loop_test.cpp)
ament_target_dependencies(is_detected_moveto_loop_test ${dependencies})
target_link_libraries(is_detected_moveto_loop_test perception::is_detected_bt_node motion::move_to_bt_node)

#Is Detected Move To and Attention - Follow test
add_executable(follow_test src/follow_test.cpp)
ament_target_dependencies(follow_test ${dependencies})
target_link_libraries(follow_test perception::is_detected_bt_node motion::move_to_bt_node motion::look_at_bt_node)

# Speak test
add_executable(speak_test src/speak_test.cpp)
ament_target_dependencies(speak_test ${dependencies})
target_link_libraries(speak_test robocup_hri::speak_bt_node robocup_hri::dialogConfirmation_bt_node)
# ExtractObjectFromScene test
add_executable(extract_object_from_scene_test src/extract_object_from_scene_test.cpp)
ament_target_dependencies(extract_object_from_scene_test ${dependencies})
target_link_libraries(extract_object_from_scene_test perception::extract_object_from_scene_bt_node)

# GenerateTextFromObjets test
add_executable(generate_text_from_objects_test src/generate_text_from_objects_test.cpp)
ament_target_dependencies(generate_text_from_objects_test ${dependencies})
target_link_libraries(generate_text_from_objects_test robocup_hri::generate_text_from_objects_bt_node)

# pick_demo test
add_executable(pick_demo_test src/pick_demo_test.cpp)
ament_target_dependencies(pick_demo_test ${dependencies})
target_link_libraries(pick_demo_test
  arm::pick_bt_node
  robocup_hri::generate_text_from_objects_bt_node
  perception::extract_object_from_scene_bt_node
  perception::extract_collision_scene_bt_node
  robocup_hri::speak_bt_node
  robocup_hri::dialogConfirmation_bt_node
)

# Ask test
add_executable(ask_test src/ask_test.cpp)
ament_target_dependencies(ask_test ${dependencies})
target_link_libraries(ask_test robocup_hri::query_bt_node robocup_hri::speak_bt_node robocup_hri::listen_bt_node robocup_hri::dialogConfirmation_bt_node)

# init_carry test
add_executable(init_carry src/init_carry_test.cpp)
ament_target_dependencies(init_carry ${dependencies})
target_link_libraries(configuration::init_carry_bt_node)

# follow_person test
add_executable(follow_person_test src/follow_person_test.cpp)
ament_target_dependencies(follow_person_test ${dependencies})
target_link_libraries(follow_person_test perception::follow_person_bt_node)

# find_person test
add_executable(find_person_test src/find_person_test.cpp)
ament_target_dependencies(find_person_test ${dependencies})
target_link_libraries(find_person_test
                      perception::convert_color_bt_node
                      perception::is_detected_bt_node
                      motion::pan_bt_node
                      motion::move_to_bt_node
                      perception::is_gesturing_bt_node)

# count person test
add_executable(count_people_test src/count_people_test.cpp)
ament_target_dependencies(count_people_test ${dependencies})
target_link_libraries(count_people_test perception::count_people_bt_node motion::rotate_bt_node)

# rotate test
add_executable(rotate_test src/rotate_test.cpp)
ament_target_dependencies(rotate_test ${dependencies})
target_link_libraries(rotate_test motion::rotate_bt_node)

# Change perception model test
add_executable(get_perception_model_test src/get_perception_model_test.cpp)
ament_target_dependencies(get_perception_model_test ${dependencies})
target_link_libraries(get_perception_model_test
  configuration::get_perception_model_bt_node
)

add_executable(set_perception_model_test src/set_perception_model_test.cpp)
ament_target_dependencies(set_perception_model_test ${dependencies})
target_link_libraries(set_perception_model_test
  configuration::set_perception_model_bt_node
)

# music tests
add_executable(music_test src/music_test.cpp)
ament_target_dependencies(music_test ${dependencies})
target_link_libraries(music_test 
  robocup_hri::start_music_bt_node
  robocup_hri::stop_music_bt_node
  configuration::sleep_bt_node
)

# motion action tests
add_executable(torsoheight_test src/torsoheight_test.cpp)
ament_target_dependencies(torsoheight_test ${dependencies})
target_link_libraries(torsoheight_test
  motion::set_torso_height_bt_node
)

add_executable(headposition_test src/headposition_test.cpp)
ament_target_dependencies(headposition_test ${dependencies})
target_link_libraries(headposition_test
  motion::set_head_joint_position_bt_node
)

# filter object by type, size and weight test
add_executable(filter_object_test src/filter_object_test.cpp)
ament_target_dependencies(filter_object_test ${dependecies})
target_link_libraries(filter_object_test perception::filter_object_bt_node)

# extract the description of the person test
add_executable(extract_person_description_test src/extract_person_description_test.cpp)
ament_target_dependencies(extract_person_description_test ${dependecies})
target_link_libraries(extract_person_description_test perception::extract_person_description_bt_node perception::is_detected_bt_node robocup_hri::speak_bt_node)

# gpsr test
add_executable(gpsr_test src/gpsr_test.cpp)
ament_target_dependencies(gpsr_test ${dependencies})
target_link_libraries(gpsr_test
perception::is_detected_bt_node
perception::extract_object_from_scene_bt_node
perception::convert_color_bt_node
perception::is_gesturing_bt_node
perception::filter_entity_bt_node
perception::is_entity_moving_bt_node
perception::extract_entity_color_bt_node
perception::extract_collision_scene_bt_node
robocup_hri::generate_text_from_objects_bt_node
robocup_hri::speak_bt_node
robocup_hri::dialogConfirmation_bt_node
robocup_hri::listen_bt_node
robocup_hri::query_bt_node
robocup_hri::command_planning_bt_node
arm::move_to_predefined_bt_node
arm::pick_bt_node
motion::move_to_bt_node
motion::look_at_bt_node
motion::configure_navigate_back_bt_node
motion::pan_bt_node
motion::follow_entity_bt_node
motion::goal_publisher_bt_node
configuration::deferred_bt_node
configuration::setup_gpsr_bt_node
)

list(APPEND GPSR_TESTS
  gpsr_answerquiz_test gpsr_countobject_test gpsr_findperson_test gpsr_findpersonname_test gpsr_followperson_test
  gpsr_guideperson_test gpsr_moveto_test gpsr_offerobject_test gpsr_speak_test gpsr_findobject_test
)

# gpsr tests
foreach(test ${GPSR_TESTS})
  add_executable(${test} src/${test}.cpp)
  ament_target_dependencies(${test} ${dependencies})
  target_link_libraries(${test}
    perception::is_detected_bt_node
    perception::extract_object_from_scene_bt_node
    perception::convert_color_bt_node
    perception::is_gesturing_bt_node
    perception::filter_entity_bt_node
    perception::is_entity_moving_bt_node
    perception::extract_entity_color_bt_node
    perception::extract_collision_scene_bt_node
    robocup_hri::generate_text_from_objects_bt_node
    robocup_hri::speak_bt_node
    robocup_hri::dialogConfirmation_bt_node
    robocup_hri::listen_bt_node
    robocup_hri::query_bt_node
    robocup_hri::command_planning_bt_node
    arm::move_to_predefined_bt_node
    arm::pick_bt_node
    motion::move_to_bt_node
    motion::look_at_bt_node
    motion::configure_navigate_back_bt_node
    motion::pan_bt_node
    motion::follow_entity_bt_node
    motion::goal_publisher_bt_node
    configuration::deferred_bt_node
    configuration::setup_gpsr_bt_node
    configuration::set_perception_model_bt_node
  )
endforeach()

# carry my luggage test
add_executable(carry_my_luggage_test src/carry_my_luggage_test.cpp)
ament_target_dependencies(carry_my_luggage_test ${dependencies})
target_link_libraries(carry_my_luggage_test
  configuration::init_carry_bt_node
  perception::is_detected_bt_node
  perception::is_pointing_bt_node
  perception::is_entity_moving_bt_node
  perception::extract_entity_color_bt_node
  arm::move_to_predefined_bt_node
  motion::move_to_bt_node
  motion::look_at_bt_node
  motion::configure_navigate_back_bt_node
  robocup_hri::speak_bt_node
  robocup_hri::dialogConfirmation_bt_node
)
# Follow entity test
add_executable(follow_entity_test src/follow_entity_test.cpp)
ament_target_dependencies(follow_entity_test ${dependencies})
target_link_libraries(follow_entity_test
  configuration::init_carry_bt_node
  perception::is_detected_bt_node
  # perception::is_entity_moving_bt_node
  # arm::move_to_predefined_bt_node
  # motion::move_to_bt_node
  motion::look_at_bt_node
  motion::follow_entity_bt_node
  # robocup_hri::speak_bt_node
  # robocup_hri::dialogConfirmation_bt_node
)

# SetBlackboardInt test
add_executable(set_blackboard_test src/set_blackboard_test.cpp)
ament_target_dependencies(set_blackboard_test ${dependencies})
target_link_libraries(set_blackboard_test configuration::set_blackboard_int_bt_node)

# FilterEntity test
add_executable(filter_entity_test src/filter_entity_test.cpp)
ament_target_dependencies(filter_entity_test ${dependencies})
target_link_libraries(filter_entity_test perception::filter_entity_bt_node)

# Receptionist test
add_executable(receptionist_test src/receptionist_test.cpp)
ament_target_dependencies(receptionist_test ${dependencies})
target_link_libraries(receptionist_test
  configuration::set_wp_bt_node
  # configuration::init_receptionist_bt_node
  motion::move_to_bt_node
  robocup_hri::speak_bt_node
  perception::is_detected_bt_node
  # perception::store_detection_bt_node
  motion::look_at_bt_node
  robocup_hri::listen_bt_node
  robocup_hri::query_bt_node
  robocup_hri::dialogConfirmation_bt_node
  # motion::look_around_bt_node
  perception::is_sittable_bt_node
  # arm::point_at_bt_node
  arm::move_to_predefined_bt_node
  # configuration::add_guest_to_count_bt_node
)

# Inspection test
add_executable(inspection_test src/inspection_test.cpp)
ament_target_dependencies(inspection_test ${dependencies})
target_link_libraries(inspection_test
  configuration::set_wp_bt_node
  perception::is_door_open_bt_node
  motion::move_to_bt_node
)

# Storing groceries test
add_executable(storing_groceries_test src/storing_test.cpp)
ament_target_dependencies(storing_groceries_test ${dependencies})
target_link_libraries(storing_groceries_test
  configuration::set_wp_bt_node
  # configuration::init_receptionist_bt_node
  motion::move_to_bt_node
  robocup_hri::speak_bt_node
  perception::is_detected_bt_node
  # perception::store_detection_bt_node
  motion::look_at_bt_node
  robocup_hri::listen_bt_node
  robocup_hri::query_bt_node
  robocup_hri::dialogConfirmation_bt_node
  # motion::look_around_bt_node
  perception::is_sittable_bt_node
  # arm::point_at_bt_node
  arm::move_to_predefined_bt_node
  perception::is_door_open_bt_node
  # configuration::add_guest_to_count_bt_node
)

install(TARGETS
  set_wp_test
  look_at_test
  move_to_test
  move_to_test_sim
  setwp_and_moveto_test
  is_detected_test
  is_detected_real_test
  isdetected_moveto_test
  is_detected_moveto_loop_test
  follow_test
  speak_test
  ask_test
  extract_object_from_scene_test
  generate_text_from_objects_test
  pick_demo_test
  follow_person_test
  init_carry
  carry_my_luggage_test
  follow_entity_test
  filter_entity_test
  receptionist_test
  inspection_test
  storing_groceries_test
  find_person_test
  count_people_test
  rotate_test
  get_perception_model_test
  set_perception_model_test
  torsoheight_test
  headposition_test
  filter_object_test
  extract_person_description_test
  music_test
  set_blackboard_test
  
  gpsr_test
  gpsr_answerquiz_test
  gpsr_countobject_test
  gpsr_findperson_test
  gpsr_findpersonname_test
  gpsr_followperson_test
  gpsr_guideperson_test
  gpsr_moveto_test
  gpsr_offerobject_test
  gpsr_speak_test
  gpsr_findobject_test

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY bt_xml config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})

ament_package()
