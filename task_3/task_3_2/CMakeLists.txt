cmake_minimum_required(VERSION 3.5)
project( emergency_services_logistics )


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    plansys2_executor
)

add_executable(fill_box_action_node src/fill_box_action_node.cpp)
ament_target_dependencies(fill_box_action_node ${dependencies})

add_executable(give_content_action_node src/give_content_action_node.cpp)
ament_target_dependencies(give_content_action_node ${dependencies})

add_executable(hold_carrier_action_node src/hold_carrier_action_node.cpp)
ament_target_dependencies(hold_carrier_action_node ${dependencies})

add_executable(load_carrier_action_node src/load_carrier_action_node.cpp)
ament_target_dependencies(load_carrier_action_node ${dependencies})

add_executable(move_action_node src/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})

add_executable(move_carrier_action_node src/move_carrier_action_node.cpp)
ament_target_dependencies(move_carrier_action_node ${dependencies})

add_executable(release_carrier_action_node src/release_carrier_action_node.cpp)
ament_target_dependencies(release_carrier_action_node ${dependencies})

add_executable(satisfied_with_at_least_one_action_node src/satisfied_with_at_least_one_action_node.cpp)
ament_target_dependencies(satisfied_with_at_least_one_action_node ${dependencies})

add_executable(unfill_box_action_node src/unfill_box_action_node.cpp)
ament_target_dependencies(unfill_box_action_node ${dependencies})

add_executable(unload_carrier_action_node src/unload_carrier_action_node.cpp)
ament_target_dependencies(unload_carrier_action_node ${dependencies})


install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
  fill_box_action_node
  give_content_action_node
  hold_carrier_action_node
  load_carrier_action_node
  move_action_node
  move_carrier_action_node
  release_carrier_action_node
  satisfied_with_at_least_one_action_node
  unfill_box_action_node
  unload_carrier_action_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
