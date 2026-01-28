cmake_minimum_required(VERSION 3.5)
project(path_planner)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(control_task_msgs REQUIRED)

add_executable(planner_node src/planner_node.cpp)
target_link_libraries(planner_node PUBLIC ${control_task_msgs_TARGETS} ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${visualization_msgs_TARGETS} rclcpp::rclcpp)

install(TARGETS
  planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
