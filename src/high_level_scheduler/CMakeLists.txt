cmake_minimum_required(VERSION 3.5)
project(high_level_scheduler)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_task_msgs REQUIRED)

add_executable(scheduler_node src/scheduler_node.cpp)
target_link_libraries(scheduler_node PUBLIC ${control_task_msgs_TARGETS} ${std_msgs_TARGETS} rclcpp::rclcpp)

install(TARGETS
  scheduler_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
