cmake_minimum_required(VERSION 3.5)
project(dynamic_map)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(dynamic_map_node src/dynamic_map_node.cpp)
target_link_libraries(dynamic_map_node PUBLIC ${nav_msgs_TARGETS} ${std_msgs_TARGETS} ${sensor_msgs_TARGETS} ${geometry_msgs_TARGETS} rclcpp::rclcpp)

install(TARGETS
  dynamic_map_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/mock_map_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
