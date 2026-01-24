#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <control_task_msgs/msg/task_goal_data.hpp>

using control_task_msgs::msg::TaskGoalData;

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode(): Node("planner_node")
  {
    sub_task_ = this->create_subscription<TaskGoalData>("/planner/task", 10, std::bind(&PlannerNode::on_task, this, std::placeholders::_1));
    sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map/combined", 10, std::bind(&PlannerNode::on_map, this, std::placeholders::_1));
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/planner/global_path", 10);
  }

private:
  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    // store or use map for planning; placeholder stores pointer
    last_map_ = msg;
  }

  void on_task(const TaskGoalData::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Planner received task id=%u", msg->order_id);
    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = "map";
    // simple straight-line path from (0,0) to end_point
    int num = 10;
    for (int i=0;i<=num;++i) {
      double t = double(i)/double(num);
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = (1.0 - t) * 0.0 + t * msg->end_point.x;
      ps.pose.position.y = (1.0 - t) * 0.0 + t * msg->end_point.y;
      ps.pose.position.z = msg->end_point.z;
      path.poses.push_back(ps);
    }
    pub_path_->publish(path);
    RCLCPP_INFO(this->get_logger(), "Published simple path for task %u", msg->order_id);
  }

  rclcpp::Subscription<TaskGoalData>::SharedPtr sub_task_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  nav_msgs::msg::OccupancyGrid::SharedPtr last_map_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
