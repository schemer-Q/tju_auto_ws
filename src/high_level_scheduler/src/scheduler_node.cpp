#include <rclcpp/rclcpp.hpp>
#include <control_task_msgs/msg/task_goal_data.hpp>

using control_task_msgs::msg::TaskGoalData;

class SchedulerNode : public rclcpp::Node
{
public:
  SchedulerNode(): Node("scheduler_node")
  {
    sub_ = this->create_subscription<TaskGoalData>("/task_goal", 10, std::bind(&SchedulerNode::on_task, this, std::placeholders::_1));
    pub_ = this->create_publisher<TaskGoalData>("/planner/task", 10);
  }

private:
  void on_task(const TaskGoalData::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received task id=%u", msg->order_id);
    // forward to planner for now
    pub_->publish(*msg);
    RCLCPP_INFO(this->get_logger(), "Forwarded task id=%u to /planner/task", msg->order_id);
  }

  rclcpp::Subscription<TaskGoalData>::SharedPtr sub_;
  rclcpp::Publisher<TaskGoalData>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SchedulerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
