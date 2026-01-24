#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>

using std::placeholders::_1;

class DynamicMapNode : public rclcpp::Node
{
public:
  DynamicMapNode(): Node("dynamic_map_node")
  {
    pub_static_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map/static", 10);
    pub_dynamic_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map/dynamic", 10);
    pub_inflated_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map/inflated", 10);
    pub_combined_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map/combined", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&DynamicMapNode::on_timer, this));
    size_x_ = 100;
    size_y_ = 100;
    resolution_ = 0.1;
  }

private:
  void on_timer()
  {
    auto now = this->now();
    auto static_grid = make_grid(0);
    auto dynamic_grid = make_grid(0);
    auto inflated_grid = make_grid(0);

    // simulate one dynamic obstacle at center
    int cx = size_x_/2;
    int cy = size_y_/2;
    int idx = cy * size_x_ + cx;
    if (idx >= 0 && idx < (int)dynamic_grid.data.size())
      dynamic_grid.data[idx] = 100;

    // simple inflation (3x3)
    for (int dx=-1; dx<=1; ++dx) {
      for (int dy=-1; dy<=1; ++dy) {
        int x = cx + dx;
        int y = cy + dy;
        if (x>=0 && x<size_x_ && y>=0 && y<size_y_) {
          inflated_grid.data[y*size_x_ + x] = 100;
        }
      }
    }

    // combined
    auto combined = make_grid(0);
    for (size_t i=0;i<combined.data.size();++i) {
      combined.data[i] = std::max({static_grid.data[i], dynamic_grid.data[i], inflated_grid.data[i]});
    }

    static_grid.header.stamp = now;
    dynamic_grid.header.stamp = now;
    inflated_grid.header.stamp = now;
    combined.header.stamp = now;

    pub_static_->publish(static_grid);
    pub_dynamic_->publish(dynamic_grid);
    pub_inflated_->publish(inflated_grid);
    pub_combined_->publish(combined);
  }

  nav_msgs::msg::OccupancyGrid make_grid(int fill)
  {
    nav_msgs::msg::OccupancyGrid g;
    g.header.frame_id = "map";
    g.info.resolution = resolution_;
    g.info.width = size_x_;
    g.info.height = size_y_;
    g.info.origin.position.x = 0.0;
    g.info.origin.position.y = 0.0;
    g.data.assign(size_x_ * size_y_, fill);
    return g;
  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_static_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_dynamic_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_inflated_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_combined_;
  rclcpp::TimerBase::SharedPtr timer_;
  int size_x_;
  int size_y_;
  double resolution_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynamicMapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
