#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>

using std::placeholders::_1;

class DynamicMapNode : public rclcpp::Node
{
public:
  DynamicMapNode(): Node("dynamic_map_node")
  {
    // parameters
    this->declare_parameter<int>("map_size_x", 100);
    this->declare_parameter<int>("map_size_y", 100);
    this->declare_parameter<double>("map_resolution", 0.1);
    this->declare_parameter<double>("inflation_radius", 0.3); // meters

    size_x_ = this->get_parameter("map_size_x").as_int();
    size_y_ = this->get_parameter("map_size_y").as_int();
    resolution_ = this->get_parameter("map_resolution").as_double();
    inflation_radius_ = this->get_parameter("inflation_radius").as_double();

    pub_static_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map/static", 10);
    pub_dynamic_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map/dynamic", 10);
    pub_inflated_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map/inflated", 10);
    pub_combined_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map/combined", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&DynamicMapNode::on_timer, this));
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

    // inflation using BFS (radius in cells)
    int radius_cells = std::max(0, (int)std::ceil(inflation_radius_ / resolution_));
    if (radius_cells == 0) {
      // no inflation
      inflated_grid = dynamic_grid;
    } else {
      // visited distances (-1 = unvisited)
      std::vector<int> dist(size_x_ * size_y_, -1);
      std::deque<std::pair<int,int>> qx;
      std::deque<std::pair<int,int>> q;
      // push all occupied cells as sources
      for (int y=0;y<size_y_;++y){
        for (int x=0;x<size_x_;++x){
          int i = y*size_x_ + x;
          if (dynamic_grid.data[i] > 50) {
            dist[i] = 0;
            inflated_grid.data[i] = 100;
            q.emplace_back(x,y);
          }
        }
      }
      // 4-neighborhood BFS
      const int dxs[4] = {1,-1,0,0};
      const int dys[4] = {0,0,1,-1};
      while (!q.empty()) {
        auto p = q.front(); q.pop_front();
        int cx2 = p.first;
        int cy2 = p.second;
        int ci = cy2 * size_x_ + cx2;
        int cd = dist[ci];
        if (cd >= radius_cells) continue;
        for (int k=0;k<4;++k){
          int nx = cx2 + dxs[k];
          int ny = cy2 + dys[k];
          if (nx<0 || nx>=size_x_ || ny<0 || ny>=size_y_) continue;
          int ni = ny*size_x_ + nx;
          if (dist[ni] != -1) continue;
          dist[ni] = cd + 1;
          inflated_grid.data[ni] = 100;
          q.emplace_back(nx, ny);
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
  double inflation_radius_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynamicMapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
