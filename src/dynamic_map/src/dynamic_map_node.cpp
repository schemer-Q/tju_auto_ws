#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <cmath>

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
    this->declare_parameter<std::string>("lidar_topic", "/rslidar_points");
    this->declare_parameter<std::string>("odom_topic", "/sensor/odometry");
    this->declare_parameter<std::string>("map_static_topic", "/map/static");
    this->declare_parameter<std::string>("map_dynamic_topic", "/map/dynamic");
    this->declare_parameter<std::string>("map_inflated_topic", "/map/inflated");
    this->declare_parameter<std::string>("map_combined_topic", "/map/combined");

    size_x_ = this->get_parameter("map_size_x").as_int();
    size_y_ = this->get_parameter("map_size_y").as_int();
    resolution_ = this->get_parameter("map_resolution").as_double();
    inflation_radius_ = this->get_parameter("inflation_radius").as_double();
    
    std::string lidar_topic = this->get_parameter("lidar_topic").as_string();
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    std::string map_static_topic = this->get_parameter("map_static_topic").as_string();
    std::string map_dynamic_topic = this->get_parameter("map_dynamic_topic").as_string();
    std::string map_inflated_topic = this->get_parameter("map_inflated_topic").as_string();
    std::string map_combined_topic = this->get_parameter("map_combined_topic").as_string();

    pub_static_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_static_topic, 10);
    pub_dynamic_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_dynamic_topic, 10);
    pub_inflated_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_inflated_topic, 10);
    pub_combined_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_combined_topic, 10);
    
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10, std::bind(&DynamicMapNode::on_odom, this, _1));
    sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic, 10, std::bind(&DynamicMapNode::on_lidar, this, _1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DynamicMapNode::on_timer, this));
  }

private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = msg;
  }

  void on_lidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    latest_cloud_ = msg;
  }

  double quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q) {
      double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
      double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      return std::atan2(siny_cosp, cosy_cosp);
  }

  void on_timer()
  {
    auto now = this->now();
    auto static_grid = make_grid(0);
    auto dynamic_grid = make_grid(0);
    auto inflated_grid = make_grid(0);

    if (current_odom_ && latest_cloud_) {
        double yaw = quaternion_to_yaw(current_odom_->pose.pose.orientation);
        double rx = current_odom_->pose.pose.position.x;
        double ry = current_odom_->pose.pose.position.y;
        double cos_yaw = std::cos(yaw);
        double sin_yaw = std::sin(yaw);

        // Process PointCloud2
        sensor_msgs::PointCloud2Iterator<float> iter_x(*latest_cloud_, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*latest_cloud_, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*latest_cloud_, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            float px = *iter_x;
            float py = *iter_y;
            float pz = *iter_z;

            if (std::isnan(px) || std::isnan(py) || std::isnan(pz)) continue;
            
            // Simple filter for ground or too high
            if (pz < -0.5 || pz > 3.0) continue; 

            // Transform to map frame (Assuming point cloud is in base_link or similar relative frame)
            // P_map = R * P_sensor + T_robot
            double ox = rx + (px * cos_yaw - py * sin_yaw);
            double oy = ry + (px * sin_yaw + py * cos_yaw);
            
            // Convert to grid coords
            int gx = (int)(ox / resolution_);
            int gy = (int)(oy / resolution_);
            
            if (gx >= 0 && gx < size_x_ && gy >= 0 && gy < size_y_) {
                int idx = gy * size_x_ + gx;
                dynamic_grid.data[idx] = 100;
            }
        }
    }

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

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
  nav_msgs::msg::Odometry::SharedPtr current_odom_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynamicMapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
