#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <control_task_msgs/msg/task_goal_data.hpp>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>

using control_task_msgs::msg::TaskGoalData;

struct HybridAStarState {
    int x, y;
    double theta;
    double g_cost;
    double h_cost;
    HybridAStarState* parent;
    HybridAStarState(int x_, int y_, double t_, double g_, double h_, HybridAStarState* p_)
        : x(x_), y(y_), theta(t_), g_cost(g_), h_cost(h_), parent(p_) {}
};

struct CompareState {
    bool operator()(const HybridAStarState* a, const HybridAStarState* b) {
        return (a->g_cost + a->h_cost) > (b->g_cost + b->h_cost);
    }
};

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode(): Node("planner_node")
  {
    sub_task_ = this->create_subscription<TaskGoalData>("/planner/task", 10, std::bind(&PlannerNode::on_task, this, std::placeholders::_1));
    sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map/combined", 10, std::bind(&PlannerNode::on_map, this, std::placeholders::_1));
    sub_start_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/start_pose", 10, std::bind(&PlannerNode::on_start, this, std::placeholders::_1));
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/planner/global_path", 10);
    // parameters
    this->declare_parameter<double>("min_turning_radius", 1.0);
    min_turning_radius_ = this->get_parameter("min_turning_radius").as_double();
  }

private:
  static double yaw_from_quat(const geometry_msgs::msg::Quaternion &q) {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    last_map_ = msg;
  }

  void on_start(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    start_pose_ = msg;
    RCLCPP_INFO(this->get_logger(), "Received start pose: x=%.3f, y=%.3f", msg->pose.position.x, msg->pose.position.y);
  }

  void on_task(const TaskGoalData::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Planner received task id=%u", msg->order_id);
    if (!last_map_) {
      RCLCPP_WARN(this->get_logger(), "No map received yet, cannot plan");
      return;
    }
    if (!start_pose_) {
      RCLCPP_WARN(this->get_logger(), "No start pose received yet, cannot plan");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Start: x=%.3f, y=%.3f | Goal: x=%.3f, y=%.3f, yaw=%.3f", 
                start_pose_->pose.position.x, start_pose_->pose.position.y,
                msg->end_point.x, msg->end_point.y, msg->end_point.yaw);
    double origin_yaw = yaw_from_quat(last_map_->info.origin.orientation);
    RCLCPP_INFO(this->get_logger(), "Map info: res=%.4f, width=%u, height=%u, origin=(%.3f, %.3f), yaw=%.3f",
                last_map_->info.resolution,
                last_map_->info.width,
                last_map_->info.height,
                last_map_->info.origin.position.x,
                last_map_->info.origin.position.y,
                origin_yaw);

    nav_msgs::msg::Path raw_path = plan_hybrid_astar(start_pose_->pose.position.x, start_pose_->pose.position.y, 0.0,
                                                       msg->end_point.x, msg->end_point.y, msg->end_point.yaw);
    nav_msgs::msg::Path smoothed_path = smooth_path(raw_path);

    smoothed_path.header.stamp = this->now();
    smoothed_path.header.frame_id = last_map_->header.frame_id;
    pub_path_->publish(smoothed_path);

    // Print path coordinates for comparison
    size_t n = smoothed_path.poses.size();
    RCLCPP_INFO(this->get_logger(), "Smoothed path size: %zu", n);
    if (n > 0) {
      const auto &p0 = smoothed_path.poses.front().pose.position;
      const auto &pN = smoothed_path.poses.back().pose.position;
      RCLCPP_INFO(this->get_logger(), "Path start: x=%.3f, y=%.3f", p0.x, p0.y);
      RCLCPP_INFO(this->get_logger(), "Path end:   x=%.3f, y=%.3f", pN.x, pN.y);
      size_t step = n >= 20 ? n / 20 : 1; // sample about 20 points
      for (size_t i = 0; i < n; i += step) {
        const auto &pi = smoothed_path.poses[i].pose.position;
        RCLCPP_INFO(this->get_logger(), "Path[%zu]: x=%.3f, y=%.3f", i, pi.x, pi.y);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Published smoothed path for task %u", msg->order_id);
  }

  nav_msgs::msg::Path plan_hybrid_astar(double start_x, double start_y, double start_theta,
                                         double goal_x, double goal_y, double goal_theta)
  {
    nav_msgs::msg::Path path;
    // Discretize map
    double res = last_map_->info.resolution;
    int width = last_map_->info.width;
    int height = last_map_->info.height;
    double origin_x = last_map_->info.origin.position.x;
    double origin_y = last_map_->info.origin.position.y;
    double origin_yaw = yaw_from_quat(last_map_->info.origin.orientation);
    double cos_y = std::cos(origin_yaw);
    double sin_y = std::sin(origin_yaw);

    auto world_to_grid = [&](double wx, double wy) {
      double dx = wx - origin_x;
      double dy = wy - origin_y;
      // inverse rotate by origin yaw: R^T * [dx, dy]
      double gx_m =  cos_y * dx + sin_y * dy;
      double gy_m = -sin_y * dx + cos_y * dy;
      int gx = std::round(gx_m / res);
      int gy = std::round(gy_m / res);
      return std::pair<int,int>(gx, gy);
    };

    // Start at world (start_x, start_y) mapped to grid indices
    auto start_idx = world_to_grid(start_x, start_y);
    int start_x_grid = start_idx.first;
    int start_y_grid = start_idx.second;

    auto goal_idx = world_to_grid(goal_x, goal_y);
    int goal_x_grid = goal_idx.first;
    int goal_y_grid = goal_idx.second;

    // Discretize theta (e.g., 8 directions)
    const int theta_bins = 8;
    double theta_res = 2 * M_PI / theta_bins;

    auto discretize_theta = [theta_res](double t) {
      int bin = std::round(t / theta_res);
      return bin * theta_res;
    };

    // Priority queue
    std::priority_queue<HybridAStarState*, std::vector<HybridAStarState*>, CompareState> open_list;
    std::unordered_map<std::string, HybridAStarState*> closed_list;
    std::vector<HybridAStarState*> all_states; // for memory cleanup

    auto key = [theta_res](int x, int y, double t) {
      int t_bin = std::round(t / theta_res);
      return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(t_bin);
    };

    double start_theta_disc = discretize_theta(start_theta);
    HybridAStarState* start = new HybridAStarState(start_x_grid, start_y_grid, start_theta_disc, 0.0, heuristic(start_x_grid, start_y_grid, goal_x_grid, goal_y_grid), nullptr);
    all_states.push_back(start);
    open_list.push(start);

    std::vector<std::pair<double, double>> actions = {
        {1.0, 0.0}, // straight
        {1.0, 1.0 / min_turning_radius_}, // left turn
        {1.0, -1.0 / min_turning_radius_} // right turn
    };

    HybridAStarState* goal_state = nullptr;
    int max_iterations = 10000; // prevent infinite loop
    int iterations = 0;
    while (!open_list.empty() && iterations++ < max_iterations) {
      HybridAStarState* current = open_list.top();
      open_list.pop();

      std::string k = key(current->x, current->y, current->theta);
      if (closed_list.find(k) != closed_list.end()) continue;
      closed_list[k] = current;

      // Check if close to goal
      if (std::abs(current->x - goal_x_grid) < 2 && std::abs(current->y - goal_y_grid) < 2) {
        goal_state = current;
        break;
      }

      for (auto& action : actions) {
        double dist = action.first;
        double steer = action.second;
        double new_theta = discretize_theta(current->theta + steer * dist);
        int new_x = current->x + std::round(dist * std::cos(new_theta) / res);
        int new_y = current->y + std::round(dist * std::sin(new_theta) / res);

        if (new_x < 0 || new_x >= width || new_y < 0 || new_y >= height) continue;
        int idx = new_y * width + new_x;
        if (last_map_->data[idx] > 50) continue; // occupied

        std::string nk = key(new_x, new_y, new_theta);
        if (closed_list.find(nk) != closed_list.end()) continue;

        double new_g = current->g_cost + dist;
        double new_h = heuristic(new_x, new_y, goal_x_grid, goal_y_grid);
        HybridAStarState* neighbor = new HybridAStarState(new_x, new_y, new_theta, new_g, new_h, current);
        all_states.push_back(neighbor);
        open_list.push(neighbor);
      }
    }

    // Reconstruct path
    if (goal_state) {
      std::vector<HybridAStarState*> path_states;
      HybridAStarState* st = goal_state;
      while (st) {
        path_states.push_back(st);
        st = st->parent;
      }
      std::reverse(path_states.begin(), path_states.end());
      auto grid_to_world = [&](double gx, double gy) {
        // center of cell
        double x_m = (gx + 0.5) * res;
        double y_m = (gy + 0.5) * res;
        double wx = cos_y * x_m - sin_y * y_m + origin_x;
        double wy = sin_y * x_m + cos_y * y_m + origin_y;
        return std::pair<double,double>(wx, wy);
      };
      for (auto& state : path_states) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        auto wxy = grid_to_world(state->x, state->y);
        ps.pose.position.x = wxy.first;
        ps.pose.position.y = wxy.second;
        double world_theta = state->theta + origin_yaw;
        ps.pose.orientation.z = std::sin(world_theta / 2.0);
        ps.pose.orientation.w = std::cos(world_theta / 2.0);
        path.poses.push_back(ps);
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Hybrid A* failed to find path");
    }

    // Clean up memory
    for (auto& s : all_states) delete s;

    return path;
  }

  double heuristic(int x, int y, int gx, int gy) {
    return std::sqrt((x - gx)*(x - gx) + (y - gy)*(y - gy));
  }

  rclcpp::Subscription<TaskGoalData>::SharedPtr sub_task_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_start_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  nav_msgs::msg::OccupancyGrid::SharedPtr last_map_;
  geometry_msgs::msg::PoseStamped::SharedPtr start_pose_;
  double min_turning_radius_;

  nav_msgs::msg::Path smooth_path(const nav_msgs::msg::Path& raw_path) {
    nav_msgs::msg::Path smoothed_path = raw_path;

    if (raw_path.poses.size() < 3) {
      RCLCPP_WARN(this->get_logger(), "Path too short to smooth");
      return smoothed_path;
    }

    const double weight_data = 0.5;
    const double weight_smooth = 0.1;
    const double tolerance = 1e-6;

    bool change = true;
    while (change) {
      change = false;
      for (size_t i = 1; i < raw_path.poses.size() - 1; ++i) {
        auto& prev = smoothed_path.poses[i - 1].pose.position;
        auto& curr = smoothed_path.poses[i].pose.position;
        auto& next = smoothed_path.poses[i + 1].pose.position;

        double new_x = curr.x + weight_data * (raw_path.poses[i].pose.position.x - curr.x) +
                       weight_smooth * (prev.x + next.x - 2 * curr.x);
        double new_y = curr.y + weight_data * (raw_path.poses[i].pose.position.y - curr.y) +
                       weight_smooth * (prev.y + next.y - 2 * curr.y);

        if (std::abs(new_x - curr.x) > tolerance || std::abs(new_y - curr.y) > tolerance) {
          change = true;
          curr.x = new_x;
          curr.y = new_y;
        }
      }
    }

    return smoothed_path;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
