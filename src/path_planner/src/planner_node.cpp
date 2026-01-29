#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <control_task_msgs/msg/task_goal_data.hpp>
#include <planning_msgs/msg/local_trajectory_points.hpp>
#include <common_msgs/msg/pose_point.hpp>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>

using control_task_msgs::msg::TaskGoalData;

// --- Dubins Path Helper ---
struct DubinsPath {
    double length;
    std::vector<double> lengths; // 3 segments
    std::vector<int> types;     // L=-1, S=0, R=1
    double x, y, yaw;           // Start pose
    double turning_radius;

    DubinsPath() : length(std::numeric_limits<double>::max()), turning_radius(1.0) {}
};
// --------------------------

struct PlanningResult {
    nav_msgs::msg::Path path;
    std::vector<int8_t> gears; // 1: Reverse, 3: Forward, 2: Neutral
};

struct HybridAStarState {
    int x, y; // Grid indices for lookup
    double cx, cy; // Continuous world coordinates within the grid frame (meters)
    double theta; // Continuous theta
    double steering; // Steering angle used to reach this state
    int direction; // 1: Forward, -1: Reverse
    double g_cost;
    double h_cost;
    HybridAStarState* parent;
    
    HybridAStarState(double cx_, double cy_, double t_, double steer_, int dir_, double g_, double h_, HybridAStarState* p_, double res)
        : cx(cx_), cy(cy_), theta(t_), steering(steer_), direction(dir_), g_cost(g_), h_cost(h_), parent(p_) {
        x = std::round(cx / res);
        y = std::round(cy / res);
    }
    
    // Legacy constructor for compatibility if needed, but better to use continuous
    HybridAStarState(int x_, int y_, double t_, double g_, double h_, HybridAStarState* p_)
        : x(x_), y(y_), cx(x_ * 0.1), cy(y_ * 0.1), theta(t_), steering(0.0), direction(1), g_cost(g_), h_cost(h_), parent(p_) {}
};

struct CompareState {
    bool operator()(const HybridAStarState* a, const HybridAStarState* b) {
        return (a->g_cost + a->h_cost) > (b->g_cost + b->h_cost);
    }
};

class PlannerNode : public rclcpp::Node
{
public:
  // --- Dubins Solver Implementation ---
  // Simple Dubins implementation for LSL, LSR, RSL, RSR cases
  double mod2pi(double theta) {
      return theta - 2.0 * M_PI * std::floor(theta / (2.0 * M_PI));
  }
  
  DubinsPath get_dubins_path(double sx, double sy, double syaw, double ex, double ey, double eyaw, double r) {
      double dx = ex - sx;
      double dy = ey - sy;
      double D = std::sqrt(dx*dx + dy*dy);
      double d = D / r; // normalized distance
      
      // Coordinate transformation to standard frame
      double theta = mod2pi(std::atan2(dy, dx));
      double alpha = mod2pi(syaw - theta);
      double beta = mod2pi(eyaw - theta);
      
      DubinsPath best_path;
      
      auto update_path = [&](double t, double p, double q, int type1, int type2, int type3) {
          double L = std::abs(t) + std::abs(p) + std::abs(q);
          if (L < best_path.length) { // In normalized units? 
             // We need real length
             best_path.length = L * r; // Convert back just for comparison? No, wait. formula gives normalized.
             best_path.lengths = {t, p, q};
             best_path.types = {type1, type2, type3};
             best_path.length = (std::abs(t) + std::abs(p) + std::abs(q)) * r;
          }
      };

      // LSL
      double tmp_lsl = alpha - beta;
      double sa = std::sin(alpha), sb = std::sin(beta);
      double ca = std::cos(alpha), cb = std::cos(beta);
      double c_ab = std::cos(alpha - beta);
      
      double tmp0 = d + sa - sb;
      double p_squared = 2 + (d*d) - (2*c_ab) + (2*d*(sa - sb));
      if(p_squared >= 0) {
          double tmp1 = std::atan2((cb - ca), tmp0);
          double t = mod2pi(-alpha + tmp1);
          double p = std::sqrt(p_squared);
          double q = mod2pi(beta - tmp1);
          update_path(t, p, q, -1, 0, -1); // LSL
      }
      
      // RSR
      double p_squared_rsr = 2 + (d*d) - (2*c_ab) + (2*d*(sb - sa));
      if(p_squared_rsr >= 0) {
          double tmp1 = std::atan2((ca - cb), (d - sa + sb));
          double t = mod2pi(alpha - tmp1);
          double p = std::sqrt(p_squared_rsr);
          double q = mod2pi(-beta + tmp1);
          update_path(t, p, q, 1, 0, 1); // RSR
      }
      
      // LSR
      double p_squared_lsr = -2 + (d*d) + (2*c_ab) + (2*d*(sa + sb));
      if(p_squared_lsr >= 0) {
          double p = std::sqrt(p_squared_lsr);
          double tmp1 = std::atan2((-ca - cb), (d + sa + sb));
          double t = mod2pi(-alpha + tmp1 - std::atan2(-2.0, p));
          double q = mod2pi(-mod2pi(beta) + tmp1 - std::atan2(-2.0, p));
          update_path(t, p, q, -1, 0, 1); // LSR
      }

      // RSL
      double p_squared_rsl = -2 + (d*d) + (2*c_ab) - (2*d*(sa + sb));
      if(p_squared_rsl >= 0) {
          double p = std::sqrt(p_squared_rsl);
          double tmp1 = std::atan2((ca + cb), (d - sa - sb));
          double t = mod2pi(alpha - tmp1 + std::atan2(2.0, p));
          double q = mod2pi(beta - tmp1 + std::atan2(2.0, p));
          update_path(t, p, q, 1, 0, -1); // RSL
      }
      
      best_path.x = sx; best_path.y = sy; best_path.yaw = syaw; best_path.turning_radius = r;
      return best_path;
  }
  
  std::vector<HybridAStarState*> sample_dubins(DubinsPath dpath, HybridAStarState* parent, double res, int target_dir, double cost_scale = 1.0) {
      std::vector<HybridAStarState*> states;
      double step = res * 0.5; // Sampling step - finer than grid
      
      double c_x = dpath.x, c_y = dpath.y, c_yaw = dpath.yaw;
      double r = dpath.turning_radius;
      
      HybridAStarState* current_parent = parent;

      for (size_t i = 0; i < 3; ++i) {
          double mode_len = dpath.lengths[i] * r;
          int mode = dpath.types[i]; // -1, 0, 1
          
          if (mode_len < 1e-4) continue;
          
          // Use while loop to cover exact distance
          double dist_covered = 0.0;
          
          while (dist_covered < mode_len) {
              double remain = mode_len - dist_covered;
              double dist = (remain < step) ? remain : step;
              if (dist < 1e-6) break; // Avoid floating point infinite loop
              
              dist_covered += dist;

              // Update state (Geometry Simulation)
              if (mode == 0) { // Straight
                  c_x += dist * std::cos(c_yaw);
                  c_y += dist * std::sin(c_yaw);
              } else { // Turn
                  // Left (-1) -> +dtheta, Right (1) -> -dtheta
                  double dir = (mode == -1) ? 1.0 : -1.0;
                  double dtheta = dir * (dist / r);
                  double chord = 2.0 * r * std::sin(std::abs(dtheta) / 2.0);
                  
                  c_x += chord * std::cos(c_yaw + dtheta/2.0);
                  c_y += chord * std::sin(c_yaw + dtheta/2.0);
                  c_yaw += dtheta;
              }
              
              // Normalize Yaw for simulation state
              while(c_yaw > M_PI) c_yaw -= 2.0*M_PI;
              while(c_yaw <= -M_PI) c_yaw += 2.0*M_PI;
              
              // Create Node logic
              double g = current_parent->g_cost + (dist * cost_scale); 
              if (mode != 0) g += dist * 0.05 * cost_scale; // Turn penalty

              double steer = (mode == 0) ? 0.0 : ((mode == -1) ? 1.0/r : -1.0/r);
              
              // Interpret Pose for Target Direction
              double actual_theta = c_yaw;
              if (target_dir == -1) {
                  actual_theta += M_PI; // Flip back
              }
              // Norm
              while(actual_theta > M_PI) actual_theta -= 2.0*M_PI;
              while(actual_theta <= -M_PI) actual_theta += 2.0*M_PI;

              HybridAStarState* node = new HybridAStarState(c_x, c_y, actual_theta, steer, target_dir, g, 0.0, current_parent, res);
              states.push_back(node);
              current_parent = node;
          }
      }
      return states;
  }
  
  bool check_collision(double cx, double cy, double theta) {
      if (!last_map_) return false;
      int width = last_map_->info.width;
      int height = last_map_->info.height;
      double res = last_map_->info.resolution;
      
      double cos_t = std::cos(theta);
      double sin_t = std::sin(theta);
      double hw = vehicle_width_ / 2.0;
      
      // Calculate AABB of rotated vehicle footprint
      // Corners relative to center
      double dx[4] = {front_to_axle_, front_to_axle_, -back_to_axle_, -back_to_axle_};
      double dy[4] = {hw, -hw, hw, -hw};
      
      double min_x = 1e10, max_x = -1e10, min_y = 1e10, max_y = -1e10;
      
      for(int i=0; i<4; ++i) {
          double wx = cx + dx[i]*cos_t - dy[i]*sin_t;
          double wy = cy + dx[i]*sin_t + dy[i]*cos_t;
          if(wx < min_x) min_x = wx;
          if(wx > max_x) max_x = wx;
          if(wy < min_y) min_y = wy;
          if(wy > max_y) max_y = wy;
      }
      
      int min_gx = std::max(0, (int)std::floor(min_x / res));
      int max_gx = std::min(width - 1, (int)std::ceil(max_x / res));
      int min_gy = std::max(0, (int)std::floor(min_y / res));
      int max_gy = std::min(height - 1, (int)std::ceil(max_y / res));
      
      for (int y = min_gy; y <= max_gy; ++y) {
          for (int x = min_gx; x <= max_gx; ++x) {
              int idx = y * width + x;
              if (last_map_->data[idx] > 50) { // If obstacle
                   // Check if inside oriented rectangle
                   double cell_wx = (x + 0.5) * res;
                   double cell_wy = (y + 0.5) * res;
                   
                   // Transform to vehicle frame
                   double tx = cell_wx - cx;
                   double ty = cell_wy - cy;
                   
                   // Rotate back by -theta
                   double local_x = tx * cos_t + ty * sin_t;
                   double local_y = -tx * sin_t + ty * cos_t;
                   
                   if (local_x >= -back_to_axle_ && local_x <= front_to_axle_ &&
                       local_y >= -hw && local_y <= hw) {
                       return true; // Collision
                   }
              }
          }
      }
      return false; // Safe
  }

  bool is_collision_free(const std::vector<HybridAStarState*>& states, int width, int height) {
      (void)width; (void)height;
      
      // Optimization: Skip checking every single interpolated point.
      // We check points every ~0.3m to 0.5m interval.
      // Assuming 'states' typically comes from Dubins sampling with ~0.1m step.
      
      if (states.empty()) return true;

      // Always check the first and last point
      if (check_collision(states.front()->cx, states.front()->cy, states.front()->theta)) return false;
      if (states.size() > 1) {
          if (check_collision(states.back()->cx, states.back()->cy, states.back()->theta)) return false;
      }
      
      // Check intermediate points with stride
      // Stride of 4 implies checking every ~0.4m if res=0.1
      size_t stride = 4; 
      
      for (size_t i = 1; i < states.size() - 1; i += stride) {
         if (check_collision(states[i]->cx, states[i]->cy, states[i]->theta)) return false;
      }
      return true;
  }
  
  PlannerNode(): Node("planner_node")
  {
    sub_task_ = this->create_subscription<TaskGoalData>("/planner/task", 10, std::bind(&PlannerNode::on_task, this, std::placeholders::_1));
    sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map/combined", 10, std::bind(&PlannerNode::on_map, this, std::placeholders::_1));
    sub_start_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/start_pose", 10, std::bind(&PlannerNode::on_start, this, std::placeholders::_1));
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/planner/global_path", 10);
    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/planner/footprints", 10);
    pub_traj_ = this->create_publisher<planning_msgs::msg::LocalTrajectoryPoints>("/planner/local_trajectory", 10);
    // parameters
    this->declare_parameter<double>("min_turning_radius", 1.0);
    min_turning_radius_ = this->get_parameter("min_turning_radius").as_double();
    
    // Default vehicle dimensions (Standard Sedan)
    vehicle_length_ = 4.7; 
    vehicle_width_ = 2.0; 
    back_to_axle_ = 1.0; 
    front_to_axle_ = vehicle_length_ - back_to_axle_;
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

    double start_yaw = yaw_from_quat(start_pose_->pose.orientation);
    PlanningResult result = plan_hybrid_astar(start_pose_->pose.position.x, start_pose_->pose.position.y, start_yaw,
                                                       msg->end_point.x, msg->end_point.y, msg->end_point.yaw);
    // nav_msgs::msg::Path smoothed_path = smooth_path(result);
    // Skip smoothing to preserve kinematic constraints (curvature limits) enforced by Hybrid A*
    nav_msgs::msg::Path smoothed_path = result.path; 

    // Post-process: Recalculate Orientations and enforce exact start/goal
    if (!smoothed_path.poses.empty()) {
        // Enforce exact Start Pose
        smoothed_path.poses.front().pose.position.x = start_pose_->pose.position.x;
        smoothed_path.poses.front().pose.position.y = start_pose_->pose.position.y;
        smoothed_path.poses.front().pose.orientation = start_pose_->pose.orientation;
        
      // Enforce exact Goal Pose
        smoothed_path.poses.back().pose.position.x = msg->end_point.x;
        smoothed_path.poses.back().pose.position.y = msg->end_point.y;
        
        // Since we guided A* to align with goal yaw, we can safely snap the final orientation
        // without causing a huge jump.
        double half_yaw = msg->end_point.yaw / 2.0;
        smoothed_path.poses.back().pose.orientation.z = std::sin(half_yaw);
        smoothed_path.poses.back().pose.orientation.w = std::cos(half_yaw);
        
        // Ensure path continuity at connection point (if two-stage)
        // If we have a sharp jump in index 1..N due to append, we trust append's linear interpolation.
        
        // Hybrid A* provides valid orientations (kinematic). 
        // Do NOT overwrite them with geometric tangents, as that destroys the smoothness property.
        
        /* 
        // Recalculate yaw for all points based on tangent
        for (size_t i = 0; i < smoothed_path.poses.size(); ++i) {
            double yaw = 0.0;
            if (i == 0) {
                 // Use start orientation
                 yaw = yaw_from_quat(start_pose_->pose.orientation);
            } else if (i == smoothed_path.poses.size() - 1) {
                 // Use goal orientation
                 yaw = msg->end_point.yaw;
            } else {
                 // Central difference for internal points
                 double dx = smoothed_path.poses[i+1].pose.position.x - smoothed_path.poses[i-1].pose.position.x;
                 double dy = smoothed_path.poses[i+1].pose.position.y - smoothed_path.poses[i-1].pose.position.y;
                 yaw = std::atan2(dy, dx);
            }
            smoothed_path.poses[i].pose.orientation.z = std::sin(yaw / 2.0);
            smoothed_path.poses[i].pose.orientation.w = std::cos(yaw / 2.0);
        }
        
        // Second pass: interpolate positions near start to respect start heading
        // If the path tries to move effectively perpendicular to start heading immediately,
        // we push the second point out along the start heading vector.
        if (smoothed_path.poses.size() > 2) {
             double start_yaw = yaw_from_quat(smoothed_path.poses[0].pose.orientation);
             double dx = smoothed_path.poses[1].pose.position.x - smoothed_path.poses[0].pose.position.x;
             double dy = smoothed_path.poses[1].pose.position.y - smoothed_path.poses[0].pose.position.y;
             double dist = std::sqrt(dx*dx + dy*dy);
             
             // Project onto start vector? Or just force second point to be [dist] away along start_yaw
             // Let's blend: 70% along start_yaw, 30% original position
             double target_x = smoothed_path.poses[0].pose.position.x + dist * std::cos(start_yaw);
             double target_y = smoothed_path.poses[0].pose.position.y + dist * std::sin(start_yaw);
             
             smoothed_path.poses[1].pose.position.x = 0.7 * target_x + 0.3 * smoothed_path.poses[1].pose.position.x;
             smoothed_path.poses[1].pose.position.y = 0.7 * target_y + 0.3 * smoothed_path.poses[1].pose.position.y;
             
             // Re-update yaw for index 1 after moving potential position
             double dx_new = smoothed_path.poses[2].pose.position.x - smoothed_path.poses[0].pose.position.x;
             double dy_new = smoothed_path.poses[2].pose.position.y - smoothed_path.poses[0].pose.position.y;
             double new_yaw = std::atan2(dy_new, dx_new);
             smoothed_path.poses[1].pose.orientation.z = std::sin(new_yaw / 2.0);
             smoothed_path.poses[1].pose.orientation.w = std::cos(new_yaw / 2.0);
        }
        */
    }

    smoothed_path.header.stamp = this->now();
    smoothed_path.header.frame_id = last_map_->header.frame_id;
    pub_path_->publish(smoothed_path);
    
    // Visualize Vehicle Footprints periodically
    publish_vehicle_footprints(smoothed_path);

    // Publish Custom Trajectory Message
    planning_msgs::msg::LocalTrajectoryPoints traj_msg;
    traj_msg.header = smoothed_path.header;
    traj_msg.task_type = 0; // Normal tracking
    traj_msg.points_cnt = smoothed_path.poses.size();
    traj_msg.replan_counter = msg->order_id; 
    
    double total_dist = 0.0;
    
    for (size_t i = 0; i < smoothed_path.poses.size(); ++i) {
        common_msgs::msg::PosePoint pp;
        pp.x = smoothed_path.poses[i].pose.position.x;
        pp.y = smoothed_path.poses[i].pose.position.y;
        pp.z = 0.0;
        pp.yaw = yaw_from_quat(smoothed_path.poses[i].pose.orientation);
        pp.pitch = 0.0;
        pp.roll = 0.0;
        pp.speed = 0.0; 
        pp.acc = 0.0;
        pp.curve = 0.0; 
        
        // Use internal gear state from A* (Ensures consistency with planner direction)
        if (i < result.gears.size()) {
            pp.gear = result.gears[i];
            // Fix: Start point usually has direction=0 (Neutral), overwrite with 2nd point's gear 
            // so controller knows initial direction immediately.
            if (i == 0 && result.gears.size() > 1) {
                pp.gear = result.gears[1];
            }
        } else {
            pp.gear = 2; // Neutral fallback
        }
        
        traj_msg.trajectory.push_back(pp);
        
        if (i > 0) {
            double dx = pp.x - traj_msg.trajectory[i-1].x;
            double dy = pp.y - traj_msg.trajectory[i-1].y;
            total_dist += std::hypot(dx, dy);
        }
    }
    
    if (traj_msg.points_cnt > 1) {
        traj_msg.step_length = total_dist / (traj_msg.points_cnt - 1);
    } else {
        traj_msg.step_length = 0.0;
    }
    
    pub_traj_->publish(traj_msg);

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

  bool check_segment_collision(double x1, double y1, double x2, double y2, double theta) {
       double dist = std::hypot(x2-x1, y2-y1);
       if (dist < 1e-3) return check_collision(x1, y1, theta);
       
       int steps = std::ceil(dist / 0.1); 
       for(int i=0; i<=steps; ++i) {
           double t = (double)i / steps;
           double x = x1 + t*(x2-x1);
           double y = y1 + t*(y2-y1);
           if (check_collision(x, y, theta)) return true;
       }
       return false;
  }

  PlanningResult plan_hybrid_astar(double start_x, double start_y, double start_theta,
                                         double goal_x, double goal_y, double goal_theta)
  {
      double dist_sg = std::hypot(goal_x - start_x, goal_y - start_y);
      double pre_dist = 1.0;
      
      // Calculate Candidates
      struct Candidate {
          double x, y;
          bool is_reverse;
      };
      std::vector<Candidate> candidates;
      
      // 1. Forward Pre-Goal (Behind the goal)
      {
          double px = goal_x - pre_dist * std::cos(goal_theta);
          double py = goal_y - pre_dist * std::sin(goal_theta);
          // Only if far enough and segment safe
          if (dist_sg > pre_dist + 0.5 && !check_segment_collision(px, py, goal_x, goal_y, goal_theta)) {
              candidates.push_back({px, py, false});
          }
      }
      
      // 2. Reverse Pre-Goal (In front of the goal)
      {
          double px = goal_x + pre_dist * std::cos(goal_theta);
          double py = goal_y + pre_dist * std::sin(goal_theta);
          // Only if segment safe. Distance check is less critical for reverse approach, but let's keep it sane.
          double dist_rev = std::hypot(px - start_x, py - start_y);
           if (dist_rev > 0.5 && !check_segment_collision(px, py, goal_x, goal_y, goal_theta)) {
              candidates.push_back({px, py, true});
          }
      }
      
      // If we have candidates, try them
      PlanningResult best_result;
      double min_len = 1e9;
      bool found_pre = false;
      
      for (const auto& cand : candidates) {
          std::string type = cand.is_reverse ? "Reverse" : "Forward";
          RCLCPP_INFO(this->get_logger(), "Attempting Pre-Goal (%s) Approach...", type.c_str());
          
          PlanningResult p = compute_hybrid_path(start_x, start_y, start_theta, cand.x, cand.y, goal_theta);
          
          if (!p.path.poses.empty()) {
              // Calculate length
              double len = 0.0;
              for(size_t i=1; i<p.path.poses.size(); ++i) {
                  double dx = p.path.poses[i].pose.position.x - p.path.poses[i-1].pose.position.x;
                  double dy = p.path.poses[i].pose.position.y - p.path.poses[i-1].pose.position.y;
                  len += std::hypot(dx, dy);
              }
              
              if (len < min_len) {
                  min_len = len;
                  append_straight_line(p, cand.x, cand.y, goal_x, goal_y, goal_theta);
                  best_result = p;
                  found_pre = true;
              }
          }
      }
      
      if (found_pre) {
          RCLCPP_INFO(this->get_logger(), "Selected optimal Pre-Goal approach.");
          return best_result;
      }
      
      RCLCPP_WARN(this->get_logger(), "All Pre-Goal approaches failed or invalid. Falling back to Direct Goal.");
      
      // Fallback or Direct
      return compute_hybrid_path(start_x, start_y, start_theta, goal_x, goal_y, goal_theta);
  }
  
  void append_straight_line(PlanningResult& res, double px, double py, double gx, double gy, double theta) {
      nav_msgs::msg::Path& path = res.path;
      double dist = std::hypot(gx - px, gy - py);
      int steps = std::ceil(dist / 0.1); // 1m -> 10 steps
      
      geometry_msgs::msg::Quaternion q;
      q.z = std::sin(theta/2.0);
      q.w = std::cos(theta/2.0);
      
      // Determine gear for straight segment
      double dx = gx - px;
      double dy = gy - py;
      int8_t seg_gear = 3; // Forward
      if ((dx * std::cos(theta) + dy * std::sin(theta)) < 0) seg_gear = 1; // Reverse
      
      // Start from 1 because 0 is pre_goal which is already in path
      for(int i=1; i<=steps; ++i) {
          double t = (double)i / steps;
          geometry_msgs::msg::PoseStamped ps;
          ps.header = path.header;
          ps.pose.position.x = px + t*(gx-px);
          ps.pose.position.y = py + t*(gy-py);
          ps.pose.position.z = 0.0;
          ps.pose.orientation = q;
          path.poses.push_back(ps);
          res.gears.push_back(seg_gear);
      }
  }

  PlanningResult compute_hybrid_path(double start_x, double start_y, double start_theta,
                                         double goal_x, double goal_y, double goal_theta)
  {
    PlanningResult result;
    nav_msgs::msg::Path& path = result.path;
    // Discretize map
    double res = last_map_->info.resolution;
    int width = last_map_->info.width;
    int height = last_map_->info.height;
    double origin_x = last_map_->info.origin.position.x;
    double origin_y = last_map_->info.origin.position.y;
    double origin_yaw = yaw_from_quat(last_map_->info.origin.orientation);
    double cos_y = std::cos(origin_yaw);
    double sin_y = std::sin(origin_yaw);

    auto world_to_grid_continuous = [&](double wx, double wy) {
      double dx = wx - origin_x;
      double dy = wy - origin_y;
      // inverse rotate by origin yaw: R^T * [dx, dy]
      double gx_m =  cos_y * dx + sin_y * dy;
      double gy_m = -sin_y * dx + cos_y * dy;
      return std::pair<double,double>(gx_m, gy_m);
    };

    // Start at world (start_x, start_y)
    auto start_grid_xy = world_to_grid_continuous(start_x, start_y);
    int start_x_grid = std::round(start_grid_xy.first / res);
    int start_y_grid = std::round(start_grid_xy.second / res);

    auto goal_grid_xy = world_to_grid_continuous(goal_x, goal_y);
    int goal_x_grid = std::round(goal_grid_xy.first / res);
    int goal_y_grid = std::round(goal_grid_xy.second / res);

    // Discretize theta (Increased for smoother straight lines)
    const int theta_bins = 24;
    double theta_res = 2 * M_PI / theta_bins;

    auto discretize_theta = [theta_res](double t) {
      int bin = std::round(t / theta_res);
      return bin * theta_res;
    };

    // Priority queue
    std::priority_queue<HybridAStarState*, std::vector<HybridAStarState*>, CompareState> open_list;
    std::unordered_map<std::string, HybridAStarState*> closed_list;
    std::vector<HybridAStarState*> all_states; // for memory cleanup

    // Fix: Canonicalize theta bin to handle PI/-PI wrapping
    // In C++, -12 % 24 = -12. We want it to be 12.
    auto key = [theta_res, theta_bins](int x, int y, double t) {
      int t_bin = std::round(t / theta_res);
      t_bin = (t_bin % theta_bins + theta_bins) % theta_bins;
      return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(t_bin);
    };

    // Fix heuristic scaling: ensure h_cost matches g_cost physics units (meters)
    
    double start_theta_norm = start_theta;
    while (start_theta_norm > M_PI) start_theta_norm -= 2.0 * M_PI;
    while (start_theta_norm <= -M_PI) start_theta_norm += 2.0 * M_PI;
    
    double h_start = heuristic(start_grid_xy.first, start_grid_xy.second, start_theta_norm, 
                               goal_grid_xy.first, goal_grid_xy.second, goal_theta);
    // double h_start = heuristic(start_x_grid, start_y_grid, goal_x_grid, goal_y_grid) * res;
    
    // Initial steering is 0.0, g_cost=0.0
    // Constructor args: cx, cy, theta, steer, dir, g, h, parent, res
    // Change start direction to 0 (Stopped/Neutral) so first move (Forward/Reverse) has NO penalty
    HybridAStarState* start = new HybridAStarState(start_grid_xy.first, start_grid_xy.second, start_theta_norm, 0.0, 0, 0.0, h_start, nullptr, res);
    
    all_states.push_back(start);
    open_list.push(start);

    // Reduced step size for finer control
    // If the step is too small compared to resolution (0.1), discretization errors pile up.
    // Resolution = 0.1. Step = 0.5 is 5 cells. Should be ok.
    double step_size = 0.5;
    
    // Generate steering primitives (curvature k)
    // We use a set of curvatures from -max to +max to allow smooth transitions
    double max_curvature = 1.0 / min_turning_radius_;
    std::vector<double> steer_actions;
    int num_steer = 7; // -max, -2/3, -1/3, 0, 1/3, 2/3, max
    for (int i = 0; i < num_steer; ++i) {
        double alpha = (double)i / (num_steer - 1); // 0 to 1
        double k = max_curvature * (2.0 * alpha - 1.0);
        steer_actions.push_back(k);
    }
    
    // Reverse Penalty parameters
    double reverse_penalty_factor = 1.05; // Make reverse slightly more expensive
    double gear_switch_penalty = 2.0;    // Cost to change between forward/reverse

    // Penalize turning slightly more to encourage straight paths if costs are similar
    double turn_penalty_factor = 1.05; 
    // Penalize changing steering angle (limit angular jerk)
    double steer_change_penalty = 2.0; 

    HybridAStarState* goal_state = nullptr;
    int max_iterations = 50000; 
    int iterations = 0;
    while (!open_list.empty() && iterations++ < max_iterations) {
      HybridAStarState* current = open_list.top();
      open_list.pop();

      std::string k = key(current->x, current->y, current->theta);
      if (closed_list.find(k) != closed_list.end()) continue;
      closed_list[k] = current;

      // Check if close to goal (Stricter check including orientation)
      double dist_to_goal = std::sqrt(std::pow(current->cx - goal_grid_xy.first, 2) + std::pow(current->cy - goal_grid_xy.second, 2));
      double angle_to_goal = get_angle_diff(current->theta, goal_theta);
      
      // Termination thresholds: 0.5m distance, ~15 degrees orientation
      if (dist_to_goal < 0.5 && angle_to_goal < 0.26) {
        goal_state = current;
        break;
      }
      
      // analytic_expansion (Dubins Shot) with probability or closeness
      // If we are somewhat close (within 10m?) and heading is aligned, try to shoot
      if (dist_to_goal < 10.0 && (iterations % 5 == 0)) {
         // --- Forward Shot ---
         if (current->direction == 1 || current->direction == 0) {
             DubinsPath dpath = get_dubins_path(current->cx, current->cy, current->theta, 
                                                goal_grid_xy.first, goal_grid_xy.second, goal_theta, 
                                                min_turning_radius_);
             if (dpath.length < 20.0) { // Feasible length
                 std::vector<HybridAStarState*> shot_states = sample_dubins(dpath, current, res, 1, 1.0);
                 if (!shot_states.empty()) {
                    if (is_collision_free(shot_states, width, height)) {
                       goal_state = shot_states.back(); 
                       for(auto s : shot_states) all_states.push_back(s);
                       break;
                    } else {
                       for(auto s : shot_states) delete s;
                    }
                 }
             }
         }
         
         // --- Reverse Shot ---
         // Try reverse analytic expansion if direction is reverse or neutral
         if (current->direction == -1 || current->direction == 0) {
             // Flip start and goal by PI to solve as forward Dubins
             double start_theta_flip = current->theta + M_PI;
             double goal_theta_flip = goal_theta + M_PI;
             
             DubinsPath dpath = get_dubins_path(current->cx, current->cy, start_theta_flip, 
                                                goal_grid_xy.first, goal_grid_xy.second, goal_theta_flip, 
                                                min_turning_radius_);
                                                
             if (dpath.length < 20.0) {
                 // Pass -1 as target_dir, and reverse_penalty_factor (need access to it here, or hardcode 1.05)
                 std::vector<HybridAStarState*> shot_states = sample_dubins(dpath, current, res, -1, 1.05);
                  if (!shot_states.empty()) {
                    if (is_collision_free(shot_states, width, height)) {
                       goal_state = shot_states.back(); 
                       for(auto s : shot_states) all_states.push_back(s);
                       break; // Found Reverse path
                    } else {
                       for(auto s : shot_states) delete s;
                    }
                 }
             }
         }
      }

      std::vector<int> directions = {1, -1};

      for (int dir : directions) { // Iterate Forward (1) and Reverse (-1)
          for (double steer : steer_actions) {
            double dist = step_size * dir;
            
            // Calculate costs
            double base_cost = std::abs(dist); // step_size
            if (dir == -1) base_cost *= reverse_penalty_factor;
            
            // Turn penalty applies to both directions
            if (std::abs(steer) > 0.01) base_cost *= turn_penalty_factor;
            
            // Gear switch penalty (if direction changes)
            double switch_cost = 0.0;
            if (current->direction != 0 && dir != current->direction) {
                switch_cost = gear_switch_penalty;
            }
            
            double steer_diff = std::abs(steer - current->steering);
            double change_cost = (switch_cost > 0) ? 0.0 : (steer_diff * steer_change_penalty);
            
            double new_g = current->g_cost + base_cost + switch_cost + change_cost;

            double dtheta = steer * dist;
            double new_theta = current->theta + dtheta;
            // Normalize theta to [-PI, PI]
            while (new_theta > M_PI) new_theta -= 2.0 * M_PI;
            while (new_theta <= -M_PI) new_theta += 2.0 * M_PI;
            
            // Position update - Simple integration
            // c_x += dist * cos
            double new_cx = current->cx + dist * std::cos(current->theta + dtheta/2.0);
            double new_cy = current->cy + dist * std::sin(current->theta + dtheta/2.0);
            
            int grid_new_x = std::round(new_cx / res);
            int grid_new_y = std::round(new_cy / res);

            if (grid_new_x < 0 || grid_new_x >= width || grid_new_y < 0 || grid_new_y >= height) continue;
            
            // Replaced point check with Full Body Collision Check
            if (check_collision(new_cx, new_cy, new_theta)) continue; 
            
            // int idx = grid_new_y * width + grid_new_x;
            // if (last_map_->data[idx] > 50) continue; // occupied

            std::string nk = key(grid_new_x, grid_new_y, new_theta);
            if (closed_list.find(nk) != closed_list.end()) continue;

            // Apply scaling to H cost to match G cost units
            double new_h = heuristic(new_cx, new_cy, new_theta, goal_grid_xy.first, goal_grid_xy.second, goal_theta);
            
            HybridAStarState* neighbor = new HybridAStarState(new_cx, new_cy, new_theta, steer, dir, new_g, new_h, current, res);
            all_states.push_back(neighbor);
            open_list.push(neighbor);
          }
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
        // Continuous coordinates in meters from map origin
        double x_m = gx;
        double y_m = gy;
        double wx = cos_y * x_m - sin_y * y_m + origin_x;
        double wy = sin_y * x_m + cos_y * y_m + origin_y;
        return std::pair<double,double>(wx, wy);
      };
      for (auto& state : path_states) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        auto wxy = grid_to_world(state->cx, state->cy);
        ps.pose.position.x = wxy.first;
        ps.pose.position.y = wxy.second;
        double world_theta = state->theta + origin_yaw;
        ps.pose.orientation.z = std::sin(world_theta / 2.0);
        ps.pose.orientation.w = std::cos(world_theta / 2.0);
        path.poses.push_back(ps);
        
        int8_t g = 2;
        if (state->direction == 1) g = 3;
        else if (state->direction == -1) g = 1;
        result.gears.push_back(g);
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Hybrid A* failed to find path");
    }

    // Clean up memory
    for (auto& s : all_states) delete s;

    return result;
  }

  double get_angle_diff(double a, double b) {
    double d = a - b;
    while (d > M_PI) d -= 2.0 * M_PI;
    while (d <= -M_PI) d += 2.0 * M_PI;
    return std::abs(d);
  }

  double heuristic(double cx, double cy, double ct, double gx, double gy, double gt) {
    double dist = std::sqrt((cx - gx)*(cx - gx) + (cy - gy)*(cy - gy));
    double ang_diff = get_angle_diff(ct, gt);
    // Combined heuristic: Distance + Orientation penalty
    // Weighting: 1 radian error is roughly equivalent to 2.0 meters of travel cost
    return dist + 2.0 * ang_diff;
  }

  rclcpp::Subscription<TaskGoalData>::SharedPtr sub_task_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_start_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<planning_msgs::msg::LocalTrajectoryPoints>::SharedPtr pub_traj_;
  nav_msgs::msg::OccupancyGrid::SharedPtr last_map_;
  geometry_msgs::msg::PoseStamped::SharedPtr start_pose_;
  double min_turning_radius_;

  // Vehicle Dimensions for Collision Checking
  double vehicle_length_;
  double vehicle_width_;
  double back_to_axle_;
  double front_to_axle_;

  nav_msgs::msg::Path smooth_path(const PlanningResult& result) {
    const nav_msgs::msg::Path& raw_path = result.path;
    const std::vector<int8_t>& gears = result.gears;
    
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
        
        // Cusp protection: If direction changes at this point, do not smooth it.
        // gears[i] is direction arriving at i. gears[i+1] is direction leaving i.
        if (i + 1 < gears.size()) {
            if (gears[i] != gears[i+1]) continue; 
        }

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

  void publish_vehicle_footprints(const nav_msgs::msg::Path& path) {
      if (path.poses.empty()) return;
      
      visualization_msgs::msg::MarkerArray mk_array;
      // Delete old markers
      visualization_msgs::msg::Marker delete_mk;
      delete_mk.action = visualization_msgs::msg::Marker::DELETEALL;
      mk_array.markers.push_back(delete_mk);
      
      // Calculate geometric center offset from rear axle
      // Rear axle is at 0 in vehicle frame.
      // Box range: [-back_to_axle_, front_to_axle_]
      // Center = (-back + front) / 2
      double center_offset = (front_to_axle_ - back_to_axle_) / 2.0;
      
      int id = 0;
      
      // Select points based on distance
      std::vector<size_t> indices;
      if (!path.poses.empty()) {
          indices.push_back(0); // Start
          
          double accumulated_dist = 0.0;
          double sample_interval = 2.0; // Draw every 2 meters
          
          for (size_t i = 1; i < path.poses.size(); ++i) {
              const auto& p_prev = path.poses[i-1].pose.position;
              const auto& p_curr = path.poses[i].pose.position;
              double d = std::hypot(p_curr.x - p_prev.x, p_curr.y - p_prev.y);
              accumulated_dist += d;
              
              if (accumulated_dist >= sample_interval) {
                  indices.push_back(i);
                  accumulated_dist = 0.0;
              }
          }
          
          // Ensure goal is included if not too close to last sample
          if (indices.back() != path.poses.size() - 1) {
              indices.push_back(path.poses.size() - 1);
          }
      }
      
      for(auto idx : indices) {
          const auto& pose = path.poses[idx];
          
          visualization_msgs::msg::Marker curr_mk;
          curr_mk.header = path.header;
          curr_mk.ns = "vehicle_footprints";
          curr_mk.id = id++;
          curr_mk.type = visualization_msgs::msg::Marker::CUBE;
          curr_mk.action = visualization_msgs::msg::Marker::ADD;
          
          double yaw = yaw_from_quat(pose.pose.orientation);
          
          // Position: Shift from Axle to Geometric Center
          curr_mk.pose.position.x = pose.pose.position.x + center_offset * std::cos(yaw);
          curr_mk.pose.position.y = pose.pose.position.y + center_offset * std::sin(yaw);
          curr_mk.pose.position.z = 0.75; // Half of 1.5 height
          
          curr_mk.pose.orientation = pose.pose.orientation;
          
          curr_mk.scale.x = vehicle_length_;
          curr_mk.scale.y = vehicle_width_;
          curr_mk.scale.z = 1.5;
          
          curr_mk.color.r = 0.0;
          curr_mk.color.g = 0.5;
          curr_mk.color.b = 1.0;
          curr_mk.color.a = 0.3; // Transparent
          
          mk_array.markers.push_back(curr_mk);
      }
      
      pub_markers_->publish(mk_array);
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
