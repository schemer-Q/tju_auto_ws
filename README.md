# 车辆路径规划系统 (ROS Foxy)

基于ROS 2 Foxy的模块化车辆路径规划系统，包含动态地图构建、高层调度和路径规划三个核心功能层。

## 系统架构

┌─────────────────────────────────────────────────────┐
│                  高层调度模块                        │
│      (high_level_scheduler)                         │
│                                                     │
│  • 任务队列管理                                     │
│  • 状态机控制                                       │
│  • 模块间协调                                       │
└───────────────┬─────────────────────────────────────┘
                │ ROS 2接口
                ▼
┌─────────────────────────────────────────────────────┐
│                  路径规划模块                        │
│      (path_planner)                                │
│                                                     │
│  • 混合A*                                         │
│  • 轨迹优化                                        │
│  • 碰撞检测                                        │
└───────────────┬─────────────────────────────────────┘
                │ ROS 2接口
                ▼
┌─────────────────────────────────────────────────────┐
│                  动态地图模块                        │
│      (dynamic_map)                                 │
│                                                     │
│  • 静态层 (SLAM/预建图)                            │
│  • 动态层 (障碍物检测)                             │
│  • 膨胀层 (安全边界)                               │
└─────────────────────────────────────────────────────┘

## 功能模块
### 1. 动态地图模块 (dynamic_map)

功能: 实时构建和维护占据栅格地图，支持分层表示

核心组件:

    静态层: 基于激光雷达/GNSS的静态环境地图

    动态层: 实时检测和跟踪动态障碍物

    膨胀层: 为障碍物添加安全边界，考虑车辆尺寸

主要话题:

    /map/static (nav_msgs/OccupancyGrid) - 静态地图

    /map/dynamic (nav_msgs/OccupancyGrid) - 动态障碍物

    /map/inflated (nav_msgs/OccupancyGrid) - 膨胀后最终地图

    /sensor/lidar (sensor_msgs/LaserScan) - 激光雷达数据

    /sensor/odometry (nav_msgs/Odometry) - 里程计数据

### 2. 高层调度模块 (high_level_scheduler)

功能: 系统任务调度和模块协调

核心功能:

    接收上层任务指令（目标点、任务类型）

    管理任务队列和优先级

    监控系统状态和异常处理

    协调动态地图和路径规划模块工作流

主要话题:

    /mission/task (自定义消息) - 任务指令

    /mission/status (自定义消息) - 系统状态

    /scheduler/plan_request (自定义消息) - 路径规划请求

    /scheduler/plan_result (自定义消息) - 路径规划结果

### 3. 路径规划模块 (path_planner)

功能: 基于动态地图进行路径规划

核心功能:

    全局路径规划（A*、Dijkstra等）

    局部轨迹规划（DWA、TEB等）

    实时避障和重规划

    轨迹平滑和优化

主要话题:

    /planner/global_path (nav_msgs/Path) - 全局路径

    /planner/local_trajectory (nav_msgs/Path) - 局部轨迹

    /planner/waypoints (geometry_msgs/PoseArray) - 路径点

    /planner/status (std_msgs/String) - 规划器状态