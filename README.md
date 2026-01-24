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

## 构建与运行（ROS 2 Foxy - C++ / ament_cmake）

快速步骤：

1. 在终端加载 ROS Foxy 环境并构建工作区：

```bash
source /opt/ros/foxy/setup.bash
cd /home/nvidia/tju_auto_ws
colcon build --symlink-install
source install/setup.bash
```

2. 启动节点（在不同终端分别运行）：

```bash
# 启动动态地图节点
ros2 run dynamic_map dynamic_map_node

# 启动高层调度节点
ros2 run high_level_scheduler scheduler_node

# 启动路径规划节点
ros2 run path_planner planner_node
```

或者使用包内的 launch 文件：

```bash
ros2 launch dynamic_map dynamic_map_launch.py
ros2 launch high_level_scheduler scheduler_launch.py
ros2 launch path_planner planner_launch.py
```

3. 示例任务发布（需构造 `control_task_msgs/TaskGoalData` 消息并发布到 `/task_goal`）：

```bash
# 示例：使用 rclpy/rclcpp 节点或自定义发布工具发布 TaskGoalData
```

## 已实现的话题（摘要）

- `/map/static` (nav_msgs/OccupancyGrid) - 静态层
- `/map/dynamic` (nav_msgs/OccupancyGrid) - 动态障碍物层
- `/map/inflated` (nav_msgs/OccupancyGrid) - 膨胀层
- `/map/combined` (nav_msgs/OccupancyGrid) - 合并后栅格
- `/task_goal` (control_task_msgs/TaskGoalData) - 上层任务输入（建议话题名）
- `/planner/task` (control_task_msgs/TaskGoalData) - 调度发往规划器的任务
- `/planner/global_path` (nav_msgs/Path) - 规划器输出路径

## 自定义消息位置

自定义消息位于工作区 `src/tju_msgs`：

- `control_task_msgs/msg/TaskGoalData.msg`
- `common_msgs/msg/PosePoint.msg`

## 说明与建议的下一步

- 当前实现为可运行原型：动态地图节点发布示例占据栅格，调度器做消息转发，规划器生成简单直线路径。
- 建议下一步完善：
  - 在 `dynamic_map` 中加入参数化的膨胀半径与更高效的膨胀算法；
  - 在 `high_level_scheduler` 中实现任务队列、优先级与状态反馈；
  - 在 `path_planner` 中集成更完善的全局/局部规划算法与轨迹优化。

如果你同意，我可以依次实现上述一项并运行构建和集成测试。
