#!/usr/bin/env python3
import time
import subprocess
import os
import signal
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from control_task_msgs.msg import TaskGoalData

class VerificationNode(Node):
    def __init__(self):
        super().__init__('verification_node')
        self.pub_start = self.create_publisher(PoseStamped, '/start_pose', 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.sub_path = self.create_subscription(Path, '/planner/global_path', self.on_path, 10)
        
        self.path_received = False
        self.received_path = None

    def on_path(self, msg):
        self.get_logger().info(f"Received path with {len(msg.poses)} poses")
        self.received_path = msg
        self.path_received = True

    def send_start(self, x, y, yaw=0.0):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = 1.0 # Simplified
        self.pub_start.publish(msg)
        self.get_logger().info(f"Published Start: ({x}, {y})")

    def send_goal(self, x, y, yaw=0.0):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = 1.0 # Simplified
        self.pub_goal.publish(msg)
        self.get_logger().info(f"Published Goal: ({x}, {y})")

def run_test():
    # 1. Start System Nodes
    print("Starting nodes...")
    
    # Source environment
    setup_cmd = "source /home/nvidia/tju_auto_ws/install/setup.bash && "
    
    # Mock Map (Blank map 50x50m, resolution 0.1)
    # Origin at (0,0) so typical range is 0..50
    cmd_map = setup_cmd + "python3 /home/nvidia/tju_auto_ws/src/dynamic_map/scripts/mock_map_pub.py --width 500 --height 500 --resolution 0.1"
    proc_map = subprocess.Popen(cmd_map, shell=True, executable='/bin/bash', preexec_fn=os.setsid)
    
    # Scheduler
    # cmd_sch = setup_cmd + "ros2 run high_level_scheduler scheduler_node"
    # proc_sch = subprocess.Popen(cmd_sch, shell=True, executable='/bin/bash', preexec_fn=os.setsid)
    
    # Planner
    # cmd_plan = setup_cmd + "ros2 run path_planner planner_node"
    # Use direct path to avoid 'Package not found' issues in subprocess
    cmd_plan = setup_cmd + "/home/nvidia/tju_auto_ws/install/path_planner/lib/path_planner/planner_node"
    # Capture planner output to check for logs
    proc_plan = subprocess.Popen(cmd_plan, shell=True, executable='/bin/bash', preexec_fn=os.setsid, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    try:
        rclpy.init()
        node = VerificationNode()
        
        # Give nodes time to spin up
        print("Waiting for nodes to initialize (3s)...")
        time.sleep(3)
        
        # Spin a bit to ensure connections
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)

        # 2. Send Start Pose
        start_x, start_y = 5.0, 5.0
        node.send_start(start_x, start_y)
        
        # Wait a bit
        time.sleep(1)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)

        # 3. Send Goal Pose
        goal_x, goal_y = 10.0, 10.0
        node.send_goal(goal_x, goal_y)
        
        # 4. Wait for Path
        print("Waiting for path...")
        start_wait = time.time()
        while time.time() - start_wait < 5.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.path_received:
                break
        
        if node.path_received:
            path = node.received_path
            p_start = path.poses[0].pose.position
            p_end = path.poses[-1].pose.position
            
            print(f"\nOptimization Result:")
            print(f"Requested Start: ({start_x}, {start_y})")
            print(f"Path Start:      ({p_start.x:.3f}, {p_start.y:.3f})")
            print(f"Requested Goal:  ({goal_x}, {goal_y})")
            print(f"Path End:        ({p_end.x:.3f}, {p_end.y:.3f})")
            
            # Validation
            dist_start = ((p_start.x - start_x)**2 + (p_start.y - start_y)**2)**0.5
            dist_end = ((p_end.x - goal_x)**2 + (p_end.y - goal_y)**2)**0.5
            
            print(f"Start Drift: {dist_start:.4f} m")
            print(f"Goal Drift:  {dist_end:.4f} m")
            
            if dist_start < 0.2 and dist_end < 0.2:
                print("SUCCESS: Path is aligned.")
            else:
                print("FAILURE: Path drifted > 0.2m")
        else:
            print("FAILURE: No path received.")
            outs, errs = proc_plan.communicate(timeout=1)
            print("Planner Output:")
            print(outs.decode())
            print("Planner Error:")
            print(errs.decode())
            
    except Exception as e:
        print(f"Error: {e}")
        
    finally:
        print("Shutting down...")
        os.killpg(os.getpgid(proc_map.pid), signal.SIGTERM)
        # os.killpg(os.getpgid(proc_sch.pid), signal.SIGTERM)
        os.killpg(os.getpgid(proc_plan.pid), signal.SIGTERM)
        rclpy.shutdown()

if __name__ == '__main__':
    run_test()
