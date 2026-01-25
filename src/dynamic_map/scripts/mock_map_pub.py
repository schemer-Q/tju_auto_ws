#!/usr/bin/env python3
import argparse
import math
import os
import sys
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from control_task_msgs.msg import TaskGoalData
from common_msgs.msg import PosePoint
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import TransformBroadcaster

def quaternion_to_yaw(q):
    """Simple conversion (assuming almost planar)"""
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw):
    """Convert planar yaw to quaternion (x=y=0, z,w set)."""
    half = yaw * 0.5
    q = type('Q', (), {})()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def read_pgm(path):
    with open(path, 'rb') as f:
        header = f.readline().strip()
        if header not in [b'P5', b'P2']:
            raise ValueError(f"Unsupported PGM header: {header}")

        def _read_non_comment():
            line = f.readline()
            while line.startswith(b'#'):
                line = f.readline()
            return line

        dims = _read_non_comment().split()
        if len(dims) != 2:
            # some files put width/height on separate lines
            dims += _read_non_comment().split()
        width, height = int(dims[0]), int(dims[1])
        maxval = int(_read_non_comment().strip())
        if maxval > 255:
            raise ValueError("Only 8-bit PGM supported")

        data = f.read(width * height)
        if len(data) != width * height:
            raise ValueError("PGM data length mismatch")
        return width, height, maxval, list(data)


def load_map_from_yaml(yaml_path):
    with open(yaml_path, 'r') as f:
        y = yaml.safe_load(f)
    img_path = y['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(yaml_path), img_path)

    resolution = float(y.get('resolution', 0.05))
    origin = y.get('origin', [0.0, 0.0, 0.0])
    negate = int(y.get('negate', 0))
    occ_thresh = float(y.get('occupied_thresh', 0.65))
    free_thresh = float(y.get('free_thresh', 0.196))

    width, height, maxval, data = read_pgm(img_path)

    def to_occ(v):
        if negate:
            v = 255 - v
        occ_prob = (255 - v) / 255.0  # darker -> higher occupancy
        if occ_prob > occ_thresh:
            return 100
        if occ_prob < free_thresh:
            return 0
        return -1

    occupancy = [to_occ(v) for v in data]
    return {
        'width': width,
        'height': height,
        'resolution': resolution,
        'origin': origin,
        'occupancy': occupancy,
    }


class MockMapPublisher(Node):
    def __init__(self, args):
        super().__init__('mock_map_pub')
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(OccupancyGrid, '/map/combined', qos)
        self.pub_task = self.create_publisher(TaskGoalData, '/planner/task', 10)
        # Publisher to forward initialpose to start_pose for the planner
        self.pub_start = self.create_publisher(PoseStamped, '/start_pose', 10)

        # Subscribe to PoseStamped start and goal
        self.sub_start = self.create_subscription(
            PoseStamped,
            '/start_pose',
            self.on_start_pose,
            10
        )
        self.sub_goal = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.on_goal_pose,
            10
        )
        # Bind RViz 2D Pose Estimate (PoseWithCovarianceStamped)
        self.sub_init = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.on_initialpose,
            10
        )
        self.get_logger().info("Subscribed to /start_pose, /goal_pose (PoseStamped) and /initialpose (PoseWithCovarianceStamped)")
        
        self.frame_id = args.frame_id
        self.timer_period = 1.0 / args.rate
        self.task_order_id = 0

        # Initialize start_pose to origin so TF can be broadcast immediately
        self.start_pose = PoseStamped()
        self.start_pose.header.frame_id = 'map'
        self.start_pose.pose.orientation.w = 1.0

        self.last_goal_pose = None
        self.map_published = False
        self.last_sub_count = 0

        # TF broadcaster to publish map->base_link based on start_pose
        self.tf_broadcaster = TransformBroadcaster(self)

        if args.map_yaml:
            map_data = load_map_from_yaml(args.map_yaml)
            self.width = map_data['width']
            self.height = map_data['height']
            self.resolution = map_data['resolution']
            self.origin = map_data['origin']
            self.base_origin = list(self.origin)
            self.occupancy = map_data['occupancy']
            self.get_logger().info(f"Loaded map from {args.map_yaml} ({self.width}x{self.height}, res={self.resolution})")
        else:
            self.width = args.width
            self.height = args.height
            self.resolution = args.resolution
            self.origin = [args.origin_x, args.origin_y, 0.0]
            self.base_origin = list(self.origin)
            self.occupancy = [0] * (self.width * self.height)
            self.get_logger().info(f"Publishing blank map {self.width}x{self.height}, res={self.resolution}")

        # Initial publish; RViz uses latched message
        self.on_timer()
        # Republish when new subscribers connect
        self.timer = self.create_timer(self.timer_period, self.on_timer)
        # TF broadcast enabled to provide map->base_link transform for visualization
        self.tf_timer = self.create_timer(0.1, self.broadcast_tf)

    def publish_map_unconditional(self):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # global frame
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        # Use original YAML origin in map frame
        msg.info.origin.position.x = float(self.base_origin[0])
        msg.info.origin.position.y = float(self.base_origin[1])
        msg.info.origin.position.z = 0.0
        q = yaw_to_quaternion(float(self.base_origin[2]))
        msg.info.origin.orientation.x = q.x
        msg.info.origin.orientation.y = q.y
        msg.info.origin.orientation.z = q.z
        msg.info.origin.orientation.w = q.w
        msg.data = self.occupancy
        self.pub.publish(msg)
        self.map_published = True

    def shifted_origin(self):
        if self.start_pose is None:
            return self.base_origin
        return [
            self.base_origin[0] - self.start_pose.pose.position.x,
            self.base_origin[1] - self.start_pose.pose.position.y,
            self.base_origin[2]
        ]

    def on_start_pose(self, msg: PoseStamped):
        self.start_pose = msg
        yaw = quaternion_to_yaw(msg.pose.orientation)
        self.get_logger().info(
            f"Set start pose (PoseStamped): x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, yaw={yaw:.3f}, frame={msg.header.frame_id}"
        )
        # Do NOT auto re-plan when start changes - wait for explicit goal

    def broadcast_tf(self):
        """Publish TF from 'map' (global) to 'base_link' (local frame) based on start_pose."""
        if self.start_pose is None:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.start_pose.pose.position.x
        t.transform.translation.y = self.start_pose.pose.position.y
        t.transform.translation.z = self.start_pose.pose.position.z
        t.transform.rotation = self.start_pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)
    
    def on_initialpose(self, msg: PoseWithCovarianceStamped):
        """Handle 2D Pose Estimate from RViz and republish as /start_pose."""
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        # Publish to /start_pose so planner_node receives it
        self.pub_start.publish(ps)
        # on_start_pose will be called via subscription callback
        self.get_logger().info(f"Received /initialpose, republished to /start_pose")

    def on_goal_pose(self, msg):
        """Handle 2D Goal Pose from RViz and publish TaskGoalData for planner"""
        self.task_order_id += 1
        yaw_goal = quaternion_to_yaw(msg.pose.orientation)
        self.get_logger().info(
            f"Received Goal Pose (PoseStamped): x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, yaw={yaw_goal:.3f}, frame={msg.header.frame_id}"
        )
        self.last_goal_pose = msg
        self.publish_task_for_goal(msg)

    def publish_task_for_goal(self, msg: PoseStamped):
        """Create and publish TaskGoalData from goal/start PoseStamped in map coordinates."""
        self.task_order_id += 1

        task_msg = TaskGoalData()
        task_msg.header = Header()
        task_msg.header.stamp = self.get_clock().now().to_msg()
        task_msg.header.frame_id = 'map'  # global frame
        task_msg.order_id = self.task_order_id
        
        # PosePoint with goal in map coordinates
        pp = PosePoint()
        pp.x = msg.pose.position.x
        pp.y = msg.pose.position.y
        pp.z = 0.0
        pp.pitch = 0.0
        pp.roll = 0.0
        pp.yaw = quaternion_to_yaw(msg.pose.orientation)
        pp.speed = 0.0
        pp.curve = 0.0
        pp.acc = 0.0
        pp.gear = 3
        task_msg.end_point = pp
        if self.start_pose is not None:
            self.get_logger().info(
                f"Start pose used: x={self.start_pose.pose.position.x:.3f}, y={self.start_pose.pose.position.y:.3f}"
            )
        self.get_logger().info(
            f"Publishing TaskGoalData id={task_msg.order_id} goal=({task_msg.end_point.x:.3f}, {task_msg.end_point.y:.3f}), yaw={task_msg.end_point.yaw:.3f} in map frame"
        )
        self.pub_task.publish(task_msg)

    def on_timer(self):
        sub_count = self.pub.get_subscription_count()
        if (not self.map_published) or (sub_count != self.last_sub_count):
            msg = OccupancyGrid()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.info.resolution = self.resolution
            msg.info.width = self.width
            msg.info.height = self.height
            org = self.shifted_origin()
            msg.info.origin.position.x = float(org[0])
            msg.info.origin.position.y = float(org[1])
            msg.info.origin.position.z = 0.0
            # Apply yaw from YAML origin (org[2])
            q = yaw_to_quaternion(float(org[2]))
            msg.info.origin.orientation.x = q.x
            msg.info.origin.orientation.y = q.y
            msg.info.origin.orientation.z = q.z
            msg.info.origin.orientation.w = q.w
            msg.data = self.occupancy
            self.pub.publish(msg)
            self.map_published = True
            self.last_sub_count = sub_count


def parse_args():
    parser = argparse.ArgumentParser(description='Mock map publisher for /map/combined')
    parser.add_argument('--map_yaml', type=str, default=None, help='Path to map yaml (with pgm)')
    parser.add_argument('--width', type=int, default=50, help='Width of blank map (cells)')
    parser.add_argument('--height', type=int, default=50, help='Height of blank map (cells)')
    parser.add_argument('--resolution', type=float, default=0.1, help='Resolution (m/cell) for blank map')
    parser.add_argument('--origin_x', type=float, default=0.0, help='Origin x for blank map')
    parser.add_argument('--origin_y', type=float, default=0.0, help='Origin y for blank map')
    parser.add_argument('--rate', type=float, default=5.0, help='Publish rate in Hz')
    parser.add_argument('--frame_id', type=str, default='map', help='Frame id for published map')
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = MockMapPublisher(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
