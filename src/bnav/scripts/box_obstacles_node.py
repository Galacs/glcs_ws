#!/usr/bin/env python3
"""
box_obstacles: turns the boxes detected by aruco_node into an occupancy grid for Nav2.

Subscribes
  /aruco/boxes (geometry_msgs/PoseArray)  box centres + yaw in the map frame (from aruco_node)
  /map         (nav_msgs/OccupancyGrid)   only used to copy the grid geometry
Publishes
  /box_map               (nav_msgs/OccupancyGrid)  boxes only, same geometry as /map
  /box_obstacles/markers (visualization_msgs/MarkerArray)  footprints for RViz

/box_map is consumed by an extra nav2_costmap_2d::StaticLayer in the global and
local costmaps (see nav2_params.yaml), placed before the inflation layer.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from bnav.box_map import BoxTracker, rasterize_boxes


class BoxObstaclesNode(Node):
    def __init__(self):
        super().__init__("box_obstacles")

        p = self.declare_parameter
        self.boxes_topic = p("boxes_topic", "/aruco/boxes").value
        self.base_map_topic = p("base_map_topic", "/map").value
        self.grid_topic = p("grid_topic", "/box_map").value
        self.viz_topic = p("viz_topic", "/box_obstacles/markers").value
        self.box_length = p("box_length", 0.32).value
        self.box_width = p("box_width", 0.11).value
        self.footprint_padding = p("footprint_padding", 0.0).value
        self.timeout = p("timeout", 0.0).value
        self.no_message_timeout = p("no_message_timeout", 1.0).value
        self.heartbeat = p("heartbeat_period", 2.0).value
        publish_rate = p("publish_rate", 10.0).value

        self.tracker = BoxTracker(p("association_distance", 0.08).value, self.timeout,
                                  p("max_boxes", 15).value)
        self.base_map = None
        self.last_grid = None
        self.last_publish = 0.0

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, self.base_map_topic, self.on_base_map, latched)
        self.create_subscription(PoseArray, self.boxes_topic, self.on_boxes, 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, self.grid_topic, latched)
        self.viz_pub = self.create_publisher(MarkerArray, self.viz_topic, 10)
        self.create_timer(1.0 / publish_rate, self.on_timer)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_base_map(self, msg):
        self.base_map = msg

    def on_boxes(self, msg):
        if self.base_map and msg.header.frame_id != self.base_map.header.frame_id:
            self.get_logger().warning(
                f"boxes are in '{msg.header.frame_id}', map is '{self.base_map.header.frame_id}'",
                throttle_duration_sec=5.0)
        boxes = [dict(x=b.position.x, y=b.position.y,
                      yaw=2.0 * math.atan2(b.orientation.z, b.orientation.w),
                      hl=self.box_length / 2.0, hw=self.box_width / 2.0)
                 for b in msg.poses]
        if self.tracker.update(boxes, self.now_s()):
            self.get_logger().warning("too many boxes, ignoring extras", throttle_duration_sec=5.0)

    def on_timer(self):
        self.tracker.expire(self.now_s(), self.no_message_timeout)
        boxes = self.tracker.boxes()
        stamp = self.get_clock().now().to_msg()

        if self.base_map is None:
            self.get_logger().warning(f"waiting for {self.base_map_topic}", throttle_duration_sec=5.0)
        else:
            info = self.base_map.info
            grid = rasterize_boxes(boxes, info.width, info.height, info.resolution,
                                   info.origin.position.x, info.origin.position.y,
                                   self.footprint_padding)
            now = self.now_s()
            if (self.last_grid is None or not np.array_equal(grid, self.last_grid)
                    or now - self.last_publish >= self.heartbeat):
                out = OccupancyGrid()
                out.header.stamp = stamp
                out.header.frame_id = self.base_map.header.frame_id
                out.info = info
                out.data = grid.ravel()
                self.grid_pub.publish(out)
                self.last_grid, self.last_publish = grid, now

        self.publish_markers(boxes, stamp)

    def publish_markers(self, boxes, stamp):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        frame = self.base_map.header.frame_id if self.base_map else "map"
        for i, b in enumerate(boxes):
            m = Marker()
            m.header.stamp, m.header.frame_id = stamp, frame
            m.ns, m.id, m.type, m.action = "boxes", i, Marker.CUBE, Marker.ADD
            m.pose.position.x, m.pose.position.y = b["x"], b["y"]
            m.pose.position.z = self.box_width / 2.0
            m.pose.orientation.z = math.sin(b["yaw"] / 2.0)
            m.pose.orientation.w = math.cos(b["yaw"] / 2.0)
            m.scale.x, m.scale.y, m.scale.z = 2 * b["hl"], 2 * b["hw"], self.box_width
            m.color.r, m.color.g, m.color.b, m.color.a = 0.8, 0.5, 0.2, 0.7
            arr.markers.append(m)
        self.viz_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = BoxObstaclesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()