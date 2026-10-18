#!/usr/bin/env python3
"""
box_obstacles: turns ArUco box detections into an occupancy grid for Nav2.

Subscribes
  /aruco/markers (aruco_interfaces/ArucoMarkers)  poses in camera_link_optical
  /map           (nav_msgs/OccupancyGrid)         only used to copy the grid geometry
Publishes
  /box_map                  (nav_msgs/OccupancyGrid)  boxes only, same geometry as /map
  /box_obstacles/markers    (visualization_msgs/MarkerArray)  footprints for RViz

/box_map is consumed by an extra nav2_costmap_2d::StaticLayer in the global and
local costmaps (see nav2_params.yaml), placed before the inflation layer.
"""

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
from rclpy.time import Time

import tf2_ros
from aruco_interfaces.msg import ArucoMarkers
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

from bnav.box_map import (BoxTracker, box_from_tag, pose_to_matrix,
                          rasterize_boxes)


class BoxObstaclesNode(Node):
    def __init__(self):
        super().__init__("box_obstacles")

        p = self.declare_parameter
        self.markers_topic = p("markers_topic", "/aruco/markers").value
        self.base_map_topic = p("base_map_topic", "/map").value
        self.grid_topic = p("grid_topic", "/box_map").value
        self.viz_topic = p("viz_topic", "/box_obstacles/markers").value
        self.target_frame = p("target_frame", "map").value
        self.box_ids = set(p("box_marker_ids", [13]).value)

        # Box geometry (lying on its long side)
        self.box_length = p("box_length", 0.32).value
        self.box_width = p("box_width", 0.11).value
        self.tag_to_center = p("tag_to_center", 0.055).value
        self.long_axis_index = {"x": 0, "y": 1}[p("tag_long_axis", "x").value]
        self.footprint_padding = p("footprint_padding", 0.0).value

        # The detector computes every pose assuming `assumed_marker_size`
        # (aruco_node's marker_size). If the box tag really has another size the
        # estimated distance is off by the ratio; fix it here.
        self.assumed_marker_size = p("assumed_marker_size", 0.071).value
        self.box_tag_size = p("box_tag_size", 0.080).value

        # Tracking
        self.timeout = p("timeout", 0.0).value
        self.no_message_timeout = p("no_message_timeout", 1.0).value
        self.association_distance = p("association_distance", 0.08).value
        self.max_boxes = p("max_boxes", 15).value
        self.tf_timeout = p("tf_timeout", 0.1).value
        publish_rate = p("publish_rate", 10.0).value

        self.tracker = BoxTracker(self.association_distance, self.timeout,
                                  self.max_boxes)
        self.scale = (self.box_tag_size / self.assumed_marker_size
                      if self.box_tag_size > 0 and self.assumed_marker_size > 0 else 1.0)
        self.base_map = None
        self.last_grid = None
        self.last_publish = 0.0
        self.heartbeat = p("heartbeat_period", 2.0).value  # s, republish even if unchanged

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(OccupancyGrid, self.base_map_topic,
                                 self.on_base_map, latched)
        self.create_subscription(ArucoMarkers, self.markers_topic,
                                 self.on_markers, best_effort)
        self.grid_pub = self.create_publisher(OccupancyGrid, self.grid_topic, latched)
        self.viz_pub = self.create_publisher(MarkerArray, self.viz_topic, 10)
        self.create_timer(1.0 / publish_rate, self.on_timer)

        self.get_logger().info(
            f"ids={sorted(self.box_ids)} timeout={self.timeout}s "
            f"size scale={self.scale:.3f} long axis=tag {['x', 'y'][self.long_axis_index]}")

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_base_map(self, msg):
        o = msg.info.origin.orientation
        if abs(o.z) > 1e-6 or abs(o.x) > 1e-6 or abs(o.y) > 1e-6:
            self.get_logger().warning("map origin is rotated; box grid assumes yaw = 0")
        self.base_map = msg

    def on_markers(self, msg):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id, Time(),
                Duration(seconds=self.tf_timeout))
        except tf2_ros.TransformException as e:
            self.get_logger().warning(f"no TF {self.target_frame} <- {msg.header.frame_id}: {e}",
                                   throttle_duration_sec=2.0)
            return

        tr, ro = tf.transform.translation, tf.transform.rotation
        t_map_cam = pose_to_matrix([tr.x, tr.y, tr.z], [ro.x, ro.y, ro.z, ro.w])
        cam_origin = t_map_cam[:3, 3]

        detections = []
        for marker_id, pose in zip(msg.marker_ids, msg.poses):
            if int(marker_id) not in self.box_ids:
                continue
            pos = np.array([pose.position.x, pose.position.y, pose.position.z]) * self.scale
            q = pose.orientation
            t_cam_tag = pose_to_matrix(pos, [q.x, q.y, q.z, q.w])
            t_map_tag = t_map_cam @ t_cam_tag

            cx, cy, yaw, ok = box_from_tag(t_map_tag, self.tag_to_center,
                                           self.long_axis_index)
            if ok:
                hl, hw = self.box_length / 2.0, self.box_width / 2.0
            else:
                # Long axis is vertical: not lying down. Be conservative with a
                # square of the box length until standing boxes are supported.
                hl = hw = self.box_length / 2.0
                self.get_logger().warning("box tag long axis is vertical; using square footprint",
                                       throttle_duration_sec=5.0)

            # quality = how face-on the tag is to the camera
            to_cam = cam_origin - t_map_tag[:3, 3]
            quality = float(np.dot(t_map_tag[:3, 2], to_cam) / (np.linalg.norm(to_cam) + 1e-9))
            detections.append(dict(x=cx, y=cy, yaw=yaw, hl=hl, hw=hw, quality=quality))

        if self.tracker.update(detections, self.now_s()):
            self.get_logger().warning(f"more than {self.max_boxes} boxes detected, ignoring extras",
                                   throttle_duration_sec=5.0)

    def on_timer(self):
        self.tracker.expire(self.now_s(), self.no_message_timeout)
        boxes = self.tracker.boxes()
        stamp = self.get_clock().now().to_msg()

        if self.base_map is None:
            self.get_logger().warning(f"waiting for {self.base_map_topic} to copy its geometry",
                                   throttle_duration_sec=5.0)
        else:
            info = self.base_map.info
            grid = rasterize_boxes(boxes, info.width, info.height, info.resolution,
                                   info.origin.position.x, info.origin.position.y,
                                   self.footprint_padding)
            now = self.now_s()
            changed = self.last_grid is None or not np.array_equal(grid, self.last_grid)
            if changed or now - self.last_publish >= self.heartbeat:
                out = OccupancyGrid()
                out.header.stamp = stamp
                out.header.frame_id = self.base_map.header.frame_id
                out.info = info
                out.data = grid.ravel()
                self.grid_pub.publish(out)
                self.last_grid = grid
                self.last_publish = now

        self.publish_markers(boxes, stamp)

    def publish_markers(self, boxes, stamp):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, b in enumerate(boxes):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self.target_frame
            m.ns = "boxes"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y = b["x"], b["y"]
            m.pose.position.z = self.box_width / 2.0
            m.pose.orientation.z = float(np.sin(b["yaw"] / 2.0))
            m.pose.orientation.w = float(np.cos(b["yaw"] / 2.0))
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