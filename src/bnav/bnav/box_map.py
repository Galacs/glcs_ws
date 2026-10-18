#!/usr/bin/env python3
"""
ROS-free helpers for box_obstacles_node: tag pose -> box footprint, tracking
with a configurable timeout, and rasterization into an occupancy grid.

Conventions
-----------
* T_a_b is a 4x4 matrix mapping points in frame b into frame a.
* Quaternions are (x, y, z, w), like geometry_msgs.
* The tag frame is the OpenCV marker frame produced by aruco_node: x right,
  y up along the marker, z pointing OUT of the marker towards the viewer.
  The box centre is therefore at  tag_position - R * [0, 0, tag_to_center].
* Boxes are assumed to lie on their long side, so the footprint is
  length x width (0.32 x 0.11) and its long axis is one of the tag's in-plane
  axes (selected by `long_axis_index`).
"""

import math

import numpy as np


def quat_to_matrix(q):
    x, y, z, w = np.asarray(q, dtype=np.float64) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def pose_to_matrix(position, quaternion):
    t = np.eye(4)
    t[:3, :3] = quat_to_matrix(quaternion)
    t[:3, 3] = position
    return t


def box_from_tag(t_map_tag, tag_to_center, long_axis_index=0, min_horizontal=0.3):
    """
    Box centre (map x, y) and yaw of its long axis from the pose of one tag.

    Works for any face that is visible (top or a side face), because the centre
    is always `tag_to_center` behind the tag along its -Z, and the long axis is
    always parallel to a fixed in-plane tag axis.

    Returns (cx, cy, yaw, ok). ok is False when the long axis is (nearly)
    vertical, i.e. the box is not lying down; the yaw is then meaningless.
    """
    r = t_map_tag[:3, :3]
    p = t_map_tag[:3, 3]
    centre = p - r[:, 2] * tag_to_center
    axis = r[:, long_axis_index]
    if math.hypot(axis[0], axis[1]) < min_horizontal:
        return float(centre[0]), float(centre[1]), 0.0, False
    return float(centre[0]), float(centre[1]), float(math.atan2(axis[1], axis[0])), True


class BoxTracker:
    """
    Keeps the current set of boxes.

    timeout <= 0 : the box set is exactly what the latest message contained.
    timeout  > 0 : a box that is no longer detected is kept for `timeout`
                   seconds after it was last seen.

    Boxes all share the same marker id, so there is no identity: detections are
    matched to existing boxes by distance (greedy nearest). Detections of the
    same box through two faces in one message are merged, keeping the one seen
    most face-on (highest `quality`).
    """

    def __init__(self, association_distance=0.08, timeout=0.0, max_boxes=15):
        self.association_distance = association_distance
        self.timeout = timeout
        self.max_boxes = max_boxes
        self.tracks = []

    def update(self, detections, now):
        """
        detections: list of dicts with x, y, yaw, hl, hw (half length / width),
                    quality. Returns True if some detection was dropped because
                    max_boxes was reached.
        """
        dist = self.association_distance
        overflow = False

        kept = []
        for d in sorted(detections, key=lambda d: -d["quality"]):
            if all(math.hypot(d["x"] - k["x"], d["y"] - k["y"]) > dist for k in kept):
                kept.append(d)

        for t in self.tracks:
            t["seen_now"] = False

        for d in kept:
            best, best_d = None, dist
            for t in self.tracks:
                if t["seen_now"]:
                    continue
                dd = math.hypot(d["x"] - t["x"], d["y"] - t["y"])
                if dd <= best_d:
                    best, best_d = t, dd
            if best is None:
                if len(self.tracks) >= self.max_boxes:
                    stale = [t for t in self.tracks if not t["seen_now"]]
                    if not stale:
                        overflow = True
                        continue
                    self.tracks.remove(min(stale, key=lambda t: t["last_seen"]))
                best = {}
                self.tracks.append(best)
            best.update(x=d["x"], y=d["y"], yaw=d["yaw"], hl=d["hl"], hw=d["hw"],
                        last_seen=now, seen_now=True)

        if self.timeout <= 0.0:
            self.tracks = [t for t in self.tracks if t["seen_now"]]
        return overflow

    def expire(self, now, no_message_timeout):
        """
        Call periodically. With timeout > 0 drops boxes not seen for `timeout`.
        With timeout <= 0 boxes are removed as soon as a message omits them, so
        this only acts as a safety net if messages stop arriving altogether.
        """
        limit = self.timeout if self.timeout > 0.0 else no_message_timeout
        self.tracks = [t for t in self.tracks if now - t["last_seen"] <= limit]

    def boxes(self):
        return list(self.tracks)


def rasterize_boxes(boxes, width, height, resolution, origin_x, origin_y,
                    padding=0.0):
    """
    Occupancy grid (height x width, int8, 0 free / 100 occupied) containing the
    given rectangles. Each box is a dict with x, y, yaw, hl, hw. Rectangles are
    grown by half a cell so that thin boxes never fall between cell centres.
    Assumes the grid is not rotated (origin yaw = 0).
    """
    grid = np.zeros((height, width), dtype=np.int8)
    grow = padding + resolution / 2.0
    for b in boxes:
        hl, hw = b["hl"] + grow, b["hw"] + grow
        radius = math.hypot(hl, hw)
        i0 = max(int(math.floor((b["x"] - radius - origin_x) / resolution)), 0)
        i1 = min(int(math.ceil((b["x"] + radius - origin_x) / resolution)) + 1, width)
        j0 = max(int(math.floor((b["y"] - radius - origin_y) / resolution)), 0)
        j1 = min(int(math.ceil((b["y"] + radius - origin_y) / resolution)) + 1, height)
        if i0 >= i1 or j0 >= j1:
            continue
        xs = origin_x + (np.arange(i0, i1) + 0.5) * resolution - b["x"]
        ys = origin_y + (np.arange(j0, j1) + 0.5) * resolution - b["y"]
        dx, dy = np.meshgrid(xs, ys)
        c, s = math.cos(b["yaw"]), math.sin(b["yaw"])
        u = c * dx + s * dy
        v = -s * dx + c * dy
        mask = (np.abs(u) <= hl) & (np.abs(v) <= hw)
        grid[j0:j1, i0:i1][mask] = 100
    return grid