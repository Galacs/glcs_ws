#!/usr/bin/env python3
"""
ROS-free helpers for box_obstacles_node: tracking of boxes with a configurable
timeout, and rasterization of their footprints into an occupancy grid.

The box centres and yaw come from aruco_node (/aruco/boxes, in the map frame).
"""

import math

import numpy as np


class BoxTracker:
    """
    Keeps the current set of boxes.

    timeout <= 0 : the box set is exactly what the latest message contained.
    timeout  > 0 : a box that is no longer detected is kept for `timeout`
                   seconds after it was last seen.

    Boxes all share the same marker id, so there is no identity: detections are
    matched to existing boxes by distance (greedy nearest). Detections of the
    same box through two faces in one message are merged, keeping the one seen
    first.
    """

    def __init__(self, association_distance=0.08, timeout=0.0, max_boxes=15):
        self.association_distance = association_distance
        self.timeout = timeout
        self.max_boxes = max_boxes
        self.tracks = []

    def update(self, detections, now):
        """
        detections: list of dicts with x, y, yaw, hl, hw (half length / width).
                    Returns True if some detection was dropped because
                    max_boxes was reached.
        """
        dist = self.association_distance
        overflow = False

        kept = []
        for d in detections:
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