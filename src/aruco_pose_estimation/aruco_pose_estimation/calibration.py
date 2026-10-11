#!/usr/bin/env python3
"""
Math helpers used by aruco_node to calibrate the camera pose (map -> camera)
from reference ArUco tags whose positions in the map frame are known.

Deliberately free of ROS imports so it can be unit tested on its own.

Conventions
-----------
* T_a_b is a 4x4 homogeneous matrix that maps points expressed in frame b into
  frame a (so it is "the pose of b expressed in a").
* Quaternions are (x, y, z, w), like geometry_msgs.
* The reference tags are assumed to lie in the map plane (z = 0).

Only the *centres* of the tags are used, not their orientation. This means:
  - the result does not depend on how the tags are rotated on the table,
  - the result does not depend on `marker_size` (the scale comes from the known
    spacing between the tags).
"""

import cv2
import numpy as np


class CalibrationError(Exception):
    pass


def tag_center_normalized(corners, camera_matrix, distortion):
    """
    Projection of a tag's centre as undistorted *normalized* image coordinates
    (x/z, y/z). The centre is the intersection of the two diagonals of the
    detected quad, which is exact under a pinhole projection (lines stay lines,
    and the diagonals of a square meet at its centre).

    corners: detectMarkers output for one marker, shape (1, 4, 2) or (4, 2)
    Returns a (2,) array, or None if the quad is degenerate.
    """
    pts = np.asarray(corners, dtype=np.float64).reshape(-1, 1, 2)
    dist = distortion if (distortion is not None and np.size(distortion) > 0) else None
    und = cv2.undistortPoints(pts, np.asarray(camera_matrix, dtype=np.float64), dist)
    p0, p1, p2, p3 = und.reshape(4, 2)

    # p0 + s*(p2-p0) = p1 + u*(p3-p1)
    a = np.column_stack((p2 - p0, -(p3 - p1)))
    if abs(np.linalg.det(a)) < 1e-12:
        return None
    s, _ = np.linalg.solve(a, p1 - p0)
    return p0 + s * (p2 - p0)


def solve_camera_pose(tag_xy, centers_norm):
    """
    Solve the pose of the camera optical frame in the map frame.

    tag_xy:       (N, 2) tag centre positions in the map frame [m], N >= 4
    centers_norm: (N, 2) matching tag centres, normalized image coordinates

    Returns (T_map_optical, rms_norm, info)
      rms_norm is the RMS reprojection error in normalized units
      (multiply by the focal length to get pixels).
    """
    tag_xy = np.asarray(tag_xy, dtype=np.float64).reshape(-1, 2)
    img = np.asarray(centers_norm, dtype=np.float64).reshape(-1, 1, 2)
    if len(tag_xy) < 4 or len(tag_xy) != len(img):
        raise CalibrationError("need at least 4 matching reference tags")

    obj = np.column_stack((tag_xy, np.zeros(len(tag_xy))))
    eye = np.eye(3)

    n, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, img, eye, None, flags=cv2.SOLVEPNP_IPPE)
    if n == 0:
        raise CalibrationError("solvePnP failed")

    candidates = []
    for rvec, tvec in zip(rvecs, tvecs):
        rvec, tvec = cv2.solvePnPRefineLM(obj, img, eye, None, rvec.copy(), tvec.copy())
        proj, _ = cv2.projectPoints(obj, rvec, tvec, eye, None)
        rms = float(np.sqrt(np.mean(np.sum((proj - img) ** 2, axis=2))))

        # PnP gives T_optical_map; we want the camera pose in the map
        t_optical_map = np.eye(4)
        t_optical_map[:3, :3] = cv2.Rodrigues(rvec)[0]
        t_optical_map[:3, 3] = tvec.ravel()
        t_map_optical = np.linalg.inv(t_optical_map)
        candidates.append((rms, t_map_optical))

    # The camera must be above the table (map z > 0). This also rejects the
    # mirrored solution you would get if the tag layout had the wrong handedness.
    valid = [c for c in candidates if c[1][2, 3] > 0.0]
    if not valid:
        raise CalibrationError(
            "camera ends up below the map plane; the reference tag layout "
            "(ids/positions) probably has the wrong handedness or wrong ids")

    valid.sort(key=lambda c: c[0])
    rms, t_map_optical = valid[0]
    info = {"num_solutions": len(candidates),
            "rms_all": [c[0] for c in candidates]}
    return t_map_optical, rms, info


# --- small rotation helpers (avoid pulling in tf_transformations / scipy) ---

def quat_to_matrix(q):
    x, y, z, w = np.asarray(q, dtype=np.float64) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(r):
    r = np.asarray(r, dtype=np.float64)
    tr = np.trace(r[:3, :3])
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (r[2, 1] - r[1, 2]) * s
        y = (r[0, 2] - r[2, 0]) * s
        z = (r[1, 0] - r[0, 1]) * s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def transform_to_matrix(translation, quaternion):
    """geometry_msgs-style translation (x,y,z) + quaternion (x,y,z,w) -> 4x4."""
    t = np.eye(4)
    t[:3, :3] = quat_to_matrix(quaternion)
    t[:3, 3] = translation
    return t


def matrix_to_rpy(r):
    """Fixed-axis roll/pitch/yaw (ROS convention), for logging only."""
    r = np.asarray(r)
    pitch = -np.arcsin(np.clip(r[2, 0], -1.0, 1.0))
    roll = np.arctan2(r[2, 1], r[2, 2])
    yaw = np.arctan2(r[1, 0], r[0, 0])
    return roll, pitch, yaw
