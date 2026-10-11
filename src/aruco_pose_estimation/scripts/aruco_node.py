#!/usr/bin/env python3
"""
ROS2 wrapper code taken from:
https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco/tree/main

This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /aruco_markers (aruco_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding
       marker ids.

    /aruco_image (sensor_msgs.msg.Image)
       Annotated image with marker locations and ids, with markers drawn on it

Services:
    ~/calibrate (std_srvs.srv.Trigger)
       Re-calibrates the camera pose from the reference tags (see below).

Camera calibration:
    The node estimates the pose of the camera in the map frame from the 4 reference
    tags fixed on the playing area (positions given by the reference_tag_* parameters)
    and publishes it on /tf as `map_frame -> camera_link_frame` at tf_publish_rate Hz.
    Nothing is published on that transform until a calibration has succeeded.
    A calibration runs at startup (calibrate_on_start) and every time ~/calibrate
    is called. A failed calibration never overwrites the last good one.

Parameters:
    marker_size - size of the markers in meters (default .065)
    aruco_dictionary_id - dictionary that was used to generate markers (default DICT_5X5_250)
    image_topic - image topic to subscribe to (default /camera/color/image_raw)
    camera_info_topic - camera info topic to subscribe to (default /camera/camera_info)
    camera_frame - camera optical frame to use (default "camera_depth_optical_frame")
    detected_markers_topic - topic to publish detected markers (default /aruco_markers)
    markers_visualization_topic - topic to publish markers visualization (default /aruco_poses)
    output_image_topic - topic to publish annotated image (default /aruco_image)
    map_frame - parent frame of the published camera transform (default "map")
    camera_link_frame - child frame of the published camera transform (default "camera_link")
    tf_publish_rate - rate [Hz] at which the calibrated transform is published (default 20)
    calibrate_on_start - calibrate automatically at startup, retrying until it works (default True)
    calibration_frames - number of detections of every reference tag to collect (default 30)
    calibration_timeout - give up a calibration attempt after this many seconds (default 15)
    max_reprojection_error_px - reject a calibration with a larger RMS error (default 2.0)
    reference_tag_ids / reference_tag_x / reference_tag_y - tag ids and centre positions
        in the map frame [m]

Author: Simone Giampà
Version: 2024-01-29

"""

# ROS2 imports
import rclpy
import rclpy.node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import rclpy.time
import tf2_ros
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import message_filters

# Python imports
import threading
import time
import numpy as np
import cv2

# Local imports for custom defined functions
from aruco_pose_estimation.utils import ARUCO_DICT
from aruco_pose_estimation.pose_estimation import pose_estimation
from aruco_pose_estimation.calibration import (
    CalibrationError, tag_center_normalized, solve_camera_pose,
    transform_to_matrix, matrix_to_quat, matrix_to_rpy,
)

# ROS2 message imports
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped, TransformStamped
from std_srvs.srv import Trigger
from aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class _CalibrationRequest:
    """One calibration attempt. `result` is (success, message) once `event` is set."""

    def __init__(self, num_frames, timeout, tag_ids):
        self.num_frames = num_frames
        self.deadline = time.monotonic() + timeout
        self.samples = {i: [] for i in tag_ids}
        self.event = threading.Event()
        self.result = None
        self._lock = threading.Lock()

    def finish(self, success, message):
        """Set the result once; later calls are ignored. Returns True if this call set it."""
        with self._lock:
            if self.event.is_set():
                return False
            self.result = (success, message)
            self.event.set()
            return True


class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")

        self.initialize_parameters()

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(self.dictionary_id_name)
            # check if the dictionary_id is a valid dictionary inside ARUCO_DICT values
            if dictionary_id not in ARUCO_DICT.values():
                raise AttributeError
        except AttributeError:
            self.get_logger().error(
                "bad aruco_dictionary_id: {}".format(self.dictionary_id_name)
            )
            options = "\n".join([s for s in ARUCO_DICT])
            self.get_logger().error("valid options: {}".format(options))

        # Set up subscriptions to the camera info and camera image topics

        # camera info topic for the camera calibration parameters
        self.info_sub = self.create_subscription(
            CameraInfo, self.info_topic, self.info_callback, qos_profile_sensor_data
        )

        # select the type of input to use for the pose estimation
        if (bool(self.use_depth_input)):
            # use both rgb and depth image topics for the pose estimation

            # create a message filter to synchronize the image and depth image topics
            self.image_sub = message_filters.Subscriber(self, Image, self.image_topic,
                                                        qos_profile=qos_profile_sensor_data)
            self.depth_image_sub = message_filters.Subscriber(self, Image, self.depth_image_topic,
                                                              qos_profile=qos_profile_sensor_data)

            # create synchronizer between the 2 topics using message filters and approximate time policy
            # slop is the maximum time difference between messages that are considered synchronized
            self.synchronizer = message_filters.ApproximateTimeSynchronizer(
                [self.image_sub, self.depth_image_sub], queue_size=10, slop=0.05
            )
            self.synchronizer.registerCallback(self.rgb_depth_sync_callback)
        else:
            # rely only on the rgb image topic for the pose estimation

            # create a subscription to the image topic
            self.image_sub = self.create_subscription(
                Image, self.image_topic, self.image_callback, qos_profile_sensor_data
            )

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, self.markers_visualization_topic, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/pose', 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, self.detected_markers_topic, 10)
        self.image_pub = self.create_publisher(Image, self.output_image_topic, 10)

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        # code for updated version of cv2 (4.7.0)
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)

        # old code version
        # self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        # self.aruco_parameters = cv2.aruco.DetectorParameters_create()

        self.bridge = CvBridge()

        self.initialize_calibration()

    def initialize_calibration(self):
        self._cal_lock = threading.Lock()
        self._cal_request = None
        self._pose_lock = threading.Lock()
        self._T_map_link = None
        self.calibrated = False

        # TF: the listener is used to read the fixed camera_link -> optical frame
        # transform from the URDF, the broadcaster to publish map -> camera_link
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # separate callback groups so the (blocking) service and the TF timer
        # are not stalled by image processing
        self.create_service(Trigger, "~/calibrate", self.calibrate_callback,
                            callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(1.0 / max(self.tf_publish_rate, 0.1), self.publish_tf,
                          callback_group=MutuallyExclusiveCallbackGroup())
        # handles calibration timeouts and startup retries
        self.create_timer(0.5, self.calibration_watchdog)

        if self.calibrate_on_start:
            self.get_logger().info("Calibration will run at startup.")
        self.get_logger().info(
            "Calibration service available at {}".format(self.get_fully_qualified_name() + "/calibrate"))

    def _start_calibration(self) -> _CalibrationRequest:
        """Start a calibration attempt, or join the one already running."""
        with self._cal_lock:
            req = self._cal_request
            if req is not None and not req.event.is_set():
                return req
            req = _CalibrationRequest(self.calibration_frames, self.calibration_timeout, self.ref_ids)
            self._cal_request = req
        self.get_logger().info("Calibration started: collecting {} detections of tags {}".format(
            self.calibration_frames, self.ref_ids))
        return req

    def _fail_calibration(self, req, message):
        if req.finish(False, message):
            self.get_logger().warn("Calibration failed: {}".format(message))

    def _optical_frame(self):
        if self.camera_frame != "":
            return self.camera_frame
        return self.info_msg.header.frame_id

    def calibrate_callback(self, request, response):
        req = self._start_calibration()
        # the watchdog enforces the timeout; the extra margin is only a backstop
        if not req.event.wait(timeout=self.calibration_timeout + 5.0):
            self._fail_calibration(req, "calibration did not finish in time")
        response.success, response.message = req.result
        return response

    def calibration_watchdog(self):
        with self._cal_lock:
            req = self._cal_request
        active = req is not None and not req.event.is_set()

        if active and time.monotonic() > req.deadline:
            if self.info_msg is None:
                msg = "no camera info received on {}".format(self.info_topic)
            else:
                counts = {i: len(v) for i, v in req.samples.items()}
                missing = [i for i, n in counts.items() if n < req.num_frames]
                msg = "timed out; detections per reference tag: {} (need {}), missing/occluded: {}".format(
                    counts, req.num_frames, missing)
            self._fail_calibration(req, msg)
            active = False

        if not active and self.calibrate_on_start and not self.calibrated:
            self._start_calibration()

    def _calibration_active(self):
        req = self._cal_request
        return req is not None and not req.event.is_set()

    def _calibration_step(self, cv_image):
        """Collect reference tag detections from one frame; solve once enough were collected."""
        req = self._cal_request
        if req is None or req.event.is_set() or self.intrinsic_mat is None:
            return

        corners, marker_ids, _ = self.aruco_detector.detectMarkers(image=cv_image)
        if marker_ids is not None:
            for corner, marker_id in zip(corners, marker_ids.flatten()):
                marker_id = int(marker_id)
                if marker_id not in req.samples:
                    continue
                centre = tag_center_normalized(corner, self.intrinsic_mat, self.distortion)
                if centre is not None:
                    req.samples[marker_id].append(centre)

        if all(len(v) >= req.num_frames for v in req.samples.values()):
            self._finish_calibration(req)

    def _finish_calibration(self, req):
        K = self.intrinsic_mat
        focal = 0.5 * (K[0, 0] + K[1, 1])

        # the camera is static, so the median over many frames rejects noise/outliers
        centres = np.array([np.median(req.samples[i], axis=0) for i in self.ref_ids])
        spread_px = {i: float(np.linalg.norm(np.std(req.samples[i], axis=0)) * focal) for i in self.ref_ids}

        try:
            T_map_optical, rms_norm, info = solve_camera_pose(self.ref_xy, centres)
        except CalibrationError as e:
            self._fail_calibration(req, str(e))
            return

        rms_px = rms_norm * focal
        if rms_px > self.max_reprojection_error_px:
            self._fail_calibration(
                req, "reprojection error {:.2f} px exceeds the {:.2f} px limit; check reference_tag_* "
                     "parameters and tag ids (detection jitter per tag [px]: {})".format(
                         rms_px, self.max_reprojection_error_px,
                         {i: round(v, 2) for i, v in spread_px.items()}))
            return

        # T_map_optical = T_map_link * T_link_optical, with T_link_optical being the fixed
        # transform from the URDF
        try:
            tf = self.tf_buffer.lookup_transform(self.camera_link_frame, self._optical_frame(),
                                                 rclpy.time.Time())
        except Exception as e:  # tf2 raises several exception types
            self._fail_calibration(req, "cannot look up {} -> {}: {}".format(
                self.camera_link_frame, self._optical_frame(), e))
            return
        t, q = tf.transform.translation, tf.transform.rotation
        T_link_optical = transform_to_matrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
        T_map_link = T_map_optical @ np.linalg.inv(T_link_optical)

        with self._pose_lock:
            self._T_map_link = T_map_link
        self.calibrated = True

        roll, pitch, yaw = matrix_to_rpy(T_map_link[:3, :3])
        message = ("camera_link in {}: xyz = [{:.4f}, {:.4f}, {:.4f}] m, rpy = [{:.4f}, {:.4f}, {:.4f}] rad, "
                   "reprojection RMS {:.2f} px".format(
                       self.map_frame, *T_map_link[:3, 3], roll, pitch, yaw, rms_px))
        if req.finish(True, message):
            self.get_logger().info("Calibration succeeded: " + message)

    def publish_tf(self):
        with self._pose_lock:
            T = self._T_map_link
        if T is None:
            return
        qx, qy, qz, qw = matrix_to_quat(T[:3, :3])
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.camera_link_frame
        msg.transform.translation.x = float(T[0, 3])
        msg.transform.translation.y = float(T[1, 3])
        msg.transform.translation.z = float(T[2, 3])
        msg.transform.rotation.x = float(qx)
        msg.transform.rotation.y = float(qy)
        msg.transform.rotation.z = float(qz)
        msg.transform.rotation.w = float(qw)
        self.tf_broadcaster.sendTransform(msg)

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        # get the intrinsic matrix and distortion coefficients from the camera info
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)

        self.get_logger().info("Camera info received.")
        self.get_logger().info("Intrinsic matrix: {}".format(self.intrinsic_mat))
        self.get_logger().info("Distortion coefficients: {}".format(self.distortion))
        self.get_logger().info("Camera frame: {}x{}".format(self.info_msg.width, self.info_msg.height))

        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg: Image):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        # convert the image messages to cv2 format
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8")

        if self._calibration_active():
            self._calibration_step(cv_image)

        # create the ArucoMarkers and PoseArray messages
        markers = ArucoMarkers()
        pose_array = PoseArray()

        # Set the frame id and timestamp for the markers and pose array
        if self.camera_frame == "":
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame

        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        """
        # OVERRIDE: use calibrated intrinsic matrix and distortion coefficients
        self.intrinsic_mat = np.reshape([615.95431, 0., 325.26983,
                                         0., 617.92586, 257.57722,
                                         0., 0., 1.], (3, 3))
        self.distortion = np.array([0.142588, -0.311967, 0.003950, -0.006346, 0.000000])
        """
        
        # call the pose estimation function
        frame, pose_array, markers = pose_estimation(rgb_frame=cv_image, depth_frame=None,
                                                     aruco_detector=self.aruco_detector,
                                                     marker_size=self.marker_size, matrix_coefficients=self.intrinsic_mat,
                                                     distortion_coefficients=self.distortion, pose_array=pose_array, markers=markers)
        if len(markers.marker_ids) > 0:
            if 1 in markers.marker_ids:
                idx = markers.marker_ids.index(1)

                pose_msg = PoseWithCovarianceStamped()
                pose_msg.header = pose_array.header
                pose_msg.pose.pose = pose_array.poses[idx]
                pose_msg.pose.covariance = [
                    0.25, 0, 0, 0, 0, 0,
                    0, 0.25, 0, 0, 0, 0,
                    0, 0, 999.0, 0, 0, 0,
                    0, 0, 0, 999.0, 0, 0,
                    0, 0, 0, 0, 999.0, 0,
                    0, 0, 0, 0, 0, 0.1,
                ]
                self.pose_pub.publish(pose_msg)

            # Publish the results with the poses and markers positions
            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)

        # publish the image frame with computed markers positions over the image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))

    def depth_image_callback(self, depth_msg: Image):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

    def rgb_depth_sync_callback(self, rgb_msg: Image, depth_msg: Image):

        # convert the image messages to cv2 format
        cv_depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
        cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")

        if self._calibration_active():
            self._calibration_step(cv_image)

        # create the ArucoMarkers and PoseArray messages
        markers = ArucoMarkers()
        pose_array = PoseArray()

        # Set the frame id and timestamp for the markers and pose array
        if self.camera_frame == "":
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame

        markers.header.stamp = rgb_msg.header.stamp
        pose_array.header.stamp = rgb_msg.header.stamp

        # call the pose estimation function
        frame, pose_array, markers = pose_estimation(rgb_frame=cv_image, depth_frame=cv_depth_image,
                                                     aruco_detector=self.aruco_detector,
                                                     marker_size=self.marker_size, matrix_coefficients=self.intrinsic_mat,
                                                     distortion_coefficients=self.distortion, pose_array=pose_array, markers=markers)

        # if some markers are detected
        if len(markers.marker_ids) > 0:
            if 1 in markers.marker_ids:
                idx = markers.marker_ids.index(1)

                pose_msg = PoseWithCovarianceStamped()
                pose_msg.header = pose_array.header
                pose_msg.pose.pose = pose_array.poses[idx]
                pose_msg.pose.covariance = [
                    0.25, 0, 0, 0, 0, 0,
                    0, 0.25, 0, 0, 0, 0,
                    0, 0, 999.0, 0, 0, 0,
                    0, 0, 0, 999.0, 0, 0,
                    0, 0, 0, 0, 999.0, 0,
                    0, 0, 0, 0, 0, 0.1,
                ]
                self.pose_pub.publish(pose_msg)

            # Publish the results with the poses and markers positions
            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)

        # publish the image frame with computed markers positions over the image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))

    def initialize_parameters(self):
        # Declare and read parameters from aruco_params.yaml
        self.declare_parameter(
            name="marker_size",
            value=0.0625,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters.",
            ),
        )

        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_5X5_250",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary that was used to generate markers.",
            ),
        )

        self.declare_parameter(
            name="use_depth_input",
            value=True,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Use depth camera input for pose estimation instead of RGB image",
            ),
        )

        self.declare_parameter(
            name="image_topic",
            value="/camera/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="depth_image_topic",
            value="/camera/depth/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Depth camera topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            value="/camera/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_frame",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame to use.",
            ),
        )

        self.declare_parameter(
            name="detected_markers_topic",
            value="/aruco_markers",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish detected markers as array of marker ids and poses",
            ),
        )

        self.declare_parameter(
            name="markers_visualization_topic",
            value="/aruco_poses",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish markers as pose array",
            ),
        )

        self.declare_parameter(
            name="output_image_topic",
            value="/aruco_image",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish annotated images with markers drawn on them",
            ),
        )

        self.declare_parameter(
            name="map_frame", value="map",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Parent frame of the published camera transform."))
        self.declare_parameter(
            name="camera_link_frame", value="camera_link",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Child frame of the published camera transform (the camera body link)."))
        self.declare_parameter(
            name="tf_publish_rate", value=20.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Rate [Hz] at which map -> camera_link is published."))
        self.declare_parameter(
            name="calibrate_on_start", value=True,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Calibrate automatically at startup (retries until it succeeds)."))
        self.declare_parameter(
            name="calibration_frames", value=30,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Detections of every reference tag to collect per calibration."))
        self.declare_parameter(
            name="calibration_timeout", value=15.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Seconds before a calibration attempt is abandoned."))
        self.declare_parameter(
            name="max_reprojection_error_px", value=2.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Calibrations with a larger RMS reprojection error are rejected."))
        self.declare_parameter(
            name="reference_tag_ids", value=[20, 21, 22, 23],
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER_ARRAY,
                description="Ids of the reference tags fixed on the playing area."))
        self.declare_parameter(
            name="reference_tag_x", value=[2.4, 0.6, 2.4, 0.6],
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                description="x of the reference tag centres in the map frame [m] (same order as ids)."))
        self.declare_parameter(
            name="reference_tag_y", value=[0.6, 0.6, 1.4, 1.4],
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                description="y of the reference tag centres in the map frame [m] (same order as ids)."))

        # read parameters from aruco_params.yaml and store them
        self.marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )
        self.get_logger().info(f"Marker size: {self.marker_size}")

        self.dictionary_id_name = (
            self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        )
        self.get_logger().info(f"Marker type: {self.dictionary_id_name}")

        self.use_depth_input = (
            self.get_parameter("use_depth_input").get_parameter_value().bool_value
        )
        self.get_logger().info(f"Use depth input: {self.use_depth_input}")

        self.image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Input image topic: {self.image_topic}")

        self.depth_image_topic = (
            self.get_parameter("depth_image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Input depth image topic: {self.depth_image_topic}")

        self.info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image camera info topic: {self.info_topic}")

        self.camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )
        self.get_logger().info(f"Camera frame: {self.camera_frame}")

        # Output topics
        self.detected_markers_topic = (
            self.get_parameter("detected_markers_topic").get_parameter_value().string_value
        )

        self.markers_visualization_topic = (
            self.get_parameter("markers_visualization_topic").get_parameter_value().string_value
        )

        self.output_image_topic = (
            self.get_parameter("output_image_topic").get_parameter_value().string_value
        )

        # Calibration parameters
        self.map_frame = self.get_parameter("map_frame").value
        self.camera_link_frame = self.get_parameter("camera_link_frame").value
        self.tf_publish_rate = float(self.get_parameter("tf_publish_rate").value)
        self.calibrate_on_start = bool(self.get_parameter("calibrate_on_start").value)
        self.calibration_frames = int(self.get_parameter("calibration_frames").value)
        self.calibration_timeout = float(self.get_parameter("calibration_timeout").value)
        self.max_reprojection_error_px = float(self.get_parameter("max_reprojection_error_px").value)

        self.ref_ids = [int(i) for i in self.get_parameter("reference_tag_ids").value]
        ref_x = list(self.get_parameter("reference_tag_x").value)
        ref_y = list(self.get_parameter("reference_tag_y").value)
        if not (len(self.ref_ids) == len(ref_x) == len(ref_y)) or len(self.ref_ids) < 4:
            raise ValueError("reference_tag_ids/x/y must have the same length and contain at least 4 tags")
        if len(set(self.ref_ids)) != len(self.ref_ids):
            raise ValueError("reference_tag_ids contains duplicates")
        self.ref_xy = np.column_stack((ref_x, ref_y)).astype(np.float64)
        self.get_logger().info("Reference tags (id: x, y in {}): {}".format(
            self.map_frame, {i: tuple(p) for i, p in zip(self.ref_ids, self.ref_xy)}))


def main():
    rclpy.init()
    node = ArucoNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()