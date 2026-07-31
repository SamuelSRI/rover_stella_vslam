#!/usr/bin/env python3

import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / norm


def quaternion_to_rotation(q: np.ndarray) -> np.ndarray:
    x, y, z, w = normalize_quaternion(q)
    return np.array([
        [1.0 - 2.0 * (y*y + z*z), 2.0 * (x*y - z*w),       2.0 * (x*z + y*w)],
        [2.0 * (x*y + z*w),       1.0 - 2.0 * (x*x + z*z), 2.0 * (y*z - x*w)],
        [2.0 * (x*z - y*w),       2.0 * (y*z + x*w),       1.0 - 2.0 * (x*x + y*y)],
    ], dtype=float)


def rotation_to_quaternion(r: np.ndarray) -> np.ndarray:
    trace = float(np.trace(r))

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = np.array([
            (r[2, 1] - r[1, 2]) / s,
            (r[0, 2] - r[2, 0]) / s,
            (r[1, 0] - r[0, 1]) / s,
            0.25 * s,
        ])
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        q = np.array([
            0.25 * s,
            (r[0, 1] + r[1, 0]) / s,
            (r[0, 2] + r[2, 0]) / s,
            (r[2, 1] - r[1, 2]) / s,
        ])
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        q = np.array([
            (r[0, 1] + r[1, 0]) / s,
            0.25 * s,
            (r[1, 2] + r[2, 1]) / s,
            (r[0, 2] - r[2, 0]) / s,
        ])
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        q = np.array([
            (r[0, 2] + r[2, 0]) / s,
            (r[1, 2] + r[2, 1]) / s,
            0.25 * s,
            (r[1, 0] - r[0, 1]) / s,
        ])

    return normalize_quaternion(q)


def pose_to_matrix(position, orientation) -> np.ndarray:
    t = np.eye(4, dtype=float)
    t[:3, :3] = quaternion_to_rotation(np.array([
        orientation.x, orientation.y, orientation.z, orientation.w
    ], dtype=float))
    t[:3, 3] = [position.x, position.y, position.z]
    return t


def transform_to_matrix(transform) -> np.ndarray:
    return pose_to_matrix(transform.translation, transform.rotation)


class StellaPoseAdapter(Node):
    """
    Converts Stella-VSLAM's map->camera pose into map->base_link.

    Input:
      nav_msgs/Odometry on /stella_vslam/camera_pose

    Output:
      geometry_msgs/PoseWithCovarianceStamped on /visual_slam/pose

    Required static TF:
      base_link -> camera optical frame
    """

    def __init__(self) -> None:
        super().__init__("stella_pose_adapter")

        self.declare_parameter("input_topic", "/stella_vslam/camera_pose")
        self.declare_parameter("output_topic", "/visual_slam/pose")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame_override", "")
        self.declare_parameter("tf_timeout_sec", 0.1)

        self.declare_parameter("position_variance", 0.25)
        self.declare_parameter("orientation_variance", 0.10)
        self.declare_parameter("force_z_zero", False)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.camera_frame_override = str(
            self.get_parameter("camera_frame_override").value
        )
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self.position_variance = float(
            self.get_parameter("position_variance").value
        )
        self.orientation_variance = float(
            self.get_parameter("orientation_variance").value
        )
        self.force_z_zero = bool(self.get_parameter("force_z_zero").value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, output_topic, 10
        )
        self.subscription = self.create_subscription(
            Odometry,
            input_topic,
            self.pose_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Converting {input_topic} from camera pose to {output_topic} "
            f"for {self.base_frame}"
        )

    def pose_callback(self, msg: Odometry) -> None:
        camera_frame = self.camera_frame_override or msg.child_frame_id
        if not camera_frame:
            self.get_logger().error(
                "Stella camera_pose has an empty child_frame_id and "
                "camera_frame_override is empty"
            )
            return

        # Stella gives T_map_camera.
        map_camera = pose_to_matrix(
            msg.pose.pose.position,
            msg.pose.pose.orientation,
        )

        try:
            # lookup_transform(target=camera, source=base) gives T_camera_base.
            camera_base_tf = self.tf_buffer.lookup_transform(
                camera_frame,
                self.base_frame,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"Cannot transform {camera_frame} <- {self.base_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        camera_base = transform_to_matrix(camera_base_tf.transform)
        map_base = map_camera @ camera_base

        if self.force_z_zero:
            map_base[2, 3] = 0.0

        q = rotation_to_quaternion(map_base[:3, :3])

        output = PoseWithCovarianceStamped()
        output.header = msg.header
        output.pose.pose.position.x = float(map_base[0, 3])
        output.pose.pose.position.y = float(map_base[1, 3])
        output.pose.pose.position.z = float(map_base[2, 3])
        output.pose.pose.orientation.x = float(q[0])
        output.pose.pose.orientation.y = float(q[1])
        output.pose.pose.orientation.z = float(q[2])
        output.pose.pose.orientation.w = float(q[3])

        covariance = [0.0] * 36
        covariance[0] = self.position_variance
        covariance[7] = self.position_variance
        covariance[14] = self.position_variance
        covariance[21] = self.orientation_variance
        covariance[28] = self.orientation_variance
        covariance[35] = self.orientation_variance
        output.pose.covariance = covariance

        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StellaPoseAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
