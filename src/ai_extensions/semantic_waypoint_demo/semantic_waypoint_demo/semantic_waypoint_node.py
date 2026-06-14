from __future__ import annotations

import json
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, Joy
from std_msgs.msg import Float32, String

from semantic_waypoint_demo.detectors import Detection, create_detector


class SemanticWaypointNode(Node):
    """Convert a semantic image detection into the repo's waypoint interface."""

    def __init__(self) -> None:
        super().__init__("semantic_waypoint_node")

        self._declare_parameters()
        self._read_parameters()

        self._latest_odom: Optional[Odometry] = None
        self._latest_image: Optional[object] = None
        self._latest_image_width = self._configured_image_width
        self._latest_image_height = self._configured_image_height
        self._stable_detection_count = 0
        self._last_waypoint_time = self.get_clock().now()
        self._joy_sent = False

        self._detector = create_detector(
            backend=self._detector_backend,
            target_label=self._target_label,
            mock_confidence=self._mock_confidence,
            mock_center_x=self._mock_center_x,
            mock_center_y=self._mock_center_y,
            mock_width=self._mock_width,
            mock_height=self._mock_height,
        )

        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 5)
        self._create_image_subscription()

        self._waypoint_pub = self.create_publisher(PointStamped, self._waypoint_topic, 5)
        self._joy_pub = self.create_publisher(Joy, self._joy_topic, 5)
        self._speed_pub = self.create_publisher(Float32, self._speed_topic, 5)
        self._boundary_pub = self.create_publisher(PolygonStamped, self._boundary_topic, 5)
        self._target_pub = self.create_publisher(PointStamped, self._debug_target_topic, 5)
        self._detection_pub = self.create_publisher(String, self._debug_detection_topic, 5)

        self.create_timer(1.0 / self._publish_rate_hz, self._on_timer)
        self.get_logger().info(
            "semantic_waypoint_demo started: "
            f"backend={self._detector_backend}, target={self._target_label}, "
            f"image_transport={self._image_transport}, require_image={self._require_image}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("odom_topic", "/state_estimation")
        self.declare_parameter("image_topic", "/camera/image/transmitted")
        self.declare_parameter("image_transport", "none")
        self.declare_parameter("require_image", False)
        self.declare_parameter("configured_image_width", 640)
        self.declare_parameter("configured_image_height", 480)

        self.declare_parameter("waypoint_topic", "/way_point")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("speed_topic", "/speed")
        self.declare_parameter("boundary_topic", "/navigation_boundary")
        self.declare_parameter("debug_target_topic", "/ai_extension/semantic_waypoint/target")
        self.declare_parameter("debug_detection_topic", "/ai_extension/semantic_waypoint/detection")

        self.declare_parameter("detector_backend", "mock")
        self.declare_parameter("target_label", "chair")
        self.declare_parameter("mock_confidence", 0.9)
        self.declare_parameter("mock_center_x", 0.5)
        self.declare_parameter("mock_center_y", 0.5)
        self.declare_parameter("mock_width", 0.25)
        self.declare_parameter("mock_height", 0.35)
        self.declare_parameter("min_confidence", 0.5)
        self.declare_parameter("stable_detection_frames", 2)

        self.declare_parameter("target_distance", 1.5)
        self.declare_parameter("horizontal_fov_deg", 70.0)
        self.declare_parameter("max_heading_offset_deg", 45.0)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("waypoint_update_rate_hz", 2.0)
        self.declare_parameter("reached_radius", 0.5)

        self.declare_parameter("publish_joy", True)
        self.declare_parameter("publish_speed", True)
        self.declare_parameter("speed", 0.25)
        self.declare_parameter("joy_forward_axis", 1.0)

        self.declare_parameter("publish_boundary", False)
        self.declare_parameter("boundary_half_size", 5.0)
        self.declare_parameter("boundary_z", 0.0)

    def _read_parameters(self) -> None:
        self._odom_topic = self.get_parameter("odom_topic").value
        self._image_topic = self.get_parameter("image_topic").value
        self._image_transport = self.get_parameter("image_transport").value
        self._require_image = bool(self.get_parameter("require_image").value)
        self._configured_image_width = int(self.get_parameter("configured_image_width").value)
        self._configured_image_height = int(self.get_parameter("configured_image_height").value)

        self._waypoint_topic = self.get_parameter("waypoint_topic").value
        self._joy_topic = self.get_parameter("joy_topic").value
        self._speed_topic = self.get_parameter("speed_topic").value
        self._boundary_topic = self.get_parameter("boundary_topic").value
        self._debug_target_topic = self.get_parameter("debug_target_topic").value
        self._debug_detection_topic = self.get_parameter("debug_detection_topic").value

        self._detector_backend = self.get_parameter("detector_backend").value
        self._target_label = self.get_parameter("target_label").value
        self._mock_confidence = float(self.get_parameter("mock_confidence").value)
        self._mock_center_x = float(self.get_parameter("mock_center_x").value)
        self._mock_center_y = float(self.get_parameter("mock_center_y").value)
        self._mock_width = float(self.get_parameter("mock_width").value)
        self._mock_height = float(self.get_parameter("mock_height").value)
        self._min_confidence = float(self.get_parameter("min_confidence").value)
        self._stable_detection_frames = int(self.get_parameter("stable_detection_frames").value)

        self._target_distance = float(self.get_parameter("target_distance").value)
        self._horizontal_fov = math.radians(float(self.get_parameter("horizontal_fov_deg").value))
        self._max_heading_offset = math.radians(float(self.get_parameter("max_heading_offset_deg").value))
        self._publish_rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self._waypoint_update_period = 1.0 / max(
            0.1, float(self.get_parameter("waypoint_update_rate_hz").value)
        )
        self._reached_radius = float(self.get_parameter("reached_radius").value)

        self._publish_joy = bool(self.get_parameter("publish_joy").value)
        self._publish_speed = bool(self.get_parameter("publish_speed").value)
        self._speed = float(self.get_parameter("speed").value)
        self._joy_forward_axis = float(self.get_parameter("joy_forward_axis").value)

        self._publish_boundary = bool(self.get_parameter("publish_boundary").value)
        self._boundary_half_size = float(self.get_parameter("boundary_half_size").value)
        self._boundary_z = float(self.get_parameter("boundary_z").value)

    def _create_image_subscription(self) -> None:
        if self._image_transport == "none":
            return
        if self._image_transport == "raw":
            self.create_subscription(Image, self._image_topic, self._on_raw_image, 5)
            return
        if self._image_transport == "compressed":
            self.create_subscription(CompressedImage, self._image_topic, self._on_compressed_image, 5)
            return
        raise ValueError("image_transport must be one of: none, raw, compressed")

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _on_raw_image(self, msg: Image) -> None:
        self._latest_image = msg
        self._latest_image_width = msg.width
        self._latest_image_height = msg.height

    def _on_compressed_image(self, msg: CompressedImage) -> None:
        self._latest_image = msg
        self._latest_image_width = self._configured_image_width
        self._latest_image_height = self._configured_image_height

    def _on_timer(self) -> None:
        if self._latest_odom is None:
            self.get_logger().debug("waiting for odometry")
            return
        if self._require_image and self._latest_image is None:
            self.get_logger().debug("waiting for image")
            return

        detection = self._detect_target()
        self._publish_detection_debug(detection)
        if detection is None:
            self._stable_detection_count = 0
            return

        if detection.confidence < self._min_confidence:
            self._stable_detection_count = 0
            return

        self._stable_detection_count += 1
        if self._stable_detection_count < self._stable_detection_frames:
            return

        now = self.get_clock().now()
        if (now - self._last_waypoint_time).nanoseconds * 1e-9 < self._waypoint_update_period:
            return

        waypoint = self._detection_to_waypoint(detection)
        if waypoint is None:
            return

        self._publish_autonomy_enable()
        self._waypoint_pub.publish(waypoint)
        self._target_pub.publish(waypoint)
        if self._publish_speed:
            self._speed_pub.publish(Float32(data=self._speed))
        if self._publish_boundary:
            self._boundary_pub.publish(self._make_boundary())

        self._last_waypoint_time = now

    def _detect_target(self) -> Optional[Detection]:
        try:
            return self._detector.detect(
                self._latest_image,
                self._latest_image_width,
                self._latest_image_height,
            )
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return None

    def _detection_to_waypoint(self, detection: Detection) -> Optional[PointStamped]:
        assert self._latest_odom is not None

        pose = self._latest_odom.pose.pose
        yaw = _yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

        centered_x = detection.center_x - 0.5
        heading_offset = -centered_x * self._horizontal_fov
        heading_offset = max(-self._max_heading_offset, min(self._max_heading_offset, heading_offset))
        target_yaw = yaw + heading_offset

        waypoint = PointStamped()
        waypoint.header.frame_id = "map"
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.point.x = pose.position.x + self._target_distance * math.cos(target_yaw)
        waypoint.point.y = pose.position.y + self._target_distance * math.sin(target_yaw)
        waypoint.point.z = pose.position.z

        dx = waypoint.point.x - pose.position.x
        dy = waypoint.point.y - pose.position.y
        if math.hypot(dx, dy) < self._reached_radius:
            return None
        return waypoint

    def _publish_autonomy_enable(self) -> None:
        if not self._publish_joy:
            return
        if self._joy_sent:
            return

        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        joy.header.frame_id = "semantic_waypoint_demo"
        joy.axes = [0.0, 0.0, -1.0, 0.0, self._joy_forward_axis, 1.0, 0.0, 0.0]
        joy.buttons = [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
        self._joy_pub.publish(joy)
        self._joy_sent = True

    def _make_boundary(self) -> PolygonStamped:
        assert self._latest_odom is not None

        pose = self._latest_odom.pose.pose.position
        half = self._boundary_half_size
        boundary = PolygonStamped()
        boundary.header.frame_id = "map"
        boundary.header.stamp = self.get_clock().now().to_msg()
        boundary.polygon.points = [
            Point32(x=pose.x - half, y=pose.y - half, z=self._boundary_z),
            Point32(x=pose.x + half, y=pose.y - half, z=self._boundary_z),
            Point32(x=pose.x + half, y=pose.y + half, z=self._boundary_z),
            Point32(x=pose.x - half, y=pose.y + half, z=self._boundary_z),
            Point32(x=pose.x - half, y=pose.y - half, z=self._boundary_z),
        ]
        return boundary

    def _publish_detection_debug(self, detection: Optional[Detection]) -> None:
        payload = {"detected": False, "target_label": self._target_label}
        if detection is not None:
            payload.update(
                {
                    "detected": True,
                    "label": detection.label,
                    "confidence": detection.confidence,
                    "center_x": detection.center_x,
                    "center_y": detection.center_y,
                    "width": detection.width,
                    "height": detection.height,
                    "stable_frames": self._stable_detection_count,
                }
            )
        self._detection_pub.publish(String(data=json.dumps(payload)))


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = SemanticWaypointNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
