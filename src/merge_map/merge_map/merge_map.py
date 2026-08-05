import math
from typing import List, Optional, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert a quaternion to planar yaw."""
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def transform_xy(
    x: float,
    y: float,
    translation_x: float,
    translation_y: float,
    yaw: float,
) -> Tuple[float, float]:
    """Apply a planar rigid-body transform."""
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    transformed_x = (
        translation_x
        + cos_yaw * x
        - sin_yaw * y
    )

    transformed_y = (
        translation_y
        + sin_yaw * x
        + cos_yaw * y
    )

    return transformed_x, transformed_y


def map_point_to_source_frame(
    map_message: OccupancyGrid,
    local_x: float,
    local_y: float,
) -> Tuple[float, float]:
    """
    Convert a grid-local coordinate into the map message's frame.

    local_x/local_y are measured from the OccupancyGrid origin before
    applying the origin pose orientation.
    """
    origin = map_message.info.origin

    origin_yaw = quaternion_to_yaw(
        origin.orientation.x,
        origin.orientation.y,
        origin.orientation.z,
        origin.orientation.w,
    )

    return transform_xy(
        local_x,
        local_y,
        origin.position.x,
        origin.position.y,
        origin_yaw,
    )


class MergeMapNode(Node):
    def __init__(self) -> None:
        super().__init__("merge_map_node")

        self.frame_id = self.declare_parameter(
            "frame_id",
            "merged_map",
        ).get_parameter_value().string_value

        self.robot_count = self.declare_parameter(
            "robot_count",
            2,
        ).get_parameter_value().integer_value

        self.tf_timeout_seconds = self.declare_parameter(
            "tf_timeout_seconds",
            0.5,
        ).get_parameter_value().double_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        map_qos = QoSProfile(depth=10)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE

        self.publisher = self.create_publisher(
            OccupancyGrid,
            f"/{self.frame_id}",
            map_qos,
        )

        self.maps: List[Optional[OccupancyGrid]] = [
            None
        ] * self.robot_count

        # Keep subscription references alive.
        self.map_subscriptions = []

        for index in range(self.robot_count):
            topic = f"/robot{index + 1}/map"

            subscription = self.create_subscription(
                OccupancyGrid,
                topic,
                lambda message, map_index=index:
                    self.map_callback(message, map_index),
                map_qos,
            )

            self.map_subscriptions.append(subscription)

            self.get_logger().info(
                f"Subscribing to {topic}"
            )

        self.get_logger().info(
            f"TF-aware map merger started: "
            f"target_frame='{self.frame_id}', "
            f"robot_count={self.robot_count}"
        )

    def map_callback(
        self,
        message: OccupancyGrid,
        index: int,
    ) -> None:
        if not message.header.frame_id:
            self.get_logger().warning(
                f"Map from robot{index + 1} has an empty frame_id"
            )
            return

        expected_size = (
            message.info.width * message.info.height
        )

        if len(message.data) != expected_size:
            self.get_logger().error(
                f"Invalid map from robot{index + 1}: "
                f"expected {expected_size} cells but received "
                f"{len(message.data)}"
            )
            return

        self.maps[index] = message

        self.get_logger().debug(
            f"Received map from robot{index + 1}: "
            f"frame={message.header.frame_id}, "
            f"size={message.info.width}x{message.info.height}"
        )

        self.try_merge_and_publish()

    def lookup_map_transform(
        self,
        map_message: OccupancyGrid,
    ):
        source_frame = map_message.header.frame_id

        try:
            return self.tf_buffer.lookup_transform(
                self.frame_id,
                source_frame,
                Time(),
                timeout=Duration(
                    seconds=self.tf_timeout_seconds
                ),
            )
        except TransformException as exception:
            self.get_logger().warning(
                f"Cannot transform map frame "
                f"'{source_frame}' to '{self.frame_id}': "
                f"{exception}"
            )
            return None

    def transform_source_point_to_merged(
        self,
        source_x: float,
        source_y: float,
        transform,
    ) -> Tuple[float, float]:
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        transform_yaw = quaternion_to_yaw(
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )

        return transform_xy(
            source_x,
            source_y,
            translation.x,
            translation.y,
            transform_yaw,
        )

    def calculate_transformed_bounds(
        self,
        map_message: OccupancyGrid,
        transform,
    ) -> List[Tuple[float, float]]:
        width_metres = (
            map_message.info.width
            * map_message.info.resolution
        )

        height_metres = (
            map_message.info.height
            * map_message.info.resolution
        )

        # Outer grid corners, not cell centres.
        local_corners = [
            (0.0, 0.0),
            (width_metres, 0.0),
            (0.0, height_metres),
            (width_metres, height_metres),
        ]

        transformed_corners = []

        for local_x, local_y in local_corners:
            source_x, source_y = map_point_to_source_frame(
                map_message,
                local_x,
                local_y,
            )

            merged_x, merged_y = (
                self.transform_source_point_to_merged(
                    source_x,
                    source_y,
                    transform,
                )
            )

            transformed_corners.append(
                (merged_x, merged_y)
            )

        return transformed_corners

    @staticmethod
    def merge_occupancy_value(
        existing_value: int,
        incoming_value: int,
    ) -> int:
        """
        Merge policy:

        unknown (-1) < free/low occupancy < obstacle/high occupancy.

        If both cells are known, retain the larger occupancy probability.
        """
        if incoming_value < 0:
            return existing_value

        if existing_value < 0:
            return incoming_value

        return max(existing_value, incoming_value)

    def build_merged_map(
        self,
        maps: List[OccupancyGrid],
        transforms: List,
    ) -> Optional[OccupancyGrid]:
        all_corners: List[Tuple[float, float]] = []

        for map_message, transform in zip(
            maps,
            transforms,
        ):
            all_corners.extend(
                self.calculate_transformed_bounds(
                    map_message,
                    transform,
                )
            )

        if not all_corners:
            return None

        min_x = min(point[0] for point in all_corners)
        min_y = min(point[1] for point in all_corners)
        max_x = max(point[0] for point in all_corners)
        max_y = max(point[1] for point in all_corners)

        resolution = min(
            map_message.info.resolution
            for map_message in maps
        )

        if resolution <= 0.0:
            self.get_logger().error(
                "Map resolution must be greater than zero"
            )
            return None

        width = max(
            1,
            int(math.ceil((max_x - min_x) / resolution)),
        )

        height = max(
            1,
            int(math.ceil((max_y - min_y) / resolution)),
        )

        merged_map = OccupancyGrid()

        current_stamp = self.get_clock().now().to_msg()

        merged_map.header.stamp = current_stamp
        merged_map.header.frame_id = self.frame_id

        merged_map.info.map_load_time = current_stamp
        merged_map.info.resolution = resolution
        merged_map.info.width = width
        merged_map.info.height = height

        merged_map.info.origin.position.x = min_x
        merged_map.info.origin.position.y = min_y
        merged_map.info.origin.position.z = 0.0

        merged_map.info.origin.orientation.x = 0.0
        merged_map.info.origin.orientation.y = 0.0
        merged_map.info.origin.orientation.z = 0.0
        merged_map.info.origin.orientation.w = 1.0

        merged_data = [-1] * (width * height)

        for map_message, transform in zip(
            maps,
            transforms,
        ):
            source_resolution = (
                map_message.info.resolution
            )

            for source_y_index in range(
                map_message.info.height
            ):
                for source_x_index in range(
                    map_message.info.width
                ):
                    source_index = (
                        source_x_index
                        + source_y_index
                        * map_message.info.width
                    )

                    incoming_value = int(
                        map_message.data[source_index]
                    )

                    # Unknown cells contribute no information.
                    if incoming_value < 0:
                        continue

                    # Use the centre of each source grid cell.
                    local_x = (
                        source_x_index + 0.5
                    ) * source_resolution

                    local_y = (
                        source_y_index + 0.5
                    ) * source_resolution

                    source_x, source_y = (
                        map_point_to_source_frame(
                            map_message,
                            local_x,
                            local_y,
                        )
                    )

                    merged_x, merged_y = (
                        self.transform_source_point_to_merged(
                            source_x,
                            source_y,
                            transform,
                        )
                    )

                    merged_x_index = int(
                        math.floor(
                            (merged_x - min_x)
                            / resolution
                        )
                    )

                    merged_y_index = int(
                        math.floor(
                            (merged_y - min_y)
                            / resolution
                        )
                    )

                    if not (
                        0 <= merged_x_index < width
                        and 0 <= merged_y_index < height
                    ):
                        continue

                    merged_index = (
                        merged_x_index
                        + merged_y_index * width
                    )

                    merged_data[merged_index] = (
                        self.merge_occupancy_value(
                            merged_data[merged_index],
                            incoming_value,
                        )
                    )

        merged_map.data = merged_data
        return merged_map

    def try_merge_and_publish(self) -> None:
        if not all(
            map_message is not None
            for map_message in self.maps
        ):
            return

        maps = [
            map_message
            for map_message in self.maps
            if map_message is not None
        ]

        transforms = []

        for map_message in maps:
            transform = self.lookup_map_transform(
                map_message
            )

            if transform is None:
                # Do not publish a geometrically invalid map.
                return

            transforms.append(transform)

        merged_map = self.build_merged_map(
            maps,
            transforms,
        )

        if merged_map is None:
            return

        self.publisher.publish(merged_map)

        self.get_logger().info(
            f"Merged map published: "
            f"{merged_map.info.width}x"
            f"{merged_map.info.height}, "
            f"resolution={merged_map.info.resolution:.3f}, "
            f"frame='{merged_map.header.frame_id}'"
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = MergeMapNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()