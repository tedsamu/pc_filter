#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
from sensor_msgs_py import point_cloud2


class SimplePointCloudFilter(Node):
    def __init__(self):
        super().__init__('pointcloud_filter')

        # Declare parameters
        self.declare_parameter('input_topic', '/lidar/points')
        self.declare_parameter('output_topic', '/lidar/filtered')
        self.declare_parameter('filter_field', 'x')  # x, y, z
        self.declare_parameter('filter_min', 0.0)
        self.declare_parameter('filter_max', 100.0)
        self.declare_parameter('filter_negative', False)

        # Get parameters
        input_topic = self.get_parameter(
            'input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter(
            'output_topic').get_parameter_value().string_value
        self.filter_field = self.get_parameter(
            'filter_field').get_parameter_value().string_value
        self.filter_min = self.get_parameter(
            'filter_min').get_parameter_value().double_value
        self.filter_max = self.get_parameter(
            'filter_max').get_parameter_value().double_value
        self.filter_negative = self.get_parameter(
            'filter_negative').get_parameter_value().bool_value

        # Field mapping
        self.field_map = {'x': 0, 'y': 1, 'z': 2}

        if self.filter_field not in self.field_map:
            self.get_logger().error(
                f'Invalid filter field: {self.filter_field}. '
                'Must be x, y, or z'
            )
            return

        # Create subscription and publisher
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.pointcloud_callback,
            10
        )

        self.publisher = self.create_publisher(PointCloud2, output_topic, 10)

        self.get_logger().info('Point Cloud Filter started')
        self.get_logger().info(f'Input topic: {input_topic}')
        self.get_logger().info(f'Output topic: {output_topic}')
        self.get_logger().info(
            f'Filter field: {self.filter_field}, range: ['
            f'{self.filter_min}, {self.filter_max}]'
        )

    def pointcloud_callback(self, msg):
        try:
            # Convert PointCloud2 to list of points
            points_list = list(
                point_cloud2.read_points(
                    msg, field_names=("x", "y", "z"), skip_nans=True))

            if len(points_list) == 0:
                self.get_logger().warn('Received empty point cloud')
                return

            if len(points_list) == 0:
                filtered_points = np.empty((0, 3), dtype=np.float32)
                points_array = filtered_points
            else:
                points_array = np.array(
                    [(p[0], p[1], p[2]) for p in points_list],
                    dtype=np.float32
                )
            field_idx = self.field_map[self.filter_field]
            field_values = points_array[:, field_idx]
            if self.filter_negative:
                mask = ((field_values < self.filter_min) |
                        (field_values > self.filter_max))
            else:
                mask = ((field_values >= self.filter_min) &
                        (field_values <= self.filter_max))
            filtered_points = points_array[mask]
            filtered_msg = point_cloud2.create_cloud_xyz32(
                msg.header, filtered_points)
            self.publisher.publish(filtered_msg)
            self.get_logger().debug(
                f'Filtered cloud: {len(points_array)} -> '
                f'{len(filtered_points)} points'
            )

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    pointcloud_filter = SimplePointCloudFilter()

    try:
        rclpy.spin(pointcloud_filter)
    except KeyboardInterrupt:
        pass
    finally:
        pointcloud_filter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
