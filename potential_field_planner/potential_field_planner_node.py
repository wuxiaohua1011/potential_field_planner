#!/usr/bin/env python3
from re import A
from std_msgs.msg import Header
import rclpy
import rclpy.node
import numpy as np
from sensor_msgs.msg import Image
import rclpy
import rclpy.node

import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped, Transform
from typing import Optional, Tuple
from pathlib import Path
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations
import cv2
import time

from costmap_msgs.msg import CostMapMsg
from costmap_msgs.srv import GetCostMap, GetInflatedCostMap
from example_interfaces.srv import AddTwoInts
from cv_bridge import CvBridge
import tf_transformations
import imutils


class PotentialFieldPlannerNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("potential_field_planner")
        self.declare_parameter("inflated_costmap_srv", "inflated_costmap_srv")
        self.declare_parameter("duration", 0.1)  # seconds
        self.declare_parameter("lag_toleration", 0.1)  # seconds

        self.cli = self.create_client(
            GetInflatedCostMap,
            self.get_parameter("inflated_costmap_srv")
            .get_parameter_value()
            .string_value,
        )
        self.get_logger().info(f"Subscribing to service at [{self.cli.srv_name}]")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.duration_nano = (
            self.get_parameter("duration").get_parameter_value().double_value
        ) * 1e9
        self.lag_toleration_nano = (
            self.get_parameter("lag_toleration").get_parameter_value().double_value
        ) * 1e9

        self.get_logger().info(
            f"Duration = {self.duration_nano/1e9}. Lag toleration = {self.lag_toleration_nano/1e9}"
        )

        self.create_timer(self.duration_nano / 1e9, self.timer_callback)
        self.client_futures = []
        self.latest_costmap_msg = None
        self.bridge = CvBridge()
        self.visual_publisher = self.create_publisher(Image, "/inflated_map", 10)

    def get_transform(self, from_frame_rel, to_frame_rel) -> Optional[TransformStamped]:
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, now)
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {to_frame_rel} to {from_frame_rel}: {ex}"
            )
            return
        return trans

    def timer_callback(self):
        trans_stamped: Optional[TransformStamped] = self.get_transform(
            "ego_vehicle", "map"
        )
        if trans_stamped is None:
            return
        else:
            self.send_request(transform=trans_stamped.transform)
            if self.check_msg_time():
                image = self.bridge.imgmsg_to_cv2(self.latest_costmap_msg.map)
                self.publish_visual(image, transform=trans_stamped.transform)

    def check_msg_time(self):
        if self.latest_costmap_msg is not None:
            msg_time_nano = (
                self.latest_costmap_msg.header.stamp.sec * 1e9
                + self.latest_costmap_msg.header.stamp.nanosec
            )

            curr_time_nano = self.get_clock().now().nanoseconds
            time_diff_nano = curr_time_nano - msg_time_nano

            # if the map returned within time
            if time_diff_nano < self.duration_nano + self.lag_toleration_nano:
                return True

            else:
                msg = f"Response is taking too long: lag toleration = {self.lag_toleration_nano/1e9}. time taken = {time_diff_nano/1e9}"
                self.get_logger().info(msg)
                return False

    def publish_visual(self, image: np.ndarray, transform=Transform):
        """Given a gray scale image, turn it into jet color scheme and publish it

        Args:
            image (np.ndarray): raw gray scale image
        """
        image = image / np.max(image)
        image = cv2.applyColorMap((image * 255).astype(np.uint8), cv2.COLORMAP_JET)
        cv2.imshow("image", image)
        cv2.waitKey(1)
        self.visual_publisher.publish(self.bridge.cv2_to_imgmsg(image))

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            # manage incomplete futures
            incomplete_futures = []
            for f in self.client_futures:
                if f.done():
                    res = f.result()
                    self.latest_costmap_msg = res.map
                else:
                    incomplete_futures.append(f)
            self.client_futures = incomplete_futures

    def send_request(self, transform: Transform):
        req = GetInflatedCostMap.Request()
        req.width = 100
        req.height = 100
        req.min_x = transform.translation.x - req.width / 2
        req.min_y = transform.translation.y - req.height / 2
        req.obstacle_kernel_len = 51
        req.obstacle_kernel_std = 15
        req.obstacle_threshold = 0.5
        self.client_futures.append(self.cli.call_async(req))


def main(args=None):
    rclpy.init()
    node = PotentialFieldPlannerNode()
    node.spin()
    node.destroy_node()


if __name__ == "__main__":
    main()
