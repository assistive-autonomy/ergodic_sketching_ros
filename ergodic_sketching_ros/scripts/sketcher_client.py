#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
#
# SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
#
# SPDX-License-Identifier: GPL-3.0-only

import pathlib
from datetime import datetime

import numpy
import cv2
import threading
import queue

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ergodic_sketching_msgs.srv import Sketch
import rclpy.publisher
from sensor_msgs.msg import JointState
from nav_msgs.msg import Path
from rcl_interfaces.srv import GetParameters
from cv_bridge import CvBridge


class Sketcher(Node):
    def __init__(self):
        super().__init__("sketcher_client")

        self._read_params()
        self._configure()

    def _read_params(self):
        self.declare_parameter("image_path", "random")
        self._image_path = self.get_parameter('image_path').value


    def _configure(self):
        # read image
        image_path = pathlib.Path(self._image_path)

        if not image_path.exists() and self._image_path != "random":
            raise FileNotFoundError(
                f'provided image file "{self._image_path}" does not exist!'
            )
        if self._image_path != "random":
            # Generate a random noise image, used to test.
            self._image = cv2.imread(self._image_path)
        else:
            image = numpy.random.rand(930, 640, 3) * 255
            self._image = image.astype(numpy.uint8)


        # Configure ROS stuff
        self._stop_ctrl_thread_event = threading.Event()
        self._traj_queue = queue.Queue()

        # Path publisher
        self._path_pub = self.create_publisher(Path, "sketch_path", 10)

        # Client
        self._sketch_client = self.create_client(Sketch, "sketch")
        self._sketch_client.wait_for_service()

    def run(self):
        bridge = CvBridge()
        sketch_req = Sketch.Request()
        sketch_req.drawing_zone_idx.data = 0
        sketch_req.image = bridge.cv2_to_imgmsg(self._image, "bgr8")
        sketch_future = self._sketch_client.call_async(sketch_req)
        rclpy.spin_until_future_complete(self, sketch_future)
        result = sketch_future.result()
        self._path_pub.publish(result.path)
        self._start_planner(result.path)

    def _start_planner(self, path: Path) -> None:
        self.get_logger().info("Start Planning")
        # Send goal to planner (CRISP)
        # goal = Planner.Goal()
        # goal.path = path
        # self._client.wait_for_server()
        # future = self._client.send_goal_async(
        #     goal, feedback_callback=self._planner_feedback
        # )
        # future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self._stop_ctrl_thread_event.set()

        self._ctrl_thread.join()
        self.get_logger().info("Sketching over")

    def _planner_feedback(self, feedback):
        print(feedback)


def main() -> None:
    rclpy.init()
    node = Sketcher()
    node.run()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
