# SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
#
# SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
#
# SPDX-License-Identifier: GPL-3.0-only

import os
import pathlib
import argparse
from datetime import datetime
import py_panda2.robot
from py_panda2 import utils
import numpy
import threading
import queue

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ergodic_sketching_msgs.srv import PostProcess
from ergodic_sketching_msgs.action import Planner
import rclpy.publisher
from sensor_msgs.msg import JointState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import GetParameters

class Sketcher(Node):
    def __init__(self, strokes: list[Path], log_dir: pathlib.Path):
        super().__init__("drozbot_client")
        
        self._strokes = strokes

        self._param_client = self.create_client(GetParameters, "/planner_action_server/get_parameters")
        self._param_client.wait_for_service()
        param_req = GetParameters.Request()
        param_req.names=["joint_names"]
        param_future = self._param_client.call_async(param_req)
        rclpy.spin_until_future_complete(self,param_future)
        param_res = param_future.result()
        self._joint_names = param_res.values[0].string_array_value

        self._path_pub= self.create_publisher(Path,"/rob_sim/path",10)
        
        self._sketch_client = self.create_client(PostProcess,"/post_process")
        self._sketch_client.wait_for_service()

        self._client = ActionClient(self,Planner,"/plan")

        self._log_dir = log_dir
        self._traj = []
        self._commands = []
        self._costs = []        

    def process(self):
        sketch_req = PostProcess.Request()
        sketch_req.strokes = self._strokes
        sketch_future = self._sketch_client.call_async(sketch_req)
        rclpy.spin_until_future_complete(self,sketch_future)
        result = sketch_future.result()
        self._path_pub.publish(result.path)
        self._start_planner(result.path)

    def _start_planner(self,path: Path)->None:
        goal = Planner.Goal()
        goal.path = path
        self._client.wait_for_server()
        future = self._client.send_goal_async(goal,feedback_callback=self._planner_feedback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result

        log_dir = self._log_dir / datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        log_dir.mkdir()

        costs = numpy.asarray(self._costs)
        joint_positions = numpy.asarray(self._traj)
        joint_velocities = numpy.asarray(self._commands)

        info = f"cost: min={costs.min()}, max={costs.max()}, std={costs.std()}, mean={costs.mean()} \n"
        info += f"q: min={joint_positions.min(axis=0)}, max={joint_positions.max(axis=0)}, std={joint_positions.std(axis=0)}, mean={joint_positions.mean(axis=0)} \n"
        info += f"dq: min={joint_velocities.min(axis=0)}, max={joint_velocities.max(axis=0)}, std={joint_velocities.std(axis=0)}, mean={joint_velocities.mean(axis=0)} \n"

        info_file = log_dir / "sumary.txt"
        info_file.touch()
        info_file.write_text(info)

        numpy.savetxt(str(log_dir/"costs.csv"),costs,delimiter=",")
        numpy.savetxt(str(log_dir/"joint_positions.csv"),joint_positions,delimiter=",")
        numpy.savetxt(str(log_dir/"joint_velocities.csv"),joint_velocities,delimiter=",")

        rbt = py_panda2.robot.Robot(use_gripper=False,tip_link="pen_link",sim=False)
        utils.joint_state_trajectory_following(rbt,joint_positions,[1]*7,0.01,True)

        self.get_logger().info("Sketching over")

    def _planner_feedback(self, feedback):
        traj = []
        cmd = []
        for i,p in enumerate(feedback.feedback.traj.points): # Go through the received trajectory
            traj += [p.positions]
            cmd += [p.velocities]

        self._costs += [feedback.feedback.cost]
        self._traj += traj
        self._commands += cmd

def drozbot() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("-i","--input-file",dest="input_csv", type=str, help="csv file to draw")
    parser.add_argument("-l","--logdir",dest="log_dir",type=str,help="log dir")
    args = parser.parse_args()

    log_path = pathlib.Path(args.log_dir)
    if not log_path.exists():
        log_path.mkdir()

    csv_path = pathlib.Path(args.input_csv)
    if not csv_path.exists():
        raise FileNotFoundError(f"provided csv file \"{str(csv_path)}\" does not exist!")

    strokes = []
    for stroke_str in csv_path.read_text().split(";"):
        if len(stroke_str) < 1:
            continue
        
        stroke = Path()

        for p in stroke_str.split("\n"):
            if len(p) < 1:
                continue
            points_str = p.split(" ")
            p_ros = PoseStamped()
            p_ros.pose.position.x = float(points_str[0])
            p_ros.pose.position.y = float(points_str[1])
            stroke.poses.append(
                p_ros
            )
        stroke.poses = stroke.poses[::4]
        strokes.append(stroke)

    rclpy.init()
    node = Sketcher(strokes,log_path)
    node.process()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    drozbot()