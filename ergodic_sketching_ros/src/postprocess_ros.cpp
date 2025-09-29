// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
//
// SPDX-License-Identifier: GPL-3.0-only

#include <algorithm>
#include <string>
#include <vector>
#include <array>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include "rclcpp/rclcpp.hpp"

#include <sackmesser/ConfigurationServer.hpp>
#include <sackmesser_runtime/Interface.hpp>

#include <ergodic_sketching/ErgodicControl.hpp>
#include <ergodic_sketching/ErgodicSketching.hpp>
#include <ergodic_sketching/RobotDrawing.hpp>
#include <ergodic_sketching/TrajectoryUtils.hpp>

#include <ergodic_sketching_msgs/srv/post_process.hpp>

sackmesser::runtime::Interface::Ptr interface;
std::unique_ptr<sketching::RobotDrawing> robot_drawing;
std::string base_frame = "world";
std::shared_ptr<rclcpp::Node> node;

void postprocess(const std::shared_ptr<ergodic_sketching_msgs::srv::PostProcess::Request>& req, std::shared_ptr<ergodic_sketching_msgs::srv::PostProcess::Response> resp) {
    RCLCPP_INFO(node->get_logger(),"Sketch request received");
    
    std::vector<std::vector<Eigen::Vector2d>> strokes;
    for(auto const& ros_stroke: req->strokes){
        std::vector<Eigen::Vector2d> stroke;
        for(auto const& ros_point: ros_stroke.poses){
            Eigen::Vector2d point(ros_point.pose.position.x,ros_point.pose.position.y);
            stroke.push_back(point);
        }
        strokes.push_back(stroke);
    }

    std::vector<Eigen::Matrix<double, 7, 1>> path_7d = robot_drawing->process(strokes, robot_drawing->getDrawingZonesTransforms()[req->drawing_zone_idx.data]);

    resp->path.header.stamp = node->get_clock()->now();
    resp->path.header.frame_id = base_frame;

    for (auto const& pose : path_7d) {
        geometry_msgs::msg::PoseStamped pose_ros;
        pose_ros.header.stamp = node->get_clock()->now();
        pose_ros.header.frame_id = base_frame;
        pose_ros.pose.position.x = pose(0);
        pose_ros.pose.position.y = pose(1);
        pose_ros.pose.position.z = pose(2);

        pose_ros.pose.orientation.w = pose(3);
        pose_ros.pose.orientation.x = pose(4);
        pose_ros.pose.orientation.y = pose(5);
        pose_ros.pose.orientation.z = pose(6);
        resp->path.poses.push_back(pose_ros);
    }

    return;
}

std::array<double,3> stringArrayToDoubleArray(std::string raw_array) {
    // Trim from end
    raw_array.erase(std::find_if(raw_array.rbegin(), raw_array.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(), raw_array.end());

    const std::string delimiter = " ";
    std::array<double,3> array;

    size_t index = 0;
    std::string token;
    while (true) {
        size_t pos = raw_array.find(delimiter);

        if (index > 2) {
            throw std::runtime_error("[urdf_parsing] Expected a 3d array got more, check arrays format.");
        }

        if (pos > 0) {
            token = raw_array.substr(0, pos);

            array[index] = std::stod(token);
            index++;
        }

        if (pos == std::string::npos) {
            break;
        }

        raw_array.erase(0, pos + delimiter.length());
    }
    if (index != 3) {
        throw std::runtime_error("[urdf_parsing] Expected a 3d array got less, check arrays format.");
    }

    return array;
}

int main(int argc, char** argv) {
    rclcpp::init(argc,argv);
    node = std::make_shared<rclcpp::Node>("ergodic_sketching_ros");

    node->declare_parameter<std::string>("path");
    node->declare_parameter<std::string>("config_file");
    node->declare_parameter<std::string>("drawing_frame_rpy");
    node->declare_parameter<std::string>("drawing_frame_xyz");
    node->declare_parameter<std::string>("base_frame");

    std::string path;
    std::string config_file;
    std::string drawing_frame_rpy_str;
    std::string drawing_frame_xyz_str;

    if (!node->get_parameter("path",path)) {
        RCLCPP_ERROR(node->get_logger(),"No parameters ~/path");
        return 1;
    }

    if (!node->get_parameter("config_file",config_file)) {
        RCLCPP_ERROR(node->get_logger(),"No parameters ~/config_file");
        return 1;
    }

    if(!node->get_parameter("drawing_frame_rpy",drawing_frame_rpy_str)){
        RCLCPP_ERROR(node->get_logger(),"No parameters /drawing_frame_rpy");
        return 1;
    }

    if(!node->get_parameter("drawing_frame_xyz",drawing_frame_xyz_str)){
        RCLCPP_ERROR(node->get_logger(),"No parameters /drawing_frame_xyz");
        return 1;
    }

    if(!node->get_parameter("base_frame", base_frame)){
        RCLCPP_ERROR(node->get_logger(),"No parameters base_frame");
    }

    std::array<double,3> drawing_frame_rpy = stringArrayToDoubleArray(drawing_frame_rpy_str);
    std::array<double,3> drawing_frame_xyz = stringArrayToDoubleArray(drawing_frame_xyz_str);

    // Adapt for different representation between Eigen and ROS...
    drawing_frame_rpy[0] = - drawing_frame_rpy[0];
    drawing_frame_rpy[1] = - drawing_frame_rpy[1];

    RCLCPP_INFO(node->get_logger(),"Ergodic sketching: path=%s", path.c_str());
    RCLCPP_INFO(node->get_logger(),"Ergodic sketching: config_file=%s", config_file.c_str());

    interface = std::make_shared<sackmesser::runtime::Interface>(path, config_file);
    robot_drawing = std::make_unique<sketching::RobotDrawing>(interface, "ergodic_sketching/robot_drawing/",drawing_frame_xyz,drawing_frame_rpy);

    rclcpp::Service<ergodic_sketching_msgs::srv::PostProcess>::SharedPtr service = node->create_service<ergodic_sketching_msgs::srv::PostProcess>(
        "post_process",
        &postprocess
    );

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
