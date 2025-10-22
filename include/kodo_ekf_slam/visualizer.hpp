// Copyright 2025 Kodo Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <Eigen/Dense>
#include <vector>

namespace kodo_ekf_slam {

class Visualizer {
public:
    Visualizer(rclcpp::Node::SharedPtr node);
    void publish_landmarks(const std::vector<Eigen::Vector2d> &landmarks);
    void publish_pose(const Eigen::Vector3d &pose, const Eigen::MatrixXd &Sigma_robot);
    void publish_path_append(const Eigen::Vector3d &pose);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_;
};

}   // namespace