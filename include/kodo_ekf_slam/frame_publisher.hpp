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
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <Eigen/Dense>

namespace kodo_ekf_slam {

class FramePublisher {
public:
    FramePublisher(rclcpp::Node::SharedPtr node,
                   const std::string &map_frame = "map",
                   const std::string &odom_frame = "odom");
    void broadcast(const Eigen::Vector3d &ekf_pose,
                   const rclcpp::Time &stamp,
                   const geometry_msgs::msg::Pose &odom_pose);
private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string map_frame_, odom_frame_;
};

}   // namespace