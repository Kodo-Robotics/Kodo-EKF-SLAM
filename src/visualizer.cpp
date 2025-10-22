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

#include "kodo_ekf_slam/visualizer.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.h>

using namespace std::chrono_literals;

namespace kodo_ekf_slam {

Visualizer::Visualizer(rclcpp::Node::SharedPtr node) : node_(node) {
    landmark_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("ekf_landmarks", 10);
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("ekf_pose", 10);
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("ekf_path", 10);
    path_.header.frame_id = "map";
}

void Visualizer::publish_landmarks(const std::vector<Eigen::Vector2d> &landmarks) {
    visualization_msgs::msg::MarkerArray ma;
    for (size_t i = 0; i < landmarks.size(); i++) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = node_->now();
        m.ns = "ekf_landmarks";
        m.id = (int)i;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = landmarks[i][0];
        m.pose.position.y = landmarks[i][1];
        m.pose.position.z = 0.0;
        m.scale.x = 0.15; m.scale.y = 0.15; m.scale.z = 0.15;
        m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f; m.color.a = 0.9f;
        ma.markers.push_back(m);
    }
    landmark_pub_->publish(ma);
}

void Visualizer::publish_pose(const Eigen::Vector3d &pose, const Eigen::MatrixXd &Sigma_robot) {
    nav_msgs::msg::Odometry od;
    od.header.stamp = node_->now();
    od.header.frame_id = "map";
    od.child_frame_id = "base_link";
    od.pose.pose.position.x = pose(0);
    od.pose.pose.position.y = pose(1);
    od.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, pose(2));
    od.pose.pose.orientation = tf2::toMsg(q);
    // fill covariance (3x3 robot block into pose.covariance layout)
    od.pose.covariance.fill(0.0);
    if (Sigma_robot.rows() >= 3) {
        od.pose.covariance[0] = Sigma_robot(0,0);
        od.pose.covariance[1] = Sigma_robot(0,1);
        od.pose.covariance[5] = Sigma_robot(0,2);
        od.pose.covariance[6] = Sigma_robot(1,0);
        od.pose.covariance[7] = Sigma_robot(1,1);
        od.pose.covariance[11] = Sigma_robot(1,2);
        od.pose.covariance[30] = Sigma_robot(2,0);
        od.pose.covariance[31] = Sigma_robot(2,1);
        od.pose.covariance[35] = Sigma_robot(2,2);
    }
    odom_pub_->publish(od);
}

void Visualizer::publish_path_append(const Eigen::Vector3d &pose) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = node_->now();
    ps.header.frame_id = "map";
    ps.pose.position.x = pose(0);
    ps.pose.position.y = pose(1);
    ps.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, pose(2));
    ps.pose.orientation = tf2::toMsg(q);
    path_.poses.push_back(ps);
    if (path_.poses.size() > 500) path_.poses.erase(path_.poses.begin());
    path_.header.stamp = node_->now();
    path_pub_->publish(path_);
}

}   // namespace