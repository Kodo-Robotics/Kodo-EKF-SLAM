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

#include "kodo_ekf_slam/frame_publisher.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace kodo_ekf_slam {

FramePublisher::FramePublisher(rclcpp::Node::SharedPtr node,
                               const std::string &map_frame,
                               const std::string &odom_frame)
    : node_(node), map_frame_(map_frame), odom_frame_(odom_frame) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_.get());
}

void FramePublisher::broadcast(const Eigen::Vector3d &ekf_pose,
                               const rclcpp::Time &stamp,
                               const geometry_msgs::msg::Pose &odom_pose) {
    // compute transform T_map_odom = T_map_ekf * T_odom_base^{-1}
    tf2::Transform T_map_ekf;
    T_map_ekf.setOrigin(tf2::Vector3(ekf_pose(0), ekf_pose(1), 0));
    tf2::Quaternion q_map; q_map.setRPY(0, 0, ekf_pose(2));
    T_map_ekf.setRotation(q_map);

    tf2::Transform T_odom_base;
    tf2::Quaternion q_od; tf2::fromMsg(odom_pose.orientation, q_od);
    T_odom_base.setOrigin(tf2::Vector3(odom_pose.position.x, odom_pose.position.y, 0));
    T_odom_base.setRotation(q_od);

    tf2::Transform T_map_odom = T_map_ekf * T_odom_base.inverse();

    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp = stamp;
    ts.header.frame_id = map_frame_;
    ts.child_frame_id = odom_frame_;
    ts.transform.translation.x = T_map_odom.getOrigin().x();
    ts.transform.translation.y = T_map_odom.getOrigin().y();
    ts.transform.rotation = tf2::toMsg(T_map_odom.getRotation());

    tf_broadcaster_->sendTransform(ts);
}

}   // namespace