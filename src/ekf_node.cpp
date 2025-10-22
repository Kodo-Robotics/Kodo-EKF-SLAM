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

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "kodo_ekf_slam/ekf_core.hpp"
#include "kodo_ekf_slam/landmark_detector.hpp"
#include "kodo_ekf_slam/visualizer.hpp"
#include "kodo_ekf_slam/frame_publisher.hpp"

using std::placeholders::_1;
using namespace kodo_ekf_slam;

class EKFNode : public rclcpp::Node {
public:
    EKFNode(): Node("kodo_ekf_slam"), cmd_valid_(false) {
        // params
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        this->declare_parameter<std::string>("odom_topic", "/odom");
        this->declare_parameter<std::string>("scan_topic", "/scan");
        this->declare_parameter<std::string>("landmark_topic", "/landmark_observations");

        this->declare_parameter<double>("motion_noise_v", 0.02);
        this->declare_parameter<double>("motion_noise_omega", 0.02);
        this->declare_parameter<double>("meas_r", 0.05);
        this->declare_parameter<double>("meas_b", 0.02);
        this->declare_parameter<double>("assoc_chi2", 5.99);
        this->declare_parameter<double>("scan_max_range", 6.0);
        this->declare_parameter<double>("scan_gap", 0.2);
        this->declare_parameter<int>("scan_min_size", 3);

        this->get_parameter("cmd_vel_topic", cmd_vel_topic_);
        this->get_parameter("odom_topic", odom_topic_);
        this->get_parameter("scan_topic", scan_topic_);
        this->get_parameter("landmark_topic", landmark_topic_);

        double nv, nom, mr, mb, chi2;
        this->get_parameter("motion_noise_v", nv);
        this->get_parameter("motion_noise_omega", nom);
        this->get_parameter("meas_r", mr);
        this->get_parameter("meas_b", mb);
        this->get_parameter("assoc_chi2", chi2);

        ekf_ = std::make_shared<EKFCore>(nv, nom, mr, mb, chi2);
        double smax, sgap; int smin;
        this->get_parameter("scan_max_range", smax);
        this->get_parameter("scan_gap", sgap);
        this->get_parameter("scan_min_size", smin);
        detector_ = std::make_shared<LandmarkDetector>(smax, sgap, smin);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&EKFNode::initialize_, this));

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            cmd_vel_topic_, 10, std::bind(&EKFNode::cmd_vel_cb, this, _1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10, std::bind(&EKFNode::odom_cb, this, _1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, 5, std::bind(&EKFNode::scan_cb, this, _1));
        landmark_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            landmark_topic_, 5, std::bind(&EKFNode::landmark_cb, this, _1));

        // timers
        double update_rate = 20.0;      // Hz for prediction/udpate
        double vis_rate = 10.0;         // Hz for visualization
        update_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / update_rate),
            std::bind(&EKFNode::timerUpdateStep, this));
        vis_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / vis_rate),
            std::bind(&EKFNode::timerVisualizeStep, this));

        RCLCPP_INFO(this->get_logger(), "EKF SLAM Node Initialized");
    }

private:
    void initialize_() {
        timer_->cancel();
        viz_ = std::make_shared<Visualizer>(shared_from_this());
        frame_pub_ = std::make_shared<FramePublisher>(shared_from_this());
    }

    void cmd_vel_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        last_cmd_time_ = this->now();
        last_v_ = msg->twist.linear.x;
        last_omega_ = msg->twist.angular.z;
        cmd_valid_ = true;
    }

    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(odom_mutex_);
        last_odom_pose_ = msg->pose.pose;
        odom_stamp_ = msg->header.stamp;
    }

    void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(scan_mutex_);
        latest_scan_ = msg;
    }

    void landmark_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(lm_mutex_);
        latest_landmarks_msg_ = msg;
    }

    void timerUpdateStep() {
        // 1. Prediction from cmd_vel
        if (cmd_valid_) {
            double dt = (this->now() - last_cmd_time_).seconds();
            ekf_->predict(last_v_, last_omega_, dt);
            cmd_valid_ = false;
        }

        // 2. Correction from scan clusters
        sensor_msgs::msg::LaserScan::SharedPtr scan;
        {
            std::lock_guard<std::mutex> lk(scan_mutex_);
            scan = latest_scan_;
            latest_scan_.reset();
        }
        if (scan) {
            auto obs = detector_->detect(scan);
            for (const auto &p: obs) {
                ekf_->process_observation(p.first, p.second);
            }
        }

        // 3. Correction from external landmark observations if available
        {
            std::lock_guard<std::mutex> lk(lm_mutex_);
            if (latest_landmarks_msg_) {
                for (const auto &p: latest_landmarks_msg_->poses) {
                    double r = std::hypot(p.position.x, p.position.y);
                    double b = std::atan2(p.position.y, p.position.x);
                    ekf_->process_observation(r, b);
                }
                latest_landmarks_msg_.reset();
            }
        }
    }

    void timerVisualizeStep() {
        // Broadcast map->odom transform
        geometry_msgs::msg::Pose odom_pose;
        rclcpp::Time stamp;
        {
            std::lock_guard<std::mutex> lk(odom_mutex_);
            odom_pose = last_odom_pose_;
            stamp = odom_stamp_;
        }
        frame_pub_->broadcast(ekf_->robot_pose(), stamp, odom_pose);

        // Publish visuals
        viz_->publish_landmarks(ekf_->landmarks());
        viz_->publish_pose(ekf_->robot_pose(), ekf_->robot_covariance());
        viz_->publish_path_append(ekf_->robot_pose());
    }

    // members
    std::string cmd_vel_topic_, odom_topic_, scan_topic_, landmark_topic_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr landmark_sub_;

    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr vis_timer_;

    bool cmd_valid_;
    double last_v_, last_omega_;
    rclcpp::Time last_cmd_time_;

    std::mutex scan_mutex_, lm_mutex_, odom_mutex_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    geometry_msgs::msg::PoseArray::SharedPtr latest_landmarks_msg_;
    geometry_msgs::msg::Pose last_odom_pose_;
    rclcpp::Time odom_stamp_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<EKFCore> ekf_;
    std::shared_ptr<LandmarkDetector> detector_;
    std::shared_ptr<Visualizer> viz_;
    std::shared_ptr<FramePublisher> frame_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EKFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}