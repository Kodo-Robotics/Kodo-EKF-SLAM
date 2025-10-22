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
#include <Eigen/Dense>
#include <vector>
#include <mutex>

#include "kodo_ekf_slam/motion_model.hpp"
#include "kodo_ekf_slam/measurement_model.hpp"
#include "kodo_ekf_slam/data_association.hpp"

namespace kodo_ekf_slam {

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;

class EKFCore{
public:
    EKFCore(double noise_v, double noise_omega,
            double meas_r, double meas_b,
            double assoc_chi2);
    // robot pose is first 3 elements of mu
    const VectorXd &state() const;
    const MatrixXd &covariance() const;
    Vector3d robot_pose() const;
    MatrixXd robot_covariance() const;
    std::vector<Vector2d> landmarks() const;

    // operations
    void predict(double v, double omega, double dt);
    // measurement z in robot frame (r, b) and optional prior odomPose for TF broadcast
    void process_observation(double r, double b);
    // helper to get measurement predicted/H for association
    void get_pred_and_H_for_landmark(int idx, Vector2d &zpred, MatrixXd &H) const;

private:
    VectorXd mu_;
    MatrixXd Sigma_;
    MotionModel motion_;
    MeasurementModel meas_;
    DataAssociation assoc_;
    std::mutex mtx_;

    // utilities
    void add_landmark_from_rb(double r, double b);
};

}   // namespace