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

#include "kodo_ekf_slam/measurement_model.hpp"
#include <cmath>

namespace kodo_ekf_slam {

MeasurementModel::MeasurementModel(double meas_r, double meas_b)
: mr_(meas_r), mb_(meas_b) {}

MeasOutput MeasurementModel::predict_measurement(const Vector3d &robot_pose,
                                                 const Vector2d &landmark,
                                                 int landmark_state_index,
                                                 int state_size) const {
    double xr = robot_pose(0), yr = robot_pose(1), tr = robot_pose(2);
    double lx = landmark(0), ly = landmark(1);
    double dx = lx - xr, dy = ly - yr;
    double q = dx * dx + dy * dy;
    double r = std::sqrt(q);
    double b = std::atan2(dy, dx) - tr;
    while (b > M_PI) b -= 2*M_PI;
    while (b < -M_PI) b += 2*M_PI;

    MeasOutput out;
    out.z_pred = Vector2d(r, b);
    out.H = MatrixXd::Zero(2, state_size);
    // robot parts
    out.H(0,0) = -dx / r;
    out.H(0,1) = -dy / r;
    out.H(0,2) = 0;
    out.H(1,0) = dy / q;
    out.H(1,1) = -dx / q;
    out.H(1,2) = 0;
    // landmark parts
    out.H(0, landmark_state_index)   = dx / r;
    out.H(0, landmark_state_index+1) = dy / r;
    out.H(1, landmark_state_index)   = -dy / q;
    out.H(1, landmark_state_index+1) = dx / q;
    return out;
}

Vector2d MeasurementModel::inverse_measurement(const Vector3d &robot_pose, double r, double b) const {
    double xr = robot_pose(0), yr = robot_pose(1), tr = robot_pose(2);
    double lx = xr + r * std::cos(tr + b);
    double ly = yr + r * std::sin(tr + b);
    return Vector2d(lx, ly);
}

MatrixXd MeasurementModel::measurement_noise() const {
    MatrixXd Q = MatrixXd::Zero(2,2);
    Q(0,0) = mr_*mr_; Q(1,1) = mb_*mb_;
    return Q;
}

}   // namespace