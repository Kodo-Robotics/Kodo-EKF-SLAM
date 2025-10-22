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
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace kodo_ekf_slam {

struct MeasOutput {
    Vector2d z_pred;    // expected measurement r,b
    MatrixXd H;         // Jacobian wrt full state (2 x state_size)
};

class MeasurementModel {
public:
    MeasurementModel(double meas_r, double meas_b);
    // compute expected measurement and H for landmark index (li is index to state vector)
    MeasOutput predict_measurement(const Vector3d &robot_pose,
                                   const Vector2d &landmark, int landmark_state_index,
                                   int state_size) const;
    // inverse observation: from robot pose and r,b -> landmark position in map frame
    Vector2d inverse_measurement(const Vector3d &robot_pose, double r, double b) const;
    MatrixXd measurement_noise() const;

private:
    double mr_, mb_;
};

}   // namespace