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

#include "kodo_ekf_slam/motion_model.hpp"
#include <cmath>

namespace kodo_ekf_slam {

MotionModel::MotionModel(double noise_v, double noise_omega)
: nv_(noise_v), nom_(noise_omega) {}

MotionOutput MotionModel::predict(const Vector3d &pose, double v, double omega, double dt) const {
    double xr = pose(0), yr = pose(1), tr = pose(2);
    Vector3d new_pose;

    if (fabs(omega) < 1e-6) {
        // Straight motion
        new_pose(0) = xr + v*dt*cos(tr);
        new_pose(1) = yr + v*dt*sin(tr);
        new_pose(2) = tr;
    } else {
        new_pose(0) = xr + (v/omega) * (sin(tr + omega*dt) - sin(tr));
        new_pose(1) = yr - (v/omega) * (cos(tr + omega*dt) - cos(tr));
        new_pose(2) = tr + omega*dt;
    }

    while (new_pose(2) > M_PI) new_pose(2) -= 2*M_PI;
    while (new_pose(2) < -M_PI) new_pose(2) += 2*M_PI;

    // Jacobian G (3x3)
    MatrixXd G = MatrixXd::Identity(3, 3);
    if (fabs(omega) < 1e-6) {
        G(0,2) = -v * dt * sin(tr);
        G(1,2) = -v * dt * cos(tr);
    } else {
        G(0,2) = (v/omega) * (cos(tr + omega*dt) - cos(tr));
        G(1,2) = (v/omega) * (sin(tr + omega*dt) - sin(tr));
    }

    // Process noise R (3x3) in robot pose space
    MatrixXd R = MatrixXd::Zero(3, 3);
    R(0,0) = nv_ * fabs(v) * dt; 
    R(1,1) = nv_ * fabs(v) * dt; 
    R(2,2) = nom_ * fabs(omega) * dt;
    
    MotionOutput out;
    out.pose = new_pose;
    out.G = G;
    out.R = R;
    return out;
}

}   // namespace