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

#include "kodo_ekf_slam/data_association.hpp"
#include <limits>

namespace kodo_ekf_slam {

DataAssociation::DataAssociation(double chi2_gate) : gate_(chi2_gate) {}

int DataAssociation::associate(const Vector2d &z, const MatrixXd &Sigma,
        const std::vector<Vector2d> &landmarks, const Vector2d &robot_pose, double robot_theta,
        const std::function<void(int, Vector2d&, MatrixXd&)> &getPredAndH) const {
    
    int N = (int)landmarks.size();
    if (N == 0) return -1;

    double bestM = std::numeric_limits<double>::infinity();
    int bestIdx = -1;
    Vector2d zpred; MatrixXd H;
    for (int i = 0; i < N; i++) {
        getPredAndH(i, zpred, H);
        Vector2d v = z - zpred;
        // normalize angle
        while (v(1) > M_PI) v(1) -= 2*M_PI;
        while (v(1) < -M_PI) v(1) += 2*M_PI;
        MatrixXd S = H * Sigma * H.transpose();
        // Add small measurement noise inside S if singular
        Eigen::FullPivLU<MatrixXd> lu(S);
        if (!lu.isInvertible()) {
            S += MatrixXd::Identity(S.rows(), S.cols()) * 1e-6;
        }
        double m = v.transpose() * S.inverse() * v;
        if (m < bestM) { bestM = m; bestIdx = i; }
    }
    if (bestM < gate_) return bestIdx;
    return -1;
}

}   // namespace