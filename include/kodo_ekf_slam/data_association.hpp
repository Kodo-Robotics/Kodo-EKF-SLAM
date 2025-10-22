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
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::VectorXd;

namespace kodo_ekf_slam {

class DataAssociation {
public:
    DataAssociation(double chi2_gate);
    // returns index of associated landmark (0..N-1) or -1 if new
    int associate(const Vector2d &z, const MatrixXd &Sigma, const std::vector<Vector2d> &landmarks,
                  const Vector2d &robot_pose, double robot_theta,
                  const std::function<void(int, Vector2d&, MatrixXd&)> &getPredAndH) const;

private:
    double gate_;
};

}   // namespace