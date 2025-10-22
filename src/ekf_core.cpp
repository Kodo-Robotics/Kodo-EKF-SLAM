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

#include "kodo_ekf_slam/ekf_core.hpp"
#include <cmath>

namespace kodo_ekf_slam {

EKFCore::EKFCore(double noise_v, double noise_omega,
                 double meas_r, double meas_b,
                 double assoc_chi2)
: motion_(noise_v, noise_omega),
  meas_(meas_r, meas_b),
  assoc_(assoc_chi2) {
  mu_ = VectorXd::Zero(3);
  Sigma_ = MatrixXd::Zero(3, 3);
  Sigma_.diagonal() << 1e-6, 1e-6, 1e-6;
}

const VectorXd &EKFCore::state() const { return mu_; }
const MatrixXd &EKFCore::covariance() const { return Sigma_; }

Vector3d EKFCore::robot_pose() const {
    Vector3d p; p << mu_(0), mu_(1), mu_(2); return p;
}
MatrixXd EKFCore::robot_covariance() const {
    return Sigma_.block(0, 0, 3, 3);
}

std::vector<Vector2d> EKFCore::landmarks() const {
    std::vector<Vector2d> out;
    int N = (mu_.size() - 3) / 2;
    for (int i = 0; i < N; i++) {
        int li = 3 + 2*i;
        out.emplace_back(Vector2d(mu_(li), mu_(li+1)));
    }
    return out;
}

void EKFCore::predict(double v, double omega, double dt) {
    std::lock_guard<std::mutex> lk(mtx_);
    Vector3d pose(mu_(0), mu_(1), mu_(2));
    auto mop = motion_.predict(pose, v, omega, dt);
    int S = mu_.size();
    MatrixXd Gbig = MatrixXd::Identity(S, S);
    Gbig.block(0, 0, 3, 3) = mop.G;
    MatrixXd Rbig = MatrixXd::Zero(S, S);
    Rbig.block(0, 0, 3, 3) = mop.R;
    mu_.segment(0, 3) = mop.pose;
    Sigma_ = Gbig * Sigma_ * Gbig.transpose() + Rbig;
}

void EKFCore::add_landmark_from_rb(double r, double b) {
    Vector3d robot; robot << mu_(0), mu_(1), mu_(2);
    Vector2d lm = meas_.inverse_measurement(robot, r, b);
    int old = mu_.size();
    VectorXd mu_n = VectorXd::Zero(old + 2);
    mu_n.head(old) = mu_;
    mu_n(old) = lm(0);
    mu_n(old+1) = lm(1);
    // expand Sigma
    MatrixXd Snew = MatrixXd::Zero(old+2, old+2);
    Snew.topLeftCorner(old, old) = Sigma_;
    // Jxr & Jz for covariance initialization
    double tr = robot(2);
    MatrixXd Jxr(2, 3);
    Jxr << 1, 0, -r*std::sin(tr + b),
           0, 1,  r*std::cos(tr + b);
    MatrixXd Jz(2, 2);
    Jz << std::cos(tr + b), -r*std::sin(tr + b),
          std::sin(tr + b),  r*std::cos(tr + b);
    MatrixXd Q = meas_.measurement_noise();
    MatrixXd Sll = Jxr * Sigma_.block(0, 0, 3, 3) * Jxr.transpose() + Jz * Q * Jz.transpose();
    Snew.block(old, old, 2, 2) = Sll;
    Snew.block(0, old, old, 2) = Sigma_.block(0, 0, old, 3) * Jxr.transpose();
    Snew.block(old, 0, 2, old) = Snew.block(0, old, old, 2).transpose();
    mu_ = mu_n;
    Sigma_ = Snew;
}

void EKFCore::process_observation(double r, double b) {
    std::lock_guard<std::mutex> lk(mtx_);
    Vector2d z; z << r, b;
    auto lms = landmarks();
    // get predicted/H via lambda
    auto getter = [&](int idx, Vector2d &zpred, MatrixXd &H) {
        int li = 3 + 2*idx;
        Vector2d lm(mu_(li), mu_(li+1));
        MeasOutput mo = meas_.predict_measurement(robot_pose(), lm, li, mu_.size());
        zpred = mo.z_pred;
        H = mo.H;
    };
    int assoc_idx = assoc_.associate(z, Sigma_, lms, Vector2d(mu_(0), mu_(1)), mu_(2), getter);
    if (assoc_idx < 0) {
        add_landmark_from_rb(r, b);
        return;
    }

    // In-place EKF update
    Vector2d zpred; MatrixXd H;
    getter(assoc_idx, zpred, H);
    MatrixXd Q = meas_.measurement_noise();
    MatrixXd S = H * Sigma_ * H.transpose() + Q;
    MatrixXd K = Sigma_ * H.transpose() * S.inverse();
    VectorXd zvec(2); zvec << r, b;
    VectorXd v = zvec - zpred;
    while (v(1) > M_PI) v(1) -= 2*M_PI;
    while (v(1) < -M_PI) v(1) += 2*M_PI;
    mu_ = mu_ + K * v;
    mu_(2) = std::fmod(mu_(2) + M_PI, 2*M_PI) - M_PI;
    MatrixXd I = MatrixXd::Identity(Sigma_.rows(), Sigma_.cols());
    Sigma_ = (I - K * H) * Sigma_;
}

void EKFCore::get_pred_and_H_for_landmark(int idx, Vector2d &zpred, MatrixXd &H) const {
    int li = 3 + 2*idx;
    Vector2d lm(mu_(li), mu_(li+1));
    MeasOutput mo = meas_.predict_measurement(robot_pose(), lm, li, mu_.size());
    zpred = mo.z_pred; H = mo.H;
}

}   // namespace