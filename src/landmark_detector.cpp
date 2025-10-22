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

#include "kodo_ekf_slam/landmark_detector.hpp"
#include <cmath>

namespace kodo_ekf_slam {

LandmarkDetector::LandmarkDetector(double max_range, double gap, int min_size)
: max_range_(max_range), gap_(gap), min_size_(min_size) {}

std::vector<std::pair<double,double>> LandmarkDetector::detect(const sensor_msgs::msg::LaserScan::SharedPtr &msg) const {
    std::vector<std::pair<double,double>> found;
    const auto &ranges = msg->ranges;
    int n = ranges.size();
    double ang_min = msg->angle_min;
    double ang_inc = msg->angle_increment;

    std::vector<int> cluster;
    auto flush = [&](void) {
        if ((int)cluster.size() >= min_size_) {
            double sx = 0, sy = 0;
            for (int idx : cluster) {
                double a = ang_min + idx*ang_inc;
                double r = ranges[idx];
                sx += r * std::cos(a);
                sy += r * std::sin(a);
            }
            sx /= cluster.size(); sy /= cluster.size();
            double rr = std::hypot(sx, sy);
            double bb = std::atan2(sy, sx);
            found.emplace_back(rr, bb);
        }
        cluster.clear();
    };

    for (int i = 0; i < n; i++) {
        double r = ranges[i];
        double ang = ang_min + i*ang_inc;
        if (!std::isfinite(r) || r <= 0.0 || r > max_range_) {
            flush();
            continue;
        }

        if (cluster.empty()) { cluster.push_back(i); continue; }
        int prev = cluster.back();
        double a_prev = ang_min + prev*ang_inc;
        double r_prev = ranges[prev];
        double dx = r*std::cos(ang) - r_prev*std::cos(a_prev);
        double dy = r*std::sin(ang) - r_prev*std::sin(a_prev);
        double dist = std::hypot(dx, dy);
        if (dist < gap_) cluster.push_back(i);
        else { flush(); cluster.push_back(i); }
    }
    flush();
    return found;
}

}   // namespace