// MIT License

// Copyright (c) 2024 Fulvio Di Luzio

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "oriens_kalman/kalman.hpp"


Kalman::Kalman(const std::string &node_name,
                   const rclcpp::NodeOptions &options)
    : Core(node_name, options) {

}

Eigen::Matrix4d Kalman::makeSkewMatrix() {
   Eigen::Matrix4d omega = Eigen::Matrix4d::Zero();

   omega.row(0) << 0.0, -gyroscope_.z(), gyroscope_.y(), gyroscope_.x();
   omega.row(1) << gyroscope_.z(), 0.0, -gyroscope_.x(), gyroscope_.y();
   omega.row(2) << -gyroscope_.y(), gyroscope_.x(), 0.0, gyroscope_.z();
   omega.row(3) << -gyroscope_.x(), -gyroscope_.y(), -gyroscope_.z(), 0.0;

  return omega;
}
