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

#include <eigen3/Eigen/Dense>

#include "oriens_core/core.hpp"

class Madgwick : public Core {
public:
  Madgwick(const std::string &node_name,
           const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~Madgwick() = default;

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr) override;
  void margCallback(const oriens_msgs::msg::Marg::SharedPtr) override;

  void orientationFromAngularRate(const Eigen::Quaterniond &quaternion,
                                  const Eigen::Vector3d &gyroscope,
                                  const double dt, Eigen::Quaterniond &q_ar);

  void orientationFromVectorObservation(const Eigen::Quaterniond &quaternion,
                                        const Eigen::Vector3d &sensor,
                                        const Eigen::Vector3d &referece_field,
                                        Eigen::Vector3d &cost_function,
                                        Eigen::Matrix<double, 3, 4> &jacobian);

  Eigen::Quaterniond q_est_;      // Estimated orientation
  Eigen::Quaterniond q_est_prev_; // Previous estimated orientation
  Eigen::Quaterniond q_dot_w_;    // rate of change earth frame wrt sensor frame
  Eigen::Quaterniond q_w_;        // estimated orientation based on gyroscope

  Eigen::Vector3d gyroscope_data_;
  Eigen::Vector3d accelerometer_data_;
  Eigen::Vector3d magnetometer_data_;

  Eigen::Vector3d gravity_reference_field_{0, 0, 1};
  Eigen::Vector3d magnetic_reference_field_{1, 0, 1};
  double mu_{0.1};
  double gamma_{0.8};
};
