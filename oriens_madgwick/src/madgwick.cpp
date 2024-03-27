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

#include "oriens_madgwick/madgwick.hpp"


Madgwick::Madgwick(const std::string &node_name,
                   const rclcpp::NodeOptions &options)
    : Core(node_name, options) {

  declare_parameter("mu", rclcpp::PARAMETER_DOUBLE);
  declare_parameter("gamma", rclcpp::PARAMETER_DOUBLE);
  declare_parameter("gravity_reference_field.x", 0.0);
  declare_parameter("gravity_reference_field.y", 0.0);
  declare_parameter("gravity_reference_field.z", 1.0);
  declare_parameter("magnetic_reference_field.x", 1.0);
  declare_parameter("magnetic_reference_field.y", 0.0);
  declare_parameter("magnetic_reference_field.z", 1.0);

  q_est_ = Eigen::Quaterniond::Identity();
  q_est_prev_ = Eigen::Quaterniond::Identity();
  q_dot_w_ = Eigen::Quaterniond::Identity();
  q_w_ = Eigen::Quaterniond::Identity();

  std::string sensor_type, input_topic, output_topic;
  get_parameter("sensor_type", sensor_type);
  get_parameter("input_topic", input_topic);
  get_parameter("output_topic", output_topic);
  get_parameter("mu", mu_);
  get_parameter("gamma", gamma_);
  get_parameter("gravity_reference_field.x", gravity_reference_field_[0]);
  get_parameter("gravity_reference_field.y", gravity_reference_field_[1]);
  get_parameter("gravity_reference_field.z", gravity_reference_field_[2]);
  get_parameter("magnetic_reference_field.x", magnetic_reference_field_[0]);
  get_parameter("magnetic_reference_field.y", magnetic_reference_field_[1]);
  get_parameter("magnetic_reference_field.z", magnetic_reference_field_[2]);

  if (sensor_type == "imu") {
    auto callback =
        std::bind(&Madgwick::imuCallback, this, std::placeholders::_1);
    initialiseAsImu(input_topic, output_topic, callback);
  } else if (sensor_type == "marg") {
    auto callback =
        std::bind(&Madgwick::margCallback, this, std::placeholders::_1);
    initialiseAsMarg(input_topic, output_topic, callback);
  } else {
    RCLCPP_ERROR(get_logger(), "Invalid sensor type");
  }

  RCLCPP_INFO(this->get_logger(), "Madgwick Node (%s) has been created", sensor_type.c_str());
}

void Madgwick::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                       "madgwick: imu callback");

  updateSensors(msg);
  updateSamplingTime(msg->header);
  if (isSamplingTimeTooBig())
    return;

  gyroscope_.getData(gyroscope_data_);
  orientationFromAngularRate(q_est_prev_, gyroscope_data_, dt_, q_w_);

  accelerometer_.getData(accelerometer_data_);
  Eigen::Vector3d cost_function;
  Eigen::Matrix<double, 3, 4> jacobian;
  orientationFromVectorObservation(q_est_prev_, accelerometer_data_,
                                   gravity_reference_field_, cost_function,
                                   jacobian);

  Eigen::Vector4d g_gradient;
  g_gradient = jacobian.transpose() * cost_function;
  double grad_norm = g_gradient.norm();
  if (grad_norm > 1e-6)
    g_gradient /= grad_norm;

  Eigen::Quaterniond q_gradient;
  q_gradient.w() = q_est_prev_.w() - mu_ * g_gradient(0);
  q_gradient.x() = q_est_prev_.x() - mu_ * g_gradient(1);
  q_gradient.y() = q_est_prev_.y() - mu_ * g_gradient(2);
  q_gradient.z() = q_est_prev_.z() - mu_ * g_gradient(3);
  q_gradient.normalize();

  q_est_.w() = q_gradient.w() * gamma_ + q_w_.w() * (1 - gamma_);
  q_est_.x() = q_gradient.x() * gamma_ + q_w_.x() * (1 - gamma_);
  q_est_.y() = q_gradient.y() * gamma_ + q_w_.y() * (1 - gamma_);
  q_est_.z() = q_gradient.z() * gamma_ + q_w_.z() * (1 - gamma_);
  q_est_.normalize();

  q_est_prev_ = q_est_;

  publishImuMsg(accelerometer_data_, gyroscope_data_, q_est_);
}

void Madgwick::margCallback(const oriens_msgs::msg::Marg::SharedPtr msg) {

  updateSensors(msg);
  updateSamplingTime(msg->header);
  if (isSamplingTimeTooBig())
    return;

  gyroscope_.getData(gyroscope_data_);
  orientationFromAngularRate(q_est_prev_, gyroscope_data_, dt_, q_w_);

  accelerometer_.getData(accelerometer_data_);
  magnetometer_.getData(magnetometer_data_);
  Eigen::Vector3d cost_function_g, cost_function_m;
  Eigen::Matrix<double, 3, 4> jacobian_g, jacobian_m;
  orientationFromVectorObservation(q_est_prev_, accelerometer_data_,
                                   gravity_reference_field_, cost_function_g,
                                   jacobian_g);
  orientationFromVectorObservation(q_est_prev_, magnetometer_data_,
                                   magnetic_reference_field_, cost_function_m,
                                   jacobian_m);

  Eigen::Matrix<double, 6, 1> cost_function;
  cost_function.block<3, 1>(0, 0) = cost_function_g;
  cost_function.block<3, 1>(3, 0) = cost_function_m;

  Eigen::Matrix<double, 6, 4> jacobian;
  jacobian.block<3, 4>(0, 0) = jacobian_g;
  jacobian.block<3, 4>(3, 0) = jacobian_m;

  Eigen::Matrix<double, 4, 1> g_gradient;
  g_gradient = jacobian.transpose() * cost_function;
  double grad_norm = g_gradient.norm();
  if (grad_norm > 1e-6)
    g_gradient /= grad_norm;

  Eigen::Quaterniond q_gradient;
  q_gradient.w() = q_est_prev_.w() - mu_ * g_gradient(0);
  q_gradient.x() = q_est_prev_.x() - mu_ * g_gradient(1);
  q_gradient.y() = q_est_prev_.y() - mu_ * g_gradient(2);
  q_gradient.z() = q_est_prev_.z() - mu_ * g_gradient(3);
  q_gradient.normalize();

  q_est_.w() = q_gradient.w() * gamma_ + q_w_.w() * (1 - gamma_);
  q_est_.x() = q_gradient.x() * gamma_ + q_w_.x() * (1 - gamma_);
  q_est_.y() = q_gradient.y() * gamma_ + q_w_.y() * (1 - gamma_);
  q_est_.z() = q_gradient.z() * gamma_ + q_w_.z() * (1 - gamma_);
  q_est_.normalize();

  q_est_prev_ = q_est_;

  publishMargMsg(accelerometer_data_, gyroscope_data_, magnetometer_data_, q_est_);
}

void Madgwick::orientationFromAngularRate(const Eigen::Quaterniond &quaternion,
                                          const Eigen::Vector3d &gyroscope,
                                          const double dt,
                                          Eigen::Quaterniond &q_ar) {

  Eigen::Quaterniond gyro_quat(0, gyroscope[0], gyroscope[1], gyroscope[2]);
  Eigen::Quaterniond q_dot_w = quaternion * gyro_quat;
  q_dot_w.w() *= 0.5;
  q_dot_w.x() *= 0.5;
  q_dot_w.y() *= 0.5;
  q_dot_w.z() *= 0.5;

  q_ar.w() = quaternion.w() + dt * q_dot_w.w();
  q_ar.x() = quaternion.x() + dt * q_dot_w.x();
  q_ar.y() = quaternion.y() + dt * q_dot_w.y();
  q_ar.z() = quaternion.z() + dt * q_dot_w.z();
  q_ar.normalize();
}

void Madgwick::orientationFromVectorObservation(
    const Eigen::Quaterniond &quaternion, const Eigen::Vector3d &sensor,
    const Eigen::Vector3d &referece_field, Eigen::Vector3d &cost_function,
    Eigen::Matrix<double, 3, 4> &jacobian) {

  double qw = quaternion.w();
  double qx = quaternion.x();
  double qy = quaternion.y();
  double qz = quaternion.z();
  double sx = sensor[0];
  double sy = sensor[1];
  double sz = sensor[2];
  double dx = referece_field[0];
  double dy = referece_field[1];
  double dz = referece_field[2];

  cost_function(0) = 2 * dx * (0.5 - qy * qy - qz * qz) +
                     2 * dy * (qw * qz + qx * qy) +
                     2 * dz * (qx * qz - qw * qy) - sx;
  cost_function(1) = 2 * dx * (qx * qy - qw * qz) +
                     2 * dy * (0.5 - qx * qx - qz * qz) +
                     2 * dz * (qw * qx + qy * qz) - sy;
  cost_function(2) = 2 * dx * (qw * qy + qx * qz) +
                     2 * dy * (qy * qz - qw * qx) +
                     2 * dz * (0.5 - qx * qx - qy * qy) - sz;

  jacobian(0, 0) = 2 * dy * qz - 2 * dz * qy;
  jacobian(0, 1) = 2 * dy * qy + 2 * dz * qz;
  jacobian(0, 2) = -4 * dx * qy + 2 * dy * qx - 2 * dz * qw;
  jacobian(0, 3) = -4 * dx * qz + 2 * dy * qw + 2 * dz * qx;

  jacobian(1, 0) = -2 * dx * qz + 2 * dz * qx;
  jacobian(1, 1) = 2 * dx * qy - 4 * dy * qx + 2 * dz * qw;
  jacobian(1, 2) = 2 * dx * qx + 2 * dz * qz;
  jacobian(1, 3) = -2 * dx * qw - 4 * dy * qz + 2 * dz * qy;

  jacobian(2, 0) = 2 * dx * qy - 2 * dy * qx;
  jacobian(2, 1) = 2 * dx * qz - 2 * dy * qw - 4 * dz * qx;
  jacobian(2, 2) = 2 * dx * qw + 2 * dy * qz - 4 * dz * qy;
  jacobian(2, 3) = 2 * dx * qx + 2 * dy * qy;
}
