// MIT License

// Copyright (c) 2024 Fulvio Di Luzio (fulvio.diluzio@gmail.com)

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#include "oriens_complementary_filter/complementary_filter.hpp"

ComplementaryFilter::ComplementaryFilter(const std::string &node_name,
    const rclcpp::NodeOptions &options) : Core(node_name, options) {

    this->declare_parameter("alpha", 0.98);
    this->declare_parameter("magnetometer_declination", 0.0);
    this->declare_parameter("magnetometer_negate_y", false);

    this->get_parameter("alpha", this->alpha_);
    this->get_parameter("magnetometer_declination", this->magnetometer_declination_);
    this->get_parameter("magnetometer_negate_y", this->magnetometer_negate_y_);

    std::string sensor_type = "marg";
    auto callback = std::bind(&ComplementaryFilter::margCallback, this, std::placeholders::_1);
    initialiseAsMarg(
        "marg/raw_data", 
        "marg/orientation", 
        callback);

    RCLCPP_INFO(get_logger(), "Complementary Filter Node (%s) has been created", sensor_type.c_str());    
}

void ComplementaryFilter::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
}

void ComplementaryFilter::margCallback(const oriens_msgs::msg::Marg::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Received MARG data");

    updateSensors(msg);
    updateSamplingTime(msg->header);
    if (isSamplingTimeTooBig())
        return;
    
    Eigen::Vector3d gyro_angles = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_angles = Eigen::Vector3d::Zero();
    Eigen::Vector3d mag_angles = Eigen::Vector3d::Zero();
    gyroscope_.getData(gyroscope_data_);
    accelerometer_.getData(accelerometer_data_);
    magnetometer_.getData(magnetometer_data_);

    this->getOrientationFromGyroscope(gyroscope_data_, dt_, gyro_angles);
    this->getOrientationFromAccelerometer(accelerometer_data_, acc_angles);
    this->getOrientationFromMagnetometer(magnetometer_data_, mag_angles);

    euler_angles_[0] = alpha_ * (euler_angles_[0] + gyro_angles[0]) 
        + (1 - alpha_) * acc_angles[0];
    euler_angles_[1] = alpha_ * (euler_angles_[1] + gyro_angles[1])
        + (1 - alpha_) * acc_angles[1];
    euler_angles_[2] = alpha_ * (euler_angles_[2] + gyro_angles[2])
        + (1 - alpha_) * mag_angles[2];
    
    Eigen::Quaterniond q = Eigen::AngleAxisd(euler_angles_[0], Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(euler_angles_[1], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(euler_angles_[2], Eigen::Vector3d::UnitZ());

    publishMargMsg(accelerometer_data_, gyroscope_data_, magnetometer_data_, q);
}

void ComplementaryFilter::getOrientationFromGyroscope(Eigen::Vector3d &angular_rate, double &dt, Eigen::Vector3d &angles) {
    angles = angular_rate * dt;
}

void ComplementaryFilter::getOrientationFromAccelerometer(Eigen::Vector3d &acceleration, Eigen::Vector3d &angles) {
    double norm = acceleration.norm();
    if (norm < 1e-6)
        return;

    Eigen::Vector3d normalized_acceleration = acceleration / norm;
    double roll = atan2(normalized_acceleration.y(), normalized_acceleration.z());
    double pitch = atan2(-normalized_acceleration.x(),
        sqrt(normalized_acceleration.y() * normalized_acceleration.y()
        + normalized_acceleration.z() * normalized_acceleration.z()));
    angles << roll, pitch, 0.0;
}

void ComplementaryFilter::getOrientationFromMagnetometer(Eigen::Vector3d &magnetic_field, Eigen::Vector3d &angles) {
    double mag_y = magnetic_field.y() ? !magnetometer_negate_y_ : -magnetic_field.y();
    double mag_yaw = 0;

    if (mag_y > 0)
        mag_yaw = M_PI / 2 - atan2(-magnetic_field.x(), mag_y);
    else if (mag_y < 0)
        mag_yaw = 3 * M_PI / 2 - atan2(magnetic_field.x(), -mag_y);

    if (fabs(mag_y) < 1e-6 && magnetic_field.x() < 0)
        mag_yaw = M_PI;
    else if (fabs(mag_y) < 1e-6 && magnetic_field.x() > 0)
        mag_yaw = 0;

    if (mag_yaw > M_PI)
        mag_yaw -= 2 * M_PI;
    else if (mag_yaw < -M_PI)
        mag_yaw += 2 * M_PI;

    mag_yaw += magnetometer_declination_;
    angles << 0.0, 0.0, mag_yaw;
}