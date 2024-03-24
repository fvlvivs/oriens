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


#ifndef ORIENS_COMPLEMENTARY_FILTER_COMPLEMENTARY_FILTER_HPP
#define ORIENS_COMPLEMENTARY_FILTER_COMPLEMENTARY_FILTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <eigen3/Eigen/Dense>

#include "oriens_core/core.hpp"
#include "oriens_msgs/msg/marg.hpp"

class ComplementaryFilter : public Core{
    
public:
    ComplementaryFilter(const std::string &node_name,
                        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~ComplementaryFilter() = default;

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr) override;
    void margCallback(const oriens_msgs::msg::Marg::SharedPtr) override;
    void getOrientationFromGyroscope(Eigen::Vector3d &angular_rate,
        double &dt, Eigen::Vector3d &angles);
    void getOrientationFromAccelerometer(Eigen::Vector3d &acceleration,
        Eigen::Vector3d &angles);
    void getOrientationFromMagnetometer(Eigen::Vector3d &magnetic_field,
        Eigen::Vector3d &angles);

    Eigen::Vector3d euler_angles_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyroscope_data_;
    Eigen::Vector3d accelerometer_data_;
    Eigen::Vector3d magnetometer_data_;

    double alpha_{0.98};
    double magnetometer_declination_{0.0};
    bool magnetometer_negate_y_{false};
};

#endif // ORIENS_COMPLEMENTARY_FILTER_COMPLEMENTARY_FILTER_HPP