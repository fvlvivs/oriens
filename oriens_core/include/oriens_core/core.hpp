// MIT License

// Copyright (c) 2024 Fulvio Di Luzio (fulvio.diluzio@gmail.com)

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


#ifndef  ORIENS_CORE_CORE_HPP
#define  ORIENS_CORE_CORE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "oriens_msgs/msg/marg.hpp"
#include "oriens_core/accelerometer.hpp"
#include "oriens_core/gyroscope.hpp"
#include "oriens_core/magnetometer.hpp"

class Core : public rclcpp::Node {

public:
  Core(const std::string &node_name,
       const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~Core() = default;

protected:
  virtual void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {};
  virtual void margCallback(const oriens_msgs::msg::Marg::SharedPtr msg) {};
  void updateSensors(const sensor_msgs::msg::Imu::SharedPtr msg);
  void updateSensors(const oriens_msgs::msg::Marg::SharedPtr msg);
  void updateSamplingTime(const std_msgs::msg::Header &msg);
  bool isSamplingTimeTooBig();
  void initialiseAsImu(
    const std::string subscriber_topic,
    const std::string publisher_topic,
    std::function<void(const sensor_msgs::msg::Imu::SharedPtr)> callback);
  void initialiseAsMarg(
    const std::string subscriber_topic,
    const std::string publisher_topic,
    std::function<void(const oriens_msgs::msg::Marg::SharedPtr)> callback);
  void publishImuMsg(
    Eigen::Vector3d &acc, Eigen::Vector3d &gyro, Eigen::Quaterniond &q);
  void publishMargMsg(
    Eigen::Vector3d &acc, Eigen::Vector3d &gyro, Eigen::Vector3d &mag, Eigen::Quaterniond &q);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<oriens_msgs::msg::Marg>::SharedPtr marg_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<oriens_msgs::msg::Marg>::SharedPtr marg_publisher_;

  Accelerometer accelerometer_;
  Gyroscope gyroscope_;
  Magnetometer magnetometer_;
  double dt_;
  double last_time_;
};

#endif // ORIENS_CORE_CORE_HPP