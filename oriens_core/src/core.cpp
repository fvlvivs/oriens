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


#include <rclcpp/qos.hpp>
#include <string>

#include "oriens_core/core.hpp"

Core::Core(const std::string &node_name, const rclcpp::NodeOptions &options)
    : Node(node_name, options) {

  declare_parameter("sensor_type", "marg");
  declare_parameter("input_topic", "marg/raw_data");
  declare_parameter("output_topic", "marg/filtered_data");
  declare_parameter("accelerometer.bias.x", 0.0);
  declare_parameter("accelerometer.bias.y", 0.0);
  declare_parameter("accelerometer.bias.z", 0.0);
  declare_parameter("gyroscope.bias.x", 0.0);
  declare_parameter("gyroscope.bias.y", 0.0);
  declare_parameter("gyroscope.bias.z", 0.0);
  declare_parameter("magnetometer.bias.x", 0.0);
  declare_parameter("magnetometer.bias.y", 0.0);
  declare_parameter("magnetometer.bias.z", 0.0);
}

void Core::initialiseAsImu(
    const std::string subscriber_topic, const std::string publisher_topic,
    std::function<void(const sensor_msgs::msg::Imu::SharedPtr)> callback) {
  accelerometer_ = Accelerometer();
  gyroscope_ = Gyroscope();

  Eigen::Vector3d accelerometer_bias(
      get_parameter("accelerometer.bias.x").as_double(),
      get_parameter("accelerometer.bias.y").as_double(),
      get_parameter("accelerometer.bias.z").as_double());

  Eigen::Vector3d gyroscope_bias(get_parameter("gyroscope.bias.x").as_double(),
                                 get_parameter("gyroscope.bias.y").as_double(),
                                 get_parameter("gyroscope.bias.z").as_double());

  accelerometer_.setBias(accelerometer_bias);
  gyroscope_.setBias(gyroscope_bias);

  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
      subscriber_topic, qos, callback);
  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
      publisher_topic, rclcpp::SensorDataQoS());
}

void Core::initialiseAsMarg(
    const std::string topic_name, const std::string publisher_topic,
    std::function<void(const oriens_msgs::msg::Marg::SharedPtr)> callback) {

  accelerometer_ = Accelerometer();
  gyroscope_ = Gyroscope();
  magnetometer_ = Magnetometer();

  Eigen::Vector3d accelerometer_bias(
      get_parameter("accelerometer.bias.x").as_double(),
      get_parameter("accelerometer.bias.y").as_double(),
      get_parameter("accelerometer.bias.z").as_double());

  Eigen::Vector3d gyroscope_bias(get_parameter("gyroscope.bias.x").as_double(),
                                 get_parameter("gyroscope.bias.y").as_double(),
                                 get_parameter("gyroscope.bias.z").as_double());

  Eigen::Vector3d magnetometer_bias(
      get_parameter("magnetometer.bias.x").as_double(),
      get_parameter("magnetometer.bias.y").as_double(),
      get_parameter("magnetometer.bias.z").as_double());

  accelerometer_.setBias(accelerometer_bias);
  gyroscope_.setBias(gyroscope_bias);
  magnetometer_.setBias(magnetometer_bias);

  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  marg_subscriber_ = this->create_subscription<oriens_msgs::msg::Marg>(
      topic_name, qos, callback);
  marg_publisher_ = this->create_publisher<oriens_msgs::msg::Marg>(
      publisher_topic, rclcpp::SensorDataQoS());
}

void Core::publishImuMsg(Eigen::Vector3d &acc, Eigen::Vector3d &gyro,
                         Eigen::Quaterniond &q) {

  sensor_msgs::msg::Imu msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "imu_frame"; // TODO: add frame_id to the parameters

  msg.linear_acceleration.x = acc(0);
  msg.linear_acceleration.y = acc(1);
  msg.linear_acceleration.z = acc(2);

  msg.angular_velocity.x = gyro(0);
  msg.angular_velocity.y = gyro(1);
  msg.angular_velocity.z = gyro(2);

  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();

  imu_publisher_->publish(msg);
}
void Core::publishMargMsg(Eigen::Vector3d &acc, Eigen::Vector3d &gyro,
                          Eigen::Vector3d &mag, Eigen::Quaterniond &q) {

  oriens_msgs::msg::Marg msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "marg_frame"; // TODO: add frame_id to the parameters

  msg.linear_acceleration.x = acc(0);
  msg.linear_acceleration.y = acc(1);
  msg.linear_acceleration.z = acc(2);

  msg.angular_velocity.x = gyro(0);
  msg.angular_velocity.y = gyro(1);
  msg.angular_velocity.z = gyro(2);

  msg.magnetic_field.x = mag(0);
  msg.magnetic_field.y = mag(1);
  msg.magnetic_field.z = mag(2);

  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();

  marg_publisher_->publish(msg);
}

void Core::updateSensors(const sensor_msgs::msg::Imu::SharedPtr msg) {
  accelerometer_.feedData(msg->linear_acceleration.x,
                          msg->linear_acceleration.y,
                          msg->linear_acceleration.z);

  gyroscope_.feedData(msg->angular_velocity.x, msg->angular_velocity.y,
                      msg->angular_velocity.z);
}

void Core::updateSensors(const oriens_msgs::msg::Marg::SharedPtr msg) {

  accelerometer_.feedData(msg->linear_acceleration.x,
                          msg->linear_acceleration.y,
                          msg->linear_acceleration.z);

  gyroscope_.feedData(msg->angular_velocity.x, msg->angular_velocity.y,
                      msg->angular_velocity.z);

  magnetometer_.feedData(msg->magnetic_field.x, msg->magnetic_field.y,
                         msg->magnetic_field.z);
}

void Core::updateSamplingTime(const std_msgs::msg::Header &msg) {

  double current_time = msg.stamp.sec + msg.stamp.nanosec * 1e-9;
  dt_ = current_time - last_time_;
  last_time_ = current_time;
}

bool Core::isSamplingTimeTooBig() {

  if (dt_ > 1.0) {
    RCLCPP_WARN(this->get_logger(), "dt is too large: %f", dt_);
    return true;
  }

  return false;
}
