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


#ifndef ORIENS_SERIALREADER_HPP
#define ORIENS_SERIALREADER_HPP


#include <chrono>
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <oriens_msgs/msg/marg.hpp>


class SerialReader : public rclcpp::Node {

public:
    SerialReader(const std::string &node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~SerialReader();

    bool isConnected() const { return is_connected_; }
    void readSerial(std::vector<double> &data);
    void readLine(std::string &line);

private:
    void timerCallback();
    void initAsImu();
    void initAsMarg();
    void publishImuData(const std::vector<double> &data);
    void publishMargData(const std::vector<double> &data);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<oriens_msgs::msg::Marg>::SharedPtr marg_publisher_;

    std::string port_name_;
    int baudrate_;
    int serial_port_;
    int period_;
    bool is_connected_ {false};

    int data_size_;
    std::string topic_name_ {""};
    std::string sensor_type_ {""};
    std::string string_data_ {""};
};



#endif // ORIENS_SERIALREADER_HPP