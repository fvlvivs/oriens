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


#include "oriens_support/serial_reader.hpp"


SerialReader::SerialReader(const std::string &node_name, const rclcpp::NodeOptions & options) : Node(node_name, options) {

    this->declare_parameter<std::string>("port_name", "/dev/ttyACM0");
    this->declare_parameter<int>("baudrate", B500000);
    this->declare_parameter<int>("period", 5);
    this->declare_parameter<std::string>("topic_name", "raw_data");
    this->declare_parameter<std::string>("sensor_type", "marg");

    this->get_parameter("port_name", port_name_);
    this->get_parameter("baudrate", baudrate_);
    this->get_parameter("period", period_);
    this->get_parameter("topic_name", topic_name_);
    this->get_parameter("sensor_type", sensor_type_);

    // print port name
    RCLCPP_INFO(this->get_logger(), "Port name: %s", port_name_.c_str());

    if (sensor_type_ == "imu")
            this->initAsImu();
    else if (sensor_type_ == "marg")
            this->initAsMarg();
    else
        RCLCPP_ERROR(this->get_logger(), "Invalid sensor type %s", sensor_type_.c_str());


    serial_port_ = open(port_name_.c_str(), O_RDWR);

    if (serial_port_ < 0)
        RCLCPP_ERROR(this->get_logger(), "Error %i from open: %s\n", errno, strerror(errno));

    struct termios tty;

    // Read in existing settings, and handle any error
    if (tcgetattr(this->serial_port_, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
    tty.c_cflag |= CS8;            // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;                                                        // Disable echo
    tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
    tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
    tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    cfsetispeed(&tty, baudrate_);
    cfsetospeed(&tty, baudrate_);

    if (tcsetattr(this->serial_port_, TCSANOW, &tty) != 0)
      RCLCPP_ERROR(this->get_logger(), "Error %i from tcsetattr: %s\n", errno, strerror(errno));


    RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_),
        std::bind(&SerialReader::timerCallback, this));
}

void SerialReader::initAsImu() {
    data_size_ = 6;
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/" + topic_name_, rclcpp::SensorDataQoS());
    RCLCPP_INFO(this->get_logger(), "Initialized as IMU");
}

void SerialReader::initAsMarg() {
    data_size_ = 9;
    marg_publisher_ = this->create_publisher<oriens_msgs::msg::Marg>("/marg/" + topic_name_, rclcpp::SensorDataQoS());
    RCLCPP_INFO(this->get_logger(), "Initialized as MARG");
}

SerialReader::~SerialReader() {}

void SerialReader::timerCallback() {
    std::vector<double> data;
    this->readSerial(data);
    if (sensor_type_ == "imu")
        publishImuData(data);
    else if (sensor_type_ == "marg")
        publishMargData(data);
}

void SerialReader::publishImuData(const std::vector<double> &data) {

    if (data.size() != data_size_) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid data size");
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Publishing IMU data");
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "";

    imu_msg.linear_acceleration.x = data[0];
    imu_msg.linear_acceleration.y = data[1];
    imu_msg.linear_acceleration.z = data[2];

    imu_msg.angular_velocity.x = data[3];
    imu_msg.angular_velocity.y = data[4];
    imu_msg.angular_velocity.z = data[5];

    imu_publisher_->publish(imu_msg);
}

void SerialReader::publishMargData(const std::vector<double> &data) {

    if (data.size() != data_size_) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid data size");
        return;
    }
    
    oriens_msgs::msg::Marg marg_msg;
    marg_msg.header.stamp = this->now();
    marg_msg.header.frame_id = "";

    marg_msg.linear_acceleration.x = data[0];
    marg_msg.linear_acceleration.y = data[1];
    marg_msg.linear_acceleration.z = data[2];

    marg_msg.angular_velocity.x = data[3];
    marg_msg.angular_velocity.y = data[4];
    marg_msg.angular_velocity.z = data[5];

    marg_msg.magnetic_field.x = data[6];
    marg_msg.magnetic_field.y = data[7];
    marg_msg.magnetic_field.z = data[8];

    marg_publisher_->publish(marg_msg);
}

void SerialReader::readLine(std::string &line) {

    char buffer[128];
    memset(&buffer, 0, sizeof(buffer));

    int n = read(serial_port_, buffer, sizeof(buffer));
    if (n > 0) {
        string_data_ += buffer;
        std::vector<std::string::size_type> positions;
        std::string::size_type pos = 0;
        while ((pos = string_data_.find('\n', pos)) != std::string::npos) {
            positions.push_back(pos);
            ++pos;
        }

        // get the latest stream between two newlines 
        size_t pos_size = positions.size();
        if (pos_size > 2) {
            line = string_data_.substr(positions[pos_size-2], positions[pos_size-1]);
            string_data_ = string_data_.substr(positions[pos_size-1] + 1);
        } 
    }

}

void SerialReader::readSerial(std::vector<double> &data) {

    std::string line;
    this->readLine(line);

    if (line.empty())
        return;

    RCLCPP_DEBUG(this->get_logger(), "%s", line.c_str());

    std::stringstream ss(line);
    std::vector<double> v;
    double tmp;
    while (ss >> tmp) {
        v.push_back(tmp);
        if (ss.peek() == ',')
            ss.ignore();
    }

    data = v;
}
