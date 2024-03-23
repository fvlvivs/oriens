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


#ifndef ORIENS_CORE_SENSOR_HPP
#define ORIENS_CORE_SENSOR_HPP

#include <eigen3/Eigen/Dense>

class Sensor {

public:
    Sensor();
    ~Sensor() = default;

    void feedData(const double x, const double y, const double z);
    void getData(Eigen::Vector3d &data) const;
    void setBias(Eigen::Vector3d bias) {bias_ = bias;};
    double x() { return data_(0); }
    double y() { return data_(1); }
    double z() { return data_(2); }

protected:
    Eigen::Vector3d data_;
    Eigen::Vector3d bias_ = Eigen::Vector3d::Zero();
};


#endif //ORIENS_CORE_SENSOR_HPP