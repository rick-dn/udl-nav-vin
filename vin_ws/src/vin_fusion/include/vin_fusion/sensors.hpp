#pragma once

#include <Eigen/Dense>

struct ImuSample {
    double t;
    Eigen::Vector3d gyro; // [wx, wy, wz] rad/s body frame (x fwd, y left z up)
    Eigen::Vector3d accel; // [ax, ay, az] m/s^2, body frame
};

struct VioPose {
    double t; //sim time seconds
    Eigen::Vector3d p;  //[x, y, z] openVINS global frame
    Eigen::Quaterniond q; //[w, x, y, z]orientation, OpenVINS global frame
    Eigen::Matrix<double, 6, 6> cov; //[x, y, z, rx, ry, rz] covariance
};