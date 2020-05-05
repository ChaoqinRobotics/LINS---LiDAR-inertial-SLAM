// This file is part of LINS.
//
// Copyright (C) 2020 Chao Qin <cscharlesqin@gmail.com>,
// Robotics and Multiperception Lab (RAM-LAB <https://ram-lab.com>),
// The Hong Kong University of Science and Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.

#ifndef INCLUDE_SENSOR_UTILS_HPP_
#define INCLUDE_SENSOR_UTILS_HPP_

#include <math_utils.h>
#include <parameters.h>

#include <iostream>
#include <map>

using namespace std;
using namespace math_utils;
using namespace parameter;

namespace sensor_utils {

// Sensor measurement class
class Measurement {
 public:
  Measurement() {}
  virtual ~Measurement() {}
};

class Imu : public Measurement {
 public:
  Imu() {}
  Imu(double time, const V3D& acc, const V3D& gyr)
      : time(time), acc(acc), gyr(gyr) {}
  ~Imu() {}
  double time;
  V3D acc;  // accelerometer measurement (m^2/sec)
  V3D gyr;  // gyroscope measurement (rad/s)
};

class Gps : public Measurement {
 public:
  Gps() {
    time = 0.0;
    lat = 0.0;
    lon = 0.0;
    alt = 0.0;
  }
  Gps(double time, double lat, double lon, double alt)
      : time(time), lat(lat), lon(lon), alt(alt) {}
  ~Gps() {}
  int status;
  double time;
  double lat;
  double lon;
  double alt;
  inline V3D pn() const { return V3D(lat, lon, alt); }
};

class Odometry : public Measurement {
 public:
  Odometry() {
    time = 0.0;
    rotation.setIdentity();
    translation.setZero();
  }
  Odometry(double time, const Q4D& rotation, const V3D& translation)
      : time(time), rotation(rotation), translation(translation) {}
  ~Odometry() {}

  Odometry inverse(const Odometry& odom) {
    Odometry inv;
    inv.time = odom.time;
    inv.rotation = odom.rotation.inverse();
    inv.translation = -inv.rotation.toRotationMatrix() * odom.translation;
    return inv;
  }

  void boxPlus(const Odometry& increment) {
    translation =
        rotation.toRotationMatrix() * increment.translation + translation;
    rotation = rotation * increment.rotation;
  }

  Odometry boxMinus(const Odometry& odom) {
    Odometry res;
    res.translation =
        odom.rotation.inverse().toRotationMatrix() * translation -
        odom.rotation.inverse().toRotationMatrix() * odom.translation;
    res.rotation = odom.rotation.inverse() * rotation;
    return res;
  }

  double time;
  Q4D rotation;
  V3D translation;
};

class EarthParams {
 public:
  EarthParams() {}
  ~EarthParams() {}

  static M3D getDrp(const V3D& pn) {
    M3D Drp = M3D::Zero();
    double latitude = pn(0);
    double longitude = pn(1);
    double height = pn(2);
    double sq = 1 - EeEe * sin(latitude) * sin(latitude);
    double RNh = Re / sqrt(sq) + height;          // (2.5)
    double RMh = RNh * (1 - EeEe) / sq + height;  // (2.4)
    Drp << 0, 1.0 / RMh, 0, 1.0 / (RNh * cos(latitude)), 0, 0, 0, 0, 1;
    return Drp;
  }

  static M3D getDpr(const V3D& pn) {
    M3D Drp = getDrp(pn);
    return Drp.inverse();
  }

};

}  // namespace sensor_utils

#endif  // INCLUDE_SENSOR_UTILS_HPP_

