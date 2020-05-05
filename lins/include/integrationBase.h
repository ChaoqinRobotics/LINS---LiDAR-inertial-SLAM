// Copyright (C) 2016 Tong Qin
// The Hong Kong University of Science and Technology

#ifndef INCLUDE_INTEGRATIONBASE_H_
#define INCLUDE_INTEGRATIONBASE_H_

#include <math_utils.h>
#include <parameters.h>

#include <KalmanFilter.hpp>
#include <cassert>
#include <cmath>
#include <cstring>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

using namespace std;
using namespace Eigen;
using namespace filter;

namespace integration {
enum StateOrder { O_R = 0, O_P = 3, O_V = 6, O_BA = 9, O_BG = 12 };

enum NoiseOrder { O_AN = 0, O_GN = 3, O_AW = 6, O_GW = 9 };

const double ACC_N = 1e-4;
const double GYR_N = 1e-4;
const double ACC_W = 1e-8;
const double GYR_W = 1e-8;

class IntegrationBase {
 public:
  IntegrationBase() = delete;
  IntegrationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                  const Eigen::Vector3d &_linearized_ba,
                  const Eigen::Vector3d &_linearized_bg)
      : acc_0{_acc_0},
        gyr_0{_gyr_0},
        linearized_acc{_acc_0},
        linearized_gyr{_gyr_0},
        linearized_ba{_linearized_ba},
        linearized_bg{_linearized_bg},
        jacobian{Eigen::Matrix<double, 15, 15>::Identity()},
        covariance{Eigen::Matrix<double, 15, 15>::Zero()},
        sum_dt{0.0},
        delta_p{Eigen::Vector3d::Zero()},
        delta_q{Eigen::Quaterniond::Identity()},
        delta_v{Eigen::Vector3d::Zero()}

  {}

  void push_back(double dt, const Eigen::Vector3d &acc,
                 const Eigen::Vector3d &gyr) {
    dt_buf.push_back(dt);
    acc_buf.push_back(acc);
    gyr_buf.push_back(gyr);
    propagate(dt, acc, gyr);
  }

  void midPointIntegration(
      double _dt, const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
      const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
      const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q,
      const Eigen::Vector3d &delta_v, const Eigen::Vector3d &linearized_ba,
      const Eigen::Vector3d &linearized_bg, Eigen::Vector3d &result_delta_p,
      Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
      Eigen::Vector3d &result_linearized_ba,
      Eigen::Vector3d &result_linearized_bg, bool update_jacobian) {
    Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
    Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
    result_delta_q =
        delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2,
                              un_gyr(2) * _dt / 2);
    Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
    Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
    result_delta_v = delta_v + un_acc * _dt;

    result_linearized_ba = linearized_ba;
    result_linearized_bg = linearized_bg;

    if (update_jacobian) {
      Vector3d w_x =
          0.5 * (_gyr_0 + _gyr_1) - linearized_bg;  // angular_velocity
      Vector3d a_0_x =
          _acc_0 - linearized_ba;  // acceleration measurement - bias
      Vector3d a_1_x = _acc_1 - linearized_ba;
      Matrix3d R_w_x, R_a_0_x, R_a_1_x;

      R_w_x << 0, -w_x(2), w_x(1),  // [w-b]x, cross product
          w_x(2), 0, -w_x(0), -w_x(1), w_x(0), 0;
      R_a_0_x << 0, -a_0_x(2), a_0_x(1),  // [w-a]x
          a_0_x(2), 0, -a_0_x(0), -a_0_x(1), a_0_x(0), 0;
      R_a_1_x << 0, -a_1_x(2), a_1_x(1), a_1_x(2), 0, -a_1_x(0), -a_1_x(1),
          a_1_x(0), 0;

      // the order of a and theta is exchanged. and F = I + F*dt + 0.5*F^2*dt^2
      MatrixXd F = MatrixXd::Zero(15, 15);
      F.block<3, 3>(GlobalState::pos_, GlobalState::pos_) =
          Matrix3d::Identity();
      F.block<3, 3>(GlobalState::pos_, GlobalState::att_) =
          -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt +
          -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x *
              (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
      F.block<3, 3>(GlobalState::pos_, GlobalState::vel_) =
          MatrixXd::Identity(3, 3) * _dt;
      F.block<3, 3>(GlobalState::pos_, GlobalState::acc_) =
          -0.25 *
          (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) *
          _dt * _dt;
      F.block<3, 3>(GlobalState::pos_, GlobalState::gyr_) =
          -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt *
          -_dt;

      F.block<3, 3>(GlobalState::att_, GlobalState::att_) =
          Matrix3d::Identity() - R_w_x * _dt;
      F.block<3, 3>(GlobalState::att_, GlobalState::gyr_) =
          -1.0 * MatrixXd::Identity(3, 3) * _dt;

      F.block<3, 3>(GlobalState::vel_, GlobalState::att_) =
          -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt +
          -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x *
              (Matrix3d::Identity() - R_w_x * _dt) * _dt;
      F.block<3, 3>(GlobalState::vel_, GlobalState::vel_) =
          Matrix3d::Identity();
      F.block<3, 3>(GlobalState::vel_, GlobalState::acc_) =
          -0.5 *
          (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) *
          _dt;
      F.block<3, 3>(GlobalState::vel_, GlobalState::gyr_) =
          -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;

      F.block<3, 3>(GlobalState::acc_, GlobalState::acc_) =
          Matrix3d::Identity();
      F.block<3, 3>(GlobalState::gyr_, GlobalState::gyr_) =
          Matrix3d::Identity();

      jacobian = F * jacobian;
    }

    /*
    Vector3d vk0 = _acc_0*dt;
    Vector3d ak0 = _gyr_0*dt;
    Vector3d vk1 = _acc_1*dt;
    Vector3d ak1 = _gyr_1*dt;

    Vector3d dv = vk1 + 0.5*ak1.cross(vk1) + 1.0/12*(ak0.cross(vk1) +
    vk0.cross(ak1)); Vector3d da = 0.5*(ak0+ak1);

    result_delta_q = delta_q * Quaterniond(1, da(0)/2, da(1)/2, da(2)/2);
    result_delta_v = delta_v + result_delta_q*dv;
    Vector3d aver_v = 0.5*(result_delta_v + delta_v);
    result_delta_p = delta_p + aver_v * _dt;

    result_linearized_ba = linearized_ba;
    result_linearized_bg = linearized_bg;*/
  }

  // estimate alpha, beta, and gamma in eqa (5)
  void propagate(double _dt, const Eigen::Vector3d &_acc_1,
                 const Eigen::Vector3d &_gyr_1) {
    dt = _dt;
    acc_1 = _acc_1;
    gyr_1 = _gyr_1;
    Vector3d result_delta_p;
    Quaterniond result_delta_q;
    Vector3d result_delta_v;
    Vector3d result_linearized_ba;
    Vector3d result_linearized_bg;

    midPointIntegration(_dt, acc_0, gyr_0, acc_1, _gyr_1, delta_p, delta_q,
                        delta_v, linearized_ba, linearized_bg, result_delta_p,
                        result_delta_q, result_delta_v, result_linearized_ba,
                        result_linearized_bg, 1);

    // checkJacobian(_dt, acc_0, gyr_0, acc_1, gyr_1, delta_p, delta_q, delta_v,
    //                    linearized_ba, linearized_bg);
    delta_p = result_delta_p;
    delta_q = result_delta_q;
    delta_v = result_delta_v;
    linearized_ba = result_linearized_ba;
    linearized_bg = result_linearized_bg;
    delta_q.normalize();
    sum_dt += dt;
    acc_0 = acc_1;
    gyr_0 = gyr_1;
  }

  void setBa(const Eigen::Vector3d &ba) { linearized_ba = ba; }

  void setBg(const Eigen::Vector3d &bg) { linearized_bg = bg; }

  Eigen::Vector3d G;

  double dt;
  Eigen::Vector3d acc_0, gyr_0;
  Eigen::Vector3d acc_1, gyr_1;

  const Eigen::Vector3d linearized_acc, linearized_gyr;
  Eigen::Vector3d linearized_ba, linearized_bg;

  Eigen::Matrix<double, 15, 15> jacobian, covariance;
  Eigen::Matrix<double, 15, 15> step_jacobian;
  Eigen::Matrix<double, 15, 18> step_V;
  Eigen::Matrix<double, 18, 18> noise;

  double sum_dt;
  Eigen::Vector3d delta_p;
  Eigen::Quaterniond delta_q;
  Eigen::Vector3d delta_v;

  std::vector<double> dt_buf;
  std::vector<Eigen::Vector3d> acc_buf;
  std::vector<Eigen::Vector3d> gyr_buf;
};
}  // namespace integration

#endif  // INCLUDE_INTEGRATIONBASE_H_

