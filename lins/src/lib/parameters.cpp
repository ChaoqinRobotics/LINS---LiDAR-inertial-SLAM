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

#include <parameters.h>

namespace parameter {

// !@ENABLE_CALIBRATION
int CALIBARTE_IMU;
int SHOW_CONFIGURATION;
int AVERAGE_NUMS;

// !@INITIAL_PARAMETERS
double IMU_LIDAR_EXTRINSIC_ANGLE;
double IMU_MISALIGN_ANGLE;

// !@LIDAR_PARAMETERS
int LINE_NUM;
int SCAN_NUM;
double SCAN_PERIOD;
double EDGE_THRESHOLD;
double SURF_THRESHOLD;
double NEAREST_FEATURE_SEARCH_SQ_DIST;

// !@TESTING
int VERBOSE;
int ICP_FREQ;
int MAX_LIDAR_NUMS;
int NUM_ITER;
double LIDAR_SCALE;
double LIDAR_STD;

// !@SUB_TOPIC_NAME
std::string IMU_TOPIC;
std::string LIDAR_TOPIC;

// !@PUB_TOPIC_NAME
std::string LIDAR_ODOMETRY_TOPIC;
std::string LIDAR_MAPPING_TOPIC;

// !@KALMAN_FILTER
double ACC_N;
double ACC_W;
double GYR_N;
double GYR_W;
V3D INIT_POS_STD;
V3D INIT_VEL_STD;
V3D INIT_ATT_STD;
V3D INIT_ACC_STD;
V3D INIT_GYR_STD;

// !@INITIAL IMU BIASES
V3D INIT_BA;
V3D INIT_BW;

// !@EXTRINSIC_PARAMETERS
V3D INIT_TBL;
Q4D INIT_RBL;

template <typename T>
T readParam(ros::NodeHandle& n, std::string name) {
  T ans;
  if (n.getParam(name, ans)) {
    // ROS_INFO_STREAM("Loaded " << name << ": " << ans);
  } else {
    ROS_ERROR_STREAM("Failed to load " << name);
    n.shutdown();
  }
  return ans;
}

void readParameters(ros::NodeHandle& n) {
  std::string config_file;
  config_file = readParam<std::string>(n, "config_file");
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }

  CALIBARTE_IMU = fsSettings["calibrate_imu"];
  SHOW_CONFIGURATION = fsSettings["show_configuration"];
  AVERAGE_NUMS = fsSettings["average_nums"];
  IMU_LIDAR_EXTRINSIC_ANGLE = fsSettings["imu_lidar_extrinsic_angle"];
  IMU_MISALIGN_ANGLE = fsSettings["imu_misalign_angle"];
  LINE_NUM = fsSettings["line_num"];
  SCAN_NUM = fsSettings["scan_num"];
  SCAN_PERIOD = fsSettings["scan_period"];
  EDGE_THRESHOLD = fsSettings["edge_threshold"];
  SURF_THRESHOLD = fsSettings["surf_threshold"];
  NEAREST_FEATURE_SEARCH_SQ_DIST = fsSettings["nearest_feature_search_sq_dist"];
  VERBOSE = fsSettings["verbose"];
  ICP_FREQ = fsSettings["icp_freq"];
  MAX_LIDAR_NUMS = fsSettings["max_lidar_nums"];
  NUM_ITER = fsSettings["num_iter"];
  LIDAR_SCALE = fsSettings["lidar_scale"];
  LIDAR_STD = fsSettings["lidar_std"];

  fsSettings["imu_topic"] >> IMU_TOPIC;
  fsSettings["lidar_topic"] >> LIDAR_TOPIC;
  fsSettings["lidar_odometry_topic"] >> LIDAR_ODOMETRY_TOPIC;
  fsSettings["lidar_mapping_topic"] >> LIDAR_MAPPING_TOPIC;

  ACC_N = fsSettings["acc_n"];
  ACC_W = fsSettings["acc_w"];
  GYR_N = fsSettings["gyr_n"];
  GYR_W = fsSettings["gyr_w"];

  readV3D(&fsSettings, "init_pos_std", INIT_POS_STD);
  readV3D(&fsSettings, "init_vel_std", INIT_VEL_STD);
  readV3D(&fsSettings, "init_att_std", INIT_ATT_STD);
  readV3D(&fsSettings, "init_acc_std", INIT_ACC_STD);
  readV3D(&fsSettings, "init_gyr_std", INIT_GYR_STD);

  readV3D(&fsSettings, "init_ba", INIT_BA);
  readV3D(&fsSettings, "init_bw", INIT_BW);
  readV3D(&fsSettings, "init_tbl", INIT_TBL);
  readQ4D(&fsSettings, "init_rbl", INIT_RBL);
}

void readV3D(cv::FileStorage* file, const std::__cxx11::string& name,
             V3D& vec_eigen) {
  cv::Mat vec_cv;
  (*file)[name] >> vec_cv;
  cv::cv2eigen(vec_cv, vec_eigen);
}

void readQ4D(cv::FileStorage* file, const std::__cxx11::string& name,
             Q4D& quat_eigen) {
  cv::Mat mat_cv;
  (*file)[name] >> mat_cv;
  M3D mat_eigen;
  cv::cv2eigen(mat_cv, mat_eigen);
  Q4D quat(mat_eigen);
  quat_eigen = quat.normalized();
}

}  // namespace parameter
