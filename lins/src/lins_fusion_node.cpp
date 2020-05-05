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

#include <ros/ros.h>

#include <parameters.h>
#include <Estimator.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "lins_fusion_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("\033[1;32m---->\033[0m LINS Fusion Started.");

  parameter::readParameters(pnh);

  fusion::LinsFusion lins(nh, pnh);
  lins.run();

  ros::spin();
  return 0;
}
