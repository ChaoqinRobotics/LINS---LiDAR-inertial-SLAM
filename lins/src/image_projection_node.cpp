// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <parameters.h>

using namespace parameter;

class ImageProjection {
 private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  ros::Subscriber subLaserCloud;

  ros::Publisher pubFullCloud;
  ros::Publisher pubFullInfoCloud;

  ros::Publisher pubGroundCloud;
  ros::Publisher pubSegmentedCloud;
  ros::Publisher pubSegmentedCloudPure;
  ros::Publisher pubSegmentedCloudInfo;
  ros::Publisher pubOutlierCloud;

  pcl::PointCloud<PointType>::Ptr laserCloudIn;

  pcl::PointCloud<PointType>::Ptr fullCloud;
  pcl::PointCloud<PointType>::Ptr fullInfoCloud;

  pcl::PointCloud<PointType>::Ptr groundCloud;
  pcl::PointCloud<PointType>::Ptr segmentedCloud;
  pcl::PointCloud<PointType>::Ptr segmentedCloudPure;
  pcl::PointCloud<PointType>::Ptr outlierCloud;

  PointType nanPoint;

  cv::Mat rangeMat;
  cv::Mat labelMat;
  cv::Mat groundMat;
  int labelCount;

  float startOrientation;
  float endOrientation;

  cloud_msgs::cloud_info segMsg;
  std_msgs::Header cloudHeader;

  std::vector<std::pair<uint8_t, uint8_t> > neighborIterator;

  uint16_t* allPushedIndX;
  uint16_t* allPushedIndY;

  uint16_t* queueIndX;
  uint16_t* queueIndY;

 public:
  ImageProjection(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh(nh), pnh(pnh) {
    subLaserCloud = pnh.subscribe<sensor_msgs::PointCloud2>(
        LIDAR_TOPIC, 1, &ImageProjection::cloudHandler, this);

    pubFullCloud =
        pnh.advertise<sensor_msgs::PointCloud2>("/full_cloud_projected", 1);
    pubFullInfoCloud =
        pnh.advertise<sensor_msgs::PointCloud2>("/full_cloud_info", 1);

    pubGroundCloud =
        pnh.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 1);
    pubSegmentedCloud =
        pnh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud", 1);
    pubSegmentedCloudPure =
        pnh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud_pure", 1);
    pubSegmentedCloudInfo =
        pnh.advertise<cloud_msgs::cloud_info>("/segmented_cloud_info", 1);
    pubOutlierCloud =
        pnh.advertise<sensor_msgs::PointCloud2>("/outlier_cloud", 1);

    nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint.intensity = -1;

    allocateMemory();
    resetParameters();
  }

  void allocateMemory() {
    laserCloudIn.reset(new pcl::PointCloud<PointType>());

    fullCloud.reset(new pcl::PointCloud<PointType>());
    fullInfoCloud.reset(new pcl::PointCloud<PointType>());

    groundCloud.reset(new pcl::PointCloud<PointType>());
    segmentedCloud.reset(new pcl::PointCloud<PointType>());
    segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
    outlierCloud.reset(new pcl::PointCloud<PointType>());

    fullCloud->points.resize(LINE_NUM * SCAN_NUM);
    fullInfoCloud->points.resize(LINE_NUM * SCAN_NUM);

    segMsg.startRingIndex.assign(LINE_NUM, 0);
    segMsg.endRingIndex.assign(LINE_NUM, 0);

    segMsg.segmentedCloudGroundFlag.assign(LINE_NUM * SCAN_NUM, false);
    segMsg.segmentedCloudColInd.assign(LINE_NUM * SCAN_NUM, 0);
    segMsg.segmentedCloudRange.assign(LINE_NUM * SCAN_NUM, 0);

    std::pair<int8_t, int8_t> neighbor;
    neighbor.first = -1;
    neighbor.second = 0;
    neighborIterator.push_back(neighbor);
    neighbor.first = 0;
    neighbor.second = 1;
    neighborIterator.push_back(neighbor);
    neighbor.first = 0;
    neighbor.second = -1;
    neighborIterator.push_back(neighbor);
    neighbor.first = 1;
    neighbor.second = 0;
    neighborIterator.push_back(neighbor);

    allPushedIndX = new uint16_t[LINE_NUM * SCAN_NUM];
    allPushedIndY = new uint16_t[LINE_NUM * SCAN_NUM];

    queueIndX = new uint16_t[LINE_NUM * SCAN_NUM];
    queueIndY = new uint16_t[LINE_NUM * SCAN_NUM];
  }

  void resetParameters() {
    laserCloudIn->clear();
    groundCloud->clear();
    segmentedCloud->clear();
    segmentedCloudPure->clear();
    outlierCloud->clear();

    rangeMat = cv::Mat(LINE_NUM, SCAN_NUM, CV_32F, cv::Scalar::all(FLT_MAX));
    groundMat = cv::Mat(LINE_NUM, SCAN_NUM, CV_8S, cv::Scalar::all(0));
    labelMat = cv::Mat(LINE_NUM, SCAN_NUM, CV_32S, cv::Scalar::all(0));
    labelCount = 1;

    std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
    std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(),
              nanPoint);
  }

  ~ImageProjection() {}

  void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) {
    cloudHeader = laserCloudMsg->header;
    pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
  }

  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) {
    TicToc ts_total;
    copyPointCloud(laserCloudMsg);
    findStartEndAngle();
    projectPointCloud();
    groundRemoval();
    cloudSegmentation();
    publishCloud();
    resetParameters();
    double time_total = ts_total.toc();
  }

  void findStartEndAngle() {
    segMsg.startOrientation =
        -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
    segMsg.endOrientation =
        -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
               laserCloudIn->points[laserCloudIn->points.size() - 2].x) +
        2 * M_PI;
    if (segMsg.endOrientation - segMsg.startOrientation > 3 * M_PI) {
      segMsg.endOrientation -= 2 * M_PI;
    } else if (segMsg.endOrientation - segMsg.startOrientation < M_PI)
      segMsg.endOrientation += 2 * M_PI;
    segMsg.orientationDiff = segMsg.endOrientation - segMsg.startOrientation;
  }

  void projectPointCloud() {
    float verticalAngle, horizonAngle, range;
    size_t rowIdn, columnIdn, index, cloudSize;
    PointType thisPoint;

    cloudSize = laserCloudIn->points.size();

    for (size_t i = 0; i < cloudSize; ++i) {
      thisPoint.x = laserCloudIn->points[i].x;
      thisPoint.y = laserCloudIn->points[i].y;
      thisPoint.z = laserCloudIn->points[i].z;

      verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x +
                                              thisPoint.y * thisPoint.y)) *
                      180 / M_PI;
      rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
      if (rowIdn < 0 || rowIdn >= LINE_NUM) continue;

      horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

      columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + SCAN_NUM / 2;
      if (columnIdn >= SCAN_NUM) columnIdn -= SCAN_NUM;

      if (columnIdn < 0 || columnIdn >= SCAN_NUM) continue;

      range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y +
                   thisPoint.z * thisPoint.z);
      rangeMat.at<float>(rowIdn, columnIdn) = range;

      thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

      index = columnIdn + rowIdn * SCAN_NUM;
      fullCloud->points[index] = thisPoint;

      fullInfoCloud->points[index].intensity = range;
    }
  }

  void groundRemoval() {
    size_t lowerInd, upperInd;
    float diffX, diffY, diffZ, angle;

    for (size_t j = 0; j < SCAN_NUM; ++j) {
      for (size_t i = 0; i < groundScanInd; ++i) {
        lowerInd = j + (i)*SCAN_NUM;
        upperInd = j + (i + 1) * SCAN_NUM;

        if (fullCloud->points[lowerInd].intensity == -1 ||
            fullCloud->points[upperInd].intensity == -1) {
          groundMat.at<int8_t>(i, j) = -1;
          continue;
        }

        diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
        diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
        diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

        angle = atan2(diffZ, sqrt(diffX * diffX + diffY * diffY)) * 180 / M_PI;

        if (abs(angle - sensorMountAngle) <= 10) {
          groundMat.at<int8_t>(i, j) = 1;
          groundMat.at<int8_t>(i + 1, j) = 1;
        }
      }
    }

    for (size_t i = 0; i < LINE_NUM; ++i) {
      for (size_t j = 0; j < SCAN_NUM; ++j) {
        if (groundMat.at<int8_t>(i, j) == 1 ||
            rangeMat.at<float>(i, j) == FLT_MAX) {
          labelMat.at<int>(i, j) = -1;
        }
      }
    }
    if (pubGroundCloud.getNumSubscribers() != 0) {
      for (size_t i = 0; i <= groundScanInd; ++i) {
        for (size_t j = 0; j < SCAN_NUM; ++j) {
          if (groundMat.at<int8_t>(i, j) == 1)
            groundCloud->push_back(fullCloud->points[j + i * SCAN_NUM]);
        }
      }
    }
  }

  void cloudSegmentation() {
    for (size_t i = 0; i < LINE_NUM; ++i)
      for (size_t j = 0; j < SCAN_NUM; ++j)
        if (labelMat.at<int>(i, j) == 0) labelComponents(i, j);

    int sizeOfSegCloud = 0;
    for (size_t i = 0; i < LINE_NUM; ++i) {
      segMsg.startRingIndex[i] = sizeOfSegCloud - 1 + 5;

      for (size_t j = 0; j < SCAN_NUM; ++j) {
        if (labelMat.at<int>(i, j) > 0 || groundMat.at<int8_t>(i, j) == 1) {
          if (labelMat.at<int>(i, j) == 999999) {
            if (i > groundScanInd && j % 5 == 0) {
              outlierCloud->push_back(fullCloud->points[j + i * SCAN_NUM]);
              continue;
            } else {
              continue;
            }
          }
          if (groundMat.at<int8_t>(i, j) == 1) {
            if (j % 5 != 0 && j > 5 && j < SCAN_NUM - 5) continue;
          }
          segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] =
              (groundMat.at<int8_t>(i, j) == 1);
          segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
          segMsg.segmentedCloudRange[sizeOfSegCloud] = rangeMat.at<float>(i, j);
          segmentedCloud->push_back(fullCloud->points[j + i * SCAN_NUM]);
          ++sizeOfSegCloud;
        }
      }

      segMsg.endRingIndex[i] = sizeOfSegCloud - 1 - 5;
    }

    if (pubSegmentedCloudPure.getNumSubscribers() != 0) {
      for (size_t i = 0; i < LINE_NUM; ++i) {
        for (size_t j = 0; j < SCAN_NUM; ++j) {
          if (labelMat.at<int>(i, j) > 0 && labelMat.at<int>(i, j) != 999999) {
            segmentedCloudPure->push_back(fullCloud->points[j + i * SCAN_NUM]);
            segmentedCloudPure->points.back().intensity =
                labelMat.at<int>(i, j);
          }
        }
      }
    }
  }

  void labelComponents(int row, int col) {
    float d1, d2, alpha, angle;
    int fromIndX, fromIndY, thisIndX, thisIndY;
    bool lineCountFlag[LINE_NUM] = {false};

    queueIndX[0] = row;
    queueIndY[0] = col;
    int queueSize = 1;
    int queueStartInd = 0;
    int queueEndInd = 1;

    allPushedIndX[0] = row;
    allPushedIndY[0] = col;
    int allPushedIndSize = 1;

    while (queueSize > 0) {
      fromIndX = queueIndX[queueStartInd];
      fromIndY = queueIndY[queueStartInd];
      --queueSize;
      ++queueStartInd;
      labelMat.at<int>(fromIndX, fromIndY) = labelCount;

      for (auto iter = neighborIterator.begin(); iter != neighborIterator.end();
           ++iter) {
        thisIndX = fromIndX + (*iter).first;
        thisIndY = fromIndY + (*iter).second;

        if (thisIndX < 0 || thisIndX >= LINE_NUM) continue;

        if (thisIndY < 0) thisIndY = SCAN_NUM - 1;
        if (thisIndY >= SCAN_NUM) thisIndY = 0;

        if (labelMat.at<int>(thisIndX, thisIndY) != 0) continue;

        d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY),
                      rangeMat.at<float>(thisIndX, thisIndY));
        d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY),
                      rangeMat.at<float>(thisIndX, thisIndY));

        if ((*iter).first == 0)
          alpha = segmentAlphaX;
        else
          alpha = segmentAlphaY;

        angle = atan2(d2 * sin(alpha), (d1 - d2 * cos(alpha)));

        if (angle > segmentTheta) {
          queueIndX[queueEndInd] = thisIndX;
          queueIndY[queueEndInd] = thisIndY;
          ++queueSize;
          ++queueEndInd;

          labelMat.at<int>(thisIndX, thisIndY) = labelCount;
          lineCountFlag[thisIndX] = true;

          allPushedIndX[allPushedIndSize] = thisIndX;
          allPushedIndY[allPushedIndSize] = thisIndY;
          ++allPushedIndSize;
        }
      }
    }

    bool feasibleSegment = false;
    if (allPushedIndSize >= 30)
      feasibleSegment = true;
    else if (allPushedIndSize >= segmentValidPointNum) {
      int lineCount = 0;
      for (size_t i = 0; i < LINE_NUM; ++i)
        if (lineCountFlag[i] == true) ++lineCount;
      if (lineCount >= segmentValidLineNum) feasibleSegment = true;
    }

    if (feasibleSegment == true) {
      ++labelCount;
    } else {
      for (size_t i = 0; i < allPushedIndSize; ++i) {
        labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
      }
    }
  }

  void publishCloud() {
    segMsg.header = cloudHeader;
    pubSegmentedCloudInfo.publish(segMsg);

    sensor_msgs::PointCloud2 laserCloudTemp;

    pcl::toROSMsg(*outlierCloud, laserCloudTemp);
    laserCloudTemp.header.stamp = cloudHeader.stamp;
    laserCloudTemp.header.frame_id = "base_link";
    pubOutlierCloud.publish(laserCloudTemp);

    pcl::toROSMsg(*segmentedCloud, laserCloudTemp);
    laserCloudTemp.header.stamp = cloudHeader.stamp;
    laserCloudTemp.header.frame_id = "base_link";
    pubSegmentedCloud.publish(laserCloudTemp);

    if (pubFullCloud.getNumSubscribers() != 0) {
      pcl::toROSMsg(*fullCloud, laserCloudTemp);
      laserCloudTemp.header.stamp = cloudHeader.stamp;
      laserCloudTemp.header.frame_id = "base_link";
      pubFullCloud.publish(laserCloudTemp);
    }

    if (pubGroundCloud.getNumSubscribers() != 0) {
      pcl::toROSMsg(*groundCloud, laserCloudTemp);
      laserCloudTemp.header.stamp = cloudHeader.stamp;
      laserCloudTemp.header.frame_id = "base_link";
      pubGroundCloud.publish(laserCloudTemp);
    }

    if (pubSegmentedCloudPure.getNumSubscribers() != 0) {
      pcl::toROSMsg(*segmentedCloudPure, laserCloudTemp);
      laserCloudTemp.header.stamp = cloudHeader.stamp;
      laserCloudTemp.header.frame_id = "base_link";
      pubSegmentedCloudPure.publish(laserCloudTemp);
    }

    if (pubFullInfoCloud.getNumSubscribers() != 0) {
      pcl::toROSMsg(*fullInfoCloud, laserCloudTemp);
      laserCloudTemp.header.stamp = cloudHeader.stamp;
      laserCloudTemp.header.frame_id = "base_link";
      pubFullInfoCloud.publish(laserCloudTemp);
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_projection_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");


  parameter::readParameters(pnh);


  ImageProjection featureHandler(nh, pnh);

  ROS_INFO("\033[1;32m---->\033[0m Feature Extraction Module Started.");

  ros::spin();
  return 0;
}
