/*
* Copyright (c) 2014, Autonomous Systems Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef INCLUDE_MAPRINGBUFFER_H_
#define INCLUDE_MAPRINGBUFFER_H_

#include <iostream>
#include <map>

template <typename Meas>
class MapRingBuffer {
 public:
  std::map<double, Meas> measMap_;
  typename std::map<double, Meas>::iterator itMeas_;

  int size;
  double maxWaitTime_;
  double minWaitTime_;

  MapRingBuffer() {
    maxWaitTime_ = 0.1;
    minWaitTime_ = 0.0;
  }

  virtual ~MapRingBuffer() {}

  bool allocate(const int sizeBuffer) {
    if (sizeBuffer <= 0) {
      return false;
    } else {
      size = sizeBuffer;
      return true;
    }
  }

  int getSize() { return measMap_.size(); }

  void addMeas(const Meas& meas, const double& t) {
    measMap_.insert(std::make_pair(t, meas));

    // ensure the size of the map, and remove the last element
    if (measMap_.size() > size) {
      measMap_.erase(measMap_.begin());
    }
  }

  void clear() { measMap_.clear(); }

  void clean(double t) {
    while (measMap_.size() >= 1 && measMap_.begin()->first <= t) {
      measMap_.erase(measMap_.begin());
    }
  }

  bool getNextTime(double actualTime, double& nextTime) {
    itMeas_ = measMap_.upper_bound(actualTime);
    if (itMeas_ != measMap_.end()) {
      nextTime = itMeas_->first;
      return true;
    } else {
      return false;
    }
  }
  void waitTime(double actualTime, double& time) {
    double measurementTime = actualTime - maxWaitTime_;
    if (!measMap_.empty() &&
        measMap_.rbegin()->first + minWaitTime_ > measurementTime) {
      measurementTime = measMap_.rbegin()->first + minWaitTime_;
    }
    if (time > measurementTime) {
      time = measurementTime;
    }
  }
  bool getLastTime(double& lastTime) {
    if (!measMap_.empty()) {
      lastTime = measMap_.rbegin()->first;
      return true;
    } else {
      return false;
    }
  }

  bool getFirstTime(double& firstTime) {
    if (!measMap_.empty()) {
      firstTime = measMap_.begin()->first;
      return true;
    } else {
      return false;
    }
  }

  bool getLastMeas(Meas& lastMeas) {
    if (!measMap_.empty()) {
      lastMeas = measMap_.rbegin()->second;
      return true;
    } else {
      return false;
    }
  }

  bool getLastLastMeas(Meas& lastlastMeas) {
    if (measMap_.size() >= 2) {
      auto itr = measMap_.rbegin();
      itr++;
      lastlastMeas = itr->second;
      return true;
    } else {
      return false;
    }
  }

  bool getFirstMeas(Meas& firstMeas) {
    if (!measMap_.empty()) {
      firstMeas = measMap_.begin()->second;
      return true;
    } else {
      return false;
    }
  }

  bool hasMeasurementAt(double t) { return measMap_.count(t) > 0; }

  bool empty() { return measMap_.empty(); }

  void printContainer() {
    itMeas_ = measMap_.begin();
    while (measMap_.size() >= 1 && itMeas_ != measMap_.end()) {
      std::cout << itMeas_->second << " ";
      itMeas_++;
    }
  }
};

#endif  // INCLUDE_MAPRINGBUFFER_H_
