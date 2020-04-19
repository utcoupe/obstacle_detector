/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include "processing_lidar_objects/utilities/point.h"

#include <cmath>

namespace processing_lidar_objects
{

inline double signum(double x) { return (x < 0.0) ? -1.0 : 1.0; }

inline double length(const geometry_msgs::msg::Point& point) {
  return std::hypot(point.x, point.y);
}

inline double squaredLength(const geometry_msgs::msg::Point& point) {
  return point.x * point.x + point.y * point.y;
}

inline double length(const geometry_msgs::msg::Vector3& vec) {
  return std::hypot(vec.x, vec.y);
}

inline double squaredLength(const geometry_msgs::msg::Vector3& vec) {
  return vec.x * vec.x + vec.y * vec.y;
}

template<class PointType>
inline PointType transformPoint(const PointType& point, double x, double y, double theta) {
  PointType p;

  p.x = point.x * cos(theta) - point.y * sin(theta) + x;
  p.y = point.x * sin(theta) + point.y * cos(theta) + y;

  return p;
}

inline bool checkPointInLimits(const geometry_msgs::msg::Point32& p, double x_min, double x_max, double y_min, double y_max) {
  if ((p.x > x_max) || (p.x < x_min) || (p.y > y_max) || (p.y < y_min))
    return false;
  else
    return true;
}

} // namespace processing_lidar_objects
