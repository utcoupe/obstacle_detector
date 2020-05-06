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
 * Authors: Mateusz Przybyla, Gaëtan Blond
 */

#include "processing_lidar_objects/scans_merger.h"

#include <sensor_msgs/point_cloud_conversion.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <chrono>
#include <cmath>
#include <functional>

using namespace processing_lidar_objects;
using namespace std;
using namespace std::placeholders;

const auto TIMEOUT_TRANSFORM = tf2::durationFromSec(0.05);
const auto DEFAULT_NODE_NAME = "scans_merger";

ScansMerger::ScansMerger(const rclcpp::NodeOptions & node_options):
  ScansMerger(DEFAULT_NODE_NAME, node_options)
{}

ScansMerger::ScansMerger(
    std::string node_name,
    const rclcpp::NodeOptions & node_options
):
  Node(node_name, node_options),
  tf_buffer_(this->get_clock()),
  tf_ls_(tf_buffer_)
{
  p_active_ = false;

  front_scan_received_ = false;
  rear_scan_received_ = false;

  front_scan_error_ = true;
  rear_scan_error_ = true;

  params_srv_ = this->create_service<std_srvs::srv::Empty>(
    "~/params",
    std::bind(&processing_lidar_objects::ScansMerger::updateParamsCallback, this, _1, _2)
  );

  initialize();
}

ScansMerger::~ScansMerger() {
  this->undeclare_parameter("active");
  this->undeclare_parameter("publish_scan");
  this->undeclare_parameter("publish_pcl");

  this->undeclare_parameter("ranges_num");

  this->undeclare_parameter("min_scanner_range");
  this->undeclare_parameter("max_scanner_range");

  this->undeclare_parameter("min_x_range");
  this->undeclare_parameter("max_x_range");
  this->undeclare_parameter("min_y_range");
  this->undeclare_parameter("max_y_range");

  this->undeclare_parameter("fixed_frame_id");
  this->undeclare_parameter("target_frame_id");
}

void ScansMerger::initialize()
{

  this->declare_parameter("active", true);
  this->declare_parameter("publish_scan", false);
  this->declare_parameter("publish_pcl", true);

  this->declare_parameter("ranges_num", 1000);

  this->declare_parameter("min_scanner_range", 0.05);
  this->declare_parameter("max_scanner_range", 10.0);

  this->declare_parameter("min_x_range", -10.0);
  this->declare_parameter("max_x_range",  10.0);
  this->declare_parameter("min_y_range", -10.0);
  this->declare_parameter("max_y_range",  10.0);

  this->declare_parameter("fixed_frame_id", "map");
  this->declare_parameter("target_frame_id", "robot");

  updateParams();
}

void ScansMerger::updateParamsCallback(
  [[maybe_unused]] const std_srvs::srv::Empty::Request::SharedPtr req,
  [[maybe_unused]] std_srvs::srv::Empty::Response::SharedPtr res
) {
  updateParams();
}

void ScansMerger::updateParams() {
  bool prev_active = p_active_;

  p_active_ = this->get_parameter("active").get_value<bool>();
  p_publish_scan_ = this->get_parameter("publish_scan").get_value<bool>();
  p_publish_pcl_ = this->get_parameter("publish_pcl").get_value<bool>();

  p_ranges_num_ = this->get_parameter("ranges_num").get_value<int>();

  p_min_scanner_range_ = this->get_parameter("min_scanner_range").get_value<double>();
  p_max_scanner_range_ = this->get_parameter("max_scanner_range").get_value<double>();

  p_min_x_range_ = this->get_parameter("min_x_range").get_value<double>();
  p_max_x_range_ = this->get_parameter("max_x_range").get_value<double>();
  p_min_y_range_ = this->get_parameter("min_y_range").get_value<double>();
  p_max_y_range_ = this->get_parameter("max_y_range").get_value<double>();

  p_fixed_frame_id_ = this->get_parameter("fixed_frame_id").get_value<string>();
  p_target_frame_id_ = this->get_parameter("target_frame_id").get_value<string>();

  if (p_active_ != prev_active) {
    if (p_active_) {
      front_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10,
        std::bind(&ScansMerger::frontScanCallback, this, _1)
      );
      rear_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "rear_scan",
        10,
        std::bind(&ScansMerger::rearScanCallback, this, _1)
      );
      scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan_transformed", 10);
      pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>("pcl", 10);
    }
    else {
      front_scan_sub_ = nullptr;
      rear_scan_sub_ = nullptr;
      scan_pub_ = nullptr;
      pcl_pub_ = nullptr;
    }
  }
}

void ScansMerger::frontScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr front_scan) {
  try {
    handleScanUpdate(front_scan, front_pcl_);
  }
  catch (tf2::LookupException& ex) {
    // TODO catch tf2::ExtrapolationException ?
    front_scan_error_ = true;
    return;
  }

  front_scan_received_ = true;
  front_scan_error_ = false;

  if (rear_scan_received_ || rear_scan_error_)
    publishMessages();
  else
    rear_scan_error_ = true;
}

void ScansMerger::rearScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr rear_scan) {
  try {
    handleScanUpdate(rear_scan, rear_pcl_);
  }
  catch (tf2::LookupException& ex) {
    // TODO catch tf2::ExtrapolationException ?
    rear_scan_error_ = true;
    return;
  }

  rear_scan_received_ = true;
  rear_scan_error_ = false;

  if (front_scan_received_ || front_scan_error_)
    publishMessages();
  else
    front_scan_error_ = true;
}

void ScansMerger::handleScanUpdate(const sensor_msgs::msg::LaserScan::SharedPtr& received_scan, sensor_msgs::msg::PointCloud2& relatedPcl) {
  auto deltaTime = tf2::durationFromSec(static_cast<double>(received_scan->ranges.size()) * static_cast<double>(received_scan->time_increment));
  auto newTimePoint = tf2::TimePoint(
    std::chrono::seconds(received_scan->header.stamp.sec)
    + std::chrono::nanoseconds(received_scan->header.stamp.nanosec)
    + deltaTime
  );
  tf_buffer_.lookupTransform(received_scan->header.frame_id, p_fixed_frame_id_,
                              newTimePoint, TIMEOUT_TRANSFORM);
  projector_.transformLaserScanToPointCloud(p_fixed_frame_id_, *received_scan, relatedPcl, tf_buffer_);
}

bool ScansMerger::copyPointsFromPointCloud(
  std::vector<float>& ranges,
  std::vector<geometry_msgs::msg::Point32>& points,
  const sensor_msgs::msg::PointCloud2& pcl_to_copy,
  const rclcpp::Time& refTimePoint
) {

  sensor_msgs::msg::PointCloud new_pcl;
  try {
    auto transform = tf_buffer_.lookupTransform(p_target_frame_id_, pcl_to_copy.header.frame_id, tf2_ros::fromRclcpp(refTimePoint), TIMEOUT_TRANSFORM);

    sensor_msgs::msg::PointCloud2 new_pcl2;
    tf2::doTransform(pcl_to_copy, new_pcl2, transform);
    sensor_msgs::convertPointCloud2ToPointCloud(new_pcl2, new_pcl);
  }
  catch (tf2::LookupException& ex) {
    // TODO catch tf2::ExtrapolationException ?
    return false;
  }

  for (auto& point : new_pcl.points) {
    if (point.x > p_min_x_range_ && point.x < p_max_x_range_ &&
        point.y > p_min_y_range_ && point.y < p_max_y_range_) {

      double range = hypot(point.x, point.y);

      if (range > p_min_scanner_range_ && range < p_max_scanner_range_) {
        if (p_publish_pcl_) {
          points.push_back(point);
        }

        if (p_publish_scan_) {
          double angle = atan2(point.y, point.x);

          auto idx = static_cast<size_t>(p_ranges_num_ * (angle + M_PI) / (2.0 * M_PI));
          if (isnan(ranges[idx]) || range < ranges[idx])
            ranges[idx] = range;
        }
      }
    }
  }
  return true;
}

void ScansMerger::publishMessages() {
  auto now = this->now();

  vector<float> ranges;
  vector<geometry_msgs::msg::Point32> points;

  ranges.assign(p_ranges_num_, nanf("")); // Assign nan values

  if (!front_scan_error_) {
    if (!copyPointsFromPointCloud(ranges, points, front_pcl_, now)) {
      return;
    }
  }

  if (!rear_scan_error_) {
    if (!copyPointsFromPointCloud(ranges, points, rear_pcl_, now)) {
      return;
    }
  }

  if (p_publish_scan_ && scan_pub_) {
    sensor_msgs::msg::LaserScan scan_msg{};

    scan_msg.header.frame_id = p_target_frame_id_;
    scan_msg.header.stamp = now;
    scan_msg.angle_min = -M_PI;
    scan_msg.angle_max = M_PI;
    scan_msg.angle_increment = 2.0 * M_PI / (p_ranges_num_ - 1);
    scan_msg.time_increment = 0.0;
    scan_msg.scan_time = 0.1;
    scan_msg.range_min = p_min_scanner_range_;
    scan_msg.range_max = p_max_scanner_range_;
    scan_msg.ranges.assign(ranges.begin(), ranges.end());

    scan_pub_->publish(scan_msg);
  }

  if (p_publish_pcl_ && pcl_pub_) {
    sensor_msgs::msg::PointCloud pcl_msg{};

    pcl_msg.header.frame_id = p_target_frame_id_;
    pcl_msg.header.stamp = now;
    pcl_msg.points.assign(points.begin(), points.end());

    pcl_pub_->publish(pcl_msg);
  }

  front_scan_received_ = false;
  rear_scan_received_ = false;
}

#include <rclcpp_components/register_node_macro.hpp>

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(processing_lidar_objects::ScansMerger)
