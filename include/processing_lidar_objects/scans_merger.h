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

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2_ros/transform_listener.h>
#include <laser_geometry/laser_geometry.hpp>

#include <memory>

namespace processing_lidar_objects
{

class ScansMerger
{
public:
  ScansMerger(rclcpp::Node::SharedPtr& rootNode, rclcpp::Node::SharedPtr& localNode);
  ~ScansMerger();

private:
  void updateParamsCallback(const std_srvs::srv::Empty::Request::SharedPtr req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
  void updateParams();
  void frontScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr front_scan);
  void rearScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr rear_scan);
  void handleScanUpdate(const sensor_msgs::msg::LaserScan::SharedPtr& received_scan, sensor_msgs::msg::PointCloud2& relatedPcl);
  bool copyPointsFromPointCloud(
    std::vector<float>& ranges,
    std::vector<geometry_msgs::msg::Point32>& points,
    const sensor_msgs::msg::PointCloud2& pcl_to_copy,
    const rclcpp::Time& refTimePoint
  );

  void initialize();

  void publishMessages();

  rclcpp::Node::SharedPtr node_root_;
  rclcpp::Node::SharedPtr node_local_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr params_srv_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr front_scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr rear_scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pcl_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_ls_;
  laser_geometry::LaserProjection projector_;

  bool front_scan_received_;
  bool rear_scan_received_;
  bool front_scan_error_;
  bool rear_scan_error_;

  sensor_msgs::msg::PointCloud2 front_pcl_;
  sensor_msgs::msg::PointCloud2 rear_pcl_;

  // Parameters
  bool p_active_;
  bool p_publish_scan_;
  bool p_publish_pcl_;

  int p_ranges_num_;

  double p_min_scanner_range_;
  double p_max_scanner_range_;
  double p_min_x_range_;
  double p_max_x_range_;
  double p_min_y_range_;
  double p_max_y_range_;

  std::string p_fixed_frame_id_;
  std::string p_target_frame_id_;
};

} // namespace processing_lidar_objects
