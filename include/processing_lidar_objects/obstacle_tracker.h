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

#pragma once

#include <list>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <armadillo>
#include <std_srvs/srv/empty.hpp>

#include <processing_lidar_objects/msg/obstacles.hpp>

#include "processing_lidar_objects/utilities/tracked_obstacle.h"
#include "processing_lidar_objects/utilities/math_utilities.h"

namespace processing_lidar_objects
{

class ObstacleTracker : public rclcpp::Node {
public:
  ObstacleTracker(
    std::string node_name,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions()
  );
  ~ObstacleTracker();

private:
  void updateParamsCallback(const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res);
  void timerCallback();
  void obstaclesCallback(const processing_lidar_objects::msg::Obstacles::SharedPtr new_obstacles);

  void initialize();
  void updateParams();

  double obstacleCostFunction(const msg::CircleObstacle& new_obstacle, const msg::CircleObstacle& old_obstacle);
  void calculateCostMatrix(const std::vector<msg::CircleObstacle>& new_obstacles, arma::mat& cost_matrix);
  void calculateRowMinIndices(const arma::mat& cost_matrix, std::vector<int>& row_min_indices);
  void calculateColMinIndices(const arma::mat& cost_matrix, std::vector<int>& col_min_indices);

  bool fusionObstacleUsed(const int idx, const std::vector<int>& col_min_indices, const std::vector<int>& used_new, const std::vector<int>& used_old);
  bool fusionObstaclesCorrespond(const int idx, const int jdx, const std::vector<int>& col_min_indices, const std::vector<int>& used_old);
  bool fissionObstacleUsed(const int idx, const int T, const std::vector<int>& row_min_indices, const std::vector<int>& used_new, const std::vector<int>& used_old);
  bool fissionObstaclesCorrespond(const int idx, const int jdx, const std::vector<int>& row_min_indices, const std::vector<int>& used_new);

  void fuseObstacles(const std::vector<int>& fusion_indices, const std::vector<int>& col_min_indices,
                     std::vector<TrackedObstacle>& new_tracked, const msg::Obstacles::SharedPtr& new_obstacles);
  void fissureObstacle(const std::vector<int>& fission_indices, const std::vector<int>& row_min_indices,
                       std::vector<TrackedObstacle>& new_tracked, const msg::Obstacles::SharedPtr& new_obstacles);

  void updateObstacles();
  void publishObstacles();

  rclcpp::Subscription<processing_lidar_objects::msg::Obstacles>::SharedPtr obstacles_sub_;
  rclcpp::Publisher<processing_lidar_objects::msg::Obstacles>::SharedPtr obstacles_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr params_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  double radius_margin_;
  processing_lidar_objects::msg::Obstacles obstacles_;

  std::vector<TrackedObstacle> tracked_obstacles_;
  std::vector<msg::CircleObstacle> untracked_obstacles_;

  // Parameters
  bool p_active_;
  bool p_copy_segments_;

  double p_tracking_duration_;
  double p_loop_rate_;
  double p_sampling_time_;
  double p_sensor_rate_;
  double p_min_correspondence_cost_;
  double p_std_correspondence_dev_;
  double p_process_variance_;
  double p_process_rate_variance_;
  double p_measurement_variance_;

  std::string p_frame_id_;
};

} // namespace processing_lidar_objects
