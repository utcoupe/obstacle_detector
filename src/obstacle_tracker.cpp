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

#include "processing_lidar_objects/obstacle_tracker.h"

#include <chrono>
#include <functional>

using namespace processing_lidar_objects;
using namespace arma;
using namespace std;
using namespace std::placeholders;
using namespace std::chrono_literals;

const auto UPDATE_TIMER_DURATION {1s};
const auto HOKUYO_SENSOR_RATE_HZ {10.0};

ObstacleTracker::ObstacleTracker(
    std::string node_name,
    const rclcpp::NodeOptions & node_options
):
  Node(node_name, node_options)
{
  p_active_ = false;

  params_srv_ = this->create_service<std_srvs::srv::Empty>(
    "~/params",
    std::bind(&ObstacleTracker::updateParamsCallback, this, _1, _2)
  );

  initialize();
}

ObstacleTracker::~ObstacleTracker() {
  this->undeclare_parameter("~/active");
  this->undeclare_parameter("~/copy_segments");

  this->undeclare_parameter("~/loop_rate");

  this->undeclare_parameter("~/tracking_duration");
  this->undeclare_parameter("~/min_correspondence_cost");
  this->undeclare_parameter("~/std_correspondence_dev");
  this->undeclare_parameter("~/process_variance");
  this->undeclare_parameter("~/process_rate_variance");
  this->undeclare_parameter("~/measurement_variance");

  this->undeclare_parameter("~/frame_id");
}

void ObstacleTracker::initialize() {
  this->declare_parameter("~/active", true);
  this->declare_parameter("~/copy_segments", true);

  this->declare_parameter("~/loop_rate", 100.0);

  this->declare_parameter("~/tracking_duration", 2.0);
  this->declare_parameter("~/min_correspondence_cost", 0.3);
  this->declare_parameter("~/std_correspondence_dev", 0.15);
  this->declare_parameter("~/process_variance", 0.01);
  this->declare_parameter("~/process_rate_variance", 0.1);
  this->declare_parameter("~/measurement_variance", 1.0);

  this->declare_parameter("~/frame_id", "map");

  updateParams();
}

void ObstacleTracker::updateParamsCallback(
  [[maybe_unused]] const std_srvs::srv::Empty::Request::SharedPtr req,
  [[maybe_unused]] std_srvs::srv::Empty::Response::SharedPtr res
) {
  updateParams();
}

void ObstacleTracker::updateParams() {
  bool prev_active = p_active_;
  auto prev_sampling_time = p_sampling_time_;

  p_active_ = this->get_parameter("~/active").get_value<bool>();
  p_copy_segments_ = this->get_parameter("~/copy_segments").get_value<bool>();

  p_loop_rate_ = this->get_parameter("~/loop_rate").get_value<double>();
  p_sampling_time_ = 1.0 / p_loop_rate_;
  p_sensor_rate_ = HOKUYO_SENSOR_RATE_HZ;

  p_tracking_duration_ = this->get_parameter("~/tracking_duration").get_value<double>();
  p_min_correspondence_cost_ = this->get_parameter("~/min_correspondence_cost").get_value<double>();
  p_std_correspondence_dev_ = this->get_parameter("~/std_correspondence_dev").get_value<double>();
  p_process_variance_ = this->get_parameter("~/process_variance").get_value<double>();
  p_process_rate_variance_ = this->get_parameter("~/process_rate_variance").get_value<double>();
  p_measurement_variance_ = this->get_parameter("~/measurement_variance").get_value<double>();

  p_frame_id_ = this->get_parameter("~/frame_id").get_value<string>();
  obstacles_.header.frame_id = p_frame_id_;

  TrackedObstacle::setSamplingTime(p_sampling_time_);
  TrackedObstacle::setCounterSize(static_cast<int>(p_loop_rate_ * p_tracking_duration_));
  TrackedObstacle::setCovariances(p_process_variance_, p_process_rate_variance_, p_measurement_variance_);

  if (p_active_ != prev_active) {
    if (p_active_) {
      obstacles_sub_ = this->create_subscription<msg::Obstacles>(
        "raw_obstacles",
        10,
        std::bind(&ObstacleTracker::obstaclesCallback, this, _1)
      );
      obstacles_pub_ = this->create_publisher<msg::Obstacles>("tracked_obstacles", 10);
    }
    else {
      // FIXME can lead to data race with timerCallback and obstaclesCallback

      // Send empty message
      msg::Obstacles obstacles_msg {};
      obstacles_msg.header.frame_id = obstacles_.header.frame_id;
      obstacles_msg.header.stamp = this->now();
      obstacles_pub_->publish(obstacles_msg);

      obstacles_sub_ = nullptr;
      obstacles_pub_ = nullptr;

      tracked_obstacles_.clear();
      untracked_obstacles_.clear();
    }
  }

  if (p_active_ && (p_active_ != prev_active || p_sampling_time_ != prev_sampling_time)) {
    // FIXME may create undefined behavior (timer destroyed while it could be calling ObstacleTracker::timerCallback)
    // use mutex lock ?
    timer_ = this->create_wall_timer(
      rclcpp::Duration::from_seconds(p_sampling_time_).to_chrono<std::chrono::nanoseconds>(), // TODO find cleaner way to cast from double
      std::bind(&ObstacleTracker::timerCallback, this)
    );
  } else if (!p_active_ && timer_) {
    timer_->cancel(); // Useful ?
    timer_ = nullptr;
  }
}

void ObstacleTracker::timerCallback() {
  // FIXME can lead to data race with updateParams and obstaclesCallback
  updateObstacles();
  publishObstacles();
}

void ObstacleTracker::obstaclesCallback(const msg::Obstacles::SharedPtr new_obstacles) {
  // FIXME can lead to data race with updateParams and timerCallback
  if (new_obstacles->circles.size() > 0)
    radius_margin_ = new_obstacles->circles[0].radius - new_obstacles->circles[0].true_radius;

  if (p_copy_segments_) {
    obstacles_.segments.clear();
    obstacles_.segments.assign(new_obstacles->segments.begin(), new_obstacles->segments.end());
  }

  int N = new_obstacles->circles.size();
  int T = tracked_obstacles_.size();
  int U = untracked_obstacles_.size();

  if (T + U == 0) {
    untracked_obstacles_.assign(new_obstacles->circles.begin(), new_obstacles->circles.end());
    return;
  }

  mat cost_matrix;
  calculateCostMatrix(new_obstacles->circles, cost_matrix);

  vector<int> row_min_indices;
  calculateRowMinIndices(cost_matrix, row_min_indices);

  vector<int> col_min_indices;
  calculateColMinIndices(cost_matrix, col_min_indices);

  vector<int> used_old_obstacles;
  vector<int> used_new_obstacles;

  vector<TrackedObstacle> new_tracked_obstacles;
  vector<msg::CircleObstacle> new_untracked_obstacles;

  // Check for fusion (only tracked obstacles)
  for (int i = 0; i < T-1; ++i) {
    if (fusionObstacleUsed(i, col_min_indices, used_new_obstacles, used_old_obstacles))
      continue;

    vector<int> fusion_indices;
    fusion_indices.push_back(i);

    for (int j = i+1; j < T; ++j) {
      if (fusionObstaclesCorrespond(i, j, col_min_indices, used_old_obstacles))
        fusion_indices.push_back(j);
    }

    if (fusion_indices.size() > 1) {
      fuseObstacles(fusion_indices, col_min_indices, new_tracked_obstacles, new_obstacles);

      // Mark used old and new obstacles
      used_old_obstacles.insert(used_old_obstacles.end(), fusion_indices.begin(), fusion_indices.end());
      used_new_obstacles.push_back(col_min_indices[i]);
    }
  }

  // Check for fission (only tracked obstacles)
  for (int i = 0; i < N-1; ++i) {
    if (fissionObstacleUsed(i, T, row_min_indices, used_new_obstacles, used_old_obstacles))
      continue;

    vector<int> fission_indices;
    fission_indices.push_back(i);

    for (int j = i+1; j < N; ++j) {
      if (fissionObstaclesCorrespond(i, j, row_min_indices, used_new_obstacles))
        fission_indices.push_back(j);
    }

    if (fission_indices.size() > 1) {
      fissureObstacle(fission_indices, row_min_indices, new_tracked_obstacles, new_obstacles);

      // Mark used old and new obstacles
      used_old_obstacles.push_back(row_min_indices[i]);
      used_new_obstacles.insert(used_new_obstacles.end(), fission_indices.begin(), fission_indices.end());
    }
  }

  // Check for other possibilities
  for (int n = 0; n < N; ++n) {
    if (find(used_new_obstacles.begin(), used_new_obstacles.end(), n) != used_new_obstacles.end())
      continue;

    if (row_min_indices[n] == -1) {
      new_untracked_obstacles.push_back(new_obstacles->circles[n]);
    }
    else if (find(used_old_obstacles.begin(), used_old_obstacles.end(), row_min_indices[n]) == used_old_obstacles.end()) {
      if (row_min_indices[n] >= 0 && row_min_indices[n] < T) {
        tracked_obstacles_[row_min_indices[n]].correctState(new_obstacles->circles[n]);
      }
      else if (row_min_indices[n] >= T) {
        TrackedObstacle to(untracked_obstacles_[row_min_indices[n] - T]);
        to.correctState(new_obstacles->circles[n]);
        for (int i = 0; i < static_cast<int>(p_loop_rate_ / p_sensor_rate_); ++i)
          to.updateState();

        new_tracked_obstacles.push_back(to);
      }

      used_new_obstacles.push_back(n);
    }
  }

  // Remove tracked obstacles that are no longer existent due to fusion or fission and insert new ones
  // Sort in descending order to remove from back of the list
  sort(used_old_obstacles.rbegin(), used_old_obstacles.rend());
  for (int idx : used_old_obstacles)
    tracked_obstacles_.erase(tracked_obstacles_.begin() + idx);

  tracked_obstacles_.insert(tracked_obstacles_.end(), new_tracked_obstacles.begin(), new_tracked_obstacles.end());

  // Remove old untracked obstacles and save new ones
  untracked_obstacles_.clear();
  untracked_obstacles_.assign(new_untracked_obstacles.begin(), new_untracked_obstacles.end());
}

double ObstacleTracker::obstacleCostFunction(const msg::CircleObstacle& new_obstacle, const msg::CircleObstacle& old_obstacle) {
  mat distribution = mat(2, 2).zeros();
  vec relative_position = vec(2).zeros();

  double cost = 0.0;
  double penalty = 1.0;
  double tp = 1.0 / p_sensor_rate_;

  double direction = atan2(old_obstacle.velocity.y, old_obstacle.velocity.x);

  geometry_msgs::msg::Point new_center = transformPoint(new_obstacle.center, 0.0, 0.0, -direction);
  geometry_msgs::msg::Point old_center = transformPoint(old_obstacle.center, 0.0, 0.0, -direction);

  distribution(0, 0) = pow(p_std_correspondence_dev_, 2.0) + squaredLength(old_obstacle.velocity) * pow(tp, 2.0);
  distribution(1, 1) = pow(p_std_correspondence_dev_, 2.0);

  relative_position(0) = new_center.x - old_center.x - tp * length(old_obstacle.velocity);
  relative_position(1) = new_center.y - old_center.y;

  cost = sqrt(pow(new_obstacle.center.x - old_obstacle.center.x, 2.0) + pow(new_obstacle.center.y - old_obstacle.center.y, 2.0) + pow(new_obstacle.radius - old_obstacle.radius, 2.0));

  mat a = -0.5 * trans(relative_position) * distribution * relative_position;
  penalty = exp(a(0, 0));

  // TODO: Check values for cost/penalty in common situations
  // return cost / penalty;
  return cost / 1.0;
}

void ObstacleTracker::calculateCostMatrix(const vector<msg::CircleObstacle>& new_obstacles, mat& cost_matrix) {
  /*
   * Cost between two obstacles represents their difference.
   * The bigger the cost, the less similar they are.
   * N rows of cost_matrix represent new obstacles.
   * T+U columns of cost matrix represent old tracked and untracked obstacles.
   */
  int N = new_obstacles.size();
  int T = tracked_obstacles_.size();
  int U = untracked_obstacles_.size();

  cost_matrix = mat(N, T + U, fill::zeros);

  for (int n = 0; n < N; ++n) {
    for (int t = 0; t < T; ++t)
      cost_matrix(n, t) = obstacleCostFunction(new_obstacles[n], tracked_obstacles_[t].getObstacle());

    for (int u = 0; u < U; ++u)
      cost_matrix(n, u + T) = obstacleCostFunction(new_obstacles[n], untracked_obstacles_[u]);
  }
}

void ObstacleTracker::calculateRowMinIndices(const mat& cost_matrix, vector<int>& row_min_indices) {
  /*
   * Vector of row minimal indices keeps the indices of old obstacles (tracked and untracked)
   * that have the minimum cost related to each of new obstacles, i.e. row_min_indices[n]
   * keeps the index of old obstacle that has the minimum cost with n-th new obstacle.
   */
  int N = cost_matrix.n_rows;
  int T = tracked_obstacles_.size();
  int U = untracked_obstacles_.size();

  row_min_indices.assign(N, -1); // Minimum index -1 means no correspondence has been found

  for (int n = 0; n < N; ++n) {
    double min_cost = p_min_correspondence_cost_;

    for (int t = 0; t < T; ++t) {
      if (cost_matrix(n, t) < min_cost) {
        min_cost = cost_matrix(n, t);
        row_min_indices[n] = t;
      }
    }

    for (int u = 0; u < U; ++u) {
      if (cost_matrix(n, u + T) < min_cost) {
        min_cost = cost_matrix(n, u + T);
        row_min_indices[n] = u + T;
      }
    }
  }
}

void ObstacleTracker::calculateColMinIndices(const mat& cost_matrix, vector<int>& col_min_indices) {
  /*
   * Vector of column minimal indices keeps the indices of new obstacles that has the minimum
   * cost related to each of old (tracked and untracked) obstacles, i.e. col_min_indices[i]
   * keeps the index of new obstacle that has the minimum cost with i-th old obstacle.
   */
  int N = cost_matrix.n_rows;
  int T = tracked_obstacles_.size();
  int U = untracked_obstacles_.size();

  col_min_indices.assign(T + U, -1); // Minimum index -1 means no correspondence has been found

  for (int t = 0; t < T; ++t) {
    double min_cost = p_min_correspondence_cost_;

    for (int n = 0; n < N; ++n) {
      if (cost_matrix(n, t) < min_cost) {
        min_cost = cost_matrix(n, t);
        col_min_indices[t] = n;
      }
    }
  }

  for (int u = 0; u < U; ++u) {
    double min_cost = p_min_correspondence_cost_;

    for (int n = 0; n < N; ++n) {
      if (cost_matrix(n, u + T) < min_cost) {
        min_cost = cost_matrix(n, u + T);
        col_min_indices[u + T] = n;
      }
    }
  }
}

bool ObstacleTracker::fusionObstacleUsed(const int idx, const vector<int> &col_min_indices, const vector<int> &used_new, const vector<int> &used_old) {
  /*
   * This function returns true if:
   * - idx-th old obstacle was already used
   * - obstacle to which idx-th old obstacle corresponds was already used
   * - there is no corresponding obstacle
   */

  return (find(used_old.begin(), used_old.end(), idx) != used_old.end() ||
          find(used_new.begin(), used_new.end(), col_min_indices[idx]) != used_new.end() ||
          col_min_indices[idx] < 0);
}

bool ObstacleTracker::fusionObstaclesCorrespond(const int idx, const int jdx, const vector<int>& col_min_indices, const vector<int>& used_old) {
  /*
   * This function returns true if:
   * - both old obstacles correspond to the same new obstacle
   * - jdx-th old obstacle was not yet used
   */

  return (col_min_indices[idx] == col_min_indices[jdx] &&
          find(used_old.begin(), used_old.end(), jdx) == used_old.end());
}

bool ObstacleTracker::fissionObstacleUsed(const int idx, const int T, const vector<int>& row_min_indices, const vector<int>& used_new, const vector<int>& used_old) {
  /*
   * This function returns true if:
   * - idx-th new obstacle was already used
   * - obstacle to which idx-th new obstacle corresponds was already used
   * - there is no corresponding obstacle
   * - obstacle to which idx-th new obstacle corresponds is untracked
   */

  return (find(used_new.begin(), used_new.end(), idx) != used_new.end() ||
          find(used_old.begin(), used_old.end(), row_min_indices[idx]) != used_old.end() ||
          row_min_indices[idx] < 0 ||
          row_min_indices[idx] >= T);
}

bool ObstacleTracker::fissionObstaclesCorrespond(const int idx, const int jdx, const vector<int>& row_min_indices, const vector<int>& used_new) {
  /*
   * This function returns true if:
   * - both new obstacles correspond to the same old obstacle
   * - jdx-th new obstacle was not yet used
   */

  return (row_min_indices[idx] == row_min_indices[jdx] &&
          find(used_new.begin(), used_new.end(), jdx) == used_new.end());
}

void ObstacleTracker::fuseObstacles(const vector<int>& fusion_indices, const vector<int> &col_min_indices,
                                    vector<TrackedObstacle>& new_tracked, const msg::Obstacles::SharedPtr& new_obstacles) {
  msg::CircleObstacle c;

  double sum_var_x  = 0.0;
  double sum_var_y  = 0.0;
  double sum_var_vx = 0.0;
  double sum_var_vy = 0.0;
  double sum_var_r  = 0.0;

  for (int idx : fusion_indices) {
    c.center.x += tracked_obstacles_[idx].getObstacle().center.x / tracked_obstacles_[idx].getKFx().P(0,0);
    c.center.y += tracked_obstacles_[idx].getObstacle().center.y / tracked_obstacles_[idx].getKFy().P(0,0);
    c.velocity.x += tracked_obstacles_[idx].getObstacle().velocity.x / tracked_obstacles_[idx].getKFx().P(1,1);
    c.velocity.y += tracked_obstacles_[idx].getObstacle().velocity.y / tracked_obstacles_[idx].getKFy().P(1,1);
    c.radius += tracked_obstacles_[idx].getObstacle().radius / tracked_obstacles_[idx].getKFr().P(0,0);

    sum_var_x += 1.0 / tracked_obstacles_[idx].getKFx().P(0,0);
    sum_var_y += 1.0 / tracked_obstacles_[idx].getKFy().P(0,0);
    sum_var_vx += 1.0 / tracked_obstacles_[idx].getKFx().P(1,1);
    sum_var_vy += 1.0 / tracked_obstacles_[idx].getKFy().P(1,1);
    sum_var_r += 1.0 / tracked_obstacles_[idx].getKFr().P(0,0);
  }

  c.center.x /= sum_var_x;
  c.center.y /= sum_var_y;
  c.velocity.x /= sum_var_vx;
  c.velocity.y /= sum_var_vy;
  c.radius /= sum_var_r;

  TrackedObstacle to(c);
  to.correctState(new_obstacles->circles[col_min_indices[fusion_indices.front()]]);
  for (int i = 0; i < static_cast<int>(p_loop_rate_ / p_sensor_rate_); ++i)
    to.updateState();

  new_tracked.push_back(to);
}

void ObstacleTracker::fissureObstacle(const vector<int>& fission_indices, const vector<int>& row_min_indices,
                                      vector<TrackedObstacle>& new_tracked, const msg::Obstacles::SharedPtr& new_obstacles) {
  // For each new obstacle taking part in fission create a tracked obstacle from the original old one and update it with the new one
  for (int idx : fission_indices) {
    TrackedObstacle to = tracked_obstacles_[row_min_indices[idx]];
    to.correctState(new_obstacles->circles[idx]);
    for (int i = 0; i < static_cast<int>(p_loop_rate_ / p_sensor_rate_); ++i)
      to.updateState();

    new_tracked.push_back(to);
  }
}

void ObstacleTracker::updateObstacles() {
  for (std::size_t i = 0; i < tracked_obstacles_.size(); ++i) {
    if (!tracked_obstacles_[i].hasFaded())
      tracked_obstacles_[i].updateState();
    else
      tracked_obstacles_.erase(tracked_obstacles_.begin() + i--);
  }
}

void ObstacleTracker::publishObstacles() {
  msg::Obstacles obstacles_msg {};

  obstacles_.circles.clear();

  for (auto& tracked_obstacle : tracked_obstacles_) {
    msg::CircleObstacle ob = tracked_obstacle.getObstacle();
    ob.true_radius = ob.radius - radius_margin_;
    obstacles_.circles.push_back(ob);
  }

  obstacles_msg = obstacles_;
  obstacles_msg.header.stamp = this->now();

  if(obstacles_pub_) {
    obstacles_pub_->publish(obstacles_msg);
  }
}

// Ugly initialization of static members of tracked obstacles...
int    TrackedObstacle::s_fade_counter_size_     = 0;
double TrackedObstacle::s_sampling_time_         = 100.0;
double TrackedObstacle::s_process_variance_      = 0.0;
double TrackedObstacle::s_process_rate_variance_ = 0.0;
double TrackedObstacle::s_measurement_variance_  = 0.0;
