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

#include "processing_lidar_objects/obstacle_publisher.h"

#include <chrono>
#include <cmath>
#include <functional>

using namespace std;
using namespace processing_lidar_objects;
using namespace std::placeholders;
using namespace std::chrono_literals;

const auto UPDATE_TIMER_DURATION {1s};
const auto DEFAULT_NODE_NAME = "obstacle_publisher";

ObstaclePublisher::ObstaclePublisher(
    const rclcpp::NodeOptions & node_options
):
  ObstaclePublisher(DEFAULT_NODE_NAME, node_options)
{}

ObstaclePublisher::ObstaclePublisher(
    std::string node_name,
    const rclcpp::NodeOptions & node_options
):
  Node(node_name, node_options)
{
  p_active_ = false;
  t_ = 0.0;

  params_srv_ = this->create_service<std_srvs::srv::Empty>(
    "~/params",
    std::bind(&ObstaclePublisher::updateParamsCallback, this, _1, _2)
  );
  initialize();
}

ObstaclePublisher::~ObstaclePublisher() {
  this->undeclar_parameter("active");
  this->undeclar_parameter("reset");

  this->undeclar_parameter("fusion_example");
  this->undeclar_parameter("fission_example");

  this->undeclar_parameter("loop_rate");
  this->undeclar_parameter("radius_margin");

  this->undeclar_parameter("x_vector");
  this->undeclar_parameter("y_vector");
  this->undeclar_parameter("r_vector");

  this->undeclar_parameter("vx_vector");
  this->undeclar_parameter("vy_vector");

  this->undeclar_parameter("frame_id");
}

void ObstaclePublisher::initialize() {
  this->declar_parameter("active", true);
  this->declar_parameter("reset", false);

  this->declar_parameter("fusion_example", false);
  this->declar_parameter("fission_example", false);

  this->declar_parameter("loop_rate", 10.0); // Hz ?
  this->declar_parameter("radius_margin", 0.25);

  // Initializes with empty vectors
  this->declar_parameter("x_vector", p_x_vector_); 
  this->declar_parameter("y_vector", p_y_vector_);
  this->declar_parameter("r_vector", p_r_vector_);

  this->declar_parameter("vx_vector", p_vx_vector_);
  this->declar_parameter("vy_vector", p_vy_vector_);

  this->declar_parameter("frame_id", "map");

  updateParams();
}

void ObstaclePublisher::updateParamsCallback(
  [[maybe_unused]] const std_srvs::srv::Empty::Request::SharedPtr req,
  [[maybe_unused]] std_srvs::srv::Empty::Response::SharedPtr res
) {
  updateParams();
}

void ObstaclePublisher::updateParams() {
  bool prev_active = p_active_;
  auto prev_sampling_time = p_sampling_time_;

  p_active_ = this->get_parameter("~/active").get_value<bool>();
  p_reset_ = this->get_parameter("~/reset").get_value<bool>();

  p_fusion_example_ = this->get_parameter("~/fusion_example").get_value<bool>();
  p_fission_example_ = this->get_parameter("~/fission_example").get_value<bool>();

  p_loop_rate_ = this->get_parameter("~/loop_rate").get_value<double>();
  p_radius_margin_ = this->get_parameter("~/radius_margin").get_value<double>();

  p_x_vector_ = this->get_parameter("~/x_vector").get_value<std::vector<double>>();
  p_y_vector_ = this->get_parameter("~/y_vector").get_value<std::vector<double>>();
  p_r_vector_ = this->get_parameter("~/r_vector").get_value<std::vector<double>>();

  p_vx_vector_ = this->get_parameter("~/vx_vector").get_value<std::vector<double>>();
  p_vy_vector_ = this->get_parameter("~/vy_vector").get_value<std::vector<double>>();

  p_frame_id_ = this->get_parameter("~/frame_id").get_value<string>();

  p_sampling_time_ = 1.0 / p_loop_rate_;

  if (p_active_ != prev_active) {
    if (p_active_) {
      obstacle_pub_ = this->create_publisher<msg::Obstacles>("obstacles", 10);
    }
    else {
      obstacle_pub_ = nullptr;
    }
  }
  
  if (p_active_ && (p_active_ != prev_active || p_sampling_time_ != prev_sampling_time)) {
    // FIXME may create undefined behavior (timer destroyed while it could be calling ObstaclePublisher::timerCallback)
    // use mutex lock ?
    timer_ = this->create_wall_timer(
      rclcpp::Duration::from_seconds(p_sampling_time_).to_chrono<std::chrono::nanoseconds>(), // TODO find cleaner way to cast from double
      std::bind(&ObstaclePublisher::timerCallback, this)
    );
  } else if (!p_active_ && timer_) {
    timer_->cancel(); // Useful ?
    timer_ = nullptr;
  }

  obstacles_.header.frame_id = p_frame_id_;
  obstacles_.circles.clear();

  if (p_x_vector_.size() != p_y_vector_.size() || p_x_vector_.size() != p_r_vector_.size() ||
      p_x_vector_.size() != p_vx_vector_.size() || p_x_vector_.size() != p_vy_vector_.size())
    return;

  for (std::size_t idx = 0; idx < p_x_vector_.size(); ++idx) {
    msg::CircleObstacle circle;
    circle.center.x = p_x_vector_[idx];
    circle.center.y = p_y_vector_[idx];
    circle.radius = p_r_vector_[idx];
    circle.true_radius = p_r_vector_[idx] - p_radius_margin_;

    circle.velocity.x = p_vx_vector_[idx];
    circle.velocity.y = p_vy_vector_[idx];

    obstacles_.circles.push_back(circle);
  }

  if (p_reset_)
    reset();
}

void ObstaclePublisher::timerCallback() {
  t_ += p_sampling_time_;

  calculateObstaclesPositions(p_sampling_time_);

  if (p_fusion_example_)
    fusionExample(t_);
  else if (p_fission_example_)
    fissionExample(t_);

  if (obstacles_.circles.size() > 0)
    publishObstacles();
}

void ObstaclePublisher::calculateObstaclesPositions(double dt) {
  for (auto& circ : obstacles_.circles) {
    circ.center.x += circ.velocity.x * dt;
    circ.center.y += circ.velocity.y * dt;
  }
}

void ObstaclePublisher::fusionExample(double t) {
  msg::CircleObstacle circ1, circ2;

  obstacles_.circles.clear();

  if (t < 5.0) {
    circ1.center.x = -1.20 + 0.2 * t;
    circ1.center.y = 0.0;
    circ1.radius = 0.20;

    circ2.center.x = 1.20 - 0.2 * t;
    circ2.center.y = 0.0;
    circ2.radius = 0.20;

    obstacles_.circles.push_back(circ1);
    obstacles_.circles.push_back(circ2);
  }
  else if (t < 15.0) {
    circ1.center.x = 0.0;
    circ1.center.y = 0.0;
    circ1.radius = 0.20 + 0.20 * exp(-(t - 5.0) / 1.0);

    obstacles_.circles.push_back(circ1);
  }
  else  if (t > 20.0)
    reset();

  circ1.true_radius = circ1.radius;
  circ2.true_radius = circ2.radius;
}

void ObstaclePublisher::fissionExample(double t) {
  msg::CircleObstacle circ1, circ2;

  obstacles_.circles.clear();

  if (t < 5.0) {
    circ1.center.x = 0.0;
    circ1.center.y = 0.0;
    circ1.radius = 0.20;

    obstacles_.circles.push_back(circ1);
  }
  else if (t < 6.0) {
    circ1.center.x = 0.0;
    circ1.center.y = 0.0;
    circ1.radius = 0.20 + 0.20 * (1.0 - exp(-(t - 5.0) / 1.0));

    obstacles_.circles.push_back(circ1);
  }
  else if (t < 16.0){
    circ1.center.x = -0.20 - 0.2 * (t - 6.0);
    circ1.center.y = 0.0;
    circ1.radius = 0.20;

    circ2.center.x = 0.20 + 0.2 * (t - 6.0);
    circ2.center.y = 0.0;
    circ2.radius = 0.20;

    obstacles_.circles.push_back(circ1);
    obstacles_.circles.push_back(circ2);
  }
  else if (t > 20.0)
    reset();

  circ1.true_radius = circ1.radius;
  circ2.true_radius = circ2.radius;
}

void ObstaclePublisher::publishObstacles() {
  obstacles_.header.stamp = this->now();
  if (obstacle_pub_) {
    obstacle_pub_->publish(obstacles_);
  }
}

void ObstaclePublisher::reset() {
  t_ = 0.0;
  p_reset_ = false;
  this->set_parameter({"~/reset", false});
}


#include <rclcpp_components/register_node_macro.hpp>

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(processing_lidar_objects::ObstaclePublisher)
