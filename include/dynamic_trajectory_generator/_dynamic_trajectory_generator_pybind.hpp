// Copyright 2024 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//    of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
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

/**
 * @file _dynamic_trajectory_generator_pybind.hpp
 *
 * @brief Class bind to python for the Dynamic Trajectory Generator class.
 *
 * @author Rafael Pérez Seguí
 */

#ifndef _DYNAMIC_TRAJECTORY_GENERATOR_PYBIND_HPP_
#define _DYNAMIC_TRAJECTORY_GENERATOR_PYBIND_HPP_

#include <math.h>
#include <Eigen/Dense>

#include <tuple>
#include <string>
#include <memory>
#include <vector>

#include "dynamic_trajectory_generator/dynamic_trajectory.hpp"
#include "dynamic_trajectory_generator/dynamic_waypoint.hpp"

namespace dynamic_traj_generator
{

/**
 * @brief Class bind to python for the Dynamic Trajectory Generator class.
 */
class DynamicTrajectoryBind
{
public:
  /**
   * @brief Constructor of the class.
   */
  DynamicTrajectoryBind() {}

  /**
   * @brief Destructor of the class.
   */
  ~DynamicTrajectoryBind() = default;

  /**
   * @brief Generate a trajectory given an initial position, initial yaw, waypoints and max velocity.
   *
   * @param initial_position Initial position of the vehicle.
   * @param initial_yaw Initial yaw angle (rad) of the vehicle.
   * @param waypoints Waypoints of the trajectory.
   * @param max_velocity Maximum velocity of the vehicle (m/s).
   */
  void generateTrajectory(
    const Eigen::Vector3d & initial_position,
    const float initial_yaw,
    const std::vector<Eigen::Vector3d> & waypoints,
    const float max_velocity)
  {
    // Reset trajectory generator for each new trajectory
    trajectory_generator_ = std::make_shared<DynamicTrajectory>();

    if (waypoints.size() < 2) {
      throw std::runtime_error("At least two waypoints are needed to generate a trajectory");
    }
    if (max_velocity <= 0) {
      throw std::runtime_error("Max velocity must be greater than 0");
    }
    // Set initial position
    trajectory_generator_->updateVehiclePosition(initial_position);

    // Set velocity
    trajectory_generator_->setSpeed(max_velocity);

    // Eigen to DynamicWaypoint
    auto waypoints_to_set = EigenVectorToDynamicWaypointVector(waypoints);

    // Generate trajectory
    trajectory_generator_->setWaypoints(waypoints_to_set);

    // Set yaw reference
    yaw_reference_ = initial_yaw;

    // Block until the trajectory is generated
    max_time_ = getMaxTime();
    min_time_ = getMinTime();

    return;
  }

  /**
   * @brief Evaluate the trajectory at a given time.
   *
   * @param time Time to evaluate the trajectory (s).
   * @param position Position reference (m).
   * @param velocity Velocity reference (m/s).
   * @param acceleration Acceleration reference (m/s^2).
   * @param yaw Yaw reference (rad).
   */
  std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, float> evaluateTrajectory(
    const float time)
  {
    if (time < min_time_ || time > max_time_) {
      throw std::runtime_error("Time out of bounds");
    }

    dynamic_traj_generator::References references;
    trajectory_generator_->evaluateTrajectory(time, references);

    if (path_facing_) {
      yaw_reference_ = computeYawAnglePathFacing(references.velocity);
    }

    return std::make_tuple(
      references.position,
      references.velocity,
      references.acceleration,
      yaw_reference_);
  }

  /**
   * @brief Set the path facing flag.
   *
   * @param enable Enable/disable path facing.
   */
  void setPathFacing(const bool enable)
  {
    path_facing_ = enable;
  }

  /**
   * @brief Get the maximum time of the trajectory.
   *
   * @return Maximum time of the trajectory (s).
   */
  inline double getMaxTime()
  {
    return trajectory_generator_->getMaxTime();
  }

  /**
   * @brief Get the minimum time of the trajectory.
   *
   * @return Minimum time of the trajectory (s).
   */
  inline double getMinTime()
  {
    return trajectory_generator_->getMinTime();
  }

  /**
   * @brief Get the speed of the vehicle.
   *
   * @return Speed of the vehicle (m/s).
   */
  inline bool getWasTrajectoryRegenerated()
  {
    return trajectory_generator_->getWasTrajectoryRegenerated();
  }

private:
  std::shared_ptr<DynamicTrajectory> trajectory_generator_;
  bool path_facing_ = false;
  float yaw_reference_ = 0.0;
  double max_time_ = 0.0;
  double min_time_ = 0.0;

  float computeYawAnglePathFacing(const Eigen::Vector3d velocity)
  {
    if (Eigen::Vector2d(velocity.x(), velocity.y()).norm() > 0.1) {
      return atan2f(
        velocity.x(),
        velocity.y());
    }
    return yaw_reference_;
  }

  DynamicWaypoint::Vector EigenVectorToDynamicWaypointVector(
    const std::vector<Eigen::Vector3d> & vector_waypoints)
  {
    DynamicWaypoint::Vector vector_dynamic_waypoints;
    vector_dynamic_waypoints.reserve(vector_waypoints.size());
    for (auto waypoint : vector_waypoints) {
      DynamicWaypoint dynamic_waypoint;
      dynamic_waypoint.resetWaypoint(waypoint);
      vector_dynamic_waypoints.emplace_back(dynamic_waypoint);
    }
    return vector_dynamic_waypoints;
  }
};  // class DynamicTrajectoryBind

}  // namespace dynamic_traj_generator

#endif  // _DYNAMIC_TRAJECTORY_GENERATOR_PYBIND_HPP_
