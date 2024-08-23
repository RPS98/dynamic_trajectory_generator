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
 * @file pybind_test.hpp
 *
 * @brief Test class bind to python for the Dynamic Trajectory Generator.
 *
 * @author Rafael Pérez Seguí
 */

#include <Eigen/Dense>

#include <iostream>
#include <vector>

#include "_dynamic_trajectory_generator_pybind.hpp"

int main()
{
  dynamic_traj_generator::DynamicTrajectoryBind trajectory_bind;

  // Initial position of the vehicle
  const Eigen::Vector3d initial_position(0.0, 0.0, 1.0);
  const float initial_yaw = 0.0f;

  // Trajectory waypoints
  std::vector<Eigen::Vector3d> waypoints;
  waypoints.push_back(Eigen::Vector3d(1.0, 1.0, 1.0));
  waypoints.push_back(Eigen::Vector3d(1.0, -1.0, 1.0));
  waypoints.push_back(Eigen::Vector3d(-1.0, -1.0, 1.0));
  waypoints.push_back(Eigen::Vector3d(-1.0, 1.0, 1.0));
  waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));

  // Generate trajectory
  trajectory_bind.setPathFacing(true);
  const float max_velocity = 2.0f;

  std::cout << "Generating trajectory..." << std::endl;
  trajectory_bind.generateTrajectory(
    initial_position,
    initial_yaw,
    waypoints,
    max_velocity);

  // Get max and min time
  std::cout << "Getting max and min time..." << std::endl;
  const float max_time = trajectory_bind.getMaxTime();
  const float min_time = trajectory_bind.getMinTime();

  // References
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  float yaw;

  // Evaluate trajectory
  const float dt = 0.01f;

  for (float t = min_time; t < max_time; t += dt) {
    std::tie(position, velocity, acceleration, yaw) =
      trajectory_bind.evaluateTrajectory(t);
  }

  std::cout << "Generate trajectory from: " << initial_position.transpose() << std::endl;
  std::cout << "And yaw: " << initial_yaw << std::endl;
  std::cout << "Generate trajectory with max velocity: " << max_velocity << std::endl;
  std::cout << "Generate trajectory for waypoints: " << std::endl;
  for (int i = 0; i < waypoints.size(); i++) {
    std::cout << waypoints[i].transpose() << std::endl;
  }

  std::cout << "Trajectory generated with:" << std::endl;
  std::cout << "Min time: " << min_time << std::endl;
  std::cout << "Max time: " << max_time << std::endl;

  std::cout << "Trajectory final references:" << std::endl;
  std::cout << "Position: " << position.transpose() << std::endl;
  std::cout << "Velocity: " << velocity.transpose() << std::endl;
  std::cout << "Acceleration: " << acceleration.transpose() << std::endl;
  std::cout << "Yaw: " << yaw << std::endl;

  return 0;
}
