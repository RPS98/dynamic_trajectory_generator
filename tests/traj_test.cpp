/*!*******************************************************************************************
 *  \file       multirotor_simulator_traj_gen_test.cpp
 *  \brief      Class test
 *  \authors    Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "dynamic_trajectory_generator/dynamic_trajectory.hpp"
#include "dynamic_trajectory_generator/dynamic_waypoint.hpp"


class CsvLogger
{
public:
  explicit CsvLogger(const std::string & file_name)
  : file_name_(file_name)
  {
    std::cout << "Saving to file: " << file_name << std::endl;
    file_ = std::ofstream(file_name, std::ofstream::out | std::ofstream::trunc);
    file_ << "time,x,y,z,vx,vy,vz,ax,ay,az" << std::endl;
  }

  ~CsvLogger() {file_.close();}


  void save(const double time, const dynamic_traj_generator::References & references)
  {
    // Save time
    file_ << time << ",";

    for (int i = 0; i < 3; i++) {
      // Save position, velocity and acceleration
      file_ << references.position[i] << ",";
    }

    for (int i = 0; i < 3; i++) {
      // Save velocity
      file_ << references.velocity[i] << ",";
    }

    for (int i = 0; i < 3; i++) {
      // Save acceleration
      file_ << references.acceleration[i];
      if (i < 2) {
        file_ << ",";
      }
    }

    // End line
    file_ << std::endl;
  }

  void close() {file_.close();}

private:
  std::string file_name_;
  std::ofstream file_;
};


dynamic_traj_generator::DynamicWaypoint::Vector eigen_vector_to_dynamic_waypoint_vector(
  const std::vector<Eigen::Vector3d> & vector_waypoints)
{
  dynamic_traj_generator::DynamicWaypoint::Vector vector_dynamic_waypoints;
  for (auto waypoint : vector_waypoints) {
    dynamic_traj_generator::DynamicWaypoint dynamic_waypoint;
    dynamic_waypoint.resetWaypoint(waypoint);
    vector_dynamic_waypoints.push_back(dynamic_waypoint);
  }
  return vector_dynamic_waypoints;
}


int main(int argc, char ** argv)
{
  // Logger
  CsvLogger logger("trajectory.csv");

  using DynamicTrajectory = dynamic_traj_generator::DynamicTrajectory;
  using DynamicWaypoint = dynamic_traj_generator::DynamicWaypoint;

  Eigen::Vector3d initial_position = Eigen::Vector3d(0.0, 0.0, 1.23);
  double speed = 4.0;

  std::vector<Eigen::Vector3d> vector_waypoints;
  // gate_1: [3.05, 0.0, 1.23, 0.0]  # [x, y, z, yaw]
  // gate_2: [10.05, 5.0, 1.23, 0.0]  # [x, y, z]
  // gate_3: [17.05, 0.0, 1.23]  # [x, y, z]
  // gate_4: [24.05, 5.0, 1.23]  # [x, y, z]
  vector_waypoints.push_back(Eigen::Vector3d(3.05, 0.0, 1.23));
  vector_waypoints.push_back(Eigen::Vector3d(9.05, 5.0, 1.23));
  vector_waypoints.push_back(Eigen::Vector3d(10.05, 5.0, 1.23));
  vector_waypoints.push_back(Eigen::Vector3d(11.05, 5.0, 1.23));
  vector_waypoints.push_back(Eigen::Vector3d(17.05, 0.0, 1.23));
  vector_waypoints.push_back(Eigen::Vector3d(24.05, 5.0, 1.23));
  vector_waypoints.push_back(Eigen::Vector3d(27.05, 5.0, 1.23));

  float square_side = 5.0;
  // vector_waypoints.push_back(Eigen::Vector3d(8.45, 3.55, 1.23));
  // vector_waypoints.push_back(Eigen::Vector3d(14.12, 0.0, 1.23));
  // vector_waypoints.push_back(Eigen::Vector3d(20.45, 3.55, 1.23));
  // vector_waypoints.push_back(Eigen::Vector3d(26.12, 0.0, 1.23));

  // auto center = Eigen::Vector3d(-5.0, 0.0, 1.23);
  // auto corner_u_l = Eigen::Vector3d(
  //   center[0] - square_side / 2.0, center[1] + square_side / 2.0,
  //   center[2]);
  // auto corner_u_r = Eigen::Vector3d(
  //   center[0] + square_side / 2.0, center[1] + square_side / 2.0,
  //   center[2]);
  // auto corner_l_r = Eigen::Vector3d(
  //   center[0] + square_side / 2.0, center[1] - square_side / 2.0,
  //   center[2]);
  // auto corner_l_l = Eigen::Vector3d(
  //   center[0] - square_side / 2.0, center[1] - square_side / 2.0,
  //   center[2]);

  // vector_waypoints.push_back(corner_l_l);
  // vector_waypoints.push_back(corner_u_l);
  // vector_waypoints.push_back(corner_u_r);
  // vector_waypoints.push_back(corner_l_r);

  // vector_waypoints.push_back(corner_l_l);
  // vector_waypoints.push_back(corner_u_l);
  // vector_waypoints.push_back(corner_u_r);
  // vector_waypoints.push_back(corner_l_r);
  // Eigen::Vector3d takeoff = Eigen::Vector3d(0.0, 0.0, 2.23);
  // Eigen::Vector3d land = Eigen::Vector3d(0.0, 0.0, 0.0);
  // vector_waypoints.push_back(takeoff);
  // vector_waypoints.push_back(land);


  // Initialize dynamic trajectory generator
  std::unique_ptr<DynamicTrajectory> trajectory_generator = std::make_unique<DynamicTrajectory>();

  // Set trajectory generator parameters
  trajectory_generator->updateVehiclePosition(initial_position);
  trajectory_generator->setSpeed(speed);
  DynamicWaypoint::Vector waypoints_to_set = eigen_vector_to_dynamic_waypoint_vector(
    vector_waypoints);

  // Generate trajectory
  trajectory_generator->setWaypoints(waypoints_to_set);
  double max_time = trajectory_generator->getMaxTime();  // Block until trajectory is generated

  // Evaluate trajectory
  dynamic_traj_generator::References references;
  double dt = 0.01;  // seconds
  for (double t = 0.0; t < max_time - dt; t += dt) {
    trajectory_generator->evaluateTrajectory(t, references);
    logger.save(t, references);
  }


  // double distance = 10.0;
  // double v_max = 1.0;
  // double a_max = 2.0 * 9.81;
  // // double a_max = 0.1;
  // double magic_fabian_constant = 6.5;

  // /**
  //  t =
  //     distance / v_max * 2.0 * (1.0 + magic_fabian_constant * v_max / a_max * exp(-distance / v_max * 2.0))
  //  */
  // double a = distance / v_max;
  // double c = v_max / a_max;
  // double b = magic_fabian_constant * c * exp(-a * 2.0);
  // double t = a * (b + 1.0);

  // std::cout << "a: " << a << std::endl;
  // std::cout << "b: " << b << std::endl;
  // std::cout << "Time: " << t << std::endl;

  return 0;
}
