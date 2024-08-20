/*!*******************************************************************************************
 *  \file       trajectory_generator_benchmark.hpp
 *  \brief      Class benchmark
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

#include <benchmark/benchmark.h>
#include <random>

#include "dynamic_trajectory_generator/dynamic_trajectory.hpp"
#include "dynamic_trajectory_generator/dynamic_waypoint.hpp"

using namespace dynamic_traj_generator;

#define TRAJECTORY_LENGTH 100
#define SPEED 2.0
#define DISTANCE_BETWEEN_WAYPOINTS 5.0

// Class to discard the output
class NullBuffer : public std::streambuf {
public:
  int overflow(int c) { return c; }
};

std::vector<Eigen::Vector3d>
randomWaypointsGenerator(int numWaypoints, double distanceBetweenWaypoints) {
  std::vector<Eigen::Vector3d> waypoints;
  waypoints.reserve(numWaypoints);
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dis(-1.0, 1.0);

  Eigen::Vector3d currentWaypoint(0.0, 0.0, 0.0);
  waypoints.push_back(currentWaypoint);

  Eigen::Vector3d newWaypoint = Eigen::Vector3d::Zero();
  for (int i = 1; i < numWaypoints; ++i) {
    Eigen::Vector3d newWaypoint_aux = Eigen::Vector3d::Zero();
    do {
      double x = currentWaypoint.x() + distanceBetweenWaypoints * dis(gen);
      double y = currentWaypoint.y() + distanceBetweenWaypoints * dis(gen);
      double z = currentWaypoint.z() + distanceBetweenWaypoints * dis(gen);
      newWaypoint_aux = Eigen::Vector3d(x, y, z);
    } while (newWaypoint != currentWaypoint);
    newWaypoint = newWaypoint_aux;
    waypoints.push_back(newWaypoint);
    currentWaypoint = newWaypoint;
  }

  return waypoints;
}

DynamicWaypoint::Vector generateTrajectory() {

  DynamicWaypoint::Vector waypoints_to_set;
  waypoints_to_set.reserve(TRAJECTORY_LENGTH);

  std::vector<Eigen::Vector3d> waypoints =
      randomWaypointsGenerator(TRAJECTORY_LENGTH, TRAJECTORY_LENGTH);

  std::cout << "Waypoint len: " << waypoints.size() << std::endl;
  for (auto waypoint : waypoints) {
    DynamicWaypoint dynamic_waypoint;
    std::cout << "Waypoint: " << waypoint.transpose() << std::endl;
    dynamic_waypoint.resetWaypoint(waypoint);
    waypoints_to_set.push_back(dynamic_waypoint);
  }
  return waypoints_to_set;
}

// Benchmark dynamic trajectory generation
static void BM_trajectory_generator(benchmark::State &state) {
  DynamicTrajectory trajectory_generator = DynamicTrajectory();

  trajectory_generator.updateVehiclePosition(Eigen::Vector3d(1.0, 1.0, 5.0));
  trajectory_generator.setSpeed(SPEED);

  DynamicWaypoint::Vector waypoints_to_set;

  // Save the current state of cout
  std::streambuf *coutbuf = std::cout.rdbuf();

  // Redirect cout to NullBuffer
  NullBuffer nullBuffer;
  std::cout.rdbuf(&nullBuffer);

  // Perform the benchmark
  for (auto _ : state) {
    benchmark::DoNotOptimize(waypoints_to_set = generateTrajectory());
    trajectory_generator.setWaypoints(waypoints_to_set);
  }
  // Restore cout
  std::cout.rdbuf(coutbuf);
}
BENCHMARK(BM_trajectory_generator)->Threads(4)->Repetitions(1);

BENCHMARK_MAIN();
