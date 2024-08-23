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
 * @file dynamic_trajectory_pybind.cpp
 *
 * @brief Python bindings for the Dynamic Trajectory Generator.
 *
 * @author Rafael Pérez Seguí
 */

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "_dynamic_trajectory_generator_pybind.hpp"

using namespace dynamic_traj_generator;  // NOLINT
namespace py = pybind11;

PYBIND11_MODULE(trajectory_generator, m) {
  py::class_<DynamicTrajectoryBind>(m, "DynamicTrajectory")
  .def(py::init<>())        // Constructor
  .def(
    "generate_trajectory", &DynamicTrajectoryBind::generateTrajectory,
    py::arg("initial_position"),
    py::arg("initial_yaw"),
    py::arg("waypoints"),
    py::arg("max_velocity"),
    "Generate a trajectory given an initial position, initial yaw, waypoints and max velocity.")
  .def(
    "evaluate_trajectory", &DynamicTrajectoryBind::evaluateTrajectory,
    py::arg("time"),
    "Evaluate the trajectory at a given time.")
  .def(
    "set_path_facing", &DynamicTrajectoryBind::setPathFacing,
    py::arg("enable"),
    "Set the path facing flag.")
  .def(
    "get_max_time", &DynamicTrajectoryBind::getMaxTime,
    "Get the maximum time of the trajectory.")
  .def(
    "get_min_time", &DynamicTrajectoryBind::getMinTime,
    "Get the minimum time of the trajectory.")
  .def(
    "get_was_trajectory_regenerated", &DynamicTrajectoryBind::getWasTrajectoryRegenerated,
    "Check if the trajectory was regenerated.");
}
