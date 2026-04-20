"""Python bindings for the Dynamic Trajectory Generator."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from ._dynamic_trajectory_generator_cpp import DynamicTrajectory

__all__ = ['DynamicTrajectory']
