# dynamic_trajectory_generator_py — Python bindings for Dynamic Trajectory Generator

This package exposes the `DynamicTrajectory` API of
`dynamic_trajectory_generator` to Python.

## Installation

From the repository root:

```bash
pip install ./pybind
```

This builds the C++ library, its transitive dependencies
(`mav_trajectory_generation`, `nlopt`) and the pybind11 extension, and
installs them as a self-contained wheel under the `dynamic_trajectory_generator_py` package.

### Requirements

- A C++17-capable compiler.
- CMake >= 3.16.
- Eigen3 development headers.
- pybind11 (installed automatically via scikit-build-core if not present).
- NumPy >= 1.22.

## Usage

```python
import numpy as np
from dynamic_trajectory_generator_py import DynamicTrajectory

traj = DynamicTrajectory()

initial_position = np.array([0.0, 0.0, 0.0])
initial_yaw = 0.0
waypoints = [
    np.array([0.0, 0.0, 10.0]),
    np.array([10.0, 0.0, 10.0]),
    np.array([10.0, 10.0, 20.0]),
]
max_velocity = 3.0

traj.generate_trajectory(initial_position, initial_yaw, waypoints, max_velocity)
traj.set_path_facing(True)

t_min = traj.get_min_time()
t_max = traj.get_max_time()

position, velocity, acceleration, yaw = traj.evaluate_trajectory(0.5 * (t_min + t_max))
```

## License

BSD-3-Clause.
