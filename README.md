## Dynamic Traj Generator


Library for generating dynamic trajectories that changes over the time.

### Installation

Clone this repository and compile it using the following commands:

- Clone the repository
```bash
git clone https://github.com/RPS98/dynamic_trajectory_generator.git
cd dynamic_trajectory_generator
```

- Compile the library
```bash
mkdir build
cd build
cmake .. -DBUILD_PYBIND=true -DCMAKE_INSTALL_PREFIX=../install
make install
```

- Export CMAKE_PREFIX_PATH to the install directory

```bash
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$(pwd)/install
```

- Import the library in your CMakeLists.txt

```cmake
find_package(dynamic_trajectory_generator REQUIRED)
```






TODO: 

  - [x] Remove ROS stuff
  - [x] PLOT 2D and 3D plots at the same time
  - [x] yamlcpp installation
  - [x] 3D plotting
  - [ ] check Python Version for Matplotlib plotting purposes
  - [ ] Plotting only in tests?
  - [ ] Testing with Gtest
  - [ ] Relate Trajectory Modifiers with uav speed and position **!! IMPORTANTE**
  - [ ] Expand Time when modifiers are applied to mantain speed and acceleration constraints.
  - [ ] Create Dynamic Segments

Bugs known:
  - [ ] When dynamic waypoints are too close the previous gaussian modifies the actual waypoint trajectory

Dependencies:
- YamlCPP (```$ sudo apt install libyaml-cpp* ```)
