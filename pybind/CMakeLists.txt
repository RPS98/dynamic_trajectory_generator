cmake_minimum_required(VERSION 3.16)
set(PROJECT_NAME _dynamic_trajectory_generator_cpp)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# For shared libraries
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Dependencies
find_package(Eigen3 REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Add the parent project as a subdirectory if the target is not already defined
# (the target is already defined when the pybind folder is built together with
# the parent project from an outer CMake tree).
if(NOT TARGET dynamic_trajectory_generator)
  set(BUILD_TESTING OFF CACHE BOOL "Disable tests in subprojects" FORCE)
  add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_BINARY_DIR}/dynamic_trajectory_generator_build
  )
endif()

# Build the pybind11 extension
pybind11_add_module(_dynamic_trajectory_generator_cpp src/module.cpp)

target_include_directories(_dynamic_trajectory_generator_cpp PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(_dynamic_trajectory_generator_cpp PRIVATE dynamic_trajectory_generator)
target_compile_features(_dynamic_trajectory_generator_cpp PRIVATE cxx_std_17)

# Embed an RPATH pointing to the installed package directory so that the
# extension can locate its transitive shared library dependencies without
# requiring LD_LIBRARY_PATH manipulation.
set_target_properties(_dynamic_trajectory_generator_cpp PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH TRUE
)

# Install the Python package source tree (pure python wrappers)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/python/dynamic_trajectory_generator_py/
        DESTINATION ${SKBUILD_PLATLIB_DIR}/dynamic_trajectory_generator_py
)

# Install the compiled extension next to the Python package
install(TARGETS _dynamic_trajectory_generator_cpp
  LIBRARY DESTINATION ${SKBUILD_PLATLIB_DIR}/dynamic_trajectory_generator_py
  RUNTIME DESTINATION ${SKBUILD_PLATLIB_DIR}/dynamic_trajectory_generator_py
  ARCHIVE DESTINATION ${SKBUILD_PLATLIB_DIR}/dynamic_trajectory_generator_py
)

# Also install the transitive shared libraries produced during the build so
# that the wheel is self-contained. We force INSTALL_RPATH="$ORIGIN" on each
# shared dependency so that, once co-located inside the installed wheel
# directory, they can resolve their own NEEDED entries (e.g. libnlopt.so.0)
# without requiring LD_LIBRARY_PATH. Targets built as STATIC/OBJECT libraries
# are silently skipped by install(TARGETS ...) when requested as SHARED, so
# this is robust against upstream changes in library type.
foreach(_dep_target IN ITEMS dynamic_trajectory_generator mav_trajectory_generation nlopt)
  if(TARGET ${_dep_target})
    get_target_property(_dep_type ${_dep_target} TYPE)
    if(_dep_type STREQUAL "SHARED_LIBRARY")
      set_target_properties(${_dep_target} PROPERTIES
        INSTALL_RPATH "$ORIGIN"
        BUILD_WITH_INSTALL_RPATH TRUE
      )
      install(TARGETS ${_dep_target}
        LIBRARY DESTINATION ${SKBUILD_PLATLIB_DIR}/dynamic_trajectory_generator_py
        RUNTIME DESTINATION ${SKBUILD_PLATLIB_DIR}/dynamic_trajectory_generator_py
      )
    endif()
  endif()
endforeach()
