cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME dynamic_trajectory_generator)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-fPIC)

find_package(Eigen3 REQUIRED)

add_subdirectory(subpackages/mav_trajectory_generation)

set(SOURCE_CPP_FILES
  src/dynamic_trajectory.cpp
  src/dynamic_waypoint.cpp
)

include_directories(
  include
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIRS}

  # ${MATPLOTLIB_CPP_INCLUDE_DIRS}
)

# add_executable(${PROJECT_NAME}_node src/main_trajectory_publisher.cpp ${SOURCE_CPP_FILES} ${HEADER_HPP_FILES})
# target_link_libraries(${PROJECT_NAME}_node nlopt glog::glog

# create library and export it
add_library(${PROJECT_NAME} ${SOURCE_CPP_FILES})
target_link_libraries(${PROJECT_NAME} mav_trajectory_generation nlopt)
target_include_directories(${PROJECT_NAME} PUBLIC include include/${PROJECT_NAME} ${EIGEN3_INCLUDE_DIRS})

# Tests
include(CTest)

# Check if tests are enabled
if(BUILD_TESTING)
  message(STATUS "Compilation tests enabled")

  set(TEST_DEPENDENCIES
    benchmark
  )

  foreach(DEPENDENCY ${TEST_DEPENDENCIES})
    find_package(${DEPENDENCY} REQUIRED)
  endforeach()

  if(${PROJECT_NAME}_TESTS_ENABLED)
    message(STATUS "Compilation visuals tests enabled")
    set(CMAKE_CXX_FLAGS " -I/usr/include/python3.8 -lpython3.8")
    add_subdirectory(subpackages/matplotlib_cpp)
    include_directories(subpackages/matplotlib_cpp)
    include_directories(matplotlib_cpp_INCLUDE_DIRS)

    file(GLOB TEST_SOURCE "tests/*_tests.cpp")

    foreach(TEST_FILE ${TEST_SOURCE})
      get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

      add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
      target_link_libraries(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_NAME} python3.8)
    endforeach()
  endif()

  file(GLOB TEST_SOURCE "tests/*_test.cpp")

  foreach(TEST_FILE ${TEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_NAME})
  endforeach()

  file(GLOB TEST_SOURCE "tests/*_benchmark.cpp")

  foreach(TEST_FILE ${TEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} benchmark::benchmark ${PROJECT_NAME})
  endforeach()
endif()
