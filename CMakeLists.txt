project(trajectory_smoothing)
cmake_minimum_required(VERSION 3.18...3.22)
find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()



# Detect the installed nanobind package and import it into CMake
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE NB_DIR)
list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
find_package(nanobind CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(
# include
${EIGEN3_INCLUDE_DIR}
)



nanobind_add_module(trajectory_smoothing_impl src/cpp/trajectory_smoothing.cpp src/cpp/Trajectory.cpp src/cpp/Path.cpp)

install(TARGETS trajectory_smoothing_impl LIBRARY DESTINATION .)
