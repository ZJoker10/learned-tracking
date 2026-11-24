cmake_minimum_required(VERSION 3.22)
project(learned_tracking LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

add_library(trajectory_core STATIC
  cpp/src/trapezoidal_profile.cpp
  cpp/src/cubic_polynomial_profile.cpp
  cpp/src/quintic_polynomial_profile.cpp
  cpp/src/s_curve_profile.cpp
  cpp/src/multi_segment_planner.cpp
)
target_include_directories(trajectory_core PUBLIC cpp/include)
set_target_properties(trajectory_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

pybind11_add_module(_core cpp/bindings.cpp)
target_link_libraries(_core PRIVATE trajectory_core)

install(TARGETS _core LIBRARY DESTINATION learned_tracking)
