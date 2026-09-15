cmake_minimum_required(VERSION 3.20)

project(cmiltech LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(cmiltech
    src/main.cc
    src/ammo.cc
    src/io.cc

    src/drone/drone_math.cc
    src/drone/drone_simulation.cc

    src/simulation/simulation_math.cc
    src/simulation/simulation.cc

    src/targets/target_tracking.cc
    src/targets/target_evaluation.cc
    src/targets/target_selection.cc
)

target_include_directories(cmiltech PRIVATE include)