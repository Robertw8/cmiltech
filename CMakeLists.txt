cmake_minimum_required(VERSION 3.20)

project(cmiltech LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(cmiltech
    src/main.cpp
    src/ammo.cpp
    src/simulation.cpp
    src/io.cpp
)

target_include_directories(cmiltech PRIVATE include)