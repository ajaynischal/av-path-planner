cmake_minimum_required(VERSION 3.16)

project(av-path-planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -O2)

add_executable(av-path-planner
  src/main.cpp
  src/grid.cpp
  src/astar.cpp
  src/visualizer.cpp
  src/scenario.cpp
)

target_include_directories(av-path-planner PRIVATE include)

add_executable(test_astar
  tests/test_astar.cpp
  src/grid.cpp
  src/astar.cpp
  src/scenario.cpp
)

target_include_directories(test_astar PRIVATE include)
