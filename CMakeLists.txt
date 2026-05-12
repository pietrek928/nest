cmake_minimum_required(VERSION 3.18...3.31)
project(nest_graph_extensions LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Python 3 REQUIRED COMPONENTS Interpreter Development.Module)

execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE
  OUTPUT_VARIABLE nanobind_ROOT)
find_package(nanobind CONFIG REQUIRED)

option(NEST_GRAPH_BUILD_TESTS "Build C++ unit tests (Catch2 via FetchContent)" OFF)
if(NEST_GRAPH_BUILD_TESTS)
  enable_testing()
endif()

add_subdirectory(nest_graph/poly)
add_subdirectory(nest_graph/elem_graph)
