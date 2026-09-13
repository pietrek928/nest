cmake_minimum_required(VERSION 3.18...3.31)
project(nest_graph_extensions LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# First-party warning flags (not global: keeps FetchContent Catch2 / nanobind quiet).
option(NEST_GRAPH_WERROR "Treat nest_graph warnings as errors" OFF)
add_library(nest_graph_warnings INTERFACE)
target_compile_options(nest_graph_warnings INTERFACE
  $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang,AppleClang>:-Wall;-Wextra;-Wpedantic;-Wno-unused-parameter>
)
if(NEST_GRAPH_WERROR)
  target_compile_options(nest_graph_warnings INTERFACE
    $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang,AppleClang>:-Werror>
  )
endif()

find_package(Python 3 REQUIRED COMPONENTS Interpreter Development.Module)

execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE
  OUTPUT_VARIABLE nanobind_ROOT)
find_package(nanobind CONFIG REQUIRED)
# Prefer SYSTEM so nanobind headers stay quiet under -Wall (first-party only).
get_filename_component(NEST_NANOBIND_INCLUDE "${nanobind_ROOT}/../include" ABSOLUTE)
if(EXISTS "${NEST_NANOBIND_INCLUDE}/nanobind/nanobind.h")
  set(NEST_NANOBIND_SYSTEM_INCLUDE "${NEST_NANOBIND_INCLUDE}")
endif()

# Public include root for nest_graph headers (e.g. #include <vec.h>).
add_library(nest_graph_headers INTERFACE)
target_include_directories(nest_graph_headers INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/nest_graph>"
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/nest_graph/common>"
)

option(NEST_GRAPH_BUILD_TESTS "Build C++ unit tests (Catch2 via FetchContent)" OFF)
if(NEST_GRAPH_BUILD_TESTS)
  enable_testing()
endif()

add_subdirectory(nest_graph/geometry)
add_subdirectory(nest_graph/graph)
