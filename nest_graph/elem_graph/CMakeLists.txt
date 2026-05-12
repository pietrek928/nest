nanobind_add_module(_elem_graph
  elem_graph_module.cc
  elem_graph.cc
  rules.cc
)
target_include_directories(_elem_graph PRIVATE
  "${CMAKE_SOURCE_DIR}/nest_graph"
  "${CMAKE_CURRENT_SOURCE_DIR}"
)

if(DEFINED ENV{NEST_GRAPH_NATIVE})
  target_compile_options(_elem_graph PRIVATE -march=native)
endif()
target_compile_options(_elem_graph PRIVATE -O2 -g)

install(TARGETS _elem_graph LIBRARY DESTINATION nest_graph/elem_graph)

if(NEST_GRAPH_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.4.0
  )
  FetchContent_MakeAvailable(Catch2)

  add_executable(elem_graph_cpp_tests tests/test_convex_gjk.cpp)
  target_link_libraries(elem_graph_cpp_tests PRIVATE Catch2::Catch2WithMain)
  target_include_directories(elem_graph_cpp_tests PRIVATE
    "${CMAKE_SOURCE_DIR}/nest_graph"
    "${CMAKE_CURRENT_SOURCE_DIR}"
  )

  add_test(NAME elem_graph.cpp.gjk COMMAND elem_graph_cpp_tests)
endif()
