nanobind_add_module(poly_module poly_module.cc)
target_include_directories(poly_module PRIVATE
  "${CMAKE_SOURCE_DIR}/nest_graph"
  "${CMAKE_CURRENT_SOURCE_DIR}"
)

if(DEFINED ENV{NEST_GRAPH_NATIVE})
  target_compile_options(poly_module PRIVATE -march=native)
endif()
target_compile_options(poly_module PRIVATE -O2 -g)

install(TARGETS poly_module LIBRARY DESTINATION nest_graph)

if(NEST_GRAPH_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.4.0
  )
  FetchContent_MakeAvailable(Catch2)

  add_executable(poly_cpp_tests
    tests/test_convex_gjk.cc
    tests/test_poly_intersect.cc
    tests/test_poly_distance.cc
    tests/test_circle.cc
  )
  target_link_libraries(poly_cpp_tests PRIVATE Catch2::Catch2WithMain)
  target_include_directories(poly_cpp_tests PRIVATE
    "${CMAKE_SOURCE_DIR}/nest_graph"
    "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  target_compile_definitions(poly_cpp_tests PRIVATE NEST_GRAPH_DEFAULT_TRACER=DebugTracer)

  add_test(NAME poly.cpp.gjk COMMAND poly_cpp_tests)
endif()
