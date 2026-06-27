nanobind_add_module(geometry
  bindings/module.cc
  bindings/bind_module.cc
  bindings/types.cc
  bindings/python_converters.cc
  bindings/shapely.cc
  bindings/bind_geometry.cc
  bindings/bind_guidance.cc
  bindings/bind_distance.cc
  bindings/bind_intersect.cc
  bindings/bind_cast.cc
  bindings/bind_batch.cc
)
target_link_libraries(geometry PRIVATE nest_graph_headers)
target_include_directories(geometry PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/bindings"
)

if(DEFINED ENV{NEST_GRAPH_NATIVE})
  target_compile_options(geometry PRIVATE -march=native)
endif()
target_compile_options(geometry PRIVATE -O2 -g)

# Extension is nest_graph.geometry (nest_graph/geometry*.so), not nest_graph.geometry._geometry.
set_target_properties(geometry PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/nest_graph"
)

install(TARGETS geometry LIBRARY DESTINATION nest_graph)

if(NEST_GRAPH_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.4.0
  )
  FetchContent_MakeAvailable(Catch2)

  add_executable(geometry_cpp_tests
    tests/test_convex_gjk.cc
    tests/test_intersect.cc
    tests/test_distance.cc
    tests/test_circle.cc
    tests/test_decompose.cc
    tests/test_highly_curved_linestrings.cc
    tests/test_point_in_solid_bug.cc
    tests/test_holes.cc
    tests/test_transform.cc
    tests/test_geometry_api.cc
    tests/test_guide.cc
  )
  target_link_libraries(geometry_cpp_tests PRIVATE Catch2::Catch2WithMain nest_graph_headers)
  target_include_directories(geometry_cpp_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
  target_compile_definitions(geometry_cpp_tests PRIVATE NEST_GRAPH_DEFAULT_TRACER=DebugTracer)

  add_test(NAME geometry.cpp.gjk COMMAND geometry_cpp_tests)
  add_test(NAME geometry.cpp.telemetry COMMAND geometry_cpp_tests "[telemetry]")
  add_test(NAME geometry.cpp.decomposition COMMAND geometry_cpp_tests "[decomposition]")
endif()
