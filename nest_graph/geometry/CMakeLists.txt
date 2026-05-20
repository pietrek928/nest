nanobind_add_module(_geometry geometry.cc)
target_link_libraries(_geometry PRIVATE nest_graph_headers)
target_include_directories(_geometry PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")

if(DEFINED ENV{NEST_GRAPH_NATIVE})
  target_compile_options(_geometry PRIVATE -march=native)
endif()
target_compile_options(_geometry PRIVATE -O2 -g)

# Plain `cmake --build` (non-scikit-build): place the extension next to __init__.py
# so `import nest_graph.geometry` works with pytest pythonpath = ["."].
set_target_properties(_geometry PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/nest_graph/geometry"
)

install(TARGETS _geometry LIBRARY DESTINATION nest_graph/geometry)

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
  )
  target_link_libraries(geometry_cpp_tests PRIVATE Catch2::Catch2WithMain nest_graph_headers)
  target_include_directories(geometry_cpp_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
  target_compile_definitions(geometry_cpp_tests PRIVATE NEST_GRAPH_DEFAULT_TRACER=DebugTracer)

  add_test(NAME geometry.cpp.gjk COMMAND geometry_cpp_tests)
  add_test(NAME geometry.cpp.telemetry COMMAND geometry_cpp_tests "[telemetry]")
  add_test(NAME geometry.cpp.decomposition COMMAND geometry_cpp_tests "[decomposition]")
endif()
