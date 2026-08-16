set(ELEM_GRAPH_CORE_SOURCES
  internal/internal.cc
  rules/rules.cc
  graph/graph_index.cc
  scoring/scoring.cc
  selection/selection.cc
  selection/evaluate_rules.cc
  refine/refine_dfs.cc
  refine/finalize.cc
)

add_library(elem_graph_core STATIC ${ELEM_GRAPH_CORE_SOURCES})
set_target_properties(elem_graph_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(elem_graph_core PUBLIC nest_graph_headers)
target_include_directories(elem_graph_core PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}"
)

nanobind_add_module(elem_graph
  bindings/module.cc
  bindings/types.cc
  bindings/api.cc
  bindings/bind_decision.cc
)
target_link_libraries(elem_graph PRIVATE elem_graph_core)
target_include_directories(elem_graph PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/bindings"
)

if(DEFINED ENV{NEST_GRAPH_NATIVE})
  target_compile_options(elem_graph PRIVATE -march=native)
  target_compile_options(elem_graph_core PRIVATE -march=native)
endif()
target_compile_options(elem_graph PRIVATE -O2 -g)
target_compile_options(elem_graph_core PRIVATE -O2 -g)

set_target_properties(elem_graph PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/nest_graph"
)

install(TARGETS elem_graph LIBRARY DESTINATION nest_graph)

if(NEST_GRAPH_BUILD_TESTS)
  if(NOT TARGET Catch2::Catch2WithMain)
    include(FetchContent)
    FetchContent_Declare(
      Catch2
      GIT_REPOSITORY https://github.com/catchorg/Catch2.git
      GIT_TAG v3.4.0
    )
    FetchContent_MakeAvailable(Catch2)
  endif()

  add_executable(elem_graph_cpp_tests
    tests/test_selection.cc
    tests/test_refine_dfs.cc
    tests/test_se2.cc
    tests/test_contact_relation.cc
    tests/test_decision_motif.cc
  )
  target_link_libraries(elem_graph_cpp_tests PRIVATE
    elem_graph_core
    Catch2::Catch2WithMain
  )
  target_include_directories(elem_graph_cpp_tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_SOURCE_DIR}/nest_graph"
    "${CMAKE_SOURCE_DIR}/nest_graph/geometry"
  )

  add_test(NAME elem_graph.cpp.selection COMMAND elem_graph_cpp_tests "[selection]")
  add_test(NAME elem_graph.cpp.refine COMMAND elem_graph_cpp_tests "[refine]")
  add_test(NAME elem_graph.cpp.se2 COMMAND elem_graph_cpp_tests "[se2]")
  add_test(NAME elem_graph.cpp.contact COMMAND elem_graph_cpp_tests "[contact_relation]")
  add_test(NAME elem_graph.cpp.decision COMMAND elem_graph_cpp_tests "[decision_arena],[motif_base],[node_signature]")
endif()
