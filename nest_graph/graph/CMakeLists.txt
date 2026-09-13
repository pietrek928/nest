set(GRAPH_CORE_SOURCES
  internal/internal.cc
  rules/rules.cc
  pose/pose_index.cc
  scoring/scoring.cc
  selection/selection.cc
  selection/evaluate_rules.cc
  refine/refine_dfs.cc
  refine/finalize.cc
  refine/apply_dfs.cc
)

add_library(graph_core STATIC ${GRAPH_CORE_SOURCES})
set_target_properties(graph_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(graph_core PUBLIC nest_graph_headers nest_graph_warnings)
target_include_directories(graph_core PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}"
)

nanobind_add_module(graph
  bindings/module.cc
  bindings/types.cc
  bindings/api.cc
  bindings/bind_decision.cc
  bindings/bind_mcts.cc
)
target_link_libraries(graph PRIVATE graph_core nest_graph_warnings)
target_include_directories(graph PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/bindings"
)
if(DEFINED NEST_NANOBIND_SYSTEM_INCLUDE)
  target_include_directories(graph SYSTEM PRIVATE "${NEST_NANOBIND_SYSTEM_INCLUDE}")
endif()

if(DEFINED ENV{NEST_GRAPH_NATIVE})
  target_compile_options(graph PRIVATE -march=native)
  target_compile_options(graph_core PRIVATE -march=native)
endif()
target_compile_options(graph PRIVATE -O2 -g)
target_compile_options(graph_core PRIVATE -O2 -g)

set_target_properties(graph PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/nest_graph"
)

install(TARGETS graph LIBRARY DESTINATION nest_graph)

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

  add_executable(graph_cpp_tests
    tests/test_selection.cc
    tests/test_refine_dfs.cc
    tests/test_se2.cc
    tests/test_contact_relation.cc
    tests/test_decision_motif.cc
    tests/test_decision_graph.cc
  )
  target_link_libraries(graph_cpp_tests PRIVATE
    graph_core
    Catch2::Catch2WithMain
    nest_graph_warnings
  )
  target_include_directories(graph_cpp_tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_SOURCE_DIR}/nest_graph"
    "${CMAKE_SOURCE_DIR}/nest_graph/geometry"
  )

  add_test(NAME graph.cpp.selection COMMAND graph_cpp_tests "[selection]")
  add_test(NAME graph.cpp.refine COMMAND graph_cpp_tests "[refine]")
  add_test(NAME graph.cpp.se2 COMMAND graph_cpp_tests "[se2]")
  add_test(NAME graph.cpp.contact COMMAND graph_cpp_tests "[contact_relation]")
  add_test(NAME graph.cpp.decision COMMAND graph_cpp_tests "[decision_arena],[motif_base],[node_signature],[decision_graph]")
endif()
