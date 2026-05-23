nanobind_add_module(elem_graph
  bindings/module.cc
  bindings/types.cc
  bindings/api.cc
  internal/internal.cc
  rules/rules.cc
  graph/graph_index.cc
  scoring/scoring.cc
  selection/selection.cc
  selection/evaluate_rules.cc
  refine/refine_dfs.cc
)
target_link_libraries(elem_graph PRIVATE nest_graph_headers)
target_include_directories(elem_graph PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/bindings"
)

if(DEFINED ENV{NEST_GRAPH_NATIVE})
  target_compile_options(elem_graph PRIVATE -march=native)
endif()
target_compile_options(elem_graph PRIVATE -O2 -g)

set_target_properties(elem_graph PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/nest_graph"
)

install(TARGETS elem_graph LIBRARY DESTINATION nest_graph)
