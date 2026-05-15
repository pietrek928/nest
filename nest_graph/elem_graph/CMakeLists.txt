nanobind_add_module(_elem_graph
  elem_graph_module.cc
  elem_graph.cc
  rules.cc
)
target_link_libraries(_elem_graph PRIVATE nest_graph_headers)
target_include_directories(_elem_graph PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")

if(DEFINED ENV{NEST_GRAPH_NATIVE})
  target_compile_options(_elem_graph PRIVATE -march=native)
endif()
target_compile_options(_elem_graph PRIVATE -O2 -g)

install(TARGETS _elem_graph LIBRARY DESTINATION nest_graph/elem_graph)
