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
