#include <nanobind/nanobind.h>
namespace nb = nanobind;

#include "bindings.h"


NB_MODULE(geometry, m) {
    m.doc() = "2D polygon geometry (C++ core, nanobind Python bindings)";
    bind_geometry(m);
}
