#include <nanobind/nanobind.h>

#include "bindings.h"

namespace nb = nanobind;

NB_MODULE(geometry, m) {
    m.doc() = "2D polygon geometry (C++ core, nanobind Python bindings)";
    bind_geometry(m);
}
