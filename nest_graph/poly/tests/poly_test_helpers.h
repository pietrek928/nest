#pragma once

#include <cstdlib>
#include <vector>

#include "poly/poly.h"
#include "poly/tracer.h"
#include <vec.h>

using PolyTestVec2 = Vec<2, double>;
using PolyTestPolygon2 = Polygon<PolyTestVec2>;

inline PolyTestPolygon2 make_box(double cx, double cy, double half_w, double half_h) {
    PolyTestPolygon2 poly;
    std::vector<PolyTestVec2> ring{
        PolyTestVec2{{cx - half_w, cy - half_h}},
        PolyTestVec2{{cx + half_w, cy - half_h}},
        PolyTestVec2{{cx + half_w, cy + half_h}},
        PolyTestVec2{{cx - half_w, cy + half_h}},
    };
    poly.append_convex_poly(ring.data(), static_cast<int>(ring.size()));
    poly.finalize();
    return poly;
}

inline void maybe_print_physics_telemetry(const DebugTracer& tracer) {
    if (std::getenv("NEST_PRINT_TELEMETRY")) {
        tracer.print_telemetry();
    }
}
