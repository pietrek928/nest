#include "bindings.h"

#include "bind_internal.h"

void bind_geometry(nb::module_ &m) {
    bind_geometry_class(m);
    bind_guidance_types(m);
    bind_distance_types(m);
    bind_cast_types(m);
    bind_intersect_api(m);
    bind_distance_api(m);
    bind_cast_api(m);
    bind_guidance_api(m);
    bind_batch_api(m);
}
