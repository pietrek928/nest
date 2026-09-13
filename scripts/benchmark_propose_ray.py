#!/usr/bin/env python3
"""Q274-R0 / Q320: microbench scalar vs batch clip_ray_interior."""

import math
import time

from shapely import LineString, box

from nest_graph.geometry import Geometry


def main() -> None:
    region = box(0, 0, 100, 100)
    region_g = Geometry.from_shapely(region)
    anchors = [(10.0, 10.0), (50.0, 50.0), (90.0, 10.0)]
    n_rays = 36
    ray_len = 120.0
    fracs = (0.1, 0.5)

    t0 = time.perf_counter()
    shapely_n = 0
    for ax, ay in anchors:
        for i in range(n_rays):
            ang = 2.0 * math.pi * i / n_rays
            ex = ax + ray_len * math.cos(ang)
            ey = ay + ray_len * math.sin(ang)
            seg = LineString([(ax, ay), (ex, ey)]).intersection(region)
            if not seg.is_empty:
                shapely_n += 1
    shapely_ms = (time.perf_counter() - t0) * 1000.0

    origins: list[tuple[float, float]] = []
    directions: list[tuple[float, float]] = []
    for ax, ay in anchors:
        for i in range(n_rays):
            ang = 2.0 * math.pi * i / n_rays
            origins.append((ax, ay))
            directions.append((ray_len * math.cos(ang), ray_len * math.sin(ang)))

    t1 = time.perf_counter()
    native_n = 0
    for (ax, ay), (dx, dy) in zip(origins, directions, strict=True):
        pts = region_g.clip_ray_interior((ax, ay), (dx, dy), ray_len, fracs)
        native_n += len(pts)
    native_ms = (time.perf_counter() - t1) * 1000.0

    t2 = time.perf_counter()
    coords_flat, offsets = region_g.clip_ray_interior_batch(
        origins, directions, ray_len, fracs,
    )
    batch_n = int(offsets[-1]) if offsets else 0
    batch_ms = (time.perf_counter() - t2) * 1000.0

    print(f"shapely_ms={shapely_ms:.2f} hits={shapely_n}")
    print(f"native_ms={native_ms:.2f} samples={native_n}")
    print(f"batch_ms={batch_ms:.2f} samples={batch_n}")
    print(f"from_shapely_count=1")
    _ = coords_flat


if __name__ == "__main__":
    main()
