#!/usr/bin/env python3
"""Q322: standalone explorer microbench (raycast / voronoi / erosion / cloud)."""

import time

from shapely import box

from nest_graph.config import ProposeConfig
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.placements_free_space_cloud import (
    propose_placements_free_space_cloud,
)
from nest_graph.propose.placements_geo import (
    propose_placements_raycasting,
    propose_placements_voronoi,
)
from nest_graph.propose.placements_primary import propose_placements_erosion


def _bench_ray(region, propose_geom, pt, n=3):
    sheet = box(0, 0, 100, 100)
    t0 = time.perf_counter()
    for _ in range(n):
        propose_placements_raycasting(
            region, sheet, sheet, 0.1,
            propose_geom=propose_geom, pt_push=pt, top_n=8,
        )
    ms = (time.perf_counter() - t0) * 1000.0 / n
    return ms, float(getattr(propose_geom, "_last_raycast_ms", 0.0) or 0.0)


def main() -> None:
    region = box(10, 10, 90, 90)
    sheet = box(0, 0, 100, 100)
    pt = region.centroid
    cfg = ProposeConfig()
    geom = ProposeGeometry(sheet, box(20, 20, 40, 40), box(0, 0, 5, 5), 0.1, propose_cfg=cfg)

    ray_ms, propose_ray_ms = _bench_ray(region, geom, pt)
    print(f"raycast_ms={ray_ms:.2f} propose_ray_ms={propose_ray_ms:.2f}")

    t0 = time.perf_counter()
    propose_placements_voronoi(
        region, sheet, sheet, 0.1,
        propose_geom=geom, pt_push=pt, top_n=8,
        use_free_region=True, border_focus=False,
        focal_shape=None, num_angles=8,
    )
    print(f"voronoi_ms={(time.perf_counter() - t0) * 1000.0:.2f}")

    t1 = time.perf_counter()
    propose_placements_erosion(
        region, sheet, sheet, 0.1,
        propose_geom=geom, pt_push=pt, top_n=8,
        use_free_region=True, border_focus=False,
        focal_shape=None, num_angles=8,
    )
    print(f"erosion_ms={(time.perf_counter() - t1) * 1000.0:.2f}")

    t2 = time.perf_counter()
    propose_placements_free_space_cloud(
        region, propose_geom=geom, propose_cfg=cfg, top_n=8,
    )
    print(f"cloud_ms={(time.perf_counter() - t2) * 1000.0:.2f}")
    print(f"from_shapely_count={int(getattr(geom, '_last_from_shapely_count', 0) or 0)}")


if __name__ == "__main__":
    main()
