"""Shared parameterized fixtures for nesting quality benchmarks."""

from dataclasses import dataclass

import numpy as np
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

from nest_graph.utils import normalize_poly, transform_poly


@dataclass(frozen=True)
class NestFloors:
    """Pass/fail floors for CI gating."""
    parts_final: int = 0
    area_coverage: float = 0.0
    time_s: float = float("inf")


@dataclass(frozen=True)
class NestCase:
    """A parameterized nesting benchmark scenario."""
    name: str
    board: Polygon
    board_holes: tuple[Polygon, ...]
    groups: tuple[tuple[Polygon, int], ...]
    group_counts: tuple[int, ...]
    seed_placements: tuple[tuple[Polygon, int, tuple[float, float, float]], ...]
    iters: int
    tags: frozenset[str]
    floors: NestFloors

    @property
    def usable_area(self) -> float:
        return float(self.board.area - sum(h.area for h in self.board_holes))

    @property
    def demand_ratio(self) -> float:
        usable = self.usable_area
        if usable <= 0:
            return float("inf")
        part_area = sum(p.area * n for (p, _), n in zip(self.groups, self.group_counts))
        return part_area / usable


def _make_rect(w: float, h: float) -> Polygon:
    return normalize_poly(box(0, 0, w, h))


def _make_tri(w: float, h: float) -> Polygon:
    return normalize_poly(Polygon([(0, 0), (w, 0), (0, h)]))


def _make_l_shape(w: float, h: float, t: float) -> Polygon:
    return normalize_poly(Polygon([(0, 0), (w, 0), (w, t), (t, t), (t, h), (0, h)]))


def _counts_for_demand(
    parts: list[Polygon],
    usable_area: float,
    target_demand: float,
    mix: tuple[float, ...],
    *,
    max_per_group: int = 80,
) -> tuple[int, ...]:
    """Allocate integer counts so sum(area_i * count_i) ≈ target_demand * usable_area."""
    if usable_area <= 0 or target_demand <= 0:
        return tuple(1 for _ in parts)
    weights = np.asarray(mix, dtype=np.float64)
    weights = weights / weights.sum()
    budget = target_demand * usable_area
    counts: list[int] = []
    for part, w in zip(parts, weights):
        n = int(round(budget * w / max(part.area, 1e-12)))
        counts.append(int(min(max_per_group, max(1, n))))
    return tuple(counts)


def _separated_holes(
    board: Polygon,
    num_holes: int,
    hole_radius: float,
    *,
    seed: int,
    min_gap: float,
    max_tries: int = 400,
) -> list[Polygon]:
    """Sample non-overlapping holes with a clearance corridor between them."""
    minx, miny, maxx, maxy = board.bounds
    margin = hole_radius + min_gap
    rng = np.random.default_rng(seed)
    holes: list[Polygon] = []
    for _ in range(max_tries):
        if len(holes) >= num_holes:
            break
        cx = float(rng.uniform(minx + margin, maxx - margin))
        cy = float(rng.uniform(miny + margin, maxy - margin))
        cand = Point(cx, cy).buffer(hole_radius, resolution=3)
        if any(cand.distance(h) < min_gap for h in holes):
            continue
        if not board.contains(cand):
            continue
        holes.append(cand)
    if len(holes) < num_holes:
        raise ValueError(
            f"could only place {len(holes)}/{num_holes} separated holes "
            f"(radius={hole_radius}, min_gap={min_gap})"
        )
    return holes


def make_demo_triangle(scale: float = 1.0, target_demand: float = 1.25) -> NestCase:
    """Fast baseline / regression anchor."""
    board = Polygon([(0, 0), (1.2 * scale, 0), (0, 1.1 * scale)])
    rect = _make_rect(0.1 * scale, 0.1 * scale)
    tri = _make_tri(0.15 * scale, 0.07 * scale)
    counts = _counts_for_demand([rect, tri], board.area, target_demand, (0.55, 0.45))

    return NestCase(
        name=f"demo_triangle_s{scale}",
        board=board,
        board_holes=(),
        groups=((rect, 0), (tri, 1)),
        group_counts=counts,
        seed_placements=(),
        iters=2,
        tags=frozenset(["baseline", "fast"]),
        floors=NestFloors(parts_final=8, area_coverage=0.25),
    )


def make_rect_sheet_mixed(
    board_size: tuple[float, float] = (18.0, 18.0),
    part_scale: float = 1.2,
    target_demand: float = 1.15,
) -> NestCase:
    """Multi-type density on a rectangular sheet."""
    board = _make_rect(*board_size)
    rect = _make_rect(1.0 * part_scale, 1.0 * part_scale)
    tri = _make_tri(1.5 * part_scale, 0.7 * part_scale)
    strip = _make_rect(2.5 * part_scale, 0.35 * part_scale)
    counts = _counts_for_demand(
        [rect, tri, strip], board.area, target_demand, (0.4, 0.35, 0.25),
        max_per_group=90,
    )

    return NestCase(
        name=f"rect_sheet_mixed_c{sum(counts)}",
        board=board,
        board_holes=(),
        groups=((rect, 0), (tri, 1), (strip, 2)),
        group_counts=counts,
        seed_placements=(),
        iters=4,
        tags=frozenset(["dense", "mixed"]),
        floors=NestFloors(parts_final=20, area_coverage=0.35),
    )


def make_donut_void(
    board_size: tuple[float, float] = (12.0, 12.0),
    hole_size: tuple[float, float] = (4.0, 4.0),
    target_demand: float = 1.1,
) -> NestCase:
    """Real hole avoidance, stress-testing void containment."""
    board = _make_rect(*board_size)
    hx, hy = hole_size
    minx, miny, maxx, maxy = board.bounds
    cx = 0.5 * (minx + maxx)
    cy = 0.5 * (miny + maxy)
    hole = box(cx - hx / 2, cy - hy / 2, cx + hx / 2, cy + hy / 2)

    rect = _make_rect(1.1, 1.1)
    tri = _make_tri(1.6, 0.85)
    usable = board.area - hole.area
    counts = _counts_for_demand(
        [rect, tri], usable, target_demand, (0.55, 0.45), max_per_group=90,
    )

    return NestCase(
        name=f"donut_void_h{hx}x{hy}",
        board=board,
        board_holes=(hole,),
        groups=((rect, 0), (tri, 1)),
        group_counts=counts,
        seed_placements=(),
        iters=4,
        tags=frozenset(["void", "stress"]),
        floors=NestFloors(parts_final=12, area_coverage=0.25),
    )


def make_concave_interlock(
    board_size: tuple[float, float] = (16.0, 16.0),
    target_demand: float = 1.2,
) -> NestCase:
    """Concave packing: L brackets with fillers that fit the inner pocket."""
    board = _make_rect(*board_size)
    # Pocket is ~1.0 x 1.0; filler is slightly smaller for clearance.
    l_shape = _make_l_shape(2.0, 2.0, 1.0)
    filler = _make_rect(0.95, 0.95)
    counts = _counts_for_demand(
        [l_shape, filler], board.area, target_demand, (0.65, 0.35),
    )

    return NestCase(
        name=f"concave_interlock_l{counts[0]}",
        board=board,
        board_holes=(),
        groups=((l_shape, 0), (filler, 1)),
        group_counts=counts,
        seed_placements=(),
        iters=4,
        tags=frozenset(["concave", "interlock"]),
        floors=NestFloors(parts_final=10, area_coverage=0.3),
    )


def make_swiss_cheese(
    board_size: tuple[float, float] = (22.0, 22.0),
    num_holes: int = 5,
    hole_radius: float = 1.4,
    target_demand: float = 1.05,
    seed: int = 42,
) -> NestCase:
    """High-complexity void avoidance with separated holes."""
    board = _make_rect(*board_size)
    holes = _separated_holes(
        board,
        num_holes,
        hole_radius,
        seed=seed,
        min_gap=max(1.2, 2.0 * hole_radius * 0.35),
    )
    hole_union = unary_union(holes)
    usable = board.area - hole_union.area

    rect = _make_rect(1.8, 1.8)
    tri = _make_tri(2.4, 1.3)
    usable = board.area - hole_union.area
    counts = _counts_for_demand(
        [rect, tri], usable, target_demand, (0.55, 0.45), max_per_group=90,
    )

    return NestCase(
        name=f"swiss_cheese_h{num_holes}",
        board=board,
        board_holes=tuple(holes),
        groups=((rect, 0), (tri, 1)),
        group_counts=counts,
        seed_placements=(),
        iters=4,
        tags=frozenset(["void", "complex"]),
        floors=NestFloors(parts_final=15, area_coverage=0.25),
    )


def make_border_then_fill(
    board_size: tuple[float, float] = (12.0, 12.0),
    target_demand: float = 1.0,
) -> NestCase:
    """Late-stage packing with pre-filled bottom/left boundary."""
    board = _make_rect(*board_size)
    minx, miny, maxx, maxy = board.bounds

    rect = _make_rect(1.2, 1.2)
    tri = _make_tri(1.8, 0.95)
    half = 0.6
    pitch = 1.8  # clearance between seeds; dense enough for late-stage

    seeds: list[tuple[Polygon, int, tuple[float, float, float]]] = []

    x = minx + half
    while x + half <= maxx:
        t = (x, miny + half, 0.0)
        seeds.append((transform_poly(rect, t), 0, t))
        x += pitch

    y = miny + half + pitch
    while y + half <= maxy:
        t = (minx + half, y, 0.0)
        seeds.append((transform_poly(rect, t), 0, t))
        y += pitch

    counts = _counts_for_demand(
        [rect, tri], board.area, target_demand, (0.5, 0.5), max_per_group=60,
    )

    return NestCase(
        name=f"border_then_fill_s{len(seeds)}",
        board=board,
        board_holes=(),
        groups=((rect, 0), (tri, 1)),
        group_counts=counts,
        seed_placements=tuple(seeds),
        iters=2,
        tags=frozenset(["late_stage", "seeded", "tight_pack"]),
        floors=NestFloors(parts_final=len(seeds) + 2, area_coverage=0.2),
    )


def make_mixed_scale_stress(
    board_size: tuple[float, float] = (24.0, 24.0),
    target_demand: float = 1.0,
) -> NestCase:
    """Scale stress across large / medium / small parts."""
    board = _make_rect(*board_size)
    large_rect = _make_rect(3.0, 3.0)
    med_tri = _make_tri(2.0, 1.2)
    small_rect = _make_rect(0.9, 0.9)
    counts = _counts_for_demand(
        [large_rect, med_tri, small_rect],
        board.area,
        target_demand,
        (0.4, 0.35, 0.25),
        max_per_group=90,
    )

    return NestCase(
        name=f"mixed_scale_stress_p{sum(counts)}",
        board=board,
        board_holes=(),
        groups=((large_rect, 0), (med_tri, 1), (small_rect, 2)),
        group_counts=counts,
        seed_placements=(),
        iters=4,
        tags=frozenset(["stress", "scale"]),
        floors=NestFloors(parts_final=20, area_coverage=0.2),
    )


def get_all_cases() -> list[NestCase]:
    """Return a standard suite of benchmark cases."""
    return [
        make_demo_triangle(),
        make_rect_sheet_mixed(),
        make_donut_void(),
        make_concave_interlock(),
        make_swiss_cheese(),
        make_border_then_fill(),
        make_mixed_scale_stress(),
        make_tight_border_ring(),
        make_dense_cluster_with_pockets(),
        make_incomplete_interlock(),
        make_hole_trap(),
        make_narrow_corridor(),
    ]

def get_tight_pack_cases() -> list[NestCase]:
    """Return cases focused on tight packing and late-stage nesting."""
    return [
        make_border_then_fill(),
        make_tight_border_ring(),
        make_dense_cluster_with_pockets(),
        make_incomplete_interlock(),
    ]

def make_tight_border_ring(
    board_size: tuple[float, float] = (12.0, 12.0),
    target_demand: float = 1.0,
) -> NestCase:
    board = _make_rect(*board_size)
    minx, miny, maxx, maxy = board.bounds

    rect = _make_rect(1.2, 1.2)
    tri = _make_tri(1.8, 0.95)
    half = 0.6
    pitch = 1.8

    seeds = []
    # Bottom
    x = minx + half
    while x + half <= maxx:
        t = (x, miny + half, 0.0)
        seeds.append((transform_poly(rect, t), 0, t))
        x += pitch
    # Top
    x = minx + half
    while x + half <= maxx:
        t = (x, maxy - half, 0.0)
        seeds.append((transform_poly(rect, t), 0, t))
        x += pitch
    # Left
    y = miny + half + pitch
    while y + half <= maxy - pitch:
        t = (minx + half, y, 0.0)
        seeds.append((transform_poly(rect, t), 0, t))
        y += pitch
    # Right
    y = miny + half + pitch
    while y + half <= maxy - pitch:
        t = (maxx - half, y, 0.0)
        seeds.append((transform_poly(rect, t), 0, t))
        y += pitch

    counts = _counts_for_demand(
        [rect, tri], board.area, target_demand, (0.5, 0.5), max_per_group=60,
    )
    return NestCase(
        name=f"tight_border_ring_s{len(seeds)}",
        board=board,
        board_holes=(),
        groups=((rect, 0), (tri, 1)),
        group_counts=counts,
        seed_placements=tuple(seeds),
        iters=2,
        tags=frozenset(["late_stage", "tight_pack", "seeded"]),
        floors=NestFloors(parts_final=len(seeds) + 2, area_coverage=0.25),
    )

def make_dense_cluster_with_pockets(
    board_size: tuple[float, float] = (16.0, 16.0),
    target_demand: float = 1.1,
) -> NestCase:
    board = _make_rect(*board_size)
    minx, miny, maxx, maxy = board.bounds
    cx, cy = (minx + maxx) / 2, (miny + maxy) / 2
    
    rect = _make_rect(1.5, 1.5)
    small_rect = _make_rect(0.8, 0.8)
    pitch = 1.6
    
    seeds = []
    # Create a 3x3 grid with the center missing (pocket)
    for i in range(-1, 2):
        for j in range(-1, 2):
            if i == 0 and j == 0:
                continue # pocket
            t = (cx + i * pitch, cy + j * pitch, 0.0)
            seeds.append((transform_poly(rect, t), 0, t))

    counts = _counts_for_demand([rect, small_rect], board.area, target_demand, (0.4, 0.6))
    return NestCase(
        name=f"dense_cluster_pockets_s{len(seeds)}",
        board=board,
        board_holes=(),
        groups=((rect, 0), (small_rect, 1)),
        group_counts=counts,
        seed_placements=tuple(seeds),
        iters=2,
        tags=frozenset(["late_stage", "tight_pack", "seeded"]),
        floors=NestFloors(parts_final=len(seeds) + 2, area_coverage=0.15),
    )

def make_incomplete_interlock(
    board_size: tuple[float, float] = (12.0, 12.0),
    target_demand: float = 1.0,
) -> NestCase:
    board = _make_rect(*board_size)
    minx, miny, maxx, maxy = board.bounds
    cx, cy = (minx + maxx) / 2, (miny + maxy) / 2
    
    l_shape = _make_l_shape(2.0, 2.0, 0.8)
    small_rect = _make_rect(0.8, 0.8) # Fits in the L pocket
    
    seeds = []
    # Two L shapes interlocking with a gap
    t1 = (cx - 0.5, cy - 0.5, 0.0)
    t2 = (cx + 0.5, cy + 0.5, np.pi)
    seeds.append((transform_poly(l_shape, t1), 0, t1))
    seeds.append((transform_poly(l_shape, t2), 0, t2))

    counts = _counts_for_demand([l_shape, small_rect], board.area, target_demand, (0.5, 0.5))
    return NestCase(
        name=f"incomplete_interlock_s{len(seeds)}",
        board=board,
        board_holes=(),
        groups=((l_shape, 0), (small_rect, 1)),
        group_counts=counts,
        seed_placements=tuple(seeds),
        iters=2,
        tags=frozenset(["late_stage", "tight_pack", "seeded", "concave"]),
        floors=NestFloors(parts_final=len(seeds) + 1, area_coverage=0.08),
    )

def make_hole_trap(
    board_size: tuple[float, float] = (16.0, 16.0),
    target_demand: float = 1.0,
) -> NestCase:
    board = _make_rect(*board_size)
    minx, miny, maxx, maxy = board.bounds
    cx, cy = (minx + maxx) / 2, (miny + maxy) / 2
    
    # Massive hole in the middle
    hole = _make_rect(10.0, 10.0)
    hole = transform_poly(hole, (cx, cy, 0.0))
    
    rect = _make_rect(1.0, 1.0)
    small_rect = _make_rect(0.5, 0.5)
    
    usable_area = board.area - hole.area
    counts = _counts_for_demand([rect, small_rect], usable_area, target_demand, (0.5, 0.5))
    
    return NestCase(
        name=f"hole_trap_p{sum(counts)}",
        board=board,
        board_holes=(hole,),
        groups=((rect, 0), (small_rect, 1)),
        group_counts=counts,
        seed_placements=(),
        iters=4,
        tags=frozenset(["corner_case", "void"]),
        floors=NestFloors(parts_final=10, area_coverage=0.1),
    )

def make_narrow_corridor(
    board_size: tuple[float, float] = (16.0, 16.0),
    target_demand: float = 1.0,
) -> NestCase:
    board = _make_rect(*board_size)
    minx, miny, maxx, maxy = board.bounds
    cx, cy = (minx + maxx) / 2, (miny + maxy) / 2
    
    # Two large holes leaving a narrow corridor in the middle
    hole1 = _make_rect(14.0, 6.0)
    hole1 = transform_poly(hole1, (cx, cy - 4.0, 0.0))
    hole2 = _make_rect(14.0, 6.0)
    hole2 = transform_poly(hole2, (cx, cy + 4.0, 0.0))
    
    rect = _make_rect(1.0, 1.0)
    
    usable_area = board.area - hole1.area - hole2.area
    counts = _counts_for_demand([rect], usable_area, target_demand, (1.0,))
    
    return NestCase(
        name=f"narrow_corridor_p{sum(counts)}",
        board=board,
        board_holes=(hole1, hole2),
        groups=((rect, 0),),
        group_counts=counts,
        seed_placements=(),
        iters=4,
        tags=frozenset(["corner_case", "void"]),
        floors=NestFloors(parts_final=5, area_coverage=0.1),
    )
