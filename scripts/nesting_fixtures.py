"""Shared parameterized fixtures for nesting quality benchmarks."""

import hashlib
import math
import re
from dataclasses import dataclass, field, replace

import numpy as np
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

from nest_graph.utils import normalize_poly, transform_poly


@dataclass(frozen=True)
class NestFloors:
    """Pass/fail floors for CI gating.

    Kiss metrics (see evaluator):
    - kiss_seed: fraction of *newly placed* parts within ~standoff of a seed.
      Saturated rim cases often have no residual seed-facing gaps → kiss_seed≈0
      is not alone a failure; prefer kiss_standoff / ΔParts / area_coverage.
    - kiss_outline: fraction flush to the nest outline.
    - kiss_standoff: fraction near ideal clearance to border or packed groups.
    """

    parts_final: int = 0
    area_coverage: float = 0.0
    time_s: float = float("inf")
    kiss_seed: float = 0.0
    kiss_outline: float = 0.0
    kiss_standoff: float = 0.0
    outline_coverage: float = 0.0
    density_auc: float = 0.0
    # Optional max floors (fail if metric exceeds); unset = Infinity.
    largest_free_over_part: float = float("inf")
    clearance_p50: float = float("inf")


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
    group_allowed_angles: tuple[tuple[float, ...] | None, ...] = ()
    family: str = ""

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


@dataclass(frozen=True)
class CaseSpec:
    """Knob vector used to build NestCase instances deterministically."""

    family: str
    board: str = "rect"
    board_params: tuple = (12.0, 12.0)
    hole_layout: str = "none"
    hole_params: tuple = ()
    part_catalog: tuple[str, ...] = ("rect", "tri")
    part_scales: tuple[float, ...] = (1.0, 1.0)
    group_allowed_angles: tuple[tuple[float, ...] | None, ...] = ()
    seed_layout: str = "none"
    seed_params: tuple = ()
    target_demand: float = 1.0
    iters: int = 2
    max_per_group: int = 80
    tags: frozenset[str] = field(default_factory=frozenset)
    mix: tuple[float, ...] = ()
    floors: NestFloors = field(default_factory=NestFloors)


# ---------------------------------------------------------------------------
# Shape / board builders
# ---------------------------------------------------------------------------


def _make_rect(w: float, h: float) -> Polygon:
    return normalize_poly(box(0, 0, w, h))


def _make_tri(w: float, h: float) -> Polygon:
    return normalize_poly(Polygon([(0, 0), (w, 0), (0, h)]))


def _make_l_shape(w: float, h: float, t: float) -> Polygon:
    return normalize_poly(Polygon([(0, 0), (w, 0), (w, t), (t, t), (t, h), (0, h)]))


def _make_c_shape(w: float = 2.0, h: float = 2.0, t: float = 0.5) -> Polygon:
    return normalize_poly(
        Polygon([
            (0, 0), (w, 0), (w, t), (t, t), (t, h - t), (w, h - t), (w, h), (0, h),
        ])
    )


def _make_u_shape(w: float = 2.0, h: float = 2.0, t: float = 0.5) -> Polygon:
    return normalize_poly(
        Polygon([
            (0, 0), (w, 0), (w, h), (w - t, h), (w - t, t), (t, t), (t, h), (0, h),
        ])
    )


def _make_t_shape(w: float = 2.0, h: float = 2.0, t: float = 0.6) -> Polygon:
    stem = (w - t) / 2.0
    merged = unary_union([box(0, h - t, w, h), box(stem, 0, stem + t, h - t)])
    if merged.geom_type != "Polygon":
        merged = max(merged.geoms, key=lambda g: g.area)
    return normalize_poly(merged)


def _make_star(r_outer: float = 1.0, r_inner: float = 0.4, points: int = 5) -> Polygon:
    pts = []
    for i in range(points * 2):
        r = r_outer if i % 2 == 0 else r_inner
        a = math.pi * i / points - math.pi / 2
        pts.append((r * math.cos(a), r * math.sin(a)))
    return normalize_poly(Polygon(pts))


def _make_cross(arm: float = 2.0, t: float = 0.6) -> Polygon:
    c = arm / 2.0
    ht = t / 2.0
    return normalize_poly(
        Polygon([
            (-ht, -c), (ht, -c), (ht, -ht), (c, -ht), (c, ht), (ht, ht),
            (ht, c), (-ht, c), (-ht, ht), (-c, ht), (-c, -ht), (-ht, -ht),
        ])
    )


def _make_donut_part(outer: float = 1.5, inner: float = 0.6) -> Polygon:
    shell = list(Point(0, 0).buffer(outer, resolution=4).exterior.coords)
    hole = list(Point(0, 0).buffer(inner, resolution=4).exterior.coords)
    return normalize_poly(Polygon(shell, [hole]))


def _make_frame(w: float = 2.0, h: float = 2.0, t: float = 0.35) -> Polygon:
    shell = [(0, 0), (w, 0), (w, h), (0, h)]
    hole = [(t, t), (w - t, t), (w - t, h - t), (t, h - t)]
    return normalize_poly(Polygon(shell, [hole]))


def _make_needle(length: float = 4.0, thickness: float = 0.25) -> Polygon:
    return normalize_poly(box(0, 0, length, thickness))


def _make_wedge(w: float = 2.0, h: float = 0.6) -> Polygon:
    return normalize_poly(Polygon([(0, 0), (w, 0), (w * 0.5, h)]))


def _make_organic_board(
    size: float = 16.0,
    n_verts: int = 18,
    noise: float = 0.22,
    seed: int = 7,
) -> Polygon:
    """Deterministic jagged simple polygon (hide/leather-like outline)."""
    rng = np.random.default_rng(seed)
    cx = cy = size * 0.5
    base_r = size * 0.42
    pts = []
    for i in range(n_verts):
        a = 2.0 * math.pi * i / n_verts
        r = base_r * (1.0 + float(rng.uniform(-noise, noise)))
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    poly = Polygon(pts)
    if not poly.is_valid or poly.area <= 0:
        poly = poly.buffer(0)
    if poly.is_empty or not poly.is_valid:
        raise ValueError("organic board generation failed")
    if poly.geom_type == "MultiPolygon":
        poly = max(poly.geoms, key=lambda g: g.area)
    return poly


PART_BUILDERS = {
    "rect": lambda s=1.0, **_: _make_rect(1.0 * s, 1.0 * s),
    "tri": lambda s=1.0, **_: _make_tri(1.5 * s, 0.7 * s),
    "strip": lambda s=1.0, **_: _make_rect(2.5 * s, 0.35 * s),
    "L": lambda s=1.0, **_: _make_l_shape(2.0 * s, 2.0 * s, 0.8 * s),
    "C": lambda s=1.0, **_: _make_c_shape(2.0 * s, 2.0 * s, 0.5 * s),
    "U": lambda s=1.0, **_: _make_u_shape(2.0 * s, 2.0 * s, 0.5 * s),
    "T": lambda s=1.0, **_: _make_t_shape(2.0 * s, 2.0 * s, 0.6 * s),
    "star": lambda s=1.0, **_: _make_star(1.0 * s, 0.4 * s),
    "cross": lambda s=1.0, **_: _make_cross(2.0 * s, 0.6 * s),
    "donut": lambda s=1.0, **_: _make_donut_part(1.4 * s, 0.55 * s),
    "frame": lambda s=1.0, **_: _make_frame(2.0 * s, 2.0 * s, 0.35 * s),
    "needle": lambda s=1.0, **_: _make_needle(4.0 * s, 0.25 * s),
    "wedge": lambda s=1.0, **_: _make_wedge(2.0 * s, 0.55 * s),
    "small_rect": lambda s=1.0, **_: _make_rect(0.8 * s, 0.8 * s),
    "filler": lambda s=1.0, **_: _make_rect(0.95 * s, 0.95 * s),
}


def _build_board(kind: str, params: tuple) -> Polygon:
    if kind == "rect":
        w, h = (params + (12.0, 12.0))[:2]
        return box(0, 0, float(w), float(h))
    if kind == "tri":
        scale = float(params[0]) if params else 1.0
        return Polygon([(0, 0), (1.2 * scale, 0), (0, 1.1 * scale)])
    if kind == "organic":
        size = float(params[0]) if params else 16.0
        n_verts = int(params[1]) if len(params) > 1 else 18
        noise = float(params[2]) if len(params) > 2 else 0.22
        seed = int(params[3]) if len(params) > 3 else 7
        return _make_organic_board(size=size, n_verts=n_verts, noise=noise, seed=seed)
    raise ValueError(f"unknown board kind: {kind}")


def _build_holes(board: Polygon, layout: str, params: tuple) -> tuple[Polygon, ...]:
    if layout in ("", "none"):
        return ()
    minx, miny, maxx, maxy = board.bounds
    cx, cy = 0.5 * (minx + maxx), 0.5 * (miny + maxy)
    if layout == "center":
        hx = float(params[0]) if params else 4.0
        hy = float(params[1]) if len(params) > 1 else hx
        return (box(cx - hx / 2, cy - hy / 2, cx + hx / 2, cy + hy / 2),)
    if layout == "swiss":
        n = int(params[0]) if params else 5
        r = float(params[1]) if len(params) > 1 else 1.4
        seed = int(params[2]) if len(params) > 2 else 42
        gap = float(params[3]) if len(params) > 3 else max(1.2, 0.7 * r)
        return tuple(_separated_holes(board, n, r, seed=seed, min_gap=gap))
    if layout == "trap":
        hw = float(params[0]) if params else 10.0
        hh = float(params[1]) if len(params) > 1 else hw
        hole = transform_poly(_make_rect(hw, hh), (cx, cy, 0.0))
        return (hole,)
    if layout == "corridor":
        hw = float(params[0]) if params else 14.0
        hh = float(params[1]) if len(params) > 1 else 6.0
        dy = float(params[2]) if len(params) > 2 else 4.0
        h1 = transform_poly(_make_rect(hw, hh), (cx, cy - dy, 0.0))
        h2 = transform_poly(_make_rect(hw, hh), (cx, cy + dy, 0.0))
        return (h1, h2)
    raise ValueError(f"unknown hole_layout: {layout}")


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
    weights = np.asarray(mix if mix else (1.0,) * len(parts), dtype=np.float64)
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


def _seed_min_dist(board: Polygon) -> float:
    minx, miny, maxx, maxy = board.bounds
    diag = float(np.hypot(maxx - minx, maxy - miny))
    return max(diag * 0.008, 1e-3)


def _seed_clear(
    candidate: Polygon,
    seeds: list[tuple[Polygon, int, tuple[float, float, float]]],
    holes: tuple[Polygon, ...],
    min_dist: float,
) -> bool:
    if any(candidate.distance(h) < min_dist for h in holes):
        return False
    return all(candidate.distance(s[0]) >= min_dist * 0.5 for s in seeds)


def _build_seeds(
    board: Polygon,
    holes: tuple[Polygon, ...],
    parts: list[Polygon],
    layout: str,
    params: tuple,
) -> list[tuple[Polygon, int, tuple[float, float, float]]]:
    if layout in ("", "none"):
        return []
    minx, miny, maxx, maxy = board.bounds
    cx, cy = 0.5 * (minx + maxx), 0.5 * (miny + maxy)
    md = _seed_min_dist(board)
    seeds: list[tuple[Polygon, int, tuple[float, float, float]]] = []
    part0 = parts[0]

    def try_add(
        gid: int,
        t: tuple[float, float, float],
        poly: Polygon | None = None,
        *,
        check_seed_clearance: bool = True,
    ) -> None:
        p = poly or parts[gid]
        placed = transform_poly(p, t)
        if not board.intersects(placed):
            return
        if any(placed.distance(h) < md for h in holes):
            return
        if check_seed_clearance and not all(
            placed.distance(s[0]) >= md * 0.5 for s in seeds
        ):
            return
        seeds.append((placed, gid, t))

    if layout == "l_border":
        half = float(params[0]) if params else 0.6
        pitch = float(params[1]) if len(params) > 1 else 1.8
        x = minx + half
        while x + half <= maxx:
            try_add(0, (x, miny + half, 0.0))
            x += pitch
        y = miny + half + pitch
        while y + half <= maxy:
            try_add(0, (minx + half, y, 0.0))
            y += pitch
    elif layout == "full_ring":
        half = float(params[0]) if params else 0.6
        pitch = float(params[1]) if len(params) > 1 else 1.8
        x = minx + half
        while x + half <= maxx:
            try_add(0, (x, miny + half, 0.0))
            try_add(0, (x, maxy - half, 0.0))
            x += pitch
        y = miny + half + pitch
        while y + half <= maxy - pitch:
            try_add(0, (minx + half, y, 0.0))
            try_add(0, (maxx - half, y, 0.0))
            y += pitch
    elif layout == "pocket_grid":
        pitch = float(params[0]) if params else 1.6
        for i in range(-1, 2):
            for j in range(-1, 2):
                if i == 0 and j == 0:
                    continue
                try_add(0, (cx + i * pitch, cy + j * pitch, 0.0))
    elif layout == "interlock_pair":
        dx = float(params[0]) if params else 0.5
        dy = float(params[1]) if len(params) > 1 else 0.5
        # Intentional near-contact motif: skip mutual seed clearance.
        try_add(0, (cx - dx, cy - dy, 0.0), check_seed_clearance=False)
        try_add(0, (cx + dx, cy + dy, float(np.pi)), check_seed_clearance=False)
    elif layout == "two_island":
        pitch = float(params[0]) if params else 1.5
        ox = float(params[1]) if len(params) > 1 else (maxx - minx) * 0.28
        for i in range(2):
            for j in range(2):
                try_add(0, (minx + ox + i * pitch, miny + ox + j * pitch, 0.0))
                try_add(0, (maxx - ox - i * pitch, maxy - ox - j * pitch, 0.0))
    elif layout == "hole_mouth":
        if not holes:
            return []
        hole = holes[0]
        hx0, hy0, hx1, hy1 = hole.bounds
        left = 0.5 * (minx + hx0)
        right = 0.5 * (maxx + hx1)
        bottom = 0.5 * (miny + hy0)
        top = 0.5 * (maxy + hy1)
        pitch = float(params[1]) if len(params) > 1 else 1.6
        x = left
        while x <= right + 1e-9:
            try_add(0, (x, bottom, 0.0))
            try_add(0, (x, top, 0.0))
            x += pitch
        y = bottom + pitch
        while y <= top - pitch + 1e-9:
            try_add(0, (left, y, 0.0))
            try_add(0, (right, y, 0.0))
            y += pitch
    elif layout == "corridor_ends":
        half = float(params[0]) if params else 0.55
        try_add(0, (minx + half * 2, cy, 0.0))
        try_add(0, (maxx - half * 2, cy, 0.0))
    else:
        raise ValueError(f"unknown seed_layout: {layout}")
    return seeds


_SLUG_RE = re.compile(r"[^a-z0-9_.-]+")


def case_slug(spec: CaseSpec, *, n_seeds: int = 0, n_parts: int = 0) -> str:
    parts = "".join(spec.part_catalog)
    grain = ""
    if any(a is not None for a in spec.group_allowed_angles):
        bits = []
        for a in spec.group_allowed_angles:
            if a is None:
                bits.append("u")
            else:
                bits.append("-".join(str(int(round(math.degrees(x) % 360))) for x in a))
        grain = "_g" + "_".join(bits)
    raw = (
        f"{spec.family}_b{spec.board}_{spec.hole_layout}_{parts}"
        f"_d{spec.target_demand:g}_{spec.seed_layout}"
        f"{grain}_s{n_seeds}_p{n_parts}"
    )
    raw = _SLUG_RE.sub("-", raw.lower()).strip("-")
    if len(raw) <= 80:
        return raw
    digest = hashlib.sha1(raw.encode()).hexdigest()[:6]
    return f"{raw[:73]}_{digest}"


def build_nest_case(spec: CaseSpec, *, name: str | None = None) -> NestCase:
    """Build a NestCase from knobs; demand/seed clearance invariants enforced."""
    board = _build_board(spec.board, spec.board_params)
    holes = _build_holes(board, spec.hole_layout, spec.hole_params)
    if len(spec.part_catalog) != len(spec.part_scales):
        raise ValueError("part_catalog and part_scales length mismatch")
    parts = [
        PART_BUILDERS[name](s=scale)
        for name, scale in zip(spec.part_catalog, spec.part_scales)
    ]
    usable = float(board.area - sum(h.area for h in holes))
    mix = spec.mix if spec.mix else tuple(1.0 / len(parts) for _ in parts)
    demand = float(np.clip(spec.target_demand, 0.5, 1.6))
    counts = _counts_for_demand(
        parts, usable, demand, mix, max_per_group=spec.max_per_group,
    )
    seeds = _build_seeds(board, holes, parts, spec.seed_layout, spec.seed_params)
    groups = tuple((p, i) for i, p in enumerate(parts))
    angles = spec.group_allowed_angles
    if angles and len(angles) != len(groups):
        raise ValueError("group_allowed_angles must match groups length")
    slug = name or case_slug(spec, n_seeds=len(seeds), n_parts=sum(counts))
    case = NestCase(
        name=slug,
        board=board,
        board_holes=holes,
        groups=groups,
        group_counts=counts,
        seed_placements=tuple(seeds),
        iters=spec.iters,
        tags=frozenset(spec.tags),
        floors=spec.floors,
        group_allowed_angles=angles,
        family=spec.family,
    )
    if not (0.5 <= case.demand_ratio <= 1.6):
        raise ValueError(
            f"demand_ratio {case.demand_ratio:.3f} out of [0.5, 1.6] for {slug} "
            f"(counts={counts}, usable={usable:.1f}); raise max_per_group or part area"
        )
    return case


def resolve_cases(
    queries: list[str] | None = None,
    *,
    tags: list[str] | None = None,
    family: str | None = None,
    all_cases: list[NestCase] | None = None,
) -> list[NestCase]:
    """Resolve cases by exact name, family prefix, or tag."""
    pool = all_cases if all_cases is not None else get_all_cases()
    if family:
        pool = [c for c in pool if c.family == family or c.name.startswith(family)]
    if tags:
        wanted = set(tags)
        pool = [c for c in pool if wanted & set(c.tags)]
    if not queries:
        return pool
    out: list[NestCase] = []
    by_name = {c.name: c for c in get_all_cases()}
    for q in queries:
        if q in by_name:
            out.append(by_name[q])
            continue
        matches = [
            c for c in get_all_cases()
            if c.name.startswith(q) or c.family == q or q in c.tags
        ]
        if not matches:
            names = sorted(by_name)
            hint = ", ".join(names[:8])
            raise ValueError(f"no case matched {q!r}; known e.g. {hint}")
        out.extend(matches)
    # de-dupe preserving order
    seen: set[str] = set()
    uniq: list[NestCase] = []
    for c in out:
        if c.name in seen:
            continue
        seen.add(c.name)
        uniq.append(c)
    return uniq


# ---------------------------------------------------------------------------
# Curated case factories (stable names for existing suite)
# ---------------------------------------------------------------------------


def make_demo_triangle(scale: float = 1.0, target_demand: float = 1.25) -> NestCase:
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
        family="demo_triangle",
    )


def make_demo_triangle_corner_cluster(
    scale: float = 1.0,
    target_demand: float = 1.2,
) -> NestCase:
    """Loose cluster near the right-angle corner; large hypotenuse void (Mode A)."""
    board = Polygon([(0, 0), (1.2 * scale, 0), (0, 1.1 * scale)])
    rect = _make_rect(0.1 * scale, 0.1 * scale)
    tri = _make_tri(0.15 * scale, 0.07 * scale)
    counts = _counts_for_demand([rect, tri], board.area, target_demand, (0.55, 0.45))
    # Pack a few parts near (0,0); leave the long free region along the hypotenuse.
    pitch = 0.13 * scale
    half = 0.05 * scale
    seeds: list[tuple[Polygon, int, tuple[float, float, float]]] = []
    for i, j in ((0, 0), (1, 0), (0, 1), (2, 0), (0, 2), (1, 1)):
        t = (half + i * pitch, half + j * pitch, 0.0)
        placed = transform_poly(rect, t)
        if board.contains(placed) or board.intersects(placed):
            seeds.append((placed, 0, t))
    seed_cov = sum(s[0].area for s in seeds) / max(board.area, 1e-12)
    return NestCase(
        name=f"demo_triangle_corner_cluster_s{len(seeds)}",
        board=board,
        board_holes=(),
        groups=((rect, 0), (tri, 1)),
        group_counts=counts,
        seed_placements=tuple(seeds),
        iters=6,
        floors=NestFloors(
            parts_final=len(seeds) + 8,
            # E0 raise-gate under Q69 / G1: scrap area ≥ 0.585.
            area_coverage=max(seed_cov + 0.08, 0.585),
            time_s=170.0,
            # Void shrinks but need not vanish in a short mid-pack run.
            largest_free_over_part=60.0,
        ),
        tags=frozenset(["mid_pack", "void_fill", "seeded"]),
        family="demo_triangle_corner_cluster",
    )


def make_loose_cluster_compact(
    board_size: tuple[float, float] = (12.0, 12.0),
    target_demand: float = 1.0,
) -> NestCase:
    """Deliberately gappy L-border seeds for Mode B compaction."""
    case = build_nest_case(
        CaseSpec(
            family="loose_cluster_compact",
            board="rect",
            board_params=board_size,
            part_catalog=("rect", "tri"),
            part_scales=(1.0, 1.0),
            seed_layout="l_border",
            # Wide pitch → loose clearances that compaction should shrink.
            seed_params=(0.55, 2.6),
            target_demand=target_demand,
            iters=3,
            max_per_group=60,
            tags=frozenset(["mid_pack", "compact", "seeded"]),
            mix=(0.55, 0.45),
            floors=NestFloors(parts_final=2, area_coverage=0.12),
        ),
    )
    n_seed = len(case.seed_placements)
    seed_cov = (
        sum(s[0].area for s in case.seed_placements) / max(case.usable_area, 1e-12)
        if n_seed else 0.0
    )
    return replace(
        case,
        name=f"loose_cluster_compact_s{n_seed}",
        floors=NestFloors(
            parts_final=max(n_seed, 2),
            area_coverage=seed_cov + 0.03,
            # clearance_p50 max deferred while gravity compaction is experiment-gated off
        ),
    )


def make_rect_sheet_mixed(
    board_size: tuple[float, float] = (18.0, 18.0),
    part_scale: float = 1.2,
    target_demand: float = 1.15,
) -> NestCase:
    board = box(0, 0, *board_size)
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
        family="rect_sheet_mixed",
    )


def make_donut_void(
    board_size: tuple[float, float] = (12.0, 12.0),
    hole_size: tuple[float, float] = (4.0, 4.0),
    target_demand: float = 1.1,
) -> NestCase:
    return build_nest_case(
        CaseSpec(
            family="donut_void",
            board="rect",
            board_params=board_size,
            hole_layout="center",
            hole_params=hole_size,
            part_catalog=("rect", "tri"),
            part_scales=(1.1, 1.1),
            target_demand=target_demand,
            iters=4,
            max_per_group=90,
            tags=frozenset(["void", "stress"]),
            mix=(0.55, 0.45),
            floors=NestFloors(parts_final=12, area_coverage=0.25),
        ),
        name=f"donut_void_h{hole_size[0]}x{hole_size[1]}",
    )


def make_concave_interlock(
    board_size: tuple[float, float] = (16.0, 16.0),
    target_demand: float = 1.2,
) -> NestCase:
    board = box(0, 0, *board_size)
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
        family="concave_interlock",
    )


def make_swiss_cheese(
    board_size: tuple[float, float] = (22.0, 22.0),
    num_holes: int = 5,
    hole_radius: float = 1.4,
    target_demand: float = 1.05,
    seed: int = 42,
) -> NestCase:
    return build_nest_case(
        CaseSpec(
            family="swiss_cheese",
            board="rect",
            board_params=board_size,
            hole_layout="swiss",
            hole_params=(num_holes, hole_radius, seed),
            part_catalog=("rect", "tri"),
            part_scales=(1.8, 1.6),
            target_demand=target_demand,
            iters=4,
            max_per_group=90,
            tags=frozenset(["void", "complex"]),
            mix=(0.55, 0.45),
            floors=NestFloors(parts_final=15, area_coverage=0.25),
        ),
        name=f"swiss_cheese_h{num_holes}",
    )


def make_border_then_fill(
    board_size: tuple[float, float] = (12.0, 12.0),
    target_demand: float = 1.0,
) -> NestCase:
    case = build_nest_case(
        CaseSpec(
            family="border_then_fill",
            board="rect",
            board_params=board_size,
            part_catalog=("rect", "tri"),
            part_scales=(1.2, 1.2),
            seed_layout="l_border",
            seed_params=(0.6, 1.8),
            target_demand=target_demand,
            iters=2,
            max_per_group=60,
            tags=frozenset(["late_stage", "seeded", "tight_pack", "mid_pack"]),
            mix=(0.5, 0.5),
            floors=NestFloors(parts_final=2, area_coverage=0.2),
        ),
    )
    return replace(
        case,
        name=f"border_then_fill_s{len(case.seed_placements)}",
        floors=NestFloors(
            parts_final=len(case.seed_placements) + 2, area_coverage=0.15,
        ),
    )


def make_mixed_scale_stress(
    board_size: tuple[float, float] = (24.0, 24.0),
    target_demand: float = 1.0,
) -> NestCase:
    board = box(0, 0, *board_size)
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
        family="mixed_scale_stress",
    )


def make_tight_border_ring(
    board_size: tuple[float, float] = (12.0, 12.0),
    target_demand: float = 1.0,
) -> NestCase:
    case = build_nest_case(
        CaseSpec(
            family="tight_border_ring",
            board="rect",
            board_params=board_size,
            part_catalog=("rect", "tri"),
            part_scales=(1.2, 1.2),
            seed_layout="full_ring",
            seed_params=(0.6, 1.8),
            target_demand=target_demand,
            iters=2,
            max_per_group=60,
            tags=frozenset(["late_stage", "tight_pack", "seeded"]),
            mix=(0.5, 0.5),
            floors=NestFloors(parts_final=2, area_coverage=0.25),
        ),
    )
    return replace(
        case,
        name=f"tight_border_ring_s{len(case.seed_placements)}",
        floors=NestFloors(
            parts_final=len(case.seed_placements) + 2, area_coverage=0.25,
        ),
    )


def make_dense_cluster_with_pockets(
    board_size: tuple[float, float] = (16.0, 16.0),
    target_demand: float = 1.1,
) -> NestCase:
    case = build_nest_case(
        CaseSpec(
            family="dense_cluster_pockets",
            board="rect",
            board_params=board_size,
            part_catalog=("rect", "small_rect"),
            part_scales=(1.5, 1.0),
            seed_layout="pocket_grid",
            seed_params=(1.6,),
            target_demand=target_demand,
            iters=2,
            tags=frozenset(["late_stage", "tight_pack", "seeded", "mid_pack"]),
            mix=(0.4, 0.6),
            floors=NestFloors(parts_final=2, area_coverage=0.15),
        ),
    )
    return replace(
        case,
        name=f"dense_cluster_pockets_s{len(case.seed_placements)}",
        floors=NestFloors(
            parts_final=len(case.seed_placements) + 2, area_coverage=0.1,
        ),
    )


def make_incomplete_interlock(
    board_size: tuple[float, float] = (12.0, 12.0),
    target_demand: float = 1.0,
) -> NestCase:
    case = build_nest_case(
        CaseSpec(
            family="incomplete_interlock",
            board="rect",
            board_params=board_size,
            part_catalog=("L", "small_rect"),
            part_scales=(1.0, 1.0),
            seed_layout="interlock_pair",
            seed_params=(0.5, 0.5),
            target_demand=target_demand,
            iters=2,
            tags=frozenset(["late_stage", "tight_pack", "seeded", "concave"]),
            mix=(0.5, 0.5),
            floors=NestFloors(parts_final=1, area_coverage=0.08),
        ),
    )
    return replace(
        case,
        name=f"incomplete_interlock_s{len(case.seed_placements)}",
        floors=NestFloors(
            parts_final=len(case.seed_placements) + 1, area_coverage=0.08,
        ),
    )


def make_hole_trap(
    board_size: tuple[float, float] = (16.0, 16.0),
    target_demand: float = 1.0,
) -> NestCase:
    case = build_nest_case(
        CaseSpec(
            family="hole_trap",
            board="rect",
            board_params=board_size,
            hole_layout="trap",
            hole_params=(10.0, 10.0),
            part_catalog=("rect", "small_rect"),
            part_scales=(1.0, 0.625),
            target_demand=target_demand,
            iters=4,
            tags=frozenset(["corner_case", "void"]),
            mix=(0.5, 0.5),
            floors=NestFloors(parts_final=10, area_coverage=0.1),
        ),
    )
    return replace(case, name=f"hole_trap_p{sum(case.group_counts)}")


def make_narrow_corridor(
    board_size: tuple[float, float] = (16.0, 16.0),
    target_demand: float = 1.0,
) -> NestCase:
    case = build_nest_case(
        CaseSpec(
            family="narrow_corridor",
            board="rect",
            board_params=board_size,
            hole_layout="corridor",
            hole_params=(14.0, 6.0, 4.0),
            part_catalog=("rect",),
            part_scales=(1.0,),
            target_demand=target_demand,
            iters=4,
            tags=frozenset(["corner_case", "void"]),
            mix=(1.0,),
            floors=NestFloors(parts_final=5, area_coverage=0.1),
        ),
    )
    return replace(case, name=f"narrow_corridor_p{sum(case.group_counts)}")


def make_rim_kiss_ring(
    board_size: tuple[float, float] = (12.0, 12.0),
    target_demand: float = 1.05,
) -> NestCase:
    case = build_nest_case(
        CaseSpec(
            family="rim_kiss_ring",
            board="rect",
            board_params=board_size,
            part_catalog=("rect", "wedge"),
            part_scales=(1.2, 1.0),
            seed_layout="full_ring",
            seed_params=(0.6, 1.8),
            target_demand=target_demand,
            iters=2,
            max_per_group=60,
            tags=frozenset(["late_stage", "tight_pack", "seeded", "rim"]),
            mix=(0.45, 0.55),
            floors=NestFloors(parts_final=2, area_coverage=0.15, kiss_outline=0.0),
        ),
    )
    return replace(
        case,
        floors=NestFloors(
            parts_final=len(case.seed_placements) + 1, area_coverage=0.15,
        ),
    )


def make_two_cluster_bridge(
    board_size: tuple[float, float] = (16.0, 16.0),
    target_demand: float = 1.0,
) -> NestCase:
    case = build_nest_case(
        CaseSpec(
            family="two_cluster_bridge",
            board="rect",
            board_params=board_size,
            part_catalog=("rect", "small_rect"),
            part_scales=(1.2, 1.0),
            seed_layout="two_island",
            seed_params=(1.5,),
            target_demand=target_demand,
            iters=2,
            tags=frozenset(["late_stage", "tight_pack", "seeded", "inter_cluster"]),
            mix=(0.4, 0.6),
            floors=NestFloors(parts_final=2, area_coverage=0.12),
        ),
    )
    return replace(
        case,
        floors=NestFloors(
            parts_final=len(case.seed_placements) + 2, area_coverage=0.12,
        ),
    )


def make_hole_mouth_seeded(
    board_size: tuple[float, float] = (14.0, 14.0),
    target_demand: float = 1.0,
) -> NestCase:
    case = build_nest_case(
        CaseSpec(
            family="hole_mouth_seeded",
            board="rect",
            board_params=board_size,
            hole_layout="center",
            hole_params=(4.0, 4.0),
            part_catalog=("rect", "small_rect"),
            part_scales=(1.0, 0.8),
            seed_layout="hole_mouth",
            seed_params=(0.55, 1.6),
            target_demand=target_demand,
            iters=2,
            tags=frozenset(["void", "seeded", "void_seek", "late_stage"]),
            mix=(0.5, 0.5),
            floors=NestFloors(parts_final=2, area_coverage=0.1),
        ),
    )
    return replace(
        case,
        floors=NestFloors(
            parts_final=max(2, len(case.seed_placements)), area_coverage=0.1,
        ),
    )


def make_corridor_seeded(
    board_size: tuple[float, float] = (16.0, 16.0),
    target_demand: float = 1.0,
) -> NestCase:
    case = build_nest_case(
        CaseSpec(
            family="corridor_seeded",
            board="rect",
            board_params=board_size,
            hole_layout="corridor",
            hole_params=(14.0, 6.0, 4.0),
            part_catalog=("rect",),
            part_scales=(1.0,),
            seed_layout="corridor_ends",
            seed_params=(0.55,),
            target_demand=target_demand,
            iters=2,
            tags=frozenset(["void", "seeded", "corner_case", "late_stage"]),
            mix=(1.0,),
            floors=NestFloors(parts_final=2, area_coverage=0.08),
        ),
    )
    return replace(
        case,
        floors=NestFloors(
            parts_final=len(case.seed_placements) + 1, area_coverage=0.08,
        ),
    )


def make_donut_rim(
    board_size: tuple[float, float] = (14.0, 14.0),
    target_demand: float = 1.0,
) -> NestCase:
    case = build_nest_case(
        CaseSpec(
            family="donut_rim",
            board="rect",
            board_params=board_size,
            hole_layout="center",
            hole_params=(5.0, 5.0),
            part_catalog=("rect", "tri"),
            part_scales=(1.0, 1.0),
            seed_layout="full_ring",
            seed_params=(0.55, 1.7),
            target_demand=target_demand,
            iters=2,
            max_per_group=90,
            tags=frozenset(["void", "seeded", "tight_pack", "late_stage"]),
            mix=(0.5, 0.5),
            floors=NestFloors(parts_final=2, area_coverage=0.1),
        ),
    )
    return replace(
        case,
        floors=NestFloors(
            parts_final=len(case.seed_placements) + 1, area_coverage=0.1,
        ),
    )


def make_mixed_irregular(
    board_size: tuple[float, float] = (18.0, 18.0),
    target_demand: float = 1.0,
) -> NestCase:
    return build_nest_case(
        CaseSpec(
            family="mixed_irregular",
            board="rect",
            board_params=board_size,
            part_catalog=("L", "C", "T", "star"),
            part_scales=(1.0, 1.0, 1.0, 1.0),
            target_demand=target_demand,
            iters=3,
            max_per_group=40,
            tags=frozenset(["irregular", "concave", "stress"]),
            mix=(0.3, 0.25, 0.25, 0.2),
            floors=NestFloors(parts_final=8, area_coverage=0.15),
        ),
    )


def make_donut_parts_pack(
    board_size: tuple[float, float] = (16.0, 16.0),
    target_demand: float = 1.05,
) -> NestCase:
    return build_nest_case(
        CaseSpec(
            family="donut_parts_pack",
            board="rect",
            board_params=board_size,
            part_catalog=("frame", "donut", "small_rect"),
            part_scales=(1.0, 1.0, 0.8),
            target_demand=target_demand,
            iters=3,
            max_per_group=90,
            tags=frozenset(["holed_parts", "stress"]),
            mix=(0.4, 0.35, 0.25),
            floors=NestFloors(parts_final=6, area_coverage=0.12),
        ),
    )


def make_holed_parts_in_swiss(
    board_size: tuple[float, float] = (18.0, 18.0),
    target_demand: float = 1.1,
) -> NestCase:
    return build_nest_case(
        CaseSpec(
            family="holed_parts_in_swiss",
            board="rect",
            board_params=board_size,
            hole_layout="swiss",
            hole_params=(4, 1.2, 11),
            part_catalog=("frame", "small_rect"),
            part_scales=(1.0, 0.8),
            target_demand=target_demand,
            iters=3,
            max_per_group=90,
            tags=frozenset(["holed_parts", "void", "stress"]),
            mix=(0.55, 0.45),
            floors=NestFloors(parts_final=5, area_coverage=0.1),
        ),
    )


def make_organic_board_pack(
    size: float = 16.0,
    target_demand: float = 1.0,
) -> NestCase:
    return build_nest_case(
        CaseSpec(
            family="organic_board_pack",
            board="organic",
            board_params=(size, 18, 0.22, 7),
            part_catalog=("rect", "L"),
            part_scales=(1.0, 1.0),
            target_demand=target_demand,
            iters=3,
            max_per_group=50,
            tags=frozenset(["organic", "stress"]),
            mix=(0.55, 0.45),
            floors=NestFloors(parts_final=6, area_coverage=0.12),
        ),
    )


def make_needle_stress(
    board_size: tuple[float, float] = (14.0, 14.0),
    target_demand: float = 1.1,
) -> NestCase:
    return build_nest_case(
        CaseSpec(
            family="needle_stress",
            board="rect",
            board_params=board_size,
            part_catalog=("needle", "small_rect"),
            part_scales=(1.0, 1.0),
            target_demand=target_demand,
            iters=3,
            max_per_group=90,
            tags=frozenset(["aspect", "stress"]),
            mix=(0.55, 0.45),
            floors=NestFloors(parts_final=8, area_coverage=0.1),
        ),
    )


def make_grain_locked_strips(
    board_size: tuple[float, float] = (14.0, 14.0),
    target_demand: float = 1.1,
) -> NestCase:
    return build_nest_case(
        CaseSpec(
            family="grain_locked_strips",
            board="rect",
            board_params=board_size,
            part_catalog=("needle", "rect"),
            part_scales=(1.0, 1.0),
            group_allowed_angles=((0.0, math.pi), None),
            target_demand=target_demand,
            iters=3,
            max_per_group=90,
            tags=frozenset(["grain", "orientation", "stress"]),
            mix=(0.55, 0.45),
            floors=NestFloors(parts_final=6, area_coverage=0.1),
        ),
    )


def get_all_cases() -> list[NestCase]:
    """Return the standard suite of benchmark cases."""
    return [
        make_demo_triangle(),
        make_demo_triangle_corner_cluster(),
        make_rect_sheet_mixed(),
        make_donut_void(),
        make_concave_interlock(),
        make_swiss_cheese(),
        make_border_then_fill(),
        make_loose_cluster_compact(),
        make_mixed_scale_stress(),
        make_tight_border_ring(),
        make_dense_cluster_with_pockets(),
        make_incomplete_interlock(),
        make_hole_trap(),
        make_narrow_corridor(),
        make_rim_kiss_ring(),
        make_two_cluster_bridge(),
        make_hole_mouth_seeded(),
        make_corridor_seeded(),
        make_donut_rim(),
        make_mixed_irregular(),
        make_donut_parts_pack(),
        make_holed_parts_in_swiss(),
        make_organic_board_pack(),
        make_needle_stress(),
        make_grain_locked_strips(),
    ]


def get_tight_pack_cases() -> list[NestCase]:
    return get_cases(tags=["tight_pack"])


def get_cases(*, tags: list[str] | None = None, family: str | None = None) -> list[NestCase]:
    return resolve_cases(tags=tags, family=family)
