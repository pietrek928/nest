"""Shared part/board builders for demo CLI and nesting fixtures (one catalog SoT)."""

import math
from dataclasses import dataclass
from typing import Any

import numpy as np
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

from nest_graph.utils import normalize_poly, transform_poly


@dataclass(frozen=True)
class DemoPackingCase:
    """Board + K part groups for run_build_graph (NestCase-compatible fields)."""

    board: Polygon
    board_holes: tuple[Polygon, ...]
    groups: tuple[tuple[Polygon, int], ...]
    group_allowed_angles: tuple[tuple[float, ...] | None, ...] = ()
    group_counts: tuple[int, ...] = ()
    name: str = ""


def make_rect(w: float, h: float) -> Polygon:
    return normalize_poly(box(0, 0, w, h))


def make_tri(w: float, h: float) -> Polygon:
    return normalize_poly(Polygon([(0, 0), (w, 0), (0, h)]))


def make_l_shape(w: float, h: float, t: float) -> Polygon:
    return normalize_poly(Polygon([(0, 0), (w, 0), (w, t), (t, t), (t, h), (0, h)]))


def make_c_shape(w: float = 2.0, h: float = 2.0, t: float = 0.5) -> Polygon:
    return normalize_poly(
        Polygon([
            (0, 0), (w, 0), (w, t), (t, t), (t, h - t), (w, h - t), (w, h), (0, h),
        ])
    )


def make_u_shape(w: float = 2.0, h: float = 2.0, t: float = 0.5) -> Polygon:
    return normalize_poly(
        Polygon([
            (0, 0), (w, 0), (w, h), (w - t, h), (w - t, t), (t, t), (t, h), (0, h),
        ])
    )


def make_t_shape(w: float = 2.0, h: float = 2.0, t: float = 0.6) -> Polygon:
    stem = (w - t) / 2.0
    merged = unary_union([box(0, h - t, w, h), box(stem, 0, stem + t, h - t)])
    if merged.geom_type != "Polygon":
        merged = max(merged.geoms, key=lambda g: g.area)
    return normalize_poly(merged)


def make_star(r_outer: float = 1.0, r_inner: float = 0.4, points: int = 5) -> Polygon:
    pts = []
    for i in range(points * 2):
        r = r_outer if i % 2 == 0 else r_inner
        a = math.pi * i / points - math.pi / 2
        pts.append((r * math.cos(a), r * math.sin(a)))
    return normalize_poly(Polygon(pts))


def make_cross(arm: float = 2.0, t: float = 0.6) -> Polygon:
    c = arm / 2.0
    ht = t / 2.0
    return normalize_poly(
        Polygon([
            (-ht, -c), (ht, -c), (ht, -ht), (c, -ht), (c, ht), (ht, ht),
            (ht, c), (-ht, c), (-ht, ht), (-c, ht), (-c, -ht), (-ht, -ht),
        ])
    )


def make_donut_part(outer: float = 1.5, inner: float = 0.6) -> Polygon:
    shell = list(Point(0, 0).buffer(outer, resolution=4).exterior.coords)
    hole = list(Point(0, 0).buffer(inner, resolution=4).exterior.coords)
    return normalize_poly(Polygon(shell, [hole]))


def make_frame(w: float = 2.0, h: float = 2.0, t: float = 0.35) -> Polygon:
    shell = [(0, 0), (w, 0), (w, h), (0, h)]
    hole = [(t, t), (w - t, t), (w - t, h - t), (t, h - t)]
    return normalize_poly(Polygon(shell, [hole]))


def make_needle(length: float = 4.0, thickness: float = 0.25) -> Polygon:
    return normalize_poly(box(0, 0, length, thickness))


def make_wedge(w: float = 2.0, h: float = 0.6) -> Polygon:
    return normalize_poly(Polygon([(0, 0), (w, 0), (w * 0.5, h)]))


def make_organic_board(
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
    "rect": lambda s=1.0, **_: make_rect(1.0 * s, 1.0 * s),
    "tri": lambda s=1.0, **_: make_tri(1.5 * s, 0.7 * s),
    "strip": lambda s=1.0, **_: make_rect(2.5 * s, 0.35 * s),
    "L": lambda s=1.0, **_: make_l_shape(2.0 * s, 2.0 * s, 0.8 * s),
    "C": lambda s=1.0, **_: make_c_shape(2.0 * s, 2.0 * s, 0.5 * s),
    "U": lambda s=1.0, **_: make_u_shape(2.0 * s, 2.0 * s, 0.5 * s),
    "T": lambda s=1.0, **_: make_t_shape(2.0 * s, 2.0 * s, 0.6 * s),
    "star": lambda s=1.0, **_: make_star(1.0 * s, 0.4 * s),
    "cross": lambda s=1.0, **_: make_cross(2.0 * s, 0.6 * s),
    "donut": lambda s=1.0, **_: make_donut_part(1.4 * s, 0.55 * s),
    "frame": lambda s=1.0, **_: make_frame(2.0 * s, 2.0 * s, 0.35 * s),
    "needle": lambda s=1.0, **_: make_needle(4.0 * s, 0.25 * s),
    "wedge": lambda s=1.0, **_: make_wedge(2.0 * s, 0.55 * s),
    "small_rect": lambda s=1.0, **_: make_rect(0.8 * s, 0.8 * s),
    "filler": lambda s=1.0, **_: make_rect(0.95 * s, 0.95 * s),
}


def build_board(kind: str, params: tuple) -> Polygon:
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
        return make_organic_board(size=size, n_verts=n_verts, noise=noise, seed=seed)
    raise ValueError(f"unknown board kind: {kind}")


def separated_holes(
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


def build_holes(board: Polygon, layout: str, params: tuple) -> tuple[Polygon, ...]:
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
        return tuple(separated_holes(board, n, r, seed=seed, min_gap=gap))
    if layout == "trap":
        hw = float(params[0]) if params else 10.0
        hh = float(params[1]) if len(params) > 1 else hw
        hole = transform_poly(make_rect(hw, hh), (cx, cy, 0.0))
        return (hole,)
    if layout == "corridor":
        hw = float(params[0]) if params else 14.0
        hh = float(params[1]) if len(params) > 1 else 6.0
        dy = float(params[2]) if len(params) > 2 else 4.0
        h1 = transform_poly(make_rect(hw, hh), (cx, cy - dy, 0.0))
        h2 = transform_poly(make_rect(hw, hh), (cx, cy + dy, 0.0))
        return (h1, h2)
    raise ValueError(f"unknown hole_layout: {layout}")


def counts_for_demand(
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


def _parse_float_list(raw: str) -> tuple[float, ...]:
    # Accept ``12x8``, ``4,1.2``, or ``4:1.2`` (swiss:n:r / center:WxH).
    normalized = str(raw).replace("x", ",").replace(":", ",")
    parts = [p.strip() for p in normalized.split(",") if p.strip()]
    return tuple(float(p) for p in parts)


def parse_board_flag(raw: str) -> tuple[str, tuple[float, ...]]:
    """Parse ``rect:12x8`` / ``tri:1.5`` / ``organic:16`` → (kind, params)."""
    text = str(raw or "").strip()
    if not text:
        return "rect", (12.0, 12.0)
    if ":" in text:
        kind, rest = text.split(":", 1)
        kind = kind.strip().lower()
        params = _parse_float_list(rest) if rest.strip() else ()
        return kind, params
    kind = text.lower()
    if kind == "rect":
        return "rect", (12.0, 12.0)
    if kind == "tri":
        return "tri", (1.0,)
    if kind == "organic":
        return "organic", (16.0,)
    raise ValueError(f"unknown --board {raw!r}")


def parse_holes_flag(raw: str | None) -> tuple[str, tuple[float, ...]]:
    if raw is None or str(raw).strip() in ("", "none"):
        return "none", ()
    text = str(raw).strip()
    if ":" in text:
        layout, rest = text.split(":", 1)
        return layout.strip().lower(), _parse_float_list(rest)
    return text.lower(), ()


def parse_parts_flag(
    raw: str,
) -> tuple[tuple[str, ...], tuple[float, ...], tuple[int | None, ...]]:
    """Parse ``rect:1:24,tri:1.2,L:0.8:6`` → names, scales, optional counts."""
    names: list[str] = []
    scales: list[float] = []
    counts: list[int | None] = []
    for chunk in str(raw).split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        bits = [b.strip() for b in chunk.split(":")]
        name = bits[0]
        if name not in PART_BUILDERS:
            known = ", ".join(sorted(PART_BUILDERS))
            raise ValueError(f"unknown part {name!r}; known: {known}")
        scale = float(bits[1]) if len(bits) > 1 and bits[1] else 1.0
        count: int | None = None
        if len(bits) > 2 and bits[2]:
            count = max(int(bits[2]), 1)
        names.append(name)
        scales.append(scale)
        counts.append(count)
    if not names:
        raise ValueError("--parts must list at least one kind")
    return tuple(names), tuple(scales), tuple(counts)


def rotations_to_angles(n: int) -> tuple[float, ...]:
    k = max(int(n), 1)
    return tuple(float(2.0 * math.pi * i / k) for i in range(k))


def demo_case_from_flags(
    *,
    board: str | None = None,
    parts: str | None = None,
    demand: float | None = None,
    holes: str | None = None,
    rotations: int | None = None,
    max_per_group: int = 80,
) -> DemoPackingCase:
    """Build a DemoPackingCase from freeform CLI flags (no named fixture)."""
    if not parts and not board and holes is None:
        raise ValueError("demo_case_from_flags requires --board and/or --parts")
    b_kind, b_params = parse_board_flag(board or "rect:12x12")
    outline = build_board(b_kind, b_params)
    h_layout, h_params = parse_holes_flag(holes)
    hole_polys = build_holes(outline, h_layout, h_params)
    if parts:
        names, scales, counts_opt = parse_parts_flag(parts)
    else:
        names, scales, counts_opt = ("rect", "tri"), (1.0, 1.0), (None, None)
    polys = [PART_BUILDERS[n](s=s) for n, s in zip(names, scales)]
    usable = float(outline.area - sum(h.area for h in hole_polys))
    if all(c is not None for c in counts_opt):
        counts = tuple(int(c) for c in counts_opt)  # type: ignore[arg-type]
    else:
        td = float(np.clip(demand if demand is not None else 1.0, 0.5, 1.6))
        mix_w: list[float] = []
        for c in counts_opt:
            mix_w.append(float(c) if c is not None else 1.0)
        counts = counts_for_demand(
            polys, usable, td, tuple(mix_w), max_per_group=max_per_group,
        )
        # Overlay explicit :count entries.
        fixed = list(counts)
        for i, c in enumerate(counts_opt):
            if c is not None:
                fixed[i] = int(c)
        counts = tuple(fixed)
    groups = tuple((p, i) for i, p in enumerate(polys))
    angles: tuple[tuple[float, ...] | None, ...] = ()
    if rotations is not None:
        one = rotations_to_angles(int(rotations))
        angles = tuple(one for _ in groups)
    return DemoPackingCase(
        board=outline,
        board_holes=hole_polys,
        groups=groups,
        group_allowed_angles=angles,
        group_counts=counts,
        name=f"cli_{'_'.join(names)}_k{len(names)}",
    )


def demo_case_from_nest_like(case: object) -> DemoPackingCase:
    """Adapt NestCase (or compatible) into DemoPackingCase."""
    return DemoPackingCase(
        board=getattr(case, "board"),
        board_holes=tuple(getattr(case, "board_holes") or ()),
        groups=tuple(getattr(case, "groups")),
        group_allowed_angles=tuple(getattr(case, "group_allowed_angles") or ()),
        group_counts=tuple(getattr(case, "group_counts") or ()),
        name=str(getattr(case, "name", "") or ""),
    )


def ring_coords(poly: Polygon) -> tuple[tuple[float, float], ...]:
    return tuple((float(x), float(y)) for x, y in list(poly.exterior.coords)[:-1])


def apply_demo_case_to_rules(rules: Any, case: DemoPackingCase) -> Any:
    """Write board rings + ngroups onto RulesConfig before seed/mutation rules."""
    holes = tuple(ring_coords(h) for h in case.board_holes)
    return rules.model_copy(
        update={
            "board_coords": ring_coords(case.board),
            "board_holes": holes,
            "ngroups": max(len(case.groups), 1),
        }
    )
