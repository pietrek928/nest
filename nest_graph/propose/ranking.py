import math
from typing import List, Optional, Sequence, Tuple, Union

import numpy as np
from shapely import MultiPoint, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from nest_graph.config import ProposeConfig, RankingMode, as_ranking_mode
from nest_graph.elem_graph import score_transform
from nest_graph.geometry import (
    Geometry,
    PlacementRankConfig,
    PlacementRankMode,
    batch_rank_local_placements,
    convex_hull_area_of,
    find_polygon_distances_bipartite,
)
from nest_graph.placement_scene import (
    best_proposition,
    placement_clearance_epsilon,
)
from nest_graph.utils import get_shape_polygons_coords, transform_poly

from nest_graph.propose.context import placement_contact_error, should_use_border_focus
from nest_graph.propose.placement_common import as_geometry
from nest_graph.propose.placement_outline import (
    outline_kiss_tolerance,
    outline_ring_geom,
    outline_standoff_distance,
)
from nest_graph.propose.geometry import ProposeGeometry, batch_valid_flags
from nest_graph.propose.placements_guidance import (
    candidate_from_proposition,
    is_cast_move,
)


def _coord_key(coords: Tuple[float, float, float]) -> tuple[float, float, float]:
    return (round(coords[0], 4), round(coords[1], 4), round(coords[2], 4))


def _rank_mode_enum(mode: PlacementRankMode | str) -> PlacementRankMode:
    return as_ranking_mode(mode)


def _placement_rank_config(
    propose_cfg: ProposeConfig,
    *,
    min_dist: float,
    mode: str,
    part_area: float,
    sheet_area: float,
) -> PlacementRankConfig:
    band = float(propose_cfg.edge_free_band_min_dist_mult) * float(min_dist)
    cfg = PlacementRankConfig()
    cfg.min_dist = float(min_dist)
    cfg.clearance_weight = float(propose_cfg.contact_clearance_hybrid_weight)
    cfg.tightness_weight = float(propose_cfg.contact_tightness_hybrid_weight)
    cfg.edge_free_weight = float(propose_cfg.edge_free_weight)
    cfg.edge_free_band_mult = float(propose_cfg.edge_free_band_min_dist_mult)
    cfg.kiss_tol = float(outline_kiss_tolerance(min_dist))
    cfg.tight_scale = max(band, 1e-6)
    cfg.part_area = float(part_area)
    cfg.sheet_area = max(float(sheet_area), 1e-12)
    cfg.mode = _rank_mode_enum(mode)
    return cfg


def _void_pole_bonus(
    coords: Tuple[float, float, float],
    propose_geom: ProposeGeometry,
    shape_to_place: Polygon,
    propose_cfg: ProposeConfig,
    void_pole: Point | None,
) -> float:
    if void_pole is None or propose_cfg is None:
        return 0.0
    w = float(getattr(propose_cfg, "void_rank_pole_weight", 0.0) or 0.0)
    if w <= 0.0:
        return 0.0
    placed = propose_geom.placed_at(coords)
    if placed is None:
        return 0.0
    sheet = propose_geom.sheet
    minx, miny, maxx, maxy = sheet.bounds
    scale = float(math.hypot(maxx - minx, maxy - miny))
    if scale <= 1e-12:
        return 0.0
    cx, cy = placed.center()
    dist = float(Point(float(cx), float(cy)).distance(void_pole))
    area_frac = float(shape_to_place.area) / max(float(sheet.area), 1e-12)
    return max(0.0, 1.0 - dist / scale) * area_frac * w


def _cast_boost_for_coords(
    coords: Tuple[float, float, float],
    propose_geom: ProposeGeometry,
    pt_push: Point,
    propose_cfg: ProposeConfig | None,
    min_dist: float,
    feedback_cache: dict | None = None,
) -> float:
    if propose_cfg is None or propose_cfg.cast_rank_boost <= 0.0 or min_dist <= 0.0:
        return 0.0
    g = _placement_feedback(
        coords, propose_geom, pt_push, feedback_cache=feedback_cache,
    )
    if g is None:
        return 0.0
    prop = best_proposition(g)
    if prop is not None and is_cast_move(prop.move_type or ""):
        return float(propose_cfg.cast_rank_boost) * float(min_dist)
    return 0.0


def _batch_rank_results(
    candidates: Sequence[Tuple[float, float, float]],
    propose_geom: ProposeGeometry,
    propose_cfg: ProposeConfig,
    min_dist: float,
    *,
    mode: str,
    focal_shape: Optional[BaseGeometry] = None,
) -> dict[tuple[float, float, float], object]:
    """One C++ batch for geometry modes; keyed by round-4 transform."""
    if not candidates:
        return {}
    uniq: list[Tuple[float, float, float]] = []
    seen: set[tuple[float, float, float]] = set()
    for c in candidates:
        key = _coord_key(c)
        if key in seen:
            continue
        seen.add(key)
        uniq.append((float(c[0]), float(c[1]), float(c[2])))
    board_ring = outline_ring_geom(propose_geom.sheet, propose_geom=propose_geom)
    if board_ring is None:
        return {}
    obstacles = list(propose_geom.base_geoms)
    focal_geom = None
    if focal_shape is not None:
        focal_geom = (
            focal_shape if isinstance(focal_shape, Geometry) else as_geometry(focal_shape)
        )
    cfg = _placement_rank_config(
        propose_cfg,
        min_dist=min_dist,
        mode=mode,
        part_area=float(propose_geom.part_poly.area),
        sheet_area=float(propose_geom.sheet.area),
    )
    results = batch_rank_local_placements(
        propose_geom.part,
        uniq,
        board_ring,
        obstacles,
        focal_geom,
        cfg,
    )
    out: dict[tuple[float, float, float], object] = {}
    for coords, res in zip(uniq, results, strict=True):
        out[_coord_key(coords)] = res
    return out


def _overlay_rank_score(
    coords: Tuple[float, float, float],
    res,
    *,
    propose_geom: ProposeGeometry,
    shape_to_place: Polygon,
    propose_cfg: ProposeConfig,
    pt_push: Point,
    min_dist: float,
    void_pole: Point | None = None,
    feedback_cache: dict | None = None,
    include_cast_boost: bool = True,
) -> float:
    if res is None or not bool(res.valid):
        return float("-inf")
    score = float(res.rank_score)
    score += _void_pole_bonus(
        coords, propose_geom, shape_to_place, propose_cfg, void_pole,
    )
    if include_cast_boost:
        score += _cast_boost_for_coords(
            coords, propose_geom, pt_push, propose_cfg, min_dist, feedback_cache,
        )
    return score


def _score_placement_rule(
    coords: Tuple[float, float, float],
    rules,
    group_id: int,
    *,
    radius: float = 0.5,
) -> float:
    return float(score_transform(
        rules, group_id, float(coords[0]), float(coords[1]), float(coords[2]),
        radius=float(radius),
    ))


def _part_rule_radius(shape_to_place: Polygon, propose_geom: ProposeGeometry) -> float:
    try:
        return float(propose_geom.part.radius())
    except Exception:
        minx, miny, maxx, maxy = shape_to_place.bounds
        return 0.5 * math.hypot(maxx - minx, maxy - miny)


def _rule_hybrid_geometry_mode(
    propose_cfg: ProposeConfig,
    base_shape: BaseGeometry,
) -> RankingMode:
    if (
        propose_cfg.border_focus_ranking
        and should_use_border_focus(base_shape, propose_cfg)
    ):
        return RankingMode.BORDER
    if propose_cfg.use_contact_clearance_hybrid:
        return RankingMode.CONTACT_HYBRID
    return RankingMode.CONTACT


def _score_placement_rule_hybrid(
    coords: Tuple[float, float, float],
    shape_to_place: Polygon,
    base_shape: BaseGeometry,
    propose_geom: ProposeGeometry,
    propose_cfg: ProposeConfig,
    pt_push: Point,
    min_dist: float,
    focal_shape: Optional[BaseGeometry],
    rules,
    group_id: int,
    void_pole: Point | None = None,
) -> float:
    # Private geom scorers only — do not re-enter _rank_score_for_mode (R15).
    geom_mode = _rule_hybrid_geometry_mode(propose_cfg, base_shape)
    if geom_mode == RankingMode.BORDER:
        score = _score_placement_border(
            coords, shape_to_place, propose_geom, pt_push, min_dist, propose_cfg,
        )
    else:
        score = _score_placement_contact_hybrid(
            coords, shape_to_place, propose_geom, pt_push, min_dist, focal_shape,
            propose_cfg.contact_clearance_hybrid_weight,
            tightness_weight=propose_cfg.contact_tightness_hybrid_weight,
            propose_cfg=propose_cfg,
            void_pole=void_pole,
        )
    if score == float("-inf") or rules is None or rules.size() == 0:
        return score
    rule_score = _score_placement_rule(
        coords, rules, group_id,
        radius=_part_rule_radius(shape_to_place, propose_geom),
    )
    return score + propose_cfg.rule_ranking_weight * rule_score

def _neighbor_excess_gap_for_placed(
    placed_geom: Geometry,
    base_geoms: list[Geometry],
    min_dist: float,
) -> float:
    if not base_geoms:
        return 0.0
    results = find_polygon_distances_bipartite([placed_geom], base_geoms, aura=0.5)
    nearest = float("inf")
    for r in results:
        d = 0.0 if r.intersect else math.sqrt(r.distance_sq)
        nearest = min(nearest, d)
    if nearest >= float("inf"):
        return 0.0
    return max(0.0, nearest - min_dist)


def calculate_excess_and_kiss(
    geom: Geometry,
    *,
    obstacles: list[Geometry],
    outline: BaseGeometry | None,
    min_dist: float,
    ring_geom: Geometry | None = None,
) -> tuple[float, float]:
    """Single tightness math: (excess_neighbor_gap, outline_kiss_err).

    Both single-placed and pack-total wrappers must call this SoT.
    """
    excess = _neighbor_excess_gap_for_placed(geom, obstacles, min_dist) if obstacles else 0.0
    if ring_geom is not None:
        kiss = abs(geom.standoff_distance(ring_geom) - min_dist)
    elif outline is not None:
        kiss = abs(outline_standoff_distance(geom, outline) - min_dist)
    else:
        kiss = 0.0
    return excess, kiss


def pack_neighbor_excess_gap(geoms: list[Geometry], min_dist: float) -> float:
    """Sum of per-part excess gaps vs nearest neighbor in a pack."""
    if len(geoms) < 2:
        return 0.0
    from nest_graph.geometry import find_polygon_distances

    results = find_polygon_distances(geoms, aura=0.5)
    nearest = [float("inf")] * len(geoms)
    for r in results:
        d = 0.0 if r.intersect else math.sqrt(r.distance_sq)
        if 0 <= r.polyA_idx < len(nearest):
            nearest[r.polyA_idx] = min(nearest[r.polyA_idx], d)
        if 0 <= r.polyB_idx < len(nearest):
            nearest[r.polyB_idx] = min(nearest[r.polyB_idx], d)
    return sum(
        max(0.0, nd - min_dist)
        for nd in nearest
        if nd < float("inf")
    )


def pack_tightness_cost(
    geoms: Sequence[Geometry] | Sequence[BaseGeometry],
    outline: BaseGeometry,
    min_dist: float,
) -> float:
    """Lower is tighter for a full pack (2*excess + kiss). Uses shared kiss math.

    Accepts native ``Geometry`` (preferred, no conversion) or Shapely polys.
    """
    if not geoms:
        return 0.0
    if any(not isinstance(g, Geometry) for g in geoms):
        from nest_graph.propose.placement_common import as_geometry

        geoms = [g for g in (as_geometry(p) for p in geoms) if g is not None]
        if not geoms:
            return 0.0
    ring = outline_ring_geom(outline) if outline is not None else None
    kiss = 0.0
    for g in geoms:
        _excess, k = calculate_excess_and_kiss(
            g, obstacles=[], outline=outline, min_dist=min_dist, ring_geom=ring,
        )
        kiss += k
    excess_gap = pack_neighbor_excess_gap(geoms, min_dist)
    return 2.0 * excess_gap + 1.0 * kiss


def score_placement_tightness(
    coords: Tuple[float, float, float],
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
) -> float:
    """Higher score = tighter outline kiss + lower neighbor excess gap."""
    placed_geom = propose_geom.placed_at(coords)
    if not propose_geom.valid_at(coords, pt_push):
        return float("-inf")
    excess, kiss_err = calculate_excess_and_kiss(
        placed_geom,
        obstacles=list(propose_geom.base_geoms),
        outline=propose_geom.sheet,
        min_dist=min_dist,
    )
    return -(2.0 * excess + 1.0 * kiss_err)

def calculate_complex_score(base, placed, base_hull_area, centroid, pt_push, w_dist, w_dir, w_hull):
    # 1. Distance Component
    placed_centroid = (
        Point(placed.center()) if isinstance(placed, Geometry) else Point(placed.centroid)
    )
    dist_to_center = placed_centroid.distance(centroid)

    # 2. Directional Component
    direction_score = -pt_push.distance(placed_centroid)

    base_geoms: list[Geometry] = []
    placed_geoms: list[Geometry] = []
    if isinstance(base, Geometry):
        base_geoms = [base]
    elif base is not None and not getattr(base, "is_empty", False):
        g = as_geometry(base)
        if g is not None:
            base_geoms = [g]
    if isinstance(placed, Geometry):
        placed_geoms = [placed]
    else:
        g = as_geometry(placed)
        if g is not None:
            placed_geoms = [g]
    if base_geoms or placed_geoms:
        new_area = convex_hull_area_of([*base_geoms, *placed_geoms])
        hull_growth = math.sqrt(max(0.0, new_area - float(base_hull_area)))
    else:
        pts = get_shape_polygons_coords(base) + get_shape_polygons_coords(placed)
        hull_growth = np.sqrt(max(0, MultiPoint(pts).convex_hull.area - base_hull_area))

    return (w_dist * dist_to_center) + (w_dir * direction_score) + (w_hull * hull_growth)


def _native_base_hull_area(base_shape: BaseGeometry) -> float:
    if base_shape is None or getattr(base_shape, "is_empty", False):
        return 0.0
    if isinstance(base_shape, Geometry):
        return float(base_shape.convex_hull_area())
    g = as_geometry(base_shape)
    if g is not None:
        return float(g.convex_hull_area())
    return float(base_shape.convex_hull.area)


def finalize_propositions(propositions, top_n, *, coord_ndigits: int = 2):
    """
    Sorts by cost, deduplicates, and returns the top N proposition coords.
    """
    propositions.sort(key=lambda x: x['cost'])

    unique_props = []
    seen = set()
    for p in propositions:
        c = p['coords']
        key = (
            round(c[0], coord_ndigits),
            round(c[1], coord_ndigits),
            round(c[2], coord_ndigits),
        )
        if key not in seen:
            unique_props.append(c)
            seen.add(key)
        if len(unique_props) >= top_n:
            break
    return unique_props


def propositions_to_ndarray(coords_list: Sequence[Tuple[float, float, float]]) -> np.ndarray:
    if not coords_list:
        return np.zeros((0, 3), dtype=np.float64)
    return np.asarray(coords_list, dtype=np.float64)


def base_shape_from_selection(
    polys: Sequence[BaseGeometry],
    selected_indices: Sequence[int],
) -> Union[Polygon, BaseGeometry]:
    placed = [polys[i] for i in selected_indices]
    if not placed:
        return Polygon()
    return unary_union(placed)


def _score_placement_coords(
    coords: Tuple[float, float, float],
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    boundary: BaseGeometry,
    pt_push: Point,
    min_dist: float,
    propose_geom: ProposeGeometry,
) -> float:
    if not propose_geom.valid_at(coords, pt_push):
        return float("inf")
    placed = propose_geom.placed_at(coords)
    base_hull_area = _native_base_hull_area(base_shape)
    return calculate_complex_score(
        base_shape,
        placed,
        base_hull_area,
        boundary.centroid,
        pt_push,
        w_dist=0.001,
        w_dir=0.4,
        w_hull=0.1,
    )


def _score_placement_legacy(
    coords: Tuple[float, float, float],
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    boundary: BaseGeometry,
    pt_push: Point,
    min_dist: float,
    propose_geom: ProposeGeometry,
) -> float:
    if not propose_geom.valid_at(coords, pt_push):
        return float("inf")
    placed = propose_geom.placed_at(coords)
    base_hull_area = _native_base_hull_area(base_shape)
    return calculate_complex_score(
        base_shape,
        placed,
        base_hull_area,
        boundary.centroid,
        pt_push,
        w_dist=0.001,
        w_dir=0.4,
        w_hull=0.1,
    )


def _batch_placement_feedback(
    candidates: Sequence[Tuple[float, float, float]],
    propose_geom: ProposeGeometry,
    pt_push: Point,
) -> dict[Tuple[float, float, float], object | None]:
    if not candidates:
        return {}
    guidance = batch_valid_flags(
        propose_geom, candidates, pt_push, return_guidance=True,
    )
    return {c: g for c, g in zip(candidates, guidance, strict=True)}


def _placement_feedback(
    coords: Tuple[float, float, float],
    propose_geom: ProposeGeometry,
    pt_push: Point,
    *,
    feedback_cache: dict[Tuple[float, float, float], object | None] | None = None,
):
    if feedback_cache is not None and coords in feedback_cache:
        cached = feedback_cache[coords]
        return cached
    placed_geom = propose_geom.placed_at(coords)
    if not placed_geom.fully_inside(propose_geom.board_geom):
        return None
    g = propose_geom.placement_guidance(placed_geom, (coords[0], coords[1]), pt_push)
    if g.is_penetrating:
        return None
    if propose_geom._min_dist > 0.0:
        margin = propose_geom._min_dist + placement_clearance_epsilon(
            propose_geom._min_dist, ratio=propose_geom._epsilon_ratio,
        )
        if float(g.clearance) < margin:
            return None
    return g


def _score_via_batch(
    coords: Tuple[float, float, float],
    propose_geom: ProposeGeometry,
    propose_cfg: ProposeConfig | None,
    min_dist: float,
    *,
    mode: str,
    focal_shape: Optional[BaseGeometry] = None,
    shape_to_place: Polygon | None = None,
    void_pole: Point | None = None,
    pt_push: Point | None = None,
) -> float:
    if propose_cfg is None:
        propose_cfg = ProposeConfig()
    results = _batch_rank_results(
        [coords], propose_geom, propose_cfg, min_dist,
        mode=mode, focal_shape=focal_shape,
    )
    res = results.get(_coord_key(coords))
    push = pt_push if pt_push is not None else Point(0.0, 0.0)
    part = shape_to_place if shape_to_place is not None else propose_geom.part_poly
    return _overlay_rank_score(
        coords, res,
        propose_geom=propose_geom,
        shape_to_place=part,
        propose_cfg=propose_cfg,
        pt_push=push,
        min_dist=min_dist,
        void_pole=void_pole,
    )


def _score_placement_clearance(
    coords: Tuple[float, float, float],
    propose_geom: ProposeGeometry,
    pt_push: Point,
    propose_cfg: ProposeConfig | None = None,
    min_dist: float = 0.0,
) -> float:
    return _score_via_batch(
        coords, propose_geom, propose_cfg, min_dist,
        mode="clearance", pt_push=pt_push,
    )


def _score_placement_border(
    coords: Tuple[float, float, float],
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
    propose_cfg: ProposeConfig | None = None,
) -> float:
    return _score_via_batch(
        coords, propose_geom, propose_cfg, min_dist,
        mode="border", shape_to_place=shape_to_place, pt_push=pt_push,
    )


def _score_placement_contact(
    coords: Tuple[float, float, float],
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
    focal_shape: Optional[BaseGeometry] = None,
    propose_cfg: ProposeConfig | None = None,
) -> float:
    return _score_via_batch(
        coords, propose_geom, propose_cfg, min_dist,
        mode="contact", focal_shape=focal_shape,
        shape_to_place=shape_to_place, pt_push=pt_push,
    )


def _score_placement_contact_hybrid(
    coords: Tuple[float, float, float],
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
    focal_shape: Optional[BaseGeometry],
    clearance_weight: float,
    *,
    tightness_weight: float = 0.0,
    propose_cfg: ProposeConfig | None = None,
    void_pole: Point | None = None,
) -> float:
    cfg = propose_cfg
    if cfg is None:
        cfg = ProposeConfig(
            contact_clearance_hybrid_weight=clearance_weight,
            contact_tightness_hybrid_weight=tightness_weight,
        )
    else:
        cfg = cfg.model_copy(update={
            "contact_clearance_hybrid_weight": clearance_weight,
            "contact_tightness_hybrid_weight": tightness_weight,
        })
    return _score_via_batch(
        coords, propose_geom, cfg, min_dist,
        mode="contact_hybrid", focal_shape=focal_shape,
        shape_to_place=shape_to_place, void_pole=void_pole, pt_push=pt_push,
    )


def _score_placement_hybrid(
    coords: Tuple[float, float, float],
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    boundary: BaseGeometry,
    pt_push: Point,
    min_dist: float,
    propose_geom: ProposeGeometry,
    propose_cfg: ProposeConfig,
) -> float:
    clearance = _score_placement_clearance(
        coords, propose_geom, pt_push, propose_cfg, min_dist,
    )
    if clearance == float("-inf"):
        return float("-inf")
    legacy = _score_placement_legacy(
        coords, base_shape, shape_to_place, boundary, pt_push, min_dist, propose_geom,
    )
    if legacy == float("inf"):
        return float("-inf")
    w_c = propose_cfg.ranking_clearance_weight
    w_h = propose_cfg.ranking_hull_weight
    return w_c * clearance - w_h * legacy


def _rank_score_for_mode(
    coords: Tuple[float, float, float],
    *,
    rank_mode: str,
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    boundary: BaseGeometry,
    propose_geom: ProposeGeometry,
    propose_cfg: ProposeConfig,
    pt_push: Point,
    min_dist: float,
    focal_shape: Optional[BaseGeometry],
    rules=None,
    group_id: int = 0,
    void_pole: Point | None = None,
) -> float:
    """Dispatch table for ranking modes (SoT — do not OR with guidance valid_at)."""
    scorers = {
        RankingMode.RULE_HYBRID: lambda: _score_placement_rule_hybrid(
            coords, shape_to_place, base_shape, propose_geom, propose_cfg,
            pt_push, min_dist, focal_shape, rules, group_id, void_pole=void_pole,
        ),
        RankingMode.BORDER: lambda: _score_placement_border(
            coords, shape_to_place, propose_geom, pt_push, min_dist, propose_cfg,
        ),
        RankingMode.CONTACT: lambda: _score_placement_contact(
            coords, shape_to_place, propose_geom, pt_push, min_dist, focal_shape,
            propose_cfg,
        ),
        RankingMode.CONTACT_HYBRID: lambda: _score_placement_contact_hybrid(
            coords, shape_to_place, propose_geom, pt_push, min_dist, focal_shape,
            propose_cfg.contact_clearance_hybrid_weight,
            tightness_weight=propose_cfg.contact_tightness_hybrid_weight,
            propose_cfg=propose_cfg,
            void_pole=void_pole,
        ),
        RankingMode.CLEARANCE: lambda: _score_placement_clearance(
            coords, propose_geom, pt_push, propose_cfg, min_dist,
        ),
        RankingMode.HYBRID: lambda: _score_placement_hybrid(
            coords, base_shape, shape_to_place, boundary, pt_push, min_dist,
            propose_geom, propose_cfg,
        ),
    }
    try:
        key = RankingMode(rank_mode) if rank_mode is not None else None
    except ValueError:
        key = None
    scorer = scorers.get(key) if key is not None else None
    if scorer is not None:
        return scorer()
    legacy = _score_placement_legacy(
        coords, base_shape, shape_to_place, boundary, pt_push, min_dist, propose_geom,
    )
    return -legacy if legacy != float("inf") else float("-inf")


# Public alias for plan / call sites.
score_for_mode = _rank_score_for_mode


def cast_squeeze_ranked_coords(
    ranked: Sequence[Tuple[float, float, float]],
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    propose_cfg: ProposeConfig,
    pt_push: Point,
    min_dist: float,
    *,
    focal_shape: Optional[BaseGeometry],
    rank_mode: str,
    rules=None,
    group_id: int = 0,
    base_shape: BaseGeometry | None = None,
) -> List[Tuple[float, float, float]]:
    """Micro-refine top-K ranked coords with guidance cast moves."""
    k = min(max(propose_cfg.cast_squeeze_top_k, 0), len(ranked))
    if k <= 0:
        return list(ranked)
    out: list[tuple[float, float, float]] = []
    seen: set[tuple[float, float, float]] = set()
    for idx, coords in enumerate(ranked):
        if idx < k:
            coords = _cast_squeeze_one(
                coords,
                shape_to_place=shape_to_place,
                propose_geom=propose_geom,
                propose_cfg=propose_cfg,
                pt_push=pt_push,
                min_dist=min_dist,
                focal_shape=focal_shape,
                rank_mode=rank_mode,
                rules=rules,
                group_id=group_id,
                base_shape=base_shape or Polygon(),
            )
        key = (round(coords[0], 4), round(coords[1], 4), round(coords[2], 4))
        if key in seen:
            continue
        seen.add(key)
        out.append(coords)
    return out


def _cast_squeeze_one(
    coords: Tuple[float, float, float],
    *,
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    propose_cfg: ProposeConfig,
    pt_push: Point,
    min_dist: float,
    focal_shape: Optional[BaseGeometry],
    rank_mode: str,
    rules=None,
    group_id: int = 0,
    base_shape: BaseGeometry | None = None,
) -> Tuple[float, float, float]:
    placed_geom = propose_geom.placed_at(coords)
    if not propose_geom.valid_at(coords, pt_push):
        return coords
    best = coords
    best_score = _rank_score_for_mode(
        coords,
        rank_mode=rank_mode,
        base_shape=base_shape or Polygon(),
        shape_to_place=shape_to_place,
        boundary=propose_geom.sheet,
        propose_geom=propose_geom,
        propose_cfg=propose_cfg,
        pt_push=pt_push,
        min_dist=min_dist,
        focal_shape=focal_shape,
        rules=rules,
        group_id=group_id,
    )
    g = propose_geom.placement_guidance(placed_geom, (coords[0], coords[1]), pt_push)
    if g.is_penetrating:
        return coords
    x, y, theta = float(coords[0]), float(coords[1]), float(coords[2])
    for prop in g.propositions[: propose_cfg.guidance_max_propositions]:
        if not is_cast_move(prop.move_type or ""):
            continue
        use_cast = not g.is_penetrating and is_cast_move(prop.move_type or "")
        candidate = candidate_from_proposition(
            x, y, theta, prop,
        )
        trial = propose_geom.placed_at(candidate)
        if not propose_geom.valid(
            trial, pt_push, (candidate[0], candidate[1]),
        ):
            continue
        score = _rank_score_for_mode(
            candidate,
            rank_mode=rank_mode,
            base_shape=base_shape or Polygon(),
            shape_to_place=shape_to_place,
            boundary=propose_geom.sheet,
            propose_geom=propose_geom,
            propose_cfg=propose_cfg,
            pt_push=pt_push,
            min_dist=min_dist,
            focal_shape=focal_shape,
            rules=rules,
            group_id=group_id,
        )
        if score > best_score:
            best_score = score
            best = candidate
    return best


def _trim_candidates_stratified(
    candidates: Sequence[Tuple[float, float, float]],
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
    limit: int,
    focal_shape: Optional[BaseGeometry],
    *,
    contact_fraction: float,
    rank_mode: str,
    clearance_weight: float,
    propose_cfg: ProposeConfig,
    tightness_weight: float = 0.0,
    rules=None,
    group_id: int = 0,
    base_shape: BaseGeometry | None = None,
    void_pole: Point | None = None,
) -> List[Tuple[float, float, float]]:
    """Keep edge-fit and pocket candidates when trimming an oversized pool."""
    del clearance_weight, tightness_weight  # weights come from propose_cfg / C++ config
    if limit <= 0 or not candidates:
        return []
    n_contact = max(1, min(limit, int(round(limit * contact_fraction))))
    n_clear = max(0, limit - n_contact)

    geom_mode = (
        RankingMode.CONTACT_HYBRID
        if RankingMode(rank_mode) in (RankingMode.CONTACT_HYBRID, RankingMode.RULE_HYBRID)
        else RankingMode.CONTACT
    )
    rank_map = _batch_rank_results(
        candidates, propose_geom, propose_cfg, min_dist,
        mode=geom_mode, focal_shape=focal_shape,
    )
    feedback_cache = (
        _batch_placement_feedback(candidates, propose_geom, pt_push)
        if len(candidates) >= 8 else None
    )

    scored_contact: list[tuple[float, Tuple[float, float, float]]] = []
    seen: set[tuple[float, float, float]] = set()
    for coords in candidates:
        key = _coord_key(coords)
        if key in seen:
            continue
        seen.add(key)
        res = rank_map.get(key)
        score = _overlay_rank_score(
            coords, res,
            propose_geom=propose_geom,
            shape_to_place=shape_to_place,
            propose_cfg=propose_cfg,
            pt_push=pt_push,
            min_dist=min_dist,
            void_pole=void_pole,
            feedback_cache=feedback_cache,
        )
        if rank_mode == "rule_hybrid" and score > float("-inf") and rules is not None:
            if rules.size() > 0:
                score += propose_cfg.rule_ranking_weight * _score_placement_rule(
                    coords, rules, group_id,
                    radius=_part_rule_radius(shape_to_place, propose_geom),
                )
        if score > float("-inf"):
            scored_contact.append((score, coords))
    scored_contact.sort(key=lambda x: x[0], reverse=True)
    picked = [coords for _, coords in scored_contact[:n_contact]]
    picked_keys = {_coord_key(c) for c in picked}

    if n_clear <= 0:
        return picked

    remaining = [c for c in candidates if _coord_key(c) not in picked_keys]
    clearance_picked = _trim_candidates_by_clearance(
        remaining, propose_geom, pt_push, n_clear,
        propose_cfg=propose_cfg, min_dist=min_dist, rank_map=rank_map,
        feedback_cache=feedback_cache,
    )
    out = list(picked)
    for c in clearance_picked:
        key = _coord_key(c)
        if key not in picked_keys:
            out.append(c)
            picked_keys.add(key)
        if len(out) >= limit:
            break
    return out[:limit]


def select_guidance_cast_seeds(
    candidates: Sequence[Tuple[float, float, float]],
    limit: int,
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
    focal_shape: Optional[BaseGeometry] = None,
) -> List[Tuple[float, float, float]]:
    """Pick cast expansion seeds by kiss quality, not pool append order."""
    if limit <= 0 or not candidates:
        return []
    if len(candidates) <= limit:
        return list(candidates)
    n_contact = max(1, limit // 2)
    n_clear = max(0, limit - n_contact)
    contact_picked = _trim_candidates_by_contact(
        candidates,
        shape_to_place,
        propose_geom,
        pt_push,
        min_dist,
        n_contact,
        focal_shape,
    )
    picked_keys = {_coord_key(c) for c in contact_picked}
    remaining = [c for c in candidates if _coord_key(c) not in picked_keys]
    clear_picked = _trim_candidates_by_clearance(
        remaining, propose_geom, pt_push, n_clear, min_dist=min_dist,
    )
    out = list(contact_picked)
    for c in clear_picked:
        key = _coord_key(c)
        if key not in picked_keys:
            out.append(c)
            picked_keys.add(key)
        if len(out) >= limit:
            break
    return out[:limit]


def _trim_candidates_by_contact(
    candidates: Sequence[Tuple[float, float, float]],
    shape_to_place: Polygon,
    propose_geom: ProposeGeometry,
    pt_push: Point,
    min_dist: float,
    limit: int,
    focal_shape: Optional[BaseGeometry] = None,
    propose_cfg: ProposeConfig | None = None,
) -> List[Tuple[float, float, float]]:
    if limit <= 0 or not candidates:
        return []
    cfg = propose_cfg if propose_cfg is not None else ProposeConfig()
    rank_map = _batch_rank_results(
        candidates, propose_geom, cfg, min_dist,
        mode="contact", focal_shape=focal_shape,
    )
    scored: list[tuple[float, Tuple[float, float, float]]] = []
    seen: set[tuple[float, float, float]] = set()
    for coords in candidates:
        key = _coord_key(coords)
        if key in seen:
            continue
        seen.add(key)
        res = rank_map.get(key)
        score = _overlay_rank_score(
            coords, res,
            propose_geom=propose_geom,
            shape_to_place=shape_to_place,
            propose_cfg=cfg,
            pt_push=pt_push,
            min_dist=min_dist,
        )
        if score > float("-inf"):
            scored.append((score, coords))
    scored.sort(key=lambda x: x[0], reverse=True)
    return [coords for _, coords in scored[:limit]]


def _trim_candidates_by_clearance(
    candidates: Sequence[Tuple[float, float, float]],
    propose_geom: ProposeGeometry,
    pt_push: Point,
    limit: int,
    propose_cfg: ProposeConfig | None = None,
    min_dist: float = 0.0,
    rank_map: dict | None = None,
    feedback_cache: dict | None = None,
) -> List[Tuple[float, float, float]]:
    """Keep up to limit candidates with highest placement clearance."""
    if limit <= 0 or not candidates:
        return []
    cfg = propose_cfg if propose_cfg is not None else ProposeConfig()
    if rank_map is None:
        rank_map = _batch_rank_results(
            candidates, propose_geom, cfg, min_dist, mode="clearance",
        )
    if feedback_cache is None and len(candidates) >= 8:
        feedback_cache = _batch_placement_feedback(candidates, propose_geom, pt_push)
    scored: list[tuple[float, Tuple[float, float, float]]] = []
    seen: set[tuple[float, float, float]] = set()
    for coords in candidates:
        key = _coord_key(coords)
        if key in seen:
            continue
        seen.add(key)
        res = rank_map.get(key)
        if res is None or not bool(res.valid):
            continue
        score = float(res.clearance)
        score += _cast_boost_for_coords(
            coords, propose_geom, pt_push, cfg, min_dist, feedback_cache,
        )
        scored.append((score, coords))
    scored.sort(key=lambda x: x[0], reverse=True)
    return [coords for _, coords in scored[:limit]]


def _rank_proposal_coords(
    candidates: Sequence[Tuple[float, float, float]],
    base_shape: BaseGeometry,
    shape_to_place: Polygon,
    boundary: BaseGeometry,
    pt_push: Point,
    min_dist: float,
    max_n: int,
    propose_geom: ProposeGeometry,
    propose_cfg: ProposeConfig,
    *,
    rank_mode: Optional[str] = None,
    focal_shape: Optional[BaseGeometry] = None,
    rules=None,
    group_id: int = 0,
    void_pole: Point | None = None,
) -> List[Tuple[float, float, float]]:
    """Rank candidates; higher score is better. Geometry modes use one C++ batch."""
    scored: list[tuple[float, Tuple[float, float, float]]] = []
    seen: set[tuple[float, float, float]] = set()
    mode = rank_mode if rank_mode is not None else propose_cfg.ranking_mode
    mode = RankingMode(mode) if mode is not None else RankingMode.LEGACY

    geom_modes = {
        RankingMode.CLEARANCE,
        RankingMode.BORDER,
        RankingMode.CONTACT,
        RankingMode.CONTACT_HYBRID,
        RankingMode.RULE_HYBRID,
    }
    rank_map = None
    feedback_cache = None
    if mode in geom_modes:
        batch_mode = (
            _rule_hybrid_geometry_mode(propose_cfg, base_shape)
            if mode == RankingMode.RULE_HYBRID else mode
        )
        if batch_mode not in (
            RankingMode.CLEARANCE,
            RankingMode.BORDER,
            RankingMode.CONTACT,
            RankingMode.CONTACT_HYBRID,
        ):
            batch_mode = RankingMode.CONTACT_HYBRID
        rank_map = _batch_rank_results(
            candidates, propose_geom, propose_cfg, min_dist,
            mode=batch_mode, focal_shape=focal_shape,
        )
        feedback_cache = (
            _batch_placement_feedback(candidates, propose_geom, pt_push)
            if len(candidates) >= 8 else None
        )

    for coords in candidates:
        key = _coord_key(coords)
        if key in seen:
            continue
        seen.add(key)
        if rank_map is not None:
            res = rank_map.get(key)
            score = _overlay_rank_score(
                coords, res,
                propose_geom=propose_geom,
                shape_to_place=shape_to_place,
                propose_cfg=propose_cfg,
                pt_push=pt_push,
                min_dist=min_dist,
                void_pole=void_pole,
                feedback_cache=feedback_cache,
            )
            if mode == "rule_hybrid" and score > float("-inf") and rules is not None:
                if rules.size() > 0:
                    score += propose_cfg.rule_ranking_weight * _score_placement_rule(
                        coords, rules, group_id,
                        radius=_part_rule_radius(shape_to_place, propose_geom),
                    )
        else:
            score = _rank_score_for_mode(
                coords,
                rank_mode=mode,
                base_shape=base_shape,
                shape_to_place=shape_to_place,
                boundary=boundary,
                propose_geom=propose_geom,
                propose_cfg=propose_cfg,
                pt_push=pt_push,
                min_dist=min_dist,
                focal_shape=focal_shape,
                rules=rules,
                group_id=group_id,
                void_pole=void_pole,
            )
        if score > float("-inf") and score < float("inf"):
            scored.append((score, coords))
        elif mode == "legacy" and score > float("-inf"):
            scored.append((score, coords))
    reverse = mode not in ("legacy",)
    scored.sort(key=lambda x: x[0], reverse=reverse)
    return [coords for _, coords in scored[:max_n]]
