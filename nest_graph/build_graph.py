# CFLAGS="-Wno-error=incompatible-pointer-types" pip install --force-reinstall --no-binary=shapely --upgrade shapely

import cv2 as cv
import math
import numpy as np
from pydantic import BaseModel, ConfigDict
from shapely import Polygon, unary_union
from shapely.geometry import box as shapely_box
from shapely.geometry import Point
from shapely.ops import nearest_points
from shapely.geometry.base import BaseGeometry
from tqdm import tqdm
from typing import Iterator, NamedTuple, Tuple

from .config import (
    BuildGraphConfig,
    SelectionConfig,
    dedupe_transforms,
    expand_structured_transforms,
    score_rules_options,
    shuffle_transforms,
    subsample_transforms,
    subsample_transforms_with_pinned,
    trim_history,
)
from .geometry import Geometry, GuidanceConfig, find_polygon_intersections_bipartite
from .placement_scene import (
    PlacementScene,
    guidance_config_for_board_edge_anchor,
    guidance_config_for_graph,
    is_valid_placement,
    placement_footprint_inside_board,
    placement_scene_for_part,
    footprints_inside_board,
    proposition_translation,
)
from .propose.placements_guidance import (
    _candidate_from_proposition,
    _is_cast_move,
    _sorted_guidance_propositions,
)
from .board import (
    board_context_from_geometry,
    default_sheet_padding,
    padded_board_bounds,
)
from .utils import normalize_poly, transform_poly
from .propose import (
    propose_placements_point_cloud,
    proposed_transforms_for_groups,
    border_edge_transforms_for_group,
)
from .propose.context import should_use_border_focus
from .track_perf import show_performance
from .elem_graph import (
    ElemGraph, Circle, Vec2,
    PointPlaceRule, PointAngleRule, PlacementRuleSet,
    RuleMutationSettings, RefineSelectionOptions, FinalizeSelectionOptions,
    nest_by_graph, sort_graph, score_elems, augment_rules, score_rules,
    ScoreRulesOptions,
    increase_selection_dfs, increase_score_dfs,
    refine_selection, finalize_selection, selection_is_independent,
)

# Track performance
nest_by_graph = show_performance(nest_by_graph)
sort_graph = show_performance(sort_graph)
score_elems = show_performance(score_elems)
# augment_rules = show_performance(augment_rules)
score_rules = show_performance(score_rules)


class Candidate(NamedTuple):
    group_i: int
    weight: float
    t: np.ndarray
    placed: Geometry


class NestState(NamedTuple):
    polys: list
    group_id: list
    transform: list
    selected_indices: list


def _poly_and_transforms(item):
    if len(item) == 2:
        return item[0], item[1], 1.0
    return item[0], item[2], item[1]


def _selection_coverage_pct(
    selected_indices: list[int],
    group_id: list[int],
    part_areas: tuple[float, ...],
    board_area: float,
) -> float:
    if board_area <= 0:
        return 0.0
    parts_area = sum(part_areas[group_id[i]] for i in selected_indices)
    return 100.0 * parts_area / board_area


def _base_geometries(polygons) -> list[Geometry]:
    return [
        Geometry.from_shapely(_poly_and_transforms(item)[0])
        for item in polygons
    ]


def _iter_candidates(
    board: BaseGeometry,
    polygons,
    *,
    sheet=None,
    void_geoms=None,
    min_dist: float = 0.0,
    epsilon_ratio: float = 0.05,
) -> Iterator[Candidate]:
    if sheet is None or void_geoms is None:
        sheet, void_geoms = board_context_from_geometry(board)
    bounds = sheet.bounds
    guidance_cfg = guidance_config_for_graph(
        min_dist, board_bounds=bounds, epsilon_ratio=epsilon_ratio
    )
    board_geom = Geometry.from_shapely(sheet)
    bases = _base_geometries(polygons)
    for i, item in enumerate(polygons):
        _p, transforms, w = _poly_and_transforms(item)
        base = bases[i]
        for t in transforms:
            placed = base.apply_transform(t)
            scene = placement_scene_for_part(sheet, board_geom, void_geoms, base)
            cx, cy = placed.center()
            if not is_valid_placement(
                scene, placed, (cx, cy), min_dist, guidance_cfg,
                epsilon_ratio=epsilon_ratio,
            ):
                continue
            yield Candidate(i, w, t, placed)


def _bounds_overlap(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> bool:
    return a[0] <= b[2] and b[0] <= a[2] and a[1] <= b[3] and b[1] <= a[3]


def _collision_partners_vs_existing(
    placed: Geometry,
    placed_bbox: tuple[float, float, float, float],
    existing_geoms: list[Geometry],
    existing_bboxes: list[tuple[float, float, float, float]],
) -> list[int]:
    if not existing_geoms:
        return []
    index_map: list[int] = []
    candidates: list[Geometry] = []
    for j, bb in enumerate(existing_bboxes):
        if _bounds_overlap(placed_bbox, bb):
            index_map.append(j)
            candidates.append(existing_geoms[j])
    if not candidates:
        return []
    hits = find_polygon_intersections_bipartite([placed], candidates)
    return [index_map[k] for _i, k in hits]


def _fill_collision_matrix(
    M: np.ndarray,
    geoms: list[Geometry],
    bboxes: list[tuple[float, float, float, float]],
) -> None:
    for i, placed in enumerate(geoms):
        for j in _collision_partners_vs_existing(placed, bboxes[i], geoms[:i], bboxes[:i]):
            M[i, j] = M[j, i] = 1.0


def placement_board_score(
    board: BaseGeometry,
    board_geom: Geometry,
    placed: Geometry,
) -> float:
    if not placement_footprint_inside_board(placed, board_geom):
        cx, cy = placed.center()
        return -board.distance(Point(cx, cy))
    cx, cy = placed.center()
    return 2.0 * board.boundary.distance(Point(cx, cy))


def make_placement_base(base_shape, polys, exclude_p, exclude_dist=0):
    out_polys = [base_shape.exterior]
    for p in polys:
        if exclude_dist:
            if p.exterior.distance(exclude_p) > exclude_dist:
                out_polys.append(p)
        else:
            if not p.intersects(exclude_p):
                out_polys.append(p)
    return unary_union(out_polys)


def placement_base_ribbon(polys, p_center, exclued_dist, include_dist):
    out_polys = []
    for p in polys:
        dist = p.distance(p_center)
        if dist > exclued_dist and dist < include_dist:
            out_polys.append(p)
    return unary_union(out_polys)


class PolygonGroup(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)

    polygon: Polygon
    weight: float
    transforms: np.ndarray


def polygon_board_distance(b: BaseGeometry, p: Polygon):
    if b.contains(p):
        return b.exterior.distance(p) + b.exterior.distance(p.centroid)
    if p.intersects(b):
        return 0
    return -b.distance(p)


def select_non_intersecting_polygons(polygons: np.ndarray):
    selected_geoms: list[Geometry] = []
    selected: list = []
    for p in polygons:
        geom = Geometry.from_shapely(p)
        if geom.intersects_any(selected_geoms):
            continue
        selected_geoms.append(geom)
        selected.append(p)
    return selected


def select_polygons_from_edges(b: BaseGeometry, polygons: Tuple[Tuple[Polygon, float, np.ndarray], ...]):
    board_geom = Geometry.from_shapely(b)
    result = [[] for _ in range(len(polygons))]
    scored: list[tuple[float, int, np.ndarray, Geometry]] = []
    bases = _base_geometries(polygons)

    for i, item in enumerate(polygons):
        _p, transforms, w = _poly_and_transforms(item)
        base = bases[i]
        for t in transforms:
            placed = base.apply_transform(t)
            score = placement_board_score(b, board_geom, placed)
            if score > 0:
                scored.append((
                    placed.center()[0] + w + np.random.rand() * 1e-4,
                    i,
                    t,
                    placed,
                ))

    scored.sort(key=lambda x: x[0])

    selected_geoms: list[Geometry] = []
    for _, pnum, t, placed in scored:
        if not placed.intersects_any(selected_geoms):
            selected_geoms.append(placed)
            result[pnum].append(t)

    return tuple(np.array(tt) for tt in result)


def make_polygon_matrix(
    b: BaseGeometry,
    polygons: Tuple[Tuple[Polygon, float, np.ndarray], ...],
    *,
    min_dist: float = 0.0,
    epsilon_ratio: float = 0.05,
):
    sheet, void_geoms = board_context_from_geometry(b)
    selected_geoms: list[Geometry] = []
    group_weights = []

    for cand in _iter_candidates(
        b, polygons, sheet=sheet, void_geoms=void_geoms,
        min_dist=min_dist, epsilon_ratio=epsilon_ratio,
    ):
        selected_geoms.append(cand.placed)
        group_weights.append(cand.weight)

    n = len(selected_geoms)
    bboxes = [g.bounds() for g in selected_geoms]
    M = np.zeros((n, n), dtype=np.float32)
    _fill_collision_matrix(M, selected_geoms, bboxes)
    for i in range(n):
        M[i, i] = -group_weights[i]

    return M, selected_geoms


@show_performance
def make_polygon_graph(
    b: BaseGeometry,
    polygons,
    *,
    min_dist: float = 0.0,
    epsilon_ratio: float = 0.05,
):
    sheet, void_geoms = board_context_from_geometry(b)
    board_geom = Geometry.from_shapely(sheet)
    pad = default_sheet_padding(b)
    guidance_cfg = guidance_config_for_graph(
        min_dist,
        board_bounds=padded_board_bounds(b, pad),
        epsilon_ratio=epsilon_ratio,
    )
    graph = ElemGraph()
    selected_polys = []
    selected_geoms: list[Geometry] = []
    selected_bboxes: list[tuple[float, float, float, float]] = []
    selected_group_id = []
    selected_transform = []
    bases = _base_geometries(polygons)

    candidates: list[tuple] = []
    placed_solids: list[Geometry] = []
    for i, item in enumerate(polygons):
        if len(item) == 2:
            p, transforms = item
        else:
            p, _w, transforms = item
        base = bases[i]
        for t in transforms:
            placed = base.apply_transform(t)
            candidates.append((i, p, t, placed, base))
            placed_solids.append(placed)

    footprint_ok = (
        footprints_inside_board(placed_solids, board_geom)
        if placed_solids
        else []
    )

    pending: list[tuple] = []
    for k, (i, p, t, placed, base) in enumerate(candidates):
        if not footprint_ok[k]:
            continue
        scene = placement_scene_for_part(sheet, board_geom, void_geoms, base)
        cx, cy = placed.center()
        if not is_valid_placement(
            scene, placed, (cx, cy), min_dist, guidance_cfg,
            epsilon_ratio=epsilon_ratio,
            skip_footprint=True,
        ):
            continue
        pending.append((i, p, t, placed))

    graph.reserve_elems(len(pending))
    for i, p, t, placed in pending:
        bbox = placed.bounds()
        n = len(selected_geoms)
        graph.append_elem(
            i,
            Vec2(x=placed.center()[0], y=placed.center()[1]),
            Circle.from_center_radius(*placed.center(), placed.radius()),
        )
        for j in _collision_partners_vs_existing(
            placed, bbox, selected_geoms, selected_bboxes
        ):
            graph.add_collision(n, j)
        selected_polys.append(transform_poly(p, t))
        selected_geoms.append(placed)
        selected_bboxes.append(bbox)
        selected_group_id.append(i)
        selected_transform.append(t)

    return graph, selected_polys, selected_group_id, selected_transform


def optimize_polygons(M: np.ndarray, v: np.ndarray):
    score = 1e9
    for it in range(1280):
        v += np.random.rand(M.shape[0]) * 1e-5
        v -= 1.5e-2 * (M @ v)
        v = np.clip(v, 0, 1)
        score = v @ M @ v
        print(f'iter {it}, score {score}, mean {v.mean()}, max {v.max()}')
    return v


def score_transforms(b: BaseGeometry, p: Polygon, transforms: np.ndarray):
    scores = np.zeros((transforms.shape[0], ))
    for i, t in enumerate(transforms):
        p_t = transform_poly(p, t)
        scores[i] = polygon_board_distance(b, p_t)
    return scores


# random transforms in range
def transforms_around(p: np.ndarray, s: Tuple[float, float, float], n: int):
    sx, sy, sa = s
    return np.concatenate([
        p + np.random.uniform(-1, 1, (p.shape[0], 3)) * [sx, sy, sa]
        for _ in range(n)
    ])


def transform_shuffle_mix(
    sel: np.ndarray,
    hist: np.ndarray,
    count: int,
    rng: np.random.Generator,
    scale: Tuple[float, float, float],
) -> np.ndarray:
    """Resample shuffled selection/history rows with fresh jitter."""
    parts = [arr for arr in (sel, hist) if arr.shape[0] > 0]
    if not parts or count <= 0:
        return np.zeros((0, 3))
    merged = np.concatenate(parts)
    rng.shuffle(merged)
    if merged.shape[0] >= count:
        picked = merged[:count]
    else:
        extra = rng.integers(0, merged.shape[0], size=count - merged.shape[0])
        picked = np.concatenate([merged, merged[extra]])
    jitter = rng.uniform(-1, 1, (picked.shape[0], 3)) * scale
    return picked + jitter


def scale_coords(
    coords: np.ndarray,
    xstart: float, ystart: float,
    xscale: float, yscale: float
) -> np.ndarray:
    x, y = coords.T
    x = (x - xstart) * xscale
    y = (y - ystart) * yscale
    return np.stack([x, y], axis=-1).astype(np.int32)

FILL_COLORS = (
    (100, 0, 0),
    (0, 100, 0),
    (0, 0, 100),
    (100, 100, 0),
    (100, 0, 100),
    (0, 100, 100),
)


def _draw_nesting_outline(
    im,
    outline: BaseGeometry,
    xstart: float,
    ystart: float,
    xscale: float,
    yscale: float,
) -> None:
    """Draw nest outline only (not the sheet bbox or corner voids)."""
    geoms = outline.geoms if hasattr(outline, "geoms") else [outline]
    for g in geoms:
        if g.is_empty:
            continue
        if g.geom_type != "Polygon":
            continue
        cv.drawContours(im, [scale_coords(
            np.array(g.exterior.coords), xstart, ystart, xscale, yscale
        )], -1, (255, 255, 255), 3)
        for ring in g.interiors:
            cv.drawContours(im, [scale_coords(
                np.array(ring.coords), xstart, ystart, xscale, yscale
            )], -1, (160, 160, 160), 2)


def render_placement(
    b: BaseGeometry,
    elems: Tuple[Tuple[Polygon, np.ndarray], ...],
    im_shape=(1024, 1024),
    *,
    nest_outline: BaseGeometry | None = None,
):
    xstart, ystart, xend, yend = b.bounds
    xscale = im_shape[0] / (xend - xstart)
    yscale = im_shape[1] / (yend - ystart)
    im = np.zeros((im_shape[0], im_shape[1], 3), dtype=np.uint8)
    _draw_nesting_outline(im, nest_outline if nest_outline is not None else b,
                          xstart, ystart, xscale, yscale)
    for it, (p, transforms) in enumerate(elems):
        fill_col = FILL_COLORS[it % len(FILL_COLORS)]
        for t in transforms:
            p_t = transform_poly(p, t)
            cv.drawContours(im, [scale_coords(
                np.array(p_t.exterior.coords), xstart, ystart, xscale, yscale
            )], -1, fill_col, cv.FILLED)
            cv.drawContours(im, [scale_coords(
                np.array(p_t.exterior.coords), xstart, ystart, xscale, yscale
            )], -1, (255, 255, 255), 3)
    return im


def render_selection(
    b: BaseGeometry,
    polys: Tuple[Polygon, ...],
    v: np.ndarray,
    im_shape=(1024, 1024),
    *,
    nest_outline: BaseGeometry | None = None,
):
    xstart, ystart, xend, yend = b.bounds
    xscale = im_shape[0] / (xend - xstart)
    yscale = im_shape[1] / (yend - ystart)
    im = np.zeros((im_shape[0], im_shape[1], 3), dtype=np.uint8)
    _draw_nesting_outline(im, nest_outline if nest_outline is not None else b,
                          xstart, ystart, xscale, yscale)
    for w, p in sorted(zip(v, polys), key=lambda x: x[0]):
        cv.drawContours(im, [scale_coords(
            np.array(p.exterior.coords), xstart, ystart, xscale, yscale
        )], -1, (100*w, 100*w, 100*w), cv.FILLED)
        cv.drawContours(im, [scale_coords(
            np.array(p.exterior.coords), xstart, ystart, xscale, yscale
        )], -1, (255*w, 255*w, 255*w), 3)
    return im


def render_polys(
    b: BaseGeometry,
    polys: Tuple[Tuple[Polygon, ...], ...],
    im_shape=(1024, 1024),
    *,
    nest_outline: BaseGeometry | None = None,
):
    xstart, ystart, xend, yend = b.bounds
    xscale = im_shape[0] / (xend - xstart)
    yscale = im_shape[1] / (yend - ystart)
    im = np.zeros((im_shape[0], im_shape[1], 3), dtype=np.uint8)
    _draw_nesting_outline(im, nest_outline if nest_outline is not None else b,
                          xstart, ystart, xscale, yscale)
    for it, poly_set in enumerate(polys):
        fill_col = FILL_COLORS[it % len(FILL_COLORS)]
        for p in poly_set:
            cv.drawContours(im, [scale_coords(
                np.array(p.exterior.coords), xstart, ystart, xscale, yscale
            )], -1, fill_col, cv.FILLED)
            cv.drawContours(im, [scale_coords(
                np.array(p.exterior.coords), xstart, ystart, xscale, yscale
            )], -1, (255, 255, 255), 3)
    return im


def _rule_region(board: BaseGeometry) -> Circle:
    xmin, ymin, xmax, ymax = board.bounds
    return Circle.from_bounds(xmin, ymin, xmax, ymax)


def _quantize_rule_scalar(v: float, places: int = 4) -> float:
    return round(float(v), places)


def _fingerprint_rule_set(rule_set: PlacementRuleSet) -> tuple:
    parts: list[tuple] = []
    q = _quantize_rule_scalar
    for pr in rule_set.point_rules:
        parts.append(
            ("p", pr.group, q(pr.pos[0]), q(pr.pos[1]), q(pr.r), q(pr.w)),
        )
    for cr in rule_set.circle_rules:
        parts.append(
            (
                "c",
                cr.group,
                q(cr.circle.center.x),
                q(cr.circle.center.y),
                q(cr.circle.radius),
                q(cr.r),
                q(cr.w),
            ),
        )
    for pr in rule_set.point_angle_rules:
        parts.append(
            (
                "pa",
                pr.group,
                q(pr.pos[0]),
                q(pr.pos[1]),
                q(pr.a),
                q(pr.r),
                q(pr.w),
            ),
        )
    for cr in rule_set.circle_angle_rules:
        parts.append(
            (
                "ca",
                cr.group,
                q(cr.circle.center.x),
                q(cr.circle.center.y),
                q(cr.a),
                q(cr.r),
                q(cr.w),
            ),
        )
    return tuple(sorted(parts))


def dedupe_rule_sets(rule_sets: list[PlacementRuleSet]) -> list[PlacementRuleSet]:
    seen: set[tuple] = set()
    out: list[PlacementRuleSet] = []
    for rs in rule_sets:
        key = _fingerprint_rule_set(rs)
        if key in seen:
            continue
        seen.add(key)
        out.append(rs)
    return out


def truncate_rule_set(
    rule_set: PlacementRuleSet,
    max_rules: int,
) -> PlacementRuleSet:
    if rule_set.size() <= max_rules:
        return rule_set
    weighted: list[tuple[float, object]] = []
    for pr in rule_set.point_rules:
        weighted.append((pr.w, pr))
    for cr in rule_set.circle_rules:
        weighted.append((cr.w, cr))
    for pr in rule_set.point_angle_rules:
        weighted.append((pr.w, pr))
    for cr in rule_set.circle_angle_rules:
        weighted.append((cr.w, cr))
    weighted.sort(key=lambda item: item[0], reverse=True)
    out = PlacementRuleSet()
    for _, rule in weighted[:max_rules]:
        out.append_rule(rule)
    return out


def active_rule_set(rule_sets: list[PlacementRuleSet]) -> PlacementRuleSet:
    if not rule_sets:
        return PlacementRuleSet()
    return rule_sets[0]


def improve_rules(
    graphs,
    rules,
    n: int,
    board: BaseGeometry | None = None,
    *,
    mutation_presets: list[RuleMutationSettings] | None = None,
    rule_score_penalty: float = 0.03,
    elite_count: int = 16,
    seed: int = 0,
    score_options: ScoreRulesOptions | None = None,
    max_rules_per_set: int = 24,
):
    if mutation_presets is None:
        region = _rule_region(board) if board is not None else Circle.from_bounds(0, 0, 1.2, 1.1)
        ng = 2
        mutation_presets = [
            RuleMutationSettings(
                region=region, dpos=0.25, dw=0.25, da=np.pi / 4,
                insert_p=0.09, remove_p=0.02, mutate_p=0.1, ngroups=ng,
            ),
            RuleMutationSettings(
                region=region, dpos=0.05, dw=0.05, da=np.pi / 32,
                insert_p=0.04, remove_p=0.01, mutate_p=0.1, ngroups=ng,
            ),
            RuleMutationSettings(
                region=region, dpos=0.01, dw=0.01, da=np.pi / 64,
                insert_p=0.01, remove_p=0.02, mutate_p=0.1, ngroups=ng,
            ),
        ]
    if score_options is None:
        score_options = ScoreRulesOptions()
        score_options.rule_complexity_penalty = rule_score_penalty
    elif score_options.rule_complexity_penalty == 0.0:
        score_options.rule_complexity_penalty = rule_score_penalty

    parents = list(rules)
    elites = parents
    if graphs and parents:
        rank_opts = ScoreRulesOptions()
        rank_opts.latest_graph_only = score_options.latest_graph_only
        rank_opts.count_weight = score_options.count_weight
        rank_opts.rule_complexity_penalty = score_options.rule_complexity_penalty
        rank_opts.select = score_options.select
        parent_scores = score_rules(graphs, parents, rank_opts)
        ranked = sorted(
            zip(parent_scores, parents),
            key=lambda item: item[0],
            reverse=True,
        )
        k = min(max(elite_count, 1), len(ranked))
        elites = [rs for _, rs in ranked[:k]]

    pool: list[PlacementRuleSet] = list(parents)
    for preset_idx, preset in enumerate(mutation_presets):
        mutate_seed = (int(seed) + preset_idx * 10007) & 0xFFFFFFFF
        children = augment_rules(elites, preset, seed=mutate_seed)
        pool.extend(children)

    pool = dedupe_rule_sets(pool)
    if max_rules_per_set > 0:
        pool = [truncate_rule_set(rs, max_rules_per_set) for rs in pool]
    if not pool:
        return []

    fitness = score_rules(graphs, pool, score_options)
    scored = sorted(zip(fitness, pool), key=lambda item: item[0], reverse=True)
    return [rs for _, rs in scored[:n]]


def score_rule_sets_with_dfs(
    graph: ElemGraph,
    rule_sets: list[PlacementRuleSet],
    selection: SelectionConfig,
    *,
    top_k: int = 4,
) -> list[float]:
    """Tier-B fitness for benchmarks: nest + DFS count on latest graph (top_k by Tier A)."""
    if not rule_sets:
        return []
    tier_a = score_rules([graph], rule_sets, score_rules_options(selection))
    order = sorted(range(len(rule_sets)), key=lambda i: tier_a[i], reverse=True)
    out = list(tier_a)
    for idx in order[: max(top_k, 0)]:
        rs = rule_sets[idx]
        selected = list(nest_by_graph(graph, [rs])[0])
        scores = score_elems(graph, rs)
        _, final, _ = apply_dfs_refinement(
            graph, rs, selected, scores, selection=selection,
        )
        out[idx] = float(len(final)) - selection.rule_score_penalty * rs.size()
    return out


def window_selected_transforms(
    selection_window: list[tuple[np.ndarray, np.ndarray]] | None,
) -> tuple[np.ndarray, np.ndarray]:
    """Merge per-iteration nest selections from the graph window (deduped per group)."""
    if not selection_window:
        return (np.zeros((0, 3)), np.zeros((0, 3)))
    parts0 = [w[0] for w in selection_window if w[0].shape[0] > 0]
    parts1 = [w[1] for w in selection_window if w[1].shape[0] > 0]
    t0 = (
        dedupe_transforms(np.concatenate(parts0, axis=0))
        if parts0
        else np.zeros((0, 3))
    )
    t1 = (
        dedupe_transforms(np.concatenate(parts1, axis=0))
        if parts1
        else np.zeros((0, 3))
    )
    return (t0, t1)


def _append_selection_window(
    selection_window: list[tuple[np.ndarray, np.ndarray]],
    selected_t: tuple[np.ndarray, np.ndarray],
    maxlen: int,
) -> None:
    if not any(t.shape[0] for t in selected_t):
        return
    selection_window.append(selected_t)
    if len(selection_window) > maxlen:
        del selection_window[: len(selection_window) - maxlen]


def _greedy_independent_set_ordered(
    graph: ElemGraph,
    order: list[int],
) -> list[int]:
    """Greedy MIS following ``order`` (no score re-sort)."""
    kept: list[int] = []
    kept_set: set[int] = set()
    for v in order:
        if any(u in kept_set for u in graph.collisions[v]):
            continue
        kept.append(v)
        kept_set.add(v)
    return kept


def _nest_outline_boundary(outline: BaseGeometry):
    if hasattr(outline, "exterior"):
        return outline.exterior
    return outline.boundary


def _outline_standoff_distance(poly, outline: BaseGeometry) -> float:
    return float(poly.distance(_nest_outline_boundary(outline)))


def _border_kiss_tolerance(min_dist: float, *, scale: float = 5.0) -> float:
    return max(min_dist * scale, 1e-6)


def _border_kiss_indices(
    polys: list,
    outline: BaseGeometry,
    min_dist: float,
) -> list[int]:
    tol = _border_kiss_tolerance(min_dist)
    return [
        i
        for i, poly in enumerate(polys)
        if abs(_outline_standoff_distance(poly, outline) - min_dist) <= tol
    ]


def _perimeter_sort_key(poly, outline: BaseGeometry) -> float:
    ring = _nest_outline_boundary(outline)
    touch = nearest_points(ring, poly.centroid)[0]
    return float(ring.project(touch))


def _expand_border_selection(
    graph: ElemGraph,
    polys: list,
    outline: BaseGeometry,
    min_dist: float,
    scores: list[float],
    initial: list[int],
) -> list[int]:
    """Greedy MIS on outline-kiss nodes, preserving ``initial`` and walking the perimeter."""
    kept_set = set(initial)
    marked = [False] * len(polys)
    for i in kept_set:
        marked[i] = True
        for j in graph.collisions[i]:
            marked[j] = True
    border = _border_kiss_indices(polys, outline, min_dist)
    order = sorted(
        border,
        key=lambda i: (
            _perimeter_sort_key(polys[i], outline),
            abs(_outline_standoff_distance(polys[i], outline) - min_dist),
            -scores[i],
        ),
    )
    for i in order:
        if marked[i]:
            continue
        kept_set.add(i)
        marked[i] = True
        for j in graph.collisions[i]:
            marked[j] = True
    return sorted(kept_set)


def _first_pass_border_ring_selection(
    graph: ElemGraph,
    polys: list,
    outline: BaseGeometry,
    min_dist: float,
    scores: list[float],
) -> list[int]:
    """Pack as many outline-kiss nodes as possible, walking the nest perimeter."""
    border = _border_kiss_indices(polys, outline, min_dist)
    if not border:
        return []
    order = sorted(
        border,
        key=lambda i: (
            _perimeter_sort_key(polys[i], outline),
            abs(_outline_standoff_distance(polys[i], outline) - min_dist),
            -scores[i],
        ),
    )
    return _greedy_independent_set_ordered(graph, order)


def _transform_row_key(t: np.ndarray) -> tuple[float, float, float]:
    return (round(float(t[0]), 4), round(float(t[1]), 4), round(float(t[2]), 4))


def _locked_graph_indices(
    transforms: list[np.ndarray],
    phase1_transforms: list[np.ndarray],
) -> list[int]:
    keys = {_transform_row_key(t) for t in phase1_transforms}
    return [i for i, t in enumerate(transforms) if _transform_row_key(t) in keys]


def _prepend_group_transforms(
    phase1: np.ndarray,
    batch: np.ndarray,
) -> np.ndarray:
    if phase1.shape[0] == 0:
        return batch
    if batch.shape[0] == 0:
        return phase1
    return dedupe_transforms(np.concatenate([phase1, batch], axis=0))


def _border_saturation_transform_batch(
    cfg: BuildGraphConfig,
    board: BaseGeometry,
    parts: list[tuple[Polygon, int]],
    nest_state: NestState,
) -> tuple[np.ndarray, np.ndarray]:
    """Propose-only batch for outline saturation (no random/history noise)."""
    min_dist = cfg.board_min_dist(first_pass=True)
    polys = nest_state.polys
    selected = nest_state.selected_indices
    group_id = nest_state.group_id
    transform = nest_state.transform
    part_by_gid = {gid: poly for poly, gid in parts}
    pack_polys = [polys[i] for i in selected]
    phase1_by_group: list[list[np.ndarray]] = [[], []]
    for idx in selected:
        phase1_by_group[group_id[idx]].append(transform[idx])
    propose_by_group = proposed_transforms_for_groups(
        board,
        parts,
        polys,
        selected,
        cfg.first_pass_propose_config(),
        min_dist=min_dist,
        border_only_propose=True,
        use_full_packed_obstacle=cfg.propose.use_full_packed_obstacle,
    )
    out: list[np.ndarray] = []
    for gid in range(len(phase1_by_group)):
        phase1 = (
            np.asarray(phase1_by_group[gid], dtype=np.float64)
            if phase1_by_group[gid]
            else np.zeros((0, 3))
        )
        proposed = propose_by_group.get(gid, np.zeros((0, 3)))
        augment_coords = _border_augment_coords(
            cfg,
            board,
            part_by_gid[gid],
            pack_polys,
            min_dist=min_dist,
        )
        augment = (
            np.asarray(augment_coords, dtype=np.float64)
            if augment_coords
            else np.zeros((0, 3))
        )
        extra = dedupe_transforms(
            np.concatenate([proposed, augment], axis=0),
        )
        cap = max(cfg.propose.first_pass_max_proposals * 4, 128)
        room = max(cap - phase1.shape[0], 0)
        if extra.shape[0] > room:
            extra = extra[:room]
        merged = dedupe_transforms(np.concatenate([phase1, extra], axis=0))
        out.append(merged)
    return (out[0], out[1])


def _is_border_kiss_poly(poly, outline: BaseGeometry, min_dist: float) -> bool:
    tol = _border_kiss_tolerance(min_dist)
    err = abs(_outline_standoff_distance(poly, outline) - min_dist)
    return err <= tol


def _clear_of_polys(poly, others: list, min_dist: float) -> bool:
    for other in others:
        if poly.intersects(other):
            return False
        if poly.distance(other) < min_dist - 1e-9:
            return False
    return True


def _outline_anchor_inward(
    poly,
    outline: BaseGeometry,
) -> tuple[Point, tuple[float, float]]:
    ring = _nest_outline_boundary(outline)
    if hasattr(outline, "representative_point"):
        interior = outline.representative_point()
    else:
        interior = ring.interpolate(0.5, normalized=True)
    anchor, _ = nearest_points(ring, poly)
    ox = anchor.x - interior.x
    oy = anchor.y - interior.y
    dist = float(np.hypot(ox, oy))
    if dist < 1e-9:
        return anchor, (-1.0, -1.0)
    return anchor, (ox / dist, oy / dist)


def _border_tightness_cost(
    polys: list,
    outline: BaseGeometry,
    min_dist: float,
) -> float:
    """Lower is tighter: excess neighbor gap plus outline standoff error."""
    if not polys:
        return 0.0
    kiss = sum(
        abs(_outline_standoff_distance(poly, outline) - min_dist)
        for poly in polys
    )
    if len(polys) < 2:
        return kiss
    excess_gap = 0.0
    for i, poly in enumerate(polys):
        nearest = min(
            (poly.distance(other) for j, other in enumerate(polys) if j != i),
            default=float("inf"),
        )
        if nearest < float("inf"):
            excess_gap += max(0.0, nearest - min_dist)
    return 2.0 * excess_gap + 0.25 * kiss


def _border_refine_kiss_ok(poly, outline: BaseGeometry, min_dist: float) -> bool:
    tol = _border_kiss_tolerance(min_dist, scale=8.0)
    err = abs(_outline_standoff_distance(poly, outline) - min_dist)
    return err <= tol


def _border_refine_candidates(
    x: float,
    y: float,
    theta: float,
    g,
    *,
    min_dist: float,
    max_props: int,
) -> list[tuple[float, float, float]]:
    """Cast snaps plus fractional slide steps along guidance propositions."""
    out: list[tuple[float, float, float]] = []
    seen: set[tuple[float, float, float]] = set()

    def add(coords: tuple[float, float, float]) -> None:
        key = (round(coords[0], 4), round(coords[1], 4), round(coords[2], 4))
        if key in seen:
            return
        seen.add(key)
        out.append(coords)

    for prop in _sorted_guidance_propositions(g)[:max_props]:
        use_cast = not g.is_penetrating and _is_cast_move(prop.move_type or "")
        add(_candidate_from_proposition(x, y, theta, prop, use_full_cast=use_cast))
        if use_cast:
            continue
        tx, ty = proposition_translation(prop)
        mag = math.hypot(tx, ty)
        if mag < 1e-9:
            continue
        step_scale = 0.2
        step_len = step_scale * max(min_dist, 1e-4)
        if not g.is_penetrating:
            step_len = step_scale * mag
        for frac in (0.4, 0.7, 1.0):
            nx = x + tx / mag * step_len * frac
            ny = y + ty / mag * step_len * frac
            add((nx, ny, theta))
    return out


def _guidance_border_refine(
    cfg: BuildGraphConfig,
    board: BaseGeometry,
    parts: list[tuple[Polygon, int]],
    *,
    outline: BaseGeometry,
    pack_polys: list,
    pack_gids: list[int],
    pack_tr: list[np.ndarray],
) -> tuple[list, list[int], list[np.ndarray]]:
    """Tighten border ring placements using per-anchor guidance casts."""
    passes = max(cfg.propose.first_pass_guidance_refine_passes, 0)
    if passes <= 0 or len(pack_polys) < 2:
        return pack_polys, pack_gids, pack_tr

    min_dist = cfg.board_min_dist(first_pass=True)
    eps = cfg.placement_epsilon_ratio(first_pass=True)
    propose_cfg = cfg.first_pass_propose_config()
    sheet, voids = board_context_from_geometry(board)
    board_geom = Geometry.from_shapely(sheet)
    pad = default_sheet_padding(board)
    bounds = padded_board_bounds(board, pad)
    part_by_gid = {gid: poly for poly, gid in parts}
    bases = {gid: Geometry.from_shapely(part_by_gid[gid]) for gid in part_by_gid}

    polys = list(pack_polys)
    gids = list(pack_gids)
    trs = [np.asarray(t, dtype=np.float64) for t in pack_tr]
    geoms = [bases[gid].apply_transform(t) for gid, t in zip(gids, trs, strict=True)]
    max_props = max(propose_cfg.guidance_max_propositions, 1)

    for _ in range(passes):
        base_cost = _border_tightness_cost(polys, outline, min_dist)
        improved = False
        order = sorted(
            range(len(polys)),
            key=lambda i: _perimeter_sort_key(polys[i], outline),
        )
        for idx in order:
            gid = gids[idx]
            part_poly = part_by_gid[gid]
            x, y, theta = float(trs[idx][0]), float(trs[idx][1]), float(trs[idx][2])
            anchor, inward = _outline_anchor_inward(polys[idx], outline)
            others_geoms = [geoms[j] for j in range(len(geoms)) if j != idx]
            others_polys = [polys[j] for j in range(len(polys)) if j != idx]
            scene = placement_scene_for_part(
                sheet, board_geom, voids, bases[gid], base_geoms=others_geoms,
            )
            edge_cfg = guidance_config_for_board_edge_anchor(
                anchor,
                inward,
                min_dist=min_dist,
                board_bounds=bounds,
                epsilon_ratio=eps,
                target_angle_rad=theta,
                max_propositions=max_props,
                use_tight_packing=propose_cfg.guidance_use_tight_packing,
                use_corner_alignment=propose_cfg.guidance_use_corner_alignment,
                enable_grid_exploration=propose_cfg.guidance_enable_grid,
                diversity_dist_ratio=propose_cfg.guidance_diversity_dist_ratio,
            )
            placed = scene.placed_at((x, y, theta))
            g = scene.guidance(placed, (x, y), edge_cfg)
            best_coords: tuple[float, float, float] | None = None
            best_cost = base_cost
            for candidate in _border_refine_candidates(
                x, y, theta, g, min_dist=min_dist, max_props=max_props,
            ):
                cand_poly = transform_poly(part_poly, candidate)
                if not _border_refine_kiss_ok(cand_poly, outline, min_dist):
                    continue
                if not _clear_of_polys(cand_poly, others_polys, min_dist):
                    continue
                cand_geom = bases[gid].apply_transform(np.asarray(candidate, dtype=np.float64))
                if not is_valid_placement(
                    scene,
                    cand_geom,
                    cand_geom.center(),
                    min_dist,
                    edge_cfg,
                    skip_footprint=True,
                ):
                    continue
                trial_polys = list(polys)
                trial_polys[idx] = cand_poly
                cost = _border_tightness_cost(trial_polys, outline, min_dist)
                if cost + 1e-9 < best_cost:
                    best_coords = candidate
                    best_cost = cost
            if best_coords is None:
                continue
            polys[idx] = transform_poly(part_poly, best_coords)
            trs[idx] = np.asarray(best_coords, dtype=np.float64)
            geoms[idx] = bases[gid].apply_transform(trs[idx])
            base_cost = best_cost
            improved = True
        if not improved:
            break

    return polys, gids, trs


def _border_pack_graph(
    pack_polys: list,
    pack_gids: list[int],
    pack_tr: list[np.ndarray],
) -> tuple[ElemGraph, list, list[int], list[np.ndarray], list[int]]:
    placed_geoms = [Geometry.from_shapely(p) for p in pack_polys]
    graph = ElemGraph()
    bboxes = [g.bounds() for g in placed_geoms]
    graph.reserve_elems(len(placed_geoms))
    for n, geom in enumerate(placed_geoms):
        cx, cy = geom.center()
        graph.append_elem(
            pack_gids[n],
            Vec2(x=cx, y=cy),
            Circle.from_center_radius(cx, cy, geom.radius()),
        )
        for j in _collision_partners_vs_existing(
            geom, bboxes[n], placed_geoms[:n], bboxes[:n],
        ):
            graph.add_collision(n, j)
    selected_out = list(range(len(pack_polys)))
    assert selection_is_independent(graph, selected_out)
    return graph, pack_polys, pack_gids, pack_tr, selected_out


def _border_augment_coords(
    cfg: BuildGraphConfig,
    board: BaseGeometry,
    part_poly: Polygon,
    placed_polys: list,
    *,
    min_dist: float,
) -> list[tuple[float, float, float]]:
    """Fast outline candidates: board snap + chain fit against full packed union."""
    from nest_graph.propose.placements_edge import (
        propose_placements_board_edge,
        propose_placements_group_fit,
    )
    from nest_graph.propose.placements_primary import (
        propose_placements_neighbor_slide,
        propose_placements_perimeter_walk,
    )
    from nest_graph.propose.context import border_focal_for_propose, propose_push_point
    from nest_graph.propose.geometry import ProposeGeometry

    sheet, _ = board_context_from_geometry(board)
    obstacle = unary_union(placed_polys) if placed_polys else Polygon()
    propose_cfg = cfg.first_pass_propose_config()
    push = propose_push_point(
        board,
        obstacle,
        smart_push=propose_cfg.smart_push_target,
        min_dist=min_dist,
        use_border_focus=True,
    )
    geom = ProposeGeometry(
        board,
        obstacle,
        part_poly,
        min_dist,
        epsilon_ratio=propose_cfg.placement_clearance_epsilon_ratio,
        propose_cfg=propose_cfg,
    )
    coords: list[tuple[float, float, float]] = []
    coords.extend(
        propose_placements_board_edge(
            part_poly,
            sheet,
            Polygon(),
            min_dist=min_dist,
            propose_cfg=propose_cfg,
            propose_geom=geom,
            pt_push=push,
            top_n=propose_cfg.first_pass_max_proposals,
            samples_per_edge=propose_cfg.board_edge_samples_per_edge,
        ),
    )
    if not obstacle.is_empty:
        focal = unary_union([border_focal_for_propose(board, min_dist), obstacle])
        coords.extend(
            propose_placements_group_fit(
                focal,
                part_poly,
                sheet,
                obstacle,
                min_dist=min_dist,
                top_n=propose_cfg.first_pass_max_proposals,
                samples_per_edge=propose_cfg.first_pass_group_edge_samples_per_edge,
                propose_geom=geom,
                pt_push=push,
            ),
        )
        coords.extend(
            propose_placements_neighbor_slide(
                obstacle,
                part_poly,
                sheet,
                min_dist,
                propose_geom=geom,
                pt_push=push,
                num_angles=propose_cfg.placement_num_angles,
                top_n=propose_cfg.first_pass_max_proposals,
            ),
        )
        coords.extend(
            propose_placements_perimeter_walk(
                obstacle,
                part_poly,
                sheet,
                min_dist,
                propose_geom=geom,
                pt_push=push,
                use_free_region=False,
                border_focus=True,
                num_angles=propose_cfg.placement_num_angles,
                top_n=propose_cfg.first_pass_max_proposals // 2,
            ),
        )
    seen: set[tuple[float, float, float]] = set()
    out: list[tuple[float, float, float]] = []
    for c in coords:
        key = (round(c[0], 3), round(c[1], 3), round(c[2], 2))
        if key in seen:
            continue
        seen.add(key)
        out.append(c)
    return out


def _sequential_border_augment(
    cfg: BuildGraphConfig,
    board: BaseGeometry,
    parts: list[tuple[Polygon, int]],
    *,
    outline: BaseGeometry,
    polys: list,
    group_id: list[int],
    transform: list[np.ndarray],
    selected: list[int],
) -> tuple[ElemGraph, list, list[int], list[np.ndarray], list[int]]:
    """Fill outline gaps by proposing against the full packed union each step."""
    min_dist = cfg.board_min_dist(first_pass=True)
    eps = cfg.placement_epsilon_ratio(first_pass=True)
    sheet, voids = board_context_from_geometry(board)
    board_geom = Geometry.from_shapely(sheet)
    pad = default_sheet_padding(board)
    guidance_cfg = guidance_config_for_graph(
        min_dist,
        board_bounds=padded_board_bounds(board, pad),
        epsilon_ratio=eps,
    )
    part_by_gid = {gid: poly for poly, gid in parts}
    bases = {gid: Geometry.from_shapely(part_by_gid[gid]) for gid in part_by_gid}

    pack_polys = [polys[i] for i in selected]
    pack_gids = [group_id[i] for i in selected]
    pack_tr = [transform[i] for i in selected]
    placed_geoms = [Geometry.from_shapely(p) for p in pack_polys]
    max_rounds = max(cfg.propose.first_pass_sequential_augment_max, 0)

    for _ in range(max_rounds):
        candidates: list[tuple[float, int, np.ndarray, Polygon, Geometry]] = []
        for gid, part_poly in part_by_gid.items():
            for c in _border_augment_coords(
                cfg, board, part_poly, pack_polys, min_dist=min_dist,
            ):
                coords = np.asarray(c, dtype=np.float64)
                shapely_placed = transform_poly(part_poly, coords)
                if not _is_border_kiss_poly(shapely_placed, outline, min_dist):
                    continue
                if not _clear_of_polys(shapely_placed, pack_polys, min_dist):
                    continue
                geom = bases[gid].apply_transform(coords)
                scene = placement_scene_for_part(
                    sheet, board_geom, voids, bases[gid], base_geoms=placed_geoms,
                )
                if not is_valid_placement(
                    scene, geom, geom.center(), min_dist, guidance_cfg,
                    skip_footprint=True,
                ):
                    continue
                cost = abs(_outline_standoff_distance(shapely_placed, outline) - min_dist)
                candidates.append((cost, gid, coords, shapely_placed, geom))
        if not candidates:
            break
        candidates.sort(key=lambda row: row[0])
        added: list[tuple[int, np.ndarray, Polygon, Geometry]] = []
        for cost, gid, coords, shapely_placed, geom in candidates:
            blockers = pack_polys + [row[2] for row in added]
            if not _clear_of_polys(shapely_placed, blockers, min_dist):
                continue
            added.append((gid, coords, shapely_placed, geom))
        if not added:
            break
        for gid, coords, shapely_placed, geom in added:
            pack_polys.append(shapely_placed)
            pack_gids.append(gid)
            pack_tr.append(coords)
            placed_geoms.append(geom)

    pack_polys, pack_gids, pack_tr = _guidance_border_refine(
        cfg,
        board,
        parts,
        outline=outline,
        pack_polys=pack_polys,
        pack_gids=pack_gids,
        pack_tr=pack_tr,
    )
    return _border_pack_graph(pack_polys, pack_gids, pack_tr)


def _first_pass_layered_selection(
    cfg: BuildGraphConfig,
    board: BaseGeometry,
    parts: list[tuple[Polygon, int]],
    *,
    graph: ElemGraph,
    p1: Polygon,
    p2: Polygon,
    selected_t: tuple[np.ndarray, np.ndarray],
    history: tuple[np.ndarray, np.ndarray],
    rng: np.random.Generator,
    selection_window: list[tuple[np.ndarray, np.ndarray]] | None,
    polys: list,
    group_id: list[int],
    transform: list[np.ndarray],
    phase1_selected: list[int],
    rule_set: PlacementRuleSet,
    scores: list[float],
    selection: SelectionConfig,
) -> tuple[ElemGraph, list, list[int], list[np.ndarray], list[int]]:
    """Rebuild with packed obstacles; saturate outline-kiss placements along the perimeter."""
    min_dist = cfg.board_min_dist(first_pass=True)
    outline = board
    graph_cur = graph
    polys_cur = polys
    group_id_cur = group_id
    transform_cur = transform
    sel_cur = list(phase1_selected)
    passes = max(cfg.propose.first_pass_border_saturation_passes, 0)

    for _ in range(passes):
        phase1_by_group: list[list[np.ndarray]] = [[], []]
        for idx in sel_cur:
            phase1_by_group[group_id_cur[idx]].append(transform_cur[idx])
        phase1_t = tuple(
            np.asarray(rows, dtype=np.float64) if rows else np.zeros((0, 3))
            for rows in phase1_by_group
        )
        nest_state = NestState(
            polys=polys_cur,
            group_id=group_id_cur,
            transform=transform_cur,
            selected_indices=list(sel_cur),
        )
        batch2 = _border_saturation_transform_batch(
            cfg,
            board,
            parts,
            nest_state,
        )
        combined = (
            _prepend_group_transforms(phase1_t[0], batch2[0]),
            _prepend_group_transforms(phase1_t[1], batch2[1]),
        )
        graph2, polys2, group_id2, transform2 = make_polygon_graph(
            board,
            [(p1, combined[0]), (p2, combined[1])],
            min_dist=min_dist,
            epsilon_ratio=cfg.placement_epsilon_ratio(first_pass=True),
        )
        locked = _locked_graph_indices(
            transform2,
            [transform_cur[i] for i in sel_cur],
        )
        if not locked:
            break
        scores2 = score_elems(graph2, rule_set)
        new_sel = _expand_border_selection(
            graph2, polys2, outline, min_dist, scores2, locked,
        )
        graph_cur = graph2
        polys_cur = polys2
        group_id_cur = group_id2
        transform_cur = transform2
        if len(new_sel) <= len(sel_cur):
            sel_cur = new_sel
            break
        sel_cur = new_sel

    if cfg.propose.first_pass_sequential_augment_max > 0:
        return _sequential_border_augment(
            cfg,
            board,
            parts,
            outline=outline,
            polys=polys_cur,
            group_id=group_id_cur,
            transform=transform_cur,
            selected=sel_cur,
        )

    pack_polys = [polys_cur[i] for i in sel_cur]
    pack_gids = [group_id_cur[i] for i in sel_cur]
    pack_tr = [transform_cur[i] for i in sel_cur]
    pack_polys, pack_gids, pack_tr = _guidance_border_refine(
        cfg,
        board,
        parts,
        outline=outline,
        pack_polys=pack_polys,
        pack_gids=pack_gids,
        pack_tr=pack_tr,
    )
    return _border_pack_graph(pack_polys, pack_gids, pack_tr)


def _boost_border_scores(
    polys: list,
    scores: list[float],
    outline: BaseGeometry,
    min_dist: float,
    *,
    weight: float = 8.0,
) -> None:
    """Favor graph nodes flush to the nest outline when scoring nest/DFS."""
    scale = _border_kiss_tolerance(min_dist)
    for i, sc in enumerate(scores):
        err = abs(_outline_standoff_distance(polys[i], outline) - min_dist)
        scores[i] = sc + weight * max(0.0, 1.0 - err / scale)


def _build_transform_batch(
    cfg: BuildGraphConfig,
    selected_t: tuple[np.ndarray, np.ndarray],
    history: tuple[np.ndarray, np.ndarray],
    rng: np.random.Generator,
    *,
    board: BaseGeometry | None = None,
    parts: list[tuple[Polygon, int]] | None = None,
    nest_state: NestState | None = None,
    selection_window: list[tuple[np.ndarray, np.ndarray]] | None = None,
    first_pass: bool = False,
    border_saturation: bool = False,
) -> tuple[np.ndarray, np.ndarray]:
    sc = cfg.sampling
    scale = sc.transform_scale
    propose_by_group: dict[int, np.ndarray] = {}
    border_pin_by_group: dict[int, np.ndarray] = {}
    empty_sheet = (
        nest_state is None
        or not nest_state.selected_indices
    )
    if (
        board is not None
        and parts is not None
        and cfg.propose.max_proposals > 0
    ):
        polys = nest_state.polys if nest_state is not None else []
        selected = nest_state.selected_indices if nest_state is not None else []
        min_dist = cfg.board_min_dist(first_pass=first_pass)
        propose_cfg = (
            cfg.first_pass_propose_config() if first_pass else cfg.propose
        )
        border_only = (
            (empty_sheet and cfg.propose.first_pass_empty_border_only)
            or (border_saturation and cfg.propose.first_pass_border_pack)
        )
        propose_by_group = proposed_transforms_for_groups(
            board,
            parts,
            polys,
            selected,
            propose_cfg,
            min_dist=min_dist,
            border_only_propose=border_only,
            use_full_packed_obstacle=(
                cfg.propose.use_full_packed_obstacle and not empty_sheet
            ),
        )
        if empty_sheet and cfg.propose.use_board_edge_seeds:
            for part_poly, group_id in parts:
                border_pin_by_group[group_id] = border_edge_transforms_for_group(
                    board,
                    part_poly,
                    Polygon(),
                    propose_cfg,
                    min_dist=min_dist,
                )

    window_t = window_selected_transforms(selection_window)

    def one_group(
        group_id: int,
        sel: np.ndarray,
        hist: np.ndarray,
        window: np.ndarray,
    ) -> np.ndarray:
        batch_parts: list[np.ndarray] = []
        pinned = border_pin_by_group.get(group_id, np.zeros((0, 3)))
        proposed = propose_by_group.get(group_id, np.zeros((0, 3)))
        border_batch = (
            empty_sheet
            and cfg.propose.first_pass_border_pack
            and cfg.propose.first_pass_empty_border_only
        )
        if border_batch:
            if pinned.shape[0] > 0:
                batch_parts.append(pinned)
                jitter_n = sc.structured_jitter_per_proposal_empty
                if jitter_n > 0:
                    jittered = expand_structured_transforms(
                        pinned,
                        cfg.propose.structured_jitter_border_scale,
                        jitter_n,
                    )
                    if jittered.shape[0] > 0:
                        batch_parts.append(jittered)
            if proposed.shape[0] > 0:
                batch_parts.append(proposed)
        elif proposed.shape[0] > 0:
            batch_parts.append(proposed)
            if pinned.shape[0] == 0:
                jitter_n = (
                    sc.structured_jitter_per_proposal_empty
                    if empty_sheet
                    else sc.structured_jitter_per_proposal
                )
                jittered = expand_structured_transforms(
                    proposed,
                    sc.structured_jitter_scale,
                    jitter_n,
                )
                if jittered.shape[0] > 0:
                    batch_parts.append(jittered)
        n_random = (
            cfg.propose.random_per_iter_empty_border
            if empty_sheet and cfg.propose.use_border_focus and not border_batch
            else (
                sc.random_per_iter_when_proposed
                if proposed.shape[0] > 0
                else sc.random_per_iter
            )
        )
        batch_parts.append(rng.uniform(-1, 1, (n_random, 3)) * scale)
        if hist.shape[0] > 0:
            batch_parts.append(hist)
        if sel.shape[0] > 0:
            batch_parts.append(sel)
            batch_parts.extend(transform_selection(sel, sc.selection_expand_n))
            batch_parts.extend(transform_history(hist, sc.history_expand_n))
        if window.shape[0] > 0:
            batch_parts.append(window)
            batch_parts.extend(transform_selection(window, sc.selection_expand_n))
        if sc.shuffle_passes > 0 and (
            sel.shape[0] > 0 or hist.shape[0] > 0 or window.shape[0] > 0
        ):
            for _ in range(sc.shuffle_passes):
                batch_parts.append(
                    transform_shuffle_mix(
                        sel, hist, sc.shuffle_per_pass, rng, sc.shuffle_scale,
                    )
                )
        merged = dedupe_transforms(np.concatenate(batch_parts))
        merged = shuffle_transforms(merged, rng)
        return subsample_transforms_with_pinned(
            merged, pinned, sc.max_transforms_per_group, rng,
        )

    return (
        one_group(0, selected_t[0], history[0], window_t[0]),
        one_group(1, selected_t[1], history[1], window_t[1]),
    )


def _make_demo_rule_set(cfg: BuildGraphConfig) -> PlacementRuleSet:
    rc = cfg.rules
    r = rc.place_rule_radius
    wrect = rc.weight_rect
    wtri = rc.weight_tri
    aw = rc.angle_rule_weight_scale * wtri
    rule_set = PlacementRuleSet()
    rule_set.append_rule(PointPlaceRule(pos=Vec2(x=0, y=0), r=r, w=wrect, group=0))
    rule_set.append_rule(PointPlaceRule(pos=Vec2(x=0.7, y=0.7), r=r, w=wtri, group=1))
    rule_set.append_rule(PointPlaceRule(pos=Vec2(x=0, y=1.1), r=r, w=wtri, group=1))
    rule_set.append_rule(PointPlaceRule(pos=Vec2(x=1.2, y=0), r=r, w=wtri, group=1))
    for gi, wgt in ((0, wrect), (1, wtri)):
        for k in range(8):
            a = float(2.0 * np.pi * k / 8.0)
            rule_set.append_rule(
                PointAngleRule(
                    pos=Vec2(x=0.6, y=0.55),
                    r=r,
                    a=a,
                    w=aw * wgt,
                    group=gi,
                ),
            )
    return rule_set


def prune_selection_to_independent_set(
    graph: ElemGraph,
    selected: list[int],
    scores: list[float] | None = None,
) -> list[int]:
    """Greedy MIS fallback (prefer finalize_selection for score-optimal drops)."""
    if not selected:
        return []
    order = list(selected)
    if scores is not None and len(scores) == len(graph.group_id):
        order.sort(key=lambda v: scores[v], reverse=True)
    kept: list[int] = []
    kept_set: set[int] = set()
    for v in order:
        if any(u in kept_set for u in graph.collisions[v]):
            continue
        kept.append(v)
        kept_set.add(v)
    return kept


def _refine_options(
    sel: SelectionConfig,
    *,
    loose: bool,
    max_tries: int | None = None,
) -> RefineSelectionOptions:
    opts = RefineSelectionOptions()
    opts.max_tries = sel.dfs_max_tries if max_tries is None else max_tries
    opts.max_passes = sel.dfs_refine_max_passes
    opts.max_stagnant_passes = sel.dfs_refine_max_stagnant_passes
    opts.beam_width = sel.dfs_refine_beam_width
    if loose:
        opts.min_collisions = 2
        opts.max_root_collisions = 2
    else:
        opts.min_collisions = 1
        opts.max_root_collisions = 1
    return opts


def _finalize_options(sel: SelectionConfig) -> FinalizeSelectionOptions:
    opts = FinalizeSelectionOptions()
    opts.repair_passes = sel.dfs_finalize_repair_passes
    opts.max_exact_component_size = sel.dfs_finalize_max_component
    return opts


def _loose_refine_options(sel: SelectionConfig) -> RefineSelectionOptions:
    return _refine_options(sel, loose=True)


def _tight_refine_options(sel: SelectionConfig) -> RefineSelectionOptions:
    return _refine_options(sel, loose=False)


def _strict_refine_options(sel: SelectionConfig) -> RefineSelectionOptions:
    opts = _refine_options(sel, loose=False)
    opts.min_collisions = 0
    opts.max_root_collisions = 0
    return opts


def _head_loose_refine_options(sel: SelectionConfig) -> RefineSelectionOptions:
    """HEAD-style score DFS: allow transient overlaps during search."""
    return _refine_options(sel, loose=True)


def selection_score_sum(scores: list[float], selected: list[int]) -> float:
    return float(sum(scores[v] for v in selected))


def apply_dfs_refinement(
    graph: ElemGraph,
    rule_set: PlacementRuleSet,
    selected: list[int],
    scores: list[float],
    *,
    dfs_passes: int | None = None,
    dfs_max_tries: int | None = None,
    mode: str | None = None,
    selection: SelectionConfig | None = None,
) -> tuple[list[int], list[int], float]:
    """Refine selection; return (pre_finalize, final, score_sum_final)."""
    sel = selection if selection is not None else SelectionConfig()
    passes = dfs_passes if dfs_passes is not None else sel.dfs_passes
    max_tries = dfs_max_tries if dfs_max_tries is not None else sel.dfs_max_tries
    mode = mode if mode is not None else sel.dfs_mode
    finalize_opts = _finalize_options(sel)

    selected = list(selected)
    graph_sorted = sort_graph(graph, rule_set)
    graph_sorted_rev = sort_graph(graph, rule_set, reverse=True)
    pre_finalize = selected

    def _finalize() -> list[int]:
        return list(finalize_selection(graph, selected, scores, finalize_opts))

    if mode == "nest_only":
        return selected, selected, selection_score_sum(scores, selected)

    if mode == "legacy_alternating":
        for _ in range(passes):
            selected = list(increase_selection_dfs(
                graph_sorted_rev, selected, max_tries,
            ))
            selected = list(increase_selection_dfs(graph, selected, max_tries))
            selected = list(increase_score_dfs(graph_sorted_rev, selected, scores))
            selected = list(increase_selection_dfs(
                graph_sorted, selected, max_tries,
            ))
            selected = list(increase_score_dfs(graph_sorted, selected, scores))
        pre_finalize = selected
        final = _finalize()
        return pre_finalize, final, selection_score_sum(scores, final)

    if mode == "head_pipeline":
        loose = _head_loose_refine_options(sel)
        tight = RefineSelectionOptions()
        tight.min_collisions = 1
        tight.max_root_collisions = 2
        tight.max_passes = sel.dfs_refine_max_passes
        tight.max_stagnant_passes = sel.dfs_refine_max_stagnant_passes
        tight.beam_width = sel.dfs_refine_beam_width
        for _ in range(passes):
            selected = list(increase_selection_dfs(
                graph_sorted_rev, selected, max_tries,
            ))
            selected = list(increase_selection_dfs(graph, selected, max_tries))
            selected = list(increase_score_dfs(
                graph_sorted_rev, selected, scores, loose,
            ))
            selected = list(increase_selection_dfs(
                graph_sorted, selected, max_tries,
            ))
            selected = list(increase_score_dfs(graph_sorted, selected, scores, tight))
        pre_finalize = selected
        return pre_finalize, pre_finalize, selection_score_sum(scores, pre_finalize)

    if mode == "strict_no_prune":
        strict = _strict_refine_options(sel)
        for _ in range(passes):
            selected = list(refine_selection(graph_sorted_rev, selected, scores, strict))
            selected = list(refine_selection(graph, selected, scores, strict))
        pre_finalize = selected
        return pre_finalize, pre_finalize, selection_score_sum(scores, pre_finalize)

    if mode == "strict_prune":
        strict = _strict_refine_options(sel)
        for _ in range(passes):
            selected = list(refine_selection(graph_sorted_rev, selected, scores, strict))
            selected = list(refine_selection(graph, selected, scores, strict))
        pre_finalize = selected
        final = prune_selection_to_independent_set(graph, selected, scores)
        return pre_finalize, final, selection_score_sum(scores, final)

    loose = _loose_refine_options(sel)
    tight = _tight_refine_options(sel)

    if mode == "merged_single_pass":
        for _ in range(passes):
            selected = list(refine_selection(graph_sorted_rev, selected, scores, loose))
            pre_finalize = selected
            final = _finalize()
        return pre_finalize, final, selection_score_sum(scores, final)

    if mode == "merged_loose_finalize_end":
        for _ in range(passes):
            selected = list(refine_selection(graph_sorted_rev, selected, scores, loose))
        pre_finalize = selected
        final = _finalize()
        return pre_finalize, final, selection_score_sum(scores, final)

    if mode in ("merged_loose_tight_finalize_end", "high_pass_loose"):
        for _ in range(passes):
            selected = list(refine_selection(graph_sorted_rev, selected, scores, loose))
            selected = list(refine_selection(graph, selected, scores, tight))
        pre_finalize = selected
        final = _finalize()
        return pre_finalize, final, selection_score_sum(scores, final)

    # merged_loose_tight: finalize after each outer pass
    for _ in range(passes):
        selected = list(refine_selection(graph_sorted_rev, selected, scores, loose))
        selected = list(refine_selection(graph, selected, scores, tight))
    pre_finalize = selected
    final = _finalize()
    return pre_finalize, final, selection_score_sum(scores, final)


def _make_seed_rule_sets(cfg: BuildGraphConfig) -> list[PlacementRuleSet]:
    first = PlacementRuleSet()
    first.append_rule(PointPlaceRule(pos=Vec2(x=0, y=0), r=0.1, w=0.1, group=0))
    first.append_rule(PointPlaceRule(pos=Vec2(x=0, y=0), r=0.1, w=0.1, group=1))
    return [first]


def _make_initial_rule_sets(cfg: BuildGraphConfig) -> list[PlacementRuleSet]:
    return _make_seed_rule_sets(cfg) + [_make_demo_rule_set(cfg)]


def run_build_graph(cfg: BuildGraphConfig) -> None:
    """Demo build loop: propose → graph → rules → nest → DFS refine.

    Shipped defaults match nest-pipeline tuning (``SelectionConfig`` merged_loose_tight,
    ``dfs_passes=3``, ``improve_rules_rounds=4``). See docs/build_graph_tuning.md.
    """
    rng = cfg.apply_seed()
    sc = cfg.sampling
    gc = cfg.graph
    sel = cfg.selection
    out = cfg.output

    p_board = cfg.rules.board_polygon()
    sheet_pad = cfg.rules.effective_sheet_padding()
    p1 = cfg.rules.rect_polygon()
    p2 = cfg.rules.tri_polygon()

    selected_t = (
        rng.uniform(-1, 1, (sc.initial_random, 3)) * sc.transform_scale,
        rng.uniform(-1, 1, (sc.initial_random, 3)) * sc.transform_scale,
    )
    rule_sets = _make_initial_rule_sets(cfg)
    render_size = (out.render_size, out.render_size)
    video = cv.VideoWriter(
        out.video_path,
        cv.VideoWriter_fourcc(*'mp4v'),
        out.video_fps,
        render_size,
    )
    history = (np.zeros((1, 3)), np.zeros((1, 3)))
    graphs: list[ElemGraph] = []
    selection_window: list[tuple[np.ndarray, np.ndarray]] = []
    nest_state: NestState | None = None
    parts = [(p1, 0), (p2, 1)]
    board_area = p_board.area
    part_areas = (p1.area, p2.area)
    iters = tuple(range(out.n_iters))
    pbar = tqdm(
        iters,
        desc="Nesting",
        unit="iter",
        dynamic_ncols=True,
        disable=not out.progress,
    )

    for _it in pbar:
        selected_t = _build_transform_batch(
            cfg,
            selected_t,
            history,
            rng,
            board=p_board,
            parts=parts,
            nest_state=nest_state,
            selection_window=selection_window,
            first_pass=nest_state is None,
        )
        first_pass = nest_state is None
        graph, polys, group_id, transform = make_polygon_graph(
            p_board,
            [(p1, selected_t[0]), (p2, selected_t[1])],
            min_dist=cfg.board_min_dist(first_pass=first_pass),
            epsilon_ratio=cfg.placement_epsilon_ratio(first_pass=first_pass),
        )
        graphs.append(graph)
        graphs = graphs[-gc.graphs_window:]
        first_pass = nest_state is None
        seed_rules = active_rule_set(_make_seed_rule_sets(cfg))
        if first_pass and cfg.propose.first_pass_border_pack:
            scores = score_elems(graph, seed_rules)
            sheet, _ = board_context_from_geometry(p_board)
            min_dist = cfg.board_min_dist(first_pass=True)
            selected_polys = _first_pass_border_ring_selection(
                graph, polys, p_board, min_dist, scores,
            )
            old_len = 0
            if not selected_polys:
                selected_polys = list(nest_by_graph(graph, [seed_rules])[0])
                old_len = len(selected_polys)
            if cfg.propose.first_pass_layered_pack and selected_polys:
                graph, polys, group_id, transform, selected_polys = (
                    _first_pass_layered_selection(
                        cfg,
                        p_board,
                        parts,
                        graph=graph,
                        p1=p1,
                        p2=p2,
                        selected_t=selected_t,
                        history=history,
                        rng=rng,
                        selection_window=selection_window,
                        polys=polys,
                        group_id=group_id,
                        transform=transform,
                        phase1_selected=list(selected_polys),
                        rule_set=seed_rules,
                        scores=scores,
                        selection=sel,
                    )
                )
            for round_idx in range(sel.improve_rules_rounds):
                rule_sets = improve_rules(
                    graphs,
                    rule_sets,
                    sel.rules_kept,
                    p_board,
                    mutation_presets=cfg.rules.mutation_presets(),
                    rule_score_penalty=sel.rule_score_penalty,
                    elite_count=sel.improve_rules_elite_count,
                    seed=int(rng.integers(0, 2**31)) + round_idx,
                    score_options=score_rules_options(sel),
                    max_rules_per_set=cfg.rules.max_rules_per_set,
                )
        else:
            for round_idx in range(sel.improve_rules_rounds):
                rule_sets = improve_rules(
                    graphs,
                    rule_sets,
                    sel.rules_kept,
                    p_board,
                    mutation_presets=cfg.rules.mutation_presets(),
                    rule_score_penalty=sel.rule_score_penalty,
                    elite_count=sel.improve_rules_elite_count,
                    seed=int(rng.integers(0, 2**31)) + round_idx,
                    score_options=score_rules_options(sel),
                    max_rules_per_set=cfg.rules.max_rules_per_set,
                )
            active_rules = active_rule_set(rule_sets)
            scores = score_elems(graph, active_rules)
            sheet, _ = board_context_from_geometry(p_board)
            min_dist = cfg.board_min_dist(first_pass=first_pass)
            selected_polys = nest_by_graph(graph, rule_sets[: sel.nest_rule_sets_used])[0]
            old_len = len(selected_polys)
            if first_pass and should_use_border_focus(Polygon(), cfg.propose):
                _boost_border_scores(
                    polys, scores, p_board, min_dist,
                    weight=cfg.propose.border_selection_score_boost,
                )
            _, selected_polys, _ = apply_dfs_refinement(
                graph,
                active_rules,
                list(selected_polys),
                scores,
                selection=sel,
            )
        assert selection_is_independent(graph, selected_polys)
        cov = _selection_coverage_pct(
            selected_polys, group_id, part_areas, board_area,
        )
        if out.progress:
            pbar.set_postfix(
                parts=len(selected_polys),
                cov=f"{cov:.1f}%",
                pool=len(polys),
                refine=f"{old_len}->{len(selected_polys)}",
                ordered=True,
            )
        else:
            print(
                len(polys), old_len, "->", len(selected_polys),
                f"cov={cov:.1f}%",
            )
        render_frame = shapely_box(*padded_board_bounds(p_board, sheet_pad))
        im = render_polys(
            render_frame,
            [[polys[i] for i in selected_polys]],
            im_shape=render_size,
            nest_outline=p_board,
        )
        video.write(im)
        cv.imwrite(out.snapshot_path, im)

        selected_t = ([], [])
        for i in selected_polys:
            gi = group_id[i]
            selected_t[gi].append(transform[i])
        selected_t = tuple(np.array(t) for t in selected_t)
        _append_selection_window(selection_window, selected_t, gc.graphs_window)
        if len(history[0]) and len(selected_t[0]):
            history = (
                trim_history(history[0], selected_t[0], sc.history_max),
                history[1],
            )
        if len(history[1]) and len(selected_t[1]):
            history = (
                history[0],
                trim_history(history[1], selected_t[1], sc.history_max),
            )
        nest_state = NestState(
            polys=polys,
            group_id=group_id,
            transform=transform,
            selected_indices=list(selected_polys),
        )

    video.release()


def transform_selection(s, n):
    yield transforms_around(s, (0.1, 0.1, 1.5), n)
    yield transforms_around(s, (0.1, 0.1, 0), n)
    yield transforms_around(s, (0, 0, 1.5), n)
    yield transforms_around(s, (0.05, 0.05, 1), n)
    yield transforms_around(s, (0.05, 0.05, 0), n)
    yield transforms_around(s, (0, 0, 1), n)
    yield transforms_around(s, (0.01, 0.01, 0.01), n)
    yield transforms_around(s, (0.01, 0.01, 0), n)
    yield transforms_around(s, (0, 0, 0.01), n)
    yield transforms_around(s, (0.001, 0.001, 0.001), n)
    yield transforms_around(s, (0.001, 0.001, 0), n)
    yield transforms_around(s, (0, 0, 0.001), n)


def transform_history(h, n):
    yield transforms_around(h, (0.05, 0.05, 0.1), n)
    yield transforms_around(h, (0.05, 0.05, 0), n)
    yield transforms_around(h, (0, 0, 0.1), n)


def test_placement():
    p_board = Polygon([(0, 0), (1.2, 0), (0, 1.1)])
    p1 = normalize_poly(Polygon([(0, 0), (.15, 0), (0, .07)]))
    p2 = normalize_poly(Polygon([(0, 0), (.1, 0), (.1, .1), (0, .1)]))

    p1_result = []
    p2_result = []
    base_shape = Polygon()
    for _ in range(100):
        p1_places = propose_placements_point_cloud(
            base_shape, p1, p_board, min_dist=0.001, pt_push=p_board.centroid, top_n=100
        )
        print('p1', len(p1_places))
        if p1_places:
            p1_result.append(p1_places[0])
            base_shape = unary_union([base_shape, transform_poly(p1, p1_places[0])])
        p2_places = propose_placements_point_cloud(
            base_shape, p2, p_board, min_dist=0.001, pt_push=p_board.centroid, top_n=100
        )
        print('p2', len(p2_places))
        if p2_places:
            p2_result.append(p2_places[0])
            base_shape = unary_union([base_shape, transform_poly(p2, p2_places[0])])

        im = render_polys(p_board, [
            [transform_poly(p1, t) for t in p1_result],
            [transform_poly(p2, t) for t in p2_result]
        ])
        cv.imwrite('test.jpg', im)


def main():
    run_build_graph(BuildGraphConfig.from_env())


if __name__ == "__main__":
    main()
