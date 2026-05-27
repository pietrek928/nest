# CFLAGS="-Wno-error=incompatible-pointer-types" pip install --force-reinstall --no-binary=shapely --upgrade shapely

import cv2 as cv
import numpy as np
from pydantic import BaseModel, ConfigDict
from shapely import Polygon, unary_union
from shapely.geometry import Point
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
    trim_history,
)
from .geometry import Geometry, GuidanceConfig, find_polygon_intersections_bipartite
from .placement_scene import (
    PlacementScene,
    guidance_config_for_scene,
    is_valid_placement,
)
from .board import board_context_from_geometry
from .utils import normalize_poly, transform_poly
from .propose import (
    propose_placements_point_cloud,
    proposed_transforms_for_groups,
)
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
    guidance_cfg = guidance_config_for_scene(
        min_dist, board_bounds=bounds, epsilon_ratio=epsilon_ratio
    )
    bases = _base_geometries(polygons)
    for i, item in enumerate(polygons):
        _p, transforms, w = _poly_and_transforms(item)
        base = bases[i]
        for t in transforms:
            placed = base.apply_transform(t)
            scene = PlacementScene(sheet, void_geoms, [], base)
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
    cx, cy = placed.center()
    if not board_geom.contains_point(cx, cy):
        return -board.distance(Point(cx, cy))
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
    guidance_cfg = guidance_config_for_scene(
        min_dist, board_bounds=sheet.bounds, epsilon_ratio=epsilon_ratio
    )
    graph = ElemGraph()
    selected_polys = []
    selected_geoms: list[Geometry] = []
    selected_bboxes: list[tuple[float, float, float, float]] = []
    selected_group_id = []
    selected_transform = []
    bases = _base_geometries(polygons)

    pending: list[tuple] = []
    for i, item in enumerate(polygons):
        if len(item) == 2:
            p, transforms = item
        else:
            p, _w, transforms = item
        base = bases[i]
        for t in transforms:
            placed = base.apply_transform(t)
            scene = PlacementScene(sheet, void_geoms, [], base)
            cx, cy = placed.center()
            if not is_valid_placement(
                scene, placed, (cx, cy), min_dist, guidance_cfg,
                epsilon_ratio=epsilon_ratio,
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


def _draw_board_outline(im, board: BaseGeometry, xstart, ystart, xscale, yscale):
    cv.drawContours(im, [scale_coords(
        np.array(board.exterior.coords), xstart, ystart, xscale, yscale
    )], -1, (255, 255, 255), 3)
    if isinstance(board, Polygon):
        for ring in board.interiors:
            cv.drawContours(im, [scale_coords(
                np.array(ring.coords), xstart, ystart, xscale, yscale
            )], -1, (160, 160, 160), 2)

def render_placement(b: BaseGeometry, elems: Tuple[Tuple[Polygon, np.ndarray], ...], im_shape=(1024, 1024)):
    xstart, ystart, xend, yend = b.bounds
    xscale = im_shape[0] / (xend - xstart)
    yscale = im_shape[1] / (yend - ystart)
    im = np.zeros((im_shape[0], im_shape[1], 3), dtype=np.uint8)
    _draw_board_outline(im, b, xstart, ystart, xscale, yscale)
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


def render_selection(b: BaseGeometry, polys: Tuple[Polygon, ...], v: np.ndarray, im_shape=(1024, 1024)):
    xstart, ystart, xend, yend = b.bounds
    xscale = im_shape[0] / (xend - xstart)
    yscale = im_shape[1] / (yend - ystart)
    im = np.zeros((im_shape[0], im_shape[1], 3), dtype=np.uint8)
    _draw_board_outline(im, b, xstart, ystart, xscale, yscale)
    for w, p in sorted(zip(v, polys), key=lambda x: x[0]):
        cv.drawContours(im, [scale_coords(
            np.array(p.exterior.coords), xstart, ystart, xscale, yscale
        )], -1, (100*w, 100*w, 100*w), cv.FILLED)
        cv.drawContours(im, [scale_coords(
            np.array(p.exterior.coords), xstart, ystart, xscale, yscale
        )], -1, (255*w, 255*w, 255*w), 3)
    return im


def render_polys(b: BaseGeometry, polys: Tuple[Tuple[Polygon, ...], ...], im_shape=(1024, 1024)):
    xstart, ystart, xend, yend = b.bounds
    xscale = im_shape[0] / (xend - xstart)
    yscale = im_shape[1] / (yend - ystart)
    im = np.zeros((im_shape[0], im_shape[1], 3), dtype=np.uint8)
    _draw_board_outline(im, b, xstart, ystart, xscale, yscale)
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


def _build_transform_batch(
    cfg: BuildGraphConfig,
    selected_t: tuple[np.ndarray, np.ndarray],
    history: tuple[np.ndarray, np.ndarray],
    rng: np.random.Generator,
    *,
    board: BaseGeometry | None = None,
    parts: list[tuple[Polygon, int]] | None = None,
    nest_state: NestState | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    sc = cfg.sampling
    scale = sc.transform_scale
    propose_by_group: dict[int, np.ndarray] = {}
    if (
        board is not None
        and parts is not None
        and cfg.propose.max_proposals > 0
    ):
        polys = nest_state.polys if nest_state is not None else []
        selected = nest_state.selected_indices if nest_state is not None else []
        propose_by_group = proposed_transforms_for_groups(
            board,
            parts,
            polys,
            selected,
            cfg.propose,
            min_dist=cfg.board_min_dist(),
        )

    def one_group(
        group_id: int,
        sel: np.ndarray,
        hist: np.ndarray,
    ) -> np.ndarray:
        batch_parts: list[np.ndarray] = []
        proposed = propose_by_group.get(group_id, np.zeros((0, 3)))
        if proposed.shape[0] > 0:
            batch_parts.append(proposed)
            jittered = expand_structured_transforms(
                proposed,
                sc.structured_jitter_scale,
                sc.structured_jitter_per_proposal,
            )
            if jittered.shape[0] > 0:
                batch_parts.append(jittered)
        n_random = (
            sc.random_per_iter_when_proposed
            if proposed.shape[0] > 0
            else sc.random_per_iter
        )
        batch_parts.append(rng.uniform(-1, 1, (n_random, 3)) * scale)
        if hist.shape[0] > 0:
            batch_parts.append(hist)
        if sel.shape[0] > 0:
            batch_parts.append(sel)
            batch_parts.extend(transform_selection(sel, sc.selection_expand_n))
            batch_parts.extend(transform_history(hist, sc.history_expand_n))
        if sc.shuffle_passes > 0 and (sel.shape[0] > 0 or hist.shape[0] > 0):
            for _ in range(sc.shuffle_passes):
                batch_parts.append(
                    transform_shuffle_mix(
                        sel, hist, sc.shuffle_per_pass, rng, sc.shuffle_scale,
                    )
                )
        merged = dedupe_transforms(np.concatenate(batch_parts))
        merged = shuffle_transforms(merged, rng)
        return subsample_transforms(merged, sc.max_transforms_per_group, rng)

    return (
        one_group(0, selected_t[0], history[0]),
        one_group(1, selected_t[1], history[1]),
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
    rule_set.append_rule(PointAngleRule(pos=Vec2(x=0, y=1.1), r=r, a=np.pi / 4, w=aw, group=1))
    rule_set.append_rule(PointAngleRule(pos=Vec2(x=0, y=1.1), r=r, a=np.pi * 5 / 4, w=aw, group=1))
    rule_set.append_rule(PointAngleRule(pos=Vec2(x=1.2, y=0), r=r, a=np.pi / 4, w=aw, group=1))
    rule_set.append_rule(PointAngleRule(pos=Vec2(x=1.2, y=0), r=r, a=np.pi * 5 / 4, w=aw, group=1))
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
    rng = cfg.apply_seed()
    sc = cfg.sampling
    gc = cfg.graph
    sel = cfg.selection
    out = cfg.output

    p_board = cfg.rules.board_polygon()
    p_sheet = cfg.rules.board_sheet_polygon()
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
    nest_state: NestState | None = None
    parts = [(p1, 0), (p2, 1)]
    iters = tuple(range(out.n_iters))
    if out.progress:
        iters = tqdm(iters)

    for _it in iters:
        selected_t = _build_transform_batch(
            cfg,
            selected_t,
            history,
            rng,
            board=p_board,
            parts=parts,
            nest_state=nest_state,
        )
        graph, polys, group_id, transform = make_polygon_graph(
            p_board,
            [(p1, selected_t[0]), (p2, selected_t[1])],
            min_dist=cfg.board_min_dist(),
            epsilon_ratio=cfg.propose.placement_clearance_epsilon_ratio,
        )
        graphs.append(graph)
        graphs = graphs[-gc.graphs_window:]
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
        selected_polys = nest_by_graph(graph, rule_sets[: sel.nest_rule_sets_used])[0]
        old_len = len(selected_polys)
        scores = score_elems(graph, active_rules)
        _, selected_polys, _ = apply_dfs_refinement(
            graph,
            active_rules,
            list(selected_polys),
            scores,
            selection=sel,
        )
        assert selection_is_independent(graph, selected_polys)
        print(len(polys), old_len, ' -> ', len(selected_polys))
        im = render_polys(p_sheet, [[polys[i] for i in selected_polys]], im_shape=render_size)
        video.write(im)
        cv.imwrite(out.snapshot_path, im)

        selected_t = ([], [])
        for i in selected_polys:
            gi = group_id[i]
            selected_t[gi].append(transform[i])
        selected_t = tuple(np.array(t) for t in selected_t)
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
