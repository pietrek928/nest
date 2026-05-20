# CFLAGS="-Wno-error=incompatible-pointer-types" pip install --force-reinstall --no-binary=shapely --upgrade shapely

import cv2 as cv
import numpy as np
from pydantic import BaseModel, ConfigDict
from shapely import Polygon, unary_union
from shapely.geometry import Point
from shapely.geometry.base import BaseGeometry
from tqdm import tqdm
from typing import Iterator, NamedTuple, Tuple

from .geometry import Geometry, find_polygon_intersections_bipartite
from .utils import normalize_poly, transform_poly
from .propose import propose_placements_point_cloud
from .track_perf import show_performance
from .elem_graph import (
    _elem_graph,
    ElemGraph, Circle, Vec2,
    PointPlaceRule, PointAngleRule, PlacementRuleSet,
    RuleMutationSettings,
    nest_by_graph, sort_graph, score_elems, augment_rules, score_rules,
    increase_selection_dfs, increase_score_dfs
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
    board_geom: Geometry,
    polygons,
    *,
    strict_board: bool = False,
) -> Iterator[Candidate]:
    bases = _base_geometries(polygons)
    for i, item in enumerate(polygons):
        _p, transforms, w = _poly_and_transforms(item)
        base = bases[i]
        for t in transforms:
            placed = base.apply_transform(t)
            if strict_board:
                if not _placement_inside_board(board_geom, placed):
                    continue
            elif not board_geom.contains_point(*placed.center()):
                continue
            yield Candidate(i, w, t, placed)


def _placement_inside_board(board_geom: Geometry, placed: Geometry) -> bool:
    for x, y in placed.vertices():
        if not board_geom.contains_point(x, y):
            return False
    return True


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


def make_polygon_matrix(b: BaseGeometry, polygons: Tuple[Tuple[Polygon, float, np.ndarray], ...]):
    board_geom = Geometry.from_shapely(b)
    selected_geoms: list[Geometry] = []
    group_weights = []

    for cand in _iter_candidates(board_geom, polygons, strict_board=True):
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
def make_polygon_graph(b: BaseGeometry, polygons):
    board_geom = Geometry.from_shapely(b)
    selected_polys = []
    selected_geoms: list[Geometry] = []
    selected_bboxes: list[tuple[float, float, float, float]] = []
    selected_group_id = []
    selected_transform = []
    group_ids: list[int] = []
    elems: list = []
    coords: list = []
    collisions: list[list[int]] = []
    bases = _base_geometries(polygons)

    for i, item in enumerate(polygons):
        if len(item) == 2:
            p, transforms = item
        else:
            p, _w, transforms = item
        base = bases[i]
        for t in transforms:
            placed = base.apply_transform(t)
            if not _placement_inside_board(board_geom, placed):
                continue

            bbox = placed.bounds()
            n = len(selected_geoms)
            collisions.append([])

            cx, cy = placed.center()
            r = placed.radius()
            ep = _elem_graph.ElemPlace()
            ep.pos = _elem_graph.Vec2(cx, cy)
            ep.a = 0.0
            group_ids.append(i)
            elems.append(ep)
            coords.append(_elem_graph.Circle(_elem_graph.Vec2(cx, cy), r * r))

            for j in _collision_partners_vs_existing(
                placed, bbox, selected_geoms, selected_bboxes
            ):
                collisions[n].append(j)
                collisions[j].append(n)

            selected_polys.append(transform_poly(p, t))
            selected_geoms.append(placed)
            selected_bboxes.append(bbox)
            selected_group_id.append(i)
            selected_transform.append(t)

    graph = ElemGraph()
    graph._obj.group_id = group_ids
    graph._obj.elems = elems
    graph._obj.coords = coords
    graph._obj.collisions = collisions

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

def render_placement(b: BaseGeometry, elems: Tuple[Tuple[Polygon, np.ndarray], ...], im_shape=(1024, 1024)):
    xstart, ystart, xend, yend = b.bounds
    xscale = im_shape[0] / (xend - xstart)
    yscale = im_shape[1] / (yend - ystart)
    im = np.zeros((im_shape[0], im_shape[1], 3), dtype=np.uint8)
    cv.drawContours(im, [scale_coords(
        np.array(b.exterior.coords), xstart, ystart, xscale, yscale
    )], -1, (255, 255, 255), 3)
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
    cv.drawContours(im, [scale_coords(
        np.array(b.exterior.coords), xstart, ystart, xscale, yscale
    )], -1, (255, 255, 255), 3)
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
    cv.drawContours(im, [scale_coords(
        np.array(b.exterior.coords), xstart, ystart, xscale, yscale
    )], -1, (255, 255, 255), 3)
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


# @show_performance
def improve_rules(graphs, rules, n, board: BaseGeometry | None = None):
    region = _rule_region(board) if board is not None else Circle.from_bounds(0, 0, 1.2, 1.1)
    new_rules = list(rules)
    new_rules.extend(augment_rules(rules, RuleMutationSettings(
        region=region,
        dpos=.25,
        dw=.25,
        da=np.pi/4,
        insert_p=0.09,
        remove_p=0.02,
        mutate_p=0.1,
        ngroups=2
    )))
    new_rules.extend(augment_rules(rules, RuleMutationSettings(
        region=region,
        dpos=.05,
        dw=.05,
        da=np.pi/32,
        insert_p=0.04,
        remove_p=0.01,
        mutate_p=0.1,
        ngroups=2
    )))
    new_rules.extend(augment_rules(rules, RuleMutationSettings(
        region=region,
        dpos=.01,
        dw=.01,
        da=np.pi/64,
        insert_p=0.01,
        remove_p=0.001,
        mutate_p=0.1,
        ngroups=2
    )))
    scores = score_rules(graphs, new_rules)
    scored = []
    for s, r in zip(scores, new_rules):
        scored.append((s - r.size() * .01, r))
    scored = sorted(scored, key=lambda x: x[0], reverse=True)
    return [
        v[1] for v in scored[:n]
    ]


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
    graphs = []
    first_rule_set = PlacementRuleSet()
    first_rule_set.append_rule(PointPlaceRule(
        pos=Vec2(x=0, y=0), r=.1, w=.1, group=0
    ))
    first_rule_set.append_rule(PointPlaceRule(
        pos=Vec2(x=0, y=0), r=.1, w=.1, group=1
    ))
    rule_sets = [first_rule_set]

    p_board = Polygon([(0, 0), (1.2, 0), (0, 1.1)])
    p1 = normalize_poly(Polygon([(0, 0), (.1, 0), (.1, .1), (0, .1)]))
    p2 = normalize_poly(Polygon([(0, 0), (.15, 0), (0, .07)]))

    selected_t = [
        np.random.rand(128, 3) * [1.5, 1.5, 2 * np.pi],
        np.random.rand(128, 3) * [1.5, 1.5, 2 * np.pi],
    ]

    wrect = 1
    wtriang = 1
    r = .2
    rule_set = PlacementRuleSet()
    rule_set.append_rule(PointPlaceRule(
        pos=Vec2(x=0, y=0), r=r, w=wrect, group=0
    ))
    rule_set.append_rule(PointPlaceRule(
        pos=Vec2(x=0.7, y=0.7), r=r, w=wtriang, group=1
    ))
    rule_set.append_rule(PointPlaceRule(
        pos=Vec2(x=0, y=1.1), r=r, w=wtriang, group=1
    ))
    rule_set.append_rule(PointPlaceRule(
        pos=Vec2(x=1.2, y=0), r=r, w=wtriang, group=1
    ))
    rule_set.append_rule(PointAngleRule(
        pos=Vec2(x=0, y=1.1), r=r, a=np.pi/4, w=.1*wtriang, group=1
    ))
    rule_set.append_rule(PointAngleRule(
        pos=Vec2(x=0, y=1.1), r=r, a=np.pi*5/4, w=.1*wtriang, group=1
    ))
    rule_set.append_rule(PointAngleRule(
        pos=Vec2(x=1.2, y=0), r=r, a=np.pi/4, w=.1*wtriang, group=1
    ))
    rule_set.append_rule(PointAngleRule(
        pos=Vec2(x=1.2, y=0), r=r, a=np.pi*5/4, w=.1*wtriang, group=1
    ))
    video = cv.VideoWriter('test.mp4', cv.VideoWriter_fourcc(*'mp4v'), 5, (1024, 1024))

    history = [np.zeros((1, 3)), np.zeros((1, 3))]

    n_iters = int(__import__('os').environ.get('NEST_BUILD_GRAPH_ITERS', '256'))
    for it in tqdm(tuple(range(n_iters))):
        s0 = [
            np.random.rand(256, 3) * [1.5, 1.5, 2 * np.pi],
            history[0]
        ]
        if selected_t[0].shape[0] > 0:
            s0.append(selected_t[0])
            s0.extend(transform_selection(selected_t[0], 4))
            s0.extend(transform_history(history[0], 2))

        s1 = [
            np.random.rand(256, 3) * [1.5, 1.5, 2 * np.pi],
            history[1]
        ]
        if selected_t[1].shape[0] > 0:
            s1.append(selected_t[1])
            s1.extend(transform_selection(selected_t[1], 4))
            s1.extend(transform_history(history[1], 2))

        selected_t = [
            np.concatenate(s0),
            np.concatenate(s1),
        ]
        graph, polys, group_id, transform = make_polygon_graph(p_board, [(p1, selected_t[0]), (p2, selected_t[1])])
        graphs.append(graph)
        graphs = graphs[-12:]
        for _ in range(8):
            rule_sets = improve_rules(graphs, rule_sets, 64, p_board)
        selected_polys = nest_by_graph(graph, rule_sets[:1])[0]
        old_len = len(selected_polys)
        graph_sorted = sort_graph(graph, rule_set)
        graph_sorted_rev = sort_graph(graph, rule_set, reverse=True)
        scores = score_elems(graph, rule_set)
        for _ in range(2):
            selected_polys = increase_selection_dfs(graph_sorted_rev, selected_polys, 8, 2)
            selected_polys = increase_selection_dfs(graph, selected_polys, 8, 1)
            selected_polys = increase_score_dfs(graph_sorted_rev, selected_polys, scores)
            selected_polys = increase_selection_dfs(graph_sorted, selected_polys, 8, 2)
            selected_polys = increase_score_dfs(graph_sorted, selected_polys, scores)
        print(len(polys), old_len, ' -> ', len(selected_polys))
        im = render_polys(p_board, [[
            polys[i] for i in selected_polys
        ]])
        video.write(im)
        cv.imwrite('test.jpg', im)

        selected_t = [[], []]
        for i in selected_polys:
            selected_t[group_id[i]].append(transform[i])
        selected_t = tuple(np.array(t) for t in selected_t)
        if len(history[0]) and len(selected_t[0]):
            history[0] = np.unique(np.concatenate([selected_t[0], history[0]]), axis=0)[5000:, :]
        if len(history[1]) and len(selected_t[1]):
            history[1] = np.unique(np.concatenate([selected_t[1], history[1]]), axis=0)[5000:, :]

    video.release()


if __name__ == "__main__":
    main()
