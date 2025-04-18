# CFLAGS="-Wno-error=incompatible-pointer-types" pip install --force-reinstall --no-binary=shapely shapely
# pip install --force-reinstall --no-binary=rtree rtree

import numpy as np
import cv2 as cv
from pydantic import BaseModel
from shapely import Polygon
from shapely.geometry.base import BaseGeometry
from shapely.affinity import translate, rotate
from rtree.index import Index
from tqdm import tqdm
from typing import Tuple

from .elem_graph import (
    ElemGraph, BBox, Point,
    PointPlaceRule, BBoxPlaceRule, PointAngleRule, PlacementRuleSet,
    RuleMutationSettings,
    nest_by_graph, sort_graph, score_elems, augment_rules, score_rules,
    increase_selection_dfs, increase_score_dfs
)


class PolygonGroup(BaseModel):
    class Config:
        arbitrary_types_allowed = True

    polygon: Polygon
    weight: float
    transforms: np.ndarray


def transform_poly(p: Polygon, transform_data: Tuple[float, float, float]):
    x, y, angle = transform_data[:3]
    return rotate(translate(p, x, y), angle, origin='center', use_radians=True)


# TODO: make faster polygon operations, shapely lags
def polygon_board_distance(b: BaseGeometry, p: Polygon):
    if b.contains(p):
        return b.exterior.distance(p) + b.exterior.distance(p.centroid)
    if p.intersects(b):
        return 0
    return -b.distance(p)


def select_non_intersecting_polygons(polygons: np.ndarray):
    idx = Index()
    selected = []
    for p in polygons:
        intersects = False
        for i in idx.intersection(p.bounds):
            if p.intersects(selected[i]):
                intersects = True
                break
        if not intersects:
            idx.insert(len(selected), p.bounds)
            selected.append(p)
    return selected


def select_polygons_from_edges(b: BaseGeometry, polygons: Tuple[Tuple[Polygon, float, np.ndarray], ...]):
    result = [[] for _ in range(len(polygons))]
    polys_transformed = []
    for i, (p, w, transforms) in enumerate(polygons):
        for t in transforms:
            poly_t = transform_poly(p, t)
            d = polygon_board_distance(b, poly_t)
            if d > 0:
                d += np.random.rand() * 1e-4
                polys_transformed.append((poly_t.centroid.coords[0][0] + w, i, t, poly_t))
    polys_transformed = sorted(polys_transformed, key=lambda x: x[0])

    idx = Index()
    selected = []
    for _, pnum, t, poly_t in polys_transformed:
        intersects = False
        for i in idx.intersection(poly_t.bounds):
            if poly_t.intersects(selected[i]):
                intersects = True
                break
        if not intersects:
            idx.insert(len(selected), poly_t.bounds)
            selected.append(poly_t)
            result[pnum].append(t)
    return tuple(
        np.array(tt) for tt in result
    )


def make_polygon_matrix(b: BaseGeometry, polygons: Tuple[Tuple[Polygon, float, np.ndarray], ...]):
    idx = Index()
    selected = []
    group_weights = []

    for i, (p, w, transforms) in enumerate(polygons):
        n = len(transforms)
        for t in transforms:
            poly_t = transform_poly(p, t)
            if b.contains(poly_t):
                selected.append(poly_t)
                # group_weights.append(w / n)
                group_weights.append(w)

    M = np.zeros((len(selected), len(selected)), dtype=np.float32)
    for i, poly in enumerate(tqdm(selected)):
        for j in idx.intersection(poly.bounds):
            if poly.intersects(selected[j]):
                M[i, j] = M[j, i] = 1
        idx.insert(i, poly.bounds)
        M[i, i] = -group_weights[i]

    return M, selected


def make_polygon_graph(b: BaseGeometry, polygons: Tuple[Tuple[Polygon, float, np.ndarray], ...]):
    idx = Index()
    selected_polys = []
    selected_group_id = []
    selected_transform = []
    graph = ElemGraph()

    for i, (p, transforms) in enumerate(polygons):
        # n = len(transforms)
        for t in transforms:
            poly_t = transform_poly(p, t)
            if b.contains(poly_t):
                # group_weights.append(w / n)
                center = poly_t.centroid.coords[0]
                bbox = poly_t.bounds
                graph.append_elem(
                    i,
                    Point(x=center[0], y=center[1]),
                    BBox(xstart=bbox[0], ystart=bbox[1], xend=bbox[2], yend=bbox[3])
                )

                n = len(selected_polys)
                for j in idx.intersection(poly_t.bounds):
                    if poly_t.intersects(selected_polys[j]):
                        graph.add_collision(n, j)
                selected_polys.append(poly_t)
                selected_group_id.append(i)
                selected_transform.append(t)
                idx.insert(n, poly_t.bounds)

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


def render_placement(b: BaseGeometry, elems: Tuple[Tuple[Polygon, np.ndarray], ...], im_shape=(1024, 1024)):
    xstart, ystart, xend, yend = b.bounds
    xscale = im_shape[0] / (xend - xstart)
    yscale = im_shape[1] / (yend - ystart)
    im = np.zeros((im_shape[0], im_shape[1], 3), dtype=np.uint8)
    cv.drawContours(im, [scale_coords(
        np.array(b.exterior.coords), xstart, ystart, xscale, yscale
    )], -1, (255, 255, 255), 3)
    for p, transforms in elems:
        for t in transforms:
            p_t = transform_poly(p, t)
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
    for poly_set in polys:
        for p in poly_set:
            cv.drawContours(im, [scale_coords(
                np.array(p.exterior.coords), xstart, ystart, xscale, yscale
            )], -1, (255, 255, 255), 3)
    return im


def improve_rules(graphs, rules, n):
    new_rules = list(rules)
    new_rules.extend(augment_rules(rules, RuleMutationSettings(
        box=BBox(xstart=0, ystart=0, xend=1.2, yend=1.1),
        dpos=.25,
        dw=.25,
        da=np.pi/4,
        insert_p=0.09,
        remove_p=0.02,
        mutate_p=0.1,
        ngroups=2
    )))
    new_rules.extend(augment_rules(rules, RuleMutationSettings(
        box=BBox(xstart=0, ystart=0, xend=1.2, yend=1.1),
        dpos=.05,
        dw=.05,
        da=np.pi/32,
        insert_p=0.04,
        remove_p=0.01,
        mutate_p=0.1,
        ngroups=2
    )))
    new_rules.extend(augment_rules(rules, RuleMutationSettings(
        box=BBox(xstart=0, ystart=0, xend=1.2, yend=1.1),
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
    yield transforms_around(s, (0.15, 0.15, 2), n)
    yield transforms_around(s, (0.15, 0.15, 0), n)
    yield transforms_around(s, (0, 0, 2), n)
    yield transforms_around(s, (0.01, 0.01, 0.01), n)
    yield transforms_around(s, (0.01, 0.01, 0), n)
    yield transforms_around(s, (0, 0, 0.01), n)
    yield transforms_around(s, (0.001, 0.001, 0.001), n)
    yield transforms_around(s, (0.001, 0.001, 0), n)
    yield transforms_around(s, (0, 0, 0.001), n)


def transform_history(h, n):
    yield transforms_around(h, (0.1, 0.1, 0.2), n)
    yield transforms_around(h, (0.1, 0.1, 0), n)
    yield transforms_around(h, (0, 0, 0.2), n)


graphs = []
first_rule_set = PlacementRuleSet()
first_rule_set.append_rule(PointPlaceRule(
    x=0, y=0, r=.1, w=.1, group=0
))
first_rule_set.append_rule(PointPlaceRule(
    x=0, y=0, r=.1, w=.1, group=1
))
rule_sets = [first_rule_set]

p_board = Polygon([(0, 0), (1.2, 0), (0, 1.1)])
p1 = Polygon([(0, 0), (.1, 0), (.1, .1), (0, .1)])
p2 = Polygon([(0, 0), (.15, 0), (0, .07)])

selected_t = [
    np.random.rand(128, 3) * [1.5, 1.5, 2 * np.pi],
    np.random.rand(128, 3) * [1.5, 1.5, 2 * np.pi],
]

wrect = 1
wtriang = 1
r = .2
rule_set = PlacementRuleSet()
rule_set.append_rule(PointPlaceRule(
    x=0, y=0, r=r, w=wrect, group=0
))
# rule_set.append_point_rule(PointPlaceRule(
#     x=0, y=0, r=r, w=wtriang, group=1
# ))
# rule_set.append_rule(PointPlaceRule(
#     x=1.2, y=0, r=r, w=wrect, group=0
# ))
# rule_set.append_rule(PointPlaceRule(
#     x=1.2, y=0, r=r, w=wtriang, group=1
# ))
# rule_set.append_rule(PointPlaceRule(
#     x=0, y=1.1, r=r, w=wrect, group=0
# ))
# rule_set.append_rule(PointPlaceRule(
#     x=0, y=1.1, r=r, w=wtriang, group=1
# ))
# rule_set.append_rule(PointPlaceRule(
#     x=0.7, y=0.7, r=r, w=wrect, group=0
# ))
rule_set.append_rule(PointPlaceRule(
    x=0.7, y=0.7, r=r, w=wtriang, group=1
))
rule_set.append_rule(PointPlaceRule(
    x=0, y=1.1, r=r, w=wtriang, group=1
))
rule_set.append_rule(PointPlaceRule(
    x=1.2, y=0, r=r, w=wtriang, group=1
))
# rule_set.append_rule(PointAngleRule(
#     x=0.7, y=0.7, r=r, a=np.pi/4, w=.1*wtriang, group=1
# ))
# rule_set.append_rule(PointAngleRule(
#     x=0.7, y=0.7, r=r, a=np.pi*5/4, w=.1*wtriang, group=1
# ))
rule_set.append_rule(PointAngleRule(
    x=0, y=1.1, r=r, a=np.pi/4, w=.1*wtriang, group=1
))
rule_set.append_rule(PointAngleRule(
    x=0, y=1.1, r=r, a=np.pi*5/4, w=.1*wtriang, group=1
))
rule_set.append_rule(PointAngleRule(
    x=1.2, y=0, r=r, a=np.pi/4, w=.1*wtriang, group=1
))
rule_set.append_rule(PointAngleRule(
    x=1.2, y=0, r=r, a=np.pi*5/4, w=.1*wtriang, group=1
))
video = cv.VideoWriter('test.mp4', cv.VideoWriter_fourcc(*'mp4v'), 5, (1024, 1024))

history = [np.zeros((1, 3)), np.zeros((1, 3))]

for it in tqdm(tuple(range(256))):
    s0 = [
        np.random.rand(256, 3) * [1.5, 1.5, 2 * np.pi],
        history[0]
    ]
    if selected_t[0].shape[0] > 0:
        s0.append(selected_t[0])
        s0.extend(transform_selection(selected_t[0], 7))
        s0.extend(transform_history(history[0], 2))

    s1 = [
        np.random.rand(256, 3) * [1.5, 1.5, 2 * np.pi],
        history[1]
    ]
    if selected_t[1].shape[0] > 0:
        s1.append(selected_t[1])
        s1.extend(transform_selection(selected_t[1], 7))
        s1.extend(transform_history(history[1], 2))

    selected_t = [
        np.concatenate(s0),
        np.concatenate(s1),
    ]
    graph, polys, group_id, transform = make_polygon_graph(p_board, [(p1, selected_t[0]), (p2, selected_t[1])])
    graphs.append(graph)
    graphs = graphs[-12:]
    for _ in range(16):
        rule_sets = improve_rules(graphs, rule_sets, 32)
    # M, polys = make_polygon_matrix(p_board, [(p1, 20, selected_t[0]), (p2, 12, selected_t[1])])
    # print('------', M.shape, M.sum()/M.shape[0])
    # v = optimize_polygons(M, np.zeros((M.shape[0], )))
    selected_polys = nest_by_graph(graph, rule_sets[:1])[0]
    old_len = len(selected_polys)
    graph_sorted = sort_graph(graph, rule_set)
    graph_sorted_rev = sort_graph(graph, rule_set, reverse=True)
    scores = score_elems(graph, rule_set)
    for _ in range(3):
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

# selected_t = select_polygons_from_edges(p_board, [(p1, selected_t[0]), (p2, selected_t[1])])
# video.write(render_placement(p_board, [(p1, selected_t[0]), (p2, selected_t[1])]))
# video.write(render_selection(p_board, polys, v))
# cv.imwrite(f'/tmp/test_{it}.jpg', render_selection(p_board, polys, v))

video.release()

# render
# im = render_placement(p_board, [(p1, selected_t[0]), (p2, selected_t[1])])
# cv.waitKey(0)
