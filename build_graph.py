# CFLAGS="-Wno-error=incompatible-pointer-types" pip install --force-reinstall --no-binary=shapely shapely
# pip install --force-reinstall --no-binary=rtree rtree

from typing import Tuple
import numpy as np
import cv2 as cv
from shapely import Polygon
from shapely.geometry.base import BaseGeometry
from shapely.affinity import translate, rotate
from rtree.index import Index
from tqdm import tqdm


def transform_poly(p: Polygon, transform_data: Tuple[float, float, float]):
    x, y, angle = transform_data[:3]
    return rotate(translate(p, x, y), angle, origin='center', use_radians=True)


# TODO: make faster polygon operations, shapely lags
def polygon_board_distance(b: BaseGeometry, p: Polygon):
    if b.contains(p):
        return b.exterior.distance(p) + b.exterior.distance(p.centroid) * .003
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


def select_polygons_from_edges(b: BaseGeometry, polygons: Tuple[Tuple[Polygon, np.ndarray], ...]):
    result = [[] for _ in range(len(polygons))]
    polys_transformed = []
    for i, (p, transforms) in enumerate(polygons):
        for t in transforms:
            poly_t = transform_poly(p, t)
            d = polygon_board_distance(b, poly_t)
            if d > 0:
                d += np.random.rand() * 1e-4
                polys_transformed.append((d, i, t, poly_t))
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


p_board = Polygon([(0, 0), (1.2, 0), (0, 1.1)])
p1 = Polygon([(0, 0), (.1, 0), (.1, .1), (0, .1)])
p2 = Polygon([(0, 0), (.15, 0), (0, .07)])

selected_t = [
    np.random.rand(128, 3) * [1.5, 1.5, 2 * np.pi],
    np.random.rand(128, 3) * [1.5, 1.5, 2 * np.pi],
]

video = cv.VideoWriter('/tmp/test.mp4', cv.VideoWriter_fourcc(*'mp4v'), 5, (1024, 1024))

for _ in tqdm(tuple(range(150))):
    s0 = [
        np.random.rand(1024, 3) * [1.5, 1.5, 2 * np.pi],
    ]
    if selected_t[0].shape[0] > 0:
        s0.append(selected_t[0])
        s0.append(transforms_around(selected_t[0], (0.1, 0.01, 0.1), 100))

    s1 = [
        np.random.rand(1024, 3) * [1.5, 1.5, 2 * np.pi],
    ]
    if selected_t[1].shape[0] > 0:
        s1.append(selected_t[1])
        s1.append(transforms_around(selected_t[1], (0.1, 0.1, 0.1), 100))

    selected_t = [
        np.concatenate(s0),
        np.concatenate(s1),
    ]
    selected_t = select_polygons_from_edges(p_board, [(p1, selected_t[0]), (p2, selected_t[1])])
    video.write(render_placement(p_board, [(p1, selected_t[0]), (p2, selected_t[1])]))

video.release()

# render
# im = render_placement(p_board, [(p1, selected_t[0]), (p2, selected_t[1])])
# cv.imshow('im', im)
# cv.waitKey(0)
