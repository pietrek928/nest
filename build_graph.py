# CFLAGS="-Wno-error=incompatible-pointer-types" pip install --force-reinstall --no-binary=shapely shapely
# pip install --force-reinstall --no-binary=rtree rtree

from typing import Tuple
import numpy as np
import cv2 as cv
from shapely import Polygon
from shapely.geometry.base import BaseGeometry
from shapely.affinity import translate, rotate


p_board = Polygon([(0, 0), (1.2, 0), (0, 1.1)])
p1 = Polygon([(0, 0), (.1, 0), (.1, .1), (0, .1)])
p2 = Polygon([(0, 0), (.15, 0), (0, .07)])


def transform_poly(p: Polygon, transform_data: Tuple[float, float, float]):
    x, y, angle = transform_data
    return rotate(translate(p, x, y), angle, origin='center')


# TODO: make faster polygon operations, shapely lags
def polygon_board_distance(b: BaseGeometry, p: Polygon):
    if b.contains(p):
        return b.exterior.distance(p)
    if p.intersects(b):
        return 0
    return -b.distance(p)


def score_transforms(b: BaseGeometry, p: Polygon, transforms: np.ndarray):
    scores = np.zeros((transforms.shape[0], ))
    for i, t in enumerate(transforms):
        p_t = transform_poly(p, t)
        scores[i] = polygon_board_distance(b, p_t)
    return scores


# random transforms in range
def transforms_around(p: Tuple[float, float, float], s: Tuple[float, float, float], n: int):
    x, y, a = p
    sx, sy, sa = s
    return np.array([
        [x + np.random.uniform(-sx, sx), y + np.random.uniform(-sy, sy), a + np.random.uniform(-sa, sa)]
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
    return np.stack([x, y], axis=-1)


def show_placement(b: BaseGeometry, elems: Tuple[Tuple[Polygon, np.ndarray], ...], im_shape=(1024, 1024)):
    xstart, ystart, xend, yend = b.bounds
    xscale = im_shape[0] / (xend - xstart)
    yscale = im_shape[1] / (yend - ystart)
    im = np.zeros((im_shape[0], im_shape[1], 3), dtype=np.uint8)
    cv.drawContours(im, [scale_coords(
        np.array(b.exterior.coords), xstart, ystart, xscale, yscale
    )], -1, (255, 255, 255), -1)
    for p, transforms in elems:
        for t in transforms:
            p_t = transform_poly(p, t)
            cv.drawContours(im, [scale_coords(
                np.array(p_t.exterior.coords), xstart, ystart, xscale, yscale
            )], -1, (255, 255, 255), -1)


score_pts = np.random.rand(128000, 3)
scores = score_transforms(p_board, p1, score_pts)
# select best 10
pts_with_scores = np.concatenate([score_pts, scores[:, None]], axis=-1)
pts_with_scores = pts_with_scores[pts_with_scores[:, -1] > 0]
best_pts = pts_with_scores[np.argsort(pts_with_scores[:, -1])[:10]]
print(best_pts)
