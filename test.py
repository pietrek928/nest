import numpy as np
from vec_test import transform_points_g3, points_line_string_distance_g3


def render_boundary(im, coords, color, size=(1.0, 1.0), thickness=1):
    import cv2 as cv
    coords = np.rint(
        coords * (np.array(im.shape[:2][::-1]) / size)
    ).astype(np.int32).reshape((-1, 1, 2))
    cv.polylines(im, [coords], isClosed=True, color=color, thickness=thickness)


def test():
    import cv2 as cv
    box = np.array([
        [1, 1],
        [9, 1],
        [9, 9],
        [1, 9]
    ], dtype=np.float64)
    p1 = np.array([
        [2, 2],
        [3, 4],
        [5, 7]
    ], dtype=np.float64)
    p2 = np.array([
        [3, 2],
        [4, 4],
        [7, 8]
    ], dtype=np.float64)
    im = np.zeros((256, 256, 3), dtype=np.uint8)
    render_boundary(im, p1, (0, 255, 0), size=(10, 10))
    render_boundary(im, p2, (0, 0, 255), size=(10, 10))
    render_boundary(im, box, (255, 0, 0), size=(10, 10))
    cv.imshow('test', im)
    cv.waitKey(0)
    # pts1 = transform_points_g3(pts, -3, -2, 2)
    # pts2 = transform_points_g3(pts, 3, 2, 1)
    # print(pts1)
    # g = points_line_string_distance_g3(pts1, pts2)
    # print(g.shape, g)


if __name__ == '__main__':
    test()
