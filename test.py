import numpy as np
from vec_test import transform_points_g3, points_line_string_distance_g3

def test():
    pts = np.array([
        [1, 2],
        [3, 4],
        [5, 6]
    ], dtype=np.float64)
    pts1 = transform_points_g3(pts, -3, -2, 2)
    pts2 = transform_points_g3(pts, 3, 2, 1)
    print(pts1)
    g = points_line_string_distance_g3(pts1, pts2)
    print(g.shape, g)

test()
