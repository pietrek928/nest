import numpy as np
from vec_test import transform_points

def test():
    pts = np.array([
        [1, 2],
        [3, 4]
    ], dtype=np.float64)
    pts = transform_points(pts, 1, 2, 3)
    print(pts)

test()