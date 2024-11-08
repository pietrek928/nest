from shapely import distance
from shapely.geometry import Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.affinity import translate, rotate
import numpy as np


def find_approx_poly(A, y):
    c = np.mean(A, axis=0)
    A = A - c

    pt_vals = []
    for p in range(A.shape[1]):
        for pp in range(p+1):
            pt_vals.append(A[:, p] * A[:, pp])
    for p in range(A.shape[1]):
        pt_vals.append(A[:, p])
    pt_vals.append(np.ones_like(pt_vals[-1]))

    pt_vals = np.stack(pt_vals, axis=-1)
    return np.linalg.lstsq(pt_vals, y)[0], c


def compute_poly(A, p, c):
    A -= c
    it = 0
    r = np.ones_like(A[:, 0]) * p[-1]
    # r = np.zeros_like(A[:, 0])
    for i in range(A.shape[1]):
        for j in range(i+1):
            r += A[:, i] * A[:, j] * p[it]
            it += 1
    for i in range(A.shape[1]):
        r += A[:, i] * p[it]
        it += 1
        return r


def check_distance(p1: BaseGeometry, p2: BaseGeometry):
    d = p1.distance(p2)
    if d > 0:
        return d
    return p1.intersection(p2).area / np.sqrt(p1.area + p2.area)


def test():
    p1 = Polygon([(0, 0), (1, 0), (0, 1)])
    p2 = Polygon([(.7, .7), (1.1, .9), (1, 1)])

    A = []
    V = []
    for x in range(10):
        for y in range(10):
            for a in range(36):
                aa = 2 * np.pi * a / 36
                v = check_distance(translate(rotate(p1, aa, use_radians=True), xoff=x*.1, yoff=y*.1), p2)
                A.append((x*.1, y*.1, aa))
                V.append(v)
                # print(v, end=' ')
        # print()

    A = np.array(A, dtype=np.float32)
    V = np.array(V, dtype=np.float32)
    p, c = find_approx_poly(A, V)
    print(p, c)

    for it, v in enumerate(compute_poly(A, p, c)):
        if it % 10 == 0:
            print()
        print(v, end=' ')
        if it >= 100:
            break


test()
