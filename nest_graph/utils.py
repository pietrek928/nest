from shapely import Polygon
from shapely.geometry.base import BaseGeometry
from shapely.affinity import translate, rotate
from typing import Tuple


def transform_poly(p: Polygon, transform_data: Tuple[float, float, float]):
    x, y, angle = transform_data[:3]
    return translate(rotate(p, angle, origin=(0, 0), use_radians=True), x, y)


def normalize_poly(p: BaseGeometry):
    c = p.centroid
    return translate(p, -c.x, -c.y)


def get_shape_exteriors(p: BaseGeometry):
    if hasattr(p, 'geoms'):
        e = []
        for g in p.geoms:
            e.extend(get_shape_exteriors(g))
        return tuple(e)
    if hasattr(p, 'exterior'):
        return (p.exterior,)
    return (p,)
