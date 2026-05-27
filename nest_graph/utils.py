from shapely import Polygon
from shapely.geometry import LineString
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
    if p.is_empty:
        return ()
    if p.geom_type in ('MultiPolygon', 'GeometryCollection'):
        return tuple(
            e for g in p.geoms for e in get_shape_exteriors(g)
        )
    if p.geom_type == 'Polygon':
        if p.exterior.is_empty or p.exterior.length <= 0:
            return ()
        return (LineString(p.exterior.coords),)
    return ()


def get_shape_polygons_coords(g):
    """
    get coordinates from non-zero area parts of a shape.
    """
    if g.geom_type.startswith('Multi'):
        return tuple(c for poly in g.geoms for c in get_shape_polygons_coords(poly))
    return tuple(g.exterior.coords) if hasattr(g, 'exterior') else ()
