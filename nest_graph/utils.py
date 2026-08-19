import math
from typing import Tuple

from shapely import Polygon
from shapely.affinity import rotate, translate
from shapely.geometry import LineString
from shapely.geometry.base import BaseGeometry


def transform_poly(p: Polygon, transform_data: Tuple[float, float, float]):
    x, y, angle = transform_data[:3]
    return translate(rotate(p, angle, origin=(0, 0), use_radians=True), x, y)


def transform_row_key(t) -> tuple[float, float, float]:
    """Round-4 (x, y, θ) join key. Propose SoT re-export: void_selection.transform_row_key.

    Lives here so config can key subsample/history without importing propose
    (propose/__init__ → pipeline → config).
    """
    return (round(float(t[0]), 4), round(float(t[1]), 4), round(float(t[2]), 4))


def invert_transform(t: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Inverse of rotate-about-origin then translate (matches transform_poly).

    Double-precision Python SoT for propose/motif keys. C++ ``Se2`` stays float
    for MotifBase/arena; float round-trip shifted cluster_copy relatives enough
    to miss void_fill best-so-far (S1 research).
    """
    x, y, a = float(t[0]), float(t[1]), float(t[2])
    c, s = math.cos(a), math.sin(a)
    return (-c * x - s * y, s * x - c * y, -a)


def compose_transforms(
    a: Tuple[float, float, float],
    b: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """Compose SE(2) poses: apply b first, then a (a ∘ b)."""
    ax, ay, aa = float(a[0]), float(a[1]), float(a[2])
    bx, by, ba = float(b[0]), float(b[1]), float(b[2])
    c, s = math.cos(aa), math.sin(aa)
    return (ax + c * bx - s * by, ay + s * bx + c * by, aa + ba)


def relative_transform(
    ref: Tuple[float, float, float],
    t: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """Pose of t in the frame of ref: invert(ref) ∘ t."""
    return compose_transforms(invert_transform(ref), t)


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
