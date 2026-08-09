"""Shapely adapters for Geometry (boolean edges / tests)."""

from shapely import MultiPolygon, Polygon
from shapely.geometry.base import BaseGeometry

from nest_graph.geometry import Geometry

# Cache Shapely contours by Geometry object id. Transform returns a new Geometry,
# so entries stay valid for the lifetime of an immutable placed pose.
_shapely_cache: dict[int, BaseGeometry] = {}


def geometry_to_shapely(geom: Geometry) -> BaseGeometry:
    """Build a Shapely polygon from Geometry boundary rings (cached per id)."""
    key = id(geom)
    cached = _shapely_cache.get(key)
    if cached is not None:
        return cached
    rings = list(geom.boundary_rings())
    if not rings:
        coords = [(float(x), float(y)) for x, y in geom.vertices()]
        if len(coords) >= 2 and coords[0] == coords[-1]:
            coords = coords[:-1]
        poly = Polygon(coords) if len(coords) >= 3 else Polygon()
        _shapely_cache[key] = poly
        return poly
    outers: list[list[tuple[float, float]]] = []
    holes: list[list[tuple[float, float]]] = []
    for coords_list, subtractive in rings:
        pts = [(float(x), float(y)) for x, y in coords_list]
        if len(pts) >= 2 and pts[0] == pts[-1]:
            pts = pts[:-1]
        if len(pts) < 3:
            continue
        if subtractive:
            holes.append(pts)
        else:
            outers.append(pts)
    if not outers:
        poly = Polygon()
    elif len(outers) == 1:
        poly = Polygon(shell=outers[0], holes=holes or None)
    else:
        # Assign all holes to first outer; rare multi-outer solids.
        polys = [Polygon(shell=outers[0], holes=holes or None)]
        polys.extend(Polygon(shell=o) for o in outers[1:])
        poly = MultiPolygon(polys)
    _shapely_cache[key] = poly
    return poly


def geoms_to_shapely_union(geoms: list[Geometry]) -> BaseGeometry:
    from shapely.ops import unary_union

    if not geoms:
        return Polygon()
    parts = [geometry_to_shapely(g) for g in geoms]
    if len(parts) == 1:
        return parts[0]
    return unary_union(parts)


def clear_shapely_cache() -> None:
    _shapely_cache.clear()
