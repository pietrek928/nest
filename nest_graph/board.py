"""Board sheet geometry: bbox outer + void holes (forbidden regions)."""

from shapely.geometry import Polygon, box
from shapely.geometry.base import BaseGeometry

from .geometry import Geometry


def _hole_rings_from_difference(void_geom: BaseGeometry) -> list[list[tuple[float, float]]]:
    rings: list[list[tuple[float, float]]] = []
    if void_geom.is_empty:
        return rings
    geoms = void_geom.geoms if hasattr(void_geom, "geoms") else [void_geom]
    for g in geoms:
        if g.is_empty:
            continue
        if g.geom_type == "Polygon":
            rings.append(list(g.exterior.coords))
        elif g.geom_type == "MultiPolygon":
            for p in g.geoms:
                rings.append(list(p.exterior.coords))
    return rings


def board_sheet_from_outline(
    outline: Polygon,
    *,
    padding: float = 0.0,
    user_holes: tuple[tuple[tuple[float, float], ...], ...] = (),
) -> Polygon:
    if outline.is_empty:
        raise ValueError("board outline is empty")
    minx, miny, maxx, maxy = outline.bounds
    outer = box(minx - padding, miny - padding, maxx + padding, maxy + padding)
    void = outer.difference(outline)
    auto_holes = _hole_rings_from_difference(void)
    user_hole_rings = [list(h) for h in user_holes]
    holes = auto_holes + user_hole_rings
    if holes:
        return Polygon(list(outer.exterior.coords), holes=holes)
    return outer


def board_void_geometries(sheet: Polygon) -> list[Geometry]:
    """Forbidden void regions as Geometry solids (overlap => invalid)."""
    outline = Polygon(sheet.exterior.coords)
    void = outline.difference(sheet)
    if void.is_empty:
        return []
    geoms: list[Geometry] = []
    parts = void.geoms if hasattr(void, "geoms") else [void]
    for g in parts:
        if g.is_empty or g.geom_type != "Polygon":
            continue
        geoms.append(Geometry.from_shapely(g))
    return geoms


def board_context_from_geometry(
    board: BaseGeometry,
    *,
    padding: float = 0.0,
    user_holes: tuple[tuple[tuple[float, float], ...], ...] = (),
) -> tuple[Polygon, list[Geometry]]:
    if not isinstance(board, Polygon):
        board = Polygon(board)  # type: ignore[arg-type]
    sheet = board_sheet_from_outline(board, padding=padding, user_holes=user_holes)
    return sheet, board_void_geometries(sheet)
