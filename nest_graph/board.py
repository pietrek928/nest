"""Board sheet geometry: nest outline + padded outer void obstacles."""

import math

from shapely.geometry import Polygon, box
from shapely.geometry.base import BaseGeometry
from shapely.ops import polygonize

from .geometry import Geometry


def default_sheet_padding(
    outline: BaseGeometry,
    *,
    extra: float = 0.0,
    ratio: float = 0.08,
) -> float:
    """Margin beyond outline bbox for outer void obstacles (guidance stability)."""
    minx, miny, maxx, maxy = outline.bounds
    diag = math.hypot(maxx - minx, maxy - miny)
    return max(float(extra), float(ratio) * diag)


def padded_board_bounds(
    outline: BaseGeometry,
    padding: float,
) -> tuple[float, float, float, float]:
    minx, miny, maxx, maxy = outline.bounds
    return (minx - padding, miny - padding, maxx + padding, maxy + padding)


def _void_obstacle_parts(void: BaseGeometry, outline: Polygon) -> list[Polygon]:
    if void.is_empty:
        return []
    if void.geom_type == "MultiPolygon":
        candidates = list(void.geoms)
    elif void.geom_type == "Polygon":
        if void.interiors:
            candidates = [p for p in polygonize(void.boundary) if not p.is_empty]
        else:
            candidates = [void]
    else:
        return []
    parts: list[Polygon] = []
    for piece in candidates:
        if piece.is_empty:
            continue
        if outline.contains(piece.representative_point()):
            continue
        parts.append(piece)
    return parts


def board_void_obstacles(outline: Polygon, padding: float) -> list[Geometry]:
    """Corner voids outside the nest outline but inside the padded outer box."""
    if padding <= 0.0:
        return []
    minx, miny, maxx, maxy = outline.bounds
    outer = box(minx - padding, miny - padding, maxx + padding, maxy + padding)
    void = outer.difference(outline)
    return [
        Geometry.from_shapely(g)
        for g in _void_obstacle_parts(void, outline)
    ]


def board_sheet_from_outline(
    outline: Polygon,
    *,
    padding: float = 0.0,
    min_padding_ratio: float = 0.0,
    user_holes: tuple[tuple[tuple[float, float], ...], ...] = (),
) -> Polygon:
    """Nest sheet polygon (valid placement region), not the padded outer rectangle."""
    del padding, min_padding_ratio  # padding applies to void obstacles, not sheet topology
    if outline.is_empty:
        raise ValueError("board outline is empty")
    user_hole_rings = [list(h) for h in user_holes]
    if user_hole_rings:
        return Polygon(list(outline.exterior.coords), holes=user_hole_rings)
    if isinstance(outline, Polygon):
        return outline
    return Polygon(outline)  # type: ignore[arg-type]


def board_void_geometries(
    sheet: Polygon,
    *,
    outline: Polygon | None = None,
    padding: float = 0.0,
) -> list[Geometry]:
    """Forbidden void regions as Geometry solids (overlap => invalid)."""
    geoms: list[Geometry] = []
    if outline is not None:
        geoms.extend(board_void_obstacles(outline, padding))
    void = Polygon(sheet.exterior.coords).difference(sheet)
    if void.is_empty:
        return geoms
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
    min_padding_ratio: float = 0.08,
    user_holes: tuple[tuple[tuple[float, float], ...], ...] = (),
) -> tuple[Polygon, list[Geometry]]:
    if not isinstance(board, Polygon):
        board = Polygon(board)  # type: ignore[arg-type]
    pad = default_sheet_padding(
        board, extra=padding, ratio=min_padding_ratio,
    ) if min_padding_ratio > 0.0 else float(padding)
    sheet = board_sheet_from_outline(
        board,
        user_holes=user_holes,
    )
    void_geoms = board_void_geometries(sheet, outline=board, padding=pad)
    return sheet, void_geoms
