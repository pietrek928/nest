"""PlaceZone helpers: legacy wire tokens for densify maps / telem.

Leaf module: imports PlaceZone from nest_graph.graph only.
"""

from typing import Any

from nest_graph.graph import PlaceZone

_TOKEN_TO_ZONE: dict[str, PlaceZone] = {
    "empty_border": PlaceZone.EmptyBorder,
    "border_gap": PlaceZone.BorderGap,
    "interior_pocket": PlaceZone.InteriorPocket,
    "cluster_edge": PlaceZone.ClusterEdge,
    "inter_cluster": PlaceZone.InterCluster,
    "void_seek": PlaceZone.VoidSeek,
    # Legacy alias (Python edge only — C++ dropped ``"void"``).
    "void": PlaceZone.VoidSeek,
}

_ZONE_TO_TOKEN: dict[PlaceZone, str] = {
    PlaceZone.EmptyBorder: "empty_border",
    PlaceZone.BorderGap: "border_gap",
    PlaceZone.InteriorPocket: "interior_pocket",
    PlaceZone.ClusterEdge: "cluster_edge",
    PlaceZone.InterCluster: "inter_cluster",
    PlaceZone.VoidSeek: "void_seek",
}

_NAME_TO_ZONE: dict[str, PlaceZone] = {
    "EmptyBorder": PlaceZone.EmptyBorder,
    "BorderGap": PlaceZone.BorderGap,
    "InteriorPocket": PlaceZone.InteriorPocket,
    "ClusterEdge": PlaceZone.ClusterEdge,
    "InterCluster": PlaceZone.InterCluster,
    "VoidSeek": PlaceZone.VoidSeek,
    "EMPTY_BORDER": PlaceZone.EmptyBorder,
    "BORDER_GAP": PlaceZone.BorderGap,
    "INTERIOR_POCKET": PlaceZone.InteriorPocket,
    "CLUSTER_EDGE": PlaceZone.ClusterEdge,
    "INTER_CLUSTER": PlaceZone.InterCluster,
    "VOID_SEEK": PlaceZone.VoidSeek,
}


def place_zone_token(z: Any) -> str:
    """Legacy densify/telem token for a PlaceZone (or already-token str)."""
    return _ZONE_TO_TOKEN.get(place_zone_from_any(z), "")


def place_zone_from_any(v: Any) -> PlaceZone | None:
    """Parse PlaceZone from enum or legacy token; unknown → None."""
    if v is None:
        return None
    if isinstance(v, PlaceZone):
        return v
    if isinstance(v, str):
        if v in _TOKEN_TO_ZONE:
            return _TOKEN_TO_ZONE[v]
        if v in _NAME_TO_ZONE:
            return _NAME_TO_ZONE[v]
        # str(PlaceZone.X) → "PlaceZone.X"
        if v.startswith("PlaceZone."):
            return _NAME_TO_ZONE.get(v[len("PlaceZone."):])
        return None
    try:
        return PlaceZone(v.value)
    except Exception:
        pass
    try:
        return PlaceZone(int(v))
    except Exception:
        return None


def is_void_seek(v: Any) -> bool:
    return place_zone_from_any(v) == PlaceZone.VoidSeek
