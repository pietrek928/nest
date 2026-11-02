"""FreeKind helpers: wire tokens for telem/ckpt; parse legacy strings.

Leaf module: imports FreeKind from nest_graph.graph only.
"""

from typing import Any

from nest_graph.graph import FreeKind

_TOKEN_TO_KIND: dict[str, FreeKind] = {
    "": FreeKind.Empty,
    "large_void": FreeKind.LargeVoid,
    "swiss_cheese": FreeKind.SwissCheese,
    "full": FreeKind.Full,
}

_KIND_TO_TOKEN: dict[FreeKind, str] = {
    FreeKind.Empty: "",
    FreeKind.LargeVoid: "large_void",
    FreeKind.SwissCheese: "swiss_cheese",
    FreeKind.Full: "full",
}

# str(FreeKind.X) / Enum.name fallout from mechanical migrates.
_NAME_TO_KIND: dict[str, FreeKind] = {
    "Empty": FreeKind.Empty,
    "LargeVoid": FreeKind.LargeVoid,
    "SwissCheese": FreeKind.SwissCheese,
    "Full": FreeKind.Full,
    "FreeKind.Empty": FreeKind.Empty,
    "FreeKind.LargeVoid": FreeKind.LargeVoid,
    "FreeKind.SwissCheese": FreeKind.SwissCheese,
    "FreeKind.Full": FreeKind.Full,
}


def free_kind_token(fk: Any) -> str:
    """Legacy telem/ckpt token for a FreeKind (or already-token str)."""
    return _KIND_TO_TOKEN.get(free_kind_from_any(fk), "")


def free_kind_from_any(v: Any) -> FreeKind:
    """Parse FreeKind from enum, legacy token, or object with ``.kind``."""
    if v is None:
        return FreeKind.Empty
    if isinstance(v, FreeKind):
        return v
    if isinstance(v, str):
        if v in _TOKEN_TO_KIND:
            return _TOKEN_TO_KIND[v]
        if v in _NAME_TO_KIND:
            return _NAME_TO_KIND[v]
        return FreeKind.Empty
    # FreeSpaceAnalysis.kind / BoardSnapshot.free_kind / similar wrappers.
    for attr in ("free_kind", "kind"):
        kind_attr = getattr(v, attr, None)
        if kind_attr is not None and kind_attr is not v:
            return free_kind_from_any(kind_attr)
    try:
        return FreeKind(v.value)
    except Exception:
        pass
    try:
        return FreeKind(int(v))
    except Exception:
        return FreeKind.Empty


def is_large_void(v: Any) -> bool:
    return free_kind_from_any(v) == FreeKind.LargeVoid


def coalesce_free_kind(*vals: Any) -> FreeKind:
    """First non-Empty FreeKind. ``FreeKind.Empty`` is truthy — never use ``a or b``."""
    for v in vals:
        if v is None:
            continue
        kind = free_kind_from_any(v)
        if kind != FreeKind.Empty:
            return kind
    return FreeKind.Empty
