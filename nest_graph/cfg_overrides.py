"""Shared ProposeConfig/SelectionConfig override parser (bench + densify presets)."""

import hashlib
import json
from typing import Any

from nest_graph.config import BuildGraphConfig


def parse_cfg_value(raw: str) -> Any:
    s = raw.strip()
    low = s.lower()
    if low in ("true", "yes", "1"):
        return True
    if low in ("false", "no", "0"):
        return False
    try:
        if "." in s:
            return float(s)
        return int(s)
    except ValueError:
        return s


def apply_cfg_overrides(
    cfg: BuildGraphConfig,
    specs: list[str] | None,
) -> BuildGraphConfig:
    """Mute/retune ProposeConfig or SelectionConfig fields. No new flags."""
    if not specs:
        return cfg
    propose_upd: dict = {}
    selection_upd: dict = {}
    for spec in specs:
        if "=" not in spec:
            raise ValueError(f"cfg override must be key=value got {spec!r}")
        key, raw = spec.split("=", 1)
        key = key.strip()
        val = parse_cfg_value(raw)
        if key.startswith("selection."):
            selection_upd[key.split(".", 1)[1]] = val
        elif key.startswith("propose."):
            propose_upd[key.split(".", 1)[1]] = val
        else:
            propose_upd[key] = val
    out = cfg
    if propose_upd:
        out = out.model_copy(
            update={"propose": out.propose.model_copy(update=propose_upd)}
        )
    if selection_upd:
        out = out.model_copy(
            update={"selection": out.selection.model_copy(update=selection_upd)}
        )
    return out


def effective_propose_cfg_dict(
    cfg: BuildGraphConfig,
    keys: list[str] | None = None,
) -> dict[str, Any]:
    """Subset of propose (+ selection.*) values for JSONL / cfg_sig."""
    prop = cfg.propose
    sel = cfg.selection
    if keys is None:
        keys = [
            "void_island_score_boost",
            "motif_score_boost",
            "cascade_explorer_budget_scale",
            "macro_path_beam",
            "selection.dfs_passes",
        ]
    out: dict[str, Any] = {}
    for key in keys:
        if key.startswith("selection."):
            field = key.split(".", 1)[1]
            out[key] = getattr(sel, field, None)
        else:
            out[key] = getattr(prop, key, None)
    return out


def cfg_sig(cfg_dict: dict[str, Any]) -> str:
    payload = json.dumps(cfg_dict, sort_keys=True, default=str)
    return hashlib.sha1(payload.encode("utf-8")).hexdigest()[:12]
