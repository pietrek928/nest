"""Densify ProposeConfig presets as MacroAction.preset_id (DgPreset).

Apply via model_copy into that expand/outer body only — never mutate shared
pack_cache["cfg"] in place.
"""

from pathlib import Path

from nest_graph.cfg_overrides import apply_cfg_overrides
from nest_graph.config import BuildGraphConfig

_DEFAULT_PRESETS_PATH = Path("artifacts/tune/densify_presets.yaml")


def _load_yaml(path: Path) -> dict:
    try:
        import yaml
    except ImportError:
        return {}
    if not path.is_file():
        return {}
    return yaml.safe_load(path.read_text()) or {}


def load_densify_presets(path: str | Path | None = None) -> list[dict]:
    """Return presets list; always includes id=0 baseline if file missing."""
    p = Path(path) if path else _DEFAULT_PRESETS_PATH
    raw = _load_yaml(p)
    presets = list(raw.get("presets") or [])
    if not any(int(x.get("id", -1)) == 0 for x in presets):
        presets.insert(0, {"id": 0, "name": "baseline", "overrides": {}})
    return presets


def densify_preset_ids(presets: list[dict] | None) -> list[int]:
    """Ids for generate_macros. v1: only baseline unless preset has online_ok."""
    if not presets:
        return [0]
    ids: list[int] = []
    for p in presets:
        pid = int(p.get("id", 0) or 0)
        if pid == 0 or bool(p.get("online_ok")):
            ids.append(pid)
    ids = sorted(set(ids))
    return ids or [0]


def apply_densify_preset(
    cfg: BuildGraphConfig,
    preset_id: int,
    presets: list[dict] | None,
) -> BuildGraphConfig:
    """Return model_copy with propose overrides for preset_id (0 = identity)."""
    pid = int(preset_id or 0)
    if pid <= 0 or not presets:
        return cfg
    pack = None
    for p in presets:
        if int(p.get("id", -1) or -1) == pid:
            pack = p
            break
    if pack is None:
        return cfg
    overrides = pack.get("overrides") or {}
    if not overrides:
        return cfg
    specs = [f"{k}={v}" for k, v in overrides.items()]
    return apply_cfg_overrides(cfg, specs)


__all__ = [
    "apply_densify_preset",
    "densify_preset_ids",
    "load_densify_presets",
]
