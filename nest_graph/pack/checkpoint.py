"""NestState + DG memory checkpoint (save at iter N / resume extra iters).

One gate: ``save_nest_checkpoint`` / ``load_nest_checkpoint``. Omits live PoseGraph
and full DecisionArena tree (fresh arena on resume; tip BoardSnapshot as root).
"""

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from shapely import wkt as shapely_wkt
from shapely.geometry.base import BaseGeometry

from nest_graph.graph import BoardSnapshot, MacroNicheArchive, MotifBase, MotifRecord, Se2


CHECKPOINT_VERSION = 1
DEFAULT_MOTIF_INJECT_CAP = 16
DEFAULT_HISTORY_CAP = 256


@dataclass
class NestCheckpoint:
    """POD payload for densify late-stage resume."""

    iter: int
    seed: int | None
    coverage: float
    parts: int
    independent_ok: bool
    polys_wkt: list[str]
    group_id: list[int]
    transform: list[tuple[float, float, float]]
    selected_indices: list[int]
    seed_count: int
    best_pack_cov: float = 0.0
    best_pack_sel: list[int] | None = None
    best_pack_sig: float = 0.0
    best_pack_tf: list[tuple[float, float, float]] | None = None
    best_pack_seed_n: int = 0
    void_elite_by_group: dict[int, list[list[float]]] = field(default_factory=dict)
    history: list[list[list[float]]] = field(default_factory=list)
    graph_valid_carry: list[list[list[float]]] = field(default_factory=list)
    motif_inject: list[dict[str, Any]] = field(default_factory=list)
    niche_rows: list[dict[str, Any]] = field(default_factory=list)
    tip_snap: dict[str, Any] = field(default_factory=dict)
    active_rule_id: int = 0
    n_rule_sets: int = 0
    free_kind: str = ""
    version: int = CHECKPOINT_VERSION


def _poly_to_wkt(poly: Any) -> str:
    if isinstance(poly, str):
        return poly
    if isinstance(poly, BaseGeometry):
        return poly.wkt
    g = getattr(poly, "to_shapely", None)
    if callable(g):
        return g().wkt
    raise TypeError(f"cannot serialize poly type={type(poly)!r}")


def _tf_row(t: Any) -> tuple[float, float, float]:
    return (float(t[0]), float(t[1]), float(t[2]))


def _arr_list(arr: Any, *, cap: int) -> list[list[float]]:
    if arr is None:
        return []
    a = np.asarray(arr, dtype=np.float64)
    if a.ndim == 1:
        if a.size < 3:
            return []
        a = a.reshape(1, -1)
    if a.ndim != 2 or a.shape[0] == 0:
        return []
    n = min(int(a.shape[0]), max(int(cap), 0))
    return [[float(a[i, 0]), float(a[i, 1]), float(a[i, 2])] for i in range(n)]


def _motif_inject_rows(motif_base: Any, *, cap: int) -> list[dict[str, Any]]:
    if motif_base is None or int(getattr(motif_base, "size", lambda: 0)()) <= 0:
        return []
    ids = list(motif_base.list_for_inject(int(cap)))
    out: list[dict[str, Any]] = []
    for mid in ids:
        rec = motif_base.at(int(mid))
        rel = rec.relative
        out.append(
            {
                "gid_a": int(rec.gid_a),
                "gid_b": int(rec.gid_b),
                "rel": (float(rel.x), float(rel.y), float(rel.a)),
                "gci": float(getattr(rec, "gci", 0.0) or 0.0),
                "compactness": float(getattr(rec, "compactness", 0.0) or 0.0),
                "accept_count": int(getattr(rec, "accept_count", 0) or 0),
                "area_a": float(getattr(rec, "area_a", 1.0) or 1.0),
                "area_b": float(getattr(rec, "area_b", 1.0) or 1.0),
                "ttl_remaining": int(getattr(rec, "ttl_remaining", 0) or 0),
            }
        )
    return out


def _niche_rows(niche_archive: Any) -> list[dict[str, Any]]:
    if niche_archive is None or int(getattr(niche_archive, "size", 0) or 0) <= 0:
        return []
    active_fn = getattr(niche_archive, "active_by_group", None)
    if not callable(active_fn):
        return []
    try:
        by_g = active_fn(64)
    except Exception:
        return []
    out: list[dict[str, Any]] = []
    for gid, rows in (by_g or {}).items():
        for row in rows or ():
            out.append(
                {
                    "key": (int(gid), 0, 0),
                    "row": (int(gid), float(row[0]), float(row[1]), float(row[2])),
                    "hits": 1,
                    "misses": 0,
                }
            )
    return out


def _tip_snap_dict(snap: Any) -> dict[str, Any]:
    if snap is None:
        return {}
    rem = list(getattr(snap, "remaining_gids", ()) or ())
    packed_gids = list(getattr(snap, "packed_gids", ()) or ())
    packed_tf = [_tf_row(t) for t in (getattr(snap, "packed_transforms", ()) or ())]
    return {
        "packed_gids": packed_gids,
        "packed_transforms": packed_tf,
        "remaining_gids": rem,
        "coverage": float(getattr(snap, "coverage", 0.0) or 0.0),
        "kiss_pairs": int(getattr(snap, "kiss_pairs", 0) or 0),
        "mean_compactness": float(getattr(snap, "mean_compactness", 0.0) or 0.0),
        "rim_fill": float(getattr(snap, "rim_fill", 0.0) or 0.0),
        "void_fill": float(getattr(snap, "void_fill", 0.0) or 0.0),
        "free_kind": str(getattr(snap, "free_kind", "") or ""),
        "motif_ids_used": list(getattr(snap, "motif_ids_used", ()) or ()),
    }


def save_nest_checkpoint(
    path: str | Path,
    *,
    iter_n: int,
    seed: int | None,
    coverage: float,
    parts: int,
    independent_ok: bool,
    nest_state: Any,
    best_pack_cov: float = 0.0,
    best_pack_sel: list[int] | None = None,
    best_pack_sig: float = 0.0,
    best_pack_tf: list | None = None,
    best_pack_seed_n: int = 0,
    void_elite_by_group: dict | None = None,
    history: Any = None,
    graph_valid_carry: Any = None,
    motif_base: Any = None,
    niche_archive: Any = None,
    tip_snap: Any = None,
    active_rule_id: int = 0,
    n_rule_sets: int = 0,
    history_cap: int = DEFAULT_HISTORY_CAP,
    motif_inject_cap: int = DEFAULT_MOTIF_INJECT_CAP,
) -> NestCheckpoint:
    """Serialize NestState + densify memory to ``.npz`` (+ sidecar meta.json)."""
    if nest_state is None:
        raise ValueError("save_nest_checkpoint: nest_state required")
    polys_wkt = [_poly_to_wkt(p) for p in nest_state.polys]
    group_id = [int(g) for g in nest_state.group_id]
    transform = [_tf_row(t) for t in nest_state.transform]
    selected = [int(i) for i in (nest_state.selected_indices or ())]
    elite: dict[int, list[list[float]]] = {}
    for gid, rows in (void_elite_by_group or {}).items():
        elite[int(gid)] = _arr_list(np.asarray(rows, dtype=np.float64), cap=history_cap)
    hist_out: list[list[list[float]]] = []
    for arm in history or ():
        hist_out.append(_arr_list(arm, cap=history_cap))
    carry_out: list[list[list[float]]] = []
    for arm in graph_valid_carry or ():
        carry_out.append(_arr_list(arm, cap=history_cap))
    bp_tf = None
    if best_pack_tf is not None:
        bp_tf = [_tf_row(t) for t in best_pack_tf]
    ckpt = NestCheckpoint(
        iter=int(iter_n),
        seed=None if seed is None else int(seed),
        coverage=float(coverage),
        parts=int(parts),
        independent_ok=bool(independent_ok),
        polys_wkt=polys_wkt,
        group_id=group_id,
        transform=transform,
        selected_indices=selected,
        seed_count=int(nest_state.seed_count or 0),
        best_pack_cov=float(best_pack_cov),
        best_pack_sel=None if best_pack_sel is None else [int(i) for i in best_pack_sel],
        best_pack_sig=float(best_pack_sig),
        best_pack_tf=bp_tf,
        best_pack_seed_n=int(best_pack_seed_n),
        void_elite_by_group=elite,
        history=hist_out,
        graph_valid_carry=carry_out,
        motif_inject=_motif_inject_rows(motif_base, cap=motif_inject_cap),
        niche_rows=_niche_rows(niche_archive),
        tip_snap=_tip_snap_dict(tip_snap),
        active_rule_id=int(active_rule_id),
        n_rule_sets=int(n_rule_sets),
        free_kind=str((_tip_snap_dict(tip_snap) or {}).get("free_kind", "") or ""),
    )
    out_path = Path(path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    meta = {
        "version": ckpt.version,
        "iter": ckpt.iter,
        "seed": ckpt.seed,
        "coverage": ckpt.coverage,
        "parts": ckpt.parts,
        "independent_ok": ckpt.independent_ok,
        "seed_count": ckpt.seed_count,
        "best_pack_cov": ckpt.best_pack_cov,
        "best_pack_sig": ckpt.best_pack_sig,
        "best_pack_seed_n": ckpt.best_pack_seed_n,
        "best_pack_sel": ckpt.best_pack_sel,
        "active_rule_id": ckpt.active_rule_id,
        "n_rule_sets": ckpt.n_rule_sets,
        "free_kind": ckpt.free_kind,
        "motif_inject": ckpt.motif_inject,
        "niche_rows": [
            {
                "key": list(r["key"]),
                "hits": int(r.get("hits", 0) or 0),
                "misses": int(r.get("misses", 0) or 0),
                **({"row": list(r["row"])} if "row" in r else {}),
            }
            for r in ckpt.niche_rows
        ],
        "tip_snap": ckpt.tip_snap,
        "void_elite_by_group": {str(k): v for k, v in ckpt.void_elite_by_group.items()},
        "history": ckpt.history,
        "graph_valid_carry": ckpt.graph_valid_carry,
    }
    np.savez_compressed(
        out_path,
        polys_wkt=np.array(ckpt.polys_wkt, dtype=object),
        group_id=np.asarray(ckpt.group_id, dtype=np.int32),
        transform=np.asarray(ckpt.transform, dtype=np.float64),
        selected_indices=np.asarray(ckpt.selected_indices, dtype=np.int32),
        best_pack_tf=(
            np.asarray(ckpt.best_pack_tf, dtype=np.float64)
            if ckpt.best_pack_tf is not None
            else np.zeros((0, 3), dtype=np.float64)
        ),
        meta_json=np.array(json.dumps(meta), dtype=object),
    )
    return ckpt


def load_nest_checkpoint(path: str | Path) -> NestCheckpoint:
    """Load checkpoint written by ``save_nest_checkpoint``."""
    data = np.load(Path(path), allow_pickle=True)
    meta = json.loads(str(data["meta_json"].item()))
    polys_wkt = [str(x) for x in data["polys_wkt"].tolist()]
    group_id = [int(x) for x in np.asarray(data["group_id"]).tolist()]
    transform = [_tf_row(t) for t in np.asarray(data["transform"], dtype=np.float64)]
    selected = [int(x) for x in np.asarray(data["selected_indices"]).tolist()]
    bp_tf_arr = np.asarray(data["best_pack_tf"], dtype=np.float64)
    bp_tf = None
    if bp_tf_arr.ndim == 2 and bp_tf_arr.shape[0] > 0:
        bp_tf = [_tf_row(t) for t in bp_tf_arr]
    elite_raw = meta.get("void_elite_by_group") or {}
    elite = {int(k): list(v) for k, v in elite_raw.items()}
    niche_rows: list[dict[str, Any]] = []
    for r in meta.get("niche_rows") or []:
        entry: dict[str, Any] = {
            "key": tuple(int(x) for x in r["key"]),
            "hits": int(r.get("hits", 0) or 0),
            "misses": int(r.get("misses", 0) or 0),
        }
        if "row" in r:
            row = r["row"]
            entry["row"] = (int(row[0]), float(row[1]), float(row[2]), float(row[3]))
        niche_rows.append(entry)
    tip = meta.get("tip_snap") or {}
    if tip:
        tip = {
            **tip,
            "packed_gids": list(tip.get("packed_gids") or ()),
            "packed_transforms": [
                _tf_row(t) for t in (tip.get("packed_transforms") or ())
            ],
            "remaining_gids": list(tip.get("remaining_gids") or ()),
            "motif_ids_used": list(tip.get("motif_ids_used") or ()),
        }
    return NestCheckpoint(
        iter=int(meta["iter"]),
        seed=meta.get("seed"),
        coverage=float(meta.get("coverage", 0.0) or 0.0),
        parts=int(meta.get("parts", 0) or 0),
        independent_ok=bool(meta.get("independent_ok", True)),
        polys_wkt=polys_wkt,
        group_id=group_id,
        transform=transform,
        selected_indices=selected,
        seed_count=int(meta.get("seed_count", 0) or 0),
        best_pack_cov=float(meta.get("best_pack_cov", 0.0) or 0.0),
        best_pack_sel=(
            None
            if meta.get("best_pack_sel") is None
            else [int(i) for i in meta["best_pack_sel"]]
        ),
        best_pack_sig=float(meta.get("best_pack_sig", 0.0) or 0.0),
        best_pack_tf=bp_tf,
        best_pack_seed_n=int(meta.get("best_pack_seed_n", 0) or 0),
        void_elite_by_group=elite,
        history=list(meta.get("history") or []),
        graph_valid_carry=list(meta.get("graph_valid_carry") or []),
        motif_inject=list(meta.get("motif_inject") or []),
        niche_rows=niche_rows,
        tip_snap=tip,
        active_rule_id=int(meta.get("active_rule_id", 0) or 0),
        n_rule_sets=int(meta.get("n_rule_sets", 0) or 0),
        free_kind=str(meta.get("free_kind", "") or ""),
        version=int(meta.get("version", CHECKPOINT_VERSION) or CHECKPOINT_VERSION),
    )


def nest_state_from_checkpoint(ckpt: NestCheckpoint, nest_state_cls: Any) -> Any:
    """Rebuild NestState (shapely polys) from checkpoint WKT."""
    polys = [shapely_wkt.loads(w) for w in ckpt.polys_wkt]
    return nest_state_cls(
        polys=polys,
        group_id=list(ckpt.group_id),
        transform=[tuple(t) for t in ckpt.transform],
        selected_indices=list(ckpt.selected_indices),
        seed_count=int(ckpt.seed_count),
    )


def rehydrate_motif_base(
    ckpt: NestCheckpoint,
    motif_base: MotifBase | None = None,
) -> MotifBase:
    """Upsert inject pairs via MotifBase.upsert (no second archive)."""
    mb = motif_base if motif_base is not None else MotifBase()
    for row in ckpt.motif_inject:
        rec = MotifRecord()
        rec.gid_a = int(row["gid_a"])
        rec.gid_b = int(row["gid_b"])
        rel = row["rel"]
        rec.relative = Se2(float(rel[0]), float(rel[1]), float(rel[2]))
        rec.gci = float(row.get("gci", 0.0) or 0.0)
        rec.compactness = float(row.get("compactness", 0.0) or 0.0)
        rec.accept_count = int(row.get("accept_count", 0) or 0)
        rec.area_a = float(row.get("area_a", 1.0) or 1.0)
        rec.area_b = float(row.get("area_b", 1.0) or 1.0)
        ttl = int(row.get("ttl_remaining", 4) or 4)
        mb.upsert(rec, 0.0, max(ttl, 1))
    return mb


def rehydrate_niche_archive(
    ckpt: NestCheckpoint,
    niche_archive: MacroNicheArchive | None = None,
) -> MacroNicheArchive:
    """Rehydrate MacroNicheArchive active rows via append_positive."""
    arch = niche_archive if niche_archive is not None else MacroNicheArchive()
    for row in ckpt.niche_rows:
        if "row" not in row:
            continue
        key = tuple(int(x) for x in row["key"])
        r = row["row"]
        arch.append_positive(
            key,
            [(int(r[0]), float(r[1]), float(r[2]), float(r[3]))],
            int(row.get("hits", 1) or 1),
            float(row.get("hits", 1) or 1),
            4,
        )
    return arch


def tip_board_snapshot(ckpt: NestCheckpoint, *, arena_node_id: int = 0) -> BoardSnapshot:
    """Build tip BoardSnapshot POD for fresh arena root/parent."""
    tip = ckpt.tip_snap or {}
    packed_gids = tip.get("packed_gids")
    packed_tf = tip.get("packed_transforms")
    remaining = tip.get("remaining_gids")
    if not packed_gids:
        packed_gids = [int(ckpt.group_id[i]) for i in ckpt.selected_indices if i < len(ckpt.group_id)]
        packed_tf = [
            _tf_row(ckpt.transform[i])
            for i in ckpt.selected_indices
            if i < len(ckpt.transform)
        ]
        packed_set = set(packed_gids)
        remaining = [g for g in range(2) if g not in packed_set]
    return BoardSnapshot(
        packed_gids=tuple(int(g) for g in (packed_gids or ())),
        packed_transforms=tuple(_tf_row(t) for t in (packed_tf or ())),
        remaining_gids=tuple(int(g) for g in (remaining or ())),
        coverage=float(tip.get("coverage", ckpt.coverage) or 0.0),
        arena_node_id=int(arena_node_id),
        kiss_pairs=int(tip.get("kiss_pairs", 0) or 0),
        mean_compactness=float(tip.get("mean_compactness", 0.0) or 0.0),
        rim_fill=float(tip.get("rim_fill", 0.0) or 0.0),
        void_fill=float(tip.get("void_fill", 0.0) or 0.0),
        free_kind=str(tip.get("free_kind", ckpt.free_kind) or ""),
        motif_ids_used=tuple(int(x) for x in (tip.get("motif_ids_used") or ())),
    )


def history_tuple_from_ckpt(ckpt: NestCheckpoint) -> tuple[np.ndarray, ...]:
    arms = list(ckpt.history or [])
    while len(arms) < 2:
        arms.append([])
    return tuple(
        np.asarray(arm, dtype=np.float64).reshape(-1, 3)
        if arm
        else np.zeros((0, 3), dtype=np.float64)
        for arm in arms[:2]
    )


def carry_tuple_from_ckpt(ckpt: NestCheckpoint) -> tuple[np.ndarray, ...]:
    arms = list(ckpt.graph_valid_carry or [])
    while len(arms) < 2:
        arms.append([])
    return tuple(
        np.asarray(arm, dtype=np.float64).reshape(-1, 3)
        if arm
        else np.zeros((0, 3), dtype=np.float64)
        for arm in arms[:2]
    )


def void_elite_from_ckpt(ckpt: NestCheckpoint) -> dict[int, list[np.ndarray]]:
    out: dict[int, list[np.ndarray]] = {0: [], 1: []}
    for gid, rows in (ckpt.void_elite_by_group or {}).items():
        out[int(gid)] = [np.asarray(r, dtype=np.float64) for r in rows]
    return out
