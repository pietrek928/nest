"""Cheap MCTS expand cache adapter (Q143)."""

from contextlib import contextmanager
from typing import Sequence

from nest_graph.pack.execute import execute_pack
from nest_graph.pack.ctx import RefinePackBox
from nest_graph.graph import BoardSnapshot
from nest_graph.graph import score_elems
from nest_graph.propose.heavy_polish import (
    apply_dfs_refinement,
    apply_refine_with_restore,
    polish_budget_for_iter,
)
from nest_graph.propose.pattern_archive import inject_cohorts_from_patterns
from nest_graph.propose.selection_compose import (
    active_rule_set,
    compose_and_nest_selection,
    compose_nest_kwargs,
    sheet_diag_from,
)


def cheap_pack_cache_key(
    zone,
    action,
    *,
    compose_sz: int = 0,
    cohort_sig: int = 0,
) -> tuple[str, int, int, int, int]:
    """Q143/S2a + Q369 + M2a: cheap cache is (zone, motif_id, rule_id, compose_sz, cohort_sig)."""
    motif_id = -1
    rule_id = 0
    if action is not None:
        if int(getattr(action, "motif_id", -1) or -1) >= 0:
            motif_id = int(action.motif_id)
        rid = int(getattr(action, "rule_id", 0) or 0)
        rule_id = rid if rid >= 0 else 0
        if cohort_sig == 0:
            cohort_sig = int(getattr(action, "cohort_sig", 0) or 0)
    return (str(zone or ""), motif_id, rule_id, int(compose_sz), int(cohort_sig))


def snapshot_pack_cache(pack_cache: dict) -> dict:
    """Shallow copy pack_cache lists/dicts for restore after path replay."""
    return {
        k: (list(v) if isinstance(v, list) else dict(v) if isinstance(v, dict) else v)
        for k, v in pack_cache.items()
    }


@contextmanager
def with_isolated_pack_cache(pack_cache: dict):
    """One gate: path execute mutates compose_* — restore after probe."""
    snap = snapshot_pack_cache(pack_cache)
    try:
        yield pack_cache
    finally:
        pack_cache.clear()
        pack_cache.update(snap)


def compose_cached_selection(
    pack_cache: dict,
    *,
    rule_sets: list,
    sel,
    zone,
    patterns,
    native_geoms_fn,
    coverage_pct_fn,
    action=None,
) -> tuple[list[int], float]:
    """Cheap compose from last outer graph; Motif miss injects cluster_patterns."""
    if not pack_cache.get("ready"):
        return list(pack_cache.get("selected") or ()), 0.0
    graph = pack_cache["graph"]
    polys = pack_cache["polys"]
    group_id = pack_cache["group_id"]
    transform = pack_cache["transform"]
    part_areas_c = pack_cache["part_areas"]
    part_bases_c = pack_cache["part_bases"]
    p_sheet_c = pack_cache["p_sheet"]
    min_dist_c = float(pack_cache["min_dist"])
    cfg_c = pack_cache["cfg"]
    free_info_c = pack_cache.get("free_info")
    propose_stats_c = dict(pack_cache.get("propose_stats") or {})
    propose_stats_c["mcts_zone"] = zone
    pats = list(patterns or [])
    n_cc = 0
    if pats:
        n_cc = inject_cohorts_from_patterns(
            pats, group_id, transform, propose_stats_c,
        )
    else:
        n_cc = 0
    pack_cache["cheap_cluster_copy_n"] = int(n_cc)
    rid = int(getattr(action, "rule_id", 0) or 0) if action is not None else 0
    active_rules = active_rule_set(rule_sets, rid)
    scores = list(score_elems(graph, active_rules))
    sheet_diag = sheet_diag_from(pack_cache.get("sheet") or p_sheet_c)
    candidate_geoms = native_geoms_fn(
        group_id, transform, part_bases_c,
    )
    void_geoms = list(pack_cache.get("void_geoms") or [])
    packed_geoms = list(pack_cache.get("packed_geoms") or [])
    pgid = pack_cache.get("packed_group_id")
    ptf = pack_cache.get("packed_transform")
    if (
        pgid is not None
        and ptf is not None
        and len(pgid) == len(ptf)
        and len(pgid) > 0
    ):
        try:
            rebuilt = native_geoms_fn(pgid, ptf, part_bases_c)
            if rebuilt:
                seed_geoms = [
                    g for g in (pack_cache.get("seed_void_geoms") or []) if g is not None
                ]
                packed_geoms = list(seed_geoms) + list(rebuilt)
                propose_stats_c["w1_obstacle_sot"] = 1
                propose_stats_c["w1_packed_rebuilt_n"] = int(len(rebuilt))
        except Exception:
            pass
    composed = compose_and_nest_selection(
        **compose_nest_kwargs(
            graph=graph,
            rule_sets=rule_sets,
            active_rules=active_rules,
            scores=scores,
            polys=polys,
            group_id=group_id,
            transform=transform,
            candidate_geoms=candidate_geoms,
            packed_geoms=packed_geoms,
            part_areas=part_areas_c,
            free_info=free_info_c,
            cfg=cfg_c,
            selection=sel,
            first_pass=False,
            outline=p_sheet_c,
            min_dist=min_dist_c,
            sheet_area=float(pack_cache.get("sheet_area") or 0.0),
            sheet_diag=sheet_diag,
            propose_stats=propose_stats_c,
            ngroups=int(cfg_c.rules.ngroups),
            packed_group_id=pgid,
            packed_transform=ptf,
            last_leaf=False,
            void_geoms=void_geoms,
            dg=pack_cache.get("dg"),
            motif_base=pack_cache.get("motif_base"),
        )
    )
    pack_cache["compose_sel"] = list(composed.selected_nest)
    pack_cache["motif_locked"] = list(propose_stats_c.get("motif_locked") or ())
    pack_cache["compose_scores"] = composed.refine_scores
    pack_cache["compose_rules"] = composed.refine_rules
    pack_cache["compose_polys"] = polys
    pack_cache["compose_group_id"] = group_id
    pack_cache["compose_transform"] = transform
    pack_cache["compose_free_poly"] = composed.free_poly
    pack_cache["compose_free_info"] = composed.free_info
    selected_out = list(composed.selected_nest)
    coverage_out = 0.0
    try:
        coverage_out = coverage_pct_fn(
            selected_out, group_id, part_areas_c,
            float(pack_cache.get("board_area") or 1.0),
        ) / 100.0
    except Exception:
        pass
    return selected_out, float(coverage_out)


def refine_cached_selection(
    pack_cache: dict,
    *,
    sel,
    budget,
    apply_dfs_fn,
    native_geoms_fn,
    coverage_pct_fn,
    telem: dict,
) -> tuple[list[int], float]:
    """Cheap mid DFS via ``apply_refine_with_restore``."""
    telem["dfs_passes"] = int(budget.dfs_passes)
    if not pack_cache.get("ready") or "compose_sel" not in pack_cache:
        return list(pack_cache.get("selected") or ()), 0.0
    graph = pack_cache["graph"]
    polys = pack_cache["compose_polys"]
    group_id = pack_cache["compose_group_id"]
    transform = pack_cache["compose_transform"]
    part_areas_c = pack_cache["part_areas"]
    part_bases_c = pack_cache["part_bases"]
    nest_sel = list(pack_cache["compose_sel"])
    node_areas = [
        float(part_areas_c[int(g)]) if int(g) < len(part_areas_c) else 0.0
        for g in group_id
    ]
    selected_out = apply_refine_with_restore(
        budget=budget,
        apply_dfs_fn=apply_dfs_fn or apply_dfs_refinement,
        graph=graph,
        refine_rules=pack_cache["compose_rules"],
        selected_nest=nest_sel,
        refine_scores=pack_cache["compose_scores"],
        sel_iter=sel,
        node_areas=node_areas,
        refine_seed=0,
        locked_indices=list(pack_cache.get("motif_locked") or ()),
        polys=polys,
        group_id=group_id,
        transform=transform,
        part_areas=part_areas_c,
        part_bases=part_bases_c,
        sheet=pack_cache["p_sheet"],
        min_dist=float(pack_cache["min_dist"]),
        free_poly=pack_cache.get("compose_free_poly"),
        free_info=pack_cache.get("compose_free_info"),
        rim_before=0.0,
        rim_reject=0.02,
        propose_stats=telem,
        native_geoms_from_transforms_fn=native_geoms_fn,
        dg=pack_cache.get("dg"),
        propose_cfg=(
            cache_cfg.propose
            if (cache_cfg := pack_cache.get("cfg")) is not None
            else None
        ),
    )
    pack_cache["compose_sel"] = list(selected_out)
    if native_geoms_fn is not None and selected_out:
        try:
            pack_cache["compose_geoms"] = list(
                native_geoms_fn(
                    group_id,
                    transform,
                    part_bases_c,
                )
            )
        except Exception:
            pack_cache.pop("compose_geoms", None)
    coverage_out = 0.0
    try:
        coverage_out = coverage_pct_fn(
            selected_out, group_id, part_areas_c,
            float(pack_cache.get("board_area") or 1.0),
        ) / 100.0
    except Exception:
        pass
    return list(selected_out), float(coverage_out)


def pack_execute_snapshot(
    parent,
    *,
    zone=None,
    action=None,
    patterns=None,
    pack_cache: dict,
    rule_sets: list,
    sel,
    cfg,
    apply_dfs_fn,
    native_geoms_fn,
    coverage_pct_fn,
) -> BoardSnapshot:
    """Cheap expand execute_fn: cache (zone, motif_id); miss re-composes (Q143)."""
    snap = parent
    compose_sz = len(pack_cache.get("motif_locked") or ())
    pack_cache["cache_key_compose_sz"] = int(compose_sz)
    # Wp/M2a: pack_cache motif_cohort_sig is SoT (synced from propose); action attr fallback.
    cohort_sig = int(pack_cache.get("motif_cohort_sig", 0) or 0)
    if cohort_sig == 0 and action is not None:
        cohort_sig = int(getattr(action, "cohort_sig", 0) or 0)
    if cohort_sig == 0:
        ps = pack_cache.get("propose_stats")
        if isinstance(ps, dict):
            cohort_sig = int(ps.get("motif_cohort_sig", 0) or 0)
    pack_cache["cache_key_cohort_sig"] = int(cohort_sig)
    cache_key = cheap_pack_cache_key(
        zone, action, compose_sz=compose_sz, cohort_sig=cohort_sig
    )
    cheap_map: dict = pack_cache.setdefault("cheap_by_key", {})
    pack_cache["cache_lookup_n"] = int(pack_cache.get("cache_lookup_n", 0) or 0) + 1
    if cache_key in cheap_map:
        cached = cheap_map[cache_key]
        pack_cache["cache_hit_n"] = int(pack_cache.get("cache_hit_n", 0) or 0) + 1
        pack_cache["last_execute_telem"] = dict(
            pack_cache.get("cheap_telem_by_key", {}).get(cache_key) or {}
        )
        pack_cache["last_execute_telem"]["cache_hit"] = 1
        # Q384: restore compose fields so post_pack_overlap_ok sees this replay.
        comp = (pack_cache.get("cheap_compose_by_key") or {}).get(cache_key)
        if isinstance(comp, dict):
            for k, v in comp.items():
                pack_cache[k] = list(v) if isinstance(v, list) else v
        lookup_n = int(pack_cache.get("cache_lookup_n", 0) or 0)
        hit_n = int(pack_cache.get("cache_hit_n", 0) or 0)
        pack_cache["cache_miss_rate"] = 1.0 - (float(hit_n) / float(lookup_n))
        return cached

    pack_cache["cache_miss_n"] = int(pack_cache.get("cache_miss_n", 0) or 0) + 1

    pats = list(patterns or [])
    telem: dict = {
        "patterns_n": len(pats),
        "zone": zone,
        "cache_key": cache_key,
        "cache_hit": 0,
    }

    box = RefinePackBox()

    def compose_fn() -> None:
        sel_out, cov = compose_cached_selection(
            pack_cache,
            rule_sets=rule_sets,
            sel=sel,
            zone=zone,
            patterns=pats,
            native_geoms_fn=native_geoms_fn,
            coverage_pct_fn=coverage_pct_fn,
            action=action,
        )
        box.selected = list(sel_out)
        box.coverage = float(cov)

    budget = polish_budget_for_iter(
        is_last_leaf=False,
        sel=sel,
        cheap_expand=True,
        large_void=False,
    )
    box.budget = budget

    def refine_fn() -> None:
        sel_out, cov = refine_cached_selection(
            pack_cache,
            sel=sel,
            budget=budget,
            apply_dfs_fn=apply_dfs_fn or apply_dfs_refinement,
            native_geoms_fn=native_geoms_fn,
            coverage_pct_fn=coverage_pct_fn,
            telem=telem,
        )
        box.selected = list(sel_out)
        box.coverage = float(cov)

    stage = execute_pack(
        rim_only=False,
        heavy=False,
        compose_fn=compose_fn,
        refine_fn=refine_fn,
        post_pack_fn=None,
    )
    telem.update(stage)
    telem["pack_body"] = 1
    telem["polish_budget"] = "mid"
    telem["cluster_copy"] = int(pack_cache.get("cheap_cluster_copy_n") or 0)
    if telem["cluster_copy"]:
        telem.setdefault("emitted_by_proposer", {})["cluster_copy"] = telem["cluster_copy"]
    selected_out = list(box.selected)
    coverage_out = float(box.coverage)
    if not selected_out:
        selected_out = list(
            pack_cache.get("selected")
            or getattr(snap, "packed_gids", ())
            or ()
        )
        coverage_out = float(getattr(snap, "coverage", 0.0) or coverage_out)
    packed_gids = tuple(getattr(snap, "packed_gids", ()) or ())
    packed_tf = tuple(getattr(snap, "packed_transforms", ()) or ())
    rem = tuple(getattr(snap, "remaining_gids", ()) or ())
    if pack_cache.get("ready") and selected_out and "compose_group_id" in pack_cache:
        gid_list = pack_cache["compose_group_id"]
        tf_list = pack_cache["compose_transform"]
        gids = []
        tfs = []
        for i in selected_out:
            ii = int(i)
            if 0 <= ii < len(gid_list) and ii < len(tf_list):
                gids.append(int(gid_list[ii]))
                t = tf_list[ii]
                tfs.append((float(t[0]), float(t[1]), float(t[2])))
        if gids:
            packed_gids = tuple(gids)
            packed_tf = tuple(tfs)
            rem = tuple(
                g for g in range(int(pack_cache["cfg"].rules.ngroups))
                if g not in set(gids)
            )
    free_kind = str(
        getattr(pack_cache.get("compose_free_info"), "kind", None)
        or getattr(snap, "free_kind", "")
        or ""
    )
    motif_used: tuple[int, ...] = ()
    if cache_key[1] >= 0:
        motif_used = (int(cache_key[1]),)
    out = BoardSnapshot(
        packed_gids=packed_gids,
        packed_transforms=packed_tf,
        remaining_gids=rem,
        coverage=float(coverage_out),
        arena_node_id=int(getattr(snap, "arena_node_id", 0) or 0),
        kiss_pairs=int(getattr(snap, "kiss_pairs", 0) or 0),
        mean_compactness=float(getattr(snap, "mean_compactness", 0.0) or 0.0),
        rim_fill=float(getattr(snap, "rim_fill", 0.0) or 0.0),
        void_fill=float(getattr(snap, "void_fill", 0.0) or 0.0),
        free_kind=free_kind,
        motif_ids_used=motif_used or tuple(getattr(snap, "motif_ids_used", ()) or ()),
    )
    pack_cache["last_execute_telem"] = telem
    pack_cache.setdefault("cheap_telem_by_key", {})[cache_key] = dict(telem)
    pack_cache.setdefault("cheap_compose_by_key", {})[cache_key] = {
        "compose_sel": list(pack_cache.get("compose_sel") or ()),
        "compose_polys": list(pack_cache.get("compose_polys") or ()),
        "compose_group_id": list(pack_cache.get("compose_group_id") or ()),
        "compose_transform": list(pack_cache.get("compose_transform") or ()),
        "motif_locked": list(pack_cache.get("motif_locked") or ()),
        "compose_geoms": list(pack_cache.get("compose_geoms") or ()),
    }
    cheap_map[cache_key] = out
    lookup_n = int(pack_cache.get("cache_lookup_n", 0) or 0)
    hit_n = int(pack_cache.get("cache_hit_n", 0) or 0)
    pack_cache["cache_miss_rate"] = 1.0 - (float(hit_n) / float(lookup_n)) if lookup_n else 0.0
    return out


def invalidate_cheap_cache(pack_cache: dict, *, reason: str) -> None:
    """Clear cheap_by_key when proposer context shifts (D0)."""
    cheap_map = pack_cache.get("cheap_by_key")
    if isinstance(cheap_map, dict) and cheap_map:
        cheap_map.clear()
        pack_cache["cache_invalidate_reason"] = str(reason)
        pack_cache["cache_invalidate_n"] = int(
            pack_cache.get("cache_invalidate_n", 0) or 0
        ) + 1


def maybe_invalidate_cheap_cache(
    pack_cache: dict,
    *,
    remaining_gids: Sequence[int] | None = None,
    void_elite_seeded: int | None = None,
    archive_elite_n: int | None = None,
    compose_sz: int | None = None,
    cohort_sig: int | None = None,
) -> None:
    """D0 staleness guard: invalidate when pool/elite/lock context shifts."""
    if remaining_gids is not None:
        rem_key = tuple(sorted(int(g) for g in remaining_gids))
        prev_rem = pack_cache.get("last_remaining_gids")
        if prev_rem is not None and prev_rem != rem_key:
            invalidate_cheap_cache(pack_cache, reason="remaining_gids")
        pack_cache["last_remaining_gids"] = rem_key
    if void_elite_seeded is not None:
        prev_elite = int(pack_cache.get("last_void_elite_seeded", -1))
        if prev_elite >= 0 and prev_elite != int(void_elite_seeded):
            invalidate_cheap_cache(pack_cache, reason="void_elite_seeded")
        pack_cache["last_void_elite_seeded"] = int(void_elite_seeded)
    if archive_elite_n is not None:
        prev_arch = int(pack_cache.get("last_archive_elite_n", -1))
        if prev_arch >= 0 and prev_arch != int(archive_elite_n):
            invalidate_cheap_cache(pack_cache, reason="archive_elite_n")
        pack_cache["last_archive_elite_n"] = int(archive_elite_n)
    if compose_sz is not None:
        prev_sz = int(pack_cache.get("last_compose_sz", -1))
        if prev_sz >= 0 and prev_sz != int(compose_sz):
            invalidate_cheap_cache(pack_cache, reason="compose_sz")
            pack_cache["cache_invalidate_compose"] = 1
        pack_cache["last_compose_sz"] = int(compose_sz)
    if cohort_sig is not None:
        # Soft: only invalidate on nonzero→nonzero shift (ready drop to 0 must not thrash).
        prev_coh = int(pack_cache.get("last_cohort_sig", 0) or 0)
        cur = int(cohort_sig)
        if prev_coh != 0 and cur != 0 and prev_coh != cur:
            invalidate_cheap_cache(pack_cache, reason="cohort_sig")
            pack_cache["cache_invalidate_cohort"] = 1
        if cur != 0:
            pack_cache["last_cohort_sig"] = cur


__all__ = [
    "cheap_pack_cache_key",
    "compose_cached_selection",
    "invalidate_cheap_cache",
    "maybe_invalidate_cheap_cache",
    "pack_execute_snapshot",
    "refine_cached_selection",
    "snapshot_pack_cache",
    "with_isolated_pack_cache",
]
