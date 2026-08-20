"""Cheap MCTS expand cache adapter (Q143)."""

from nest_graph.decision.execute import execute_pack
from nest_graph.decision.pack_loop import RefinePackBox
from nest_graph.decision.types import BoardSnapshot
from nest_graph.elem_graph import score_elems
from nest_graph.propose.heavy_polish import (
    apply_refine_with_restore,
    polish_budget_for_iter,
)
from nest_graph.propose.pattern_archive import merge_motif_hits, motif_graph_hits
from nest_graph.propose.selection_compose import (
    active_rule_set,
    compose_and_nest_selection,
    compose_nest_kwargs,
    sheet_diag_from,
)


def cheap_pack_cache_key(zone, action) -> tuple[str, int]:
    """Q143: cheap cache is (zone, motif_id); motif_id=-1 when not Motif."""
    motif_id = -1
    if action is not None and int(getattr(action, "motif_id", -1) or -1) >= 0:
        motif_id = int(action.motif_id)
    return (str(zone or ""), motif_id)


def compose_cached_selection(
    pack_cache: dict,
    *,
    rule_sets: list,
    sel,
    zone,
    patterns,
    native_geoms_fn,
    coverage_pct_fn,
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
        motif_keys, cohorts, n_cc = motif_graph_hits(pats, group_id, transform)
        merge_motif_hits(propose_stats_c, motif_keys, cohorts)
    pack_cache["cheap_cluster_copy_n"] = int(n_cc)
    active_rules = active_rule_set(rule_sets)
    scores = list(score_elems(graph, active_rules))
    sheet_diag = sheet_diag_from(pack_cache.get("sheet") or p_sheet_c)
    candidate_geoms = native_geoms_fn(
        group_id, transform, part_bases_c,
    )
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
            packed_geoms=list(pack_cache.get("packed_geoms") or []),
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
            packed_group_id=pack_cache.get("packed_group_id"),
            packed_transform=pack_cache.get("packed_transform"),
            last_leaf=False,
            void_geoms=pack_cache.get("void_geoms"),
            dg=pack_cache.get("dg"),
        )
    )
    pack_cache["compose_sel"] = list(composed.selected_nest)
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
        apply_dfs_fn=apply_dfs_fn,
        graph=graph,
        refine_rules=pack_cache["compose_rules"],
        selected_nest=nest_sel,
        refine_scores=pack_cache["compose_scores"],
        sel_iter=sel,
        node_areas=node_areas,
        refine_seed=0,
        locked_indices=[],
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
    )
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
    cache_key = cheap_pack_cache_key(zone, action)
    cheap_map: dict = pack_cache.setdefault("cheap_by_key", {})
    pack_cache["cache_lookup_n"] = int(pack_cache.get("cache_lookup_n", 0) or 0) + 1
    if cache_key in cheap_map:
        cached = cheap_map[cache_key]
        pack_cache["cache_hit_n"] = int(pack_cache.get("cache_hit_n", 0) or 0) + 1
        pack_cache["last_execute_telem"] = dict(
            pack_cache.get("cheap_telem_by_key", {}).get(cache_key) or {}
        )
        pack_cache["last_execute_telem"]["cache_hit"] = 1
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
            apply_dfs_fn=apply_dfs_fn,
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
    motif_used = ()
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
    cheap_map[cache_key] = out
    lookup_n = int(pack_cache.get("cache_lookup_n", 0) or 0)
    hit_n = int(pack_cache.get("cache_hit_n", 0) or 0)
    pack_cache["cache_miss_rate"] = 1.0 - (float(hit_n) / float(lookup_n)) if lookup_n else 0.0
    return out


__all__ = [
    "cheap_pack_cache_key",
    "compose_cached_selection",
    "pack_execute_snapshot",
    "refine_cached_selection",
]
