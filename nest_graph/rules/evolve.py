"""Rule-set evolution: mutate, dedupe, truncate, and demo/seed factories."""

from typing import Any, Sequence

import numpy as np
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from nest_graph.board import board_context_from_geometry
from nest_graph.config import BuildGraphConfig, SelectionConfig, score_rules_options
from nest_graph.graph import (
    Circle,
    PlacementRuleSet,
    PointAngleRule,
    PointPlaceRule,
    PoseGraph,
    RuleMutationSettings,
    ScoreRulesOptions,
    Vec2,
    augment_rules,
    nest_by_graph,
    score_elems,
    score_rules,
)
from nest_graph.propose.heavy_polish import apply_dfs_refinement

# Fields that differ across coarse/mid/fine and stay ordered coarse≥mid≥fine.
_MUT_BLEND_KEYS = ("dpos", "dw", "da", "insert_p")


def credit_mut_slot_amaf_deltas(mcts_telem: dict) -> list[float]:
    """Spend prior-iter rule_id_amaf visit deltas onto mut_slot_scale ranks.

    ``rule_id`` is a rank slot in the current pool; credit the preset that
    birthed the set that earned those visits (stored in ``mut_slot_scale``).
    """
    weights = mcts_telem.get("mut_slot_weights")
    if not isinstance(weights, list) or len(weights) < 3:
        weights = [0.0, 0.0, 0.0]
    else:
        weights = [float(w) * 0.5 for w in weights[:3]]
    cursor = mcts_telem.get("mut_amaf_cursor")
    if not isinstance(cursor, list) or len(cursor) < 3:
        cursor = [0, 0, 0]
    else:
        cursor = [int(c) for c in cursor[:3]]
    slot_scale = mcts_telem.get("mut_slot_scale")
    if not isinstance(slot_scale, list):
        slot_scale = []
    for rid in range(3):
        visits = int(mcts_telem.get(f"rule_id_amaf_{rid}", 0) or 0)
        delta = max(0, visits - int(cursor[rid]))
        cursor[rid] = visits
        if delta <= 0 or rid >= len(slot_scale):
            continue
        sc = int(slot_scale[rid])
        if 0 <= sc <= 2:
            weights[sc] = float(weights[sc]) + float(delta)
    mcts_telem["mut_slot_weights"] = weights
    mcts_telem["mut_amaf_cursor"] = cursor
    return weights


def blend_live_mutation_presets(
    live: Sequence[RuleMutationSettings],
    weights: Sequence[float],
    anchors: Sequence[RuleMutationSettings],
    *,
    alpha: float = 0.2,
) -> None:
    """Soft-shift all three live slots toward the weight hull; keep coarse≥mid≥fine."""
    n = min(3, len(live), len(anchors))
    if n < 1:
        return
    raw = [max(0.0, float(w)) for w in list(weights)[:n]]
    total = float(sum(raw))
    if total <= 1e-12:
        probs = [1.0 / float(n)] * n
    else:
        probs = [w / total for w in raw]
    a = min(1.0, max(0.0, float(alpha)))
    for key in _MUT_BLEND_KEYS:
        target = sum(
            float(probs[i]) * float(getattr(anchors[i], key)) for i in range(n)
        )
        lo = min(float(getattr(anchors[i], key)) for i in range(n))
        hi = max(float(getattr(anchors[i], key)) for i in range(n))
        vals: list[float] = []
        for i in range(n):
            cur = float(getattr(live[i], key))
            nxt = (1.0 - a) * cur + a * target
            vals.append(min(hi, max(lo, nxt)))
        # Coarse (0) stays largest, fine (n-1) smallest.
        vals.sort(reverse=True)
        for i in range(n):
            setattr(live[i], key, float(vals[i]))


def write_mut_live_telem(
    mcts_telem: dict,
    live: Sequence[RuleMutationSettings],
    weights: Sequence[float] | None = None,
) -> None:
    """Expose live dpos/insert_p + weights for letter SCORECARD (mut_dpos_0/1/2)."""
    for i, preset in enumerate(list(live)[:3]):
        mcts_telem[f"mut_dpos_{i}"] = float(getattr(preset, "dpos", 0.0) or 0.0)
        mcts_telem[f"mut_insert_p_{i}"] = float(
            getattr(preset, "insert_p", 0.0) or 0.0
        )
    if weights is not None:
        for i, w in enumerate(list(weights)[:3]):
            mcts_telem[f"mut_w_{i}"] = float(w)
    # Compat aliases used by early dual letter line.
    if live:
        mid = live[min(1, len(live) - 1)]
        mcts_telem["working_dpos"] = float(getattr(mid, "dpos", 0.0) or 0.0)
        mcts_telem["working_dw"] = float(getattr(mid, "dw", 0.0) or 0.0)


def rule_region(board: BaseGeometry) -> Circle:
    xmin, ymin, xmax, ymax = board.bounds
    return Circle.from_bounds(xmin, ymin, xmax, ymax)


def _quantize_rule_scalar(v: float, places: int = 4) -> float:
    return round(float(v), places)


def _fingerprint_rule_set(rule_set: PlacementRuleSet) -> tuple:
    parts: list[tuple] = []
    q = _quantize_rule_scalar
    for pr in rule_set.point_rules:
        parts.append(
            ("p", pr.group, q(pr.pos[0]), q(pr.pos[1]), q(pr.r), q(pr.w)),
        )
    for cr in rule_set.circle_rules:
        parts.append(
            (
                "c",
                cr.group,
                q(cr.circle.center.x),
                q(cr.circle.center.y),
                q(cr.circle.radius),
                q(cr.r),
                q(cr.w),
            ),
        )
    for pr in rule_set.point_angle_rules:
        parts.append(
            (
                "pa",
                pr.group,
                q(pr.pos[0]),
                q(pr.pos[1]),
                q(pr.a),
                q(pr.r),
                q(pr.w),
            ),
        )
    for cr in rule_set.circle_angle_rules:
        parts.append(
            (
                "ca",
                cr.group,
                q(cr.circle.center.x),
                q(cr.circle.center.y),
                q(cr.a),
                q(cr.r),
                q(cr.w),
            ),
        )
    return tuple(sorted(parts))


def dedupe_rule_sets(rule_sets: list[PlacementRuleSet]) -> list[PlacementRuleSet]:
    seen: set[tuple] = set()
    out: list[PlacementRuleSet] = []
    for rs in rule_sets:
        key = _fingerprint_rule_set(rs)
        if key in seen:
            continue
        seen.add(key)
        out.append(rs)
    return out


def _iter_rule_atoms(rule_set: PlacementRuleSet):
    """Yield (fingerprint_atom, rule_obj) for either/or building-block recombine."""
    q = _quantize_rule_scalar
    for pr in rule_set.point_rules:
        yield ("p", pr.group, q(pr.pos[0]), q(pr.pos[1]), q(pr.r), q(pr.w)), pr
    for cr in rule_set.circle_rules:
        yield (
            (
                "c",
                cr.group,
                q(cr.circle.center.x),
                q(cr.circle.center.y),
                q(cr.circle.radius),
                q(cr.r),
                q(cr.w),
            ),
            cr,
        )
    for pr in rule_set.point_angle_rules:
        yield (
            (
                "pa",
                pr.group,
                q(pr.pos[0]),
                q(pr.pos[1]),
                q(pr.a),
                q(pr.r),
                q(pr.w),
            ),
            pr,
        )
    for cr in rule_set.circle_angle_rules:
        yield (
            (
                "ca",
                cr.group,
                q(cr.circle.center.x),
                q(cr.circle.center.y),
                q(cr.a),
                q(cr.r),
                q(cr.w),
            ),
            cr,
        )


def either_or_recombine_rule_sets(
    parent_a: PlacementRuleSet,
    parent_b: PlacementRuleSet,
    *,
    seed: int = 0,
) -> PlacementRuleSet:
    """Radcliffe R³ / 2412.17104 either-or: shared atoms once; exclusive either/or."""
    rng = np.random.default_rng(int(seed) & 0xFFFFFFFF)
    atoms_a = dict(_iter_rule_atoms(parent_a))
    atoms_b = dict(_iter_rule_atoms(parent_b))
    keys_a = set(atoms_a)
    keys_b = set(atoms_b)
    out = PlacementRuleSet()
    for key in keys_a & keys_b:
        out.append_rule(atoms_a[key])
    for key in keys_a - keys_b:
        if float(rng.random()) < 0.5:
            out.append_rule(atoms_a[key])
    for key in keys_b - keys_a:
        if float(rng.random()) < 0.5:
            out.append_rule(atoms_b[key])
    return out


def _atom_overlap_sim(fp_a: tuple, fp_b: tuple) -> float:
    if not fp_a and not fp_b:
        return 1.0
    sa = set(fp_a)
    sb = set(fp_b)
    denom = max(min(len(sa), len(sb)), 1)
    return float(len(sa & sb)) / float(denom)


def diversify_survivor_select(
    scored: list[tuple[float, PlacementRuleSet]],
    n: int,
    *,
    d0: float,
) -> list[PlacementRuleSet]:
    """2412.17104 iterative survivor: pick best, subtract D0·atom_overlap from rest."""
    if n <= 0 or not scored:
        return []
    remaining = [(float(f), rs, _fingerprint_rule_set(rs)) for f, rs in scored]
    selected: list[PlacementRuleSet] = []
    while remaining and len(selected) < n:
        remaining.sort(key=lambda item: item[0], reverse=True)
        _best_f, best_rs, best_fp = remaining.pop(0)
        selected.append(best_rs)
        next_remaining: list[tuple[float, PlacementRuleSet, tuple]] = []
        for fit, rs, fp in remaining:
            pen = float(d0) * _atom_overlap_sim(best_fp, fp)
            next_remaining.append((fit - pen, rs, fp))
        remaining = next_remaining
    return selected


def truncate_rule_set(
    rule_set: PlacementRuleSet,
    max_rules: int,
) -> PlacementRuleSet:
    if rule_set.size() <= max_rules:
        return rule_set
    weighted: list[tuple[float, object]] = []
    for pr in rule_set.point_rules:
        weighted.append((pr.w, pr))
    for cr in rule_set.circle_rules:
        weighted.append((cr.w, cr))
    for pr in rule_set.point_angle_rules:
        weighted.append((pr.w, pr))
    for cr in rule_set.circle_angle_rules:
        weighted.append((cr.w, cr))
    weighted.sort(key=lambda item: abs(item[0]), reverse=True)
    out = PlacementRuleSet()
    for _, rule in weighted[:max_rules]:
        out.append_rule(rule)
    return out


def pick_mutation_preset_scale(
    presets: Sequence[RuleMutationSettings],
    *,
    elite_gap: float,
    streak: int = 0,
) -> tuple[list[RuleMutationSettings], int]:
    """M1: one mutation scale per plateau (coarse / mid / fine), not all three.

    Large elite gap or long streak → coarse; tiny gap → fine; else mid.
    """
    if not presets:
        return [], -1
    if len(presets) == 1:
        return [presets[0]], 0
    gap = float(elite_gap)
    st = int(streak)
    if gap > 0.05 or st >= 3:
        idx = 0
    elif gap < 0.01:
        idx = len(presets) - 1
    else:
        idx = min(1, len(presets) - 1)
    return [presets[idx]], int(idx)


def improve_rules(
    graphs,
    rules,
    n: int,
    board: BaseGeometry | None = None,
    *,
    mutation_presets: list[RuleMutationSettings] | None = None,
    rule_score_penalty: float = 0.03,
    elite_count: int = 16,
    seed: int = 0,
    score_options: ScoreRulesOptions | None = None,
    max_rules_per_set: int = 24,
    slot_scale_out: list[int] | None = None,
    slot_weights_inout: list[float] | None = None,
    # Compat no-ops for older call sites (fp map parked — slots are SoT).
    mut_fp_scale_out: dict | None = None,
    working_head: bool = False,
):
    del mut_fp_scale_out, working_head
    if mutation_presets is None:
        region = rule_region(board) if board is not None else Circle.from_bounds(0, 0, 1.2, 1.1)
        ng = 2
        mutation_presets = [
            RuleMutationSettings(
                region=region, dpos=0.25, dw=0.25, da=np.pi / 4,
                insert_p=0.09, remove_p=0.02, mutate_p=0.1, ngroups=ng,
            ),
            RuleMutationSettings(
                region=region, dpos=0.05, dw=0.05, da=np.pi / 32,
                insert_p=0.04, remove_p=0.01, mutate_p=0.1, ngroups=ng,
            ),
            RuleMutationSettings(
                region=region, dpos=0.01, dw=0.01, da=np.pi / 64,
                insert_p=0.01, remove_p=0.02, mutate_p=0.1, ngroups=ng,
            ),
        ]
    if score_options is None:
        score_options = ScoreRulesOptions()
        score_options.rule_complexity_penalty = rule_score_penalty
    elif score_options.rule_complexity_penalty == 0.0:
        score_options.rule_complexity_penalty = rule_score_penalty

    parents = list(rules)
    elites = parents
    parent_scores: list[float] = []
    if graphs and parents:
        rank_opts = ScoreRulesOptions()
        rank_opts.latest_graph_only = score_options.latest_graph_only
        rank_opts.count_weight = score_options.count_weight
        rank_opts.rule_complexity_penalty = score_options.rule_complexity_penalty
        rank_opts.select = score_options.select
        parent_scores = list(score_rules(graphs, parents, rank_opts))
        ranked = sorted(
            zip(parent_scores, parents),
            key=lambda item: item[0],
            reverse=True,
        )
        k = min(max(elite_count, 1), len(ranked))
        elites = [rs for _, rs in ranked[:k]]

    pool: list[PlacementRuleSet] = list(parents)
    pool_scale: list[int] = [-1] * len(parents)
    # L3: one either/or child from top-two elites before mutate scales (2412.17104).
    if len(elites) >= 2:
        xo = either_or_recombine_rule_sets(
            elites[0], elites[1], seed=int(seed) ^ 0xA5A5
        )
        if xo.size() > 0:
            pool.append(xo)
            pool_scale.append(-1)
    for preset_idx, preset in enumerate(mutation_presets):
        mutate_seed = (int(seed) + preset_idx * 10007) & 0xFFFFFFFF
        children = augment_rules(elites, preset, seed=mutate_seed)
        # Live bank slots 0..2; extras clamp to 2.
        scale = min(int(preset_idx), 2)
        for child in children:
            pool.append(child)
            pool_scale.append(int(scale))

    # First-wins dedupe with parallel scale column (same SoT as dedupe_rule_sets).
    seen: set[tuple] = set()
    deduped: list[PlacementRuleSet] = []
    deduped_scale: list[int] = []
    for rs, sc in zip(pool, pool_scale):
        key = _fingerprint_rule_set(rs)
        if key in seen:
            continue
        seen.add(key)
        deduped.append(rs)
        deduped_scale.append(int(sc))
    pool = deduped
    pool_scale = deduped_scale
    if max_rules_per_set > 0:
        pool = [truncate_rule_set(rs, max_rules_per_set) for rs in pool]
    if not pool:
        if slot_scale_out is not None:
            slot_scale_out.clear()
        return []

    fitness = score_rules(graphs, pool, score_options)
    best_parent = max((float(s) for s in parent_scores), default=0.0)
    if slot_weights_inout is not None:
        while len(slot_weights_inout) < 3:
            slot_weights_inout.append(0.0)
        for fit, sc in zip(fitness, pool_scale):
            if 0 <= int(sc) <= 2:
                # Beat-parent gain, plus a presence vote so cold AMAF still drifts.
                gain = max(float(fit) - float(best_parent), 0.0) + 0.1
                slot_weights_inout[int(sc)] = float(slot_weights_inout[int(sc)]) + gain
    scored = list(zip(fitness, pool))
    # D0 in near-tie band: ≤ count_weight so one extra selected part cannot invert.
    cw = float(getattr(score_options, "count_weight", 0.02) or 0.02)
    d0 = min(cw, 0.02)
    survivors = diversify_survivor_select(scored, n, d0=d0)
    if slot_scale_out is not None:
        id_to_scale = {id(rs): int(sc) for rs, sc in zip(pool, pool_scale)}
        slot_scale_out.clear()
        slot_scale_out.extend(int(id_to_scale.get(id(rs), -1)) for rs in survivors)
    return survivors


def score_rule_sets_with_dfs(
    graph: PoseGraph,
    rule_sets: list[PlacementRuleSet],
    selection: SelectionConfig,
    *,
    top_k: int = 4,
) -> list[float]:
    """Tier-B fitness for benchmarks: nest + DFS count on latest graph (top_k by Tier A)."""
    if not rule_sets:
        return []
    tier_a = score_rules([graph], rule_sets, score_rules_options(selection))
    order = sorted(range(len(rule_sets)), key=lambda i: tier_a[i], reverse=True)
    out = list(tier_a)
    for idx in order[: max(top_k, 0)]:
        rs = rule_sets[idx]
        selected = list(nest_by_graph(graph, [rs])[0])
        scores = score_elems(graph, rs)
        _, final, _ = apply_dfs_refinement(
            graph, rs, selected, scores, selection=selection,
        )
        out[idx] = float(len(final)) - selection.rule_score_penalty * rs.size()
    return out


def make_demo_rule_set(cfg: BuildGraphConfig) -> PlacementRuleSet:
    rc = cfg.rules
    r = rc.place_rule_radius
    wrect = rc.weight_rect
    wtri = rc.weight_tri
    aw = rc.angle_rule_weight_scale * wtri
    rule_set = PlacementRuleSet()
    rule_set.append_rule(PointPlaceRule(pos=Vec2(x=0, y=0), r=r, w=wrect, group=0))
    rule_set.append_rule(PointPlaceRule(pos=Vec2(x=0.7, y=0.7), r=r, w=wtri, group=1))
    rule_set.append_rule(PointPlaceRule(pos=Vec2(x=0, y=1.1), r=r, w=wtri, group=1))
    rule_set.append_rule(PointPlaceRule(pos=Vec2(x=1.2, y=0), r=r, w=wtri, group=1))
    for gi, wgt in ((0, wrect), (1, wtri)):
        for k in range(8):
            a = float(2.0 * np.pi * k / 8.0)
            rule_set.append_rule(
                PointAngleRule(
                    pos=Vec2(x=0.6, y=0.55),
                    r=r,
                    a=a,
                    w=aw * wgt,
                    group=gi,
                ),
            )
    return rule_set


def make_seed_rule_sets(cfg: BuildGraphConfig) -> list[PlacementRuleSet]:
    first = PlacementRuleSet()
    first.append_rule(PointPlaceRule(pos=Vec2(x=0, y=0), r=0.1, w=0.1, group=0))
    first.append_rule(PointPlaceRule(pos=Vec2(x=0, y=0), r=0.1, w=0.1, group=1))
    return [first]


def make_initial_rule_sets(cfg: BuildGraphConfig) -> list[PlacementRuleSet]:
    return make_seed_rule_sets(cfg) + [make_demo_rule_set(cfg)]


def copy_rule_set(rule_set: PlacementRuleSet) -> PlacementRuleSet:
    out = PlacementRuleSet()
    for pr in rule_set.point_rules:
        out.append_rule(pr)
    for cr in rule_set.circle_rules:
        out.append_rule(cr)
    for pr in rule_set.point_angle_rules:
        out.append_rule(pr)
    for cr in rule_set.circle_angle_rules:
        out.append_rule(cr)
    return out


def repulsor_centroids(
    board: BaseGeometry,
    nest_state: Any | None,
) -> list[tuple[float, float]]:
    sheet, _ = board_context_from_geometry(board)
    centroids: list[tuple[float, float]] = []
    c = sheet.centroid
    centroids.append((float(c.x), float(c.y)))
    if nest_state is not None and nest_state.selected_indices:
        packed = [nest_state.polys[i] for i in nest_state.selected_indices]
        merged = unary_union(packed)
        if merged is not None and not merged.is_empty:
            mc = merged.centroid
            centroids.append((float(mc.x), float(mc.y)))
    return centroids


def inject_repulsor_rules(
    rule_sets: list[PlacementRuleSet],
    cfg: BuildGraphConfig,
    board: BaseGeometry,
    nest_state: Any | None,
) -> list[PlacementRuleSet]:
    if not cfg.rules.use_repulsor_rules or not rule_sets:
        return rule_sets
    centroids = repulsor_centroids(board, nest_state)
    r = max(cfg.rules.place_rule_radius * 0.5, 1e-4)
    w = cfg.rules.repulsor_weight
    ngroups = cfg.rules.ngroups
    n_touch = min(max(cfg.rules.repulsor_sets_to_touch, 0), len(rule_sets))
    out: list[PlacementRuleSet] = []
    for i, rs in enumerate(rule_sets):
        if i >= n_touch:
            out.append(rs)
            continue
        copied = copy_rule_set(rs)
        for g in range(ngroups):
            for cx, cy in centroids[:2]:
                copied.append_rule(PointPlaceRule(
                    pos=Vec2(x=cx, y=cy), r=r, w=w, group=g,
                ))
        out.append(truncate_rule_set(copied, cfg.rules.max_rules_per_set))
    return out
