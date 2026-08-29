"""Action generator: rim/void/sheet × rule, plus PLACE_MOTIF from MotifBase."""


from typing import Sequence

from nest_graph.elem_graph import MacroAction, MacroRegion, MotifBase


def generate_macros(
    remaining_gids: tuple[int, ...],
    *,
    rule_ids: tuple[int, ...] = (0,),
    motif_base: MotifBase | None = None,
    prefer_motifs: bool = True,
    warm_motif_ids: tuple[int, ...] = (),
    free_kind: str = "",
    motif_cohorts: Sequence[dict] | None = None,
) -> list[MacroAction]:
    """Yield discrete macro actions for progressive widening (Q70–Q73).

    Dg: under ``large_void``, order Motif/Void before Rim/Sheet (still emit all).
    """
    actions: list[MacroAction] = []
    rem = tuple(int(g) for g in remaining_gids)
    if not rem:
        return actions

    seen_motifs: set[int] = set()
    motif_actions: list[MacroAction] = []
    if prefer_motifs and motif_base is not None and motif_base.size() > 0:
        # Q79: related-sig warm motifs first (library still MotifBase SoT).
        ordered_ids = list(warm_motif_ids) + list(range(int(motif_base.size())))
        for mid in ordered_ids:
            mid_i = int(mid)
            if mid_i in seen_motifs or mid_i < 0 or mid_i >= int(motif_base.size()):
                continue
            seen_motifs.add(mid_i)
            rec = motif_base.at(mid_i)
            if int(rec.gid_a) in rem and int(rec.gid_b) in rem:
                a = MacroAction()
                a.region = MacroRegion.Motif
                a.motif_id = mid_i
                a.part_gid = int(rec.gid_a)
                a.rule_id = int(rule_ids[0]) if rule_ids else 0
                motif_actions.append(a)

    cohort_actions: list[MacroAction] = []
    seen_cohort: set[tuple[int, tuple[float, float, float] | None]] = set()
    for c in motif_cohorts or ():
        if not isinstance(c, dict):
            continue
        mid = int(c.get("motif_id", -1) or -1)
        leader = int(c.get("leader_gid", rem[0] if rem else -1))
        lk = c.get("leader_key")
        key_t = tuple(lk) if isinstance(lk, tuple) else None
        # Q387: PLACE_COHORT requires full member_keys (≥2) for MotifJoin star.
        members = c.get("member_keys") or ()
        if len(members) < 2:
            continue
        sig = (mid, key_t)
        if sig in seen_cohort:
            continue
        if leader not in rem:
            continue
        seen_cohort.add(sig)
        for rid in rule_ids:
            a = MacroAction()
            a.region = MacroRegion.Motif
            a.motif_id = mid
            a.part_gid = leader
            a.rule_id = int(rid)
            cohort_actions.append(a)

    if free_kind == "large_void":
        region_order = (MacroRegion.Void, MacroRegion.Rim, MacroRegion.Sheet)
    else:
        region_order = (MacroRegion.Rim, MacroRegion.Void, MacroRegion.Sheet)

    # Motifs + cohorts first when preferring; under large_void stay ahead of Rim/Sheet.
    actions.extend(motif_actions)
    actions.extend(cohort_actions)
    for region in region_order:
        for rid in rule_ids:
            a = MacroAction()
            a.region = region
            a.rule_id = int(rid)
            a.part_gid = int(rem[0])
            a.motif_id = -1
            actions.append(a)
    return actions


def region_to_zone(region: MacroRegion) -> str:
    if region == MacroRegion.Rim:
        return "cluster_edge"
    if region == MacroRegion.Void:
        return "void_seek"
    if region == MacroRegion.Motif:
        return "void_seek"  # Q104
    if region == MacroRegion.Sheet:
        return "interior_pocket"  # Q98
    return "interior_pocket"
