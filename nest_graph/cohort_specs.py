"""Convert Python motif cohort dicts for C++ generate_macros."""

from nest_graph.free_kind_util import free_kind_from_any
from nest_graph.graph import FreeKind, MotifCohortSpec, generate_macros as _generate_macros_native


def motif_cohort_specs(cohorts) -> list:
    specs: list = []
    for c in cohorts or ():
        if not isinstance(c, dict):
            continue
        members = c.get("member_keys") or ()
        if len(members) < 2:
            continue
        spec = MotifCohortSpec()
        spec.motif_id = int(c.get("motif_id", -1) or -1)
        spec.leader_gid = int(c.get("leader_gid", -1) or -1)
        spec.member_keys_count = int(len(members))
        specs.append(spec)
    return specs


def generate_macros(
    remaining_gids,
    *,
    rule_ids=(0,),
    motif_base=None,
    prefer_motifs=True,
    warm_motif_ids=(),
    free_kind=None,
    motif_cohorts=None,
    preset_ids=(),
    void_fit_r: float = 0.0,
):
    return _generate_macros_native(
        [int(g) for g in remaining_gids],
        [int(r) for r in rule_ids],
        motif_base,
        bool(prefer_motifs),
        [int(m) for m in warm_motif_ids],
        free_kind_from_any(free_kind if free_kind is not None else FreeKind.Empty),
        motif_cohort_specs(motif_cohorts),
        [int(p) for p in (preset_ids or ())],
        float(void_fit_r),
    )
