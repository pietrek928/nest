"""L3: MotifBase.truncate keeps one outsider gid-pair; inject diversifies by pair."""

from nest_graph.graph import MotifBase, MotifRecord, Se2
from nest_graph.propose.pattern_archive import (
    diversify_motif_ids_by_gid_pair,
    patterns_from_motif_base,
)


def _rec(
    ga: int,
    gb: int,
    *,
    accept: int,
    gci: float,
    dx: float = 0.0,
    dy: float = 0.0,
    ttl: int = 2,
) -> MotifRecord:
    r = MotifRecord()
    r.gid_a = int(ga)
    r.gid_b = int(gb)
    # Distinct relative → distinct motif_key (same gid pair can coexist).
    r.relative = Se2(float(dx), float(dy), 0.0)
    r.accept_count = int(accept)
    r.gci = float(gci)
    r.compactness = 0.9
    r.area_a = 1.0
    r.area_b = 1.0
    r.ttl_remaining = int(ttl)
    return r


def test_truncate_keeps_accept_gci_topk():
    """L3 outsider reserve parked — truncate is accept→gci top-k again."""
    base = MotifBase()
    for i in range(4):
        mid = base.upsert(
            _rec(0, 1, accept=10 - i, gci=0.9 - 0.01 * i, dx=float(i), dy=0.0),
            ttl=2,
        )
        assert mid >= 0
    mid_out = base.upsert(_rec(2, 3, accept=1, gci=0.5, dx=10.0, dy=10.0), ttl=2)
    assert mid_out >= 0
    assert base.size() >= 5
    base.truncate(4)
    assert base.size() == 4
    # Lowest gci outsider dropped under plain top-k.
    pairs = set()
    for i in range(base.size()):
        rec = base.at(i)
        ga, gb = int(rec.gid_a), int(rec.gid_b)
        pairs.add((ga, gb) if ga <= gb else (gb, ga))
    assert (2, 3) not in pairs


def test_diversify_motif_ids_prefers_distinct_pairs():
    base = MotifBase()
    ids = []
    for i in range(3):
        mid = base.upsert(
            _rec(0, 1, accept=5, gci=0.8 - 0.01 * i, dx=float(i), dy=1.0),
            ttl=2,
        )
        ids.append(int(mid))
    mid_out = base.upsert(_rec(4, 5, accept=2, gci=0.4, dx=20.0, dy=20.0), ttl=2)
    ids.append(int(mid_out))
    div = diversify_motif_ids_by_gid_pair(base, ids, max_keep=2)
    assert len(div) == 2
    pairs = {
        (min(int(base.at(i).gid_a), int(base.at(i).gid_b)),
         max(int(base.at(i).gid_a), int(base.at(i).gid_b)))
        for i in div
    }
    assert len(pairs) == 2


def test_patterns_from_motif_base_sets_inject_pair_telem():
    base = MotifBase()
    base.upsert(_rec(0, 1, accept=3, gci=0.8, dx=0.0, dy=0.0), ttl=2)
    base.upsert(_rec(2, 3, accept=2, gci=0.7, dx=5.0, dy=5.0), ttl=2)
    telem: dict = {}
    pats = patterns_from_motif_base(base, max_keep=4, telem=telem)
    assert len(pats) >= 1
    assert int(telem.get("inject_pair_n", 0) or 0) >= 1
