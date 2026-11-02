"""P5: cheap_expand keeps large_void dual (hard skip late-degraded vs P1)."""

from nest_graph.propose.selection_compose import dual_nest_for
from nest_graph.graph import FreeKind


class _Free:
    kind=FreeKind.LargeVoid


def test_dual_nest_cheap_expand_keeps_large_void_dual():
    # P5 research: skipping dual on cheap dropped late med; keep Q105 dual.
    assert dual_nest_for(_Free(), last_leaf=False, cheap_expand=True) is True


def test_dual_nest_outer_large_void_still_dual():
    assert dual_nest_for(_Free(), last_leaf=False, cheap_expand=False) is True


def test_dual_nest_last_leaf_always():
    assert dual_nest_for(_Free(), last_leaf=True, cheap_expand=True) is True
    assert dual_nest_for(None, last_leaf=True, cheap_expand=True) is True


def test_dual_nest_non_void_no_dual():
    class _Gap:
        kind = "gap"

    assert dual_nest_for(_Gap(), last_leaf=False, cheap_expand=True) is False
