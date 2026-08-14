"""LNS accept helper (3b uses strict lex; void-kNN destroy deleted)."""

from nest_graph.propose.lns_rebuild import apply_lns_destroy, lns_accept


def test_lns_accept_prefers_more_parts():
    assert lns_accept(
        old_count=5, old_area=1.0, old_cov=0.5,
        new_count=6, new_area=0.9, new_cov=0.4,
    )


def test_apply_lns_destroy():
    assert apply_lns_destroy([1, 2, 3, 4], [2, 4]) == [1, 3]
