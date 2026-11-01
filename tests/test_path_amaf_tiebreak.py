"""H3: path sibling order uses survive primary then AMAF soft score."""


def test_amaf_tiebreak_order_prefers_higher_amaf_on_survive_tie():
    # Mirror H3 sort: equal survive → higher AMAF first.
    siblings = [0, 1, 2]
    survive = {0: 1, 1: 1, 2: 0}
    amaf = {0: 0.1, 1: 0.9, 2: 9.0}
    order = sorted(
        siblings,
        key=lambda i: (-survive[i], -amaf[i]),
    )
    assert order == [1, 0, 2]
