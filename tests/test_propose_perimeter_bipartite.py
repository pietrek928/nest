"""P6: perimeter_walk emits on ring fixture (one bipartite path)."""

from nest_graph.config import ProposeConfig
from nest_graph.propose import propose_placements_perimeter_walk
from tests.fixtures.propose_emit import perimeter_ring_board


def test_perimeter_walk_ring_fixture_emits():
    geom, base, part = perimeter_ring_board()
    coords = propose_placements_perimeter_walk(
        base,
        part,
        geom.sheet,
        0.2,
        propose_geom=geom,
        pt_push=base.centroid,
        use_free_region=True,
        border_focus=False,
        num_angles=8,
        top_n=12,
    )
    assert coords
    assert all(len(c) == 3 for c in coords)
