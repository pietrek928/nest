"""Block replace 3a cohort swap and 3b hole-victim selection."""

from shapely.geometry import Point, box

from nest_graph.config import ProposeConfig
from nest_graph.elem_graph import (
    Circle,
    PoseGraph,
    Vec2,
)
from nest_graph.geometry import Geometry
from nest_graph.propose.block_replace import (
    lex_count_area_better,
    pick_block_hole_victim,
    try_block_cohort_swap,
)


def _square_geom(x: float, y: float, s: float = 1.0) -> Geometry:
    return Geometry.from_shapely(box(x, y, x + s, y + s))


def test_lex_count_beats_area():
    assert lex_count_area_better(
        old_count=3, old_area=10.0, new_count=4, new_area=1.0,
    )
    assert not lex_count_area_better(
        old_count=3, old_area=10.0, new_count=2, new_area=99.0,
    )
    assert lex_count_area_better(
        old_count=3, old_area=1.0, new_count=3, new_area=1.5,
    )


def test_cohort_swap_overlapping_third_never_tried():
    """A collides with B; disjoint C is never a related trial."""
    g = PoseGraph()
    xs = (0.0, 1.0, 10.0, 11.0, 30.0, 31.0, 5.0)
    for x in xs:
        g.append_elem(0, Vec2(x=x, y=0.0), Circle.from_center_radius(x, 0.0, 0.4))
    # A={0,1} collides with B={2,3}; 6 collides with A only. No intra-cohort edges.
    for a in (0, 1):
        for b in (2, 3):
            g.add_collision_pair(a, b)
        g.add_collision_pair(a, 6)
    # C collides with B (not A) so nest cannot fill C after the swap.
    for c in (4, 5):
        for b in (2, 3):
            g.add_collision_pair(c, b)
    transforms = [(float(x), 0.0, 0.0) for x in xs]
    group_id = [0] * 7
    geoms = [_square_geom(x, 0.0, 0.8) for x in xs]
    cohorts = [
        {
            "leader_key": (0.0, 0.0, 0.0),
            "leader_gid": 0,
            "member_keys": [(0, (0.0, 0.0, 0.0)), (0, (1.0, 0.0, 0.0))],
        },
        {
            "leader_key": (10.0, 0.0, 0.0),
            "leader_gid": 0,
            "member_keys": [(0, (10.0, 0.0, 0.0)), (0, (11.0, 0.0, 0.0))],
        },
        {
            "leader_key": (30.0, 0.0, 0.0),
            "leader_gid": 0,
            "member_keys": [(0, (30.0, 0.0, 0.0)), (0, (31.0, 0.0, 0.0))],
        },
    ]
    scores = [1.0, 1.0, 2.0, 2.0, 0.1, 0.1, 5.0]
    part_areas = [1.0]
    new_sel, new_locked, telem = try_block_cohort_swap(
        graph=g,
        scores=scores,
        selected=[0, 1],
        locked_motif=[0, 1],
        cohorts=cohorts,
        candidate_geoms=geoms,
        void_geoms=[],
        group_id=group_id,
        transform=transforms,
        part_areas=part_areas,
        min_dist=0.05,
    )
    assert telem["block_cohort_related"] >= 1
    assert telem["block_cohort_accepted"] == 1
    assert set(new_sel) >= {2, 3, 6}
    assert 0 not in new_sel and 1 not in new_sel
    assert 4 not in new_sel and 5 not in new_sel
    assert set(new_locked) >= {2, 3}


def test_cohort_swap_keeps_other_pinned_cohort():
    g = PoseGraph()
    xs = (0.0, 1.0, 10.0, 11.0, 20.0, 21.0, 0.5)
    for x in xs:
        g.append_elem(0, Vec2(x=x, y=0.0), Circle.from_center_radius(x, 0.0, 0.4))
    for a in (0, 1):
        for b in (2, 3):
            g.add_collision_pair(a, b)
        g.add_collision_pair(a, 6)
    transforms = [(float(x), 0.0, 0.0) for x in xs]
    geoms = [_square_geom(x, 20.0, 0.8) if i >= 4 else _square_geom(x, 0.0, 0.8)
             for i, x in enumerate(xs)]
    geoms[4] = _square_geom(20.0, 20.0, 0.8)
    geoms[5] = _square_geom(21.0, 20.0, 0.8)
    geoms[6] = _square_geom(5.0, 10.0, 0.8)
    cohorts = [
        {
            "leader_key": (0.0, 0.0, 0.0),
            "leader_gid": 0,
            "member_keys": [(0, (0.0, 0.0, 0.0)), (0, (1.0, 0.0, 0.0))],
        },
        {
            "leader_key": (10.0, 0.0, 0.0),
            "leader_gid": 0,
            "member_keys": [(0, (10.0, 0.0, 0.0)), (0, (11.0, 0.0, 0.0))],
        },
        {
            "leader_key": (20.0, 0.0, 0.0),
            "leader_gid": 0,
            "member_keys": [(0, (20.0, 0.0, 0.0)), (0, (21.0, 0.0, 0.0))],
        },
    ]
    new_sel, new_locked, telem = try_block_cohort_swap(
        graph=g,
        scores=[1.0, 1.0, 3.0, 3.0, 9.0, 9.0, 8.0],
        selected=[0, 1, 4, 5],
        locked_motif=[0, 1, 4, 5],
        cohorts=cohorts,
        candidate_geoms=geoms,
        void_geoms=[],
        group_id=[0] * 7,
        transform=transforms,
        part_areas=[1.0],
        min_dist=0.05,
    )
    assert telem["block_cohort_accepted"] == 1
    assert 4 in new_locked and 5 in new_locked
    assert 0 not in new_locked and 1 not in new_locked


def test_hole_victim_skips_board_adj():
    sheet = box(0, 0, 20, 20)
    # Rim strip (board_adj) plus interior 3-part blob nearer the pole.
    polys = [
        box(0.05, 0.05, 1.05, 1.05),
        box(1.10, 0.05, 2.10, 1.05),
        box(2.15, 0.05, 3.15, 1.05),
        box(8.0, 8.0, 9.0, 9.0),
        box(9.05, 8.0, 10.05, 9.0),
        box(8.0, 9.05, 9.0, 10.05),
    ]
    victim = pick_block_hole_victim(
        list(range(6)),
        polys,
        min_dist=0.08,
        sheet=sheet,
        pole=Point(9.0, 9.0),
        min_size=3,
        max_size=6,
    )
    assert victim is not None
    assert set(victim) <= {3, 4, 5}
    assert not (set(victim) & {0, 1, 2})


def test_enable_block_replace_default_on():
    assert ProposeConfig().enable_block_replace is True
    assert ProposeConfig().enable_lns_rebuild is True


def test_sequential_beam_lock_sets_independent():
    from shapely.geometry import Point

    from nest_graph.propose.motif_lock import sequential_accept_motif_cohorts

    class _G:
        collisions = [[] for _ in range(4)]

    geoms = [
        _square_geom(0.0, 0.0),
        _square_geom(2.0, 0.0),
        _square_geom(20.0, 0.0),
        _square_geom(22.0, 0.0),
    ]
    cohorts = [
        {
            "leader_key": (0.0, 0.0, 0.0),
            "leader_gid": 0,
            "member_keys": [(0, (0.0, 0.0, 0.0)), (0, (2.0, 0.0, 0.0))],
        },
        {
            "leader_key": (20.0, 0.0, 0.0),
            "leader_gid": 0,
            "member_keys": [(0, (20.0, 0.0, 0.0)), (0, (22.0, 0.0, 0.0))],
        },
    ]
    locked, telem = sequential_accept_motif_cohorts(
        graph=_G(),
        scores=[3.0, 2.0, 1.0, 1.0],
        group_id=[0, 0, 0, 0],
        transform=[
            (0.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
            (20.0, 0.0, 0.0),
            (22.0, 0.0, 0.0),
        ],
        cohorts=cohorts,
        candidate_geoms=geoms,
        void_geoms=[],
        packed_geoms=[],
        min_dist=0.05,
        pole=Point(0, 0),
        max_accept=3,
    )
    assert set(locked) == {0, 1, 2, 3}
    assert telem["motif_beam_sets"] == 2
    assert len(telem["motif_lock_sets"]) == 2
    assert set(telem["motif_lock_sets"][0]) == {0, 1}
    assert set(telem["motif_lock_sets"][1]) == {2, 3}


def test_incumbent_map_and_lex_hold():
    from nest_graph.propose.selection_compose import (
        _lex_pick_better,
        _map_incumbent_indices,
    )

    class _G:
        collisions = [[], [], []]

    group_id = [0, 0, 1]
    transform = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (5.0, 0.0, 0.0)]
    mapped = _map_incumbent_indices(
        group_id=group_id,
        transform=transform,
        packed_group_id=[0, 1],
        packed_transform=[(0.0, 0.0, 0.0), (5.0, 0.0, 0.0)],
        graph=_G(),
    )
    assert mapped == [0, 2]
    # New nest with fewer parts is not lex-better than incumbent.
    assert not _lex_pick_better(
        best=mapped,
        cand=[1],
        group_id=group_id,
        part_areas=[1.0, 1.0],
    )
    assert _lex_pick_better(
        best=[1],
        cand=mapped,
        group_id=group_id,
        part_areas=[1.0, 1.0],
    )


def test_void_hold_override_count_floor():
    """Void colonization may beat hold only when count ≥ 0.9× incumbent."""
    from nest_graph.propose.void_selection import count_selected_in_free

    free = box(5, 5, 15, 15)
    # Incumbent: 10 rim parts outside free; cand: 9 parts with 2 in free.
    polys = [box(float(i), 0, float(i) + 0.5, 0.5) for i in range(10)]
    polys_cand = list(polys)
    polys_cand[8] = box(6, 6, 7, 7)
    polys_cand[9] = box(8, 8, 9, 9)
    incumbent = list(range(10))
    cand = [0, 1, 2, 3, 4, 5, 6, 8, 9]  # 9 ≥ 0.9*10
    assert len(cand) >= int(0.9 * len(incumbent))
    assert count_selected_in_free(polys, incumbent, free) == 0
    assert count_selected_in_free(polys_cand, cand, free) == 2
    collapse = list(range(8))  # 8 < 9
    assert len(collapse) < int(0.9 * len(incumbent))


def test_refine_restore_lex_not_better():
    """Mirror build_graph refine restore: keep nest_before when refine loses count."""
    assert not lex_count_area_better(
        old_count=70,
        old_area=10.0,
        new_count=57,
        new_area=12.0,
    )
    assert lex_count_area_better(
        old_count=57,
        old_area=10.0,
        new_count=57,
        new_area=12.0,
    )
