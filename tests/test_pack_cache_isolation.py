"""P0: path/cch telem peeked before isolation restore; restore is bit-identical."""

from nest_graph.pack.cheap import (
    peek_isolation_telem,
    warm_outer_pack_cache,
    with_isolated_pack_cache,
)


def test_isolation_restores_compose_and_counters():
    """Restore is bit-identical — peek before exit to capture path telem."""
    cache = {
        "compose_sel": [1, 2],
        "cache_hit_n": 0,
        "cache_lookup_n": 0,
        "path_compose_ms": 0.0,
        "path_refine_ms": 0.0,
    }
    peeked = {}
    with with_isolated_pack_cache(cache):
        cache["compose_sel"] = [9]
        cache["cache_lookup_n"] = 3
        cache["cache_hit_n"] = 2
        cache["path_compose_ms"] = 12.5
        cache["path_refine_ms"] = 4.0
        peeked.update(peek_isolation_telem(cache))
    assert cache["compose_sel"] == [1, 2]
    assert int(cache["cache_lookup_n"]) == 0
    assert int(cache["cache_hit_n"]) == 0
    assert float(cache["path_compose_ms"]) == 0.0
    assert int(peeked["cache_lookup_n"]) == 3
    assert int(peeked["cache_hit_n"]) == 2
    assert float(peeked["path_compose_ms"]) == 12.5
    assert float(peeked["path_refine_ms"]) == 4.0


def test_isolation_restores_compose_fields():
    cache = {"compose_sel": [0], "motif_locked": [1], "other": 7}
    with with_isolated_pack_cache(cache):
        cache["compose_sel"] = [3, 4]
        cache["motif_locked"] = []
        cache["other"] = 99
    assert cache["compose_sel"] == [0]
    assert cache["motif_locked"] == [1]
    assert cache["other"] == 7


def test_warm_outer_pack_cache_key_set():
    """L6a: warm writes the outer multi-sim key set (clears then update)."""
    cache = {"stale": 1, "compose_sel": [9]}
    warm_outer_pack_cache(
        cache,
        graph="g",
        dg="dg",
        polys=[1],
        group_id=[0],
        transform=[(0.0, 0.0, 0.0)],
        part_areas=[1.0],
        part_bases={},
        p_sheet="sheet",
        min_dist=0.5,
        cfg=None,
        densify_presets=(),
        free_info=None,
        propose_stats={"motif_locked": (3,)},
        packed_geoms=[],
        packed_group_id=None,
        packed_transform=None,
        void_geoms=[],
        seed_void_geoms=[],
        sheet=None,
        board_area=10.0,
        selected_polys=[0],
        f1_sticky_spent=2,
    )
    expected = {
        "ready",
        "graph",
        "dg",
        "polys",
        "group_id",
        "transform",
        "part_areas",
        "part_bases",
        "p_sheet",
        "min_dist",
        "cfg",
        "densify_presets",
        "free_info",
        "propose_stats",
        "packed_geoms",
        "packed_group_id",
        "packed_transform",
        "void_geoms",
        "seed_void_geoms",
        "sheet",
        "sheet_area",
        "board_area",
        "selected",
        "compose_sel",
        "compose_polys",
        "compose_group_id",
        "compose_transform",
        "motif_locked",
        "f1_sticky_spent",
        "cache_hit_n",
        "cache_lookup_n",
        "cache_miss_n",
        "path_compose_ms",
        "path_refine_ms",
    }
    assert set(cache) == expected
    assert "stale" not in cache
    assert cache["ready"] is True
    assert cache["compose_sel"] == [0]
    assert cache["motif_locked"] == [3]
    assert int(cache["f1_sticky_spent"]) == 2
