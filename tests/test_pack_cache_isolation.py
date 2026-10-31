"""P0: path/cch telem peeked before isolation restore; restore is bit-identical."""

from nest_graph.pack.cheap import peek_isolation_telem, with_isolated_pack_cache


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
