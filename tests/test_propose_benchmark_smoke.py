"""Smoke tests tied to propose benchmark calibration."""

from nest_graph.config import BuildGraphConfig, ProposeConfig
from scripts.benchmark_propose_common import run_propose_with_metrics, shipped_propose_config


def test_shipped_preset_partial_pack_contact_smoke():
    cfg = BuildGraphConfig()
    row = run_propose_with_metrics(
        shipped_propose_config(),
        "partial_pack",
        seed=0,
        base_cfg=cfg,
        preset_label="shipped",
    )
    assert row.valid_count >= 8
    # Shipped preset keeps neighbor_slide off; partial_pack contact_min ~0.15 (see docs/propose_benchmark.md).
    assert row.contact_dist_min < 0.20
    assert row.final_count >= 8


def test_phase2_shapely_proposers_default_off():
    cfg = ProposeConfig()
    assert cfg.use_neighbor_slide is False
    assert cfg.use_axis_push is False
    assert cfg.use_bottom_left is False
    assert cfg.use_nfp_vertices is False
