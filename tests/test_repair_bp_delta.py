"""R1: gainful repair Δ folds into one expand reward (no second backprop visit)."""

from nest_graph.pack.execute import repair_bp_delta_addend


def test_repair_bp_uses_darea_over_board():
    propose_stats = {
        "hole_expand_gain": 1,
        "block_hole_dcount": 1,
        "block_hole_darea": 0.02,
    }
    mcts_telem: dict = {}
    delta = repair_bp_delta_addend(
        propose_stats=propose_stats,
        mcts_telem=mcts_telem,
        board_area=10.0,
    )
    assert abs(delta - 0.002) < 1e-9
    assert propose_stats["repair_bp_delta"] == delta
    assert mcts_telem["repair_bp_delta"] == delta


def test_repair_bp_dcount_floor_when_no_darea():
    propose_stats = {
        "hole_expand_gain": 1,
        "block_hole_dcount": 1,
        "block_hole_darea": 0.0,
    }
    mcts_telem: dict = {}
    delta = repair_bp_delta_addend(
        propose_stats=propose_stats,
        mcts_telem=mcts_telem,
        board_area=10.0,
    )
    assert delta == 0.01


def test_repair_bp_skips_without_hole_gain():
    propose_stats = {"hole_expand_gain": 0, "block_hole_dcount": 2}
    mcts_telem: dict = {}
    delta = repair_bp_delta_addend(
        propose_stats=propose_stats,
        mcts_telem=mcts_telem,
        board_area=10.0,
    )
    assert delta == 0.0
    assert propose_stats["repair_bp_delta"] == 0.0


def test_repair_bp_caps_at_005():
    propose_stats = {
        "hole_expand_gain": 1,
        "block_hole_darea": 1.0,
        "block_hole_dcount": 0,
    }
    mcts_telem: dict = {}
    delta = repair_bp_delta_addend(
        propose_stats=propose_stats,
        mcts_telem=mcts_telem,
        board_area=10.0,
    )
    assert delta == 0.05


def test_repair_bp_ge4_cheap_outer_clamp():
    propose_stats = {"hole_expand_gain": 0}
    mcts_telem = {"last_cheap_reward": 0.50}
    delta = repair_bp_delta_addend(
        propose_stats=propose_stats,
        mcts_telem=mcts_telem,
        board_area=10.0,
        outer_reward=0.58,
    )
    assert abs(delta - 0.02) < 1e-9  # capped (degrade ladder)
    assert "last_cheap_reward" not in mcts_telem  # consumed one-shot
    # Positive-only: outer below cheap yields 0.
    delta_neg = repair_bp_delta_addend(
        propose_stats={"hole_expand_gain": 0},
        mcts_telem={"last_cheap_reward": 0.60},
        outer_reward=0.50,
    )
    assert delta_neg == 0.0
