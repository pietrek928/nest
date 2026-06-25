import pytest

from scripts.guidance_placement_fixtures import SCENARIO_IDS, get_scenarios, run_scenario

_CAST_MOVE_TAGS = ("Snap", "Dock", "Corner", "Floor Walk", "Ejection", "Gravity")


@pytest.mark.parametrize("label", SCENARIO_IDS)
def test_guidance_scenario_produces_propositions(label: str) -> None:
    scenario = next(s for s in get_scenarios() if s.label == label)
    res, guidance = run_scenario(scenario)

    assert len(guidance.propositions) >= 1
    assert res.props_count >= 1

    if scenario.intentional_overlap:
        assert res.is_penetrating or not res.seed_board_valid
    else:
        assert res.seed_board_valid or res.board_valid

    if scenario.expected_move_substrings:
        haystack = " | ".join(res.move_types + [res.best_move_type])
        assert any(sub in haystack for sub in scenario.expected_move_substrings), (
            f"{label}: expected one of {scenario.expected_move_substrings}, got {haystack}"
        )


def test_nest_border_dock_prefers_cast_over_attractor() -> None:
    scenario = next(s for s in get_scenarios() if s.label == "nest_border_dock")
    res, guidance = run_scenario(scenario)

    assert len(guidance.propositions) >= 2
    assert "Target Attractor" not in res.best_move_type
    assert any(tag in res.best_move_type for tag in _CAST_MOVE_TAGS)
    assert res.board_valid


@pytest.mark.parametrize("label", SCENARIO_IDS)
def test_guidance_scenario_best_prop_improves_metrics(label: str) -> None:
    scenario = next(s for s in get_scenarios() if s.label == label)
    res, guidance = run_scenario(scenario)

    if not guidance.propositions:
        pytest.skip(f"{label}: no propositions")

    if scenario.intentional_overlap:
        assert not res.is_penetrating or "Ejection" in res.best_move_type
        return

    improved = (
        res.kiss_error + 1e-6 < res.seed_kiss_error
        or res.border_standoff_err + 1e-6 < abs(res.seed_kiss_error)
        or (res.board_valid and not res.seed_board_valid)
    )
    assert improved or res.kiss_error <= res.seed_kiss_error + 0.05, (
        f"{label}: best prop did not improve kiss/border/board metrics"
    )
