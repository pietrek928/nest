"""Record GJK-eval / sweep-pair tracer counters per guidance scenario."""

import argparse
from pathlib import Path

from nest_graph.placement_scene import guidance_config_for_propose
from scripts.guidance_placement_fixtures import get_scenarios


def main() -> None:
    from nest_graph.geometry import evaluate_local_placement_traced

    parser = argparse.ArgumentParser()
    parser.add_argument("--scenarios", nargs="+", default=["all"])
    parser.add_argument("--out", type=Path, default=Path("docs/guidance_tracer_bench.txt"))
    args = parser.parse_args()

    scenarios = get_scenarios()
    if "all" not in args.scenarios:
        scenarios = [s for s in scenarios if s.label in args.scenarios]

    lines = [
        "scenario\tsweep_pairs\tcircle_pruned\tgjk_evals\tprops\tpenetrating",
    ]

    for scenario in scenarios:
        cfg = guidance_config_for_propose(
            pt_push=scenario.pt_push,
            min_dist=scenario.min_dist,
            board_bounds=scenario.scene.sheet.bounds,
            target_angle_rad=scenario.seed[2],
            border_focus=scenario.border_focus,
        )
        placed = scenario.scene.placed_at(scenario.seed)
        geoms = scenario.scene.all_geometries(placed)
        guidance, sweep_pairs, circle_pruned, gjk_evals = evaluate_local_placement_traced(
            0,
            geoms,
            (scenario.seed[0], scenario.seed[1]),
            cfg,
        )
        lines.append(
            f"{scenario.label}\t{sweep_pairs}\t{circle_pruned}\t{gjk_evals}\t"
            f"{len(guidance.propositions)}\t{guidance.is_penetrating}"
        )

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(args.out.read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
