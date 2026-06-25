import argparse
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from shapely.affinity import rotate, translate

from nest_graph.placement_scene import tiered_propositions
from scripts.guidance_placement_fixtures import get_scenarios, run_scenario


def draw_board_context(ax, scene, board_outline) -> None:
    if hasattr(scene.sheet, "exterior"):
        sheet_patch = MplPolygon(
            list(scene.sheet.exterior.coords),
            closed=True,
            facecolor="lightgray",
            edgecolor="none",
            alpha=0.5,
        )
        ax.add_patch(sheet_patch)
        for interior in scene.sheet.interiors:
            hole_patch = MplPolygon(
                list(interior.coords),
                closed=True,
                facecolor="none",
                edgecolor="red",
                linestyle="--",
                hatch="//",
            )
            ax.add_patch(hole_patch)

    if hasattr(board_outline, "exterior"):
        outline_patch = MplPolygon(
            list(board_outline.exterior.coords),
            closed=True,
            facecolor="none",
            edgecolor="black",
            linewidth=2,
        )
        ax.add_patch(outline_patch)


def geometry_to_patch(shapely_poly, **kwargs):
    return MplPolygon(list(shapely_poly.exterior.coords), closed=True, **kwargs)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenarios", nargs="+", default=["all"])
    parser.add_argument("--out", type=str, default="docs/guidance_viz/")
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--write-summary", action="store_true", default=True)
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    all_scenarios = get_scenarios()
    if "all" not in args.scenarios:
        all_scenarios = [s for s in all_scenarios if s.label in args.scenarios]

    summary_lines = [
        "| scenario | shapes | props | penetrating | best_move | kiss_err | border_err | board_ok | exec_ms |",
        "|----------|--------|-------|-------------|-----------|----------|------------|----------|---------|",
    ]

    for scenario in all_scenarios:
        res, guidance = run_scenario(scenario)

        summary_lines.append(
            f"| {res.scenario_id} | {res.shapes} | {res.props_count} | {res.is_penetrating} "
            f"| {res.best_move_type} | {res.kiss_error:.4f} | {res.border_standoff_err:.4f} "
            f"| {res.board_valid} | {res.execution_time_ms:.3f} |"
        )

        fig, ax = plt.subplots(figsize=(8, 8))
        draw_board_context(ax, scenario.scene, scenario.board_outline)

        for obs in scenario.obstacles_shapely:
            ax.add_patch(
                geometry_to_patch(obs, facecolor="blue", edgecolor="white", alpha=0.8),
            )

        seed = scenario.seed
        seed_poly = rotate(
            scenario.part_shapely, seed[2], use_radians=True, origin=(0, 0),
        )
        seed_poly = translate(seed_poly, seed[0], seed[1])
        ax.add_patch(
            geometry_to_patch(
                seed_poly,
                facecolor="none",
                edgecolor="orange",
                linestyle="--",
                linewidth=2,
            ),
        )

        props = tiered_propositions(guidance)
        if props:
            origins_x = [seed[0]] * len(props)
            origins_y = [seed[1]] * len(props)
            vecs_x = [p.translation[0] for p in props]
            vecs_y = [p.translation[1] for p in props]
            ax.quiver(
                origins_x,
                origins_y,
                vecs_x,
                vecs_y,
                color="purple",
                angles="xy",
                scale_units="xy",
                scale=1,
                alpha=0.5,
            )

            best_p = props[0]
            best_poly = rotate(
                scenario.part_shapely,
                seed[2] + best_p.rotation_rad,
                use_radians=True,
                origin=(0, 0),
            )
            best_poly = translate(
                best_poly,
                seed[0] + best_p.translation[0],
                seed[1] + best_p.translation[1],
            )
            ax.add_patch(
                geometry_to_patch(
                    best_poly,
                    facecolor="green",
                    edgecolor="darkgreen",
                    alpha=0.6,
                ),
            )

        info_text = (
            f"Move: {res.best_move_type}\n"
            f"Kiss Err: {res.kiss_error:.4f}\n"
            f"Board Valid: {res.board_valid}\n"
            f"Exec: {res.execution_time_ms:.3f} ms"
        )
        ax.text(
            0.05,
            0.95,
            info_text,
            transform=ax.transAxes,
            fontsize=10,
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
        )

        ax.set_title(scenario.label)
        ax.autoscale_view()
        ax.set_aspect("equal")

        plt.savefig(out_dir / f"{scenario.label}.png", dpi=150, bbox_inches="tight")
        if args.show:
            plt.show()
        plt.close(fig)

    summary_text = "\n".join(summary_lines)
    print(summary_text)

    if args.write_summary:
        (out_dir / "summary.md").write_text(summary_text + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
