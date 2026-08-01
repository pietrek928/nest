"""Render nesting benchmark cases to PNG for visual case review."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon as MplPolygon
from shapely.affinity import translate
from shapely.geometry import Polygon

from nest_graph.config import (
    BuildGraphConfig,
    OutputConfig,
    ProposeConfig,
    SamplingConfig,
    SelectionConfig,
)
from nest_graph.utils import transform_poly
from scripts.nesting_evaluator import NestingPipelineEvaluator
from scripts.nesting_fixtures import NestCase, resolve_cases


GROUP_COLORS = (
    "#4C78A8",
    "#F58518",
    "#54A24B",
    "#E45756",
    "#B279A2",
)
HOLE_FACE = "#3A3A3A"
BOARD_FACE = "#E8E8E8"
SEED_EDGE = "#111111"


def _pad_bounds(bounds, pad_frac: float = 0.06):
    minx, miny, maxx, maxy = bounds
    dx = max(maxx - minx, 1e-6)
    dy = max(maxy - miny, 1e-6)
    pad = pad_frac * max(dx, dy)
    return minx - pad, miny - pad, maxx + pad, maxy + pad


def _add_poly(ax, poly: Polygon, *, face, edge, lw=1.5, alpha=1.0, z=1):
    if poly is None or poly.is_empty:
        return
    ax.add_patch(
        MplPolygon(
            list(poly.exterior.coords),
            closed=True,
            facecolor=face,
            edgecolor=edge,
            linewidth=lw,
            alpha=alpha,
            zorder=z,
        )
    )
    for interior in poly.interiors:
        ax.add_patch(
            MplPolygon(
                list(interior.coords),
                closed=True,
                facecolor=HOLE_FACE,
                edgecolor="#222222",
                linewidth=1.0,
                alpha=0.95,
                zorder=z + 0.1,
            )
        )


def _legend_parts(ax, case: NestCase, origin_xy):
    """Draw part silhouettes beside the board for scale reference."""
    ox, oy = origin_xy
    cursor_y = oy
    gap = 0.12 * max(case.board.bounds[2] - case.board.bounds[0], 1.0)
    for gi, ((poly, _gid), count) in enumerate(zip(case.groups, case.group_counts)):
        minx, miny, maxx, maxy = poly.bounds
        placed = translate(poly, ox - minx, cursor_y - miny)
        color = GROUP_COLORS[gi % len(GROUP_COLORS)]
        _add_poly(ax, placed, face=color, edge="white", lw=1.0, alpha=0.95, z=3)
        ax.text(
            ox + (maxx - minx) + gap * 0.25,
            cursor_y + 0.5 * (maxy - miny),
            f"g{gi} n={count}  A={poly.area:.3g}",
            va="center",
            fontsize=8,
            color=color,
            clip_on=False,
        )
        cursor_y -= (maxy - miny) + gap * 0.45


def _draw_case_setup(ax, case: NestCase):
    board = case.board
    _add_poly(ax, board, face=BOARD_FACE, edge="black", lw=2.0, alpha=1.0, z=1)
    for hole in case.board_holes:
        _add_poly(ax, hole, face=HOLE_FACE, edge="#111111", lw=1.2, alpha=0.95, z=2)

    for poly, gid, _t in case.seed_placements:
        color = GROUP_COLORS[gid % len(GROUP_COLORS)]
        _add_poly(ax, poly, face=color, edge=SEED_EDGE, lw=1.0, alpha=0.85, z=4)

    minx, miny, maxx, maxy = board.bounds
    ax.set_title(
        f"{case.name}\n"
        f"board A={board.area:.3g}  holes={len(case.board_holes)}  "
        f"usable={case.usable_area:.3g}  demand≈{case.demand_ratio:.2f}×  "
        f"seeds={len(case.seed_placements)}",
        fontsize=10,
    )
    # Legend below the board to avoid title overlap.
    _legend_parts(ax, case, (minx, miny - 0.12 * (maxy - miny)))
    tags = ", ".join(sorted(case.tags)) if case.tags else "-"
    ax.text(
        0.01, 0.01, f"tags: {tags}  iters={case.iters}",
        transform=ax.transAxes, fontsize=8, va="bottom", color="#444444",
    )


def _smoke_cfg(seed: int = 0) -> BuildGraphConfig:
    """Modest config: enough to leave first-pass border and fill interior a bit."""
    return BuildGraphConfig(
        sampling=SamplingConfig(
            random_per_iter=48,
            initial_random=48,
            max_transforms_per_group=160,
            history_max=64,
            seed=seed,
        ),
        selection=SelectionConfig(
            improve_rules_rounds=1,
            dfs_mode="merged_loose_tight",
            dfs_passes=2,
            dfs_max_tries=3,
        ),
        propose=ProposeConfig(
            max_proposals=20,
            candidate_pool=32,
            point_cloud_particles=8,
            point_cloud_iterations=8,
            voronoi_max_sites=24,
        ),
        output=OutputConfig(n_iters=1, progress=False),
    )


def _draw_nest_result(ax, case: NestCase, cfg: BuildGraphConfig, seed: int):
    evaluator = NestingPipelineEvaluator(case, cfg)
    metrics = evaluator.run_full_pipeline(seed=seed)
    result = evaluator.last_result or {}
    selected_polys = result.get("selected_polys", [])
    group_id = result.get("group_id", [])
    transform = result.get("transform", np.zeros((0, 3)))

    _add_poly(ax, case.board, face=BOARD_FACE, edge="black", lw=2.0, alpha=1.0, z=1)
    for hole in case.board_holes:
        _add_poly(ax, hole, face=HOLE_FACE, edge="#111111", lw=1.2, alpha=0.95, z=2)

    for i in selected_polys:
        poly = transform_poly(case.groups[group_id[i]][0], transform[i])
        color = GROUP_COLORS[group_id[i] % len(GROUP_COLORS)]
        _add_poly(ax, poly, face=color, edge="white", lw=0.8, alpha=0.9, z=4)

    ax.set_title(
        f"quick nest seed={seed}\n"
        f"parts={metrics.parts_final} (+{metrics.parts_delta})  cov={metrics.area_coverage:.1%} (+{metrics.area_coverage_delta:.1%})  "
        f"kiss={metrics.kiss_fraction:.2f}  "
        f"indep={'ok' if metrics.independent_ok else 'FAIL'}  "
        f"void={'ok' if metrics.void_ok else 'FAIL'}  "
        f"t={metrics.time_s:.1f}s",
        fontsize=10,
    )


def render_case(
    case: NestCase,
    out_path: Path,
    *,
    nest: bool,
    seed: int,
    size: int,
) -> None:
    cols = 2 if nest else 1
    fig, axes = plt.subplots(1, cols, figsize=(6.5 * cols, 6.5))
    if cols == 1:
        axes = [axes]

    _draw_case_setup(axes[0], case)
    bounds = list(case.board.bounds)
    # include legend strip / seeds in view
    for poly, _, _ in case.seed_placements:
        b = poly.bounds
        bounds = [
            min(bounds[0], b[0]),
            min(bounds[1], b[1]),
            max(bounds[2], b[2]),
            max(bounds[3], b[3]),
        ]
    # expand for legend under the board
    dy = bounds[3] - bounds[1]
    bounds[1] -= 0.35 * dy
    minx, miny, maxx, maxy = _pad_bounds(bounds)
    axes[0].set_xlim(minx, maxx)
    axes[0].set_ylim(miny, maxy)
    axes[0].set_aspect("equal")
    axes[0].axis("off")

    if nest:
        cfg = _smoke_cfg(seed=seed)
        case_fast = NestCase(
            name=case.name,
            board=case.board,
            board_holes=case.board_holes,
            groups=case.groups,
            group_counts=case.group_counts,
            seed_placements=case.seed_placements,
            iters=1,
            tags=case.tags,
            floors=case.floors,
        )
        _draw_nest_result(axes[1], case_fast, cfg, seed=seed)
        axes[1].set_xlim(minx, maxx)
        axes[1].set_ylim(miny + 0.25 * dy, maxy)
        axes[1].set_aspect("equal")
        axes[1].axis("off")

    fig.suptitle("nesting case review", fontsize=12, y=0.98)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=max(120, size // 8), bbox_inches="tight", facecolor="white")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", nargs="+", default=["all"])
    parser.add_argument("--out", type=Path, default=Path("docs/nesting_case_viz"))
    parser.add_argument("--nest", action="store_true", help="Also run a quick nest panel")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--size", type=int, default=1024)
    args = parser.parse_args()

    if "all" in args.cases:
        from scripts.nesting_fixtures import get_all_cases
        cases = get_all_cases()
    else:
        cases = resolve_cases(list(args.cases))

    args.out.mkdir(parents=True, exist_ok=True)
    for case in cases:
        out = args.out / f"{case.name}.png"
        print(f"rendering {case.name} -> {out}")
        render_case(case, out, nest=args.nest, seed=args.seed, size=args.size)
    print(f"wrote {len(cases)} png(s) under {args.out}")


if __name__ == "__main__":
    main()
