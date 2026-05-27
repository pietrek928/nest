"""Configuration for nest_graph build / nesting loops."""

import os
from typing import Literal, Optional

import numpy as np
from pydantic import BaseModel, Field
from shapely.geometry import Polygon

from .board import board_sheet_from_outline, board_void_geometries
from .elem_graph import Circle, RuleMutationSettings, Vec2
from .utils import normalize_poly


def _env_int(key: str, default: int) -> int:
    raw = os.environ.get(key)
    if raw is None or raw == "":
        return default
    return int(raw)


def _env_float(key: str, default: float) -> float:
    raw = os.environ.get(key)
    if raw is None or raw == "":
        return default
    return float(raw)


def _env_bool(key: str, default: bool) -> bool:
    raw = os.environ.get(key)
    if raw is None or raw == "":
        return default
    return raw.strip().lower() in ("1", "true", "yes", "on")


def _env_optional_int(key: str) -> Optional[int]:
    raw = os.environ.get(key)
    if raw is None or raw == "":
        return None
    return int(raw)


def _env_max_transforms() -> Optional[int]:
    """Unset -> 600; none/off/0 -> no cap."""
    raw = os.environ.get("NEST_MAX_TRANSFORMS")
    if raw is None or raw == "":
        return 900
    if raw.strip().lower() in ("none", "off", "0", "-1"):
        return None
    return int(raw)


class SamplingConfig(BaseModel):
    random_per_iter: int = 128
    """Uniform random transforms per group per iteration when propose is off."""
    random_per_iter_when_proposed: int = 48
    """Smaller random pool when structured proposals are present."""
    structured_jitter_per_proposal: int = 8
    """Deterministic (x, y, angle) offsets per proposed transform."""
    structured_jitter_scale: tuple[float, float, float] = (0.06, 0.06, 0.35)
    transform_scale: tuple[float, float, float] = (1.5, 1.5, 2 * np.pi)
    initial_random: int = 256
    selection_expand_n: int = 4
    history_expand_n: int = 2
    history_max: int = 512
    max_transforms_per_group: Optional[int] = 900
    shuffle_passes: int = 2
    shuffle_per_pass: int = 32
    shuffle_scale: tuple[float, float, float] = (0.12, 0.12, 0.5)
    seed: Optional[int] = None


class GraphConfig(BaseModel):
    graphs_window: int = 12


class SelectionConfig(BaseModel):
    improve_rules_rounds: int = 2
    rules_kept: int = 64
    improve_rules_elite_count: int = 16
    rule_score_penalty: float = 0.03
    score_rules_latest_graph_only: bool = True
    score_rules_count_weight: float = 0.02
    score_rules_local_swap: bool = True
    select_mode: Literal["weighted_greedy", "greedy_score"] = "weighted_greedy"
    dfs_max_tries: int = 4
    dfs_passes: int = 3
    nest_rule_sets_used: int = 1
    dfs_refine_max_passes: int = 12
    dfs_refine_beam_width: int = 2
    dfs_finalize_repair_passes: int = 6
    dfs_finalize_max_component: int = 18
    dfs_mode: Literal[
        "nest_only",
        "head_pipeline",
        "strict_no_prune",
        "strict_prune",
        "legacy_alternating",
        "merged_loose_tight",
        "merged_loose_finalize_end",
        "merged_loose_tight_finalize_end",
        "merged_single_pass",
        "high_pass_loose",
    ] = "merged_loose_finalize_end"


class RulesConfig(BaseModel):
    board_coords: tuple[tuple[float, float], ...] = (
        (0.0, 0.0),
        (1.2, 0.0),
        (0.0, 1.1),
    )
    rect_coords: tuple[tuple[float, float], ...] = (
        (0.0, 0.0),
        (0.1, 0.0),
        (0.1, 0.1),
        (0.0, 0.1),
    )
    tri_coords: tuple[tuple[float, float], ...] = (
        (0.0, 0.0),
        (0.15, 0.0),
        (0.0, 0.07),
    )
    place_rule_radius: float = 0.2
    weight_rect: float = 1.0
    weight_tri: float = 1.0
    angle_rule_weight_scale: float = 0.1
    ngroups: int = 2
    board_holes: tuple[tuple[tuple[float, float], ...], ...] = ()
    board_sheet_padding: float = 0.0
    max_inserts_per_type: int = 2
    max_rules_per_set: int = 24

    def board_polygon(self) -> Polygon:
        """Nest outline (exterior ring). Prefer board_sheet_polygon() for nesting/propose."""
        return Polygon(list(self.board_coords))

    def board_sheet_polygon(self) -> Polygon:
        outline = Polygon(list(self.board_coords))
        return board_sheet_from_outline(
            outline,
            padding=self.board_sheet_padding,
            user_holes=self.board_holes,
        )

    def board_void_geometries(self):
        from .geometry import Geometry

        sheet = self.board_sheet_polygon()
        return board_void_geometries(sheet)

    def rect_polygon(self) -> Polygon:
        return normalize_poly(Polygon(list(self.rect_coords)))

    def tri_polygon(self) -> Polygon:
        return normalize_poly(Polygon(list(self.tri_coords)))

    def rule_region(self) -> Circle:
        board = self.board_polygon()
        xmin, ymin, xmax, ymax = board.bounds
        return Circle.from_bounds(xmin, ymin, xmax, ymax)

    def mutation_presets(self) -> list[RuleMutationSettings]:
        region = self.rule_region()
        ng = self.ngroups
        cap = self.max_inserts_per_type
        return [
            _make_rule_mutation_settings(
                region=region,
                dpos=0.25,
                dw=0.25,
                da=np.pi / 4,
                insert_p=0.09,
                remove_p=0.02,
                mutate_p=0.1,
                ngroups=ng,
                max_inserts_per_type=cap,
            ),
            _make_rule_mutation_settings(
                region=region,
                dpos=0.05,
                dw=0.05,
                da=np.pi / 32,
                insert_p=0.04,
                remove_p=0.01,
                mutate_p=0.1,
                ngroups=ng,
                max_inserts_per_type=cap,
            ),
            _make_rule_mutation_settings(
                region=region,
                dpos=0.01,
                dw=0.01,
                da=np.pi / 64,
                insert_p=0.01,
                remove_p=0.02,
                mutate_p=0.1,
                ngroups=ng,
                max_inserts_per_type=cap,
            ),
        ]


class OutputConfig(BaseModel):
    n_iters: int = 256
    video_path: str = "test.mp4"
    snapshot_path: str = "test.jpg"
    video_fps: int = 5
    render_size: int = 1024
    progress: bool = True


class ProposeConfig(BaseModel):
    """Erosion + raycast + voronoi (light); ranked to max_proposals. See docs/first_pass_tuning.md."""
    max_proposals: int = 24
    candidate_pool: int = 48
    min_dist_ratio: float = 0.002
    placement_clearance_epsilon_ratio: float = 0.05
    erosion_num_angles: int = 12
    raycast_num_rays: int = 12
    raycast_num_angles: int = 12
    raycast_anchor_stride: int = 3
    voronoi_densify_divisor: float = 30.0
    voronoi_num_angles: int = 6
    voronoi_max_sites: int = 48
    point_cloud_particles: int = 12
    point_cloud_iterations: int = 16
    point_cloud_nudge_iters: int = 4
    point_cloud_ray_dirs: int = 8
    point_cloud_cull_ratio: float = 0.25
    use_voronoi: bool = True
    use_point_cloud: bool = False
    use_guidance_walk: bool = False
    use_free_region_search: bool = True
    smart_push_target: bool = True
    trim_candidates_by_clearance: bool = True
    use_ribbon_seeds: bool = True
    use_group_edge_seeds: bool = True
    multi_site_erosion: bool = True
    use_border_focus: bool = True
    use_border_edge_seeds: bool = True
    border_focus_ranking: bool = True
    use_contact_ranking: bool = True
    """When packed: rank/trim by tight fit to sheet border or focal group, not deep clearance."""
    use_contact_clearance_hybrid: bool = True
    """Blend contact fit with clearance so valid pocket poses are not discarded."""
    contact_clearance_hybrid_weight: float = 0.2
    use_stratified_contact_trim: bool = True
    contact_trim_fraction: float = 0.65
    ranking_mode: Literal[
        "legacy", "clearance", "hybrid", "border", "contact", "contact_hybrid",
    ] = "clearance"
    ranking_clearance_weight: float = 1.0
    ranking_hull_weight: float = 0.1
    group_edge_samples_per_edge: int = 24
    sheet_edge_samples_per_edge: int = 24


class BuildGraphConfig(BaseModel):
    sampling: SamplingConfig = Field(default_factory=SamplingConfig)
    graph: GraphConfig = Field(default_factory=GraphConfig)
    selection: SelectionConfig = Field(default_factory=SelectionConfig)
    rules: RulesConfig = Field(default_factory=RulesConfig)
    propose: ProposeConfig = Field(default_factory=ProposeConfig)
    output: OutputConfig = Field(default_factory=OutputConfig)

    def board_min_dist(self) -> float:
        board = self.rules.board_polygon()
        xmin, ymin, xmax, ymax = board.bounds
        diag = float(np.hypot(xmax - xmin, ymax - ymin))
        return diag * self.propose.min_dist_ratio

    @classmethod
    def from_env(cls) -> "BuildGraphConfig":
        sx = _env_float("NEST_TRANSFORM_SX", 1.5)
        sy = _env_float("NEST_TRANSFORM_SY", 1.5)
        sa = _env_float("NEST_TRANSFORM_SA", 2 * np.pi)
        return cls(
            sampling=SamplingConfig(
                random_per_iter=_env_int("NEST_RANDOM_PER_ITER", 128),
                transform_scale=(sx, sy, sa),
                initial_random=_env_int("NEST_INITIAL_RANDOM", 128),
                selection_expand_n=_env_int("NEST_SELECTION_EXPAND_N", 4),
                history_expand_n=_env_int("NEST_HISTORY_EXPAND_N", 2),
                history_max=_env_int("NEST_HISTORY_MAX", 512),
                max_transforms_per_group=_env_max_transforms(),
                shuffle_passes=_env_int("NEST_SHUFFLE_PASSES", 2),
                shuffle_per_pass=_env_int("NEST_SHUFFLE_PER_PASS", 32),
                seed=_env_optional_int("NEST_SEED"),
            ),
            graph=GraphConfig(
                graphs_window=_env_int("NEST_GRAPHS_WINDOW", 12),
            ),
            selection=SelectionConfig(
                improve_rules_rounds=_env_int("NEST_IMPROVE_ROUNDS", 2),
                rules_kept=_env_int("NEST_RULES_KEPT", 64),
                improve_rules_elite_count=_env_int("NEST_RULES_ELITE", 16),
                rule_score_penalty=_env_float("NEST_RULE_SIZE_PENALTY", 0.03),
                score_rules_latest_graph_only=_env_bool(
                    "NEST_SCORE_RULES_LATEST_ONLY", True,
                ),
                score_rules_count_weight=_env_float(
                    "NEST_SCORE_RULES_COUNT_WEIGHT", 0.02,
                ),
                score_rules_local_swap=_env_bool(
                    "NEST_SCORE_RULES_LOCAL_SWAP", True,
                ),
                select_mode=os.environ.get(
                    "NEST_SELECT_MODE", "weighted_greedy",
                ),
                dfs_max_tries=_env_int("NEST_DFS_MAX_TRIES", 4),
                dfs_passes=_env_int("NEST_DFS_PASSES", 3),
                dfs_refine_max_passes=_env_int("NEST_DFS_REFINE_MAX_PASSES", 12),
                dfs_refine_beam_width=_env_int("NEST_DFS_REFINE_BEAM", 2),
                dfs_finalize_repair_passes=_env_int("NEST_DFS_FINALIZE_REPAIR", 6),
                dfs_finalize_max_component=_env_int("NEST_DFS_FINALIZE_COMPONENT", 18),
                nest_rule_sets_used=_env_int("NEST_NEST_RULE_SETS", 1),
                dfs_mode=os.environ.get(
                    "NEST_DFS_MODE", "merged_loose_tight_finalize_end",
                ),
            ),
            output=OutputConfig(
                n_iters=_env_int("NEST_BUILD_GRAPH_ITERS", 256),
                video_path=os.environ.get("NEST_VIDEO_PATH", "test.mp4"),
                snapshot_path=os.environ.get("NEST_SNAPSHOT_PATH", "test.jpg"),
                video_fps=_env_int("NEST_VIDEO_FPS", 5),
                render_size=_env_int("NEST_RENDER_SIZE", 1024),
                progress=_env_bool("NEST_PROGRESS", True),
            ),
        )

    def apply_seed(self, rng: Optional[np.random.Generator] = None) -> np.random.Generator:
        if self.sampling.seed is not None:
            return np.random.default_rng(self.sampling.seed)
        return rng if rng is not None else np.random.default_rng()


def dedupe_transforms(transforms: np.ndarray) -> np.ndarray:
    if transforms.shape[0] == 0:
        return transforms
    return np.unique(transforms, axis=0)


def shuffle_transforms(
    transforms: np.ndarray,
    rng: np.random.Generator,
) -> np.ndarray:
    if transforms.shape[0] <= 1:
        return transforms
    out = transforms.copy()
    rng.shuffle(out)
    return out


def subsample_transforms(
    transforms: np.ndarray,
    max_n: Optional[int],
    rng: np.random.Generator,
) -> np.ndarray:
    if max_n is None or transforms.shape[0] <= max_n:
        return transforms
    idx = rng.choice(transforms.shape[0], size=max_n, replace=False)
    return transforms[idx]


def expand_structured_transforms(
    proposals: np.ndarray,
    jitter_scale: tuple[float, float, float],
    n_jitter: int,
) -> np.ndarray:
    """Deterministic small perturbations around each proposed placement."""
    if proposals.shape[0] == 0 or n_jitter <= 0:
        return np.zeros((0, 3), dtype=np.float64)
    sx, sy, sa = jitter_scale
    n_side = max(2, int(round(n_jitter**0.5)))
    xy = np.linspace(-1.0, 1.0, n_side)
    da = np.linspace(-1.0, 1.0, max(2, n_jitter // n_side))
    rows: list[np.ndarray] = []
    for t in proposals:
        for dx in xy:
            for dy in xy:
                for a in da:
                    rows.append(t + np.array([dx * sx, dy * sy, a * sa]))
                    if len(rows) >= proposals.shape[0] * n_jitter:
                        break
                if len(rows) >= proposals.shape[0] * n_jitter:
                    break
            if len(rows) >= proposals.shape[0] * n_jitter:
                break
    if not rows:
        return np.zeros((0, 3), dtype=np.float64)
    return np.asarray(rows, dtype=np.float64)


def _make_rule_mutation_settings(
    *,
    region: Circle,
    dpos: float,
    dw: float,
    da: float,
    insert_p: float,
    remove_p: float,
    mutate_p: float,
    ngroups: int,
    max_inserts_per_type: int = 2,
) -> RuleMutationSettings:
    preset = RuleMutationSettings(
        region=region,
        dpos=dpos,
        dw=dw,
        da=da,
        insert_p=insert_p,
        remove_p=remove_p,
        mutate_p=mutate_p,
        ngroups=ngroups,
    )
    preset.max_inserts_per_type = max_inserts_per_type
    return preset


def _make_select_options(mode: str, local_swap: bool, aggregation: str = "sum"):
    """Build SelectOptions for elem_graph tests and benchmarks."""
    from nest_graph.elem_graph import ScoreAggregation, SelectMode, SelectOptions

    opts = SelectOptions()
    if mode == "weighted_greedy":
        opts.mode = SelectMode.WeightedGreedy
    else:
        opts.mode = SelectMode.GreedyScore
    opts.local_swap = local_swap
    opts.aggregation = (
        ScoreAggregation.Sum if aggregation == "sum" else ScoreAggregation.Max
    )
    return opts


def score_rules_options(sel: SelectionConfig):
    """ScoreRulesOptions aligned with nest_by_graph selection."""
    from nest_graph.elem_graph import ScoreRulesOptions

    opts = ScoreRulesOptions()
    opts.latest_graph_only = sel.score_rules_latest_graph_only
    opts.count_weight = sel.score_rules_count_weight
    opts.rule_complexity_penalty = sel.rule_score_penalty
    opts.select = _make_select_options(
        sel.select_mode,
        sel.score_rules_local_swap,
    )
    return opts


def trim_history(
    history: np.ndarray,
    selected: np.ndarray,
    history_max: int,
) -> np.ndarray:
    if selected.shape[0] == 0:
        return history
    merged = np.unique(np.concatenate([selected, history], axis=0), axis=0)
    if history_max > 0 and merged.shape[0] > history_max:
        return merged[-history_max:, :]
    return merged
