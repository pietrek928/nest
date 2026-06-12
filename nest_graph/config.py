"""Configuration for nest_graph build / nesting loops."""

import os
from typing import Any, Literal, Optional

import numpy as np
from pydantic import BaseModel, Field
from shapely.geometry import Polygon

from .board import board_sheet_from_outline, board_void_geometries
from .elem_graph import Circle, RuleMutationSettings, Vec2
from .utils import normalize_poly


PLACE_ZONES: tuple[str, ...] = (
    "empty_border",
    "border_gap",
    "interior_pocket",
    "cluster_edge",
    "inter_cluster",
    "void_seek",
)


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
    random_per_iter_when_proposed: int = 64
    """Smaller random pool when structured proposals are present."""
    structured_jitter_per_proposal: int = 12
    """Deterministic (x, y, angle) offsets per proposed transform."""
    structured_jitter_per_proposal_empty: int = 4
    """Small edge-only jitter when border packing (see propose.structured_jitter_border_scale)."""
    structured_jitter_scale: tuple[float, float, float] = (0.06, 0.06, 0.5)
    transform_scale: tuple[float, float, float] = (1.5, 1.5, 2 * np.pi)
    initial_random: int = 256
    selection_expand_n: int = 4
    history_expand_n: int = 4
    history_max: int = 1024
    max_transforms_per_group: Optional[int] = 1200
    shuffle_passes: int = 4
    shuffle_per_pass: int = 48
    shuffle_scale: tuple[float, float, float] = (0.12, 0.12, 0.5)
    seed: Optional[int] = None


class GraphConfig(BaseModel):
    graphs_window: int = 24


class SelectionConfig(BaseModel):
    improve_rules_rounds: int = 4
    rules_kept: int = 64
    improve_rules_elite_count: int = 16
    rule_score_penalty: float = 0.03
    score_rules_latest_graph_only: bool = False
    score_rules_count_weight: float = 0.02
    score_rules_local_swap: bool = True
    select_mode: Literal["weighted_greedy", "greedy_score"] = "weighted_greedy"
    dfs_max_tries: int = 4
    dfs_passes: int = 3
    nest_rule_sets_used: int = 1
    dfs_refine_max_passes: int = 1024
    dfs_refine_max_stagnant_passes: int = 4
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
    ] = "merged_loose_tight"


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
    board_sheet_padding_ratio: float = 0.08
    max_inserts_per_type: int = 4
    max_rules_per_set: int = 24
    use_repulsor_rules: bool = False
    """Seed negative-weight point rules at sheet center to discourage void-center poses."""
    repulsor_weight: float = -0.2
    repulsor_sets_to_touch: int = 4
    """Number of top rule sets to receive repulsor seeds each iteration."""

    def board_polygon(self) -> Polygon:
        """Nest outline (exterior ring). Prefer board_sheet_polygon() for nesting/propose."""
        return Polygon(list(self.board_coords))

    def effective_sheet_padding(self) -> float:
        from .board import default_sheet_padding

        outline = self.board_polygon()
        return default_sheet_padding(
            outline,
            extra=self.board_sheet_padding,
            ratio=self.board_sheet_padding_ratio,
        )

    def board_sheet_polygon(self) -> Polygon:
        outline = Polygon(list(self.board_coords))
        return board_sheet_from_outline(outline, user_holes=self.board_holes)

    def board_void_geometries(self):
        sheet = self.board_sheet_polygon()
        return board_void_geometries(
            sheet,
            outline=self.board_polygon(),
            padding=self.effective_sheet_padding(),
        )

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
    """Perimeter walk + erosion + raycast + voronoi; ranked to max_proposals. See docs/first_pass_tuning.md."""
    max_proposals: int = 40
    candidate_pool: int = 80
    min_dist_ratio: float = 0.002
    first_pass_min_dist_ratio: float = 0.0008
    """Tighter standoff on iteration 1 for denser outline packing."""
    first_pass_clearance_epsilon_ratio: float = 0.02
    first_pass_candidate_pool: int = 96
    first_pass_max_proposals: int = 48
    first_pass_num_angles: int = 28
    first_pass_group_edge_samples_per_edge: int = 64
    first_pass_use_axis_push: bool = True
    first_pass_sequential_augment_max: int = 12
    """Greedy gap-fill steps after saturation (sheet-snap + chain-fit)."""
    first_pass_guidance_refine_passes: int = 3
    """Slide border placements with per-anchor guidance casts for tighter packing."""
    placement_clearance_epsilon_ratio: float = 0.05
    placement_num_angles: int = 18
    use_neighbor_slide: bool = True
    use_axis_push: bool = False
    neighbor_slide_pool_fraction: float = 0.5
    """Share of candidate_pool budget for neighbor_slide (was pool // 4)."""
    obstacle_nearest_k: int = 3
    """Packed clusters used as propose obstacles (graph still checks full layout)."""
    contact_tightness_hybrid_weight: float = 0.15
    """Blend geometric tightness into contact_hybrid ranking."""
    cast_squeeze_top_k: int = 8
    """Post-rank cast squeeze on top-K proposals (0 = off)."""
    cast_squeeze_passes: int = 1
    """Number of cast_squeeze iterations on top-K (2 = double-pass compaction)."""
    use_bottom_left: bool = False
    use_nfp_vertices: bool = False
    bottom_left_vertices_per_angle: int = 8
    raycast_num_rays: int = 12
    raycast_num_angles: int = 12
    raycast_anchor_stride: int = 2
    voronoi_densify_divisor: float = 30.0
    voronoi_num_angles: int = 12
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
    use_border_focus: bool = True
    use_border_edge_seeds: bool = True
    border_focus_ranking: bool = True
    use_contact_ranking: bool = True
    """When packed: rank/trim by tight fit to sheet border or focal group, not deep clearance."""
    use_contact_clearance_hybrid: bool = True
    """Blend contact fit with clearance so valid pocket poses are not discarded."""
    contact_clearance_hybrid_weight: float = 0.25
    use_stratified_contact_trim: bool = True
    contact_trim_fraction: float = 0.8
    ranking_mode: Literal[
        "legacy", "clearance", "hybrid", "border", "contact", "contact_hybrid",
        "rule_hybrid",
    ] = "clearance"
    use_rule_ranking: bool = True
    """When rules are passed to propose, blend rule score into ranking."""
    rule_ranking_weight: float = 0.3
    ranking_clearance_weight: float = 1.0
    ranking_hull_weight: float = 0.1
    group_edge_samples_per_edge: int = 32
    sheet_edge_samples_per_edge: int = 24
    use_guidance_propositions: bool = True
    guidance_max_propositions: int = 8
    guidance_use_tight_packing: bool = True
    guidance_use_corner_alignment: bool = True
    guidance_enable_grid: bool = True
    guidance_diversity_dist_ratio: float = 2.5
    guidance_proposition_seed_count: int = 16
    use_batch_pack: bool = True
    """Place one group, then pack the next against it; add both configs to proposals."""
    batch_pack_anchor_seeds: int = 6
    batch_pack_follow_proposals: int = 10
    batch_pack_follow_pool: int = 32
    batch_pack_max_pairs: int = 16
    use_board_edge_seeds: bool = True
    board_edge_samples_per_edge: int = 64
    structured_jitter_border_scale: tuple[float, float, float] = (0.02, 0.02, 0.35)
    """Tight (x,y) and modest angle jitter for outline snap seeds only."""
    board_edge_guidance_refine: bool = True
    board_edge_guidance_seeds: int = 16
    board_edge_when_packed: bool = True
    """Phase-2 propose: snap along outline with packed obstacles."""
    board_edge_batch_reserve: int = 96
    use_full_packed_obstacle: bool = True
    """When proposing into a partial pack, treat all placed parts as obstacles."""
    first_pass_border_saturation_passes: int = 6
    """Graph rebuild passes before sequential gap-fill."""
    random_per_iter_empty_border: int = 0
    border_selection_score_boost: float = 24.0
    first_pass_empty_border_only: bool = True
    """Empty sheet: only board_edge + sheet_corners proposers (no interior seeds)."""
    first_pass_layered_pack: bool = True
    """Iter 1: rebuild graph with border placements, saturate more outline-kiss nodes."""
    first_pass_border_pack: bool = True
    """Iter 1: pack outline-kiss nodes around nest perimeter before any interior fill."""
    first_pass_interior_max: int = 3
    """Max non-outline parts after border saturate (0 = border-only first pass)."""
    place_profiles_enabled: bool = True
    """Route propose config per sheet zone (border / interior / cluster edge)."""
    late_border_saturation: bool = True
    """Iter 2+: run border saturation propose when outline coverage is low."""
    place_border_coverage_threshold: float = 0.35
    """Below this parts-area/board ratio, prefer border_gap profile."""
    place_free_area_interior_threshold: float = 0.35
    """Free-region area ratio above this → interior_pocket profile."""
    place_proposer_pool_scales: dict[str, float] = Field(default_factory=dict)
    """Rolling feedback scale per proposer name (1.0 = default)."""

    @classmethod
    def proposers_for_place(cls, zone: str) -> frozenset[str] | None:
        sets: dict[str, frozenset[str]] = {
            "empty_border": frozenset({
                "board_edge", "sheet_corners", "perimeter_walk",
            }),
            "border_gap": frozenset({
                "board_edge", "sheet_corners", "perimeter_walk",
                "group_fit", "neighbor_slide", "ribbon_free",
            }),
            "interior_pocket": frozenset({
                "erosion", "voronoi", "ribbon_free", "guidance_propositions",
                "raycasting",
            }),
            "cluster_edge": frozenset({
                "group_fit", "neighbor_slide", "guidance_propositions",
                "perimeter_walk", "erosion",
            }),
            "inter_cluster": frozenset({
                "ribbon_free", "raycasting", "voronoi", "erosion",
            }),
            "void_seek": frozenset({
                "erosion", "voronoi", "ribbon_free", "raycasting",
                "guidance_propositions",
            }),
        }
        return sets.get(zone)

    @classmethod
    def obstacle_scope_for_place(cls, zone: str) -> tuple[bool, int]:
        if zone in ("interior_pocket", "inter_cluster", "void_seek"):
            return True, 0
        if zone == "border_gap":
            return False, 2
        if zone == "cluster_edge":
            return False, 3
        return False, 3

    @classmethod
    def for_place(
        cls,
        zone: str,
        base: "ProposeConfig | None" = None,
        **overrides: Any,
    ) -> "ProposeConfig":
        root = base.model_dump() if base is not None else cls().model_dump()
        profiles: dict[str, dict[str, Any]] = {
            "empty_border": {
                "ranking_mode": "border",
                "border_focus_ranking": True,
                "use_border_focus": True,
                "use_contact_ranking": False,
                "cast_squeeze_top_k": 12,
            },
            "border_gap": {
                "ranking_mode": "border",
                "border_focus_ranking": True,
                "use_contact_ranking": False,
                "use_full_packed_obstacle": False,
                "obstacle_nearest_k": 2,
                "cast_squeeze_top_k": 12,
                "use_board_edge_seeds": True,
            },
            "interior_pocket": {
                "ranking_mode": "contact_hybrid",
                "use_contact_ranking": True,
                "use_contact_clearance_hybrid": True,
                "use_full_packed_obstacle": True,
                "use_border_focus": False,
                "border_focus_ranking": False,
                "cast_squeeze_top_k": 6,
                "use_board_edge_seeds": False,
            },
            "cluster_edge": {
                "use_guidance_propositions": True,
                "guidance_use_tight_packing": True,
                "guidance_use_corner_alignment": True,
                "guidance_enable_grid": False,
                "use_contact_ranking": True,
                "use_contact_clearance_hybrid": True,
                "cast_squeeze_top_k": 8,
                "use_neighbor_slide": False,
                "use_full_packed_obstacle": False,
                "obstacle_nearest_k": 3,
            },
            "inter_cluster": {
                "ranking_mode": "clearance",
                "use_contact_ranking": False,
                "use_full_packed_obstacle": True,
                "use_board_edge_seeds": False,
                "use_group_edge_seeds": False,
                "cast_squeeze_top_k": 4,
                "use_ribbon_seeds": True,
                "use_voronoi": True,
            },
            "void_seek": {
                "ranking_mode": "clearance",
                "use_contact_ranking": False,
                "use_full_packed_obstacle": True,
                "cast_squeeze_top_k": 6,
            },
        }
        patch = dict(profiles.get(zone, {}))
        patch.update(overrides)
        root.update(patch)
        return cls(**root)

    @classmethod
    def local_compact_profile(
        cls,
        *,
        squeeze_k: int = 8,
        use_walk: bool = False,
        **overrides: Any,
    ) -> "ProposeConfig":
        """Benchmark-synthesized local compaction: tight+corner cast + post-rank squeeze."""
        data = cls().model_dump()
        data.update({
            "use_guidance_propositions": True,
            "guidance_use_tight_packing": True,
            "guidance_use_corner_alignment": True,
            "guidance_enable_grid": False,
            "guidance_max_propositions": 8,
            "cast_squeeze_top_k": squeeze_k,
            "cast_squeeze_passes": 1,
            "use_neighbor_slide": False,
            "use_guidance_walk": use_walk,
        })
        data.update(overrides)
        return cls(**data)


class BuildGraphConfig(BaseModel):
    sampling: SamplingConfig = Field(default_factory=SamplingConfig)
    graph: GraphConfig = Field(default_factory=GraphConfig)
    selection: SelectionConfig = Field(default_factory=SelectionConfig)
    rules: RulesConfig = Field(default_factory=RulesConfig)
    propose: ProposeConfig = Field(default_factory=ProposeConfig)
    output: OutputConfig = Field(default_factory=OutputConfig)

    def board_min_dist(self, *, first_pass: bool = False) -> float:
        board = self.rules.board_polygon()
        xmin, ymin, xmax, ymax = board.bounds
        diag = float(np.hypot(xmax - xmin, ymax - ymin))
        ratio = self.propose.min_dist_ratio
        if first_pass:
            ratio = self.propose.first_pass_min_dist_ratio
        return diag * ratio

    def placement_epsilon_ratio(self, *, first_pass: bool = False) -> float:
        if first_pass:
            return self.propose.first_pass_clearance_epsilon_ratio
        return self.propose.placement_clearance_epsilon_ratio

    def first_pass_propose_config(self) -> ProposeConfig:
        p = self.propose.model_copy(deep=True)
        p.candidate_pool = max(p.candidate_pool, p.first_pass_candidate_pool)
        p.max_proposals = max(p.max_proposals, p.first_pass_max_proposals)
        p.placement_num_angles = p.first_pass_num_angles
        p.placement_clearance_epsilon_ratio = p.first_pass_clearance_epsilon_ratio
        p.group_edge_samples_per_edge = p.first_pass_group_edge_samples_per_edge
        p.ranking_mode = "border"
        p.use_axis_push = p.first_pass_use_axis_push
        return p

    @classmethod
    def benchmark_aligned(cls, *, seed: int | None = None) -> "BuildGraphConfig":
        """Sampling/DFS preset used by ``scripts/benchmark_guidance_flow.py``."""
        return cls(
            sampling=SamplingConfig(
                random_per_iter=128,
                random_per_iter_when_proposed=64,
                structured_jitter_per_proposal=12,
                initial_random=256,
                max_transforms_per_group=1200,
                seed=seed,
            ),
            selection=SelectionConfig(dfs_mode="merged_loose_tight"),
            propose=ProposeConfig(),
        )

    @classmethod
    def from_env(cls) -> "BuildGraphConfig":
        sx = _env_float("NEST_TRANSFORM_SX", 1.5)
        sy = _env_float("NEST_TRANSFORM_SY", 1.5)
        sa = _env_float("NEST_TRANSFORM_SA", 2 * np.pi)
        return cls(
            sampling=SamplingConfig(
                random_per_iter=_env_int("NEST_RANDOM_PER_ITER", 128),
                transform_scale=(sx, sy, sa),
                initial_random=_env_int("NEST_INITIAL_RANDOM", 256),
                selection_expand_n=_env_int("NEST_SELECTION_EXPAND_N", 4),
                history_expand_n=_env_int("NEST_HISTORY_EXPAND_N", 4),
                history_max=_env_int("NEST_HISTORY_MAX", 1024),
                max_transforms_per_group=_env_max_transforms(),
                shuffle_passes=_env_int("NEST_SHUFFLE_PASSES", 4),
                shuffle_per_pass=_env_int("NEST_SHUFFLE_PER_PASS", 48),
                seed=_env_optional_int("NEST_SEED"),
            ),
            graph=GraphConfig(
                graphs_window=_env_int("NEST_GRAPHS_WINDOW", 24),
            ),
            selection=SelectionConfig(
                improve_rules_rounds=_env_int("NEST_IMPROVE_ROUNDS", 4),
                rules_kept=_env_int("NEST_RULES_KEPT", 64),
                improve_rules_elite_count=_env_int("NEST_RULES_ELITE", 16),
                rule_score_penalty=_env_float("NEST_RULE_SIZE_PENALTY", 0.03),
                score_rules_latest_graph_only=_env_bool(
                    "NEST_SCORE_RULES_LATEST_ONLY", False,
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
                dfs_refine_max_passes=_env_int("NEST_DFS_REFINE_MAX_PASSES", 1024),
                dfs_refine_max_stagnant_passes=_env_int(
                    "NEST_DFS_REFINE_STAGNANT_PASSES", 4,
                ),
                dfs_refine_beam_width=_env_int("NEST_DFS_REFINE_BEAM", 2),
                dfs_finalize_repair_passes=_env_int("NEST_DFS_FINALIZE_REPAIR", 6),
                dfs_finalize_max_component=_env_int("NEST_DFS_FINALIZE_COMPONENT", 18),
                nest_rule_sets_used=_env_int("NEST_NEST_RULE_SETS", 1),
                dfs_mode=os.environ.get(
                    "NEST_DFS_MODE", "merged_loose_tight",
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


def subsample_transforms_with_pinned(
    transforms: np.ndarray,
    pinned: np.ndarray,
    max_n: Optional[int],
    rng: np.random.Generator,
) -> np.ndarray:
    """Subsample while always keeping ``pinned`` rows (deduped, listed first)."""
    if pinned.shape[0] == 0:
        return subsample_transforms(transforms, max_n, rng)
    pinned = dedupe_transforms(pinned)
    if max_n is None:
        return dedupe_transforms(np.concatenate([pinned, transforms], axis=0))
    if pinned.shape[0] >= max_n:
        return pinned[:max_n]
    cap_rest = max_n - pinned.shape[0]
    if transforms.shape[0] == 0:
        return pinned
    pinned_keys = {
        (round(r[0], 4), round(r[1], 4), round(r[2], 4)) for r in pinned
    }
    rest_rows: list[np.ndarray] = []
    for row in transforms:
        key = (round(float(row[0]), 4), round(float(row[1]), 4), round(float(row[2]), 4))
        if key in pinned_keys:
            continue
        rest_rows.append(row)
    if not rest_rows:
        return pinned
    rest = np.asarray(rest_rows, dtype=np.float64)
    if rest.shape[0] <= cap_rest:
        return dedupe_transforms(np.concatenate([pinned, rest], axis=0))
    idx = rng.choice(rest.shape[0], size=cap_rest, replace=False)
    return dedupe_transforms(np.concatenate([pinned, rest[idx]], axis=0))


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
