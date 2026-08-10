"""Configuration for nest_graph build / nesting loops."""

import os
from enum import StrEnum
from typing import Any, Optional, Sequence

import numpy as np
from pydantic import BaseModel, Field
from shapely.geometry import Polygon

from .board import board_sheet_from_outline, board_void_geometries, default_sheet_padding
from .elem_graph import (
    Circle,
    RuleMutationSettings,
    ScoreAggregation,
    SelectMode,
    SelectOptions,
    ScoreRulesOptions,
)
from .geometry import PlacementRankMode
from .proposer_names import (
    PLACE_ZONES,
    PROPOSER_FLAG,
    PlaceZone,
    ProposerName,
    ZONE_PROPOSERS,
    proposers_for_zone,
)
from .utils import normalize_poly


class ScoreSelectMode(StrEnum):
    WEIGHTED_GREEDY = "weighted_greedy"
    GREEDY_SCORE = "greedy_score"


class RankingMode(StrEnum):
    LEGACY = "legacy"
    CLEARANCE = "clearance"
    HYBRID = "hybrid"
    BORDER = "border"
    CONTACT = "contact"
    CONTACT_HYBRID = "contact_hybrid"
    RULE_HYBRID = "rule_hybrid"


class DfsMode(StrEnum):
    NEST_ONLY = "nest_only"
    HEAD_PIPELINE = "head_pipeline"
    STRICT_NO_PRUNE = "strict_no_prune"
    STRICT_PRUNE = "strict_prune"
    LEGACY_ALTERNATING = "legacy_alternating"
    MERGED_LOOSE_TIGHT = "merged_loose_tight"
    MERGED_LOOSE_FINALIZE_END = "merged_loose_finalize_end"
    MERGED_LOOSE_TIGHT_FINALIZE_END = "merged_loose_tight_finalize_end"
    MERGED_SINGLE_PASS = "merged_single_pass"
    HIGH_PASS_LOOSE = "high_pass_loose"


def _coerce_nb_enum(enum_type: type, value: Any):
    """Accept nb enum member, name string, or int."""
    if isinstance(value, enum_type):
        return value
    if isinstance(value, str):
        key = value.strip()
        members = enum_type.__members__
        if key in members:
            return members[key]
        raise ValueError(f"invalid {enum_type.__name__}: {value!r}")
    if isinstance(value, int):
        return enum_type(value)
    raise TypeError(f"expected {enum_type.__name__}, got {type(value).__name__}")


def _coerce_select_mode(value: Any) -> SelectMode:
    """Map config ScoreSelectMode|str → nanobind SelectMode."""
    if isinstance(value, SelectMode):
        return value
    if isinstance(value, ScoreSelectMode):
        return _coerce_nb_enum(SelectMode, value.value)
    if isinstance(value, str):
        return _coerce_nb_enum(SelectMode, ScoreSelectMode(value).value)
    return _coerce_nb_enum(SelectMode, value)


def _coerce_ranking_mode(value: Any) -> PlacementRankMode:
    """Map config RankingMode|str → nanobind PlacementRankMode."""
    if isinstance(value, PlacementRankMode):
        return value
    if isinstance(value, RankingMode):
        return _coerce_nb_enum(PlacementRankMode, value.value)
    if isinstance(value, str):
        return _coerce_nb_enum(PlacementRankMode, RankingMode(value).value)
    return _coerce_nb_enum(PlacementRankMode, value)


def as_ranking_mode(value: Any) -> PlacementRankMode:
    """Map config/propose RankingMode|str → nanobind PlacementRankMode."""
    return _coerce_ranking_mode(value)


class ProposeAblation(StrEnum):
    """Benchmark / ablation presets applied on top of BuildGraphConfig.propose."""

    SHIPPED = "shipped"
    LOCAL_COMPACT = "local_compact"
    NO_PROPOSE = "no_propose"
    NO_VOID_BOOST = "no_void_boost"
    VOID_BOOST_HIGH = "void_boost_high"
    NO_VOID_YIELD_DENSIFY = "no_void_yield_densify"
    NO_VOID_POLE_CLEAR = "no_void_pole_clear"
    NO_FREE_SPACE_CLOUD = "no_free_space_cloud"
    NO_VOID_HIJACK = "no_void_hijack"
    NO_OVERRIDE = "no_override"
    NO_GREEDY_NEST = "no_greedy_nest"
    NO_POCKET_FIT = "no_pocket_fit"
    NO_SMALL_PREFER = "no_small_prefer"
    NO_POCKET_MIS_BOOST = "no_pocket_mis_boost"
    NO_CLUSTER_REPACK = "no_cluster_repack"
    NO_CLUSTER_COPY = "no_cluster_copy"
    NO_SIDE_PACK = "no_side_pack"
    NO_OPEN_VOID_POCKET = "no_open_void_pocket"
    NO_LOCAL_SE2 = "no_local_se2"
    NO_CLUSTER_RELOCATE = "no_cluster_relocate"
    NO_DENSIFY_HIJACK = "no_densify_hijack"
    NO_VOID_ELITE = "no_void_elite"
    NO_KEEP_HIST_STERILE = "no_keep_hist_sterile"
    NO_UNIFIED_RESERVE = "no_unified_reserve"
    NO_VOID_CONTACT_HYBRID = "no_void_contact_hybrid"
    NO_MOTIF_TOPO_ANCHORS = "no_motif_topo_anchors"
    NO_VOID_RANK = "no_void_rank"
    VOID_PSO = "void_pso"
    LEAN_VOID_COMBO = "lean_void_combo"
    CASCADE_ONLY = "cascade_only"
    NMS_ONLY = "nms_only"
    CONFLICT_DEGREE_RANK = "conflict_degree_rank"
    MULTI_POLE_VOID = "multi_pole_void"
    EMA_SCALES = "ema_scales"
    NO_EDGE_FREE = "no_edge_free"
    NO_EDGE_FREE_RANK = "no_edge_free_rank"
    NO_SELECTION_GEOM = "no_selection_geom"
    NEST_BY_GRAPH_ONLY = "nest_by_graph_only"
    GREEDY_NEST_ONLY = "greedy_nest_only"


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
    """Unset -> SamplingConfig default (5000); none/off/0 -> no cap."""
    raw = os.environ.get("NEST_MAX_TRANSFORMS")
    if raw is None or raw == "":
        return 5000
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
    max_transforms_per_group: Optional[int] = 5000
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
    select_mode: ScoreSelectMode = ScoreSelectMode.WEIGHTED_GREEDY
    dfs_max_tries: int = 4
    dfs_passes: int = 3
    nest_rule_sets_used: int = 1
    dfs_refine_max_passes: int = 1024
    dfs_refine_max_stagnant_passes: int = 4
    dfs_refine_beam_width: int = 2
    dfs_finalize_repair_passes: int = 6
    dfs_finalize_max_component: int = 18
    dfs_mode: DfsMode = DfsMode.MERGED_LOOSE_TIGHT


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



def floor_void_seek_budgets(cfg: "ProposeConfig") -> "ProposeConfig":
    """Ensure void_seek propose budgets are not lean-/scale-capped below root."""
    pool_floor, prop_floor = ProposeConfig.void_seek_budget_floors()
    pool = max(int(cfg.candidate_pool), pool_floor)
    props = max(int(cfg.max_proposals), prop_floor)
    if pool == int(cfg.candidate_pool) and props == int(cfg.max_proposals):
        return cfg
    return cfg.model_copy(update={
        "candidate_pool": pool,
        "max_proposals": props,
    })


class ProposeConfig(BaseModel):
    """Perimeter walk + erosion + raycast + voronoi; ranked to max_proposals. See docs/first_pass_tuning.md."""
    max_proposals: int = 512
    candidate_pool: int = 1024
    min_dist_ratio: float = 0.002
    first_pass_min_dist_ratio: float = 0.0008
    """Tighter standoff on iteration 1 for denser outline packing."""
    first_pass_clearance_epsilon_ratio: float = 0.02
    first_pass_candidate_pool: int = 256
    first_pass_max_proposals: int = 128
    first_pass_num_angles: int = 28
    first_pass_group_edge_samples_per_edge: int = 32
    first_pass_sequential_augment_max: int = 12
    """Greedy gap-fill steps after saturation (sheet-snap + chain-fit)."""
    first_pass_guidance_refine_passes: int = 3
    """Slide border placements with per-anchor guidance casts for tighter packing."""
    placement_clearance_epsilon_ratio: float = 0.05
    placement_num_angles: int = 18
    use_neighbor_slide: bool = False
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
    raycast_num_rays: int = 12
    raycast_num_angles: int = 12
    raycast_anchor_stride: int = 2
    voronoi_densify_divisor: float = 30.0
    voronoi_num_angles: int = 12
    voronoi_max_sites: int = 32
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
    contact_tightness_hybrid_weight: float = 0.15
    edge_free_weight: float = 6.0
    """Harmonic dual-proximity (board+pack) term in contact_hybrid rank_score / quality."""
    edge_free_band_min_dist_mult: float = 3.5
    """Band = mult * min_dist for edge_free near_* and capped clearance."""
    selection_geom_weight: float = 24.0
    """Scale non-negative C++ quality into nest/DFS scores (0 disables; replaces border boost)."""
    use_stratified_contact_trim: bool = True
    contact_trim_fraction: float = 0.8
    ranking_mode: RankingMode = RankingMode.CLEARANCE
    use_rule_ranking: bool = True
    """When rules are passed to propose, blend rule score into ranking."""
    rule_ranking_weight: float = 0.3
    ranking_clearance_weight: float = 1.0
    ranking_hull_weight: float = 0.1
    group_edge_samples_per_edge: int = 16
    sheet_edge_samples_per_edge: int = 16
    use_guidance_propositions: bool = True
    guidance_max_propositions: int = 8
    guidance_use_tight_packing: bool = True
    guidance_use_corner_alignment: bool = True
    guidance_enable_grid: bool = False
    guidance_diversity_dist_ratio: float = 2.5
    guidance_proposition_seed_count: int = 16
    guidance_cast_refine_top_k: int = 8
    """Unified cast-refine pass on top coarse seeds after all proposers."""
    cast_rank_boost: float = 0.35
    """Clearance score boost (× min_dist) for cast-snap ranked candidates."""
    use_batch_pack: bool = True
    """Place one group, then pack the next against it; add both configs to proposals."""
    batch_pack_anchor_seeds: int = 4
    batch_pack_follow_proposals: int = 6
    batch_pack_follow_pool: int = 24
    batch_pack_max_pairs: int = 12
    use_cluster_copy: bool = True
    """Rigid-copy packed cluster motifs into free pockets / between clusters."""
    cluster_copy_max_patterns: int = 2
    cluster_copy_anchor_seeds: int = 6
    cluster_copy_min_members: int = 2
    use_pocket_fit: bool = True
    """Teleport into trapped voids / hull bays with MRR/triangle align + motif holes."""
    use_open_void_pocket: bool = True
    """When large_void touches sheet exterior, also emit pocket teleports into that free region."""
    pocket_fit_max_targets: int = 8
    pocket_fit_area_ratio: float = 0.5
    """Minimum void.area / part.area to consider a pocket."""
    pocket_fit_reserve_fraction: float = 0.15
    """Fraction of max_proposals reserved for pocket_fit via reserve_coords."""
    enable_cluster_repack: bool = True
    """Post-DFS: unlock a void-adjacent contact cluster and re-propose into kept+void."""
    cluster_repack_min_size: int = 3
    cluster_repack_max_size: int = 6
    cluster_repack_area_accept_ratio: float = 0.98
    """Accept repack if new selection area >= ratio * old area."""
    cluster_repack_max_attempts: int = 1
    enable_cluster_relocate: bool = True
    """Post-DFS: rigid ΔT translate floating (non-exterior) islands toward void pole."""
    enable_local_se2: bool = True
    """Post-DFS: local SE(2) polish toward void pole via native polish_se2_part."""
    local_se2_n_angles: int = 4
    local_se2_max_coarse_steps: int = 12
    local_se2_max_fine_steps: int = 24
    use_board_edge_seeds: bool = True
    board_edge_samples_per_edge: int = 24
    use_side_pack: bool = True
    """Anti-crowd sheet-side snap (XOR board_edge when packed; XOR group_fit under void)."""
    side_pack_top_n: int = 256
    side_pack_samples_per_edge: int = 16
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
    """Below this outline-coverage ratio, prefer late border-only propose."""
    late_border_void_override_ratio: float = 2.5
    """If largest free / mean part area exceeds this, skip late border sat (0 disables)."""
    late_border_void_release_ratio: float = 1.5
    """After override fires, keep unlock while void ratio exceeds this (0 = no hysteresis)."""
    late_border_hull_threshold: float = 0.4
    """hull_rim_fill: keep late sat while pack_hull_perim / sheet_perim is below this (when not large_void)."""
    place_proposer_pool_scales: dict[str, float] = Field(default_factory=dict)
    """Rolling feedback scale per proposer name (1.0 = default)."""
    enable_gravity_compaction: bool = True
    """Floater pole SE(2) via local_se2 (requires void pole; no corner fallback)."""
    void_island_score_boost: float = 64.0
    """EMS pole weight for continuous distance-to-pole DFS score boost (0 disables)."""
    void_attractor_rule_weight: float = 16.0
    """Lighter PointPlaceRule weight for nest_by_graph void attractors (0 disables rules)."""
    void_greedy_nest_seed: bool = True
    """When large_void + any selection boost: nest seed = score-ordered greedy MIS."""
    enable_void_large_hijack: bool = True
    """Mode A: force void_seek + polylabel seeds when largest free / part_area > late_border_void_override_ratio."""
    pocket_score_boost: float = 50.0
    """Add to DFS/nest scores for graph nodes matching pocket/motif propose keys (0 disables)."""
    small_part_void_score_boost: float = 40.0
    """On large_void: boost smaller catalog groups via (1 - area/max_area) * weight (0 disables)."""
    densify_clearance_floor_ratio: float = 1.0
    """Densify if best clearance score < this × min_dist (quality gate; 0 = count-only)."""
    densify_on_void_hijack: bool = True
    """Fire densify after Mode A void_seek hijack when pocket/pool is sterile."""
    enable_void_yield_densify_accept: bool = True
    """Under void hijack: accept densify when in-void transform count rises (not raw pool size)."""
    enable_void_pole_clear_densify: bool = True
    """Under void hijack when iv==0 both sides: accept densify if pole_near count rises."""
    void_pole_near_diag_ratio: float = 0.25
    """pole_near radius = ratio * sheet_diag (shared with void_leak props_pole radius;
    densify measures placed centroid, props_pole measures transform xy)."""
    use_free_space_cloud: bool = True
    """Sterile VOID_SEEK fallback: Halton xy in void bbox filtered by void_poly.contains."""
    free_space_cloud_samples: int = 64
    """Max Halton xy samples before angle expand / trim."""
    free_space_cloud_angles: int = 8
    """Discrete angle grains for free_space_cloud."""
    enable_void_elite_archive: bool = True
    """Persist refine-rejected void transforms as next-iter stratified niche seeds."""
    keep_history_on_void_sterile: bool = True
    """Under void/sat hijack, do not sterile-drop history/window when props=0."""
    stratified_void_elite_quota: int = 15
    """Max void-elite rows reserved inside max_transforms_per_group."""
    stratified_history_quota: int = 15
    """Max history/window elite rows reserved inside max_transforms_per_group."""
    unified_void_reserve: bool = True
    """Share reserve budget across poles ⊕ pocket ⊕ motif cluster_copy."""
    poles_reserve_only_on_hijack: bool = True
    """On void hijack, poles go to reserve only (not corridor_channel pool)."""
    motif_use_topo_anchors: bool = True
    """Repack motif stamp tries snapshot/topology poles before free_pocket."""
    void_seek_contact_hybrid: bool = True
    """void_seek for_place uses contact_hybrid ranking (ablation: False → clearance)."""
    void_rank_pole_weight: float = 8.0
    """Propose-time pole bonus under void_seek: (1-dist/diag)*(part_area/sheet.area)*weight (0 disables)."""
    enable_void_nest_pin: bool = True
    """After refine: re-add nest-void idxs missing from refine if graph.collisions-clear (P3)."""
    # Lean Void Cascade / diversity (defaults off until E2E gate; lean_void_combo enables).
    propose_cascade_short_circuit: bool = True
    """Hard skip explorers after snipers/builders fill reserve (void_seek / interior_pocket)."""
    cascade_sniper_stop_n: int = 4
    """Min fast-valid pocket/motif seeds to short-circuit explorers."""
    cascade_kiss_stop_threshold: float = 0.0
    """If >0 and pool full with mean kiss score above threshold, skip explorers."""
    cascade_explorer_budget_scale: float = 0.35
    """Non-void packed zones: shrink explorer max_items by this factor when cascade on."""
    use_pose_nms: bool = False
    """SE(2) spatial-hash NMS on ranked pool (reserve-safe merge after)."""
    pose_nms_eps: float = 1.0
    pose_nms_theta_tol: float = 0.15
    use_conflict_degree_rank: bool = False
    """Pre-MIS AABB conflict-degree score penalty (STRtree)."""
    conflict_degree_lambda: float = 0.05
    conflict_degree_max_overlap: int = 5
    use_multi_pole_void: bool = True
    """Iterative polylabel spine for large remnants (topology + explorers)."""
    multi_pole_max_poles: int = 4
    use_ema_proposer_scales: bool = False
    """Update place_proposer_pool_scales from refine survival (α=0.15, floor 0.05)."""
    ema_proposer_alpha: float = 0.15
    ema_proposer_floor: float = 0.05
    # OOS-3: keep use_point_cloud/use_guidance_walk False unless props_pole≈0 after OOS-1+4.
    @classmethod
    def proposers_for_place(
        cls,
        zone: str,
        *,
        annulus: bool = False,
    ) -> frozenset[str] | None:
        return proposers_for_zone(zone, annulus=annulus)

    @classmethod
    def assert_zone_proposer_flags(cls, cfg: "ProposeConfig | None" = None) -> None:
        """Every ZONE_PROPOSERS entry with a PROPOSER_FLAG must be enabled on for_place."""
        # Call for_place without a False-locked base so profile patches apply.
        _ = cfg  # reserved for future seed configs
        for zone, proposers in ZONE_PROPOSERS.items():
            placed = cls.for_place(zone.value)
            for name in proposers:
                flag = PROPOSER_FLAG.get(name)
                if flag is None:
                    continue
                if not bool(getattr(placed, flag, False)):
                    raise AssertionError(
                        f"zone={zone.value} proposer={name.value} requires {flag}=True"
                    )
            if zone == PlaceZone.VOID_SEEK:
                assert ProposerName.VORONOI not in proposers
                assert ProposerName.NEIGHBOR_SLIDE not in proposers
                assert placed.use_voronoi is False
                assert placed.use_neighbor_slide is False

    @classmethod
    def obstacle_scope_for_place(
        cls,
        zone: str,
        *,
        n_clusters: int = 1,
        outline_coverage: float = 0.0,
        border_coverage_threshold: float = 0.35,
    ) -> tuple[bool, int]:
        if zone in (
            PlaceZone.INTERIOR_POCKET.value,
            PlaceZone.INTER_CLUSTER.value,
            PlaceZone.VOID_SEEK.value,
        ):
            return True, 0
        if zone == PlaceZone.BORDER_GAP.value:
            if (
                n_clusters >= 3
                or outline_coverage >= border_coverage_threshold
            ):
                return True, 0
            return False, 4
        if zone == PlaceZone.CLUSTER_EDGE.value:
            return False, 3
        return False, 3

    @staticmethod
    def void_seek_budget_floors() -> tuple[int, int]:
        """Minimum (candidate_pool, max_proposals) under void_seek / densify."""
        return 1024, 512

    @classmethod
    def for_place(
        cls,
        zone: str,
        base: "ProposeConfig | None" = None,
        **overrides: Any,
    ) -> "ProposeConfig":
        root = base.model_dump() if base is not None else cls().model_dump()
        profiles: dict[str, dict[str, Any]] = {
            PlaceZone.EMPTY_BORDER.value: {
                "ranking_mode": RankingMode.BORDER,
                "border_focus_ranking": True,
                "use_border_focus": True,
                "use_contact_ranking": False,
                "cast_squeeze_top_k": 12,
                "edge_free_weight": 0.0,
            },
            PlaceZone.BORDER_GAP.value: {
                "ranking_mode": RankingMode.BORDER,
                "border_focus_ranking": True,
                "use_contact_ranking": False,
                "use_full_packed_obstacle": False,
                "obstacle_nearest_k": 4,
                "cast_squeeze_top_k": 12,
                "use_board_edge_seeds": True,
                "use_side_pack": True,
                "use_neighbor_slide": True,
                "board_edge_guidance_refine": False,
                "board_edge_samples_per_edge": 12,
                "group_edge_samples_per_edge": 8,
                "placement_num_angles": 8,
            },
            PlaceZone.INTERIOR_POCKET.value: {
                "ranking_mode": RankingMode.CONTACT_HYBRID,
                "use_contact_ranking": True,
                "use_contact_clearance_hybrid": True,
                "use_full_packed_obstacle": True,
                "use_border_focus": False,
                "border_focus_ranking": False,
                "cast_squeeze_top_k": 6,
                "use_board_edge_seeds": False,
                "use_group_edge_seeds": True,
            },
            PlaceZone.CLUSTER_EDGE.value: {
                "ranking_mode": RankingMode.CONTACT_HYBRID,
                "use_guidance_propositions": True,
                "guidance_use_tight_packing": True,
                "guidance_use_corner_alignment": True,
                "guidance_enable_grid": False,
                "use_contact_ranking": True,
                "use_contact_clearance_hybrid": True,
                "cast_squeeze_top_k": 8,
                "use_neighbor_slide": True,
                "use_ribbon_seeds": True,
                "use_full_packed_obstacle": False,
                "obstacle_nearest_k": 3,
                "contact_clearance_hybrid_weight": 0.1,
                "contact_tightness_hybrid_weight": 0.25,
                "use_side_pack": True,
                "use_group_edge_seeds": True,
            },
            PlaceZone.INTER_CLUSTER.value: {
                "ranking_mode": RankingMode.CLEARANCE,
                "use_contact_ranking": False,
                "use_full_packed_obstacle": True,
                "use_board_edge_seeds": False,
                "use_group_edge_seeds": False,
                "cast_squeeze_top_k": 4,
                "use_ribbon_seeds": True,
                "use_voronoi": True,
            },
            PlaceZone.VOID_SEEK.value: {
                "ranking_mode": RankingMode.CONTACT_HYBRID,
                "use_contact_ranking": True,
                "use_contact_clearance_hybrid": True,
                "use_full_packed_obstacle": True,
                "use_group_edge_seeds": False,
                "use_side_pack": True,
                "use_neighbor_slide": False,
                "cast_squeeze_top_k": 4,
                "candidate_pool": 1024,
                "max_proposals": 512,
                "raycast_num_rays": 8,
                "use_voronoi": False,
                "use_point_cloud": False,
                "use_batch_pack": False,
            },
        }
        patch = dict(profiles.get(zone, {}))
        if (
            zone == PlaceZone.VOID_SEEK.value
            and not bool(root.get("void_seek_contact_hybrid", True))
        ):
            patch["ranking_mode"] = RankingMode.CLEARANCE
            patch["use_contact_ranking"] = False
            patch["use_contact_clearance_hybrid"] = False
        patch.update(overrides)
        # Never re-enable a flag the caller already turned off (seeded/bench caps).
        if base is not None:
            for key, val in list(patch.items()):
                if key in root and root[key] is False and val is True:
                    del patch[key]
        root.update(patch)
        cfg = cls(**root)
        if zone == PlaceZone.VOID_SEEK.value:
            cfg = floor_void_seek_budgets(cfg)
        return cfg

    def with_complexity_lean(
        self,
        *,
        n_holes: int = 0,
        max_part_vertices: int = 0,
        max_part_interiors: int = 0,
        sheet_vertices: int = 0,
        concave_parts: bool = False,
        seeded: bool = False,
    ) -> "ProposeConfig":
        """Cap expensive proposers for holed sheets / irregular parts / seeded packs."""
        lean = (
            seeded
            or n_holes > 0
            or max_part_vertices >= 12
            or max_part_interiors >= 1
            or sheet_vertices >= 16
            or concave_parts
        )
        if not lean:
            return self
        data = self.model_dump()
        data["use_batch_pack"] = False
        # Lean shrinks non-void zones; void_seek budgets are restored in for_place
        # via profile + floor_void_seek_budgets (do not rely on this base staying large).
        data["candidate_pool"] = min(int(data["candidate_pool"]), 24)
        data["max_proposals"] = min(int(data["max_proposals"]), 16)
        data["raycast_num_rays"] = min(int(data["raycast_num_rays"]), 8)
        data["cast_squeeze_top_k"] = min(int(data["cast_squeeze_top_k"]), 4)
        # Neighbor-slide is costly on jagged/holed *parts*; keep it for sheet-hole
        # corridors that rely on tube docking after cluster_edge routing.
        if concave_parts or max_part_interiors >= 1 or max_part_vertices >= 16:
            data["use_neighbor_slide"] = False
        if (
            n_holes >= 1
            or max_part_vertices >= 12
            or max_part_interiors >= 1
            or concave_parts
            or sheet_vertices >= 16
        ):
            data["use_voronoi"] = False
            data["use_point_cloud"] = False
            data["use_guidance_walk"] = False
        strong = (
            max_part_interiors >= 1
            or max_part_vertices >= 16
            or concave_parts
        )
        if strong or n_holes >= 3:
            data["use_ribbon_seeds"] = False
            data["obstacle_nearest_k"] = min(int(data["obstacle_nearest_k"]), 2)
            data["group_edge_samples_per_edge"] = min(
                int(data["group_edge_samples_per_edge"]), 2,
            )
        if n_holes >= 3 or max_part_vertices >= 16:
            data["obstacle_nearest_k"] = min(int(data["obstacle_nearest_k"]), 1)
        return ProposeConfig(**data)

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

    def board_min_dist_for(
        self,
        board,
        *,
        first_pass: bool = False,
    ) -> float:
        """Clearance from sheet diagonal × min_dist_ratio (use the nest sheet, not demo rules)."""
        xmin, ymin, xmax, ymax = board.bounds
        diag = float(np.hypot(xmax - xmin, ymax - ymin))
        ratio = self.propose.min_dist_ratio
        if first_pass:
            ratio = self.propose.first_pass_min_dist_ratio
        return diag * ratio

    def board_min_dist(self, *, first_pass: bool = False) -> float:
        return self.board_min_dist_for(self.rules.board_polygon(), first_pass=first_pass)

    def placement_epsilon_ratio(self, *, first_pass: bool = False) -> float:
        if first_pass:
            return self.propose.first_pass_clearance_epsilon_ratio
        return self.propose.placement_clearance_epsilon_ratio

    def with_runtime_lean(
        self,
        *,
        n_holes: int = 0,
        max_part_vertices: int = 0,
        max_part_interiors: int = 0,
        sheet_vertices: int = 0,
        concave_parts: bool = False,
        seeded: bool = False,
    ) -> "BuildGraphConfig":
        """Apply propose lean (+ sampling/DFS caps for seeded/holed/simple sheets)."""
        simple_sheet = n_holes == 0 and 0 < sheet_vertices <= 4
        lean = (
            seeded
            or n_holes > 0
            or max_part_vertices >= 12
            or max_part_interiors >= 1
            or sheet_vertices >= 16
            or concave_parts
            or simple_sheet
        )
        if not lean:
            return self
        propose = self.propose.with_complexity_lean(
            n_holes=n_holes,
            max_part_vertices=max_part_vertices,
            max_part_interiors=max_part_interiors,
            sheet_vertices=sheet_vertices,
            concave_parts=concave_parts,
            seeded=seeded,
        )
        if simple_sheet and propose.use_batch_pack:
            # Keep proposal volume; only kill expensive multi-anchor batch pack.
            propose = propose.model_copy(update={"use_batch_pack": False})
        sampling_update: dict = {}
        selection_update: dict = {}
        if seeded or n_holes >= 1:
            cap = min(self.sampling.max_transforms_per_group or 120, 120)
            sampling_update = {
                "max_transforms_per_group": cap,
                "initial_random": min(self.sampling.initial_random, 48),
                "random_per_iter": min(self.sampling.random_per_iter, 32),
                "random_per_iter_when_proposed": min(
                    self.sampling.random_per_iter_when_proposed, 16,
                ),
                "shuffle_passes": min(self.sampling.shuffle_passes, 1),
            }
            selection_update = {
                "improve_rules_rounds": min(self.selection.improve_rules_rounds, 1),
                "rules_kept": min(self.selection.rules_kept, 8),
                "improve_rules_elite_count": min(
                    self.selection.improve_rules_elite_count, 4,
                ),
                "dfs_passes": min(self.selection.dfs_passes, 1),
                "dfs_max_tries": min(self.selection.dfs_max_tries, 2),
                "dfs_finalize_repair_passes": min(
                    self.selection.dfs_finalize_repair_passes, 2,
                ),
            }
        elif simple_sheet:
            # Demo triangle/rect: kill batch_pack only; keep transform pool depth.
            sampling_update = {
                "shuffle_passes": min(self.sampling.shuffle_passes, 2),
            }
        return self.model_copy(
            update={
                "propose": propose,
                "sampling": (
                    self.sampling.model_copy(update=sampling_update)
                    if sampling_update else self.sampling
                ),
                "selection": (
                    self.selection.model_copy(update=selection_update)
                    if selection_update else self.selection
                ),
            },
        )

    def first_pass_propose_config(self) -> ProposeConfig:
        p = self.propose.model_copy(deep=True)
        p.candidate_pool = max(p.candidate_pool, p.first_pass_candidate_pool)
        p.max_proposals = max(p.max_proposals, p.first_pass_max_proposals)
        p.placement_num_angles = p.first_pass_num_angles
        p.placement_clearance_epsilon_ratio = p.first_pass_clearance_epsilon_ratio
        p.group_edge_samples_per_edge = p.first_pass_group_edge_samples_per_edge
        p.ranking_mode = RankingMode.BORDER
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
                max_transforms_per_group=5000,
                seed=seed,
            ),
            selection=SelectionConfig(dfs_mode=DfsMode.MERGED_LOOSE_TIGHT),
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
                    "NEST_SELECT_MODE", ScoreSelectMode.WEIGHTED_GREEDY,
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
                    "NEST_DFS_MODE", DfsMode.MERGED_LOOSE_TIGHT,
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


def _transform_row_key4(row: np.ndarray | Sequence[float]) -> tuple[float, float, float]:
    return (round(float(row[0]), 4), round(float(row[1]), 4), round(float(row[2]), 4))


def _take_niche(
    rows: np.ndarray,
    quota: int,
    claimed: set[tuple[float, float, float]],
    rng: np.random.Generator,
) -> list[np.ndarray]:
    if quota <= 0 or rows is None or getattr(rows, "shape", (0,))[0] == 0:
        return []
    picked: list[np.ndarray] = []
    order = np.arange(rows.shape[0])
    rng.shuffle(order)
    for i in order:
        row = rows[int(i)]
        key = _transform_row_key4(row)
        if key in claimed:
            continue
        claimed.add(key)
        picked.append(row)
        if len(picked) >= quota:
            break
    return picked


def subsample_transforms_stratified(
    *,
    selection: np.ndarray,
    proposals: np.ndarray,
    void_elite: np.ndarray,
    history: np.ndarray,
    expand_rest: np.ndarray,
    max_n: Optional[int],
    rng: np.random.Generator,
    n_props: int | None = None,
    n_void_elite: int | None = None,
    n_hist: int | None = None,
) -> np.ndarray:
    """Strict niche quotas; expand/random fills the remainder only.

    Order in the returned array: selection → proposals → void_elite → history → rest.
    """
    empty = np.zeros((0, 3), dtype=np.float64)
    selection = selection if selection is not None else empty
    proposals = proposals if proposals is not None else empty
    void_elite = void_elite if void_elite is not None else empty
    history = history if history is not None else empty
    expand_rest = expand_rest if expand_rest is not None else empty

    if max_n is None:
        return dedupe_transforms(
            np.concatenate(
                [selection, proposals, void_elite, history, expand_rest], axis=0,
            )
        )
    max_n = int(max_n)
    if max_n <= 0:
        return empty

    # Default niche sizes scale with cap; props/void/hist stay bounded.
    prop_q = int(n_props) if n_props is not None else min(40, max(8, max_n // 16))
    void_q = int(n_void_elite) if n_void_elite is not None else 15
    hist_q = int(n_hist) if n_hist is not None else 15

    claimed: set[tuple[float, float, float]] = set()
    out_rows: list[np.ndarray] = []

    # Selection: keep all that fit (priority).
    sel_q = max_n
    out_rows.extend(_take_niche(selection, sel_q, claimed, rng))
    remain = max_n - len(out_rows)
    if remain <= 0:
        return np.asarray(out_rows[:max_n], dtype=np.float64)

    out_rows.extend(_take_niche(proposals, min(prop_q, remain), claimed, rng))
    remain = max_n - len(out_rows)
    if remain <= 0:
        return np.asarray(out_rows[:max_n], dtype=np.float64)

    out_rows.extend(_take_niche(void_elite, min(void_q, remain), claimed, rng))
    remain = max_n - len(out_rows)
    if remain <= 0:
        return np.asarray(out_rows[:max_n], dtype=np.float64)

    out_rows.extend(_take_niche(history, min(hist_q, remain), claimed, rng))
    remain = max_n - len(out_rows)
    if remain > 0:
        out_rows.extend(_take_niche(expand_rest, remain, claimed, rng))

    if not out_rows:
        return empty
    return np.asarray(out_rows[:max_n], dtype=np.float64)


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


def _make_select_options(
    mode: ScoreSelectMode | SelectMode | str,
    local_swap: bool,
    aggregation: str = "sum",
):
    """Build SelectOptions for elem_graph tests and benchmarks."""
    opts = SelectOptions()
    opts.mode = _coerce_select_mode(mode)
    opts.local_swap = local_swap
    opts.aggregation = (
        ScoreAggregation.Sum if aggregation == "sum" else ScoreAggregation.Max
    )
    return opts


def score_rules_options(sel: SelectionConfig):
    """ScoreRulesOptions aligned with nest_by_graph selection."""
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
