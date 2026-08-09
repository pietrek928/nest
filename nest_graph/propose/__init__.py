"""Placement proposal: perimeter walk, erosion, raycast/voronoi, ranking, pipeline."""

from nest_graph.propose.context import (
    border_focal_for_propose,
    border_solid_focal,
    cluster_packed_solid_groups,
    effective_ranking_mode,
    focal_shape_for_propose,
    obstacle_polys_for_propose,
    obstacle_shape_for_propose,
    placement_free_region,
    propose_push_point,
    search_region_for_placement,
    should_use_border_focus,
)
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.proposer_names import (
    ALL_PROPOSER_NAMES,
    ProposerName,
)
from nest_graph.propose.pipeline import (
    allow_void_repack,
    base_shape_from_selection,
    border_edge_transforms_for_group,
    collect_propose_batch_for_nest,
    collect_propose_candidates,
    propose_coords_with_strategy,
    proposed_transforms_for_groups,
    propositions_to_ndarray,
)
from nest_graph.propose.placements_geo import (
    densify_points,
    propose_placements_raycasting,
    propose_placements_voronoi,
)
from nest_graph.propose.placements_guidance import (
    propose_placements_board_edge,
    propose_placements_guidance_cast,
    propose_placements_guidance_propositions,
    propose_placements_guidance_walk,
)
from nest_graph.propose.placements_edge import (
    propose_placements_group_fit,
    propose_placements_ribbon_free,
    propose_placements_sheet_corners,
    propose_placements_sheet_edge,
    propose_placements_side_pack,
    sample_placement_points_ribbon,
)
from nest_graph.propose.placements_primary import (
    propose_placements_erosion,
    propose_placements_neighbor_slide,
    propose_placements_perimeter_walk,
)
from nest_graph.propose.placements_pso import (
    evaluate_ray_placement,
    propose_placements_point_cloud,
)
from nest_graph.propose.placements_pocket import propose_placements_pocket_fit
from nest_graph.propose.placements_pattern import (
    extract_cluster_patterns,
    propose_placements_cluster_copy,
    stamp_motif_leader_follower,
    void_seek_motif_anchors,
)
from nest_graph.propose.placements_free_space_cloud import (
    propose_placements_free_space_cloud,
)
from nest_graph.propose.placements_selection_expand import (
    propose_placements_history_expand,
    propose_placements_selection_expand,
)
from nest_graph.propose.void_topology import (
    iterative_multi_poles,
    topology_pocket_poles,
)
from nest_graph.propose.pose_diversity import apply_pose_nms
from nest_graph.propose.ranking import (
    calculate_complex_score,
    finalize_propositions,
)
from nest_graph.propose.post_pack import run_post_pack_passes
from nest_graph.propose.selection_edit import SelectionEditCtx
from nest_graph.propose.types import (
    PackedProposeExtras,
    ProposeContext,
    make_propose_context,
)
from nest_graph.propose.first_pass_border import (
    first_pass_border_coords,
    guidance_border_refine,
    sequential_border_augment,
)
from nest_graph.propose.void_selection import (
    apply_void_selection_boosts,
    format_prop_accept,
    pin_nest_void_independent,
)
from nest_graph.propose.selection_compose import (
    ComposedSelection,
    compose_and_nest_selection,
)

__all__ = [
    "ALL_PROPOSER_NAMES",
    "ProposerName",
    "ProposeGeometry",
    "ProposeContext",
    "PackedProposeExtras",
    "SelectionEditCtx",
    "allow_void_repack",
    "make_propose_context",
    "run_post_pack_passes",
    "apply_void_selection_boosts",
    "compose_and_nest_selection",
    "ComposedSelection",
    "border_focal_for_propose",
    "first_pass_border_coords",
    "format_prop_accept",
    "guidance_border_refine",
    "pin_nest_void_independent",
    "sequential_border_augment",
    "border_solid_focal",
    "calculate_complex_score",
    "cluster_packed_solid_groups",
    "collect_propose_candidates",
    "collect_propose_batch_for_nest",
    "densify_points",
    "effective_ranking_mode",
    "evaluate_ray_placement",
    "finalize_propositions",
    "focal_shape_for_propose",
    "obstacle_polys_for_propose",
    "obstacle_shape_for_propose",
    "placement_free_region",
    "propose_coords_with_strategy",
    "propose_placements_board_edge",
    "propose_placements_side_pack",
    "propose_placements_erosion",
    "propose_placements_group_fit",
    "propose_placements_guidance_cast",
    "propose_placements_guidance_propositions",
    "propose_placements_guidance_walk",
    "propose_placements_neighbor_slide",
    "propose_placements_perimeter_walk",
    "propose_placements_point_cloud",
    "propose_placements_raycasting",
    "propose_placements_ribbon_free",
    "propose_placements_sheet_corners",
    "propose_placements_sheet_edge",
    "propose_placements_voronoi",
    "propose_placements_pocket_fit",
    "propose_placements_cluster_copy",
    "stamp_motif_leader_follower",
    "void_seek_motif_anchors",
    "propose_placements_free_space_cloud",
    "propose_placements_selection_expand",
    "propose_placements_history_expand",
    "extract_cluster_patterns",
    "iterative_multi_poles",
    "topology_pocket_poles",
    "apply_pose_nms",
    "propose_push_point",
    "proposed_transforms_for_groups",
    "propositions_to_ndarray",
    "base_shape_from_selection",
    "border_edge_transforms_for_group",
    "sample_placement_points_ribbon",
    "search_region_for_placement",
    "should_use_border_focus",
]
