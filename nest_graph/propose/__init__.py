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
    base_shape_from_selection,
    border_edge_transforms_for_group,
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
    propose_placements_guidance_cast,
    propose_placements_guidance_propositions,
    propose_placements_guidance_walk,
)
from nest_graph.propose.placements_edge import (
    propose_placements_board_edge,
    propose_placements_group_fit,
    propose_placements_ribbon_free,
    propose_placements_sheet_corners,
    propose_placements_sheet_edge,
    sample_placement_points_ribbon,
)
from nest_graph.propose.placements_primary import (
    propose_placements_axis_push,
    propose_placements_bottom_left,
    propose_placements_erosion,
    propose_placements_neighbor_slide,
    propose_placements_nfp_vertices,
    propose_placements_perimeter_walk,
)
from nest_graph.propose.placements_pso import (
    evaluate_ray_placement,
    propose_placements_point_cloud,
)
from nest_graph.propose.ranking import (
    calculate_complex_score,
    finalize_propositions,
)

__all__ = [
    "ALL_PROPOSER_NAMES",
    "ProposerName",
    "ProposeGeometry",
    "border_focal_for_propose",
    "border_solid_focal",
    "calculate_complex_score",
    "cluster_packed_solid_groups",
    "collect_propose_candidates",
    "densify_points",
    "effective_ranking_mode",
    "evaluate_ray_placement",
    "finalize_propositions",
    "focal_shape_for_propose",
    "obstacle_polys_for_propose",
    "obstacle_shape_for_propose",
    "placement_free_region",
    "propose_coords_with_strategy",
    "propose_placements_axis_push",
    "propose_placements_board_edge",
    "propose_placements_bottom_left",
    "propose_placements_erosion",
    "propose_placements_group_fit",
    "propose_placements_guidance_cast",
    "propose_placements_guidance_propositions",
    "propose_placements_guidance_walk",
    "propose_placements_neighbor_slide",
    "propose_placements_nfp_vertices",
    "propose_placements_perimeter_walk",
    "propose_placements_point_cloud",
    "propose_placements_raycasting",
    "propose_placements_ribbon_free",
    "propose_placements_sheet_corners",
    "propose_placements_sheet_edge",
    "propose_placements_voronoi",
    "propose_push_point",
    "proposed_transforms_for_groups",
    "propositions_to_ndarray",
    "base_shape_from_selection",
    "border_edge_transforms_for_group",
    "sample_placement_points_ribbon",
    "search_region_for_placement",
    "should_use_border_focus",
]
