"""Placement proposal: perimeter walk, erosion, raycast/voronoi, ranking, pipeline.

Heavy placement/pipeline symbols are lazy so ``nest_graph.placement.*`` can import
``propose.context`` without cycling through this package init.
"""

from nest_graph.proposer_names import (
    ALL_PROPOSER_NAMES,
    ProposerName,
)

# Lazy re-exports: (module, attr). Avoid eager pipeline/placement imports —
# placement modules import propose.context and would otherwise partial-init cycle.
_LAZY_ATTRS: dict[str, tuple[str, str]] = {
    "ProposeGeometry": ("nest_graph.propose.geometry", "ProposeGeometry"),
    "ProposeContext": ("nest_graph.propose.types", "ProposeContext"),
    "PackedProposeExtras": ("nest_graph.propose.types", "PackedProposeExtras"),
    "SelectionEditCtx": ("nest_graph.propose.selection_edit", "SelectionEditCtx"),
    "ComposedSelection": ("nest_graph.propose.selection_compose", "ComposedSelection"),
    "allow_void_repack": ("nest_graph.propose.pipeline", "allow_void_repack"),
    "make_propose_context": ("nest_graph.propose.types", "make_propose_context"),
    "run_post_pack_passes": ("nest_graph.propose.post_pack", "run_post_pack_passes"),
    "apply_void_selection_boosts": (
        "nest_graph.propose.void_selection",
        "apply_void_selection_boosts",
    ),
    "compose_and_nest_selection": (
        "nest_graph.propose.selection_compose",
        "compose_and_nest_selection",
    ),
    "border_focal_for_propose": (
        "nest_graph.propose.context",
        "border_focal_for_propose",
    ),
    "first_pass_border_coords": (
        "nest_graph.propose.first_pass_border",
        "first_pass_border_coords",
    ),
    "format_prop_accept": ("nest_graph.propose.void_selection", "format_prop_accept"),
    "guidance_border_refine": (
        "nest_graph.propose.first_pass_border",
        "guidance_border_refine",
    ),
    "pin_nest_void_independent": (
        "nest_graph.propose.void_selection",
        "pin_nest_void_independent",
    ),
    "sequential_border_augment": (
        "nest_graph.propose.first_pass_border",
        "sequential_border_augment",
    ),
    "border_solid_focal": ("nest_graph.propose.context", "border_solid_focal"),
    "calculate_complex_score": (
        "nest_graph.propose.ranking",
        "calculate_complex_score",
    ),
    "cluster_packed_solid_groups": (
        "nest_graph.propose.context",
        "cluster_packed_solid_groups",
    ),
    "collect_propose_candidates": (
        "nest_graph.propose.pipeline",
        "collect_propose_candidates",
    ),
    "collect_propose_batch_for_nest": (
        "nest_graph.propose.pipeline",
        "collect_propose_batch_for_nest",
    ),
    "densify_points": ("nest_graph.placement.geo", "densify_points"),
    "effective_ranking_mode": ("nest_graph.propose.context", "effective_ranking_mode"),
    "evaluate_ray_placement": ("nest_graph.placement.pso", "evaluate_ray_placement"),
    "finalize_propositions": ("nest_graph.propose.ranking", "finalize_propositions"),
    "focal_shape_for_propose": ("nest_graph.propose.context", "focal_shape_for_propose"),
    "obstacle_polys_for_propose": (
        "nest_graph.propose.context",
        "obstacle_polys_for_propose",
    ),
    "obstacle_shape_for_propose": (
        "nest_graph.propose.context",
        "obstacle_shape_for_propose",
    ),
    "placement_free_region": ("nest_graph.propose.context", "placement_free_region"),
    "propose_coords_with_strategy": (
        "nest_graph.propose.pipeline",
        "propose_coords_with_strategy",
    ),
    "propose_placements_board_edge": (
        "nest_graph.placement.guidance",
        "propose_placements_board_edge",
    ),
    "propose_placements_side_pack": (
        "nest_graph.placement.edge",
        "propose_placements_side_pack",
    ),
    "propose_placements_erosion": (
        "nest_graph.placement.primary",
        "propose_placements_erosion",
    ),
    "propose_placements_group_fit": (
        "nest_graph.placement.edge",
        "propose_placements_group_fit",
    ),
    "propose_placements_guidance_cast": (
        "nest_graph.placement.guidance",
        "propose_placements_guidance_cast",
    ),
    "propose_placements_guidance_propositions": (
        "nest_graph.placement.guidance",
        "propose_placements_guidance_propositions",
    ),
    "propose_placements_guidance_walk": (
        "nest_graph.placement.guidance",
        "propose_placements_guidance_walk",
    ),
    "propose_placements_neighbor_slide": (
        "nest_graph.placement.primary",
        "propose_placements_neighbor_slide",
    ),
    "propose_placements_perimeter_walk": (
        "nest_graph.placement.primary",
        "propose_placements_perimeter_walk",
    ),
    "propose_placements_point_cloud": (
        "nest_graph.placement.pso",
        "propose_placements_point_cloud",
    ),
    "propose_placements_raycasting": (
        "nest_graph.placement.geo",
        "propose_placements_raycasting",
    ),
    "propose_placements_ribbon_free": (
        "nest_graph.placement.edge",
        "propose_placements_ribbon_free",
    ),
    "propose_placements_sheet_corners": (
        "nest_graph.placement.edge",
        "propose_placements_sheet_corners",
    ),
    "propose_placements_sheet_edge": (
        "nest_graph.placement.edge",
        "propose_placements_sheet_edge",
    ),
    "propose_placements_voronoi": (
        "nest_graph.placement.geo",
        "propose_placements_voronoi",
    ),
    "propose_placements_pocket_fit": (
        "nest_graph.placement.pocket",
        "propose_placements_pocket_fit",
    ),
    "propose_placements_cluster_copy": (
        "nest_graph.placement.pattern",
        "propose_placements_cluster_copy",
    ),
    "stamp_motif_leader_follower": (
        "nest_graph.placement.pattern",
        "stamp_motif_leader_follower",
    ),
    "void_seek_motif_anchors": (
        "nest_graph.placement.pattern",
        "void_seek_motif_anchors",
    ),
    "propose_placements_free_space_cloud": (
        "nest_graph.placement.free_space_cloud",
        "propose_placements_free_space_cloud",
    ),
    "propose_placements_selection_expand": (
        "nest_graph.placement.selection_expand",
        "propose_placements_selection_expand",
    ),
    "propose_placements_history_expand": (
        "nest_graph.placement.selection_expand",
        "propose_placements_history_expand",
    ),
    "extract_cluster_patterns": (
        "nest_graph.placement.pattern",
        "extract_cluster_patterns",
    ),
    "merge_cluster_patterns": (
        "nest_graph.placement.pattern",
        "merge_cluster_patterns",
    ),
    "iterative_multi_poles": (
        "nest_graph.propose.void_topology",
        "iterative_multi_poles",
    ),
    "nearest_spine_pole": ("nest_graph.propose.void_topology", "nearest_spine_pole"),
    "preferred_spine_pole": (
        "nest_graph.propose.void_topology",
        "preferred_spine_pole",
    ),
    "topology_pocket_poles": (
        "nest_graph.propose.void_topology",
        "topology_pocket_poles",
    ),
    "apply_pose_nms": ("nest_graph.propose.pose_diversity", "apply_pose_nms"),
    "propose_push_point": ("nest_graph.propose.context", "propose_push_point"),
    "proposed_transforms_for_groups": (
        "nest_graph.propose.pipeline",
        "proposed_transforms_for_groups",
    ),
    "propositions_to_ndarray": (
        "nest_graph.propose.pipeline",
        "propositions_to_ndarray",
    ),
    "base_shape_from_selection": (
        "nest_graph.propose.pipeline",
        "base_shape_from_selection",
    ),
    "border_edge_transforms_for_group": (
        "nest_graph.propose.pipeline",
        "border_edge_transforms_for_group",
    ),
    "sample_placement_points_ribbon": (
        "nest_graph.placement.edge",
        "sample_placement_points_ribbon",
    ),
    "search_region_for_placement": (
        "nest_graph.propose.context",
        "search_region_for_placement",
    ),
    "should_use_border_focus": (
        "nest_graph.propose.context",
        "should_use_border_focus",
    ),
}


def __getattr__(name: str):
    spec = _LAZY_ATTRS.get(name)
    if spec is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    mod_name, attr = spec
    import importlib

    mod = importlib.import_module(mod_name)
    value = getattr(mod, attr)
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(_LAZY_ATTRS) | {"ALL_PROPOSER_NAMES", "ProposerName"})


__all__ = [
    "ALL_PROPOSER_NAMES",
    "ProposerName",
    *sorted(_LAZY_ATTRS),
]
