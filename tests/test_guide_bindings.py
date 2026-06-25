import math
import pytest
from nest_graph.geometry import (
    GuidanceConfig,
    PlacementProposition,
    PlacementGuidance,
    evaluate_local_placement,
    CastResult,
    find_closest_polygon_cast,
    Geometry,
)

def test_import_smoke():
    assert GuidanceConfig is not None
    assert PlacementProposition is not None
    assert PlacementGuidance is not None
    assert evaluate_local_placement is not None
    assert CastResult is not None
    assert find_closest_polygon_cast is not None

def test_guidance_config_round_trip():
    cfg = GuidanceConfig()
    
    # Test scalars and bools
    cfg.minimum_placing_distance = 0.5
    assert cfg.minimum_placing_distance == 0.5
    
    cfg.use_tight_packing = False
    assert cfg.use_tight_packing is False
    
    cfg.use_corner_alignment = False
    assert cfg.use_corner_alignment is False
    
    cfg.use_hole_seeking = False
    assert cfg.use_hole_seeking is False
    
    cfg.max_hole_size_ratio = 5.0
    assert cfg.max_hole_size_ratio == 5.0
    
    cfg.use_gravity = False
    assert cfg.use_gravity is False
    
    cfg.use_target_attractor = True
    assert cfg.use_target_attractor is True
    
    cfg.target_angle_rad = math.pi / 4
    assert cfg.target_angle_rad == math.pi / 4
    
    cfg.max_propositions = 10
    assert cfg.max_propositions == 10
    
    cfg.diversity_distance_threshold = 3.0
    assert cfg.diversity_distance_threshold == 3.0
    
    cfg.diversity_angle_rad_threshold = math.pi / 6
    assert cfg.diversity_angle_rad_threshold == math.pi / 6
    
    cfg.enable_grid_exploration = False
    assert cfg.enable_grid_exploration is False
    
    cfg.max_alternative_angles = 5
    assert cfg.max_alternative_angles == 5
    
    cfg.slide_escape_multiplier = 0.9
    assert cfg.slide_escape_multiplier == 0.9
    
    cfg.attraction_weight = 0.2
    assert cfg.attraction_weight == 0.2
    
    cfg.alignment_weight = 0.6
    assert cfg.alignment_weight == 0.6
    
    cfg.escape_radius_multiplier = 2.0
    assert cfg.escape_radius_multiplier == 2.0
    
    cfg.search_radius = 50.0
    assert cfg.search_radius == 50.0
    
    # Test vector properties
    cfg.target_position = (10.0, 20.0)
    assert cfg.target_position == (10.0, 20.0)
    
    cfg.gravity_vector = (0.0, -2.0)
    assert cfg.gravity_vector == (0.0, -2.0)

def test_return_type_shape():
    # Two separated unit squares
    square1 = Geometry.from_ring([(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)])
    square2 = Geometry.from_ring([(3, 0), (4, 0), (4, 1), (3, 1), (3, 0)])
    
    guidance = evaluate_local_placement(0, [square1, square2], (0.5, 0.5))
    assert isinstance(guidance, PlacementGuidance)
    assert isinstance(guidance.is_penetrating, bool)
    assert isinstance(guidance.clearance, float)
    assert isinstance(guidance.propositions, list)
    
    if len(guidance.propositions) > 0:
        prop = guidance.propositions[0]
        assert isinstance(prop, PlacementProposition)
        assert isinstance(prop.translation, tuple)
        assert len(prop.translation) == 2
        assert isinstance(prop.rotation_rad, float)
        assert isinstance(prop.heuristic_score, float)
        assert isinstance(prop.move_type, str)

def test_config_optional():
    square1 = Geometry.from_ring([(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)])
    square2 = Geometry.from_ring([(3, 0), (4, 0), (4, 1), (3, 1), (3, 0)])
    
    guidance_none = evaluate_local_placement(0, [square1, square2], (0.5, 0.5), config=None)
    guidance_explicit = evaluate_local_placement(0, [square1, square2], (0.5, 0.5), config=GuidanceConfig())
    
    assert len(guidance_none.propositions) == len(guidance_explicit.propositions)
    if len(guidance_none.propositions) > 0:
        assert guidance_none.propositions[0].move_type == guidance_explicit.propositions[0].move_type

def test_index_contract():
    square1 = Geometry.from_ring([(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)])
    square2 = Geometry.from_ring([(3, 0), (4, 0), (4, 1), (3, 1), (3, 0)])
    
    guidance0 = evaluate_local_placement(0, [square1, square2], (0.5, 0.5))
    guidance1 = evaluate_local_placement(1, [square1, square2], (3.5, 0.5))
    
    # The propositions should be different because gravity acts on different parts
    # or they are in different relative positions.
    # At least we verify it doesn't crash and returns valid guidance.
    assert isinstance(guidance0, PlacementGuidance)
    assert isinstance(guidance1, PlacementGuidance)
    
    # Out of bounds should raise IndexError
    with pytest.raises(IndexError):
        evaluate_local_placement(-1, [square1, square2], (0.5, 0.5))
        
    with pytest.raises(IndexError):
        evaluate_local_placement(2, [square1, square2], (0.5, 0.5))

def test_cast_binding_smoke():
    square1 = Geometry.from_ring([(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)])
    square2 = Geometry.from_ring([(3, 0), (4, 0), (4, 1), (3, 1), (3, 0)])
    
    cast_res = find_closest_polygon_cast(square1, [square2], (1.0, 0.0), 10.0)
    assert isinstance(cast_res, CastResult)
    assert isinstance(cast_res.intersects_path, bool)
    assert isinstance(cast_res.t_entry, float)
