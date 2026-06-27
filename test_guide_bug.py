from nest_graph.geometry import Geometry
from nest_graph.placement_scene import build_placement_scene, guidance_config_for_propose
from shapely.geometry import Polygon, Point

p1 = Geometry.from_shapely(Polygon([(0,0), (1,0), (1,1), (0,1)]))
p2 = Geometry.from_shapely(Polygon([(3,0), (4,0), (4,1), (3,1)]))
sheet = Polygon([(-10,-10), (10,-10), (10,10), (-10,10)])

scene = build_placement_scene(sheet, p1, [p2])
placed = scene.placed_at((0,0,0))

cfg = guidance_config_for_propose(
    pt_push=Point(0,0),
    min_dist=0.0,
    board_bounds=sheet.bounds,
    target_angle_rad=0.5,
    border_focus=False
)

guidance = scene.guidance(placed, (0.0, 0.0), cfg)
for p in guidance.propositions:
    print(f"Move: {p.move_type} | Trans: {p.translation} | Rot: {p.rotation_rad}")
