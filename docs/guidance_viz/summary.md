| scenario | shapes | props | penetrating | best_move | kiss_err | border_err | board_ok | exec_ms | notes |
|----------|--------|-------|-------------|-----------|----------|------------|----------|---------|-------|
| nest_border_dock | part + obs | 6 | False | Vertex Corner Match | 0.0010 | 0.0000 | True | 0.061 | triangle sheet, dock to border near rect cluster |
| rect_cluster_gap | part + obs | 3 | False | Vertex Corner Match | 0.0200 | 0.0000 | True | 0.174 | L-gap between three rects on triangle sheet |
| tri_rect_mixed | part + obs | 6 | False | Vertex Corner Match | 0.0010 | 0.0000 | False | 0.064 | mixed tri+rect obstacles, tri part mid-sheet |
| l_shape_pocket | part + obs | 3 | True | Primary Ejection | 0.5000 | 0.0000 | True | 0.029 | rect in L-pocket overlap, ejection recovery |
| l_snap_to_bar | part + obs | 6 | True | Primary Ejection | 0.0200 | 0.0200 | True | 0.071 | rotated L-part overlapping bar, snap after ejection |
| overlap_recovery | part + obs | 2 | True | Primary Ejection | 0.0200 | 0.0200 | False | 0.036 | deliberate overlap, primary ejection only |
| void_corner | part + obs | 6 | False | Vertex Corner Match | 0.0134 | 0.0000 | False | 0.044 | nest corner void, tri docks to origin |
| board_hole_mouth | part + obs | 6 | False | Vertex Corner Match | 0.2231 | 0.0000 | False | 0.109 | sheet void hole with flanking rects, mouth placement |
| concave_notch | part + obs | 3 | False | Vertex Corner Match | 0.5000 | 0.0000 | True | 0.094 | tri part in concave notch between L and rect |
| tri_board_dense_pack | part + obs | 2 | True | Primary Ejection | 0.0200 | 0.0000 | True | 0.071 | dense cluster with star/gear flanking seed at tight clearance |
| pentagon_board_cove | part + obs | 6 | False | Exact Gravity Dock | 0.1768 | 0.0000 | False | 0.096 | pentagon sheet, jagged s/cross/gear trap near seed |
| triple_void_lane | part + obs | 3 | False | Vertex Corner Match | 1.7442 | 0.0000 | False | 0.308 | three void lanes + gear teeth protruding near seed |
| c_shape_wrap_cluster | part + obs | 6 | True | Primary Ejection | 0.3500 | 0.0000 | False | 0.083 | C-part entangled with gear/s-shape, deeper overlap seed |
| trapezoid_l_pocket | part + obs | 2 | True | Primary Ejection | 0.3600 | 0.0000 | False | 0.076 | trapezoid vs gear/star pocket, seed at concave mouth |
| hex_ring_cavity | part + obs | 6 | False | Vertex Corner Match | 0.2074 | 0.0000 | False | 0.205 | star part tight against jagged star-ring cavity wall |
| jagged_inner_dock | part + obs | 6 | False | Exact Gravity Dock | 3.3759 | 0.1772 | False | 0.223 | U-channel dock with gear/cross guarding entrance near seed |
| irregular_heptagon_cluster | part + obs | 3 | True | Primary Ejection | 0.3200 | 0.0000 | False | 0.113 | heptagon seed amid star/gear debris field |
| l_shaped_sheet_corner | part + obs | 6 | False | Exact Gravity Dock | 0.0048 | 0.0000 | True | 0.189 | L-sheet inner corner, interlocking s/cross/gear cluster near seed |
| narrow_strip_channel | part + obs | 2 | False | Vertex Corner Match | 0.0129 | 0.0000 | False | 0.134 | strip maze with gear pinch point beside seed |
