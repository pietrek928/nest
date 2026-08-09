"""First-pass border stack: outline-kiss saturation, guidance refine, interior fill.

Extracted from ``build_graph``. Border candidate emission goes through
``collect_propose_candidates`` with ``border_focus=True`` (no private augment
proposer stack), and pack tightness uses ``ranking.pack_tightness_cost``.
"""

import math

import numpy as np
from shapely import Polygon, unary_union
from shapely.geometry import Point
from shapely.ops import nearest_points
from shapely.geometry.base import BaseGeometry

from nest_graph.board import (
    board_context_from_geometry,
    default_sheet_padding,
    padded_board_bounds,
)
from nest_graph.config import BuildGraphConfig, RankingMode, dedupe_transforms
from nest_graph.elem_graph import (
    Circle,
    ElemGraph,
    Vec2,
    selection_is_independent,
)
from nest_graph.geometry import Geometry, GuidanceConfig, find_polygon_intersections
from nest_graph.placement_scene import (
    PlacementScene,
    guidance_config_for_board_edge_anchor,
    guidance_config_for_graph,
    placement_ok_for_outline,
    placement_scene_for_part,
    proposition_translation,
)
from nest_graph.propose.context import border_focal_for_propose, propose_push_point
from nest_graph.propose.geometry import ProposeGeometry
from nest_graph.propose.pipeline import (
    _normalize_proposers,
    collect_propose_candidates,
    proposed_transforms_for_groups,
)
from nest_graph.propose.placement_common import as_geometry, is_pose_clear
from nest_graph.propose.placement_outline import (
    outline_kiss_tolerance,
    outline_standoff_distance,
)
from nest_graph.propose.placement_perimeter import edge_inward_at_point
from nest_graph.propose.placements_guidance import (
    candidate_from_proposition,
    is_cast_move,
    sorted_guidance_propositions,
)
from nest_graph.propose.ranking import pack_tightness_cost, score_placement_tightness
from nest_graph.proposer_names import ProposerName
from nest_graph.utils import transform_poly

FIRST_PASS_BORDER_PROPOSERS = frozenset({
    ProposerName.BOARD_EDGE,
    ProposerName.GROUP_FIT,
    ProposerName.NEIGHBOR_SLIDE,
})


def first_pass_border_coords(
    cfg: BuildGraphConfig,
    board: BaseGeometry,
    part_poly: Polygon,
    placed_polys: list,
    *,
    min_dist: float,
) -> list[tuple[float, float, float]]:
    """Outline candidates for first-pass augment: board snap + chain fit + slide.

    Goes through ``collect_propose_candidates`` with ``border_focus=True`` so the
    first pass shares the emit order / counting of the main propose path.
    """
    sheet, _ = board_context_from_geometry(board)
    obstacle = unary_union(placed_polys) if placed_polys else Polygon()
    base_cfg = cfg.first_pass_propose_config()
    reserve = max(int(base_cfg.first_pass_max_proposals), 1)
    propose_cfg = base_cfg.model_copy(update={
        "candidate_pool": reserve,
        "use_board_edge_seeds": True,
        "board_edge_when_packed": True,
        "use_group_edge_seeds": True,
        "use_neighbor_slide": True,
        "use_free_region_search": True,
        "group_edge_samples_per_edge": (
            base_cfg.first_pass_group_edge_samples_per_edge
        ),
    })
    push = propose_push_point(
        board,
        obstacle,
        smart_push=propose_cfg.smart_push_target,
        min_dist=min_dist,
        use_border_focus=True,
    )
    geom = ProposeGeometry(
        board,
        obstacle,
        part_poly,
        min_dist,
        epsilon_ratio=propose_cfg.placement_clearance_epsilon_ratio,
        propose_cfg=propose_cfg,
        border_focus=True,
    )
    focal = (
        unary_union([border_focal_for_propose(board, min_dist), obstacle])
        if not obstacle.is_empty
        else None
    )
    coords = collect_propose_candidates(
        obstacle,
        part_poly,
        sheet,
        propose_cfg,
        min_dist=min_dist,
        pt_push=push,
        propose_geom=geom,
        focal_shape=focal,
        enabled_proposers=FIRST_PASS_BORDER_PROPOSERS,
        border_focus=True,
    )
    seen: set[tuple[float, float, float]] = set()
    out: list[tuple[float, float, float]] = []
    for c in coords:
        key = (round(c[0], 3), round(c[1], 3), round(c[2], 2))
        if key in seen:
            continue
        seen.add(key)
        out.append(c)
    return out


def nest_outline_boundary(outline: BaseGeometry):
    if hasattr(outline, "exterior"):
        return outline.exterior
    return outline.boundary


def border_kiss_indices(
    polys: list,
    outline: BaseGeometry,
    min_dist: float,
) -> list[int]:
    tol = outline_kiss_tolerance(min_dist)
    return [
        i
        for i, poly in enumerate(polys)
        if abs(outline_standoff_distance(poly, outline) - min_dist) <= tol
    ]


def perimeter_sort_key(poly, outline: BaseGeometry) -> float:
    ring = nest_outline_boundary(outline)
    touch = nearest_points(ring, poly.centroid)[0]
    return float(ring.project(touch))


def border_saturation_transform_batch(
    cfg: BuildGraphConfig,
    board: BaseGeometry,
    parts: list[tuple[Polygon, int]],
    nest_state: "NestState",
) -> tuple[np.ndarray, np.ndarray]:
    """Propose-only batch for outline saturation (no random/history noise).

    Primary path is ``proposed_transforms_for_groups(border_only=True)``.
    Thin-augment only proposers in ``FIRST_PASS_BORDER_PROPOSERS`` that the
    border-only zone enable set did not already cover.
    """
    from nest_graph.proposer_names import (
        FIRST_PASS_EMPTY_BORDER_PROPOSERS,
        first_pass_packed_border_proposers,
    )

    min_dist = cfg.board_min_dist(first_pass=True)
    polys = nest_state.polys
    selected = nest_state.selected_indices
    group_id = nest_state.group_id
    transform = nest_state.transform
    part_by_gid = {gid: poly for poly, gid in parts}
    pack_polys = [polys[i] for i in selected]
    phase1_by_group: list[list[np.ndarray]] = [[], []]
    for idx in selected:
        phase1_by_group[group_id[idx]].append(transform[idx])
    native = nest_state.native_geoms
    full_packed_geoms = [
        native[i] for i in selected if 0 <= i < len(native)
    ]
    if len(full_packed_geoms) != len(selected):
        full_packed_geoms = None
    propose_cfg = cfg.first_pass_propose_config()
    propose_by_group = proposed_transforms_for_groups(
        board,
        parts,
        polys,
        selected,
        propose_cfg,
        min_dist=min_dist,
        border_only_propose=True,
        use_full_packed_obstacle=cfg.propose.use_full_packed_obstacle,
        packed_group_ids=group_id,
        packed_transforms=transform,
        user_holes=cfg.rules.board_holes,
        seeded=bool(nest_state.seed_count > 0),
        pocket_keys_out={},
        densify_stats_out={},
        zones_used_out=[],
        full_packed_geoms=full_packed_geoms,
    )
    # Match pipeline border_only enable: empty sheet → EMPTY_BORDER; else packed set.
    empty_sheet = not selected
    if empty_sheet:
        zone_enabled = FIRST_PASS_EMPTY_BORDER_PROPOSERS
    else:
        zone_enabled = first_pass_packed_border_proposers()
    missing = _normalize_proposers(FIRST_PASS_BORDER_PROPOSERS) - _normalize_proposers(
        zone_enabled
    )
    out: list[np.ndarray] = []
    for gid in range(len(phase1_by_group)):
        phase1 = (
            np.asarray(phase1_by_group[gid], dtype=np.float64)
            if phase1_by_group[gid]
            else np.zeros((0, 3))
        )
        proposed = propose_by_group.get(gid, np.zeros((0, 3)))
        augment = np.zeros((0, 3), dtype=np.float64)
        if missing and gid in part_by_gid:
            # Thin collect: only missing first-pass border proposers.
            sheet, _ = board_context_from_geometry(board)
            obstacle = unary_union(pack_polys) if pack_polys else Polygon()
            reserve = max(int(propose_cfg.first_pass_max_proposals), 1)
            thin_cfg = propose_cfg.model_copy(update={
                "candidate_pool": reserve,
                "use_board_edge_seeds": "board_edge" in missing,
                "board_edge_when_packed": True,
                "use_group_edge_seeds": "group_fit" in missing,
                "use_neighbor_slide": "neighbor_slide" in missing,
                "use_free_region_search": True,
            })
            push = propose_push_point(
                board,
                obstacle,
                smart_push=thin_cfg.smart_push_target,
                min_dist=min_dist,
                use_border_focus=True,
            )
            geom = ProposeGeometry(
                board,
                obstacle,
                part_by_gid[gid],
                min_dist,
                epsilon_ratio=thin_cfg.placement_clearance_epsilon_ratio,
                propose_cfg=thin_cfg,
                border_focus=True,
                full_packed_geoms=full_packed_geoms,
            )
            focal = (
                unary_union([border_focal_for_propose(board, min_dist), obstacle])
                if not obstacle.is_empty
                else None
            )
            missing_enums = frozenset(
                p for p in FIRST_PASS_BORDER_PROPOSERS if p.value in missing
            )
            augment_coords = collect_propose_candidates(
                obstacle,
                part_by_gid[gid],
                sheet,
                thin_cfg,
                min_dist=min_dist,
                pt_push=push,
                propose_geom=geom,
                focal_shape=focal,
                enabled_proposers=missing_enums,
                border_focus=True,
            )
            if augment_coords:
                augment = np.asarray(augment_coords, dtype=np.float64)
        extra = dedupe_transforms(
            np.concatenate([proposed, augment], axis=0),
        )
        cap = max(cfg.propose.first_pass_max_proposals * 4, 128)
        room = max(cap - phase1.shape[0], 0)
        if extra.shape[0] > room:
            extra = extra[:room]
        merged = dedupe_transforms(np.concatenate([phase1, extra], axis=0))
        out.append(merged)
    return (out[0], out[1])


def outline_anchor_inward(
    poly,
    outline: BaseGeometry,
) -> tuple[Point, tuple[float, float]]:
    ring = nest_outline_boundary(outline)
    anchor, _ = nearest_points(ring, poly)
    if isinstance(outline, Polygon):
        edge_info = edge_inward_at_point(outline, anchor)
        if edge_info is not None:
            return edge_info
    if hasattr(outline, "representative_point"):
        interior = outline.representative_point()
    else:
        interior = ring.interpolate(0.5, normalized=True)
    ox = anchor.x - interior.x
    oy = anchor.y - interior.y
    dist = float(np.hypot(ox, oy))
    if dist < 1e-9:
        return anchor, (-1.0, -1.0)
    return anchor, (ox / dist, oy / dist)


def border_refine_candidates(
    x: float,
    y: float,
    theta: float,
    g,
    *,
    min_dist: float,
    max_props: int,
) -> list[tuple[float, float, float]]:
    """Cast snaps plus fractional slide steps along guidance propositions."""
    out: list[tuple[float, float, float]] = []
    seen: set[tuple[float, float, float]] = set()

    def add(coords: tuple[float, float, float]) -> None:
        key = (round(coords[0], 4), round(coords[1], 4), round(coords[2], 4))
        if key in seen:
            return
        seen.add(key)
        out.append(coords)

    for prop in sorted_guidance_propositions(g)[:max_props]:
        use_cast = not g.is_penetrating and is_cast_move(prop.move_type or "")
        add(candidate_from_proposition(x, y, theta, prop))
        if use_cast:
            continue
        tx, ty = proposition_translation(prop)
        mag = math.hypot(tx, ty)
        if mag < 1e-9:
            continue
        step_scale = 0.2
        step_len = step_scale * max(min_dist, 1e-4)
        if not g.is_penetrating:
            step_len = step_scale * mag
        for frac in (0.15, 0.25, 0.4, 0.7, 1.0):
            nx = x + tx / mag * step_len * frac
            ny = y + ty / mag * step_len * frac
            add((nx, ny, theta))
    return out


def border_refine_micro_walk(
    x: float,
    y: float,
    theta: float,
    g,
    *,
    scene: PlacementScene,
    part_geom: Geometry,
    others_geoms: list[Geometry],
    outline: BaseGeometry,
    edge_cfg: GuidanceConfig,
    min_dist: float,
    eps: float,
    walk_steps: int = 4,
    step_scale: float = 0.2,
) -> tuple[float, float, float]:
    """Step along top guidance slides from an accepted refine pose."""
    cx, cy, ctheta = x, y, theta
    for _ in range(walk_steps):
        props = sorted_guidance_propositions(g)[:3]
        if not props:
            break
        moved = False
        for prop in props:
            tx, ty = proposition_translation(prop)
            mag = math.hypot(tx, ty)
            if mag < 1e-9:
                continue
            use_cast = not g.is_penetrating and is_cast_move(prop.move_type or "")
            if use_cast:
                step_len = mag
            else:
                step_len = step_scale * max(min_dist, 1e-4)
                if not g.is_penetrating:
                    step_len = step_scale * mag
            nx = cx + tx / mag * step_len
            ny = cy + ty / mag * step_len
            ntheta = ctheta
            if abs(float(prop.rotation_rad)) > 1e-6:
                delta = float(prop.rotation_rad) - ctheta
                while delta > math.pi:
                    delta -= 2 * math.pi
                while delta < -math.pi:
                    delta += 2 * math.pi
                ntheta = ctheta + delta * (1.0 if use_cast else 0.2)
            cand_geom = part_geom.apply_transform(
                np.asarray((nx, ny, ntheta), dtype=np.float64),
            )
            if not placement_ok_for_outline(
                scene,
                cand_geom,
                outline,
                others_geoms,
                min_dist,
                edge_cfg,
                epsilon_ratio=eps,
                require_outline_kiss=True,
            ):
                continue
            cx, cy, ctheta = nx, ny, ntheta
            placed = scene.placed_at((cx, cy, ctheta))
            g = scene.guidance(placed, (cx, cy), edge_cfg)
            moved = True
            break
        if not moved:
            break
    return (cx, cy, ctheta)


def guidance_border_refine(
    cfg: BuildGraphConfig,
    board: BaseGeometry,
    parts: list[tuple[Polygon, int]],
    *,
    outline: BaseGeometry,
    pack_polys: list,
    pack_gids: list[int],
    pack_tr: list[np.ndarray],
) -> tuple[list, list[int], list[np.ndarray]]:
    """Tighten border ring placements using per-anchor guidance casts."""
    passes = max(cfg.propose.first_pass_guidance_refine_passes, 0)
    if passes <= 0 or len(pack_polys) < 2:
        return pack_polys, pack_gids, pack_tr

    min_dist = cfg.board_min_dist(first_pass=True)
    eps = cfg.placement_epsilon_ratio(first_pass=True)
    propose_cfg = cfg.first_pass_propose_config()
    sheet, voids = board_context_from_geometry(board)
    board_geom = Geometry.from_shapely(sheet)
    pad = default_sheet_padding(board)
    bounds = padded_board_bounds(board, pad)
    part_by_gid = {gid: poly for poly, gid in parts}
    bases = {gid: Geometry.from_shapely(part_by_gid[gid]) for gid in part_by_gid}

    polys = list(pack_polys)
    gids = list(pack_gids)
    trs = [np.asarray(t, dtype=np.float64) for t in pack_tr]
    geoms = [bases[gid].apply_transform(t) for gid, t in zip(gids, trs, strict=True)]
    max_props = max(propose_cfg.guidance_max_propositions, 1)

    for _ in range(passes):
        base_cost = pack_tightness_cost(geoms, outline, min_dist)
        improved = False
        order = sorted(
            range(len(polys)),
            key=lambda i: perimeter_sort_key(polys[i], outline),
        )
        for idx in order:
            gid = gids[idx]
            part_poly = part_by_gid[gid]
            x, y, theta = float(trs[idx][0]), float(trs[idx][1]), float(trs[idx][2])
            anchor, inward = outline_anchor_inward(polys[idx], outline)
            others_geoms = [geoms[j] for j in range(len(geoms)) if j != idx]
            scene = placement_scene_for_part(
                sheet, board_geom, voids, bases[gid], base_geoms=others_geoms,
            )
            edge_cfg = guidance_config_for_board_edge_anchor(
                anchor,
                inward,
                min_dist=min_dist,
                board_bounds=bounds,
                epsilon_ratio=eps,
                target_angle_rad=theta,
                max_propositions=max_props,
                use_tight_packing=propose_cfg.guidance_use_tight_packing,
                use_corner_alignment=propose_cfg.guidance_use_corner_alignment,
                enable_grid_exploration=propose_cfg.guidance_enable_grid,
                diversity_dist_ratio=propose_cfg.guidance_diversity_dist_ratio,
            )
            placed = scene.placed_at((x, y, theta))
            g = scene.guidance(placed, (x, y), edge_cfg)
            best_coords: tuple[float, float, float] | None = None
            best_cost = base_cost
            for candidate in border_refine_candidates(
                x, y, theta, g, min_dist=min_dist, max_props=max_props,
            ):
                cand_poly = transform_poly(part_poly, candidate)
                cand_geom = bases[gid].apply_transform(np.asarray(candidate, dtype=np.float64))
                if not placement_ok_for_outline(
                    scene,
                    cand_geom,
                    outline,
                    others_geoms,
                    min_dist,
                    edge_cfg,
                    epsilon_ratio=eps,
                    require_outline_kiss=True,
                ):
                    continue
                trial_polys = list(polys)
                trial_polys[idx] = cand_poly
                trial_geoms = list(geoms)
                trial_geoms[idx] = cand_geom
                cost = pack_tightness_cost(trial_geoms, outline, min_dist)
                if cost + 1e-9 < best_cost:
                    best_coords = candidate
                    best_cost = cost
            if best_coords is None:
                continue
            mx, my, mtheta = best_coords
            walked = border_refine_micro_walk(
                mx, my, mtheta, g,
                scene=scene,
                part_geom=bases[gid],
                others_geoms=others_geoms,
                outline=outline,
                edge_cfg=edge_cfg,
                min_dist=min_dist,
                eps=eps,
            )
            walk_poly = transform_poly(part_poly, walked)
            walk_geom = bases[gid].apply_transform(
                np.asarray(walked, dtype=np.float64),
            )
            if placement_ok_for_outline(
                scene,
                walk_geom,
                outline,
                others_geoms,
                min_dist,
                edge_cfg,
                epsilon_ratio=eps,
                require_outline_kiss=True,
            ):
                trial_polys = list(polys)
                trial_polys[idx] = walk_poly
                trial_geoms = list(geoms)
                trial_geoms[idx] = walk_geom
                walk_cost = pack_tightness_cost(trial_geoms, outline, min_dist)
                if walk_cost + 1e-9 < best_cost:
                    best_coords = walked
                    best_cost = walk_cost
            polys[idx] = transform_poly(part_poly, best_coords)
            trs[idx] = np.asarray(best_coords, dtype=np.float64)
            geoms[idx] = bases[gid].apply_transform(trs[idx])
            base_cost = best_cost
            improved = True
        if not improved:
            break

    return polys, gids, trs


def border_pack_graph(
    pack_polys: list,
    pack_gids: list[int],
    pack_tr: list[np.ndarray],
    *,
    board_geom: Geometry | None = None,
    placed_geoms: list[Geometry] | None = None,
    bases: dict[int, Geometry] | None = None,
) -> tuple[ElemGraph, list, list[int], list[np.ndarray], list[int]]:
    if board_geom is not None:
        kept_polys: list = []
        kept_gids: list[int] = []
        kept_tr: list[np.ndarray] = []
        kept_geoms: list[Geometry] = []
        for i, (poly, gid, tr) in enumerate(
            zip(pack_polys, pack_gids, pack_tr, strict=True)
        ):
            if placed_geoms is not None and i < len(placed_geoms):
                geom = placed_geoms[i]
            elif bases is not None and gid in bases:
                geom = bases[gid].apply_transform(np.asarray(tr, dtype=np.float64))
            else:
                geom = as_geometry(poly)
                if geom is None:
                    continue
            if geom.fully_inside(board_geom):
                kept_polys.append(poly)
                kept_gids.append(gid)
                kept_tr.append(tr)
                kept_geoms.append(geom)
        pack_polys, pack_gids, pack_tr = kept_polys, kept_gids, kept_tr
        placed_geoms = kept_geoms
    if placed_geoms is None:
        if bases is not None:
            placed_geoms = [
                bases[gid].apply_transform(np.asarray(tr, dtype=np.float64))
                for gid, tr in zip(pack_gids, pack_tr, strict=True)
            ]
        else:
            placed_geoms = []
            for p in pack_polys:
                g = as_geometry(p)
                if g is not None:
                    placed_geoms.append(g)
    graph = ElemGraph()
    graph.reserve_elems(len(placed_geoms))
    for n, geom in enumerate(placed_geoms):
        cx, cy = geom.center()
        graph.append_elem(
            pack_gids[n],
            Vec2(x=cx, y=cy),
            Circle.from_center_radius(cx, cy, geom.radius()),
        )
    hits = find_polygon_intersections(placed_geoms)
    for i, j in hits:
        graph.add_collision(i, j)
    selected_out = list(range(len(pack_polys)))
    assert selection_is_independent(graph, selected_out)
    return graph, pack_polys, pack_gids, pack_tr, selected_out


def first_pass_interior_fill(
    cfg: BuildGraphConfig,
    board: BaseGeometry,
    parts: list[tuple[Polygon, int]],
    *,
    pack_polys: list,
    pack_gids: list[int],
    pack_tr: list[np.ndarray],
    placed_geoms: list[Geometry],
    sheet: Polygon,
    board_geom: Geometry,
    voids: list,
    min_dist: float,
    eps: float,
    guidance_cfg: GuidanceConfig,
    part_by_gid: dict[int, Polygon],
    bases: dict[int, Geometry],
) -> tuple[list, list[int], list[np.ndarray], list[Geometry]]:
    """Add up to first_pass_interior_max non-outline parts after border augment."""
    interior_max = max(cfg.propose.first_pass_interior_max, 0)
    if interior_max <= 0:
        return pack_polys, pack_gids, pack_tr, placed_geoms

    propose_cfg = cfg.first_pass_propose_config().model_copy(deep=True)
    propose_cfg.ranking_mode = RankingMode.CONTACT_HYBRID
    propose_cfg.use_contact_ranking = True
    propose_cfg.place_profiles_enabled = True

    polys = list(pack_polys)
    gids = list(pack_gids)
    trs = list(pack_tr)
    geoms = list(placed_geoms)
    catalog = [(part_by_gid[gid], gid) for gid in part_by_gid]

    for _ in range(interior_max):
        densify_stats: dict = {}
        pocket_keys: dict = {}
        by_group = proposed_transforms_for_groups(
            board,
            catalog,
            polys,
            list(range(len(polys))),
            propose_cfg,
            min_dist=min_dist,
            use_full_packed_obstacle=True,
            packed_group_ids=gids,
            packed_transforms=trs,
            user_holes=cfg.rules.board_holes,
            pocket_keys_out=pocket_keys,
            densify_stats_out=densify_stats,
            seeded=False,
            full_packed_geoms=geoms if geoms else None,
        )
        pack_union = unary_union(polys) if polys else Polygon()
        push = propose_push_point(
            board,
            pack_union,
            smart_push=propose_cfg.smart_push_target,
            min_dist=min_dist,
            use_border_focus=False,
        )
        candidates: list[tuple[float, int, np.ndarray, Polygon, Geometry]] = []
        for gid, part_poly in part_by_gid.items():
            arr = by_group.get(gid)
            if arr is None or arr.shape[0] == 0:
                continue
            for row in arr:
                coords = np.asarray(row, dtype=np.float64).reshape(3)
                shapely_placed = transform_poly(part_poly, coords)
                geom = bases[gid].apply_transform(coords)
                scene = placement_scene_for_part(
                    sheet, board_geom, voids, bases[gid], base_geoms=geoms,
                )
                if not placement_ok_for_outline(
                    scene,
                    geom,
                    board,
                    geoms,
                    min_dist,
                    guidance_cfg,
                    epsilon_ratio=eps,
                    require_outline_kiss=False,
                ):
                    continue
                pg = ProposeGeometry(
                    board, pack_union, part_poly, min_dist,
                    epsilon_ratio=eps, propose_cfg=propose_cfg,
                )
                tight = score_placement_tightness(
                    (float(coords[0]), float(coords[1]), float(coords[2])),
                    pg, push, min_dist,
                )
                cost = -tight if tight > float("-inf") else float("inf")
                candidates.append((cost, gid, coords, shapely_placed, geom))
        if not candidates:
            break
        candidates.sort(key=lambda row: row[0])
        added: list[tuple[int, np.ndarray, Polygon, Geometry]] = []
        for cost, gid, coords, shapely_placed, geom in candidates:
            blocker_geoms = geoms + [row[3] for row in added]
            if not is_pose_clear(geom, board_geom, blocker_geoms, min_dist):
                continue
            added.append((gid, coords, shapely_placed, geom))
            break
        if not added:
            break
        for gid, coords, shapely_placed, geom in added:
            polys.append(shapely_placed)
            gids.append(gid)
            trs.append(coords)
            geoms.append(geom)

    return polys, gids, trs, geoms


def sequential_border_augment(
    cfg: BuildGraphConfig,
    board: BaseGeometry,
    parts: list[tuple[Polygon, int]],
    *,
    outline: BaseGeometry,
    polys: list,
    group_id: list[int],
    transform: list[np.ndarray],
    selected: list[int],
    skip_guidance_refine: bool = False,
) -> tuple[ElemGraph, list, list[int], list[np.ndarray], list[int]]:
    """Fill outline gaps by proposing against the full packed union each step."""
    min_dist = cfg.board_min_dist(first_pass=True)
    eps = cfg.placement_epsilon_ratio(first_pass=True)
    sheet, voids = board_context_from_geometry(board)
    board_geom = Geometry.from_shapely(sheet)
    pad = default_sheet_padding(board)
    guidance_cfg = guidance_config_for_graph(
        min_dist,
        board_bounds=padded_board_bounds(board, pad),
        epsilon_ratio=eps,
    )
    part_by_gid = {gid: poly for poly, gid in parts}
    bases = {gid: Geometry.from_shapely(part_by_gid[gid]) for gid in part_by_gid}

    pack_polys = [polys[i] for i in selected]
    pack_gids = [group_id[i] for i in selected]
    pack_tr = [transform[i] for i in selected]
    placed_geoms = [
        bases[gid].apply_transform(np.asarray(tr, dtype=np.float64))
        for gid, tr in zip(pack_gids, pack_tr, strict=True)
    ]
    max_rounds = max(cfg.propose.first_pass_sequential_augment_max, 0)

    for _ in range(max_rounds):
        candidates: list[tuple[float, int, np.ndarray, Polygon, Geometry]] = []
        for gid, part_poly in part_by_gid.items():
            for c in first_pass_border_coords(
                cfg, board, part_poly, pack_polys, min_dist=min_dist,
            ):
                coords = np.asarray(c, dtype=np.float64)
                shapely_placed = transform_poly(part_poly, coords)
                geom = bases[gid].apply_transform(coords)
                scene = placement_scene_for_part(
                    sheet, board_geom, voids, bases[gid], base_geoms=placed_geoms,
                )
                if not placement_ok_for_outline(
                    scene,
                    geom,
                    outline,
                    placed_geoms,
                    min_dist,
                    guidance_cfg,
                    epsilon_ratio=eps,
                    require_outline_kiss=True,
                ):
                    continue
                cost = abs(outline_standoff_distance(shapely_placed, outline) - min_dist)
                candidates.append((cost, gid, coords, shapely_placed, geom))
        if not candidates:
            break
        candidates.sort(key=lambda row: row[0])
        added: list[tuple[int, np.ndarray, Polygon, Geometry]] = []
        for cost, gid, coords, shapely_placed, geom in candidates:
            blocker_geoms = placed_geoms + [row[3] for row in added]
            if not is_pose_clear(geom, board_geom, blocker_geoms, min_dist):
                continue
            added.append((gid, coords, shapely_placed, geom))
        if not added:
            break
        for gid, coords, shapely_placed, geom in added:
            pack_polys.append(shapely_placed)
            pack_gids.append(gid)
            pack_tr.append(coords)
            placed_geoms.append(geom)

    pack_polys, pack_gids, pack_tr, placed_geoms = first_pass_interior_fill(
        cfg,
        board,
        parts,
        pack_polys=pack_polys,
        pack_gids=pack_gids,
        pack_tr=pack_tr,
        placed_geoms=placed_geoms,
        sheet=sheet,
        board_geom=board_geom,
        voids=voids,
        min_dist=min_dist,
        eps=eps,
        guidance_cfg=guidance_cfg,
        part_by_gid=part_by_gid,
        bases=bases,
    )

    if not skip_guidance_refine:
        pack_polys, pack_gids, pack_tr = guidance_border_refine(
            cfg,
            board,
            parts,
            outline=outline,
            pack_polys=pack_polys,
            pack_gids=pack_gids,
            pack_tr=pack_tr,
        )
        placed_geoms = [
            bases[gid].apply_transform(np.asarray(tr, dtype=np.float64))
            for gid, tr in zip(pack_gids, pack_tr, strict=True)
        ]
    return border_pack_graph(
        pack_polys,
        pack_gids,
        pack_tr,
        board_geom=board_geom,
        placed_geoms=placed_geoms,
        bases=bases,
    )


