"""Reproduce score_rules segfault on iter-4 graph (seed=0)."""
import faulthandler

faulthandler.enable()

import numpy as np

np.random.seed(0)

from shapely.geometry import Polygon

from nest_graph.build_graph import (
    improve_rules,
    make_polygon_graph,
    transform_history,
    transform_selection,
)
from nest_graph.elem_graph import (
    PlacementRuleSet,
    PointAngleRule,
    PointPlaceRule,
    Vec2,
    increase_score_dfs,
    increase_selection_dfs,
    nest_by_graph,
    score_elems,
    score_rules,
    sort_graph,
)
from nest_graph.utils import normalize_poly

p_board = Polygon([(0, 0), (1.2, 0), (0, 1.1)])
p1 = normalize_poly(Polygon([(0, 0), (0.1, 0), (0.1, 0.1), (0, 0.1)]))
p2 = normalize_poly(Polygon([(0, 0), (0.15, 0), (0, 0.07)]))

first_rule_set = PlacementRuleSet()
first_rule_set.append_rule(PointPlaceRule(pos=Vec2(x=0, y=0), r=0.1, w=0.1, group=0))
first_rule_set.append_rule(PointPlaceRule(pos=Vec2(x=0, y=0), r=0.1, w=0.1, group=1))
rule_sets = [first_rule_set]

rule_set = PlacementRuleSet()
for pos, g in [((0, 0), 0), ((0.7, 0.7), 1), ((0, 1.1), 1), ((1.2, 0), 1)]:
    rule_set.append_rule(
        PointPlaceRule(pos=Vec2(x=pos[0], y=pos[1]), r=0.2, w=1, group=g)
    )
for a in [np.pi / 4, np.pi * 5 / 4]:
    rule_set.append_rule(
        PointAngleRule(pos=Vec2(x=0, y=1.1), r=0.2, a=a, w=0.1, group=1)
    )
    rule_set.append_rule(
        PointAngleRule(pos=Vec2(x=1.2, y=0), r=0.2, a=a, w=0.1, group=1)
    )

selected_t = [
    np.random.rand(128, 3) * [1.5, 1.5, 2 * np.pi],
    np.random.rand(128, 3) * [1.5, 1.5, 2 * np.pi],
]
history = [np.zeros((1, 3)), np.zeros((1, 3))]
graphs = []


def run_iteration():
    global selected_t, history, graphs, rule_sets
    s0 = [np.random.rand(256, 3) * [1.5, 1.5, 2 * np.pi], history[0]]
    if selected_t[0].shape[0] > 0:
        s0.append(selected_t[0])
        s0.extend(transform_selection(selected_t[0], 4))
        s0.extend(transform_history(history[0], 2))
    s1 = [np.random.rand(256, 3) * [1.5, 1.5, 2 * np.pi], history[1]]
    if selected_t[1].shape[0] > 0:
        s1.append(selected_t[1])
        s1.extend(transform_selection(selected_t[1], 4))
        s1.extend(transform_history(history[1], 2))
    selected_t = [np.concatenate(s0), np.concatenate(s1)]
    graph, polys, gid, trans = make_polygon_graph(
        p_board, [(p1, selected_t[0]), (p2, selected_t[1])]
    )
    graphs.append(graph)
    graphs = graphs[-12:]
    for _ in range(8):
        rule_sets = improve_rules(graphs, rule_sets, 64, p_board)
    sel = nest_by_graph(graph, rule_sets[:1])[0]
    gs = sort_graph(graph, rule_set)
    gsr = sort_graph(graph, rule_set, reverse=True)
    sc = score_elems(graph, rule_set)
    for _ in range(2):
        sel = increase_selection_dfs(gsr, sel, 8)
        sel = increase_selection_dfs(gs, sel, 8)
        sel = increase_score_dfs(gsr, sel, sc)
        sel = increase_selection_dfs(gs, sel, 8)
        sel = increase_score_dfs(gs, sel, sc)
    selected_t = [[], []]
    for i in sel:
        selected_t[gid[i]].append(trans[i])
    selected_t = tuple(np.array(x) for x in selected_t)
    if len(history[0]) and len(selected_t[0]):
        history[0] = np.unique(
            np.concatenate([selected_t[0], history[0]]), axis=0
        )[5000:, :]
    return graph


for _ in range(3):
    run_iteration()

print("iter 4 graph")
g4 = run_iteration()
n = len(g4.group_id)
print("nodes", n)
for i, cs in enumerate(g4.collisions):
    for j in cs:
        if j < 0 or j >= n:
            raise SystemExit(f"bad collision {i} -> {j}")

print("score_rules on g4 + first rule set")
score_rules([g4], [rule_sets[0]])
print("ok")
