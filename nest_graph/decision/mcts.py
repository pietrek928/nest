"""UCB1 Macro-MCTS over C++ DecisionArena (cheap expand / best-leaf polish)."""


import math
import time
from dataclasses import dataclass, field

from nest_graph.decision.action_gen import generate_macros
from nest_graph.decision.types import BoardSnapshot
from nest_graph.elem_graph import DecisionArena, MacroAction, MotifBase, NodeSignature, related_distance


@dataclass(slots=True)
class MacroAmaf:
    visits: int = 0
    total_reward: float = 0.0


@dataclass
class MctsAgent:
    arena: DecisionArena
    motif_base: MotifBase
    amaf: dict[tuple, MacroAmaf] = field(default_factory=dict)
    pw_c: float = 1.5
    pw_alpha: float = 0.5
    ucb_c: float = 1.4
    telem: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.telem.setdefault("pw_expand", 0)
        self.telem.setdefault("amaf_hits", 0)
        self.telem.setdefault("expand_ms", 0.0)
        self.telem.setdefault("from_shapely_count", 0)

    def _action_key(self, action: MacroAction) -> tuple:
        region = action.region
        region_i = int(getattr(region, "value", region))
        return (region_i, int(action.rule_id), int(action.motif_id))

    def _ucb(self, node_id: int, parent_visits: int) -> float:
        visits = int(self.arena.visits(node_id))
        if visits <= 0:
            return float("inf")
        mean = float(self.arena.total_reward(node_id)) / visits
        explore = self.ucb_c * math.sqrt(math.log(max(parent_visits, 1) + 1) / visits)
        action = self.arena.action(node_id)
        key = self._action_key(action)
        amaf = self.amaf.get(key)
        if amaf is not None and amaf.visits > 0:
            self.telem["amaf_hits"] = int(self.telem["amaf_hits"]) + 1
            beta = amaf.visits / (amaf.visits + visits + 1e-9)
            amaf_mean = amaf.total_reward / amaf.visits
            mean = beta * amaf_mean + (1.0 - beta) * mean
        return mean + explore

    def select_leaf(self) -> int:
        node_id = int(self.arena.root_id())
        while True:
            first = int(self.arena.first_child_id(node_id))
            if first < 0:
                return node_id
            parent_visits = max(int(self.arena.visits(node_id)), 1)
            best_id = first
            best_score = -1e300
            child = first
            while child >= 0:
                score = self._ucb(child, parent_visits)
                if score > best_score:
                    best_score = score
                    best_id = child
                child = int(self.arena.next_sibling_id(child))
            if self.arena.may_expand(node_id, self.pw_c, self.pw_alpha):
                return node_id
            node_id = best_id

    def expand(
        self,
        parent_id: int,
        action: MacroAction,
        reward: float,
    ) -> int:
        child = int(self.arena.add_node(parent_id, action))
        self.backprop(child, reward)
        self.telem["pw_expand"] = int(self.telem["pw_expand"]) + 1
        return child

    def backprop(self, node_id: int, reward: float) -> None:
        cur = int(node_id)
        while cur >= 0:
            self.arena.record_visit(cur, float(reward))
            action = self.arena.action(cur)
            key = self._action_key(action)
            bucket = self.amaf.setdefault(key, MacroAmaf())
            bucket.visits += 1
            bucket.total_reward += float(reward)
            cur = int(self.arena.parent_id(cur))

    def best_child(self, node_id: int | None = None) -> int:
        root = int(self.arena.root_id() if node_id is None else node_id)
        first = int(self.arena.first_child_id(root))
        if first < 0:
            return root
        best_id = first
        best_mean = -1e300
        child = first
        while child >= 0:
            visits = int(self.arena.visits(child))
            if visits > 0:
                mean = float(self.arena.total_reward(child)) / visits
                if mean > best_mean:
                    best_mean = mean
                    best_id = child
            child = int(self.arena.next_sibling_id(child))
        return best_id

    def _warm_motif_ids(self, snapshot: BoardSnapshot) -> tuple[int, ...]:
        """Q79/Q101: rim/void scalars + MotifBase.find_nearest (no 8×8 this plan)."""
        from nest_graph.elem_graph import Se2
        from nest_graph.utils import relative_transform

        warm: list[int] = []
        seen: set[int] = set()

        def _add(mid: int) -> None:
            mid_i = int(mid)
            if mid_i < 0 or mid_i in seen or mid_i >= int(self.motif_base.size()):
                return
            seen.add(mid_i)
            warm.append(mid_i)

        if snapshot.packed_gids:
            cur = NodeSignature()
            cur.rim_fill = float(snapshot.rim_fill)
            cur.void_fill = float(snapshot.void_fill)
            best_ids: list[int] = []
            best_d = 1e9
            for snap in getattr(self, "_related_snaps", ()) or ():
                other = NodeSignature()
                other.rim_fill = float(snap.rim_fill)
                other.void_fill = float(snap.void_fill)
                d = float(related_distance(cur, other))
                if d < best_d and snap.motif_ids_used:
                    best_d = d
                    best_ids = list(snap.motif_ids_used)
            if best_d < 0.75:
                self.telem["related_warm"] = int(self.telem.get("related_warm", 0)) + 1
                for mid in best_ids:
                    _add(mid)

        gids = snapshot.packed_gids
        tfs = snapshot.packed_transforms
        if (
            self.motif_base is not None
            and int(self.motif_base.size()) > 0
            and len(gids) >= 2
            and len(tfs) >= 2
        ):
            ga, gb = int(gids[-2]), int(gids[-1])
            ta, tb = tfs[-2], tfs[-1]
            rel = relative_transform(
                (float(ta[0]), float(ta[1]), float(ta[2])),
                (float(tb[0]), float(tb[1]), float(tb[2])),
            )
            near_id = int(
                self.motif_base.find_nearest_id(
                    ga,
                    gb,
                    Se2(float(rel[0]), float(rel[1]), float(rel[2])),
                )
            )
            if near_id >= 0:
                _add(near_id)
                self.telem["nearest_warm"] = int(
                    self.telem.get("nearest_warm", 0)
                ) + 1
        return tuple(warm)

    def remember_related(self, snapshot: BoardSnapshot) -> None:
        snaps = list(getattr(self, "_related_snaps", ()) or ())
        snaps.append(snapshot)
        self._related_snaps = tuple(snaps[-32:])

    def _tried_action_keys(self, parent_id: int) -> set[tuple]:
        tried: set[tuple] = set()
        child = int(self.arena.first_child_id(parent_id))
        while child >= 0:
            tried.add(self._action_key(self.arena.action(child)))
            child = int(self.arena.next_sibling_id(child))
        return tried

    def pick_expand_action(
        self,
        remaining_gids: tuple[int, ...],
        *,
        rule_ids: tuple[int, ...] = (0,),
        parent_id: int | None = None,
        snapshot: BoardSnapshot | None = None,
    ) -> MacroAction | None:
        warm = self._warm_motif_ids(snapshot) if snapshot is not None else ()
        free_kind = str(getattr(snapshot, "free_kind", "") or "") if snapshot else ""
        actions = generate_macros(
            remaining_gids,
            rule_ids=rule_ids,
            motif_base=self.motif_base,
            prefer_motifs=True,
            warm_motif_ids=warm,
            free_kind=free_kind,
        )
        if not actions:
            return None
        if parent_id is None:
            return actions[0]
        tried = self._tried_action_keys(int(parent_id))
        untried = [a for a in actions if self._action_key(a) not in tried]
        if not untried:
            return actions[0]
        # D1: AMAF among untried (unvisited AMAF → optimistic prior).
        best = untried[0]
        best_score = -1e300
        for a in untried:
            key = self._action_key(a)
            amaf = self.amaf.get(key)
            if amaf is None or amaf.visits <= 0:
                score = 1e9  # never tried globally → explore first
            else:
                score = float(amaf.total_reward) / float(amaf.visits)
            if score > best_score:
                best_score = score
                best = a
        self.telem["amaf_pick"] = int(self.telem.get("amaf_pick", 0)) + 1
        if best_score < 1e8:
            self.telem["amaf_hits"] = int(self.telem.get("amaf_hits", 0)) + 1
        return best


def leaf_reward(
    snapshot: BoardSnapshot,
    *,
    lam_kiss: float = 0.05,
    lam_comp: float = 0.05,
    lam_void: float = 0.5,
    lam_rim: float = 0.1,
) -> float:
    """Coverage + void/rim fills (Dg1) + optional kiss/comp."""
    packed = max(len(snapshot.packed_gids), 1)
    return (
        float(snapshot.coverage)
        + float(lam_void) * float(snapshot.void_fill)
        + float(lam_rim) * float(snapshot.rim_fill)
        + lam_kiss * (float(snapshot.kiss_pairs) / packed)
        + lam_comp * float(snapshot.mean_compactness)
    )


def timed_expand_ms(telem: dict, t0: float) -> None:
    telem["expand_ms"] = float(telem.get("expand_ms", 0.0)) + (time.perf_counter() - t0) * 1000.0
