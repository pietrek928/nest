"""UCB1 Macro-MCTS over C++ DecisionArena (cheap expand / best-leaf polish)."""

import math
import time
from dataclasses import dataclass, field
from typing import Any

from nest_graph.decision.action_gen import generate_macros
from nest_graph.decision.types import BoardSnapshot
from nest_graph.elem_graph import DecisionArena, MacroAction, MotifBase, NodeSignature, related_distance
from nest_graph.elem_graph import MacroRegion

LAMBDA_MISS = 0.5


def _empty_realized() -> dict:
    return {
        "kind": (0, 0, 0, 0),
        "attach": 0,
        "member_hits": 0,
        "sel_n": 0,
    }


@dataclass
class MctsAgent:
    arena: DecisionArena
    motif_base: MotifBase
    niche_archive: Any | None = None
    pw_c: float = 1.5
    pw_alpha: float = 0.5
    ucb_c: float = 1.4
    expand_frozen: bool = False
    telem: dict = field(default_factory=dict)
    realized: dict = field(default_factory=_empty_realized)
    motif_cohorts: tuple = ()
    _tombstoned: set[int] = field(default_factory=set)
    _idle_age: dict[int, int] = field(default_factory=dict)
    _related_snaps: tuple = ()
    _cohort_amaf: dict[tuple, dict] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.telem.setdefault("pw_expand", 0)
        self.telem.setdefault("amaf_hits", 0)
        self.telem.setdefault("expand_ms", 0.0)
        self.telem.setdefault("from_shapely_count", 0)
        self.telem.setdefault("amaf_miss", 0)
        self.telem.setdefault("browse_jump", 0)
        self.telem.setdefault("tombstone_n", 0)
        self.telem.setdefault("cohort_sig_amaf_visits", 0)
        self.telem.setdefault("mcts_cohort_macro_n", 0)

    def _cohort_sig_for_action(self, action: MacroAction) -> tuple:
        mid = int(getattr(action, "motif_id", -1) or -1)
        if mid < 0:
            return ()
        leader = int(getattr(action, "part_gid", -1) or -1)
        for c in self.motif_cohorts or ():
            if not isinstance(c, dict):
                continue
            if int(c.get("motif_id", -1) or -1) != mid:
                continue
            if int(c.get("leader_gid", -1) or -1) != leader:
                continue
            lk = c.get("leader_key")
            key_t = tuple(lk) if isinstance(lk, tuple) else None
            return (mid, leader, key_t)
        return (mid, leader, None)

    def _action_key(self, action: MacroAction) -> tuple:
        region = action.region
        region_i = int(getattr(region, "value", region))
        cohort_sig = self._cohort_sig_for_action(action)
        return (region_i, int(action.rule_id), int(action.motif_id), cohort_sig)

    def _record_cohort_amaf(self, action: MacroAction, reward: float, miss: bool) -> None:
        sig = self._cohort_sig_for_action(action)
        if not sig:
            return
        key = self._action_key(action)
        slot = self._cohort_amaf.setdefault(
            key, {"visits": 0, "reward": 0.0, "misses": 0},
        )
        slot["visits"] = int(slot["visits"]) + 1
        slot["reward"] = float(slot["reward"]) + float(reward)
        if miss:
            slot["misses"] = int(slot["misses"]) + 1
        self.telem["cohort_sig_amaf_visits"] = max(
            int(self.telem.get("cohort_sig_amaf_visits", 0)),
            int(slot["visits"]),
        )

    def _is_tombstoned(self, node_id: int) -> bool:
        return int(node_id) in self._tombstoned

    def may_expand_node(self, node_id: int) -> bool:
        if self.expand_frozen:
            return False
        return bool(self.arena.may_expand(int(node_id), self.pw_c, self.pw_alpha))

    def _ucb(self, node_id: int, parent_visits: int) -> float:
        if self._is_tombstoned(node_id):
            return -1e300
        action = self.arena.action(node_id)
        region_i = int(getattr(action.region, "value", action.region))
        if int(self.arena.amaf_visits(region_i, int(action.rule_id), int(action.motif_id))) > 0:
            self.telem["amaf_hits"] = int(self.telem["amaf_hits"]) + 1
        score = float(
            self.arena.ucb_score(int(node_id), int(parent_visits), float(self.ucb_c))
        )
        if score == float("inf") or score > 1e299:
            return float("inf")
        return score

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
            any_live = False
            while child >= 0:
                if not self._is_tombstoned(child):
                    any_live = True
                    score = self._ucb(child, parent_visits)
                    if score > best_score:
                        best_score = score
                        best_id = child
                child = int(self.arena.next_sibling_id(child))
            if not any_live:
                return node_id
            if self.may_expand_node(node_id):
                return node_id
            node_id = best_id

    def expand(
        self,
        parent_id: int,
        action: MacroAction,
        reward: float,
    ) -> int:
        if self.expand_frozen:
            self.backprop(int(parent_id), reward)
            return int(parent_id)
        child = int(self.arena.add_node(parent_id, action))
        self.backprop(child, reward)
        self.telem["pw_expand"] = int(self.telem["pw_expand"]) + 1
        self._idle_age[child] = 0
        return child

    def backprop(self, node_id: int, reward: float) -> None:
        cur = int(node_id)
        while cur >= 0:
            self.arena.record_visit(cur, float(reward))
            self._idle_age[cur] = 0
            action = self.arena.action(cur)
            region_i = int(getattr(action.region, "value", action.region))
            self.arena.amaf_record(
                region_i, int(action.rule_id), int(action.motif_id), float(reward), False,
            )
            self._record_cohort_amaf(action, float(reward), False)
            cur = int(self.arena.parent_id(cur))

    def note_macro_miss(self, action: MacroAction | None) -> None:
        if action is None:
            return
        region_i = int(getattr(action.region, "value", action.region))
        self.arena.amaf_record(
            region_i, int(action.rule_id), int(action.motif_id), 0.0, True,
        )
        self._record_cohort_amaf(action, 0.0, True)
        self.telem["amaf_miss"] = int(self.telem.get("amaf_miss", 0)) + 1

    def best_child(self, node_id: int | None = None) -> int:
        root = int(self.arena.root_id() if node_id is None else node_id)
        first = int(self.arena.first_child_id(root))
        if first < 0:
            return root
        best_id = root
        best_mean = -1e300
        found = False
        child = first
        while child >= 0:
            if not self._is_tombstoned(child):
                visits = int(self.arena.visits(child))
                if visits > 0:
                    mean = float(self.arena.total_reward(child)) / visits
                    if mean > best_mean:
                        best_mean = mean
                        best_id = child
                        found = True
            child = int(self.arena.next_sibling_id(child))
        return best_id if found else root

    def deepest_best_child(self) -> int:
        cur = int(self.arena.root_id())
        while True:
            nxt = self.best_child(cur)
            if nxt == cur:
                return cur
            cur = nxt

    def ancestor_chain(self, node_id: int) -> set[int]:
        out: set[int] = set()
        cur = int(node_id)
        while cur >= 0:
            out.add(cur)
            cur = int(self.arena.parent_id(cur))
        return out

    def age_and_tombstone(
        self,
        *,
        spine_id: int,
        idle_t: int = 4,
        force: bool = False,
    ) -> int:
        """Tombstone off-spine / non-best leaves with visits<2 and idle≥T."""
        protect = self.ancestor_chain(int(spine_id))
        protect |= self.ancestor_chain(self.deepest_best_child())
        protect.add(int(self.arena.root_id()))
        n = int(self.arena.size())
        dropped = 0
        for nid in range(n):
            if nid in protect or self._is_tombstoned(nid):
                continue
            age = int(self._idle_age.get(nid, 0)) + 1
            self._idle_age[nid] = age
            visits = int(self.arena.visits(nid))
            if visits < 2 and (age >= int(idle_t) or force):
                self._tombstoned.add(nid)
                dropped += 1
        self.telem["tombstone_n"] = int(len(self._tombstoned))
        return dropped

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

    def remember_related(self, snapshot: BoardSnapshot, *, allow: bool = True) -> None:
        if not allow:
            return
        snaps = list(getattr(self, "_related_snaps", ()) or ())
        snaps.append(snapshot)
        self._related_snaps = tuple(snaps[-32:])

    def _tried_action_keys(self, parent_id: int) -> set[tuple]:
        tried: set[tuple] = set()
        child = int(self.arena.first_child_id(parent_id))
        while child >= 0:
            if not self._is_tombstoned(child):
                tried.add(self._action_key(self.arena.action(child)))
            child = int(self.arena.next_sibling_id(child))
        return tried

    def _amaf_pick_score(
        self,
        key: tuple,
        parent_id: int,
        *,
        free_kind: str = "",
    ) -> float:
        """Q131 Progressive Bias; soft miss penalty (L1). Never use 1e9."""
        parent_visits = max(int(self.arena.visits(int(parent_id))), 0)
        region_i = int(key[0])
        rule_id = int(key[1])
        motif_id = int(key[2])
        cohort_sig = key[3] if len(key) > 3 else ()
        visits = int(self.arena.amaf_visits(region_i, rule_id, motif_id))
        mean = 0.0
        misses = 0
        if cohort_sig:
            slot = self._cohort_amaf.get(key)
            if slot and int(slot.get("visits", 0)) > 0:
                visits = int(slot["visits"])
                mean = float(slot["reward"]) / float(visits)
                misses = int(slot.get("misses", 0))
                self.telem["cohort_sig_amaf_visits"] = max(
                    int(self.telem.get("cohort_sig_amaf_visits", 0)),
                    visits,
                )
            else:
                visits = 0
        elif visits <= 0:
            visits = 0
        else:
            mean = float(self.arena.amaf_mean(region_i, rule_id, motif_id))
            misses = int(self.arena.amaf_misses(region_i, rule_id, motif_id))
        c = float(self.ucb_c)
        pb = c / math.sqrt(float(parent_visits) + 1.0)
        realized = self.realized or {}
        survive_map = realized.get("survive_by_motif") or {}
        survival_live = int(realized.get("survive_motif_n", 0) or 0) > 0 or any(
            int(v) > 0 for v in (survive_map or {}).values()
        )
        amaf_scale = 0.65 if survival_live else 1.0
        if survival_live:
            self.telem["amaf_survive_scale"] = float(amaf_scale)
        if visits <= 0:
            score = pb * amaf_scale
        else:
            miss_term = 0.0
            if misses > 0:
                miss_term = LAMBDA_MISS * (float(misses) / float(visits))
            score = (mean - miss_term + pb) * amaf_scale
        # Prefer Void macros under large_void (anti hollow-rim).
        void_i = int(getattr(MacroRegion.Void, "value", 1))
        rim_i = int(getattr(MacroRegion.Rim, "value", 0))
        motif_i = int(getattr(MacroRegion.Motif, "value", 3))
        if str(free_kind) == "large_void" and region_i == void_i:
            score += 0.35
        if str(free_kind) == "large_void" and region_i == rim_i:
            score -= 0.15
        kind_t = tuple(int(x) for x in (realized.get("kind") or (0, 0, 0, 0)))
        sel_n = max(int(realized.get("sel_n", 0) or 0), 1)
        if 0 <= region_i < 4 and region_i < len(kind_t):
            score += 0.2 * (float(kind_t[region_i]) / float(sel_n))
        attach_n = int(realized.get("attach", 0) or 0)
        member_hits = int(realized.get("member_hits", 0) or 0)
        mat_motif = int(realized.get("materialized_motif", 0) or 0)
        survive_map = realized.get("survive_by_motif") or {}
        survive_n = int(survive_map.get(motif_id, 0) or 0) if motif_id >= 0 else 0
        if survive_n <= 0:
            survive_n = int(realized.get("survive_motif_n", 0) or 0)
        if region_i in (void_i, motif_i, rim_i):
            score += 0.1 * (float(attach_n) / float(sel_n))
        if member_hits > 0 and region_i == motif_i:
            score += 0.08 * min(float(member_hits), 16.0) / 16.0
        if mat_motif > 0 and region_i == motif_i:
            score += 0.1 * min(float(mat_motif), 4.0) / 4.0
        # Q383: MotifJoin survival blend into AMAF pick.
        if survive_n > 0 and region_i in (void_i, motif_i):
            score += 0.12 * (float(survive_n) / float(sel_n))
        prior_graph_hit = int(getattr(self, "prior_motif_graph_hit_n", 0) or 0)
        if prior_graph_hit > 0 and region_i == motif_i:
            score += 0.05 * min(float(prior_graph_hit), 4.0) / 4.0
        if (
            motif_id >= 0
            and self.motif_base is not None
            and motif_id < int(self.motif_base.size())
        ):
            rec = self.motif_base.at(motif_id)
            score += 0.04 * min(float(int(rec.accept_count)), 8.0)
            score += 0.12 * float(rec.gci)
        arch = self.niche_archive
        buckets = getattr(arch, "buckets", None) if arch is not None else None
        raw = None
        if buckets:
            raw = buckets.get((region_i, rule_id, motif_id))
            if raw is None:
                raw = buckets.get((region_i, 0, motif_id))
        if raw:
            tot = int(raw.get("hits", 0) or 0) + int(raw.get("misses", 0) or 0)
            if tot > 0:
                score -= 0.2 * (float(raw.get("misses", 0) or 0) / float(tot))
        prop_h = float((realized or {}).get("proposer_pb", 0.0) or 0.0)
        prop_h = max(0.0, min(1.0, prop_h))
        if prop_h > 0.0:
            den = float(visits) + 1.0 if visits > 0 else float(parent_visits) + 1.0
            score += prop_h / den
        return score

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
            motif_cohorts=self.motif_cohorts or None,
        )
        cohort_n = sum(
            1 for a in actions
            if self._cohort_sig_for_action(a) and int(getattr(a, "motif_id", -1) or -1) >= 0
        )
        if cohort_n > 0:
            self.telem["mcts_cohort_macro_n"] = int(cohort_n)
        if not actions:
            return None
        if parent_id is None:
            return actions[0]
        tried = self._tried_action_keys(int(parent_id))
        untried = [a for a in actions if self._action_key(a) not in tried]
        if not untried:
            return actions[0]
        best = untried[0]
        best_score = -1e300
        for a in untried:
            key = self._action_key(a)
            score = self._amaf_pick_score(
                key, int(parent_id), free_kind=free_kind
            )
            if score > best_score:
                best_score = score
                best = a
        self.telem["amaf_pick"] = int(self.telem.get("amaf_pick", 0)) + 1
        key = self._action_key(best)
        if int(self.arena.amaf_visits(int(key[0]), int(key[1]), int(key[2]))) > 0:
            self.telem["amaf_hits"] = int(self.telem.get("amaf_hits", 0)) + 1
        return best


def coverage_delta(child: BoardSnapshot, parent: BoardSnapshot) -> float:
    """Normalized coverage gain (child vs parent)."""
    pc = max(float(getattr(parent, "coverage", 0.0) or 0.0), 1e-9)
    cc = float(getattr(child, "coverage", 0.0) or 0.0)
    return (cc - pc) / pc


def path_reward_beats(
    parent: BoardSnapshot,
    child: BoardSnapshot,
    *,
    base_reward: float,
    alt_reward: float,
    min_abs_cov: float = 0.005,
    min_reward_eps: float = 0.01,
) -> bool:
    """Path accept: reward edge + normalized coverage delta (not absolute cov)."""
    if alt_reward <= float(base_reward) + float(min_reward_eps):
        return False
    bc = float(getattr(parent, "coverage", 0.0) or 0.0)
    cc = float(getattr(child, "coverage", 0.0) or 0.0)
    if cc + 1e-9 >= bc + float(min_abs_cov):
        return True
    return coverage_delta(child, parent) >= float(min_abs_cov) / max(bc, 1e-9)


def leaf_reward(
    snapshot: BoardSnapshot,
    *,
    lam_kiss: float = 0.05,
    lam_comp: float = 0.05,
    lam_void: float = 0.5,
    lam_rim: float = 0.1,
    rule_id: int = -1,
    member_hits: int = 0,
    materialized_motif: int = 0,
    survive_motif_n: int = 0,
    macro_survive_n: int = 0,
) -> float:
    """Coverage + void/rim fills (Dg1) + optional kiss/comp.

    Q108: under ``large_void``, raise void λ and cut rim so AMAF prefers
    colonization over hollow-rim kiss cosmetics.
    """
    packed = max(int(snapshot.n_packed), 1)
    void_w = float(lam_void)
    rim_w = float(lam_rim)
    if str(snapshot.free_kind or "") == "large_void":
        void_w = max(void_w, 0.75)
        rim_w = min(rim_w, 0.05)
    base = (
        float(snapshot.coverage)
        + void_w * float(snapshot.void_fill)
        + rim_w * float(snapshot.rim_fill)
        + lam_kiss * (float(snapshot.kiss_pairs) / packed)
        + lam_comp * float(snapshot.mean_compactness)
    )
    if int(rule_id) > 0:
        base += 0.02 * min(float(int(rule_id)), 3.0)
    mh = max(int(member_hits), 0)
    mm = max(int(materialized_motif), 0)
    if mh > 0:
        base += 0.03 * min(float(mh), 12.0) / 12.0
    if mm > 0:
        base += 0.05 * min(float(mm), 4.0) / 4.0
    sn = max(int(survive_motif_n), 0)
    msn = max(int(macro_survive_n), 0)
    if sn > 0:
        base += 0.04 * min(float(sn), 8.0) / 8.0
    if msn > 0:
        base += 0.06 * min(float(msn), 4.0) / 4.0
    return base


def timed_expand_ms(telem: dict, t0: float) -> None:
    telem["expand_ms"] = float(telem.get("expand_ms", 0.0)) + (time.perf_counter() - t0) * 1000.0
