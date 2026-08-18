"""Wire Macro-MCTS into a nest job (cheap expand + best-leaf polish hook)."""


from dataclasses import dataclass, field
from typing import Any, Callable

from nest_graph.decision.mcts import MctsAgent
from nest_graph.decision.niche_archive import MacroNicheArchive
from nest_graph.decision.slave_pack import cheap_expand_slave
from nest_graph.decision.types import BoardSnapshot
from nest_graph.elem_graph import DecisionArena, MotifBase


@dataclass
class MacroMctsRunner:
    arena: DecisionArena = field(default_factory=DecisionArena)
    motif_base: MotifBase = field(default_factory=MotifBase)
    niche_archive: MacroNicheArchive = field(default_factory=MacroNicheArchive)
    agent: MctsAgent | None = None
    execute_fn: Callable[..., BoardSnapshot] | None = None

    def __post_init__(self) -> None:
        self.agent = MctsAgent(arena=self.arena, motif_base=self.motif_base)
        root = int(self.arena.root_id())
        self.arena.set_snapshot(root, BoardSnapshot(arena_node_id=root))

    def snapshot_at(
        self,
        node_id: int,
        default: BoardSnapshot | None = None,
        *,
        missing_ok: bool = False,
    ) -> BoardSnapshot | None:
        nid = int(node_id)
        if nid < 0 or nid >= int(self.arena.size()):
            if missing_ok or default is not None:
                return default
            raise IndexError(f"snapshot {nid} out of range")
        return self.arena.snapshot(nid)

    def store_snapshot(self, node_id: int, snap: BoardSnapshot) -> None:
        self.arena.set_snapshot(int(node_id), snap)

    def run(self, root_snapshot: BoardSnapshot, *, n_sims: int = 32) -> BoardSnapshot:
        assert self.agent is not None
        root = int(self.arena.root_id())
        root_snapshot.arena_node_id = root
        self.store_snapshot(root, root_snapshot)

        for _ in range(max(int(n_sims), 1)):
            leaf = self.agent.select_leaf()
            parent_snap = self.snapshot_at(leaf, root_snapshot)
            if not parent_snap.has_remaining:
                self.agent.backprop(leaf, float(parent_snap.coverage))
                continue
            action = self.agent.pick_expand_action(
                parent_snap.remaining_gids,
                parent_id=leaf,
                snapshot=parent_snap,
            )
            if action is None:
                break
            if not self.agent.may_expand_node(leaf):
                # Descend UCB child instead
                self.agent.backprop(leaf, float(parent_snap.coverage))
                continue
            result = cheap_expand_slave(
                parent_snap,
                action,
                motif_base=self.motif_base,
                execute_fn=self.execute_fn,
                telem=self.agent.telem,
            )
            child = self.agent.expand(leaf, action, result.reward)
            result.snapshot.arena_node_id = child
            self.store_snapshot(child, result.snapshot)

        best = self.agent.best_child()
        # Walk to deepest best-mean leaf
        cur = best
        while True:
            nxt = self.agent.best_child(cur)
            if nxt == cur:
                break
            cur = nxt
        return self.snapshot_at(cur, root_snapshot)

    def best_leaf_polish(self, snapshot: BoardSnapshot, polish_fn: Callable[[BoardSnapshot], BoardSnapshot]) -> BoardSnapshot:
        """Q69: heavy DFS/3b/local_se2 only here."""
        return polish_fn(snapshot)
