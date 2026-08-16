"""Thin board ledger for Macro-MCTS (no polygon blobs)."""


from dataclasses import dataclass, field


@dataclass(slots=True)
class BoardSnapshot:
    packed_gids: tuple[int, ...] = ()
    packed_transforms: tuple[tuple[float, float, float], ...] = ()
    remaining_gids: tuple[int, ...] = ()
    coverage: float = 0.0
    arena_node_id: int = 0
    kiss_pairs: int = 0
    mean_compactness: float = 0.0
    rim_fill: float = 0.0
    void_fill: float = 0.0
    free_kind: str = ""
    motif_ids_used: tuple[int, ...] = ()
    telem: dict = field(default_factory=dict)
