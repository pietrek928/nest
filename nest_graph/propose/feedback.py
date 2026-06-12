from dataclasses import dataclass, field


@dataclass
class ProposeFeedbackState:
    proposer_pool_scales: dict[str, float] = field(default_factory=dict)
    zone_graph_yields: dict[str, list[float]] = field(default_factory=dict)
    last_proposer_counts: dict[str, int] = field(default_factory=dict)
    last_graph_yield: float = 0.0

    def record_iteration(
        self,
        *,
        proposer_counts: dict[str, int],
        graph_yield: float,
        zone: str | None = None,
    ) -> None:
        self.last_proposer_counts = dict(proposer_counts)
        self.last_graph_yield = graph_yield
        if zone:
            self.zone_graph_yields.setdefault(zone, []).append(graph_yield)
            if len(self.zone_graph_yields[zone]) > 8:
                self.zone_graph_yields[zone] = self.zone_graph_yields[zone][-8:]
        if graph_yield < 0.5 and proposer_counts:
            total = sum(proposer_counts.values()) or 1
            for name, count in proposer_counts.items():
                share = count / total
                scale = self.proposer_pool_scales.get(name, 1.0)
                if share > 0.3 and graph_yield < 0.4:
                    scale = max(0.5, scale * 0.9)
                elif share < 0.05 and graph_yield > 0.7:
                    scale = min(2.0, scale * 1.05)
                self.proposer_pool_scales[name] = scale
