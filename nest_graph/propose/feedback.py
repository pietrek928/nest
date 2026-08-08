from dataclasses import dataclass, field


@dataclass
class ProposeFeedbackState:
    proposer_pool_scales: dict[str, float] = field(default_factory=dict)
    zone_graph_yields: dict[str, list[float]] = field(default_factory=dict)
    last_proposer_counts: dict[str, int] = field(default_factory=dict)
    last_graph_yield: float = 0.0
    last_proposal_yield: float = 0.0
    last_refine_by_proposer: dict[str, int] = field(default_factory=dict)

    def record_iteration(
        self,
        *,
        proposer_counts: dict[str, int],
        graph_yield: float,
        proposal_yield: float | None = None,
        zone: str | None = None,
        refine_by_proposer: dict[str, int] | None = None,
        emitted_by_proposer: dict[str, int] | None = None,
        use_ema: bool = False,
        ema_alpha: float = 0.15,
        ema_floor: float = 0.05,
    ) -> None:
        self.last_proposer_counts = dict(proposer_counts)
        self.last_graph_yield = graph_yield
        if proposal_yield is not None:
            self.last_proposal_yield = proposal_yield
        if refine_by_proposer is not None:
            self.last_refine_by_proposer = dict(refine_by_proposer)
        if zone:
            self.zone_graph_yields.setdefault(zone, []).append(graph_yield)
            if len(self.zone_graph_yields[zone]) > 8:
                self.zone_graph_yields[zone] = self.zone_graph_yields[zone][-8:]

        if use_ema and (emitted_by_proposer or refine_by_proposer):
            self._update_ema_scales(
                emitted_by_proposer or {},
                refine_by_proposer or {},
                alpha=ema_alpha,
                floor=ema_floor,
            )
            return

        yield_signal = proposal_yield if proposal_yield is not None else graph_yield
        if yield_signal < 0.5 and proposer_counts:
            # Ignore telemetry meta keys (prefixed with _).
            usable = {k: v for k, v in proposer_counts.items() if not str(k).startswith("_")}
            total = sum(usable.values()) or 1
            for name, count in usable.items():
                share = count / total
                scale = self.proposer_pool_scales.get(name, 1.0)
                if share > 0.3 and yield_signal < 0.4:
                    scale = max(0.5, scale * 0.9)
                elif share < 0.05 and yield_signal > 0.7:
                    scale = min(2.0, scale * 1.05)
                self.proposer_pool_scales[name] = scale

    def _update_ema_scales(
        self,
        emitted: dict[str, int],
        refine: dict[str, int],
        *,
        alpha: float,
        floor: float,
    ) -> None:
        names = set(emitted) | set(refine) | set(self.proposer_pool_scales)
        a = min(max(float(alpha), 0.01), 1.0)
        fl = min(max(float(floor), 0.0), 1.0)
        for name in names:
            if str(name).startswith("_"):
                continue
            e = int(emitted.get(name, 0))
            r = int(refine.get(name, 0))
            survival = (r / e) if e > 0 else 0.0
            old = float(self.proposer_pool_scales.get(name, 1.0))
            new = a * survival + (1.0 - a) * old
            # Map survival EMA into a [floor, 2] budget multiplier centered at 1.
            # High survival → scale up toward 2; low → toward floor.
            budget = fl + (2.0 - fl) * min(max(new, 0.0), 1.0)
            self.proposer_pool_scales[name] = max(fl, min(2.0, budget))
