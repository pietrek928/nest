"""Proposer registry: standard runner signature for collect_propose_candidates."""

from collections.abc import Callable

from nest_graph.propose.query_context import ProposeContext

ProposerRunner = Callable[[ProposeContext], list[tuple[float, float, float]]]
