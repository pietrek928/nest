"""MacroNicheArchive: per-AMAF-key abs-SE2 history rings (Path C banned)."""

from collections import deque
from dataclasses import dataclass, field

import numpy as np

NICHE_RING_H = 6


@dataclass(slots=True)
class NicheEpisode:
    polarity: int  # +1 void gain, -1 hollow miss
    void_nest: int = 0
    score: float = 0.0
    # + only: list of (gid, x, y, theta); - is metadata-only
    rows: tuple[tuple[int, float, float, float], ...] = ()


@dataclass
class MacroNicheBucket:
    hits: int = 0
    misses: int = 0
    best_void_nest: int = 0
    ttl_remaining: int = 4
    ring: deque[NicheEpisode] = field(default_factory=lambda: deque(maxlen=NICHE_RING_H))
    # flattened positive survivors: gid -> rows (N,3)
    active_rows: dict[int, list[np.ndarray]] = field(default_factory=dict)

    @property
    def miss_rate(self) -> float:
        tot = int(self.hits) + int(self.misses)
        if tot <= 0:
            return 0.0
        return float(self.misses) / float(tot)


@dataclass
class MacroNicheArchive:
    """Abs SE2 niches keyed like AMAF ``(region, rule_id, motif_id)``."""

    buckets: dict[tuple, MacroNicheBucket] = field(default_factory=dict)
    place_fail_streak: int = 0
    last_feed_keys: set[tuple[float, float, float]] = field(default_factory=set)
    max_rows_per_episode: int = 64

    def get(self, key: tuple) -> MacroNicheBucket:
        return self.buckets.setdefault(key, MacroNicheBucket())

    def append_positive(
        self,
        key: tuple,
        *,
        rows: list[tuple[int, float, float, float]],
        void_nest: int,
        score: float = 0.0,
        ttl: int = 4,
    ) -> None:
        bucket = self.get(key)
        capped = tuple(rows[: max(int(self.max_rows_per_episode), 0)])
        bucket.ring.append(
            NicheEpisode(
                polarity=1,
                void_nest=int(void_nest),
                score=float(score),
                rows=capped,
            )
        )
        bucket.hits += 1
        bucket.best_void_nest = max(int(bucket.best_void_nest), int(void_nest))
        bucket.ttl_remaining = max(int(ttl), 1)
        for gid, x, y, th in capped:
            arr = np.asarray([x, y, th], dtype=np.float64).reshape(3)
            bucket.active_rows.setdefault(int(gid), []).append(arr)
            # keep per-gid cap light
            if len(bucket.active_rows[int(gid)]) > int(self.max_rows_per_episode):
                bucket.active_rows[int(gid)] = bucket.active_rows[int(gid)][
                    -int(self.max_rows_per_episode) :
                ]

    def append_negative(
        self,
        key: tuple,
        *,
        void_nest: int = 0,
        score: float = 0.0,
    ) -> None:
        bucket = self.get(key)
        bucket.ring.append(
            NicheEpisode(
                polarity=-1,
                void_nest=int(void_nest),
                score=float(score),
                rows=(),
            )
        )
        bucket.misses += 1

    def any_void_miss_rate_high(self, threshold: float = 0.8) -> bool:
        for bucket in self.buckets.values():
            if bucket.hits + bucket.misses <= 0:
                continue
            if float(bucket.miss_rate) > float(threshold):
                return True
        return False

    def active_by_group(self, ngroups: int) -> dict[int, list[np.ndarray]]:
        out: dict[int, list[np.ndarray]] = {}
        for bucket in self.buckets.values():
            if bucket.hits <= 0 or bucket.ttl_remaining <= 0:
                continue
            for gid, rows in bucket.active_rows.items():
                if int(gid) < 0 or int(gid) >= int(ngroups):
                    continue
                out.setdefault(int(gid), []).extend(rows)
        return out

    def age(self, step: int = 1) -> int:
        """Decrement TTL; drop sterile (all-miss, ttl exhausted) buckets."""
        dropped = 0
        dead: list[tuple] = []
        for key, bucket in self.buckets.items():
            bucket.ttl_remaining -= int(step)
            if bucket.ttl_remaining > 0:
                continue
            if bucket.hits > 0 and bucket.best_void_nest > 0:
                bucket.ttl_remaining = 1  # protect proven niches
                continue
            dead.append(key)
            dropped += 1
        for key in dead:
            del self.buckets[key]
        return dropped

    def note_place_outcome(self, placed: bool) -> None:
        if placed:
            self.place_fail_streak = 0
        else:
            self.place_fail_streak = int(self.place_fail_streak) + 1
