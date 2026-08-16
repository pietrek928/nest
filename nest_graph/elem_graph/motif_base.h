#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <tuple>
#include <vector>

#include "se2.h"

/** In-RAM motif library with geometric-hash keys (order-invariant gid pair + round-3 relative). */

struct MotifRecord {
    int32_t gid_a = -1;
    int32_t gid_b = -1;
    Se2 relative{};
    float gci = 0.f;
    float compactness = 0.f;
    int32_t accept_count = 0;
    float area_a = 0.f;
    float area_b = 0.f;
    int32_t ttl_remaining = 0;  // Q92: 0 = not on archive TTL; >0 alive
};

inline void motif_order_invariant(
    int32_t &gid_a,
    int32_t &gid_b,
    Se2 &relative,
    float &area_a,
    float &area_b
) {
    if (area_b > area_a + 1e-12f
        || (std::fabs(area_b - area_a) <= 1e-12f && gid_b < gid_a)) {
        // Swap: B becomes A; relative for swapped pair is invert(relative).
        std::swap(gid_a, gid_b);
        std::swap(area_a, area_b);
        relative = se2_invert(relative);
    } else if (gid_b < gid_a && std::fabs(area_b - area_a) <= 1e-12f) {
        std::swap(gid_a, gid_b);
        relative = se2_invert(relative);
    }
}

inline std::tuple<int32_t, int32_t, int32_t, int32_t, int32_t> motif_key(const MotifRecord &m) {
    Se2 rel = m.relative;
    int32_t ga = m.gid_a;
    int32_t gb = m.gid_b;
    float aa = m.area_a;
    float ab = m.area_b;
    motif_order_invariant(ga, gb, rel, aa, ab);
    const auto q = se2_key3(rel);
    return {ga, gb, std::get<0>(q), std::get<1>(q), std::get<2>(q)};
}

class MotifBase {
public:
    /**
     * Upsert if compactness >= min_compactness; rank by gci (Q77/Q78).
     * When ttl > 0, reset ttl_remaining on accept (Q92 archival path).
     */
    int32_t upsert(MotifRecord rec, float min_compactness = 0.f, int32_t ttl = 0) {
        if (rec.compactness < min_compactness) {
            return -1;
        }
        motif_order_invariant(rec.gid_a, rec.gid_b, rec.relative, rec.area_a, rec.area_b);
        const auto key = motif_key(rec);
        for (std::size_t i = 0; i < motifs_.size(); ++i) {
            if (motif_key(motifs_[i]) == key) {
                MotifRecord &dst = motifs_[i];
                dst.accept_count += 1;
                if (rec.gci > dst.gci) {
                    dst.gci = rec.gci;
                    dst.compactness = rec.compactness;
                    dst.relative = rec.relative;
                }
                if (ttl > 0) {
                    dst.ttl_remaining = ttl;
                }
                return static_cast<int32_t>(i);
            }
        }
        rec.accept_count = 1;
        if (ttl > 0) {
            rec.ttl_remaining = ttl;
        }
        motifs_.push_back(rec);
        return static_cast<int32_t>(motifs_.size() - 1);
    }

    /** Decrement TTL; drop records with ttl_remaining <= 0 that were TTL-tracked. */
    int32_t age(int32_t step = 1) {
        int32_t dropped = 0;
        std::vector<MotifRecord> kept;
        kept.reserve(motifs_.size());
        for (auto &m : motifs_) {
            if (m.ttl_remaining > 0) {
                m.ttl_remaining -= step;
                if (m.ttl_remaining <= 0) {
                    ++dropped;
                    continue;
                }
            }
            kept.push_back(m);
        }
        motifs_.swap(kept);
        return dropped;
    }

    /** Truncate to max_keep by accept_count then gci (Q92). Alive TTL preferred. */
    void truncate(int32_t max_keep) {
        if (max_keep <= 0 || static_cast<int32_t>(motifs_.size()) <= max_keep) {
            return;
        }
        std::vector<std::size_t> order(motifs_.size());
        for (std::size_t i = 0; i < order.size(); ++i) {
            order[i] = i;
        }
        std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
            const MotifRecord &ma = motifs_[a];
            const MotifRecord &mb = motifs_[b];
            if (ma.accept_count != mb.accept_count) {
                return ma.accept_count > mb.accept_count;
            }
            return ma.gci > mb.gci;
        });
        std::vector<MotifRecord> kept;
        kept.reserve(static_cast<std::size_t>(max_keep));
        for (int32_t i = 0; i < max_keep; ++i) {
            kept.push_back(motifs_[order[static_cast<std::size_t>(i)]]);
        }
        motifs_.swap(kept);
    }

    /** Alive indices (ttl>0 or ttl==0 legacy), sorted accept_count→gci, capped. */
    std::vector<int32_t> list_for_inject(int32_t max_keep) const {
        std::vector<int32_t> ids;
        ids.reserve(motifs_.size());
        for (std::size_t i = 0; i < motifs_.size(); ++i) {
            const MotifRecord &m = motifs_[i];
            // ttl==0: legacy / non-archive slot still injectable; ttl>0: must be alive
            if (m.ttl_remaining < 0) {
                continue;
            }
            ids.push_back(static_cast<int32_t>(i));
        }
        std::sort(ids.begin(), ids.end(), [&](int32_t a, int32_t b) {
            const MotifRecord &ma = motifs_[static_cast<std::size_t>(a)];
            const MotifRecord &mb = motifs_[static_cast<std::size_t>(b)];
            if (ma.accept_count != mb.accept_count) {
                return ma.accept_count > mb.accept_count;
            }
            return ma.gci > mb.gci;
        });
        if (max_keep > 0 && static_cast<int32_t>(ids.size()) > max_keep) {
            ids.resize(static_cast<std::size_t>(max_keep));
        }
        return ids;
    }

    const MotifRecord *find_exact(int32_t gid_a, int32_t gid_b, Se2 relative, float area_a = 1.f, float area_b = 1.f) const {
        MotifRecord probe;
        probe.gid_a = gid_a;
        probe.gid_b = gid_b;
        probe.relative = relative;
        probe.area_a = area_a;
        probe.area_b = area_b;
        const auto key = motif_key(probe);
        for (const auto &m : motifs_) {
            if (motif_key(m) == key) {
                return &m;
            }
        }
        return nullptr;
    }

    /** Flat nearest by SE2 key L2 on quantized relative (same gid pair). */
    const MotifRecord *find_nearest(
        int32_t gid_a,
        int32_t gid_b,
        Se2 relative,
        float max_key_dist = 5.f,
        float area_a = 1.f,
        float area_b = 1.f
    ) const {
        MotifRecord probe;
        probe.gid_a = gid_a;
        probe.gid_b = gid_b;
        probe.relative = relative;
        probe.area_a = area_a;
        probe.area_b = area_b;
        motif_order_invariant(probe.gid_a, probe.gid_b, probe.relative, probe.area_a, probe.area_b);
        const auto q = se2_key3(probe.relative);
        const MotifRecord *best = nullptr;
        float best_d = max_key_dist;
        for (const auto &m : motifs_) {
            if (m.gid_a != probe.gid_a || m.gid_b != probe.gid_b) {
                continue;
            }
            const auto mq = se2_key3(m.relative);
            const float dx = static_cast<float>(std::get<0>(mq) - std::get<0>(q));
            const float dy = static_cast<float>(std::get<1>(mq) - std::get<1>(q));
            const float da = static_cast<float>(std::get<2>(mq) - std::get<2>(q));
            const float d = std::sqrt(dx * dx + dy * dy + da * da);
            if (d < best_d) {
                best_d = d;
                best = &m;
            }
        }
        return best;
    }

    /** Index of find_nearest match, or -1 (Mg hygiene). */
    int32_t find_nearest_id(
        int32_t gid_a,
        int32_t gid_b,
        Se2 relative,
        float max_key_dist = 5.f,
        float area_a = 1.f,
        float area_b = 1.f
    ) const {
        const MotifRecord *p = find_nearest(gid_a, gid_b, relative, max_key_dist, area_a, area_b);
        if (p == nullptr) {
            return -1;
        }
        for (std::size_t i = 0; i < motifs_.size(); ++i) {
            if (&motifs_[i] == p) {
                return static_cast<int32_t>(i);
            }
        }
        return -1;
    }

    int32_t size() const { return static_cast<int32_t>(motifs_.size()); }
    const MotifRecord &at(int32_t id) const { return motifs_.at(static_cast<std::size_t>(id)); }
    MotifRecord &at(int32_t id) { return motifs_.at(static_cast<std::size_t>(id)); }

    float moving_median_compactness() const {
        if (motifs_.empty()) {
            return 0.f;
        }
        std::vector<float> vals;
        vals.reserve(motifs_.size());
        for (const auto &m : motifs_) {
            vals.push_back(m.compactness);
        }
        std::sort(vals.begin(), vals.end());
        const std::size_t mid = vals.size() / 2;
        if (vals.size() % 2 == 0) {
            return 0.5f * (vals[mid - 1] + vals[mid]);
        }
        return vals[mid];
    }

private:
    std::vector<MotifRecord> motifs_;
};
