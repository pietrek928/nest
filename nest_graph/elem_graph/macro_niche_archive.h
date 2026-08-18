#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

constexpr int32_t NICHE_RING_H = 6;

struct NicheRow {
    int32_t gid = 0;
    float x = 0.f;
    float y = 0.f;
    float theta = 0.f;
};

struct NicheEpisode {
    int32_t polarity = 0;
    int32_t void_nest = 0;
    float score = 0.f;
    std::vector<NicheRow> rows;
};

struct MacroNicheBucket {
    int32_t hits = 0;
    int32_t misses = 0;
    int32_t best_void_nest = 0;
    int32_t ttl_remaining = 4;
    std::vector<NicheEpisode> ring;
    std::map<int32_t, std::vector<std::array<float, 3>>> active_rows;

    float miss_rate() const {
        const int32_t tot = hits + misses;
        if (tot <= 0) {
            return 0.f;
        }
        return static_cast<float>(misses) / static_cast<float>(tot);
    }
};

using NicheKey = std::tuple<int32_t, int32_t, int32_t>;

class MacroNicheArchive {
public:
    int32_t place_fail_streak = 0;
    int32_t max_rows_per_episode = 64;
    std::set<std::tuple<double, double, double>> last_feed_keys;

    MacroNicheBucket &get(NicheKey key) {
        return buckets_[key];
    }

    const std::map<NicheKey, MacroNicheBucket> &buckets() const {
        return buckets_;
    }
    std::map<NicheKey, MacroNicheBucket> &buckets() {
        return buckets_;
    }

    bool empty() const { return buckets_.empty(); }
    int32_t size() const { return static_cast<int32_t>(buckets_.size()); }

    int32_t total_hits() const {
        int32_t n = 0;
        for (const auto &kv : buckets_) {
            n += kv.second.hits;
        }
        return n;
    }

    void append_positive(
        NicheKey key,
        const std::vector<NicheRow> &rows,
        int32_t void_nest,
        float score,
        int32_t ttl
    ) {
        MacroNicheBucket &bucket = get(key);
        const int32_t cap = max_rows_per_episode > 0 ? max_rows_per_episode : 0;
        NicheEpisode ep;
        ep.polarity = 1;
        ep.void_nest = void_nest;
        ep.score = score;
        if (cap > 0 && static_cast<int32_t>(rows.size()) > cap) {
            ep.rows.assign(rows.begin(), rows.begin() + cap);
        } else {
            ep.rows = rows;
        }
        if (static_cast<int32_t>(bucket.ring.size()) >= NICHE_RING_H) {
            bucket.ring.erase(bucket.ring.begin());
        }
        bucket.ring.push_back(ep);
        bucket.hits += 1;
        if (void_nest > bucket.best_void_nest) {
            bucket.best_void_nest = void_nest;
        }
        bucket.ttl_remaining = ttl > 1 ? ttl : 1;
        for (const NicheRow &row : ep.rows) {
            auto &lst = bucket.active_rows[row.gid];
            lst.push_back({row.x, row.y, row.theta});
            if (cap > 0 && static_cast<int32_t>(lst.size()) > cap) {
                lst.erase(lst.begin(), lst.begin() + (static_cast<int32_t>(lst.size()) - cap));
            }
        }
    }

    void append_negative(NicheKey key, int32_t void_nest, float score) {
        MacroNicheBucket &bucket = get(key);
        NicheEpisode ep;
        ep.polarity = -1;
        ep.void_nest = void_nest;
        ep.score = score;
        if (static_cast<int32_t>(bucket.ring.size()) >= NICHE_RING_H) {
            bucket.ring.erase(bucket.ring.begin());
        }
        bucket.ring.push_back(std::move(ep));
        bucket.misses += 1;
    }

    bool any_void_miss_rate_high(float threshold = 0.8f) const {
        for (const auto &kv : buckets_) {
            const MacroNicheBucket &bucket = kv.second;
            if (bucket.hits + bucket.misses <= 0) {
                continue;
            }
            if (bucket.miss_rate() > threshold) {
                return true;
            }
        }
        return false;
    }

    std::map<int32_t, std::vector<std::array<float, 3>>> active_by_group(int32_t ngroups) const {
        std::map<int32_t, std::vector<std::array<float, 3>>> out;
        for (const auto &kv : buckets_) {
            const MacroNicheBucket &bucket = kv.second;
            if (bucket.hits <= 0 || bucket.ttl_remaining <= 0) {
                continue;
            }
            for (const auto &gid_rows : bucket.active_rows) {
                const int32_t gid = gid_rows.first;
                if (gid < 0 || gid >= ngroups) {
                    continue;
                }
                auto &dst = out[gid];
                dst.insert(dst.end(), gid_rows.second.begin(), gid_rows.second.end());
            }
        }
        return out;
    }

    int32_t age(int32_t step = 1) {
        int32_t dropped = 0;
        std::vector<NicheKey> dead;
        for (auto &kv : buckets_) {
            kv.second.ttl_remaining -= step;
            if (kv.second.ttl_remaining > 0) {
                continue;
            }
            if (kv.second.hits > 0) {
                kv.second.ttl_remaining = 1;
                continue;
            }
            dead.push_back(kv.first);
            dropped += 1;
        }
        for (const NicheKey &key : dead) {
            buckets_.erase(key);
        }
        return dropped;
    }

    void note_place_outcome(bool placed) {
        if (placed) {
            place_fail_streak = 0;
        } else {
            place_fail_streak += 1;
        }
    }

private:
    std::map<NicheKey, MacroNicheBucket> buckets_;
};
