#pragma once

#include <algorithm>
#include <utility>
#include <vector>
#include <iostream>
#include <type_traits> // For std::is_same_v


// -------------------------------------------------------------------------
// 1. PRODUCTION TRACER (Zero Overhead Dummy)
// -------------------------------------------------------------------------
struct NullTracer {
    // Empty definitions just to satisfy the compiler if strictly checked
    void push_pair(int, int) {}
    void pop_pair() {}
    void record_penetration() {}
    void record_distance() {}
    void count_sweep_pair() {}
    void count_circle_prune() {}
    void count_gjk_eval() {}
    void count_hole_invalidation() {}
};

// -------------------------------------------------------------------------
// 2. DEBUG TRACER (Stateful Instance)
// -------------------------------------------------------------------------
struct DebugTracer {
    // Isolated instance state! No more thread_local globals.
    std::vector<std::pair<int, int>> pair_stack;
    std::vector<std::pair<int, int>> penetration_pairs;
    std::vector<std::pair<int, int>> distance_pairs;

    int stat_sweep_pairs = 0;
    int stat_circle_pruned = 0;
    int stat_gjk_evals = 0;
    int stat_hole_invalidations = 0;

    void push_pair(int a, int b) {
        pair_stack.push_back({std::min(a, b), std::max(a, b)});
    }

    void pop_pair() {
        if (!pair_stack.empty()) pair_stack.pop_back();
    }

    void record_penetration() {
        if (!pair_stack.empty()) penetration_pairs.push_back(pair_stack.back());
    }

    void record_distance() {
        if (!pair_stack.empty()) distance_pairs.push_back(pair_stack.back());
    }

    void count_sweep_pair() { stat_sweep_pairs++; }
    void count_circle_prune() { stat_circle_pruned++; }
    void count_gjk_eval() { stat_gjk_evals++; }
    void count_hole_invalidation() { stat_hole_invalidations++; }

    void reset() {
        pair_stack.clear();
        penetration_pairs.clear();
        distance_pairs.clear();
        stat_sweep_pairs = 0;
        stat_circle_pruned = 0;
        stat_gjk_evals = 0;
        stat_hole_invalidations = 0;
    }

    // --- Testing Queries ---
    void print_telemetry() const {
        std::cout << "[Physics Telemetry] "
                  << "Sweep Pairs: " << stat_sweep_pairs
                  << " | Circle Pruned: " << stat_circle_pruned
                  << " | GJK Evals: " << stat_gjk_evals
                  << " | Hole Invalidations: " << stat_hole_invalidations
                  << "\n";
    }

    bool saw_penetration_pair(int a, int b) const {
        const int lo = std::min(a, b);
        const int hi = std::max(a, b);
        for (const auto& p : penetration_pairs) {
            if (p.first == lo && p.second == hi) return true;
        }
        return false;
    }

    bool saw_distance_pair(int a, int b) const {
        const int lo = std::min(a, b);
        const int hi = std::max(a, b);
        for (const auto& p : distance_pairs) {
            if (p.first == lo && p.second == hi) return true;
        }
        return false;
    }

    bool saw_any_narrow_pair(int a, int b) const {
        return saw_penetration_pair(a, b) || saw_distance_pair(a, b);
    }
};

// -------------------------------------------------------------------------
// 3. ZERO-COST RAII SCOPE HELPER
// -------------------------------------------------------------------------
template <typename Tracer>
struct TracerScope {
    Tracer* tracer;

    TracerScope(Tracer* t, int a, int b) : tracer(t) {
        // If Tracer is NullTracer, the compiler deletes this entire block!
        // No runtime `if (tracer)` branches in production.
        if constexpr (!std::is_same_v<Tracer, NullTracer>) {
            if (tracer) tracer->push_pair(a, b);
        }
    }

    ~TracerScope() {
        if constexpr (!std::is_same_v<Tracer, NullTracer>) {
            if (tracer) tracer->pop_pair();
        }
    }
};
