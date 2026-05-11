#pragma once

#include <algorithm>


// -------------------------------------------------------------------------
// STANDARD SUPPORT FUNCTION (Hill-Climbing / Linear Search)
// -------------------------------------------------------------------------
template <class T, class VecType>
inline int get_extreme_index(
    const VecType* poly, int n,
    const VecType& dir, int start_idx
) {
    int curr = start_idx;
    T max_dot = poly[curr].dp(dir);

    auto next = [](int i, int size) { return (i + 1 < size) ? i + 1 : 0; };
    auto prev = [](int i, int size) { return (i > 0) ? i - 1 : size - 1; };

    // Try moving forward
    int nxt = next(curr, n);
    T nxt_dot = poly[nxt].dp(dir);

    if (nxt_dot > max_dot) {
        while (true) {
            curr = nxt;
            max_dot = nxt_dot;
            nxt = next(curr, n);
            nxt_dot = poly[nxt].dp(dir);
            if (nxt_dot <= max_dot) break;
        }
        return curr;
    }

    // Try moving backward
    int prv = prev(curr, n);
    T prv_dot = poly[prv].dp(dir);

    if (prv_dot > max_dot) {
        while (true) {
            curr = prv;
            max_dot = prv_dot;
            prv = prev(curr, n);
            prv_dot = poly[prv].dp(dir);
            if (prv_dot <= max_dot) break;
        }
    }

    return curr;
}

// -------------------------------------------------------------------------
// GRADIENT-BOOSTED SUPPORT FUNCTION (Galloping / Exponential Search)
// -------------------------------------------------------------------------
template <class T, class VecType>
inline int get_extreme_index_gradient(
    const VecType* poly, int n,
    const VecType& dir, int start_idx
) {
    // Lambda for fast, safe array wrapping (handles negative indices)
    auto wrap = [&](int i) { return (i % n + n) % n; };

    // Clean N-dimensional dot product evaluation
    auto get_dot = [&](int i) { return poly[i].dp(dir); };

    int curr = start_idx;
    T d_curr = get_dot(curr);
    T d_next = get_dot(wrap(curr + 1));
    T d_prev = get_dot(wrap(curr - 1));

    // 1. Is the cached vertex already the maximum?
    if (d_next <= d_curr && d_prev <= d_curr) {
        return curr;
    }

    // 2. Determine Gradient Direction (+1 for CCW, -1 for CW)
    int step_dir = (d_next > d_curr) ? 1 : -1;

    int step = 1;
    int prev_idx = curr;
    T d_prev_val = d_curr;

    // 3. Galloping Phase (Jump 1, 2, 4, 8... to find the peak's bounds)
    while (step < n) {
        int next_idx = wrap(start_idx + step_dir * step);
        T d_next_val = get_dot(next_idx);

        // If the gradient drops, we have successfully jumped over the peak
        if (d_next_val <= d_prev_val) {
            break;
        }

        prev_idx = next_idx;
        d_prev_val = d_next_val;
        step *= 2;
    }

    // 4. Binary Search Phase (Pinpoint the peak within the discovered bounds)
    int L = step / 2;
    int R = std::min(step, n);

    int best_idx = prev_idx;
    T max_val = d_prev_val;

    while (L <= R) {
        int mid = L + (R - L) / 2;
        int mid_idx = wrap(start_idx + step_dir * mid);
        T val = get_dot(mid_idx);

        // Calculate gradient at the midpoint to know which half to discard
        int mid_next_idx = wrap(start_idx + step_dir * (mid + 1));
        T val_next = get_dot(mid_next_idx);

        if (val > max_val) {
            max_val = val;
            best_idx = mid_idx;
        }

        if (val_next > val) {
            // Gradient is still pointing forward, peak is further ahead
            L = mid + 1;
        } else {
            // Gradient points backward, peak is behind
            R = mid - 1;
        }
    }

    return best_idx;
}
