#pragma once


// -------------------------------------------------------------------------
// EXTREME POINT SEARCH (LINE STRING BOUNDARIES) - CACHE AWARE EARLY-EXIT
// -------------------------------------------------------------------------
template <class VecType>
inline int get_extreme_index_linestring(const VecType* ls, int n, const VecType& dir, int cache_idx) {
    // 1. Start safely at the cached index
    int curr = (cache_idx >= 0 && cache_idx < n) ? cache_idx : 0;
    auto curr_dot = ls[curr].dp(dir);

    // 2. Try walking FORWARD
    if (curr < n - 1) {
        auto next_dot = ls[curr + 1].dp(dir);
        if (next_dot > curr_dot) {
            curr_dot = next_dot;
            curr++;
            // Ride the forward gradient using a highly predictable for-loop
            for (int i = curr + 1; i < n; ++i) {
                auto d = ls[i].dp(dir);
                if (d > curr_dot) {
                    curr_dot = d;
                    curr = i;
                } else {
                    break; // Peak found! Stop browsing the line.
                }
            }
            return curr;
        }
    }

    // 3. Try walking BACKWARD (only evaluated if forward failed)
    if (curr > 0) {
        auto prev_dot = ls[curr - 1].dp(dir);
        if (prev_dot > curr_dot) {
            curr_dot = prev_dot;
            curr--;
            // Ride the backward gradient
            for (int i = curr - 1; i >= 0; --i) {
                auto d = ls[i].dp(dir);
                if (d > curr_dot) {
                    curr_dot = d;
                    curr = i;
                } else {
                    break; // Peak found! Stop browsing the line.
                }
            }
            return curr;
        }
    }

    // 4. We were already sitting exactly on the peak!
    return curr;
}

template <class VecType>
inline int get_extreme_index_linestring_gradient(const VecType* ls, int n, const VecType& dir, int cache_idx) {
    int curr = (cache_idx >= 0 && cache_idx < n) ? cache_idx : 0;
    auto curr_dot = ls[curr].dp(dir);

    // 1. Try moving FORWARD
    if (curr < n - 1) {
        auto next_dot = ls[curr + 1].dp(dir);
        if (next_dot > curr_dot) {
            curr++;
            curr_dot = next_dot;

            // We are on the forward gradient. Ride it to the peak!
            while (curr < n - 1) {
                next_dot = ls[curr + 1].dp(dir);
                if (next_dot > curr_dot) {
                    curr++;
                    curr_dot = next_dot;
                } else {
                    break; // Reached the peak
                }
            }
            return curr;
        }
    }

    // 2. Try moving BACKWARD (Only executes if forward failed)
    if (curr > 0) {
        auto prev_dot = ls[curr - 1].dp(dir);
        if (prev_dot > curr_dot) {
            curr--;
            curr_dot = prev_dot;

            // We are on the backward gradient. Ride it to the peak!
            while (curr > 0) {
                prev_dot = ls[curr - 1].dp(dir);
                if (prev_dot > curr_dot) {
                    curr--;
                    curr_dot = prev_dot;
                } else {
                    break; // Reached the peak
                }
            }
            return curr;
        }
    }

    // 3. We are already at the peak!
    return curr;
}
