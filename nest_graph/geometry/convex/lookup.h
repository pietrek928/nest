#pragma once


// -------------------------------------------------------------------------
// EXTREME POINT SEARCH (CLOSED POLYGONS) - CACHE AWARE EARLY-EXIT
// -------------------------------------------------------------------------
template <class VecType>
inline int get_extreme_index_polygon(const VecType* poly, int n, const VecType& dir, int cache_idx) {
    int curr = (cache_idx >= 0 && cache_idx < n) ? cache_idx : 0;
    auto curr_dot = poly[curr].dp(dir);

    int next = (curr + 1) % n;
    auto next_dot = poly[next].dp(dir);
    if (next_dot > curr_dot) {
        curr_dot = next_dot;
        curr = next;
        while (true) {
            next = (curr + 1) % n;
            next_dot = poly[next].dp(dir);
            if (next_dot > curr_dot) {
                curr_dot = next_dot;
                curr = next;
            } else {
                break;
            }
        }
        return curr;
    }

    int prev = (curr - 1 + n) % n;
    auto prev_dot = poly[prev].dp(dir);
    if (prev_dot > curr_dot) {
        curr_dot = prev_dot;
        curr = prev;
        while (true) {
            prev = (curr - 1 + n) % n;
            prev_dot = poly[prev].dp(dir);
            if (prev_dot > curr_dot) {
                curr_dot = prev_dot;
                curr = prev;
            } else {
                break;
            }
        }
        return curr;
    }

    return curr;
}

template <class VecType>
inline int get_extreme_index_polygon_gradient(const VecType* poly, int n, const VecType& dir, int cache_idx) {
    return get_extreme_index_polygon(poly, n, dir, cache_idx);
}

// -------------------------------------------------------------------------
// EXTREME POINT SEARCH (LINE STRING BOUNDARIES) - CACHE AWARE EARLY-EXIT
// -------------------------------------------------------------------------
template <class VecType>
inline int get_extreme_index_linestring(const VecType* ls, int n, const VecType& dir, int cache_idx) {
    if (n > 2 && ls[0] == ls[n - 1]) {
        return get_extreme_index_polygon(ls, n - 1, dir, cache_idx);
    }

    int curr = (cache_idx >= 0 && cache_idx < n) ? cache_idx : 0;
    auto curr_dot = ls[curr].dp(dir);

    if (curr < n - 1) {
        auto next_dot = ls[curr + 1].dp(dir);
        if (next_dot > curr_dot) {
            curr_dot = next_dot;
            curr++;
            for (int i = curr + 1; i < n; ++i) {
                auto d = ls[i].dp(dir);
                if (d > curr_dot) {
                    curr_dot = d;
                    curr = i;
                } else {
                    break;
                }
            }
            return curr;
        }
    }

    if (curr > 0) {
        auto prev_dot = ls[curr - 1].dp(dir);
        if (prev_dot > curr_dot) {
            curr_dot = prev_dot;
            curr--;
            for (int i = curr - 1; i >= 0; --i) {
                auto d = ls[i].dp(dir);
                if (d > curr_dot) {
                    curr_dot = d;
                    curr = i;
                } else {
                    break;
                }
            }
            return curr;
        }
    }

    return curr;
}

template <class VecType>
inline int get_extreme_index_linestring_gradient(const VecType* ls, int n, const VecType& dir, int cache_idx) {
    return get_extreme_index_linestring(ls, n, dir, cache_idx);
}
