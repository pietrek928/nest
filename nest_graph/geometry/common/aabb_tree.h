#pragma once

// Flat 2D AABB BVH (prefilter only — never replaces Scene / sweep / contains_point).
// Build: longest-extent-axis + centroid nth_element split; leaf size AABB_TREE_LEAF_SIZE.
// Boxes reuse nest_graph/common/bbox.h (BBox<2,double>).
// Query kernels take raw stack/out buffers (ptr+cap) — vector-independent.

#include <bbox.h>
#include <vec.h>

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

inline constexpr int AABB_TREE_LEAF_SIZE = 12;

using Aabb2 = BBox<2, double>;

inline Aabb2 aabb2_from_ltrb(double minx, double miny, double maxx, double maxy) {
    return Aabb2(Vec<2, double>{minx, miny}, Vec<2, double>{maxx, maxy});
}

inline Aabb2 aabb2_from_point(double x, double y) {
    return aabb2_from_ltrb(x, y, x, y);
}

// Inclusive overlap (kissing edges count). Uses BBox start/end; avoids
// object_bbox_intersects which needs Vec::get_pos not present on Vec<2,double>.
inline bool aabb2_intersects(const Aabb2& a, const Aabb2& b) {
    if (a.end[0] < b.start[0] || b.end[0] < a.start[0]) {
        return false;
    }
    if (a.end[1] < b.start[1] || b.end[1] < a.start[1]) {
        return false;
    }
    return true;
}

inline Aabb2 aabb2_merge(const Aabb2& a, const Aabb2& b) {
    return aabb2_from_ltrb(
        std::min(a.start[0], b.start[0]),
        std::min(a.start[1], b.start[1]),
        std::max(a.end[0], b.end[0]),
        std::max(a.end[1], b.end[1]));
}

// Split mid without summing start+end (avoids overflow near DBL_MAX/2).
inline double aabb2_centroid_axis(const Aabb2& b, int axis) {
    return 0.5 * b.start[axis] + 0.5 * b.end[axis];
}

// Squared distance from point (qx,qy) to closest point on box (0 if inside/on).
inline double aabb2_point_dist_sq(const Aabb2& b, double qx, double qy) {
    const double dx = std::max({b.start[0] - qx, 0.0, qx - b.end[0]});
    const double dy = std::max({b.start[1] - qy, 0.0, qy - b.end[1]});
    return dx * dx + dy * dy;
}

// Finite + oriented LTRB (shared by rebuild ingest and query_box).
inline bool aabb2_ltrb_valid(double minx, double miny, double maxx, double maxy) {
    return std::isfinite(minx) && std::isfinite(miny) && std::isfinite(maxx)
        && std::isfinite(maxy) && minx <= maxx && miny <= maxy;
}

// Flat BVH node. left < 0 => leaf: first = -left-1 into leaf_items, count = right.
// Internal: left/right are child node indices.
struct AabbTreeNode {
    Aabb2 box;
    int left = -1;
    int right = 0;
};

// Caller-owned flat AABB BVH (build state only). Rebuild clears/reuses members
// only (may grow vectors via reserve/push; no other heap allocations).
//
// Buffer contract:
//   boxes:       n leaf AABBs indexed by original item id 0..n-1
//   indices:     build permutation over item ids
//   nodes:       flat node array; nodes[0] is root when !empty
//   leaf_items:  packed leaf payloads (original item ids)
struct AabbTree {
    std::vector<Aabb2> boxes;
    std::vector<int> indices;
    std::vector<AabbTreeNode> nodes;
    std::vector<int> leaf_items;
};

// Optional growable DFS stack for queries (one buffer). Prefer the ptr+cap
// query overloads; this helper only owns storage for callers that want reuse.
struct AabbTreeScratch {
    std::vector<int> stack;
};

// Resize scratch.stack to exactly need elements; returns data() (null if need==0).
inline int* aabb_tree_scratch_prepare(AabbTreeScratch& scratch, std::size_t need) {
    scratch.stack.resize(need);
    return scratch.stack.empty() ? nullptr : scratch.stack.data();
}

inline void aabb_tree_clear_build(AabbTree& tree) {
    tree.boxes.clear();
    tree.indices.clear();
    tree.nodes.clear();
    tree.leaf_items.clear();
}

inline Aabb2 aabb_tree_union_range(
    const AabbTree& tree,
    const int* ids,
    int n
) {
    Aabb2 u = tree.boxes[static_cast<std::size_t>(ids[0])];
    for (int i = 1; i < n; ++i) {
        u = aabb2_merge(u, tree.boxes[static_cast<std::size_t>(ids[i])]);
    }
    return u;
}

inline int aabb_tree_build_range(AabbTree& tree, int begin, int end) {
    const int n = end - begin;
    const int node_i = static_cast<int>(tree.nodes.size());
    tree.nodes.push_back(AabbTreeNode{});
    int* ids = tree.indices.data() + begin;
    Aabb2 ub = aabb_tree_union_range(tree, ids, n);
    if (n <= AABB_TREE_LEAF_SIZE) {
        const int first = static_cast<int>(tree.leaf_items.size());
        for (int i = 0; i < n; ++i) {
            tree.leaf_items.push_back(ids[i]);
        }
        tree.nodes[static_cast<std::size_t>(node_i)].box = ub;
        tree.nodes[static_cast<std::size_t>(node_i)].left = -first - 1;
        tree.nodes[static_cast<std::size_t>(node_i)].right = n;
        return node_i;
    }
    const double ext_x = ub.end[0] - ub.start[0];
    const double ext_y = ub.end[1] - ub.start[1];
    const int axis = (ext_x >= ext_y) ? 0 : 1;
    const int mid = begin + n / 2;
    std::nth_element(
        tree.indices.begin() + begin,
        tree.indices.begin() + mid,
        tree.indices.begin() + end,
        [&](int ia, int ib) {
            return aabb2_centroid_axis(tree.boxes[static_cast<std::size_t>(ia)], axis)
                < aabb2_centroid_axis(tree.boxes[static_cast<std::size_t>(ib)], axis);
        });
    const int left = aabb_tree_build_range(tree, begin, mid);
    const int right = aabb_tree_build_range(tree, mid, end);
    tree.nodes[static_cast<std::size_t>(node_i)].box = ub;
    tree.nodes[static_cast<std::size_t>(node_i)].left = left;
    tree.nodes[static_cast<std::size_t>(node_i)].right = right;
    return node_i;
}

// Shared finish after boxes/indices filled: reserve + build. n must be in 1..INT_MAX.
inline void aabb_tree_finish_build(AabbTree& tree, std::size_t n) {
    tree.nodes.reserve(n * 2);
    tree.leaf_items.reserve(n);
    aabb_tree_build_range(tree, 0, static_cast<int>(n));
}

inline bool aabb_knn_pair_less(
    const std::pair<double, int>& a,
    const std::pair<double, int>& b
) {
    if (a.first != b.first) {
        return a.first < b.first;
    }
    return a.second < b.second;
}

// Parallel-array max-heap helpers for knn (out_dist_sq[i], out_idx[i]).
inline bool aabb_knn_at_less(
    const double* dist,
    const int* idx,
    std::size_t a,
    std::size_t b
) {
    if (dist[a] != dist[b]) {
        return dist[a] < dist[b];
    }
    return idx[a] < idx[b];
}

inline void aabb_knn_sift_up(double* dist, int* idx, std::size_t i) {
    while (i > 0) {
        const std::size_t parent = (i - 1) / 2;
        // max-heap: parent should be >= child under aabb_knn_pair_less order
        // (largest at front). push_heap uses less → parent !less child.
        if (!aabb_knn_at_less(dist, idx, parent, i)) {
            break;
        }
        std::swap(dist[parent], dist[i]);
        std::swap(idx[parent], idx[i]);
        i = parent;
    }
}

inline void aabb_knn_sift_down(
    double* dist,
    int* idx,
    std::size_t n,
    std::size_t i
) {
    for (;;) {
        std::size_t largest = i;
        const std::size_t left = 2 * i + 1;
        const std::size_t right = 2 * i + 2;
        if (left < n && aabb_knn_at_less(dist, idx, largest, left)) {
            largest = left;
        }
        if (right < n && aabb_knn_at_less(dist, idx, largest, right)) {
            largest = right;
        }
        if (largest == i) {
            break;
        }
        std::swap(dist[i], dist[largest]);
        std::swap(idx[i], idx[largest]);
        i = largest;
    }
}

// rebuild_boxes(const double* ltrb, n_boxes, AabbTree& tree):
//   ltrb = 4*n doubles (minx,miny,maxx,maxy); null only if n==0.
//   Returns false (tree left empty) if: n>0 && ltrb==nullptr, n>INT_MAX, or any
//   box fails aabb2_ltrb_valid. Caller must fall back to brute / skip prefilter;
//   empty tree must not be treated as "no hits".
[[nodiscard]] inline bool aabb_tree_rebuild_boxes(
    const double* ltrb,
    std::size_t n_boxes,
    AabbTree& tree
) {
    aabb_tree_clear_build(tree);
    if (n_boxes == 0) {
        return true;
    }
    if (ltrb == nullptr || n_boxes > static_cast<std::size_t>(INT_MAX)) {
        return false;
    }
    tree.boxes.reserve(n_boxes);
    tree.indices.reserve(n_boxes);
    for (std::size_t i = 0; i < n_boxes; ++i) {
        const double* p = ltrb + 4 * i;
        if (!aabb2_ltrb_valid(p[0], p[1], p[2], p[3])) {
            aabb_tree_clear_build(tree);
            return false;
        }
        tree.boxes.push_back(aabb2_from_ltrb(p[0], p[1], p[2], p[3]));
        tree.indices.push_back(static_cast<int>(i));
    }
    aabb_tree_finish_build(tree, n_boxes);
    return true;
}

// rebuild_points(const double* xy, n_pts, tree): xy = 2*n; points as degenerate boxes.
//   Returns false if n>0 && xy==nullptr, n>INT_MAX, or any point non-finite.
[[nodiscard]] inline bool aabb_tree_rebuild_points(
    const double* xy,
    std::size_t n_pts,
    AabbTree& tree
) {
    aabb_tree_clear_build(tree);
    if (n_pts == 0) {
        return true;
    }
    if (xy == nullptr || n_pts > static_cast<std::size_t>(INT_MAX)) {
        return false;
    }
    tree.boxes.reserve(n_pts);
    tree.indices.reserve(n_pts);
    for (std::size_t i = 0; i < n_pts; ++i) {
        const double* p = xy + 2 * i;
        if (!aabb2_ltrb_valid(p[0], p[1], p[0], p[1])) {
            aabb_tree_clear_build(tree);
            return false;
        }
        tree.boxes.push_back(aabb2_from_point(p[0], p[1]));
        tree.indices.push_back(static_cast<int>(i));
    }
    aabb_tree_finish_build(tree, n_pts);
    return true;
}

// query_box / query_radius / knn (ptr+cap; vector-independent):
//   stack: DFS node indices; REQUIRE stack_cap >= tree.nodes.size().
//          Undersized / null stack → return 0, write nothing (defined, not UB).
//   out_idx capacity out_cap; return written count; cap too small → return 0,
//          write nothing.
//   Return 0 also means: no hits, invalid query, or empty tree — wires must size
//   out_cap >= n_items for box/radius (guarantees no overflow) and treat rebuild
//   false / empty tree as "skip prefilter", never as "clear".
//   box/radius: out ids ascending by id. knn: ascending by (dist_sq, id).
//   distances are squared for radius/knn; inclusive <=.
//   knn: heap lives in out_idx + dist_buf; out_cap must be >= min(k, n).
//        Prefer out_dist_sq for keys; if null, pass dist_scratch[out_cap] instead
//        (dist_scratch required non-null when out_dist_sq is null and k_eff>0).
//   Short out_cap: count pass first (out untouched on overflow); write only when
//   count fits. out_cap >= n_boxes skips the count pass (single write cannot overflow).

// Count box overlaps (no out write). stack_cap already validated by caller.
inline std::size_t aabb_tree_count_box(
    const AabbTree& tree,
    int* stack,
    double minx,
    double miny,
    double maxx,
    double maxy
) {
    const Aabb2 q = aabb2_from_ltrb(minx, miny, maxx, maxy);
    std::size_t stack_n = 0;
    stack[stack_n++] = 0;
    std::size_t n_hit = 0;
    while (stack_n > 0) {
        const int ni = stack[--stack_n];
        const AabbTreeNode& node = tree.nodes[static_cast<std::size_t>(ni)];
        if (!aabb2_intersects(node.box, q)) {
            continue;
        }
        if (node.left < 0) {
            const int first = -node.left - 1;
            const int count = node.right;
            for (int i = 0; i < count; ++i) {
                const int id = tree.leaf_items[static_cast<std::size_t>(first + i)];
                if (aabb2_intersects(tree.boxes[static_cast<std::size_t>(id)], q)) {
                    ++n_hit;
                }
            }
        } else {
            stack[stack_n++] = node.left;
            stack[stack_n++] = node.right;
        }
    }
    return n_hit;
}

inline std::size_t aabb_tree_count_radius(
    const AabbTree& tree,
    int* stack,
    double qx,
    double qy,
    double radius_sq
) {
    std::size_t stack_n = 0;
    stack[stack_n++] = 0;
    std::size_t n_hit = 0;
    while (stack_n > 0) {
        const int ni = stack[--stack_n];
        const AabbTreeNode& node = tree.nodes[static_cast<std::size_t>(ni)];
        if (aabb2_point_dist_sq(node.box, qx, qy) > radius_sq) {
            continue;
        }
        if (node.left < 0) {
            const int first = -node.left - 1;
            const int count = node.right;
            for (int i = 0; i < count; ++i) {
                const int id = tree.leaf_items[static_cast<std::size_t>(first + i)];
                if (aabb2_point_dist_sq(tree.boxes[static_cast<std::size_t>(id)], qx, qy)
                    <= radius_sq) {
                    ++n_hit;
                }
            }
        } else {
            stack[stack_n++] = node.left;
            stack[stack_n++] = node.right;
        }
    }
    return n_hit;
}

[[nodiscard]] inline std::size_t aabb_tree_query_box(
    const AabbTree& tree,
    int* stack,
    std::size_t stack_cap,
    double minx,
    double miny,
    double maxx,
    double maxy,
    int* out_idx,
    std::size_t out_cap
) {
    if (tree.nodes.empty() || out_idx == nullptr || out_cap == 0) {
        return 0;
    }
    if (stack == nullptr || stack_cap < tree.nodes.size()) {
        return 0;
    }
    if (!aabb2_ltrb_valid(minx, miny, maxx, maxy)) {
        return 0;
    }
    // out_cap < n_boxes: count first so overflow leaves out_idx untouched.
    if (out_cap < tree.boxes.size()) {
        const std::size_t n_hit =
            aabb_tree_count_box(tree, stack, minx, miny, maxx, maxy);
        if (n_hit == 0 || n_hit > out_cap) {
            return 0;
        }
    }
    const Aabb2 q = aabb2_from_ltrb(minx, miny, maxx, maxy);
    std::size_t stack_n = 0;
    stack[stack_n++] = 0;
    std::size_t n_hit = 0;
    while (stack_n > 0) {
        const int ni = stack[--stack_n];
        const AabbTreeNode& node = tree.nodes[static_cast<std::size_t>(ni)];
        if (!aabb2_intersects(node.box, q)) {
            continue;
        }
        if (node.left < 0) {
            const int first = -node.left - 1;
            const int count = node.right;
            for (int i = 0; i < count; ++i) {
                const int id = tree.leaf_items[static_cast<std::size_t>(first + i)];
                if (aabb2_intersects(tree.boxes[static_cast<std::size_t>(id)], q)) {
                    out_idx[n_hit++] = id;
                }
            }
        } else {
            stack[stack_n++] = node.left;
            stack[stack_n++] = node.right;
        }
    }
    std::sort(out_idx, out_idx + n_hit);
    return n_hit;
}

[[nodiscard]] inline std::size_t aabb_tree_query_radius(
    const AabbTree& tree,
    int* stack,
    std::size_t stack_cap,
    double qx,
    double qy,
    double radius_sq,
    int* out_idx,
    std::size_t out_cap
) {
    if (tree.nodes.empty() || out_idx == nullptr || out_cap == 0) {
        return 0;
    }
    if (stack == nullptr || stack_cap < tree.nodes.size()) {
        return 0;
    }
    // Reject NaN / negative; +inf is valid (no prune; returns all if cap allows).
    if (!(radius_sq >= 0.0) || !std::isfinite(qx) || !std::isfinite(qy)) {
        return 0;
    }
    if (out_cap < tree.boxes.size()) {
        const std::size_t n_hit =
            aabb_tree_count_radius(tree, stack, qx, qy, radius_sq);
        if (n_hit == 0 || n_hit > out_cap) {
            return 0;
        }
    }
    std::size_t stack_n = 0;
    stack[stack_n++] = 0;
    std::size_t n_hit = 0;
    while (stack_n > 0) {
        const int ni = stack[--stack_n];
        const AabbTreeNode& node = tree.nodes[static_cast<std::size_t>(ni)];
        // Prune solely by point–AABB dist_sq (no sqrt box — ULP-safe for kiss).
        if (aabb2_point_dist_sq(node.box, qx, qy) > radius_sq) {
            continue;
        }
        if (node.left < 0) {
            const int first = -node.left - 1;
            const int count = node.right;
            for (int i = 0; i < count; ++i) {
                const int id = tree.leaf_items[static_cast<std::size_t>(first + i)];
                if (aabb2_point_dist_sq(tree.boxes[static_cast<std::size_t>(id)], qx, qy)
                    <= radius_sq) {
                    out_idx[n_hit++] = id;
                }
            }
        } else {
            stack[stack_n++] = node.left;
            stack[stack_n++] = node.right;
        }
    }
    std::sort(out_idx, out_idx + n_hit);
    return n_hit;
}

[[nodiscard]] inline std::size_t aabb_tree_knn(
    const AabbTree& tree,
    int* stack,
    std::size_t stack_cap,
    double qx,
    double qy,
    std::size_t k,
    int* out_idx,
    double* out_dist_sq,
    std::size_t out_cap,
    double* dist_scratch = nullptr
) {
    if (tree.nodes.empty() || out_idx == nullptr || k == 0 || out_cap == 0) {
        return 0;
    }
    if (stack == nullptr || stack_cap < tree.nodes.size()) {
        return 0;
    }
    if (!std::isfinite(qx) || !std::isfinite(qy)) {
        return 0;
    }
    const std::size_t n_boxes = tree.boxes.size();
    const std::size_t k_eff = std::min(k, n_boxes);
    if (out_cap < k_eff) {
        return 0;
    }
    double* dist = out_dist_sq != nullptr ? out_dist_sq : dist_scratch;
    if (dist == nullptr) {
        return 0;
    }
    std::size_t heap_n = 0;
    std::size_t stack_n = 0;
    stack[stack_n++] = 0;
    while (stack_n > 0) {
        const int ni = stack[--stack_n];
        const AabbTreeNode& node = tree.nodes[static_cast<std::size_t>(ni)];
        const double nd = aabb2_point_dist_sq(node.box, qx, qy);
        if (heap_n == k_eff && nd > dist[0]) {
            continue;
        }
        if (node.left < 0) {
            const int first = -node.left - 1;
            const int count = node.right;
            for (int i = 0; i < count; ++i) {
                const int id = tree.leaf_items[static_cast<std::size_t>(first + i)];
                const double d =
                    aabb2_point_dist_sq(tree.boxes[static_cast<std::size_t>(id)], qx, qy);
                if (heap_n < k_eff) {
                    dist[heap_n] = d;
                    out_idx[heap_n] = id;
                    aabb_knn_sift_up(dist, out_idx, heap_n);
                    ++heap_n;
                } else if (aabb_knn_pair_less({d, id}, {dist[0], out_idx[0]})) {
                    dist[0] = d;
                    out_idx[0] = id;
                    aabb_knn_sift_down(dist, out_idx, heap_n, 0);
                }
            }
        } else {
            const AabbTreeNode& L = tree.nodes[static_cast<std::size_t>(node.left)];
            const AabbTreeNode& R = tree.nodes[static_cast<std::size_t>(node.right)];
            const double dl = aabb2_point_dist_sq(L.box, qx, qy);
            const double dr = aabb2_point_dist_sq(R.box, qx, qy);
            // Push farther first so nearer is popped next.
            if (dl <= dr) {
                stack[stack_n++] = node.right;
                stack[stack_n++] = node.left;
            } else {
                stack[stack_n++] = node.left;
                stack[stack_n++] = node.right;
            }
        }
    }
    // In-place insertion sort ascending by (dist_sq, id) — no heap alloc.
    for (std::size_t i = 1; i < heap_n; ++i) {
        const double d = dist[i];
        const int id = out_idx[i];
        std::size_t j = i;
        while (j > 0 && aabb_knn_pair_less({d, id}, {dist[j - 1], out_idx[j - 1]})) {
            dist[j] = dist[j - 1];
            out_idx[j] = out_idx[j - 1];
            --j;
        }
        dist[j] = d;
        out_idx[j] = id;
    }
    return heap_n;
}

// Convenience: prepare scratch.stack to tree.nodes.size(), then call ptr+cap API.
[[nodiscard]] inline std::size_t aabb_tree_query_box(
    const AabbTree& tree,
    AabbTreeScratch& scratch,
    double minx,
    double miny,
    double maxx,
    double maxy,
    int* out_idx,
    std::size_t out_cap
) {
    const std::size_t need = tree.nodes.size();
    int* stack = aabb_tree_scratch_prepare(scratch, need);
    return aabb_tree_query_box(
        tree, stack, need, minx, miny, maxx, maxy, out_idx, out_cap);
}

[[nodiscard]] inline std::size_t aabb_tree_query_radius(
    const AabbTree& tree,
    AabbTreeScratch& scratch,
    double qx,
    double qy,
    double radius_sq,
    int* out_idx,
    std::size_t out_cap
) {
    const std::size_t need = tree.nodes.size();
    int* stack = aabb_tree_scratch_prepare(scratch, need);
    return aabb_tree_query_radius(
        tree, stack, need, qx, qy, radius_sq, out_idx, out_cap);
}

[[nodiscard]] inline std::size_t aabb_tree_knn(
    const AabbTree& tree,
    AabbTreeScratch& scratch,
    double qx,
    double qy,
    std::size_t k,
    int* out_idx,
    double* out_dist_sq,
    std::size_t out_cap,
    double* dist_scratch = nullptr
) {
    const std::size_t need = tree.nodes.size();
    int* stack = aabb_tree_scratch_prepare(scratch, need);
    return aabb_tree_knn(
        tree,
        stack,
        need,
        qx,
        qy,
        k,
        out_idx,
        out_dist_sq,
        out_cap,
        dist_scratch);
}
