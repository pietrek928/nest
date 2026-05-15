#pragma once

#include <vector>
#include <cmath>
#include <algorithm>


// -------------------------------------------------------------------------
// 2D MATH HELPERS
// -------------------------------------------------------------------------

template <class VecType>
inline typename VecType::Scalar cross_2d(const VecType& a, const VecType& b, const VecType& c) {
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
}

template <class VecType>
inline bool is_point_in_triangle(const VecType& p, const VecType& a, const VecType& b, const VecType& c) {
    using Scalar = typename VecType::Scalar;
    Scalar cp1 = cross_2d(a, b, p);
    Scalar cp2 = cross_2d(b, c, p);
    Scalar cp3 = cross_2d(c, a, p);
    // Point is strictly inside if all cross products share the same sign
    return (cp1 > 0 && cp2 > 0 && cp3 > 0) || (cp1 < 0 && cp2 < 0 && cp3 < 0);
}

// -------------------------------------------------------------------------
// PHASE 1: EAR CLIPPING TRIANGULATION
// -------------------------------------------------------------------------

template <class VecType>
std::vector<std::vector<VecType>> triangulate(std::vector<VecType> vertices) {
    using Scalar = typename VecType::Scalar;
    std::vector<std::vector<VecType>> triangles;
    if (vertices.size() < 3) return triangles;

    // Ensure CCW winding
    Scalar area = 0;
    for (size_t i = 0; i < vertices.size(); ++i) {
        size_t j = (i + 1) % vertices.size();
        area += (vertices[j][0] - vertices[i][0]) * (vertices[j][1] + vertices[i][1]);
    }
    if (area > 0) {
        std::reverse(vertices.begin(), vertices.end());
    }

    while (vertices.size() > 3) {
        bool ear_found = false;
        int n = static_cast<int>(vertices.size());

        for (int i = 0; i < n; ++i) {
            int prev = (i - 1 + n) % n;
            int next = (i + 1) % n;

            const VecType& v_prev = vertices[prev];
            const VecType& v_curr = vertices[i];
            const VecType& v_next = vertices[next];

            // 1. Check if angle is convex (cross product > 0 for CCW)
            if (cross_2d(v_prev, v_curr, v_next) <= 0) continue;

            // 2. Check if any other vertex is inside this triangle
            bool is_ear = true;
            for (int j = 0; j < n; ++j) {
                if (j == prev || j == i || j == next) continue;
                if (is_point_in_triangle(vertices[j], v_prev, v_curr, v_next)) {
                    is_ear = false;
                    break;
                }
            }

            if (is_ear) {
                triangles.push_back({v_prev, v_curr, v_next});
                vertices.erase(vertices.begin() + i);
                ear_found = true;
                break;
            }
        }

        // Fallback for self-intersecting or degenerate polygons
        if (!ear_found) {
            triangles.push_back({vertices[0], vertices[1], vertices[2]});
            vertices.erase(vertices.begin() + 1);
        }
    }

    // Push the final remaining triangle
    triangles.push_back({vertices[0], vertices[1], vertices[2]});
    return triangles;
}

// -------------------------------------------------------------------------
// PHASE 2: HERTEL-MEHLHORN MERGING (OPTIMIZATION)
// -------------------------------------------------------------------------

template <class VecType>
std::vector<std::vector<VecType>> decompose_to_convex_optimal(const std::vector<VecType>& poly) {
    using Scalar = typename VecType::Scalar;

    // 1. Get raw triangles
    std::vector<std::vector<VecType>> convex_parts = triangulate(poly);

    // 2. Greedy merge adjacent parts if the result remains convex
    bool merged_this_pass = true;
    while (merged_this_pass) {
        merged_this_pass = false;

        for (size_t i = 0; i < convex_parts.size(); ++i) {
            for (size_t j = i + 1; j < convex_parts.size(); ++j) {
                auto& p1 = convex_parts[i];
                auto& p2 = convex_parts[j];

                // Find shared edge (if any)
                int e1_idx = -1, e2_idx = -1;
                for (size_t a = 0; a < p1.size(); ++a) {
                    size_t a_next = (a + 1) % p1.size();
                    for (size_t b = 0; b < p2.size(); ++b) {
                        size_t b_next = (b + 1) % p2.size();

                        // Edges must be exact reverse of each other in CCW polygons
                        // Floating point exact match is usually safe here since they share memory copies from triangulate,
                        // but you can add an epsilon length check here if generating from dirty data.
                        if (p1[a] == p2[b_next] && p1[a_next] == p2[b]) {
                            e1_idx = static_cast<int>(a);
                            e2_idx = static_cast<int>(b);
                            break;
                        }
                    }
                    if (e1_idx != -1) break;
                }

                if (e1_idx == -1) continue; // No shared edge

                // We found a shared edge. If we merge, we remove this edge.
                // We must check the two new angles formed at the weld points.
                int a_prev = (e1_idx - 1 + p1.size()) % p1.size();
                int a_next = (e1_idx + 2) % p1.size();

                int b_prev = (e2_idx - 1 + p2.size()) % p2.size();
                int b_next = (e2_idx + 2) % p2.size();

                // Check convexity at Weld Point 1 (p1[e1_idx])
                Scalar cross1 = cross_2d(p1[a_prev], p1[e1_idx], p2[b_next]);
                // Check convexity at Weld Point 2 (p1[a_next])
                Scalar cross2 = cross_2d(p2[b_prev], p1[e1_idx + 1 == p1.size() ? 0 : e1_idx + 1], p1[a_next]);

                // If both are strictly convex, we can merge!
                if (cross1 > 0 && cross2 > 0) {
                    std::vector<VecType> merged;
                    merged.reserve(p1.size() + p2.size() - 2);

                    // Push p1 points, skipping the shared edge
                    for (size_t k = 0; k < p1.size(); ++k) {
                        merged.push_back(p1[(e1_idx + 1 + k) % p1.size()]);
                    }
                    // Push p2 points, skipping the shared edge
                    for (size_t k = 0; k < p2.size() - 2; ++k) {
                        merged.push_back(p2[(e2_idx + 2 + k) % p2.size()]);
                    }

                    convex_parts[i] = std::move(merged);
                    convex_parts.erase(convex_parts.begin() + j);

                    merged_this_pass = true;
                    break; // Break inner loop to restart the sweep
                }
            }
            if (merged_this_pass) break;
        }
    }

    return convex_parts;
}
