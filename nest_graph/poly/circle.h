#pragma once

#include <algorithm>
#include <stdexcept>
#include <vector>
#include <random>
#include <cmath>


// -------------------------------------------------------------------------
// N-DIMENSIONAL CIRCLE PRIMITIVE
// -------------------------------------------------------------------------
template <class VecType>
class Circle {
public:
    using Scalar = typename VecType::Scalar;

    VecType c;
    Scalar r_sq;

    Circle() : c(), r_sq(static_cast<Scalar>(0)) {}

    Circle(const VecType& center, Scalar r_sq_in) : c(center), r_sq(r_sq_in) {}

    // 2-point fallback (Diameter)
    static Circle from(const VecType& A, const VecType& B) {
        Scalar half = static_cast<Scalar>(0.5);
        Scalar quarter = static_cast<Scalar>(0.25);

        return Circle((A + B) * half, (B - A).len_sq() * quarter);
    }

    // 3-point Circumcircle (N-Dimensional)
    static Circle from(const VecType& A, const VecType& B, const VecType& C) {
        // Offset to origin A
        VecType U = B - A;
        VecType V = C - A;

        Scalar d_UU = U.len_sq();
        Scalar d_VV = V.len_sq();
        Scalar d_UV = U.dp(V);

        // Determinant of the Gram matrix (equivalent to squared cross product length)
        Scalar D = d_UU * d_VV - d_UV * d_UV;

        Scalar half = static_cast<Scalar>(0.5);
        Scalar quarter = static_cast<Scalar>(0.25);

        // Collinear fallback
        if (std::abs(D) < static_cast<Scalar>(1e-8) * std::max(d_UU, d_VV)) {
            Scalar d_W = (C - B).len_sq();

            if (d_UU >= d_VV && d_UU >= d_W) {
                return Circle(A + U * half, d_UU * quarter);
            }
            if (d_VV >= d_UU && d_VV >= d_W) {
                return Circle(A + V * half, d_VV * quarter);
            }
            return Circle(A + (U + V) * half, d_W * quarter);
        }

        // Barycentric-style circumcenter projection
        Scalar inv_2D = half / D;

        Scalar alpha = d_VV * (d_UU - d_UV) * inv_2D;
        Scalar beta  = d_UU * (d_VV - d_UV) * inv_2D;

        VecType offset = U * alpha + V * beta;

        return Circle(A + offset, offset.len_sq());
    }

    static Circle from(const VecType* pts, int n) {
        switch (n) {
            case 1:
                return Circle(pts[0], static_cast<Scalar>(0));
            case 2:
                return from(pts[0], pts[1]);
            case 3:
                return from(pts[0], pts[1], pts[2]);
            default:
                throw std::invalid_argument("Invalid points number for exact fit");
        }
    }

    // Added an epsilon check to prevent floating-point snapping errors on boundaries
    bool is_inside(const VecType& p, Scalar epsilon = static_cast<Scalar>(1e-8)) const {
        return p.qdist(c) <= r_sq + epsilon;
    }

    const VecType& center() const {
        return c;
    }

    Scalar square_radius() const {
        return r_sq;
    }
};

// -------------------------------------------------------------------------
// EXACT ITERATIVE BOUNDING CIRCLE (Welzl's Iterative / Skyum Algorithm)
// -------------------------------------------------------------------------
template <class VecType>
inline Circle<VecType> compute_exact_bounding_circle(
    const VecType* poly, int n, std::mt19937 &rand_gen
) {
    using Scalar = typename VecType::Scalar;
    if (n == 0) return Circle<VecType>(VecType{}, static_cast<Scalar>(0));
    if (n == 1) return Circle<VecType>(poly[0], static_cast<Scalar>(0));

    // 1. Shuffling is STRICTLY REQUIRED for O(N) expected performance.
    std::vector<VecType> P(poly, poly + n);
    std::shuffle(P.begin(), P.end(), rand_gen);

    // 2. Start with the smallest possible circle (a point)
    Circle<VecType> c(P[0], static_cast<Scalar>(0));

    // 3. The 3-Nested-Loop Magic
    for (int i = 1; i < n; ++i) {
        if (!c.is_inside(P[i])) {
            // If point is outside, it MUST be on the boundary of the new circle
            c = Circle<VecType>(P[i], static_cast<Scalar>(0));

            for (int j = 0; j < i; ++j) {
                if (!c.is_inside(P[j])) {
                    // If j is outside, both i and j must be on the boundary
                    c = Circle<VecType>::from(P[i], P[j]);

                    for (int k = 0; k < j; ++k) {
                        if (!c.is_inside(P[k])) {
                            // If k is outside, i, j, and k define the circle
                            c = Circle<VecType>::from(P[i], P[j], P[k]);
                        }
                    }
                }
            }
        }
    }
    return c;
}
