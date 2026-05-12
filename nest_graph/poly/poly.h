#pragma once

#include <cstddef>
#include <vector>
#include <algorithm>
#include <cmath>

#include "circle.h"


template<class T, class VecType>
class ConvexPolygon {
    public:

    std::vector<VecType> points;
    Circle<T, VecType> bounding_circle;

    ConvexPolygon() {}
    ConvexPolygon(const VecType* poly, int n)
        : points(poly, poly+n) {
        std::random_device rd;
        std::mt19937 gen(rd());
        bounding_circle = compute_exact_bounding_circle<T, VecType>(
            poly, n, gen
        );
    }
};

template<class T, class VecType>
class Polygon {
    typedef struct {
        std::size_t start_point;
        Circle<T, VecType> bounding_circle;
    } ConvexPolygonEntry;

public:
    std::vector<ConvexPolygonEntry> convex_parts;
    std::vector<VecType> poly_points;
    std::vector<ConvexPolygonEntry> convex_holes;
    std::vector<VecType> hole_points;
    Circle<T, VecType> bounding_circle;

    void append_convex_poly(const VecType* poly, int n) {
        std::random_device rd;
        std::mt19937 gen(rd());
        convex_parts.push_back({
            poly_points.size(),
            compute_exact_bounding_circle<T, VecType>(poly, n, gen)
        });
        poly_points.insert(poly_points.end(), poly, poly+n);
    }

    void append_convex_hole(const VecType* hole, int n) {
        std::random_device rd;
        std::mt19937 gen(rd());
        convex_holes.push_back({
            hole_points.size(),
            compute_exact_bounding_circle<T, VecType>(hole, n, gen)
        });
        hole_points.insert(hole_points.end(), hole, hole+n);
    }

    void finalize() {
        std::random_device rd;
        std::mt19937 gen(rd());
        bounding_circle = compute_exact_bounding_circle<T, VecType>(
            poly_points.data(),
            static_cast<int>(poly_points.size()),
            gen
        );
        poly_points.shrink_to_fit();
        hole_points.shrink_to_fit();
    }

    const Circle<T, VecType>& get_bounding_circle() const {
        return bounding_circle;
    }

    // Helpers to safely extract arrays and sizes for GJK
    int get_part_size(size_t part_idx) const {
        if (part_idx + 1 < convex_parts.size()) {
            return convex_parts[part_idx + 1].start_point - convex_parts[part_idx].start_point;
        }
        return poly_points.size() - convex_parts[part_idx].start_point;
    }

    const VecType* get_part_points(size_t part_idx) const {
        return &poly_points[convex_parts[part_idx].start_point];
    }

    int get_hole_size(size_t hole_idx) const {
        if (hole_idx + 1 < convex_holes.size()) {
            return convex_holes[hole_idx + 1].start_point - convex_holes[hole_idx].start_point;
        }
        return hole_points.size() - convex_holes[hole_idx].start_point;
    }

    const VecType* get_hole_points(size_t hole_idx) const {
        return &hole_points[convex_holes[hole_idx].start_point];
    }
};
