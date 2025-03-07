#include <cmath>
#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

using namespace std;

#include "algo.h"
#include "nesting.h"

using namespace std;

int main() {
    // Mat<3, 3, double> m1(1.), m2(3.);
    // cout << m1.qdiff(m2) << endl;
    // Vec<2, double> v1(0., 0.), v2(0., 1.), v3(0., 1.);
    // cout << v2+v3 << endl;
    // cout << is_point_on_left(v1, v2, v3) << endl;

    // auto a = Diff2<3, double>::from_var(1., 0);
    // auto b = Diff2<3, double>::from_var(2., 2);
    // auto c = a * b;
    // c ^= .5;
    // c = c.cossin().cos;
    // cout << c.get_x() << endl;
    // cout << c.get_dx() << endl;
    // cout << c.get_d2x() << endl;

    // Vec<2, double> p1_data[] = {{0, 1}, {1, 2}, {1, 1}, {0, 0}};
    // Vec<2, double> p2_data[] = {{3, 2}, {3, 0}, {1, 0}};

    // auto p1_it = PolygonIterator(p1_data, sizeof(p1_data) / sizeof(p1_data[0]));
    // auto p2_it = PolygonIterator(p2_data, sizeof(p2_data) / sizeof(p2_data[0]));

    // Vec<2, double> v1 = {1, 0};
    // Vec<2, double> v2 = {0, 1};
    // cout << turns_left(v1, v2) << endl;
    // cout << convex_polygons_intersect(p1_it, p2_it) << endl;
    // cout << convex_polygons_intersect(p1_it, p2_it, true) << endl;
    // cout << convex_polygons_qdist(p1_it, p2_it) << endl;

    // Vec<2, double> p1 = {0., 0.};
    // Vec<2, double> p2 = {1., 0.};
    // Vec<2, double> p3 = {0., 2.};
    // auto d = p2 - p1;
    // auto v = p3 - p2;
    // cout << segment_qdist(d, v) << endl;

    std::vector<Vec<2, double>> pborder_data = {{0, 0}, {0, 10}, {10, 10}, {10, 0}};
    std::vector<Vec<2, double>> p_data = {{0, 0}, {0, 1}, {1, 1}, {1, 0}};

    cv::Mat_<double> x(12, 1), v(12, 1), m(12, 12);
    x = {0, 0, 0, 1, 1, M_PI/4, 5, 6, -M_PI/3, 3, 2, 0};
    std::cout << x << std::endl;

    for (int i=0; i<5; i++) {
        v.setTo(0);
        m.setTo(0);

        std::vector<Vec<2, Diff2<3, double>>> pborder_grad(4), p1_grad(4), p2_grad(4), p3_grad(4);
        transform_points_g3(&pborder_grad[0], &pborder_data[0], pborder_data.size(), {x.at<double>(0), x.at<double>(1)}, x.at<double>(2));
        transform_points_g3(&p1_grad[0], &p_data[0], p_data.size(), {x.at<double>(3), x.at<double>(4)}, x.at<double>(5));
        transform_points_g3(&p2_grad[0], &p_data[0], p_data.size(), {x.at<double>(6), x.at<double>(7)}, x.at<double>(8));
        transform_points_g3(&p3_grad[0], &p_data[0], p_data.size(), {x.at<double>(9), x.at<double>(10)}, x.at<double>(11));

        Poly2DGradPtr<double> grad_ptrs[] = {
            {.size = pborder_grad.size(), .coords = &pborder_grad[0], .grad_pos = {0, 1, 2}},
            {.size = p1_grad.size(), .coords = &p1_grad[0], .grad_pos = {3, 4, 5}},
            {.size = p2_grad.size(), .coords = &p2_grad[0], .grad_pos = {6, 7, 8}},
            {.size = p3_grad.size(), .coords = &p3_grad[0], .grad_pos = {9, 10, 11}},
        };
        accum_dist_gradients(v.ptr<double>(0), m.ptr<double>(0, 0), 12, grad_ptrs, 4, 0.1);

        // std::cout << m.inv() << std::endl;
        cv::Mat_<double> x2 = x - .4 * m.inv() * v;
        double xs = x2.at<double>(0), ys = x2.at<double>(1), as = x2.at<double>(2);
        x2.at<double>(0) -= xs;
        x2.at<double>(1) -= ys;
        x2.at<double>(2) -= as;
        x2.at<double>(3) -= xs;
        x2.at<double>(4) -= ys;
        x2.at<double>(5) -= as;
        x2.at<double>(6) -= xs;
        x2.at<double>(7) -= ys;
        x2.at<double>(8) -= as;
        x2.at<double>(9) -= xs;
        x2.at<double>(10) -= ys;
        x2.at<double>(11) -= as;
        std::cout << x2 << std::endl << std::endl;

        x = x2;
    }

    return 0;
}
