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

    std::vector<Vec<2, double>> pborder_data = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    std::vector<Vec<2, double>> p_data = {{0, 0}, {0, 1}, {1, 1}, {1, 0}};

    cv::Mat_<double> x(9, 1), v(9, 1), m(9, 9);
    x = {0, 0, 0, 1, 1, M_PI/4, 8, 7, -M_PI/4};
    v.setTo(0);
    m.setTo(0);

    std::vector<Vec<2, Diff2<3, double>>> pborder_grad(4), p1_grad(4), p2_grad(4);
    transform_points_g3(&pborder_grad[0], &pborder_data[0], pborder_data.size(), {x.at<double>(0), x.at<double>(1)}, x.at<double>(2));
    transform_points_g3(&p1_grad[0], &p_data[0], p_data.size(), {x.at<double>(3), x.at<double>(4)}, x.at<double>(5));
    transform_points_g3(&p2_grad[0], &p_data[0], p_data.size(), {x.at<double>(6), x.at<double>(7)}, x.at<double>(8));

    Poly2DGradPtr<double> grad_ptrs[] = {
        {.size = pborder_grad.size(), .coords = &pborder_grad[0], .grad_pos = {0, 1, 2}},
        {.size = p1_grad.size(), .coords = &p1_grad[0], .grad_pos = {3, 4, 5}},
        {.size = p2_grad.size(), .coords = &p2_grad[0], .grad_pos = {6, 7, 8}},
    };
    accum_dist_gradients(v.ptr<double>(0), m.ptr<double>(0, 0), 9, grad_ptrs, 3, 0.0);

    std::cout << x << std::endl;
    // std::cout << m.inv() << std::endl;
    std::cout << x - .03f * m.inv() * v << std::endl;

    return 0;
}
