#include <iostream>

using namespace std;

#include "algo.h"
// #include "mat.h"

// #include "diff2.h"

using namespace std;

int main() {
    // Mat<3, 3, double> m1(1.), m2(3.);
    // cout << m1.qdiff(m2) << endl;
    // Vec<2, double> v1(0., 0.), v2(0., 1.), v3(0., 1.);
    // cout << v2+v3 << endl;
    // cout << is_point_on_left(v1, v2, v3) << endl;

    // auto a = Diff2<3, double>::from_var(1., 1);
    // auto b = Diff2<3, double>::from_var(2., 1);
    // auto c = a * b;
    // a ^= 5.;
    // cout << ((-1) % 5) << endl;

    Vec<2, double> p1_data[] = {{0, 1}, {1, 2}, {1, 1}, {0, 0}};
    Vec<2, double> p2_data[] = {{3, 2}, {3, 0}, {1, 1.5}};

    auto p1_it = PolygonIterator(p1_data, sizeof(p1_data) / sizeof(p1_data[0]));
    auto p2_it = PolygonIterator(p2_data, sizeof(p2_data) / sizeof(p2_data[0]));

    // Vec<2, double> v1 = {1, 0};
    // Vec<2, double> v2 = {0, 1};
    // cout << turns_left(v1, v2) << endl;
    // cout << convex_polygons_intersect(p1_it, p2_it) << endl;
    // cout << convex_polygons_intersect(p1_it, p2_it, true) << endl;
    cout << convex_polygons_qdist(p1_it, p2_it) << endl;

    return 0;
}
