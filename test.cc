#include <iostream>

// #include "algo.h"
// #include "mat.h"

#include "diff2.h"

using namespace std;

int main() {
    // Mat<3, 3, double> m1(1.), m2(3.);
    // cout << m1.qdiff(m2) << endl;
    // Vec<2, double> v1(0., 0.), v2(0., 1.), v3(0., 1.);
    // cout << v2+v3 << endl;
    // cout << is_point_on_left(v1, v2, v3) << endl;
    auto a = Diff2<3, double>:: from_var(1., 1);
    auto b = Diff2<3, double>::from_var(2., 1);
    auto c = a * b;
    a ^= 5.;
    return 0;
}
