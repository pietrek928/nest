#include <iostream>
#include <opencv2/opencv.hpp>

#include "matcher.h"
#include "opencv2/core/types.hpp"
using namespace std;

int main() {
    auto m = PlaneMatcher<double>({0, 0, 0}, .1, 1.);

    for (int i = 0; i < 40; i++) {
        Vec<3, double> pp[] = {
            {1, 1, 1},
            {-1, 0, -1},
            {1, 1, 0},
            {-1.1, .1, -1.1},
        };
        for (int i = 0; i < sizeof(pp) / sizeof(pp[0]); i++) {
            cout << m.match_point(pp[i]) << " ";
            m.update_score(pp[i]);
        }
        cout << endl;

        m.accum_fit(1.);
    }

    return 0;
}