#include <fstream>
#include <iostream>
#include <iterator>
#include <opencv2/opencv.hpp>
#include <ostream>
#include <string>
#include <vector>

#include "matcher.h"
#include "opencv2/core/types.hpp"

using namespace std;

auto load_points(const char* fname) {
    std::vector<Vec<3, double>> ret;

    ifstream f;
    f.open(fname);

    double mx = 1e18, my = 1e18, mz = 1e18;

    double x, y, z;
    while (f >> x >> y >> z) {
        ret.emplace_back(x, y, z);
        mx = min(mx, x);
        my = min(my, y);
        mz = min(mz, z);
    }

    for (auto& v : ret) {
        v.get_(0) -= mx;
        v.get_(1) -= my;
        v.get_(2) -= mz;
    }

    f.close();

    return ret;
}

void save_points(const char* fname, std::vector<Vec<3, double>>& pts) {
    ofstream f;
    f.open(fname);

    for (auto& v : pts) {
        f << v.get_(0) << "," << v.get_(1) << "," << v.get_(2) << endl;
    }

    f.close();
}

int main() {
    auto v = load_points("/home/pietrek/Downloads/margaret2.csv");
    cout << v.size() << endl;
    // save_points("/home/pietrek/Downloads/b.csv", v);
    auto m1 = PlaneMatcher<double>({5, 5, 2}, .05, .05);
    auto m2 = PlaneMatcher<double>({18, 18, 18}, .1, 1.);

    int n_start = -1;

    for (int i = 0; i < 120; i++) {
        std::vector<Vec<3, double>> v1;
        std::vector<Vec<3, double>> v2;
        for (auto& p : v) {
            auto s1 = m1.match_point(p);
            // auto s2 = m2.match_point(p);
            auto s2 = m2.match_point(p);
            // cout << s1 << endl;
            if (s1 < s2) {
                if (s1 <= 1.) {
                    m1.update_score(p);
                    v1.push_back(p);
                }
            } else {
                if (s2 <= 1.) {
                    m2.update_score(p);
                    v2.push_back(p);
                }
            }
        }

        cout << m1.n << " " << m1.center << " " << m1.orto << endl;
        m1.accum_fit(1.);
        m2.accum_fit(1.);

        save_points(
            ("/home/pietrek/Downloads/test/1_" + to_string(i) + ".csv").c_str(),
            v1);
        save_points(
            ("/home/pietrek/Downloads/test/2_" + to_string(i) + ".csv").c_str(),
            v2);
    }

    // load_points("/home/pietrek/Downloads/b.csv");
    // auto m = PlaneMatcher<double>({0, 0, 0}, .1, .1);

    // Vec<3, double> pp[] = {
    //     {0, 0, 0},
    //     {1, 0, 0},
    //     {1, 1.1, 0},
    //     {.1, 1.1, 0},
    // };
    // for (int i = 0; i < 10; i++) {
    //     for (int i = 0; i < sizeof(pp) / sizeof(pp[0]); i++) {
    //         // cout << m.match_point(pp[i]) << " ";
    //         m.update_score(pp[i]);
    //     }
    //     // cout << endl;

    //     m.accum_fit(1);
    //     cout << m.center << " " << m.orto << endl;
    // }

    return 0;
}