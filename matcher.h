#ifndef __MATCHER_H_
#define __MATCHER_H_

#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ostream>

#include "bbox.h"
#include "diff2.h"
#include "vec.h"

template <class T>
class PlaneMatcher {
   public:
    using Tdiff = Diff2<6, T>;

    Vec<3, T> orto, center;
    Tdiff score = 0;
    T orto_weight = 0;
    T dist_weight = 0;
    int n = 0;

    inline Vec<3, Tdiff> v3grad(Vec<3, T> v, int p) {
        return {
            Tdiff::from_var(v[0], p),
            Tdiff::from_var(v[1], p + 1),
            Tdiff::from_var(v[2], p + 2),
        };
    }

    inline Vec<3, Tdiff> v3nograd(Vec<3, T> v) {
        return {
            Tdiff(v[0]),
            Tdiff(v[1]),
            Tdiff(v[2]),
        };
    }

    inline Vec<3, Tdiff> orto_grad() {
        return v3grad(orto, 0);
    };

    inline Vec<3, Tdiff> center_grad() {
        return v3grad(center, 3);
    };

   public:
    PlaneMatcher(Vec<3, T> center, T dist_weight, T orto_weight)
        : center(center),
          dist_weight(dist_weight),
          orto_weight(orto_weight),
          orto({1, 1, 1}) {
        reset_score();
    }

    void reset_score() {
        score = 0;
        n = 0;
    }

    // For <1.0 point matches plane
    T match_point(Vec<3, T> p) {
        p -= center;
        auto orto_dp = p.dp(orto);
        auto qdist = p.qlen();
        return .4 + pow(qdist, .6) * dist_weight -
               pow(orto_dp * orto_dp + .4, -1.85) * orto_weight;  // ?????????
    }

    void update_score(Vec<3, T> p_) {
        n++;

        auto p = v3nograd(p_) - center_grad();
        auto qdist = p.qlen();
        auto o = orto_grad();
        auto orto_dp = p.dp(o);
        score += ((qdist ^ .6) * dist_weight) -
                 //  ((orto_dp * orto_dp + .4) ^ -1.85) * orto_weight;
                 orto_dp * orto_dp * orto_weight;
        // score += orto_dp * orto_dp + qdist * dist_weight;
        // std::cout << (double)(orto_dp * orto_dp + qdist * dist_weight) << std::endl;
    }

    void accum_fit(double fit_speed) {
        if (n) {
            auto score_grad = score; // / n; // - (orto_grad().abssum() ^ .5) * orto_weight;
            // std::cout << "!!!!!!!!!! " << (score_grad * score_grad).get_d2x() << std::endl;
            // std::cout << "!!!!!!!!!! " << (score_grad ^ 2.).get_d2x()
            //           << std::endl;
            //   << score_grad << std::endl;

            cv::Vec<double, 6> dx_cv = 0;
            score_grad.get_dx().add_to_vec(dx_cv, {0, 1, 2, 3, 4, 5});

            cv::Mat d2x_cv(6, 6, CV_64F, cv::Scalar(0));
            score_grad.get_d2x().add_to_mat(d2x_cv, {0, 1, 2, 3, 4, 5});

            cv::Mat diff_cv = d2x_cv.inv() * (dx_cv * fit_speed);
            
            auto orto_diff = Vec<3, T>(
                diff_cv.at<T>(0, 0), diff_cv.at<T>(0, 1), diff_cv.at<T>(0, 2));
            auto center_diff = Vec<3, T>(
                diff_cv.at<T>(0, 3), diff_cv.at<T>(0, 4),
                diff_cv.at<T>(0, 5));
            auto center_diff_len = std::sqrt(center_diff.qlen());
            if (center_diff_len > .5) {
                center_diff *= .5 / center_diff_len;
            }

            orto -= orto_diff;
            orto *= (1. / std::sqrt(orto.qlen()));
            center -= center_diff;
        }

        reset_score();
    }

    auto bbox() {
        Vec<3, T> a = 1. / std::sqrt(dist_weight);
        return BBox<3, T>(center - a, center + a);
    }
};

#endif /* __MATCHER_H_ */