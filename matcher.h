#ifndef __MATCHER_H_
#define __MATCHER_H_

#include <iostream>
#include <opencv2/opencv.hpp>

#include "diff2.h"
#include "vec.h"

template <class T>
class PlaneMatcher {
    using Tdiff = Diff2<6, T>;

    Vec<3, T> orto, center;
    Tdiff score = 0;
    T orto_weight = 0;
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
        return v3grad(orto, 3);
    };

   public:
    PlaneMatcher(Vec<3, T> center, T orto_weight)
        : center(center), orto_weight(orto_weight), orto({1, 1, 1}) {
        reset_score();
    }

    void reset_score() {
        score = 0;
        n = 0;
    }

    T match_point(Vec<3, T> p) {
        p -= center;
        auto orto_dp = p.dp(orto);
        auto qdist = p.qlen();
        return orto_dp * orto_dp + qdist;  // ?????????
    }

    void update_score(Vec<3, T> p_) {
        n++;

        auto p = v3nograd(p_) - center_grad();
        // std::cout << p.get_d2x();
        auto orto_dp = p.dp(orto_grad());
        auto qdist = p.qlen();
        score += orto_dp * orto_dp + qdist;
    }

    void accum_fit(double fit_speed) {
        auto score_grad = score / n - orto_grad().qlen() * orto_weight;
        std::cout << "!!!!!!!!!! " << (double)score_grad << std::endl;

        cv::Vec<double, 6> dx_cv = 0;
        score_grad.get_dx().add_to_vec(dx_cv, {0, 1, 2, 3, 4, 5});

        cv::Mat d2x_cv(6, 6, CV_64F, cv::Scalar(0));
        score_grad.get_d2x().add_to_mat(d2x_cv, {0, 1, 2, 3, 4, 5});

        cv::Mat diff_cv = d2x_cv.inv() * (dx_cv * fit_speed);
        // std::cout << cv::format(diff_cv, cv::Formatter::FMT_C) << std::endl;

        orto -= Vec<3, T>(
            diff_cv.at<T>(0, 0), diff_cv.at<T>(0, 1), diff_cv.at<T>(0, 2));
        center -= Vec<3, T>(
            diff_cv.at<T>(0, 3), diff_cv.at<T>(0, 4), diff_cv.at<T>(0, 5));

        reset_score();
    }
};

#endif /* __MATCHER_H_ */