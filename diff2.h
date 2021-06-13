#ifndef __DIFF2_H_
#define __DIFF2_H_

#include <cmath>

#include "vec.h"
#include "mat_triangle.h"

template<int K, class T>
class Diff2 {
    T x;
    Vec<K, T> dx;
    MatTriangle<K, T> d2x;

public:

    Diff2(T x) : x(x) {}
    
    static auto from_var(T x, int k) {
        Diff2 r(x);
        r.dx.get_(k) = 1;
        return r;
    }

    operator T() const {
        return x;
    }

    template<class T2>
    inline auto operator+(const Diff2<K, T2> &v) const {
        auto xcp = *this;
        xcp.x += v.x;
        xcp.dx += v.dx;
        xcp.d2x += v.d2x;
        return xcp;
    }

    template<class T2>
    inline auto &operator+=(const Diff2<K, T2> &v) {
        x += v.x;
        dx += v.dx;
        d2x += v.d2x;
        return *this;
    }

    template<class T2>
    inline auto operator+(const T2 v) const {
        auto xcp = *this;
        xcp.x += v;
        return xcp;
    }

    template<class T2>
    inline auto &operator+=(const T2 v) {
        x += v;
        return *this;
    }

    template<class T2>
    inline auto operator-(const Diff2<K, T2> &v) const {
        auto xcp = *this;
        xcp.x -= v.x;
        xcp.dx -= v.dx;
        xcp.d2x -= v.d2x;
        return xcp;
    }

    template<class T2>
    inline auto &operator-=(const Diff2<K, T2> &v) {
        x -= v.x;
        dx -= v.dx;
        d2x -= v.d2x;
        return *this;
    }

    template<class T2>
    inline auto operator-(const T2 v) const {
        auto xcp = *this;
        xcp.x -= v;
        return xcp;
    }

    template<class T2>
    inline auto &operator-=(const T2 v) {
        x -= v;
        return *this;
    }

    template<class T2>
    inline auto &operator*=(const Diff2<K, T2> &v) const {
        auto x_ = x;
        auto dx_ = dx;
        x *= v.x;
        dx = dx_ * v.x + v.dx * x_;
        d2x = d2x * v.x + v.d2x * x_ + MatTriangle<K, T>::from_2vec(dx_, v.dx);
        return *this;
    }

    template<class T2>
    inline auto operator*(const Diff2<K, T2> &v) const {
        auto xcp = *this;
        xcp *= v;
        return xcp;
    }

    template<class T2>
    inline auto &operator*=(const T2 v) {
        x *= v;
        dx *= v;
        d2x *= v;
        return *this;
    }

    template<class T2>
    inline auto operator*(const T2 v) const {
        auto xcp = *this;
        xcp *= v;
        return xcp;
    }

    template<class T2>
    inline auto &operator^=(const T2 a) {
        auto p = std::pow(x, a);
        auto p_1 = p / x;
        auto p_2 = p_1 / x;
        auto dx_ = dx;

        x = p_1;
        dx = dx_ * (a * p_1);
        d2x = d2x * (a * p_1) + MatTriangle<K, T>::from_1vec(dx_) * (a * (a-1) * p_2);

        return *this;
    }

    template<class T2>
    inline auto operator^(const T2 v) const {
        auto xcp = *this;
        xcp ^= v;
        return xcp;
    }
};

#endif /* __DIFF2_H_ */
