#ifndef __DIFF2_H_
#define __DIFF2_H_

#include <cmath>

#include "mat_triangle.h"
#include "vec.h"

template <int K, class T>
class Diff2 {
    T x;
    Vec<K, T> dx;
    MatTriangle<K, T> d2x;

   public:
    Diff2() : x(0) {}
    Diff2(T x) : x(x) {}

    static auto from_var(T x, int k) {
        Diff2 r(x);
        r.dx.get_(k) = 1;
        return r;
    }

    operator T() const {
        return x;
    }

    inline auto get_dx() const {
        return dx;
    }

    inline auto get_d2x() const {
        return d2x;
    }

    template <class T2>
    inline auto operator+(const Diff2<K, T2>& v) const {
        auto xcp = *this;
        xcp.x += v.x;
        xcp.dx += v.dx;
        xcp.d2x += v.d2x;
        return xcp;
    }

    template <class T2>
    inline auto& operator+=(const Diff2<K, T2>& v) {
        x += v.x;
        dx += v.dx;
        d2x += v.d2x;
        return *this;
    }

    template <class T2>
    inline auto operator+(const T2 v) const {
        auto xcp = *this;
        xcp.x += v;
        return xcp;
    }

    template <class T2>
    inline auto& operator+=(const T2 v) {
        x += v;
        return *this;
    }

    template <int K2, class T2>
    inline auto& add_mapped(const Diff2<K2, T2>& v2, const Vec<K2, int>& mapping) {
        x += v2.x;
        dx.add_vec_mapped(v2.dx, mapping);
        d2x.add_mat_triangle(v2.d2x, mapping);
        return *this;
    }

    template <class T2>
    inline auto operator-(const Diff2<K, T2>& v) const {
        auto xcp = *this;
        xcp.x -= v.x;
        xcp.dx -= v.dx;
        xcp.d2x -= v.d2x;
        return xcp;
    }

    template <class T2>
    inline auto& operator-=(const Diff2<K, T2>& v) {
        x -= v.x;
        dx -= v.dx;
        d2x -= v.d2x;
        return *this;
    }

    template <class T2>
    inline auto operator-(const T2 v) const {
        auto xcp = *this;
        xcp.x -= v;
        return xcp;
    }

    template <class T2>
    inline auto& operator-=(const T2 v) {
        x -= v;
        return *this;
    }

    template <class T2>
    inline auto& operator*=(const Diff2<K, T2>& v) {
        auto x_ = x;
        auto dx_ = dx;
        x *= v.x;
        dx = dx_ * v.x + v.dx * x_;
        d2x = d2x * v.x + v.d2x * x_ + MatTriangle<K, T>::from_2vec(dx_, v.dx);
        return *this;
    }

    template <class T2>
    inline auto operator*(const Diff2<K, T2>& v) const {
        auto xcp = *this;
        // std::cout << xcp.d2x << std::endl;
        xcp *= v;
        return xcp;
    }

    inline auto square() {
        return *this * *this;
    }

    template <class T2>
    inline auto& operator*=(const T2 v) {
        x *= v;
        dx *= v;
        d2x *= v;
        return *this;
    }

    template <class T2>
    inline auto operator*(const T2 v) const {
        auto xcp = *this;
        xcp *= v;
        return xcp;
    }

    inline auto inv() {
        auto xcp = *this;
        auto x_1 = T(1) / x;
        auto x_2 = x_1 * x_1;
        auto x_3 = x_2 * x_1;

        xcp.x = x_1;
        xcp.dx *= -x_2;
        xcp.d2x = d2x * -x_2 - MatTriangle<K, T>::from_1vec(dx) * (x_3 * 2);

        return xcp;
    }

    template <class T2>
    inline auto operator/(const Diff2<K, T2>& v) const {
        return *this * v.inv();
    }

    template <class T2>
    inline auto& operator/=(const Diff2<K, T2>& v) {
        return *this *= v.inv();
    }

    template <class T2>
    inline auto& operator/=(const T2 v) {
        auto a = T(1) / v;
        x *= a;
        dx *= a;
        d2x *= a;
        return *this;
    }

    template <class T2>
    inline auto operator/(const T2 v) const {
        auto xcp = *this;
        xcp /= v;
        return xcp;
    }

    template <class T2>
    inline auto& operator^=(const T2 a) {
        auto p = std::pow(x, a);
        auto p_1 = p / x;
        auto p_2 = p_1 / x;
        auto dx_ = dx;

        x = p;
        dx = dx_ * (a * p_1);
        d2x = d2x * (a * p_1) +
              MatTriangle<K, T>::from_1vec(dx_) * (a * (a - 1) * p_2);

        return *this;
    }

    template <class T2>
    inline auto operator^(const T2 v) const {
        auto xcp = *this;
        xcp ^= v;
        return xcp;
    }

    inline auto cossin() const {
        class {
            Diff2 cos, sin;
        } sc;

        auto sx = std::sin(x);
        auto cx = std::cos(x);
        auto dx_mul = MatTriangle<K, T>::from_1vec(dx);

        sc.cos.x = cx;
        sc.cos.dx = dx * (-sx);
        sc.sin.d2x = d2x * (-sx) - dx_mul * cx;
        sc.sin.x = sx;
        sc.sin.dx = dx * cx;
        sc.sin.d2x = d2x * cx - dx_mul * sx;

        return sc;
    }
};

template <int K, class T>
inline Diff2<K, T> operator+(const T v1, const Diff2<K, T> v2) {
    return v2 + v1;
}

template <int K, class T>
inline Diff2<K, T> operator-(const T v1, const Diff2<K, T> v2) {
    return Diff2<K, T>(v1) - v2;
}

template <int K, class T>
inline Diff2<K, T> operator*(const T v1, const Diff2<K, T> v2) {
    return v2 * v1;
}

template <int K, class T>
inline Diff2<K, T> operator/(const T v1, Diff2<K, T> v2) {
    return Diff2<K, T>(v1) / v2;
}

// template <int K, class T>
// inline Diff2<K, T> operator^(T v1, Diff2<K, T> v2) {
//     return Diff2<K, T>(v1) ^ v2;
// }  !!!!!!!!!!!!!!!!!

#endif /* __DIFF2_H_ */
