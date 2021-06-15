#ifndef __MAT_H_
#define __MAT_H_

#include "vec.h"

template <int K, int N, class T>
class Mat {
    T items[K][N];

    inline T& get_(int k, int n) {
        // TODO: safety if?
        return items[k][n];
    }

    template <int IT, class Tv>
    inline void set_diag_vec(const Tv& v) {
        get_(IT, IT) = v.get(IT);
        if constexpr (IT > 0) {
            set_diag_vec<IT - 1, Tv>(v);
        }
    }

    template <int IT, class Tv>
    inline void set_diag_one(Tv v) {
        get_(IT, IT) = v;
        if constexpr (IT > 0) {
            set_diag_one<IT - 1, Tv>(v);
        }
    }

    template <int ITK, int ITN, class T2>
    inline T calc_qdiff_n(const Mat<K, N, T2>& m2) const {
        T v = get_(ITK, ITN) - m2.get_(ITK, ITN);
        T r = v * v;
        if constexpr (ITN > 0) {
            r += calc_qdiff_n<ITK, ITN - 1, T2>(m2);
        }
        return r;
    }

    template <int ITK, int ITN, class T2>
    inline T calc_qdiff_k(const Mat<K, N, T2>& m2) const {
        T r = calc_qdiff_n<ITK, ITN, T2>(m2);
        if constexpr (ITK > 0) {
            r += calc_qdiff_k<ITK - 1, ITN, T2>(m2);
        }
        return r;
    }

    template <int ITK, int ITN, class T2>
    inline void accum_sum_n(const Mat<K, N, T2>& m2) {
        get_(ITK, ITN) += m2.get_(ITK, ITN);
        if constexpr (ITN > 0) {
            accum_sum_n<ITK, ITN - 1, T2>(m2);
        }
    }

    template <int ITK, int ITN, class T2>
    inline void accum_sum_k(const Mat<K, N, T2>& m2) {
        accum_sum_n<ITK, ITN, T2>(m2);
        if constexpr (ITK > 0) {
            accum_sum_k<ITK - 1, ITN, T2>(m2);
        }
    }

    template <int ITK, int ITN, class T2>
    inline void accum_sub_n(const Mat<K, N, T2>& m2) {
        get_(ITK, ITN) -= m2.get_(ITK, ITN);
        if constexpr (ITN > 0) {
            accum_sub_n<ITK, ITN - 1, T2>(m2);
        }
    }

    template <int ITK, int ITN, class T2>
    inline void accum_sub_k(const Mat<K, N, T2>& m2) {
        accum_sub_n<ITK, ITN, T2>(m2);
        if constexpr (ITK > 0) {
            accum_sub_k<ITK - 1, ITN, T2>(m2);
        }
    }

    template <int ITK1, int ITN1, int ITN2, int N2, class T2>
    inline void accum_mul_n2(
        Mat<K, N2, T2>& mout, const Mat<N, N2, T2>& m2) const {
        mout.get_(ITK1, ITN2) += get_(ITK1, ITN1) * m2.get_(ITN1, ITN2);
        if constexpr (ITN2 > 0) {
            accum_mul_n2<ITK1, ITN1, ITN2 - 1, N2, T2>(mout, m2);
        }
    }

    template <int ITK1, int ITN1, int ITN2, int N2, class T2>
    inline void accum_mul_n1(
        Mat<K, N2, T2>& mout, const Mat<N, N2, T2>& m2) const {
        accum_mul_n2<ITK1, ITN1, ITN2, N2, T2>(mout, m2);
        if constexpr (ITN1 > 0) {
            accum_mul_n1<ITK1, ITN1 - 1, ITN2, N2, T2>(mout, m2);
        }
    }

    template <int ITK1, int ITN1, int ITN2, int N2, class T2>
    inline void accum_mul_k1(
        Mat<K, N2, T2>& mout, const Mat<N, N2, T2>& m2) const {
        accum_mul_n1<ITK1, ITN1, ITN2, N2, T2>(mout, m2);
        if constexpr (ITK1 > 0) {
            accum_mul_k1<ITK1 - 1, ITN1, ITN2, N2, T2>(mout, m2);
        }
    }

   public:
    Mat() : items{0} {}

    template <class Tv>
    Mat(const Tv diag_val) {
        if constexpr (N < K) {
            set_diag_one<N - 1, Tv>(diag_val);
        } else {
            set_diag_one<K - 1, Tv>(diag_val);
        }
    }

    template <int Kv, class Tv>
    Mat(const Vec<Kv, Tv>& diag_vec) {
        if constexpr (N < K) {
            set_diag_vec<N - 1, Tv>(diag_vec);
        } else {
            set_diag_vec<K - 1, Tv>(diag_vec);
        }
    }

    template <class T2>
    inline T qdiff(const Mat<K, N, T2>& m2) const {
        return calc_qdiff_k<K - 1, N - 1, T2>(m2);
    }

    template <class T2>
    inline auto operator+(const Mat<K, N, T2>& m2) const {
        auto mcp = *this;
        mcp.template accum_sum_k<K - 1, N - 1, T2>(m2);
        return mcp;
    }

    template <class T2>
    inline auto& operator+=(const Mat<K, N, T2>& m2) {
        accum_sum_k<K - 1, N - 1, T2>(m2);
        return *this;
    }

    template <class T2>
    inline auto operator-(const Mat<K, N, T2>& m2) const {
        auto mcp = *this;
        mcp.template accum_sub_k<K - 1, N - 1, T2>(m2);
        return mcp;
    }

    template <class T2>
    inline auto& operator-=(const Mat<K, N, T2>& m2) {
        accum_sub_k<K - 1, N - 1, T2>(m2);
        return *this;
    }

    template <int N2, class T2>
    inline auto operator*(const Mat<N, N2, T2>& m2) const {
        Mat<K, N2, T> mout;
        accum_mul_k1<K - 1, N - 1, N2 - 1, N2, T2>(mout, m2);
        return mout;
    }
};

#endif /* __MAT_H_ */
