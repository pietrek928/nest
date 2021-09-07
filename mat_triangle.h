#ifndef __MAT_TRIANGLE_H_
#define __MAT_TRIANGLE_H_

#include "vec.h"

template <int K, class T>
class MatTriangle {
    static constexpr int items_count = K * (K + 1) / 2;

    T items[items_count];

    inline T& get_(int k, int n) const {
        // TODO: safety if?
        if (k >= n) {
            return items[k * (k + 1) / 2 + n];
        } else {
            return items[n * (n + 1) / 2 + k];
        }
    }

    template <int IT, class T2>
    inline void set_diag_vec(const Vec<K, T2>& v) {
        get_(IT, IT) = v.get(IT);
        if constexpr (IT > 0) {
            set_diag_vec<IT - 1, T2>(v);
        }
    }

    template <int IT>
    inline void set_diag_one(T v) {
        get_(IT, IT) = v;
        if constexpr (IT > 0) {
            set_diag_one<IT - 1>(v);
        }
    }

    template <int ITK, int ITN, class Tm>
    inline void add_to_mat_n(
        const Tm& M, const Vec<K, int>& pos_mapping) const {
        M[pos_mapping.get(ITK)][pos_mapping.get(ITN)] += get_(ITK, ITN);
        if constexpr (ITN > 0) {
            add_to_mat_n<ITK, ITN - 1, Tm>(pos_mapping);
        }
    }

    template <int ITK, class Tm>
    inline void add_to_mat_k(
        const Tm& M, const Vec<K, int>& pos_mapping) const {
        add_to_mat_n<ITK, K - 1, Tm>(M, pos_mapping);
        if constexpr (ITK > 0) {
            add_to_mat_k<ITK - 1, Tm>(pos_mapping);
        }
    }

    template <int IT, class T2>
    inline void accum_sum_it(const MatTriangle<K, T2>& m2) {
        items[IT] += m2.items[IT];
        if constexpr (IT > 0) {
            accum_sum_it<IT - 1, T2>(m2);
        }
    }

    template <int IT, class T2>
    inline void accum_sub_it(const MatTriangle<K, T2>& m2) {
        items[IT] -= m2.items[IT];
        if constexpr (IT > 0) {
            accum_sub_it<IT - 1, T2>(m2);
        }
    }

    template <int IT>
    inline void accum_mul_it(const T v) {
        items[IT] *= v;
        if constexpr (IT > 0) {
            accum_mul_it<IT - 1>(v);
        }
    }

    template <int ITK, int ITN, class T2>
    inline void accum_mul_1vec_n(const Vec<K, T2>& v2) {
        get_(ITK, ITN) += v2.get(ITK) * v2.get(ITN);
        if constexpr (ITN > 0) {
            accum_mul_1vec_n<ITK, ITN - 1, T2>(v2);
        }
    }

    template <int ITK, class T2>
    inline void accum_mul_1vec_k(const Vec<K, T2>& v2) {
        accum_mul_1vec_n<ITK, ITK, T2>(v2);
        if constexpr (ITK > 0) {
            accum_mul_1vec_k<ITK - 1, T2>(v2);
        }
    }

    template <int ITK, int ITN, class T1, class T2>
    inline void accum_mul_2vec_n(const Vec<K, T1>& v1, const Vec<K, T2>& v2) {
        if constexpr (ITK == ITN) {
            auto v = v1.get(ITK) * v2.get(ITN);
            get_(ITK, ITN) += v + v;
        } else {
            get_(ITK, ITN) +=
                v1.get(ITK) * v2.get(ITN) + v1.get(ITK) * v2.get(ITN);
        }
        if constexpr (ITN > 0) {
            accum_mul_2vec_n<ITK, ITN - 1, T1, T2>(v1, v2);
        }
    }

    template <int ITK, class T1, class T2>
    inline void accum_mul_2vec_k(const Vec<K, T1>& v1, const Vec<K, T2>& v2) {
        accum_mul_2vec_n<ITK, ITK, T1, T2>(v1, v2);
        if constexpr (ITK > 0) {
            accum_mul_2vec_k<ITK - 1, T1, T2>(v1, v2);
        }
    }

   public:
    MatTriangle() : items{0} {}

    MatTriangle(const T v) : items{0} {
        set_diag_one<K - 1>(v);
    }

    MatTriangle(const Vec<K, T>& v) : items{0} {
        set_diag_vec<K - 1>(v);
    }

    template <class T2>
    static auto from_1vec(const Vec<K, T2>& v2) {
        MatTriangle m;
        m.accum_mul_1vec_k<K - 1, T2>(v2);
        return m;
    }

    template <class T1, class T2>
    static auto from_2vec(const Vec<K, T1>& v1, const Vec<K, T2>& v2) {
        MatTriangle m;
        m.accum_mul_2vec_k<K - 1, T1, T2>(v1, v2);
        return m;
    }

    inline T get(int k, int n) const {
        // TODO: safety if?
        return get_(k, n);
    }

    // inline void set(int k, int n, T v) {
    // TODO: safety if?
    // items[k * (k+1) / 2 + n] = v;
    //}

    template <class Tm>
    inline void add_to_mat(const Tm& M, const Vec<K, int>& pos_mapping) const {
        add_to_mat_k<K - 1, Tm>(M, pos_mapping);
    }

    template <class T2>
    inline auto operator+(const MatTriangle<K, T2>& m2) const {
        auto mcp = *this;
        mcp.template accum_sum_it<items_count - 1, T2>(m2);
        return mcp;
    }

    template <class T2>
    inline auto& operator+=(const MatTriangle<K, T2>& m2) {
        accum_sum_it<items_count - 1, T2>(m2);
        return *this;
    }

    template <class T2>
    inline auto operator-(const MatTriangle<K, T2>& m2) const {
        auto mcp = *this;
        mcp.template accum_sub_it<items_count - 1, T2>(m2);
        return mcp;
    }

    template <class T2>
    inline auto& operator-=(const MatTriangle<K, T2>& m2) {
        accum_sub_it<items_count - 1, T2>(m2);
        return *this;
    }

    inline auto operator*(const T a) const {
        auto mcp = *this;
        mcp.template accum_mul_it<items_count - 1>(a);
        return mcp;
    }

    inline auto& operator*=(const T a) {
        accum_mul_it<items_count - 1>(a);
        return *this;
    }
};

#endif /* __MAT_TRIANGLE_H_ */
