#pragma once

#include <initializer_list>

template <int K, class T>
class Vec {
    T items[K];

    template <int IT, class T2>
    inline T calc_dp(const Vec<K, T2>& v2) const {
        T r = items[IT] * v2[IT];
        if constexpr (IT > 0) {
            r += calc_dp<IT - 1, T2>(v2);
        }
        return r;
    }

    template <int IT>
    inline T calc_sum() const {
        T r = items[IT];
        if constexpr (IT > 0) {
            r += calc_sum<IT - 1>();
        }
        return r;
    }

    template <int IT>
    inline T calc_qlen() const {
        auto& v = items[IT];
        T r = v * v;
        if constexpr (IT > 0) {
            r += calc_qlen<IT - 1>();
        }
        return r;
    }

    template <int IT>
    inline T calc_abssum() const {
        auto& v = items[IT];
        T r = v;
        if (r < 0.) {
            r = -r;
        }
        if constexpr (IT > 0) {
            r += calc_abssum<IT - 1>();
        }
        return r;
    }

    template <int IT>
    inline T calc_qdist(const Vec<K, T>& v2) const {
        T diff = items[IT] - v2[IT];
        T r = diff * diff;
        if constexpr (IT > 0) {
            r += calc_qdist<IT - 1>(v2);
        }
        return r;
    }

    template <int IT, class T2>
    inline void accum_sum(const Vec<K, T2>& v2) {
        items[IT] += v2[IT];
        if constexpr (IT > 0) {
            accum_sum<IT - 1, T2>(v2);
        }
    }

    template <int IT, class T2>
    inline void accum_sum(const T2& v2) {
        items[IT] += v2;
        if constexpr (IT > 0) {
            accum_sum<IT - 1, T2>(v2);
        }
    }

    template <int IT>
    inline void accum_neg() {
        items[IT] = -items[IT];
        if constexpr (IT > 0) {
            accum_neg<IT - 1>();
        }
    }

    template <int IT, class T2>
    inline void accum_sub(const Vec<K, T2>& v2) {
        items[IT] -= v2[IT];
        if constexpr (IT > 0) {
            accum_sub<IT - 1, T2>(v2);
        }
    }

    template <int IT, class T2>
    inline void accum_sub(const T2& v2) {
        items[IT] -= v2;
        if constexpr (IT > 0) {
            accum_sub<IT - 1, T2>(v2);
        }
    }

    template <int IT, class T2>
    inline void accum_mul(const Vec<K, T2>& v2) {
        items[IT] *= v2[IT];
        if constexpr (IT > 0) {
            accum_mul<IT - 1, T2>(v2);
        }
    }

    template <int IT, class T2>
    inline void accum_mul(const T2& v2) {
        items[IT] *= v2;
        if constexpr (IT > 0) {
            accum_mul<IT - 1, T2>(v2);
        }
    }

    template <int IT, class Tv>
    inline void add_to_vec_it(Tv *V, const Vec<K, int>& pos_mapping) const {
        V[pos_mapping[IT]] = items[IT];
        if constexpr (IT > 0) {
            add_to_vec_it<IT - 1, Tv>(V, pos_mapping);
        }
    }

    template <int IT, int K2>
    inline void add_vec_mapped_it(
        const Vec<K2, T>& V, const Vec<K2, int>& pos_mapping) {
        get_(pos_mapping[IT]) += V[IT];
        if constexpr (IT > 0) {
            add_vec_mapped_it<IT - 1, K2>(V, pos_mapping);
        }
    }

    template <class Tstream, int IT>
    inline void put_items_to(Tstream& os) const {
        os << items[IT];
        if constexpr (IT < K - 1) {
            os << ", ";
            put_items_to<Tstream, IT + 1>(os);
        }
    }

   public:
    using Tnum = T;

    inline Vec() : items{0} {}
    inline Vec(std::initializer_list<Tnum> il) {
        int i = 0;
        for (const auto& v : il) {
            items[i++] = v;
        }
    }

    template <int IT, class Tv, class... Targs>
    inline void set_pos(Tv v, Targs... vs) {
        items[IT] = v;
        if constexpr (IT < K - 1) {
            set_pos<IT + 1, Targs...>(vs...);
        }
    }

    inline T& get_(int k) {
        return items[k];
    }

    template <int Kmax = 1024>
    inline T get(int k) const {
        if constexpr (Kmax <= K) {
            return items[k];
        } else {
            if (k < K) {
                return items[k];
            }
            return 0;
        }
    }

    inline T operator[](int k) const {
        return get(k);
    }

    template <int Kmax = 1024>
    inline auto get_start(int k) const {
        return get<Kmax>(k);
    }

    template <int Kmax = 1024>
    inline auto get_end(int k) const {
        return get<Kmax>(k);
    }

    template <class T2>
    inline auto operator+(const Vec<K, T2>& v2) const {
        auto vcp = *this;
        vcp.template accum_sum<K - 1, T2>(v2);
        return vcp;
    }

    template <class T2>
    inline auto& operator+=(const Vec<K, T2>& v2) {
        accum_sum<K - 1, T2>(v2);
        return *this;
    }

    template <class T2>
    inline auto operator+(const T2& v2) const {
        auto vcp = *this;
        vcp.template accum_sum<K - 1, T2>(v2);
        return vcp;
    }

    template <class T2>
    inline auto& operator+=(const T2& v2) {
        accum_sum<K - 1, T2>(v2);
        return *this;
    }

    template <class T2>
    inline auto operator-(const Vec<K, T2>& v2) const {
        auto vcp = *this;
        vcp.template accum_sub<K - 1, T2>(v2);
        return vcp;
    }

    inline auto operator-() const {
        auto vcp = *this;
        vcp.template accum_neg<K - 1>();
        return vcp;
    }

    template <class T2>
    inline auto& operator-=(const Vec<K, T2>& v2) {
        accum_sub<K - 1, T2>(v2);
        return *this;
    }

    template <class T2>
    inline auto operator-(const T2& v2) const {
        auto vcp = *this;
        vcp.template accum_sub<K - 1, T2>(v2);
        return vcp;
    }

    template <class T2>
    inline auto& operator-=(const T2& v2) {
        accum_sub<K - 1, T2>(v2);
        return *this;
    }

    template <class T2>
    inline auto operator*(const Vec<K, T2>& v2) const {
        auto vcp = *this;
        vcp.template accum_mul<K - 1, T2>(v2);
        return vcp;
    }

    template <class T2>
    inline auto& operator*=(const Vec<K, T2>& v2) {
        accum_mul<K - 1, T2>(v2);
        return *this;
    }

    template <class T2>
    inline auto operator*(const T2& v2) const {
        auto vcp = *this;
        vcp.template accum_mul<K - 1, T2>(v2);
        return vcp;
    }

    template <class T2>
    inline auto& operator*=(const T2& v2) {
        accum_mul<K - 1, T2>(v2);
        return *this;
    }

    template <class T2>
    inline T dp(const Vec<K, T2>& v2) const {
        return calc_dp<K - 1, T2>(v2);
    }

    inline T qlen() const {
        return calc_qlen<K - 1>();
    }

    inline T abssum() const {
        return calc_abssum<K - 1>();
    }

    inline T qdist(const Vec<K, T>& v2) const {
        return calc_qdist<K - 1>(v2);
    }

    inline T sum() const {
        return calc_sum<K - 1>();
    }

    template <class Tv>
    inline void add_to_vec(Tv *V, const Vec<K, int>& pos_mapping) const {
        add_to_vec_it<K - 1, Tv>(V, pos_mapping);
    }

    template <int K2>
    inline void add_vec_mapped(
        const Vec<K2, T>& V, const Vec<K2, int>& pos_mapping
    ) {
        add_vec_mapped_it<K - 1, K2>(V, pos_mapping);
    }

    template <class Tstream>
    Tstream& put_to(Tstream& os) const {
        os << "(";
        put_items_to<Tstream, 0>(os);
        os << ")";
        return os;
    }
};

template <class T, class... Targs>
inline auto v(Targs... pos_values) {
    Vec<sizeof...(Targs), T> r;
    r.template set_pos<0>(pos_values...);
    return r;
}

template <class T>
inline auto cross2d(const Vec<2, T>& v1, const Vec<2, T>& v2) {
    return v1.get(0) * v2.get(1) - v1.get(1) * v2.get(0);
}

template <class T>
inline bool turns_left(const Vec<2, T>& v1, const Vec<2, T>& v2) {
    return v1.get(0) * v2.get(1) > v1.get(1) * v2.get(0);
}

template <class T>
inline bool turns_right(const Vec<2, T>& v1, const Vec<2, T>& v2) {
    return v1.get(0) * v2.get(1) < v1.get(1) * v2.get(0);
}

template <class T>
inline bool turns_direction(
    const Vec<2, T>& v1, const Vec<2, T>& v2, const bool direction = false) {
    return direction ? turns_right(v1, v2) : turns_left(v1, v2);
}

template <class Tstream, int K, class T>
Tstream& operator<<(Tstream& os, const Vec<K, T>& v) {
    return v.template put_to<Tstream>(os);
}
