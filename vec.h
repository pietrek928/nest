#ifndef __VEC_H_
#define __VEC_H_

template <int K, class T>
class Vec {
    T items[K];

    template <int IT, class Tv, class... Targs>
    inline void set_pos(Tv v, Targs... vs) {
        items[IT] = v;
        if constexpr (IT < K - 1) {
            set_pos<IT + 1, Targs...>(vs...);
        }
    }

    template <int IT, class T2>
    inline T calc_dp(const Vec<K, T2>& v2) const {
        T r = items[IT] * v2.items[IT];
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

    template <int IT, class T2>
    inline T calc_qdist(const Vec<K, T2>& v2) const {
        T diff = items[IT] - v2.items[IT];
        T r = diff * diff;
        if constexpr (IT > 0) {
            r += calc_qlen<IT - 1>();
        }
        return r;
    }

    template <int IT, class T2>
    inline void accum_sum(const Vec<K, T2>& v2) {
        items[IT] += v2.items[IT];
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

    template <int IT, class T2>
    inline void accum_sub(const Vec<K, T2>& v2) {
        items[IT] -= v2.items[IT];
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
        items[IT] *= v2.items[IT];
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

    template <class Tstream, int IT>
    inline void put_items_to(Tstream& os) const {
        os << items[IT];
        if constexpr (IT < K - 1) {
            os << ", ";
            put_items_to<Tstream, IT + 1>(os);
        }
    }

   public:
    Vec() : items{0} {}

    template <class... Targs>
    Vec(Targs... pos_values) {
        set_pos<0>(pos_values...);
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

    template <class T2>
    inline T qdist(const Vec<K, T2>& v2) const {
        return calc_qdist<K - 1, T2>(v2);
    }

    inline T sum() const {
        return calc_sum<K - 1>();
    }

    template <class Tstream>
    Tstream& put_to(Tstream& os) const {
        os << "(";
        put_items_to<Tstream, 0>(os);
        os << ")";
        return os;
    }
};

template <class Tstream, int K, class T>
Tstream& operator<<(Tstream& os, const Vec<K, T>& v) {
    return v.template put_to<Tstream>(os);
}

#endif /* __VEC_H_ */
