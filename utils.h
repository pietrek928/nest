#ifndef __UTILS_H_
#define __UTILS_H_

template <class Tfirst, class Tsecond, class... Targs>
inline void rotl(Tfirst& vfirst, Tsecond& vsecond, Targs&... vs) {
    vfirst = vsecond;
    rotl(vsecond, vs...);
}

template <class Tfirst, class Tlast>
inline void rotl(Tfirst& vfirst, const Tlast& vlast) {
    vfirst = vlast;
}

// template <class... Targs, class T2, class T1>
// inline void rotr(Targs&... vs, T2& v2, T1& v1) {
//     v1 = v2;
//     rotr(vs..., v2);
// }

// template <class Tfirst, class T1>
// inline void rotr(const Tfirst& vfirst, T1& v1) {
//     v1 = vfirst;
// }

#endif /* __UTILS_H_ */
