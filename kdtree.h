#ifndef __KD_TREE_H_
#define __KD_TREE_H_

#include <algorithm>
#include <cmath>
#include <vector>

#include "bbox.h"

#define CONST_SWITCH_5(expr, var_name, ...)    \
    switch (expr) {                            \
        case 0: {                              \
            constexpr static int var_name = 0; \
            __VA_ARGS__;                       \
        } break;                               \
        case 1: {                              \
            constexpr static int var_name = 1; \
            __VA_ARGS__;                       \
        } break;                               \
        case 2: {                              \
            constexpr static int var_name = 2; \
            __VA_ARGS__;                       \
        } break;                               \
        case 3: {                              \
            constexpr static int var_name = 3; \
            __VA_ARGS__;                       \
        } break;                               \
        case 4: {                              \
            constexpr static int var_name = 4; \
            __VA_ARGS__;                       \
        } break;                               \
        case 5: {                              \
            constexpr static int var_name = 5; \
            __VA_ARGS__;                       \
        } break;                               \
    }

template <int ndims, class T>
class KDTree {
    using Tpos = typename T::Tnum;

    class Lnode {
       public:
        T obj;
        int next = -1;

        Lnode(T& obj) : obj(obj) {}
    };

    class Tnode {
       public:
        Tpos v = 0.;
        int left = -1, right = -1;
        int m = -1;
        int dim = 0;
    };

    struct split_score_t {
        double score;
        Tpos pos;
        int dim;
    };

    std::vector<Tnode> tree;
    std::vector<Lnode> list;

    template <int K>
    split_score_t score_split(int* items, int n, Tpos* buf /*size: 2n*/) {
        Tpos* bufb = buf;
        Tpos* bufe = buf + n;

        for (int i = 0; i < n; i++) {
            auto list_node_n = items[i];
            bufb[i] = list[list_node_n].obj.get_pos_start(K);
            bufe[i] = list[list_node_n].obj.get_pos_end(K);
        }

        auto bufb_end = bufb + n;
        auto bufe_end = bufe + n;
        std::sort(bufb, bufb_end);
        std::sort(bufe, bufe_end);

        int insides = 0;
        int before = 0;

        double best_score = 1e9;
        Tpos best_pos = (bufb[0] + bufe[n - 1]) * .5;

        do {
            if (*bufb < *bufe) {
                bufb++;
                insides++;
                before++;
            } else {
                double max_d = std::max(before, n - before);
                double new_score =
                    std::log2(max_d + 2.) + std::log2(insides + 2.) * 0.2;
                if (new_score < best_score) {
                    best_score = new_score;
                    best_pos = (*bufb + *bufe) * .5;
                }

                bufe++;
                insides--;
            }
        } while (bufb < bufb_end && bufe < bufe_end);

        return {
            .score = best_score,
            .pos = best_pos,
            .dim = K,
        };
    }

    template <int N>
    inline auto __select_split(int* items, int n, Tpos* buf) {
        auto k_split = score_split<N - 1>(items, n, buf);
        if constexpr (N > 1) {
            auto k_1_split = __select_split<N - 1>(items, n, buf);
            if (k_1_split.score < k_split.score) {
                return k_1_split;
            }
        }
        return k_split;
    }

    template <int K>
    int split_and_build(
        int* items, int* items_buf, int n, Tpos* buf /*size: 2n*/, Tpos pos) {
        if constexpr (K >= ndims) {
            // !!!!!!!!!!!!! should never come here
            return -1;
        }

        int tree_node_n = tree.size();
        auto tree_node = &tree.emplace_back();

        auto items_1 = items_buf;
        auto items_buf_e = items_buf + n;
        auto items_2 = items_buf_e;
        auto m = &tree_node->m;

        tree_node->v = pos;
        tree_node->dim = K;
        for (int i = 0; i < n; i++) {
            auto list_node_n = items[i];
            auto& list_node = list[list_node_n];
            if (list_node.obj.get_pos_end(K) < pos) {
                *(items_1++) = list_node_n;
            } else if (list_node.obj.get_pos_start(K) > pos) {
                *(--items_2) = list_node_n;
            } else {
                *m = list_node_n;
                m = &list_node.next;
            }
        }
        *m = -1;

        tree[tree_node_n].left =
            build(items_buf, items, items_1 - items_buf, buf);
        tree[tree_node_n].right =
            build(items_2, items, items_buf_e - items_2, buf);

        return tree_node_n;
    }

    int build(int* items, int* items_buf, int n, Tpos* buf /*size: 2n*/) {
        if (!n) {
            return -1;
        }

        if (n <= 2) {
            int tree_node_n = tree.size();
            auto tree_node = &tree.emplace_back();
            if (n == 2) {
                list[items[0]].next = items[1];
            }
            tree_node->m = items[0];
            return tree_node_n;
        }

        auto best_split = __select_split<ndims>(items, n, buf);
        CONST_SWITCH_5(
            best_split.dim, K,
            return split_and_build<K>(items, items_buf, n, buf, best_split.pos))

        // !!!!!!!!!!!!! should never come here
        return -1;
    }

   public:
    template <class Tpos_lookup>
    void match_point(std::vector<int>& out, const Tpos_lookup& pos) {
        // asm("!!!!!!!!!!!!!!!!");
        if (!tree.size()) {
            return;
        }

        int node_n = 0;
        do {
            auto& tree_node = tree[node_n];
            auto lnode = tree_node.m;
            while (lnode >= 0) {
                auto& list_node = list[lnode];
                if (object_bbox_intersects<ndims>(list_node.obj, pos)) {
                    out.push_back(lnode);
                }
                lnode = list_node.next;
            }
            if constexpr (ndims == 1) {
                auto dim_pos = pos.get_pos(0);
                node_n =
                    dim_pos < tree_node.v ? tree_node.left : tree_node.right;
            } else if constexpr (ndims == 2) {
                if (tree_node.dim) {
                    auto dim_pos = pos.get_pos(1);
                    node_n = dim_pos < tree_node.v ? tree_node.left
                                                   : tree_node.right;
                } else {
                    auto dim_pos = pos.get_pos(0);
                    node_n = dim_pos < tree_node.v ? tree_node.left
                                                   : tree_node.right;
                }
            } else {
                auto dim_pos = pos.template get_pos<ndims>(tree_node.dim);
                node_n =
                    dim_pos < tree_node.v ? tree_node.left : tree_node.right;
            }
        } while (node_n >= 0);
    }

    int max_calc_path(int o = 0) {
        if (o == -1) {
            return 0;
        }
        int list_cnt = 0;
        auto m = tree[o].m;
        while (m >= 0) {
            list_cnt++;
            m = list[m].next;
        }
        return std::max(
                   max_calc_path(tree[o].left), max_calc_path(tree[o].right)) +
               list_cnt + 1;
    }

    KDTree(std::vector<T>& objs) : list(objs.begin(), objs.end()) {
        if (!list.size()) {
            return;
        }
        int n = list.size();
        std::vector<int> items(n);
        for (int i = 0; i < n; i++) {
            items[i] = i;
        }
        std::vector<int> items_buf(items.begin(), items.end());
        std::vector<Tpos> sort_buf(2 * n);
        build(&*items.begin(), &*items_buf.begin(), n, &*sort_buf.begin());
    }
};

#endif /* __KD_TREE_H_ */
