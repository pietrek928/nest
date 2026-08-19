#pragma once

#include <cstdint>
#include <vector>

#include "decision_arena.h"
#include "pose_graph.h"

/** Untagged MemberOf (history / elite / carry unless stamped). Q158. */
inline constexpr uint8_t kPoseKindUntagged = 255;

struct AttachNode {
    Tvertex a = -1;
    Tvertex b = -1;
    bool realized = false;
};

struct MotifJoin {
    int32_t motif_id = -1;
    Tvertex a = -1;
    Tvertex b = -1;
    bool realized = false;
};

struct MaterializeStats {
    int materialized_attach = 0;
    int materialized_motif = 0;
    int member_hits = 0;
    int kind_count[4] = {0, 0, 0, 0};
};

/** Hybrid owner: Arena Sequence + Pose MWIS + epoch Attach/MotifJoin/MemberOf. */
class DecisionGraph {
public:
    DecisionArena &macros() { return macros_; }
    const DecisionArena &macros() const { return macros_; }
    PoseGraph &poses() { return poses_; }
    const PoseGraph &poses() const { return poses_; }

    const std::vector<uint8_t> &pose_kind() const { return pose_kind_; }
    const std::vector<AttachNode> &attach() const { return attach_; }
    const std::vector<MotifJoin> &motifs() const { return motifs_; }

    int attach_n() const { return static_cast<int>(attach_.size()); }
    int motif_n() const { return static_cast<int>(motifs_.size()); }

    /** Copy-in. Wipe epoch overlays; keep Sequence/AMAF/snapshots (Q150/Q161). */
    void replace_poses(const PoseGraph &g) {
        poses_ = g;
        pose_kind_.assign(g.size(), kPoseKindUntagged);
        attach_.clear();
        motifs_.clear();
    }

    void set_pose_kind(Tvertex i, uint8_t kind) {
        if (i < 0 || static_cast<std::size_t>(i) >= pose_kind_.size()) {
            return;
        }
        pose_kind_[static_cast<std::size_t>(i)] = kind;
    }

    void set_pose_kinds(const std::vector<uint8_t> &kinds) {
        const std::size_t n = poses_.size();
        pose_kind_.assign(n, kPoseKindUntagged);
        const std::size_t m = kinds.size() < n ? kinds.size() : n;
        for (std::size_t i = 0; i < m; ++i) {
            pose_kind_[i] = kinds[i];
        }
    }

    /** Same skip as PoseGraph.add_attract (Collision / identity / OOB). */
    void add_attach(Tvertex a, Tvertex b) {
        if (pose_pair_skip(poses_, a, b)) {
            return;
        }
        Tvertex lo = a < b ? a : b;
        Tvertex hi = a < b ? b : a;
        for (const AttachNode &e : attach_) {
            if (e.a == lo && e.b == hi) {
                return;
            }
        }
        attach_.push_back(AttachNode{lo, hi});
    }

    void add_motif_join(int32_t motif_id, Tvertex a, Tvertex b) {
        if (pose_pair_skip(poses_, a, b)) {
            return;
        }
        Tvertex lo = a < b ? a : b;
        Tvertex hi = a < b ? b : a;
        for (const MotifJoin &e : motifs_) {
            if (e.motif_id == motif_id && e.a == lo && e.b == hi) {
                return;
            }
        }
        motifs_.push_back(MotifJoin{motif_id, lo, hi});
    }

    bool attach_conflicts(std::size_t i, std::size_t j) const {
        if (i >= attach_.size() || j >= attach_.size() || i == j) {
            return false;
        }
        return member_sets_collide(attach_[i].a, attach_[i].b, attach_[j].a, attach_[j].b);
    }

    bool motif_conflicts(std::size_t i, std::size_t j) const {
        if (i >= motifs_.size() || j >= motifs_.size() || i == j) {
            return false;
        }
        return member_sets_collide(motifs_[i].a, motifs_[i].b, motifs_[j].a, motifs_[j].b);
    }

    bool attach_motif_conflicts(std::size_t ai, std::size_t mi) const {
        if (ai >= attach_.size() || mi >= motifs_.size()) {
            return false;
        }
        return member_sets_collide(attach_[ai].a, attach_[ai].b, motifs_[mi].a, motifs_[mi].b);
    }

    /** One-shot derived Mutex count (no stored CSR). */
    int mutex_n() const {
        int n = 0;
        for (std::size_t i = 0; i < attach_.size(); ++i) {
            for (std::size_t j = i + 1; j < attach_.size(); ++j) {
                if (attach_conflicts(i, j)) {
                    ++n;
                }
            }
        }
        for (std::size_t i = 0; i < motifs_.size(); ++i) {
            for (std::size_t j = i + 1; j < motifs_.size(); ++j) {
                if (motif_conflicts(i, j)) {
                    ++n;
                }
            }
        }
        for (std::size_t i = 0; i < attach_.size(); ++i) {
            for (std::size_t j = 0; j < motifs_.size(); ++j) {
                if (attach_motif_conflicts(i, j)) {
                    ++n;
                }
            }
        }
        return n;
    }

    int kind_tagged_n() const {
        int n = 0;
        for (uint8_t k : pose_kind_) {
            if (k != kPoseKindUntagged) {
                ++n;
            }
        }
        return n;
    }

    MaterializeStats materialize_selection(const std::vector<Tvertex> &selected) {
        MaterializeStats st;
        const std::size_t n = poses_.size();
        std::vector<uint8_t> on(n, 0);
        for (Tvertex v : selected) {
            if (v < 0 || static_cast<std::size_t>(v) >= n) {
                continue;
            }
            on[static_cast<std::size_t>(v)] = 1;
            if (static_cast<std::size_t>(v) < pose_kind_.size()) {
                const uint8_t k = pose_kind_[static_cast<std::size_t>(v)];
                if (k != kPoseKindUntagged) {
                    st.member_hits += 1;
                    if (k < 4) {
                        st.kind_count[k] += 1;
                    }
                }
            }
        }
        for (AttachNode &e : attach_) {
            e.realized = false;
            if (e.a >= 0 && e.b >= 0
                && static_cast<std::size_t>(e.a) < n
                && static_cast<std::size_t>(e.b) < n
                && on[static_cast<std::size_t>(e.a)]
                && on[static_cast<std::size_t>(e.b)]) {
                e.realized = true;
                st.materialized_attach += 1;
            }
        }
        for (MotifJoin &e : motifs_) {
            e.realized = false;
            if (e.a >= 0 && e.b >= 0
                && static_cast<std::size_t>(e.a) < n
                && static_cast<std::size_t>(e.b) < n
                && on[static_cast<std::size_t>(e.a)]
                && on[static_cast<std::size_t>(e.b)]) {
                e.realized = true;
                st.materialized_motif += 1;
            }
        }
        return st;
    }

private:
    bool member_sets_collide(Tvertex a, Tvertex b, Tvertex c, Tvertex d) const {
        return pose_has_collision(poses_, a, c) || pose_has_collision(poses_, a, d)
            || pose_has_collision(poses_, b, c) || pose_has_collision(poses_, b, d);
    }

    DecisionArena macros_;
    PoseGraph poses_;
    std::vector<uint8_t> pose_kind_;
    std::vector<AttachNode> attach_;
    std::vector<MotifJoin> motifs_;
};

inline std::vector<Tvertex> nest_by_scores(
    const DecisionGraph &dg,
    const std::vector<Tscore> &scores,
    const SelectOptions &select = SelectOptions{}
) {
    return nest_by_scores(dg.poses(), scores, select);
}
