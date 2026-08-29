#pragma once

#include <cstdint>
#include <utility>
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

/** Typed walk handle over DecisionGraph partitions (Q378). */
enum class PathKind : uint8_t { Macro = 0, MotifJoin = 1, Attach = 2, Pose = 3 };

struct PathNode {
    PathKind kind = PathKind::Macro;
    int32_t id = -1;       // arena node | motifs_/attach_ index | pose vertex
    int32_t motif_id = -1;
    int32_t a = -1;
    int32_t b = -1;
};

struct RealizeStats {
    int attach = 0;
    int motif = 0;
    int member_hits = 0;
    int kind_count[4] = {0, 0, 0, 0};
};

inline PathNode node_macro(int32_t node_id) {
    PathNode n;
    n.kind = PathKind::Macro;
    n.id = node_id;
    return n;
}

inline PathNode node_motif(int32_t idx, int32_t motif_id, Tvertex a, Tvertex b) {
    PathNode n;
    n.kind = PathKind::MotifJoin;
    n.id = idx;
    n.motif_id = motif_id;
    n.a = a;
    n.b = b;
    return n;
}

inline PathNode node_attach(int32_t idx, Tvertex a, Tvertex b) {
    PathNode n;
    n.kind = PathKind::Attach;
    n.id = idx;
    n.a = a;
    n.b = b;
    return n;
}

inline PathNode node_pose(Tvertex v) {
    PathNode n;
    n.kind = PathKind::Pose;
    n.id = v;
    n.a = v;
    return n;
}

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

    PathNode path_node_motif(std::size_t i) const {
        if (i >= motifs_.size()) {
            return PathNode{};
        }
        const MotifJoin &e = motifs_[i];
        return node_motif(static_cast<int32_t>(i), e.motif_id, e.a, e.b);
    }

    PathNode path_node_attach(std::size_t i) const {
        if (i >= attach_.size()) {
            return PathNode{};
        }
        const AttachNode &e = attach_[i];
        return node_attach(static_cast<int32_t>(i), e.a, e.b);
    }

    /** Soft mutex for Macro↔*; Join/Attach use member-set Collision (Q380). */
    bool conflicts(PathNode u, PathNode v) const {
        if (u.kind == PathKind::Macro || v.kind == PathKind::Macro) {
            return false;
        }
        if (u.kind == PathKind::Pose && v.kind == PathKind::Pose) {
            return pose_has_collision(poses_, static_cast<Tvertex>(u.id), static_cast<Tvertex>(v.id));
        }
        Tvertex ua = u.a;
        Tvertex ub = u.b;
        Tvertex va = v.a;
        Tvertex vb = v.b;
        if (u.kind == PathKind::Pose) {
            ua = ub = static_cast<Tvertex>(u.id);
        }
        if (v.kind == PathKind::Pose) {
            va = vb = static_cast<Tvertex>(v.id);
        }
        if (ua < 0 || ub < 0 || va < 0 || vb < 0) {
            return false;
        }
        return member_sets_collide(ua, ub, va, vb);
    }

    /** One-shot derived Mutex count via conflicts(PathNode, PathNode) (Q380). */
    int mutex_n() const {
        int n = 0;
        for (std::size_t i = 0; i < attach_.size(); ++i) {
            for (std::size_t j = i + 1; j < attach_.size(); ++j) {
                if (conflicts(path_node_attach(i), path_node_attach(j))) {
                    ++n;
                }
            }
        }
        for (std::size_t i = 0; i < motifs_.size(); ++i) {
            for (std::size_t j = i + 1; j < motifs_.size(); ++j) {
                if (conflicts(path_node_motif(i), path_node_motif(j))) {
                    ++n;
                }
            }
        }
        for (std::size_t i = 0; i < attach_.size(); ++i) {
            for (std::size_t j = 0; j < motifs_.size(); ++j) {
                if (conflicts(path_node_attach(i), path_node_motif(j))) {
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

    /** Sole writer of Attach/MotifJoin.realized flags (Q379/Q381). */
    RealizeStats realize(const std::vector<Tvertex> &selected) {
        RealizeStats st;
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
                st.attach += 1;
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
                st.motif += 1;
            }
        }
        return st;
    }

    bool realized(PathNode step) const {
        if (step.kind == PathKind::MotifJoin) {
            if (step.id < 0 || static_cast<std::size_t>(step.id) >= motifs_.size()) {
                return false;
            }
            return motifs_[static_cast<std::size_t>(step.id)].realized;
        }
        if (step.kind == PathKind::Attach) {
            if (step.id < 0 || static_cast<std::size_t>(step.id) >= attach_.size()) {
                return false;
            }
            return attach_[static_cast<std::size_t>(step.id)].realized;
        }
        if (step.kind == PathKind::Pose) {
            return step.id >= 0 && static_cast<std::size_t>(step.id) < poses_.size();
        }
        if (step.kind == PathKind::Macro) {
            if (step.id < 0 || step.id >= macros_.size()) {
                return false;
            }
            return !macros_.snapshot(step.id).motif_ids_used.empty();
        }
        return false;
    }

    /** motif_id → realized MotifJoin count (D1 SoT; Q381). */
    std::vector<std::pair<int32_t, int>> survive_counts() const {
        std::vector<std::pair<int32_t, int>> out;
        for (const MotifJoin &e : motifs_) {
            if (!e.realized || e.motif_id < 0) {
                continue;
            }
            bool found = false;
            for (auto &p : out) {
                if (p.first == e.motif_id) {
                    p.second += 1;
                    found = true;
                    break;
                }
            }
            if (!found) {
                out.push_back({e.motif_id, 1});
            }
        }
        return out;
    }

    std::vector<Tvertex> poses_of(PathNode step) const {
        std::vector<Tvertex> out;
        if (step.kind == PathKind::MotifJoin || step.kind == PathKind::Attach) {
            if (step.a >= 0) {
                out.push_back(step.a);
            }
            if (step.b >= 0 && step.b != step.a) {
                out.push_back(step.b);
            }
            return out;
        }
        if (step.kind == PathKind::Pose && step.id >= 0) {
            out.push_back(static_cast<Tvertex>(step.id));
        }
        return out;
    }

    /** Structural adjacency only — PW invent stays in Python (Q378). */
    std::vector<PathNode> neighbors(PathNode step) const {
        std::vector<PathNode> out;
        if (step.kind == PathKind::Macro) {
            if (step.id < 0 || step.id >= macros_.size()) {
                return out;
            }
            int32_t c = macros_.node(step.id).first_child_id;
            while (c >= 0) {
                out.push_back(node_macro(c));
                c = macros_.node(c).next_sibling_id;
            }
            for (std::size_t i = 0; i < motifs_.size(); ++i) {
                const MotifJoin &e = motifs_[i];
                if (e.a < 0 || e.b < 0) {
                    continue;
                }
                const bool tagged =
                    (static_cast<std::size_t>(e.a) < pose_kind_.size()
                     && pose_kind_[static_cast<std::size_t>(e.a)] != kPoseKindUntagged)
                    || (static_cast<std::size_t>(e.b) < pose_kind_.size()
                        && pose_kind_[static_cast<std::size_t>(e.b)] != kPoseKindUntagged);
                if (tagged) {
                    out.push_back(path_node_motif(i));
                }
            }
            return out;
        }
        if (step.kind == PathKind::MotifJoin) {
            if (step.a >= 0) {
                out.push_back(node_pose(step.a));
            }
            if (step.b >= 0 && step.b != step.a) {
                out.push_back(node_pose(step.b));
            }
            for (std::size_t i = 0; i < motifs_.size(); ++i) {
                if (static_cast<int32_t>(i) == step.id) {
                    continue;
                }
                const MotifJoin &e = motifs_[i];
                if (e.a == step.a || e.a == step.b || e.b == step.a || e.b == step.b) {
                    out.push_back(path_node_motif(i));
                }
            }
            for (std::size_t i = 0; i < attach_.size(); ++i) {
                const AttachNode &e = attach_[i];
                if (e.a == step.a || e.a == step.b || e.b == step.a || e.b == step.b) {
                    out.push_back(path_node_attach(i));
                }
            }
            return out;
        }
        if (step.kind == PathKind::Attach) {
            if (step.a >= 0) {
                out.push_back(node_pose(step.a));
            }
            if (step.b >= 0 && step.b != step.a) {
                out.push_back(node_pose(step.b));
            }
            return out;
        }
        if (step.kind == PathKind::Pose) {
            const Tvertex v = static_cast<Tvertex>(step.id);
            for (std::size_t i = 0; i < motifs_.size(); ++i) {
                const MotifJoin &e = motifs_[i];
                if (e.a == v || e.b == v) {
                    out.push_back(path_node_motif(i));
                }
            }
            for (std::size_t i = 0; i < attach_.size(); ++i) {
                const AttachNode &e = attach_[i];
                if (e.a == v || e.b == v) {
                    out.push_back(path_node_attach(i));
                }
            }
        }
        return out;
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
