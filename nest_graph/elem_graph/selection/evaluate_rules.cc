#include "graph/graph.h"

#include <vector>

#include "graph/graph_index.h"
#include "internal/internal.h"
#include "scoring/scoring.h"
#include "selection/selection.h"

std::vector<Tscore> score_rules(
    const std::vector<ElemGraph> &graphs,
    const std::vector<PlacementRuleSet> &rule_sets,
    const ScoreRulesOptions &options
) {
    std::vector<Tscore> rule_scores(rule_sets.size(), 0.0f);
    if (graphs.empty() || rule_sets.empty()) {
        return rule_scores;
    }

    std::vector<Tscore> elem_scores;
    std::vector<bool> marked;
    std::vector<Tvertex> selected;
    std::vector<Tvertex> order_buf;

    const std::size_t graph_begin =
        options.latest_graph_only ? graphs.size() - 1 : 0;

    for (std::size_t g_idx = graph_begin; g_idx < graphs.size(); ++g_idx) {
        const auto &graph = graphs[g_idx];
        const auto elems_by_group = get_elems_by_group(graph);

        for (int i = 0; i < static_cast<int>(rule_sets.size()); ++i) {
            compute_scores(
                graph, elems_by_group, elem_scores, rule_sets[i],
                options.select.aggregation);
            select_elems(
                graph, elems_by_group, rule_sets[i], elem_scores,
                marked, selected, order_buf, options.select);

            const float score_sum = sum_selected_scores_vec(elem_scores, selected);
            float fitness = score_sum;
            if (!options.selection_score_only) {
                fitness += options.count_weight * static_cast<float>(selected.size());
                if (options.mean_score_weight > 0.0f && !selected.empty()) {
                    fitness += options.mean_score_weight
                        * (score_sum / static_cast<float>(selected.size()));
                }
            }
            fitness -= options.rule_complexity_penalty
                * static_cast<float>(rule_sets[i].size());
            rule_scores[i] += fitness;
        }
    }

    return rule_scores;
}
