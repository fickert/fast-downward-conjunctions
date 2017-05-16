#ifndef CONJUNCTIONS_ENFORCED_HILL_CLIMBING_SEARCH_H
#define CONJUNCTIONS_ENFORCED_HILL_CLIMBING_SEARCH_H

#include "online_learning_search_engine.h"

#include "../evaluation_context.h"
#include "../search_engine.h"
#include "../open_lists/open_list.h"

#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>
#include <random>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(default: 4800 4512 4706 4100 4127 4702 4239 4996 4456 4458 4505)
#endif

namespace options {
class Options;
}

namespace conjunctions {
// Enforced hill-climbing with deferred evaluation and online learning of explicit conjunctions in local minima.
class EnforcedHillClimbingSearch : public OnlineLearningSearchEngine {
public:
	explicit EnforcedHillClimbingSearch(const options::Options &opts);
	~EnforcedHillClimbingSearch() override;

	void print_statistics() const override;

	enum class PreferredUsage {
		PRUNE_BY_PREFERRED,
		RANK_PREFERRED_FIRST
	};

protected:
	void initialize() override;
	SearchStatus step() override;

private:
	auto get_successors(EvaluationContext &eval_context) -> std::vector<const GlobalOperator *>;
	void expand(EvaluationContext &eval_context, SearchSpace &current_search_space);
	void reach_state(const GlobalState &parent, const GlobalOperator &op, const GlobalState &state);
	auto ehc(SearchSpace &current_search_space) -> SearchStatus;

	std::unique_ptr<EdgeOpenList> open_list;

	ConjunctionsHeuristic *heuristic;
	std::vector<Heuristic *> preferred_operator_heuristics;
	std::set<Heuristic *> heuristics;
	bool use_preferred;
	PreferredUsage preferred_usage;

	EvaluationContext current_eval_context;

	// Statistics
	std::map<int, std::pair<int, int>> d_counts;
	int num_ehc_phases;
	int last_num_expanded;

	std::shared_ptr<ConjunctionGenerationStrategy> conjunctions_strategy;

	struct EHCCStatistics {
		EHCCStatistics() :
			num_dead_ends_during_learning(0),
			num_no_better_state_after_learning(0),
			num_saved_evaluations(0),
			num_search_space_exhaustion(0),
			num_dead_ends(0),
			total_dead_end_backjump_length(0) {}

		int num_dead_ends_during_learning;
		int num_no_better_state_after_learning;
		int num_saved_evaluations;
		int num_search_space_exhaustion;
		int num_dead_ends;
		int total_dead_end_backjump_length;

	} ehcc_statistics;

	void print_ehcc_statistics() const;
	void print_intermediate_statistics(const ConjunctionsHeuristic &) const override;

	auto evaluate_if_neccessary(EvaluationContext &eval_context, const GlobalState &parent_state, const GlobalOperator &last_op) -> int;
	auto evaluate_if_neccessary(EvaluationContext &eval_context) -> int;

	auto escape_local_minimum() -> SearchStatus;

	auto handle_safe_dead_end() -> SearchStatus;
	auto handle_search_space_exhaustion() -> SearchStatus;

	auto escape_dead_end(const SearchNode &node) -> SearchStatus;
	auto escape_potential_dead_end() -> SearchStatus;

	auto restart() -> SearchStatus;
	auto restart_in_parent() -> SearchStatus;

	const int bfs_bound;
	int local_minimum_d;

	// bool indicating whether a new best state was found after learning
	bool is_learning_stagnation;

	std::unordered_map<StateID, std::pair<int, std::vector<const GlobalOperator *>>> heuristic_cache;

	void update_eval_context(EvaluationContext &eval_context, const decltype(heuristic_cache)::mapped_type &cache_entry);

	int bfs_lowest_h_value;
	bool solved;

	bool k_cutoff;

	// settings
	const bool no_learning;
	const bool restart_in_dead_ends;
	const bool always_reevaluate;
	const bool enable_heuristic_cache;
	const bool randomize_successors;

	enum class LearningStagnation {
		CONTINUE,
		RESTART,
		BACKJUMP
	} const learning_stagnation;

	enum class SearchSpaceExhaustion {
		CONTINUE,
		RESTART,
		BACKJUMP
	} const search_space_exhaustion;

	std::unordered_set<StateID> excluded_states;

	// random number generator for successor randomization
	std::mt19937 urng;
};
}

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif
