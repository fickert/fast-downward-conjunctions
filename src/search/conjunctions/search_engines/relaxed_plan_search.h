#ifndef CONJUNCTIONS_RELAXED_PLAN_SEARCH_H
#define CONJUNCTIONS_RELAXED_PLAN_SEARCH_H

#include "online_learning_search_engine.h"

#include "../evaluation_context.h"
#include "../search_engine.h"

#include <memory>
#include <vector>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(default: 4800 4512 4706 4100 4127 4702 4239 4996 4456 4458 4505)
#endif

namespace options {
class Options;
}

namespace conjunctions {

class RelaxedPlanSearch : public OnlineLearningSearchEngine {
public:
	explicit RelaxedPlanSearch(const options::Options &opts);
	~RelaxedPlanSearch() override;

	void print_statistics() const override;

protected:
	void initialize() override;
	SearchStatus step() override;

private:
	void update_global_search_space();

	auto handle_dead_end(SearchNode &global_node, SearchNode &local_node) -> SearchStatus;
	auto handle_stagnation() -> SearchStatus;
	auto handle_duplicate() -> SearchStatus;
	auto advance_to_next_state() -> SearchStatus;

	std::unique_ptr<SearchSpace> current_search_space;
	ConjunctionsHeuristic *heuristic;
	GlobalState last_state;
	GlobalState current_initial_state;
	EvaluationContext current_eval_context;
	std::shared_ptr<ConjunctionGenerationStrategy> conjunctions_strategy;

	struct RPSStatistics {
		RPSStatistics() :
			num_episodes(0),
			num_duplicates(0),
			num_stagnation(0),
			num_advanced(0),
			max_rp_sequence_length(0),
			sum_rp_sequence_length(0) {}

		int num_episodes, num_duplicates, num_stagnation, num_advanced, max_rp_sequence_length;
		long sum_rp_sequence_length;

		void advance(int sequence_length) {
			++num_advanced;
			max_rp_sequence_length = std::max(max_rp_sequence_length, sequence_length);
			sum_rp_sequence_length += sequence_length;
		}

		auto get_avg_rp_sequence_length() const {
			return num_advanced == 0 ? 1. : sum_rp_sequence_length / static_cast<double>(num_advanced);
		}

	} rps_statistics;

	auto get_sequence_length() const -> decltype(heuristic->get_last_bsg().nodes.size());

	void print_rps_statistics() const;
	void print_intermediate_statistics(const ConjunctionsHeuristic &) const override;

	void refine_once(bool refine_on_previous);

	auto restart() -> SearchStatus;
	auto continue_in_parent(SearchNode &local_node) -> SearchStatus;

	enum class RPSequence {
		ALL,
		ALL_INTENDED,
		FIRST_DELETER
	} const rp_sequence;

	int num_states_since_last_improvement;
	int last_improved_heuristic_value;
	const int improvement_tolerance;

	const bool restart_in_dead_ends;
	const bool restart_in_stagnation;
	const bool refine_on_previous;
};
}

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif
