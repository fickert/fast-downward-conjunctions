#include "conjunctions_subgoal_count_heuristic.h"

#include "utils.h"
#include "../options/options.h"
#include "../options/option_parser.h"

namespace conjunctions {

ConjunctionsSubgoalHeuristic::ConjunctionsSubgoalHeuristic(SubgoalAggregationMethod subgoal_aggregation_method, bool path_dependent_subgoals)
	: subgoal_aggregation_method(subgoal_aggregation_method),
	  path_dependent_subgoals(path_dependent_subgoals),
	  max_value(-1) {}

void ConjunctionsSubgoalHeuristic::initialize(std::vector<Conjunction *> &&conjunctions, const StateID &root) {
	if (path_dependent_subgoals) {
		achieved_subgoals.clear();
		achieved_subgoals[root] = {};
	}
	this->conjunctions = std::move(conjunctions);
	assert(std::is_sorted(std::begin(this->conjunctions), std::end(this->conjunctions)));
	assert(std::unique(std::begin(this->conjunctions), std::end(this->conjunctions)) == std::end(this->conjunctions));
	if (subgoal_aggregation_method != SubgoalAggregationMethod::COUNT) {
		cost.clear();
		for (auto *conjunction : this->conjunctions) {
			assert(conjunction->cost >= 0);
			cost.push_back(conjunction->cost);
		}
	}
	max_value = 0;
	for (auto i = 0u; i < this->conjunctions.size(); ++i)
		max_value = aggregate(max_value, i);
}

auto ConjunctionsSubgoalHeuristic::aggregate(int value, size_t index) -> int {
	switch (subgoal_aggregation_method) {
	case SubgoalAggregationMethod::COUNT:
		return value + 1;
	case SubgoalAggregationMethod::MAX:
		return std::max(value, cost[index]);
	case SubgoalAggregationMethod::SUM:
		return value + cost[index];
	default:
		std::cerr << "unknown subgoal aggregation method: " << static_cast<int>(subgoal_aggregation_method) << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
	}
}

auto ConjunctionsSubgoalHeuristic::compute_result(const StateID &parent_state_id, const GlobalState &state) -> int {
	if (!path_dependent_subgoals) {
		auto value = 0;
		for (auto i = 0u; i < conjunctions.size(); ++i)
			if (is_subset(conjunctions[i]->facts, state))
				value = aggregate(value, i);
		assert(max_value >= value);
		return max_value - value;
	}

	assert(achieved_subgoals.find(parent_state_id) != std::end(achieved_subgoals));
	assert(achieved_subgoals.find(state.get_id()) == std::end(achieved_subgoals));
	const auto &parent_reached_conjunctions = achieved_subgoals.find(parent_state_id)->second;
	auto &reached_conjunctions = achieved_subgoals.emplace(state.get_id(), std::vector<Conjunction *>()).first->second;
	assert(reached_conjunctions.empty());

	auto value = 0;
	auto parent_reached_conjunctions_it = std::begin(parent_reached_conjunctions);

	const auto reached_conjunction = [this, &value, &reached_conjunctions](auto index) {
		reached_conjunctions.push_back(conjunctions[index]);
		value = aggregate(value, index);
	};

	for (auto i = 0u; i < conjunctions.size(); ++i) {
		if (parent_reached_conjunctions_it != std::end(parent_reached_conjunctions) && *parent_reached_conjunctions_it == conjunctions[i]) {
			reached_conjunction(i);
			++parent_reached_conjunctions_it;
		} else if (is_subset(conjunctions[i]->facts, state)) {
			reached_conjunction(i);
		}
	}
	assert(parent_reached_conjunctions_it == std::end(parent_reached_conjunctions));
	assert(reached_conjunctions.size() >= parent_reached_conjunctions.size());
	return max_value - value;
}

auto ConjunctionsSubgoalHeuristic::get_preferred_ops(const std::vector<const GlobalOperator *> &) -> std::vector<const GlobalOperator *> {
	std::cerr << "preferred operators not implemented for subgoal heuristic" << std::endl;
	utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
}

}
