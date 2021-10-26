#include "subgoal_count_heuristic.h"

#include "../options/options.h"
#include "../options/option_parser.h"
#include <numeric>
#include "../global_state.h"

namespace subgoal_count_heuristic {

SubgoalHeuristic::SubgoalHeuristic(SubgoalAggregationMethod subgoal_aggregation_method, bool path_dependent_subgoals)
	: subgoal_aggregation_method(subgoal_aggregation_method),
	  path_dependent_subgoals(path_dependent_subgoals),
	  max_value(-1) {}

void SubgoalHeuristic::initialize(std::vector<std::pair<FactPair, int>> &&subgoals_and_costs, const StateID &root) {
	if (path_dependent_subgoals) {
		achieved_subgoals.clear();
		achieved_subgoals[root] = {};
	}
	this->subgoals_and_costs = std::move(subgoals_and_costs);
#ifndef NDEBUG
	auto sorted_facts = std::vector<FactPair>();
	sorted_facts.reserve(this->subgoals_and_costs.size());
	std::transform(std::begin(this->subgoals_and_costs), std::end(this->subgoals_and_costs), std::back_inserter(sorted_facts), [](const auto &fact_and_cost) {
		return fact_and_cost.first;
	});
	std::sort(std::begin(sorted_facts), std::end(sorted_facts));
	assert(std::unique(std::begin(sorted_facts), std::end(sorted_facts)) == std::end(sorted_facts));
	assert(std::all_of(std::begin(this->subgoals_and_costs), std::end(this->subgoals_and_costs), [](const auto &fact_and_cost) { return fact_and_cost.second >= 0; }));
#endif
	max_value = std::accumulate(std::begin(this->subgoals_and_costs), std::end(this->subgoals_and_costs), 0, [this](auto value, const auto &subgoal_and_cost) {
		return aggregate(value, subgoal_and_cost.second);
	});
}

auto SubgoalHeuristic::aggregate(int value, int cost) const -> int {
	switch (subgoal_aggregation_method) {
	case SubgoalAggregationMethod::COUNT:
		return value + 1;
	case SubgoalAggregationMethod::MAX:
		return std::max(value, cost);
	case SubgoalAggregationMethod::SUM:
		return value + cost;
	default:
		std::cerr << "unknown subgoal aggregation method: " << static_cast<int>(subgoal_aggregation_method) << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
	}
}

auto SubgoalHeuristic::compute_result(const StateID &parent_state_id, const GlobalState &state) -> int {
	if (!path_dependent_subgoals) {
		auto value = 0;
		for (const auto &[subgoal, cost] : subgoals_and_costs)
			if (state[subgoal.var] == subgoal.value)
				value = aggregate(value, cost);
		assert(max_value >= value);
		return max_value - value;
	}

	assert(achieved_subgoals.find(parent_state_id) != std::end(achieved_subgoals));
	assert(achieved_subgoals.find(state.get_id()) == std::end(achieved_subgoals));
	const auto &parent_achieved_subgoals = achieved_subgoals.find(parent_state_id)->second;
	auto &reached_facts = achieved_subgoals.emplace(state.get_id(), std::vector<FactPair>()).first->second;
	assert(reached_facts.empty());

	auto value = 0;
	auto parent_achieved_subgoals_it = std::begin(parent_achieved_subgoals);

	const auto reached_fact = [this, &value, &reached_facts](const auto &fact, auto cost) {
		reached_facts.push_back(fact);
		value = aggregate(value, cost);
	};

	for (const auto &[subgoal, cost] : subgoals_and_costs) {
		if (parent_achieved_subgoals_it != std::end(parent_achieved_subgoals) && *parent_achieved_subgoals_it == subgoal) {
			reached_fact(subgoal, cost);
			++parent_achieved_subgoals_it;
		} else if (state[subgoal.var] == subgoal.value) {
			reached_fact(subgoal, cost);
		}
	}
	assert(parent_achieved_subgoals_it == std::end(parent_achieved_subgoals));
	assert(reached_facts.size() >= parent_achieved_subgoals.size());
	return max_value - value;
}

}
