#ifndef HEURISTICS_SUBGOAL_COUNT_HEURISTIC_H
#define HEURISTICS_SUBGOAL_COUNT_HEURISTIC_H

#include "../abstract_task.h"
#include "../state_id.h"

#include <unordered_map>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(default: 4800 4512 4706 4100 4127 4702 4239 4996 4456 4458 4505)
#endif

namespace subgoal_count_heuristic {

class SubgoalHeuristic {
public:
	enum class SubgoalAggregationMethod {
		COUNT,
		SUM,
		MAX
	};

private:
	const SubgoalAggregationMethod subgoal_aggregation_method;
	const bool path_dependent_subgoals;
	std::unordered_map<StateID, std::vector<FactPair>> achieved_subgoals;
	int max_value;

	std::vector<std::pair<FactPair, int>> subgoals_and_costs;

	auto aggregate(int value, int cost) const -> int;
public:
	SubgoalHeuristic(SubgoalAggregationMethod subgoal_aggregation_method, bool path_dependent_subgoals);

	void initialize(std::vector<std::pair<FactPair, int>> &&subgoals_and_costs, const StateID &root);
	auto compute_result(const StateID &parent_state_id, const GlobalState &state) -> int;
};


}

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif
