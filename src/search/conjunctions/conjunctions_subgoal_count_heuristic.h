#ifndef CONJUNCTIONS_SUBGOAL_COUNT_HEURISTIC_H
#define CONJUNCTIONS_SUBGOAL_COUNT_HEURISTIC_H

#include "../heuristic.h"

#include "conjunctions.h"

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(default: 4800 4512 4706 4100 4127 4702 4239 4996 4456 4458 4505)
#endif

namespace conjunctions {

class ConjunctionsSubgoalHeuristic {
public:
	enum class SubgoalAggregationMethod {
		COUNT,
		SUM,
		MAX
	};

private:
	const SubgoalAggregationMethod subgoal_aggregation_method;
	const bool path_dependent_subgoals;
	std::unordered_map<StateID, std::vector<Conjunction *>> achieved_subgoals;
	int max_value;

	std::vector<Conjunction *> conjunctions;
	std::vector<int> cost;

	auto aggregate(int value, size_t index) -> int;
public:
	ConjunctionsSubgoalHeuristic(SubgoalAggregationMethod subgoal_aggregation_method, bool path_dependent_subgoals);

	void initialize(std::vector<Conjunction *> &&conjunctions, const StateID &root);
	auto compute_result(const StateID &parent_state_id, const GlobalState &state) -> int;
	auto get_preferred_ops(const std::vector<const GlobalOperator *> &applicable_ops) -> std::vector<const GlobalOperator *>;
};


}

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif
