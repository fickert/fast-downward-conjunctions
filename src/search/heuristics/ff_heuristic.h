#ifndef HEURISTICS_FF_HEURISTIC_H
#define HEURISTICS_FF_HEURISTIC_H

#include "additive_heuristic.h"

#include <vector>

namespace ff_heuristic {
using Proposition = relaxation_heuristic::Proposition;
using UnaryOperator = relaxation_heuristic::UnaryOperator;

/*
  TODO: In a better world, this should not derive from
        AdditiveHeuristic. Rather, the common parts should be
        implemented in a common base class. That refactoring could be
        made at the same time at which we also unify this with the
        other relaxation heuristics and the additional FF heuristic
        implementation in the landmark code.
*/
class FFHeuristic : public additive_heuristic::AdditiveHeuristic {
    // Relaxed plans are represented as a set of operators implemented
    // as a bit vector.
    typedef std::vector<bool> RelaxedPlan;
    RelaxedPlan relaxed_plan;

	std::vector<std::pair<FactPair, int>> subgoals_and_costs;

    void mark_preferred_operators_and_relaxed_plan(
        const State &state, Proposition *goal);
protected:
    virtual int compute_heuristic(const GlobalState &global_state);
public:
    FFHeuristic(const options::Options &options);
    ~FFHeuristic();

	auto get_last_subgoals_and_costs() const -> std::vector<std::pair<FactPair, int>> override;
	auto get_last_relaxed_plan() const -> std::vector<const GlobalOperator *> override;

};
}

#endif
