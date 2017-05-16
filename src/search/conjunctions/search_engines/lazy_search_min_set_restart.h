#ifndef CONJUNCTIONS_LAZY_SEARCH_MIN_SET_RESTART_H
#define CONJUNCTIONS_LAZY_SEARCH_MIN_SET_RESTART_H

#include "lazy_search_min_set.h"

#include <memory>
#include <vector>

namespace conjunctions {

template<bool generalized, bool individual_min>
class LazySearchMinSetRestart : public LazySearchMinSet<generalized, individual_min> {
protected:
	std::unique_ptr<SearchSpace> current_search_space;
	std::vector<StateID> current_phase_initial_states;

	auto step() -> SearchStatus override;

	auto fetch_next_state() -> SearchStatus override;
	auto restart() -> SearchStatus;

	auto escape_local_minimum(int target_h) -> SearchStatus override;

public:
	explicit LazySearchMinSetRestart(const options::Options &opts);
	virtual ~LazySearchMinSetRestart() = default;
};
}

#endif
