#ifndef CONJUNCTIONS_LAZY_SEARCH_MIN_SET_CONTINUE_H
#define CONJUNCTIONS_LAZY_SEARCH_MIN_SET_CONTINUE_H

#include "lazy_search_min_set.h"

namespace conjunctions {

template<bool generalized, bool individual_min>
class LazySearchMinSetContinue : public LazySearchMinSet<generalized, individual_min> {
protected:
	auto escape_local_minimum(int target_h) -> SearchStatus override;

public:
	explicit LazySearchMinSetContinue(const options::Options &opts);
	virtual ~LazySearchMinSetContinue() = default;
};
}

#endif
