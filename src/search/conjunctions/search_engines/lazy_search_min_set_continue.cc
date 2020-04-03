#include "lazy_search_min_set_continue.h"

#include "../../options/option_parser.h"
#include "../../options/plugin.h"

namespace conjunctions {
template<bool generalized, bool individual_min>
LazySearchMinSetContinue<generalized, individual_min>::LazySearchMinSetContinue(const options::Options &opts) :
	LazySearchMinSet<generalized, individual_min>(opts) {}

template<bool generalized, bool individual_min>
auto LazySearchMinSetContinue<generalized, individual_min>::escape_local_minimum(int target_h) -> SearchStatus {
	this->notify_escape_local_minimum();
	// learn until the heuristic value of all local minima is above the target threshold
	for (auto local_min : this->get_learning_states()) {
		this->current_state = this->state_registry.lookup_state(local_min);
		auto node = this->search_space.get_node(this->current_state);
		assert(node.is_closed());
		this->current_eval_context = EvaluationContext(this->current_state, node.get_g(), true, &this->statistics);
		if (this->current_eval_context.is_heuristic_infinite(this->conjunctions_heuristics.front())) {
			node.mark_as_dead_end();
			this->statistics.inc_dead_ends();
		} else {
			while (this->current_eval_context.get_heuristic_value(this->conjunctions_heuristics.front()) < target_h) {
				auto result = this->generate_conjunctions(*this->conjunctions_heuristics.front(), ConjunctionGenerationStrategy::Event::LOCAL_MINIMUM, this->current_eval_context);
				assert(result != ConjunctionGenerationStrategy::Result::UNMODIFIED);
				if (result == ConjunctionGenerationStrategy::Result::SOLVED)
					return SOLVED;
				if (result == ConjunctionGenerationStrategy::Result::DEAD_END) {
					node.mark_as_dead_end();
					this->statistics.inc_dead_ends();
					break;
				}
				if (this->learning_target == LazySearchMinSetContinue<generalized, individual_min>::LearningTarget::ONE_CONJUNCTION)
					break;
			}
		}
	}
	this->reset_min();
	return this->fetch_next_state();
}

static SearchEngine *_parse(options::OptionParser &parser) {
	LazySearchMinSet<>::_parse_base(parser);
	auto opts = parser.parse();
	auto generalized = opts.get<int>("k") != 1;
	auto individual_min = opts.get<bool>("individual_min");
	if (generalized) {
		if (individual_min)
			return LazySearchMinSet<true, true>::_set_open_list_and_create_search_engine<LazySearchMinSetContinue<true, true>>(parser);
		else
			return LazySearchMinSet<true, false>::_set_open_list_and_create_search_engine<LazySearchMinSetContinue<true, false>>(parser);
	} else {
		if (individual_min)
			return LazySearchMinSet<false, true>::_set_open_list_and_create_search_engine<LazySearchMinSetContinue<false, true>>(parser);
		else
			return LazySearchMinSet<false, false>::_set_open_list_and_create_search_engine<LazySearchMinSetContinue<false, false>>(parser);
	}
}

static options::Plugin<SearchEngine> _plugin("lazy_greedy_c_min_set_continue", _parse);
}
