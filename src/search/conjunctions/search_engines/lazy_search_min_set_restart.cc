#include "lazy_search_min_set_restart.h"

#include "../../options/option_parser.h"
#include "../../options/plugin.h"
#include "../utils.h"

namespace conjunctions {
template<bool generalized, bool individual_min>
LazySearchMinSetRestart<generalized, individual_min>::LazySearchMinSetRestart(const options::Options &opts) :
	LazySearchMinSet<generalized, individual_min>(opts),
	current_search_space(new SearchSpace(this->state_registry, this->cost_type)),
	current_phase_initial_states({this->current_state.get_id()}) {}

template<bool generalized, bool individual_min>
auto LazySearchMinSetRestart<generalized, individual_min>::fetch_next_state() -> SearchStatus {
	assert(!current_phase_initial_states.empty());
	if (this->open_list->empty()) {
		for (auto current_phase_initial_state_id : current_phase_initial_states) {
			if (current_phase_initial_state_id == this->state_registry.get_initial_state().get_id()) {
				std::cout << "Completely explored state space -- no solution!" << std::endl;
				return FAILED;
			}
			this->search_space.get_node(this->state_registry.lookup_state(current_phase_initial_state_id)).mark_as_dead_end();
		}
		return restart();
	}

	auto next = this->open_list->remove_min();

	this->current_predecessor_id = next.first;
	this->current_operator = next.second;
	auto current_predecessor = this->state_registry.lookup_state(this->current_predecessor_id);
	assert(this->current_operator->is_applicable(current_predecessor));
	this->current_state = this->state_registry.get_successor_state(current_predecessor, *this->current_operator);

	auto pred_node = current_search_space->get_node(current_predecessor);
	this->current_g = pred_node.get_g() + this->get_adjusted_cost(*this->current_operator);
	this->current_real_g = pred_node.get_real_g() + this->current_operator->get_cost();

	/*
	  Note: We mark the node in current_eval_context as "preferred"
	  here. This probably doesn't matter much either way because the
	  node has already been selected for expansion, but eventually we
	  should think more deeply about which path information to
	  associate with the expanded vs. evaluated nodes in lazy search
	  and where to obtain it from.
	*/
	this->current_eval_context = EvaluationContext(this->current_state, this->current_g, true, &this->statistics);

	return IN_PROGRESS;
}

template<bool generalized, bool individual_min>
auto LazySearchMinSetRestart<generalized, individual_min>::restart() -> SearchStatus {
	current_search_space = std::make_unique<SearchSpace>(this->state_registry, this->cost_type);
	this->current_state = this->state_registry.get_initial_state();
	this->current_g = 0;
	this->current_real_g = 0;
	current_phase_initial_states = {this->current_state.get_id()};
	this->current_eval_context = EvaluationContext(this->current_state, 0, true, &this->statistics);
	this->reset_min();
	if (this->current_eval_context.is_heuristic_infinite(this->conjunctions_heuristics.front())) {
		std::cout << "Initial state is a dead end, no solution" << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_UNSOLVABLE);
	}
	auto node = current_search_space->get_node(this->current_state);
	node.open_initial();
	this->generate_successors();
	node.close();
	return fetch_next_state();
}

template<bool generalized, bool individual_min>
auto LazySearchMinSetRestart<generalized, individual_min>::step() -> SearchStatus {
	// Invariants:
	// - current_state is the next state for which we want to compute the heuristic.
	// - current_predecessor is a permanent pointer to the predecessor of that state.
	// - current_operator is the operator which leads to current_state from predecessor.
	// - current_g is the g value of the current state according to the cost_type
	// - current_real_g is the g value of the current state (using real costs)

	// stop immediately if a solution was found during the initialization
	if (this->solved)
		return SOLVED;

	auto node = current_search_space->get_node(this->current_state);
	auto global_node = this->search_space.get_node(this->current_state);

	if (!global_node.is_dead_end() && node.is_new()) {
		auto dummy_id = this->current_predecessor_id;
		// HACK! HACK! we do this because SearchNode has no default/copy constructor
		if (dummy_id == StateID::no_state) {
			const auto &initial_state = this->state_registry.get_initial_state();
			dummy_id = initial_state.get_id();
		}
		auto parent_state = this->state_registry.lookup_state(dummy_id);
		auto parent_node = current_search_space->get_node(parent_state);

		if (this->do_learning())
			return escape_local_minimum(this->get_learning_target());

		if (this->current_operator)
			for (auto *heuristic : this->heuristics)
				heuristic->notify_state_transition(parent_state, *this->current_operator, this->current_state);

		if (this->current_predecessor_id == StateID::no_state)
			print_initial_h_values(this->current_eval_context);
		for (auto conjunctions_heuristic : this->conjunctions_heuristics)
			this->check_timer_and_print_intermediate_statistics(*conjunctions_heuristic);

		this->statistics.inc_evaluated_states();
		if (!this->open_list->is_dead_end(this->current_eval_context)) {
			if (this->current_predecessor_id == StateID::no_state) {
				node.open_initial();
				if (global_node.is_new()) {
					global_node.open_initial();
					global_node.close();
				}
				if (this->search_progress.check_progress(this->current_eval_context))
					this->print_checkpoint_line(this->current_g);
			} else {
				node.open(parent_node, this->current_operator);
				if (global_node.is_new()) {
					global_node.open(this->search_space.get_node(parent_state), this->current_operator);
					global_node.close();
				}
			}

			assert(this->conjunctions_heuristics.front()->is_last_bsg_valid_for_state(this->current_state));
			if (this->check_relaxed_plans && is_valid_plan_in_the_original_task(this->conjunctions_heuristics.front()->get_last_bsg(), this->current_state.get_values(), *g_root_task())) {
				this->set_solution(this->conjunctions_heuristics.front()->get_last_relaxed_plan(), this->current_state);
				return SOLVED;
			}

			this->h_cache[this->current_state] = this->current_eval_context.get_heuristic_value(this->conjunctions_heuristics.front());

			// update min states
			this->update_min_set();

			node.close();
			if (this->check_goal_and_set_plan(this->current_state))
				return SOLVED;
			if (this->search_progress.check_progress(this->current_eval_context)) {
				this->print_checkpoint_line(this->current_g);
				this->reward_progress();
			}
			this->generate_successors();
			this->statistics.inc_expanded();
		} else {
			global_node.mark_as_dead_end();
			this->statistics.inc_dead_ends();
		}
		this->remove_current_state_from_open_successors(node);
	}
	return fetch_next_state();
}

template<bool generalized, bool individual_min>
auto LazySearchMinSetRestart<generalized, individual_min>::escape_local_minimum(int target_h) -> SearchStatus {
	this->notify_escape_local_minimum();
	// learn until the heuristic value of all local minima is above the target threshold
	this->open_list->clear();
	auto new_search_space = std::make_unique<SearchSpace>(this->state_registry, this->cost_type);
	auto next_phase_initial_states = std::vector<StateID>();
	for (auto local_min : this->get_learning_states()) {
		this->current_state = this->state_registry.lookup_state(local_min);
		auto node = current_search_space->get_node(this->current_state);
		auto global_node = this->search_space.get_node(this->current_state);
		assert(node.is_closed());
		this->current_eval_context = EvaluationContext(this->current_state, node.get_g(), true, &this->statistics);
		if (this->current_eval_context.is_heuristic_infinite(this->conjunctions_heuristics.front())) {
			global_node.mark_as_dead_end();
			this->statistics.inc_dead_ends();
		} else {
			while (this->current_eval_context.get_heuristic_value(this->conjunctions_heuristics.front()) < target_h) {
				auto result = this->generate_conjunctions(*this->conjunctions_heuristics.front(), ConjunctionGenerationStrategy::Event::LOCAL_MINIMUM, this->current_eval_context);
				assert(result != ConjunctionGenerationStrategy::Result::UNMODIFIED);
				if (result == ConjunctionGenerationStrategy::Result::SOLVED)
					return SOLVED;
				if (result == ConjunctionGenerationStrategy::Result::DEAD_END) {
					global_node.mark_as_dead_end();
					this->statistics.inc_dead_ends();
					break;
				}
				if (this->learning_target == LazySearchMinSetRestart<generalized, individual_min>::LearningTarget::ONE_CONJUNCTION)
					break;
			}
		}
		if (!global_node.is_dead_end()) {
			auto new_node = new_search_space->get_node(this->current_state);
			new_node.open_initial();
			this->generate_successors();
			new_node.close();
			this->h_cache[this->current_state] = this->current_eval_context.get_heuristic_value(this->conjunctions_heuristics.front());
			next_phase_initial_states.push_back(local_min);
		}
	}
	if (this->open_list->empty())
		return restart();
	this->reset_min();
	current_search_space = std::move(new_search_space);
	current_phase_initial_states = std::move(next_phase_initial_states);
	return fetch_next_state();
}


static SearchEngine *_parse(options::OptionParser &parser) {
	LazySearchMinSet<false, false>::_parse_base(parser);
	auto opts = parser.parse();
	auto generalized = opts.get<int>("k") != 1;
	auto individual_min = opts.get<bool>("individual_min");
	if (generalized) {
		if (individual_min)
			return LazySearchMinSet<true, true>::_set_open_list_and_create_search_engine<LazySearchMinSetRestart<true, true>>(parser);
		else
			return LazySearchMinSet<true, false>::_set_open_list_and_create_search_engine<LazySearchMinSetRestart<true, false>>(parser);
	} else {
		if (individual_min)
			return LazySearchMinSet<false, true>::_set_open_list_and_create_search_engine<LazySearchMinSetRestart<false, true>>(parser);
		else
			return LazySearchMinSet<false, false>::_set_open_list_and_create_search_engine<LazySearchMinSetRestart<false, false>>(parser);
	}
}

static options::Plugin<SearchEngine> _plugin("lazy_greedy_c_min_set_restart", _parse);
}
