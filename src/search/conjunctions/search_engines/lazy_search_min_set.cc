#include "lazy_search_min_set.h"

#include "../../open_lists/open_list_factory.h"
#include "../../task_utils/successor_generator.h"
#include "../utils.h"

namespace conjunctions {

namespace detail {
LazySearchMinSetBase<false, false>::LazySearchMinSetBase(const options::Options &) {}

LazySearchMinSetBase<false, true>::LazySearchMinSetBase(const options::Options &) :
	open_successors(),
	learning_state(StateID::no_state) {}

LazySearchMinSetBase<true, false>::LazySearchMinSetBase(const options::Options &opts) :
	open_successors(),
	all_successors(),
	k(opts.get<int>("k")) {}

LazySearchMinSetBase<true, true>::LazySearchMinSetBase(const options::Options &opts) :
	open_successors(),
	all_successors(),
	learning_state(StateID::no_state),
	k(opts.get<int>("k")) {}
}

static const int DEFAULT_LAZY_BOOST = 1000;

template<bool generalized, bool individual_min>
LazySearchMinSet<generalized, individual_min>::LazySearchMinSet(const options::Options &opts) :
	OnlineLearningSearchEngine(opts),
	detail::LazySearchMinSetBase<generalized, individual_min>(opts),
	open_list(opts.get<std::shared_ptr<OpenListFactory>>("open")->create_edge_open_list()),
	randomize_successors(opts.get<bool>("randomize_successors")),
	urng(opts.get<int>("seed") == -1 ? std::random_device()() : opts.get<int>("seed")),
	preferred_usage(PreferredUsage(opts.get_enum("preferred_usage"))),
	heuristics(),
	preferred_operator_heuristics(),
	conjunctions_heuristics(),
	current_state(state_registry.get_initial_state()),
	current_predecessor_id(StateID::no_state),
	current_operator(nullptr),
	current_g(0),
	current_real_g(0),
	current_eval_context(current_state, 0, true, &statistics),
	strategy(opts.get<std::shared_ptr<ConjunctionGenerationStrategy>>("strategy")),
	solved(false),
	h_cache(EvaluationResult::INFTY),
	min_states(),
	min_h(EvaluationResult::INFTY),
	min_set_statistics(online_learning_statistics),
	learning_target(LearningTarget(opts.get_enum("learning_target"))) {}

template<bool generalized, bool individual_min>
void LazySearchMinSet<generalized, individual_min>::set_pref_operator_heuristics(std::vector<Heuristic *> &heur) {
	preferred_operator_heuristics = heur;
}

template<bool generalized, bool individual_min>
void LazySearchMinSet<generalized, individual_min>::initialize() {
	auto initialization_timer = utils::Timer();
	std::cout << "Conducting lazy best first search with online learning of conjunctions, (real) bound = " << bound << std::endl;

	assert(open_list && "open list should have been set during _parse");
	std::set<Heuristic *> hset;
	open_list->get_involved_heuristics(hset);

	for (auto h : hset)
		if (dynamic_cast<ConjunctionsHeuristic *>(h))
			conjunctions_heuristics.push_back(static_cast<ConjunctionsHeuristic *>(h));
	assert(!conjunctions_heuristics.empty() && "There must be at least one heuristic eligible for conjunction learning.");

	// Add heuristics that are used for preferred operators (in case they are
	// not also used in the open list).
	hset.insert(preferred_operator_heuristics.begin(),
				preferred_operator_heuristics.end());

	heuristics.assign(hset.begin(), hset.end());
	assert(!heuristics.empty());
	const auto &initial_state = state_registry.get_initial_state();
	for (auto heuristic : heuristics)
		heuristic->notify_initial_state(initial_state);

	for (auto conjunctions_heuristic : conjunctions_heuristics) {
		solved |= generate_conjunctions(*conjunctions_heuristic, ConjunctionGenerationStrategy::Event::INITIALIZATION, current_eval_context) == ConjunctionGenerationStrategy::Result::SOLVED;
		conjunctions_heuristic->print_statistics();
	}

	std::cout << "Finished initialization, t = " << initialization_timer << std::endl;
	print_intermediate_statistics(*conjunctions_heuristics.front());

	start_search_timer();
}

template<bool generalized, bool individual_min>
void LazySearchMinSet<generalized, individual_min>::get_successor_operators(std::vector<const GlobalOperator *> &ops) {
	assert(ops.empty());

	auto all_operators = std::vector<const GlobalOperator *>();
	g_successor_generator->generate_applicable_ops(current_state, all_operators);

	auto preferred_operators = std::vector<const GlobalOperator *>();
	for (auto *heur : preferred_operator_heuristics) {
		if (!current_eval_context.is_heuristic_infinite(heur)) {
			auto preferred = current_eval_context.get_preferred_operators(heur);
			preferred_operators.insert(
				preferred_operators.end(), preferred.begin(), preferred.end());
		}
	}

	if (randomize_successors) {
		std::shuffle(std::begin(all_operators), std::end(all_operators), urng);
		// Note that preferred_operators can contain duplicates that are
		// only filtered out later, which gives operators "preferred
		// multiple times" a higher chance to be ordered early.
		std::shuffle(std::begin(preferred_operators), std::end(preferred_operators), urng);
	}

	switch (preferred_usage) {
	case PreferredUsage::ALTERNATING:
		for (const auto op : preferred_operators)
			if (!op->is_marked())
				op->mark();
		ops.swap(all_operators);
		break;
	case PreferredUsage::PRUNE_BY_PREFERRED:
		for (const auto op : preferred_operators)
			if (!op->is_marked())
				op->mark();
		ops.swap(preferred_operators);
		break;
	case PreferredUsage::RANK_PREFERRED_FIRST:
		for (const auto op : preferred_operators) {
			if (!op->is_marked()) {
				ops.push_back(op);
				op->mark();
			}
		}
		for (const auto op : all_operators)
			if (!op->is_marked())
				ops.push_back(op);
		break;
	}
}

template<>
template<>
void LazySearchMinSet<false, true>::add_new_successors(const std::vector<StateID> &open_successor_states) {
	if (min_states.back() == current_state.get_id())
		open_successors[current_state.get_id()].insert(std::begin(open_successor_states), std::end(open_successor_states));
}

template<>
template<>
void LazySearchMinSet<true, false>::add_new_successors(const std::vector<StateID> &open_successor_states, const std::vector<StateID> &all_successor_states) {
	auto successor_it = all_successors.find(current_state.get_id());
	if (successor_it == std::end(all_successors))
		return;
	auto successor_distance = successor_it->second + 1;
	assert(successor_distance >= 1);
	if (successor_distance > k)
		return;
	for (const auto &successor_state : all_successor_states) {
		auto successor_state_it = all_successors.find(successor_state);
		if (successor_state_it != std::end(all_successors))
			successor_state_it->second = std::min(successor_state_it->second, successor_distance);
		else
			all_successors[successor_state] = successor_distance;
	}
	for (const auto &open_successor_state : open_successor_states) {
		auto open_successor_state_it = open_successors.find(open_successor_state);
		if (open_successor_state_it != std::end(open_successors))
			open_successor_state_it->second = std::min(open_successor_state_it->second, successor_distance);
		else
			open_successors[open_successor_state] = successor_distance;
	}
}

template<>
template<>
void LazySearchMinSet<true, true>::add_new_successors(const std::vector<StateID> &open_successor_states, const std::vector<StateID> &all_successor_states) {
	for (auto &all_successors_by_min_state : all_successors) {
		auto current_state_it = all_successors_by_min_state.second.find(current_state.get_id());
		if (current_state_it == std::end(all_successors_by_min_state.second))
			continue;
		auto successor_distance = current_state_it->second + 1;
		assert(successor_distance >= 1);
		if (successor_distance <= k) {
			for (const auto &successor_state : all_successor_states) {
				auto successor_state_it = all_successors_by_min_state.second.find(successor_state);
				if (successor_state_it != std::end(all_successors_by_min_state.second))
					successor_state_it->second = std::min(successor_state_it->second, successor_distance);
				else
					all_successors_by_min_state.second[successor_state] = successor_distance;
			}
			auto &current_open_successors = open_successors[all_successors_by_min_state.first];
			for (const auto &open_successor_state : open_successor_states) {
				auto open_successor_state_it = current_open_successors.find(open_successor_state);
				if (open_successor_state_it != std::end(current_open_successors))
					open_successor_state_it->second = std::min(open_successor_state_it->second, successor_distance);
				else
					current_open_successors[open_successor_state] = successor_distance;
			}
		}
	}
}

template<>
void LazySearchMinSet<false, false>::generate_successors() {
	auto operators = std::vector<const GlobalOperator *>();
	get_successor_operators(operators);
	statistics.inc_generated(operators.size());

	for (const auto op : operators) {
		auto new_g = current_g + get_adjusted_cost(*op);
		auto new_real_g = current_real_g + op->get_cost();
		auto is_preferred = op->is_marked();
		if (is_preferred)
			op->unmark();
		if (new_real_g < bound) {
			auto new_eval_context = EvaluationContext(current_eval_context.get_cache(), new_g, is_preferred, nullptr);
			open_list->insert(new_eval_context, std::make_pair(current_state.get_id(), op));
		}
	}
}

template<>
void LazySearchMinSet<false, true>::generate_successors() {
	auto operators = std::vector<const GlobalOperator *>();
	get_successor_operators(operators);
	statistics.inc_generated(operators.size());

	auto open_successor_states = std::vector<StateID>();
	for (const auto op : operators) {
		auto new_g = current_g + get_adjusted_cost(*op);
		auto new_real_g = current_real_g + op->get_cost();
		auto is_preferred = op->is_marked();
		if (is_preferred)
			op->unmark();
		if (new_real_g < bound) {
			auto successor_state = state_registry.get_successor_state(current_state, *op);
			auto new_eval_context = EvaluationContext(current_eval_context.get_cache(), new_g, is_preferred, nullptr);
			if (search_space.get_node(successor_state).is_new()) {
				open_list->insert(new_eval_context, std::make_pair(current_state.get_id(), op));
				if (!std::binary_search(std::begin(open_successor_states), std::end(open_successor_states), successor_state.get_id())) {
					open_successor_states.push_back(successor_state.get_id());
					std::inplace_merge(std::begin(open_successor_states), std::end(open_successor_states) - 1, std::end(open_successor_states));
				}
			}
		}
	}
	add_new_successors(open_successor_states);
}

template<>
void LazySearchMinSet<true, false>::generate_successors() {
	auto operators = std::vector<const GlobalOperator *>();
	get_successor_operators(operators);
	statistics.inc_generated(operators.size());

	auto open_successor_states = std::vector<StateID>();
	auto all_successor_states = std::vector<StateID>();
	all_successor_states.reserve(operators.size());
	for (const auto op : operators) {
		auto new_g = current_g + get_adjusted_cost(*op);
		auto new_real_g = current_real_g + op->get_cost();
		auto is_preferred = op->is_marked();
		if (is_preferred)
			op->unmark();
		if (new_real_g < bound) {
			auto successor_state = state_registry.get_successor_state(current_state, *op);
			auto new_eval_context = EvaluationContext(current_eval_context.get_cache(), new_g, is_preferred, nullptr);
			if (search_space.get_node(successor_state).is_new()) {
				open_list->insert(new_eval_context, std::make_pair(current_state.get_id(), op));
				if (!std::binary_search(std::begin(open_successor_states), std::end(open_successor_states), successor_state.get_id())) {
					open_successor_states.push_back(successor_state.get_id());
					std::inplace_merge(std::begin(open_successor_states), std::end(open_successor_states) - 1, std::end(open_successor_states));
				}
			}
			all_successor_states.push_back(successor_state.get_id());
		}
	}
	add_new_successors(open_successor_states, all_successor_states);
}

template<>
void LazySearchMinSet<true, true>::generate_successors() {
	auto operators = std::vector<const GlobalOperator *>();
	get_successor_operators(operators);
	statistics.inc_generated(operators.size());

	auto open_successor_states = std::vector<StateID>();
	auto all_successor_states = std::vector<StateID>();
	all_successor_states.reserve(operators.size());
	for (const auto op : operators) {
		auto new_g = current_g + get_adjusted_cost(*op);
		auto new_real_g = current_real_g + op->get_cost();
		auto is_preferred = op->is_marked();
		if (is_preferred)
			op->unmark();
		if (new_real_g < bound) {
			auto successor_state = state_registry.get_successor_state(current_state, *op);
			auto new_eval_context = EvaluationContext(current_eval_context.get_cache(), new_g, is_preferred, nullptr);
			if (search_space.get_node(successor_state).is_new()) {
				open_list->insert(new_eval_context, std::make_pair(current_state.get_id(), op));
				if (!std::binary_search(std::begin(open_successor_states), std::end(open_successor_states), successor_state.get_id())) {
					open_successor_states.push_back(successor_state.get_id());
					std::inplace_merge(std::begin(open_successor_states), std::end(open_successor_states) - 1, std::end(open_successor_states));
				}
			}
			all_successor_states.push_back(successor_state.get_id());
		}
	}
	add_new_successors(open_successor_states, all_successor_states);
}

template<bool generalized, bool individual_min>
auto LazySearchMinSet<generalized, individual_min>::step() -> SearchStatus {
	// Invariants:
	// - current_state is the next state for which we want to compute the heuristic.
	// - current_predecessor is a permanent pointer to the predecessor of that state.
	// - current_operator is the operator which leads to current_state from predecessor.
	// - current_g is the g value of the current state according to the cost_type
	// - current_real_g is the g value of the current state (using real costs)

	// stop immediately if a solution was found during the initialization
	if (solved)
		return SOLVED;

	auto node = search_space.get_node(current_state);

	if (node.is_new()) {
		auto dummy_id = current_predecessor_id;
		// HACK! HACK! we do this because SearchNode has no default/copy constructor
		if (dummy_id == StateID::no_state) {
			const auto &initial_state = state_registry.get_initial_state();
			dummy_id = initial_state.get_id();
		}
		auto parent_state = state_registry.lookup_state(dummy_id);
		auto parent_node = search_space.get_node(parent_state);

		if (do_learning())
			return escape_local_minimum(get_learning_target());

		if (current_operator)
			for (auto *heuristic : heuristics)
				heuristic->notify_state_transition(parent_state, *current_operator, current_state);

		if (current_predecessor_id == StateID::no_state)
			print_initial_h_values(current_eval_context);
		for (auto conjunctions_heuristic : conjunctions_heuristics)
			check_timer_and_print_intermediate_statistics(*conjunctions_heuristic);

		statistics.inc_evaluated_states();
		if (!open_list->is_dead_end(current_eval_context)) {
			if (current_predecessor_id == StateID::no_state) {
				node.open_initial();
				if (search_progress.check_progress(current_eval_context))
					print_checkpoint_line(current_g);
			} else {
				node.open(parent_node, current_operator);
			}

			assert(conjunctions_heuristics.front()->is_last_bsg_valid_for_state(current_state));
			if (check_relaxed_plans && is_valid_plan_in_the_original_task(conjunctions_heuristics.front()->get_last_bsg(), current_state.get_values(), *g_root_task())) {
				set_solution(conjunctions_heuristics.front()->get_last_relaxed_plan(), current_state);
				return SOLVED;
			}

			// generate conjunctions according to the selected strategy for this step
			for (auto conjunctions_heuristic : conjunctions_heuristics) {
				auto result = generate_conjunctions(*conjunctions_heuristic, ConjunctionGenerationStrategy::Event::STEP, current_eval_context);
				if (result == ConjunctionGenerationStrategy::Result::SOLVED)
					return SOLVED;
				if (result == ConjunctionGenerationStrategy::Result::DEAD_END) {
					node.mark_as_dead_end();
					statistics.inc_dead_ends();
					remove_current_state_from_open_successors(node);
					return fetch_next_state();
				}
			}

			h_cache[current_state] = current_eval_context.get_heuristic_value(conjunctions_heuristics.front());

			// update min states
			auto improved = update_min_set();
			assert(!min_states.empty());

			node.close();
			if (check_goal_and_set_plan(current_state))
				return SOLVED;
			if (search_progress.check_progress(current_eval_context)) {
				print_checkpoint_line(current_g);
				reward_progress();
			}
			generate_successors();
			statistics.inc_expanded();
			if (improved) {
				// generate conjunctions according to the selected strategy after heuristic improvement
				for (auto conjunctions_heuristic : conjunctions_heuristics) {
					auto result = generate_conjunctions(*conjunctions_heuristic, ConjunctionGenerationStrategy::Event::NEW_BEST_H, current_eval_context);
					if (result == ConjunctionGenerationStrategy::Result::SOLVED)
						return SOLVED;
					if (result == ConjunctionGenerationStrategy::Result::DEAD_END) {
						node.mark_as_dead_end();
						statistics.inc_dead_ends();
						remove_current_state_from_open_successors(node);
						return fetch_next_state();
					}
				}
			}
		} else {
			node.mark_as_dead_end();
			statistics.inc_dead_ends();
		}
		remove_current_state_from_open_successors(node);
	}
	return fetch_next_state();
}

template<bool generalized, bool individual_min>
auto LazySearchMinSet<generalized, individual_min>::fetch_next_state() -> SearchStatus {
	if (open_list->empty()) {
		// TODO: when using PreferredUsage::PRUNE_BY_PREFERRED, we should restart (or similar) instead of failing
		std::cout << "Completely explored state space -- no solution!" << std::endl;
		return FAILED;
	}

	auto next = open_list->remove_min();

	current_predecessor_id = next.first;
	current_operator = next.second;
	auto current_predecessor = state_registry.lookup_state(current_predecessor_id);
	assert(current_operator->is_applicable(current_predecessor));
	current_state = state_registry.get_successor_state(current_predecessor, *current_operator);

	auto pred_node = search_space.get_node(current_predecessor);
	current_g = pred_node.get_g() + get_adjusted_cost(*current_operator);
	current_real_g = pred_node.get_real_g() + current_operator->get_cost();

	/*
	  Note: We mark the node in current_eval_context as "preferred"
	  here. This probably doesn't matter much either way because the
	  node has already been selected for expansion, but eventually we
	  should think more deeply about which path information to
	  associate with the expanded vs. evaluated nodes in lazy search
	  and where to obtain it from.
	*/
	current_eval_context = EvaluationContext(current_state, current_g, true, &statistics);

	return IN_PROGRESS;
}

template<>
auto LazySearchMinSet<false, false>::update_min_set() -> bool {
	auto improved = false;
	if (current_eval_context.get_heuristic_value(conjunctions_heuristics.front()) < min_h) {
		min_states.clear();
		min_h = current_eval_context.get_heuristic_value(conjunctions_heuristics.front());
		improved = true;
	}
	if (current_eval_context.get_heuristic_value(conjunctions_heuristics.front()) == min_h)
		min_states.push_back(current_state.get_id());
	return improved;
}

template<>
auto LazySearchMinSet<false, true>::update_min_set() -> bool {
	auto improved = false;
	if (current_eval_context.get_heuristic_value(conjunctions_heuristics.front()) < min_h) {
		min_states.clear();
		min_h = current_eval_context.get_heuristic_value(conjunctions_heuristics.front());
		open_successors.clear();
		improved = true;
	}
	if (current_eval_context.get_heuristic_value(conjunctions_heuristics.front()) == min_h)
		min_states.push_back(current_state.get_id());
	return improved;
}

template<>
auto LazySearchMinSet<true, false>::update_min_set() -> bool {
	auto improved = false;
	if (current_eval_context.get_heuristic_value(conjunctions_heuristics.front()) < min_h) {
		min_states.clear();
		min_h = current_eval_context.get_heuristic_value(conjunctions_heuristics.front());
		open_successors.clear();
		all_successors.clear();
		improved = true;
	}
	if (current_eval_context.get_heuristic_value(conjunctions_heuristics.front()) == min_h) {
		min_states.push_back(current_state.get_id());
		all_successors[current_state.get_id()] = 0;
	}
	return improved;
}

template<>
auto LazySearchMinSet<true, true>::update_min_set() -> bool {
	auto improved = false;
	if (current_eval_context.get_heuristic_value(conjunctions_heuristics.front()) < min_h) {
		min_states.clear();
		min_h = current_eval_context.get_heuristic_value(conjunctions_heuristics.front());
		open_successors.clear();
		all_successors.clear();
		improved = true;
	}
	if (current_eval_context.get_heuristic_value(conjunctions_heuristics.front()) == min_h) {
		min_states.push_back(current_state.get_id());
		all_successors[current_state.get_id()][current_state.get_id()] = 0;
	}
	return improved;
}

template<>
auto LazySearchMinSet<false, false>::do_learning() -> bool {
	return current_predecessor_id != StateID::no_state && h_cache[state_registry.lookup_state(current_predecessor_id)] > min_h;
}

template<>
auto LazySearchMinSet<false, true>::do_learning() -> bool {
	if (min_states.empty())
		return false;
	for (auto &open_successors_by_min : open_successors) {
		if (open_successors_by_min.second.empty()) {
			learning_state = open_successors_by_min.first;
			return true;
		}
	}
	return false;
}

template<>
auto LazySearchMinSet<true, false>::do_learning() -> bool {
	return !min_states.empty() && open_successors.empty();
}

template<>
auto LazySearchMinSet<true, true>::do_learning() -> bool {
	if (min_states.empty())
		return false;
	for (auto &open_successors_by_min : open_successors) {
		if (open_successors_by_min.second.empty()) {
			learning_state = open_successors_by_min.first;
			return true;
		}
	}
	return false;
}

template<bool generalized, bool individual_min>
auto LazySearchMinSet<generalized, individual_min>::get_learning_target() const -> int {
	switch (learning_target) {
	case LearningTarget::PLUS_ONE:
		return min_h + 1;
	case LearningTarget::NEXT_BEST_IN_OPEN_LIST:
		assert(current_predecessor_id != StateID::no_state);
		return h_cache[state_registry.lookup_state(current_predecessor_id)];
	case LearningTarget::ONE_CONJUNCTION:
		return EvaluationResult::INFTY;
	default:
		std::cerr << "Unknown learning target option." << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
	}
}

template <>
void LazySearchMinSet<false, false>::remove_current_state_from_open_successors(const SearchNode &) {}

template <>
void LazySearchMinSet<false, true>::remove_current_state_from_open_successors(const SearchNode &node) {
	for (auto &open_successors_by_state : open_successors)
		open_successors_by_state.second.erase(node.get_state_id());
}

template <>
void LazySearchMinSet<true, false>::remove_current_state_from_open_successors(const SearchNode &node) {
	open_successors.erase(node.get_state_id());
}

template <>
void LazySearchMinSet<true, true>::remove_current_state_from_open_successors(const SearchNode &node) {
	for (auto &open_successors_by_state : open_successors)
		open_successors_by_state.second.erase(node.get_state_id());
}

template<>
auto LazySearchMinSet<false, false>::get_learning_states() const -> const std::vector<StateID> & {
	return min_states;
}

template<>
auto LazySearchMinSet<false, true>::get_learning_states() const -> std::vector<StateID> {
	return std::vector<StateID>{learning_state};
}

template<>
auto LazySearchMinSet<true, false>::get_learning_states() const -> const std::vector<StateID> & {
	return min_states;
}

template<>
auto LazySearchMinSet<true, true>::get_learning_states() const -> std::vector<StateID> {
	return std::vector<StateID>{learning_state};
}

template<bool generalized, bool individual_min>
void LazySearchMinSet<generalized, individual_min>::notify_escape_local_minimum() {
	assert(!min_states.empty());
	assert(min_h < EvaluationResult::INFTY);
	++online_learning_statistics.num_learning_calls;
	min_set_statistics.total_min_states_size += min_states.size();
	min_set_statistics.max_min_states_size = std::max<int>(min_set_statistics.max_min_states_size, min_states.size());
}

template<>
void LazySearchMinSet<false, false>::reset_min() {
	min_states.clear();
	min_h = std::numeric_limits<int>::max();
}

template<>
void LazySearchMinSet<false, true>::reset_min() {
	min_states.clear();
	min_h = std::numeric_limits<int>::max();
	open_successors.clear();
	learning_state = StateID::no_state;
}

template<>
void LazySearchMinSet<true, false>::reset_min() {
	min_states.clear();
	min_h = std::numeric_limits<int>::max();
	open_successors.clear();
	all_successors.clear();
}

template<>
void LazySearchMinSet<true, true>::reset_min() {
	min_states.clear();
	min_h = std::numeric_limits<int>::max();
	open_successors.clear();
	all_successors.clear();
	learning_state = StateID::no_state;
}

template<bool generalized, bool individual_min>
void LazySearchMinSet<generalized, individual_min>::reward_progress() {
	open_list->boost_preferred();
}

template<bool generalized, bool individual_min>
void LazySearchMinSet<generalized, individual_min>::print_checkpoint_line(int g) const {
	std::cout << "[g=" << g << ", ";
	statistics.print_basic_statistics();
	std::cout << "]" << std::endl;
}

template<bool generalized, bool individual_min>
void LazySearchMinSet<generalized, individual_min>::print_min_set_statistics() const {
	std::cout << "Average min states size: " << min_set_statistics.get_avg_min_states_size() << std::endl;
	std::cout << "Maximum min states size: " << min_set_statistics.max_min_states_size << std::endl;
}

template<bool generalized, bool individual_min>
void LazySearchMinSet<generalized, individual_min>::print_intermediate_statistics(const ConjunctionsHeuristic &heuristic) const {
	OnlineLearningSearchEngine::print_intermediate_statistics(heuristic);
	print_min_set_statistics();
}

template<bool generalized, bool individual_min>
void LazySearchMinSet<generalized, individual_min>::print_statistics() const {
	statistics.print_detailed_statistics();
	print_intermediate_statistics(*conjunctions_heuristics.front());
	search_space.print_statistics();
}


static void _add_succ_order_options(options::OptionParser &parser) {
	std::vector<std::string> options;
	parser.add_option<bool>(
		"randomize_successors",
		"randomize the order in which successors are generated",
		"true");
	parser.add_option<int>("seed", "Random seed (for successor randomization). If this is set to -1, an arbitrary seed is used.", "-1");
	parser.add_enum_option("preferred_usage", {"ALTERNATING", "PRUNE_BY_PREFERRED", "RANK_PREFERRED_FIRST"}, "preferred operator usage", "ALTERNATING");
	parser.document_note(
		"Successor ordering",
		"When using randomize_successors=true and "
		"preferred_usage=RANK_PREFERRED_FIRST, randomization happens before "
		"preferred operators are moved to the front.");
}

template<bool generalized, bool individual_min>
void LazySearchMinSet<generalized, individual_min>::_parse_base(options::OptionParser &parser) {
	parser.document_synopsis("Greedy search (lazy)", "");
	parser.document_note(
		"Open lists",
		"In most cases, lazy greedy best first search uses "
		"an alternation open list with one queue for each evaluator. "
		"If preferred operator heuristics are used, it adds an "
		"extra queue for each of these evaluators that includes "
		"only the nodes that are generated with a preferred operator. "
		"If only one evaluator and no preferred operator heuristic is used, "
		"the search does not use an alternation open list "
		"but a standard open list with only one queue.");
	parser.document_note(
		"Equivalent statements using general lazy search",
		"\n```\n--heuristic h2=eval2\n"
		"--search lazy_greedy([eval1, h2], preferred=h2, boost=100)\n```\n"
		"is equivalent to\n"
		"```\n--heuristic h1=eval1 --heuristic h2=eval2\n"
		"--search lazy(alt([single(h1), single(h1, pref_only=true), single(h2),\n"
		"                  single(h2, pref_only=true)], boost=100),\n"
		"              preferred=h2)\n```\n"
		"------------------------------------------------------------\n"
		"```\n--search lazy_greedy([eval1, eval2], boost=100)\n```\n"
		"is equivalent to\n"
		"```\n--search lazy(alt([single(eval1), single(eval2)], boost=100))\n```\n"
		"------------------------------------------------------------\n"
		"```\n--heuristic h1=eval1\n--search lazy_greedy(h1, preferred=h1)\n```\n"
		"is equivalent to\n"
		"```\n--heuristic h1=eval1\n"
		"--search lazy(alt([single(h1), single(h1, pref_only=true)], boost=1000),\n"
		"              preferred=h1)\n```\n"
		"------------------------------------------------------------\n"
		"```\n--search lazy_greedy(eval1)\n```\n"
		"is equivalent to\n"
		"```\n--search lazy(single(eval1))\n```\n",
		true);

	parser.add_list_option<ScalarEvaluator *>("evals", "scalar evaluators");
	parser.add_list_option<Heuristic *>(
		"preferred",
		"use preferred operators of these heuristics", "[]");
	parser.add_option<int>(
		"boost",
		"boost value for alternation queues that are restricted "
		"to preferred operator nodes",
		options::OptionParser::to_str(DEFAULT_LAZY_BOOST));
	_add_succ_order_options(parser);
	SearchEngine::add_options_to_parser(parser);
	OnlineLearningSearchEngine::add_options_to_parser(parser);
	parser.add_enum_option("learning_target", {"PLUS_ONE", "NEXT_BEST_IN_OPEN_LIST", "ONE_CONJUNCTION"}, "Target heuristic value for learning.", "NEXT_BEST_IN_OPEN_LIST",
		{"Minimum heuristic value plus one.", "Next lowest heuristic value in open list.", "Learn a single conjunction."});
	parser.add_option<int>("k", "Local minimum lookahead bound", "1");
	parser.add_option<bool>("individual_min", "Call learning if all successsors of one minimum were evaluated instead of all of them.", "false");
}

template class LazySearchMinSet<false, false>;
template class LazySearchMinSet<false, true>;
template class LazySearchMinSet<true, false>;
template class LazySearchMinSet<true, true>;

}
