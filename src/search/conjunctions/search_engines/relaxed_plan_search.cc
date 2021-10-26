#include "relaxed_plan_search.h"

#include "../global_operator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../task_utils/successor_generator.h"

#include "../open_lists/standard_scalar_open_list.h"
#include "../open_lists/tiebreaking_open_list.h"

#include "../utils/system.h"
#include "../utils.h"


namespace conjunctions {

RelaxedPlanSearch::RelaxedPlanSearch(const options::Options &opts) :
	OnlineLearningSearchEngine(opts),
	current_search_space(std::make_unique<SearchSpace>(state_registry, cost_type)),
	heuristic(static_cast<ConjunctionsHeuristic *>(opts.get<Heuristic *>("h"))),
	last_state(state_registry.get_initial_state()),
	current_initial_state(state_registry.get_initial_state()),
	current_eval_context(state_registry.get_initial_state(), &statistics),
	conjunctions_strategy(opts.get<std::shared_ptr<ConjunctionGenerationStrategy>>("strategy")),
	rps_statistics(),
	rp_sequence(RPSequence(opts.get_enum("rp_sequence"))),
	num_states_since_last_improvement(0),
	last_improved_heuristic_value(std::numeric_limits<decltype(last_improved_heuristic_value)>::max()),
	improvement_tolerance(opts.get<int>("improvement_tolerance")),
	restart_in_dead_ends(opts.get<bool>("restart_in_dead_ends")),
	restart_in_stagnation(opts.get<bool>("restart_in_stagnation")),
	refine_on_previous(opts.get<bool>("refine_on_previous")) {}

RelaxedPlanSearch::~RelaxedPlanSearch() {}

void RelaxedPlanSearch::initialize() {
	auto initialization_timer = utils::Timer();
	assert(heuristic);
	std::cout << "Conducting relaxed plan search with explicit conjunctions, (real) bound = " << bound << std::endl;

	conjunctions_strategy->dump_options();

	current_search_space->get_node(current_eval_context.get_state()).open_initial();
	search_space.get_node(current_eval_context.get_state()).open_initial();

	heuristic->notify_initial_state(current_eval_context.get_state());
	generate_conjunctions(*heuristic, ConjunctionGenerationStrategy::Event::INITIALIZATION, current_eval_context);
	heuristic->print_statistics();
	std::cout << "Finished initialization, t = " << initialization_timer << std::endl;
	print_intermediate_statistics(*heuristic);

	start_search_timer();

	if (!heuristic->is_last_bsg_valid_for_state(current_eval_context.get_state()))
		current_eval_context = EvaluationContext(current_eval_context.get_state(), &statistics);

	current_eval_context.is_heuristic_infinite(heuristic);
	print_initial_h_values(current_eval_context);

	assert(current_eval_context.is_heuristic_infinite(heuristic) || heuristic->is_last_bsg_valid_for_state(current_eval_context.get_state()));
	++rps_statistics.num_episodes;
}

void RelaxedPlanSearch::update_global_search_space() {
	auto local_plan = Plan();
	current_search_space->trace_path(current_eval_context.get_state(), local_plan);
	assert(!search_space.get_node(current_initial_state).is_new());
	auto current_state = current_initial_state;
	for (const auto op : local_plan) {
		auto current_parent_node = search_space.get_node(current_state);
		assert(op->is_applicable(current_state));
		auto successor = state_registry.get_successor_state(current_state, *op);
		auto successor_node = search_space.get_node(successor);
		if (successor_node.is_new())
			successor_node.open(current_parent_node, op);
		current_state = successor;
	}
}

auto RelaxedPlanSearch::get_sequence_length() const -> decltype(heuristic->get_last_bsg().nodes.size()) {
	assert(heuristic->is_last_bsg_valid_for_state(current_eval_context.get_state()));
	const auto &bsg = heuristic->get_last_bsg();
	const auto &state_values = current_eval_context.get_state().get_values();
	switch (rp_sequence) {
	case RPSequence::ALL:
		return get_applicable_sequence_length(bsg, state_values, *g_root_task());
	case RPSequence::ALL_INTENDED:
		return get_applicable_as_intended_sequence_length(bsg, state_values, *g_root_task());
	case RPSequence::FIRST_DELETER:
		return get_sequence_to_first_deleter_length(bsg, state_values, *g_root_task());
	default:
		std::cerr << "Unknown PO sequence option: " << static_cast<int>(rp_sequence) << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
	}
}

auto RelaxedPlanSearch::advance_to_next_state() -> SearchStatus {
	auto sequence_length = get_sequence_length();
	rps_statistics.advance(sequence_length);
	const auto &bsg = heuristic->get_last_bsg();
	if (sequence_length == bsg.nodes.size()) {
		update_global_search_space();
		set_solution(heuristic->get_last_relaxed_plan(), current_eval_context.get_state());
#ifndef NDEBUG
		auto state = state_registry.get_initial_state();
		for (auto op : get_plan()) {
			assert(op->is_applicable(state));
			state = state_registry.get_successor_state(state, *op);
		}
		assert(test_goal(state));
#endif
		return SOLVED;
	}
	last_state = current_eval_context.get_state();
	for (auto bsg_it = std::rbegin(bsg.nodes); bsg_it != std::rbegin(bsg.nodes) + sequence_length; ++bsg_it) {
		assert(bsg_it->action);
		const auto op = bsg_it->action->op->get_global_operator();
		auto node = current_search_space->get_node(current_eval_context.get_state());
		if (node.get_real_g() + op->get_cost() > bound)
			return FAILED;
		current_eval_context = EvaluationContext(state_registry.get_successor_state(current_eval_context.get_state(), *op), node.get_g() + get_adjusted_cost(*op), true, &statistics);
		if (node.is_open())
			node.close();
		auto next_node = current_search_space->get_node(current_eval_context.get_state());
		if (!next_node.is_closed())
			next_node.open(node, op);
	}
	return IN_PROGRESS;
}

auto RelaxedPlanSearch::handle_dead_end(SearchNode &global_node, SearchNode &local_node) -> SearchStatus {
	assert(global_node.get_state_id() == local_node.get_state_id());
	assert(global_node.get_state_id() == current_eval_context.get_state().get_id());
	statistics.inc_dead_ends();
	if (!global_node.is_dead_end()) {
		global_node.mark_as_dead_end();
		if (global_node.get_state_id() == state_registry.get_initial_state().get_id()) {
			std::cout << "Initial state is a dead end, no solution" << std::endl;
			return FAILED;
		}
	}
	return restart_in_dead_ends ? restart() : continue_in_parent(local_node);
}

auto RelaxedPlanSearch::handle_duplicate() -> SearchStatus {
	++rps_statistics.num_duplicates;
	++rps_statistics.num_episodes;
	update_global_search_space();
	current_initial_state = current_eval_context.get_state();
	last_state = current_initial_state;
	current_search_space = std::make_unique<SearchSpace>(state_registry, cost_type);
	current_search_space->get_node(current_eval_context.get_state()).open_initial();
	refine_once(refine_on_previous);
	return IN_PROGRESS;
}

auto RelaxedPlanSearch::handle_stagnation() -> SearchStatus {
	++rps_statistics.num_stagnation;
	if (restart_in_stagnation)
		return restart();
	refine_once(refine_on_previous);
	return IN_PROGRESS;
}

auto RelaxedPlanSearch::restart() -> SearchStatus {
	current_eval_context = EvaluationContext(state_registry.get_initial_state(), &statistics);
	++rps_statistics.num_episodes;
	last_improved_heuristic_value = std::numeric_limits<decltype(last_improved_heuristic_value)>::max();
	num_states_since_last_improvement = 0;
	current_initial_state = current_eval_context.get_state();
	last_state = current_initial_state;
	current_search_space = std::make_unique<SearchSpace>(state_registry, cost_type);
	current_search_space->get_node(current_eval_context.get_state()).open_initial();
	return IN_PROGRESS;
}

auto RelaxedPlanSearch::continue_in_parent(SearchNode &local_node) -> SearchStatus {
	assert(search_space.get_node(current_eval_context.get_state()).is_dead_end());
	if (local_node.get_parent_state_id() == StateID::no_state)
		return restart();
	last_state = current_eval_context.get_state();
	current_eval_context = EvaluationContext(state_registry.lookup_state(local_node.get_parent_state_id()), &statistics);
	return IN_PROGRESS;
}

auto RelaxedPlanSearch::step() -> SearchStatus {
	check_timer_and_print_intermediate_statistics(*heuristic);

	auto global_node = search_space.get_node(current_eval_context.get_state());
	auto node = current_search_space->get_node(current_eval_context.get_state());

	if (global_node.is_dead_end() || current_eval_context.is_heuristic_infinite(heuristic)) {
		if (global_node.get_state_id() == state_registry.get_initial_state().get_id()) {
			std::cout << "Initial state is a dead end, no solution" << std::endl;
			return FAILED;
		}
		refine_once(true);
		return handle_dead_end(global_node, node);
	}

	if (!node.is_open())
		return handle_duplicate();

	if (current_eval_context.get_heuristic_value(heuristic) > last_improved_heuristic_value) {
		if (++num_states_since_last_improvement > improvement_tolerance)
			return handle_stagnation();
	} else {
		last_improved_heuristic_value = current_eval_context.get_heuristic_value(heuristic);
		num_states_since_last_improvement = 0;
	}

	search_progress.check_progress(current_eval_context);

	return advance_to_next_state();
}


void RelaxedPlanSearch::refine_once(bool refine_on_last) {
	++online_learning_statistics.num_learning_calls;

	auto generate_conjunctions_wrapper = [this, refine_on_last]() {
		if (!refine_on_last)
			return generate_conjunctions(*heuristic, ConjunctionGenerationStrategy::Event::LOCAL_MINIMUM, current_eval_context, false);
		auto last_eval_context = EvaluationContext(last_state, &statistics);
		return generate_conjunctions(*heuristic, ConjunctionGenerationStrategy::Event::LOCAL_MINIMUM, last_eval_context, false);
	};

	auto learning_result = generate_conjunctions_wrapper();
	if (learning_result == ConjunctionGenerationStrategy::Result::MODIFIED) {
		assert(!heuristic->is_last_bsg_valid_for_state(current_eval_context.get_state()));
		// clear the cached heuristic value
		current_eval_context = EvaluationContext(current_eval_context.get_state(), &statistics);
		return;
	}
	// dead end should only be possible if more than one iteration of refinement was done
	// solved should only be possible if
	//    a) this was called because of stagnation and the relaxed plan in this state happens to be a real plan
	//    b) this was called in a duplicate state but with better tie breaking the relaxed plan in this state is now a real plan
	assert(learning_result == ConjunctionGenerationStrategy::Result::SOLVED || learning_result == ConjunctionGenerationStrategy::Result::DEAD_END);
	if (refine_on_last && learning_result == ConjunctionGenerationStrategy::Result::SOLVED) {
		assert(!current_search_space->get_node(last_state).is_new());
		// we found a solution from the last state, so let's go back to it
		current_eval_context = EvaluationContext(last_state, &statistics);
	}
	assert(learning_result != ConjunctionGenerationStrategy::Result::SOLVED || heuristic->is_last_bsg_valid_for_state(current_eval_context.get_state()));
}

void RelaxedPlanSearch::print_rps_statistics() const {
	std::cout << "RPS episodes: " << rps_statistics.num_episodes << std::endl;
	std::cout << "RPS duplicates: " << rps_statistics.num_duplicates << std::endl;
	std::cout << "RPS stagnation: " << rps_statistics.num_stagnation << std::endl;
	std::cout << "RPS max sequence: " << rps_statistics.max_rp_sequence_length << std::endl;
	std::cout << "RPS avg sequence: " << rps_statistics.get_avg_rp_sequence_length() << std::endl;
}

void RelaxedPlanSearch::print_intermediate_statistics(const ConjunctionsHeuristic &heuristic) const {
	OnlineLearningSearchEngine::print_intermediate_statistics(heuristic);
	print_rps_statistics();
}


void RelaxedPlanSearch::print_statistics() const {
	statistics.print_detailed_statistics();
	print_intermediate_statistics(*heuristic);
}

static auto _parse(OptionParser &parser) -> SearchEngine * {
	parser.document_synopsis("Relaxed plan search", "");
	parser.add_option<Heuristic *>("h", "heuristic used for search, must be of type ConjunctionsHeuristic");
	parser.add_option<int>("improvement_tolerance", "Number of times the next state may be worse until refinement is triggered.", "10");
	parser.add_enum_option("rp_sequence", {"ALL", "ALL_INTENDED", "FIRST_DELETER"},
		"Use leading operator sequence from relaxed plan to generate the next state.", "FIRST_DELETER");
	parser.add_option<bool>("restart_in_dead_ends", "Restart in dead ends. If this is set to false, search will continue from the parent node instead.", "true");
	parser.add_option<bool>("restart_in_stagnation", "Restart after the not finding a better state for a while.", "false");
	parser.add_option<bool>("refine_on_previous", "Call refinement on the previous state instead of the current one (avoids reevaluating the heuristic on the current state).", "false");
	OnlineLearningSearchEngine::add_options_to_parser(parser);
	SearchEngine::add_options_to_parser(parser);
	auto opts = parser.parse();

	if (parser.help_mode() || parser.dry_run())
		return nullptr;
	return new RelaxedPlanSearch(opts);
}

static Plugin<SearchEngine> _plugin("rps", _parse);
}
