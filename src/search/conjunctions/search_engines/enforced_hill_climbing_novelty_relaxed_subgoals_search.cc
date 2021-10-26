#include "enforced_hill_climbing_novelty_relaxed_subgoals_search.h"

#include "../novelty_heuristic.h"

#include "../global_operator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../task_utils/successor_generator.h"

#include "../utils/system.h"
#include "../utils.h"
#include "../conjunctions_subgoal_count_heuristic.h"


namespace conjunctions {

auto EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch::create_open_list(int w) -> open_list_t {
	if (w == 0)
		return open_list_t([](const auto &lhs, const auto &rhs) {
			return lhs.g > rhs.g;
		});
	if (w == std::numeric_limits<int>::max())
		return open_list_t([](const auto &lhs, const auto &rhs) {
			assert(lhs.h >= 0 && rhs.h >= 0);
			return lhs.h > rhs.h;
		});
	return open_list_t([w](const auto &lhs, const auto &rhs) {
		assert(lhs.h >= 0 && rhs.h >= 0);
		const auto lhs_f = lhs.g + w * lhs.h;
		const auto rhs_f = rhs.g + w * rhs.h;
		if (lhs_f != rhs_f)
			return lhs_f > rhs_f;
		return lhs.h > rhs.h;
	});
}

EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch::EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch(const options::Options &opts) :
	OnlineLearningSearchEngine(opts),
	weight(opts.get<int>("w")),
	open_list(create_open_list(weight)),
	heuristic(static_cast<ConjunctionsHeuristic *>(opts.get<Heuristic *>("h"))),
	novelty_heuristic(static_cast<novelty::NoveltyHeuristic *>(opts.get<Heuristic *>("novelty"))),
	current_eval_context(state_registry.get_initial_state(), &statistics),
	current_real_g(0),
	subgoal_heuristic(ConjunctionsSubgoalHeuristic::SubgoalAggregationMethod(opts.get_enum("subgoal_aggregation_method")), opts.get<bool>("path_dependent_subgoals")),
	conjunctions_strategy(opts.get<std::shared_ptr<ConjunctionGenerationStrategy>>("strategy")),
	bfs_bound(opts.get<int>("bfs_bound")),
	learning_stagnation_counter(0),
	solved(false),
	bfs_cutoff(false),
	no_learning(opts.get<bool>("no_learning")),
	restart_in_dead_ends(opts.get<bool>("restart_in_dead_ends")),
	always_reevaluate(opts.get<bool>("always_reevaluate")),
	randomize_successors(opts.get<bool>("randomize_successors")),
	learning_stagnation_threshold(opts.get<int>("learning_stagnation_threshold")),
	learning_stagnation_restart(opts.get<bool>("learning_stagnation_restart")),
	search_space_exhaustion(SearchSpaceExhaustion(opts.get_enum("search_space_exhaustion"))),
	max_growth(opts.get<double>("max_growth")),
	urng(opts.get<int>("seed") == -1 ? std::random_device()() : opts.get<int>("seed")) {
	const auto &initial_state = state_registry.get_initial_state();
	heuristic->notify_initial_state(initial_state);
	novelty_heuristic->notify_initial_state(initial_state);
}

EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch::~EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch() = default;

void EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch::reach_state(const GlobalState &parent, const GlobalOperator &op, const GlobalState &state) {
	heuristic->notify_state_transition(parent, op, state);
	novelty_heuristic->notify_state_transition(parent, op, state);
}

void EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch::initialize() {
	auto initialization_timer = utils::Timer();
	assert(heuristic);
	std::cout << "Conducting enforced hill-climbing search with explicit conjunctions, (real) bound = " << bound << std::endl;

	std::cout << "Breadth first search depth bound: " << bfs_bound << std::endl;
	std::cout << "Always reevaluate the local minimum neighborhood after adding a conjunction: " << always_reevaluate << std::endl;
	conjunctions_strategy->dump_options();

	auto node = search_space.get_node(current_eval_context.get_state());
	node.open_initial();

	solved = generate_conjunctions(*heuristic, ConjunctionGenerationStrategy::Event::INITIALIZATION, current_eval_context, true, bound) == ConjunctionGenerationStrategy::Result::SOLVED
		&& heuristic->get_last_bsg().get_real_cost() <= bound;
	heuristic->print_statistics();
	std::cout << "Finished initialization, t = " << initialization_timer << std::endl;
	print_intermediate_statistics(*heuristic);

	start_search_timer();

	if (!heuristic->is_last_bsg_valid_for_state(current_eval_context.get_state()))
		current_eval_context = EvaluationContext(current_eval_context.get_state(), &statistics);

	evaluate_if_neccessary(current_eval_context);
	auto dead_end = current_eval_context.is_heuristic_infinite(heuristic);
	print_initial_h_values(current_eval_context);

	if (dead_end) {
		std::cout << "Initial state is a dead end, no solution" << std::endl;
		assert(heuristic->dead_ends_are_reliable());
		utils::exit_with(utils::ExitCode::SEARCH_UNSOLVABLE);
	}

	assert(heuristic->is_last_bsg_valid_for_state(current_eval_context.get_state()));
	if (!solved && check_relaxed_plans && is_valid_plan_in_the_original_task(heuristic->get_last_bsg(), current_eval_context.get_state().get_values(), *g_root_task()) && heuristic->get_last_bsg().get_real_cost() <= bound) {
		solved = true;
		set_solution(heuristic->get_last_relaxed_plan(), current_eval_context.get_state());
	}
}

auto EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch::get_applicable_ops(const GlobalState &state) -> std::vector<const GlobalOperator *> {
	auto ops = std::vector<const GlobalOperator *>();
	g_successor_generator->generate_applicable_ops(state, ops);
	statistics.inc_expanded();
	statistics.inc_generated_ops(ops.size());
	if (randomize_successors)
		std::shuffle(std::begin(ops), std::end(ops), urng);
	return ops;
}

void EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch::expand(SearchNode &node, SearchSpace &current_search_space, StateRegistry &current_state_registry) {
	node.close();
	if (node.get_g() >= bfs_bound) {
		bfs_cutoff = true;
		return;
	}
	const auto state = node.get_state();
	assert(!state_registry.lookup_state(state) || !search_space.get_node(*state_registry.lookup_state(state)).is_dead_end());
	const auto applicable_ops = get_applicable_ops(state);
	for (const auto *op : applicable_ops) {
		auto succ_g = node.get_g() + get_adjusted_cost(*op);
		if (current_real_g + node.get_real_g() + op->get_cost() > bound)
			continue;
		auto succ = current_state_registry.get_successor_state(state, *op);
		statistics.inc_generated();
		if (excluded_states.find(succ.get_id()) != std::end(excluded_states)) {
			bfs_cutoff = true;
			continue;
		}
		const auto global_succ = state_registry.lookup_state(succ);
		if (global_succ && search_space.get_node(*global_succ).is_dead_end())
			continue;
		auto succ_node = current_search_space.get_node(succ);
		if (!succ_node.is_new()) {
			if (succ_node.get_g() > succ_g)
				succ_node.update_parent(node, op);
			continue;
		}
		succ_node.open(node, op);
		open_list.emplace(open_list_entry{succ.get_id(), succ_g, subgoal_heuristic.compute_result(state.get_id(), succ)});
	}
}

auto EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch::step() -> SearchStatus {
	search_progress.check_progress(current_eval_context);

	if (solved || check_goal_and_set_plan(current_eval_context.get_state()))
		return SOLVED;
	if (current_eval_context.is_heuristic_infinite(heuristic))
		return handle_safe_dead_end();
	if (heuristic->get_counter_growth() > max_growth)
		return FAILED;
	bfs_cutoff = false;
	auto local_state_registry = StateRegistry(*g_root_task(), *g_state_packer, *g_axiom_evaluator, g_initial_state_data);
	const auto local_state = local_state_registry.import_state(current_eval_context.get_state());
	auto local_search_space = SearchSpace(local_state_registry, cost_type);
	auto node = local_search_space.get_node(local_state);
	node.open_initial();

	if (!heuristic->is_last_bsg_valid_for_state(current_eval_context.get_state())) {
		current_eval_context = EvaluationContext(current_eval_context.get_state(), &statistics);
		if (current_eval_context.is_heuristic_infinite(heuristic))
			return handle_safe_dead_end();
	}
	assert(heuristic->is_last_bsg_valid_for_state(current_eval_context.get_state()));
	// evaluate_if_neccessary(current_eval_context);
	auto conjunctions = std::vector<Conjunction *>();
	for (auto &bsg_node : heuristic->get_last_bsg().nodes)
		conjunctions.insert(std::end(conjunctions), std::begin(bsg_node.supported_conjunctions), std::end(bsg_node.supported_conjunctions));
	std::sort(std::begin(conjunctions), std::end(conjunctions));
	conjunctions.erase(std::unique(std::begin(conjunctions), std::end(conjunctions)), std::end(conjunctions));
	subgoal_heuristic.initialize(std::move(conjunctions), local_state.get_id());

	expand(node, local_search_space, local_state_registry);
	return ehc(local_search_space, local_state_registry);
}

SearchStatus EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch::ehc(SearchSpace &current_search_space, StateRegistry &current_state_registry) {
	++ehcc_statistics.num_ehc_phases;

	novelty_heuristic->reset();
	if (current_eval_context.get_heuristic_value(novelty_heuristic) == 1) {
		assert(false && "should be unreachable");
		utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
	}
	auto num_expansions_this_ehc_phase = 0;

	auto best_state_id = StateID::no_state;
	auto best_state_h = std::numeric_limits<int>::max();

	while (!open_list.empty()) {
		if (is_time_expired())
			return TIMEOUT;

		check_timer_and_print_intermediate_statistics(*heuristic);
		auto [state_id, g, h] = open_list.top();
		open_list.pop();
		const auto state = current_state_registry.lookup_state(state_id);
		auto eval_context = EvaluationContext(state, &statistics);
		auto node = current_search_space.get_node(state);
		assert(node.is_open());

		auto is_novel = [this, &eval_context]() {
			ehcc_statistics.novelty_timer.resume();
			const auto novel = eval_context.get_heuristic_value(novelty_heuristic) == 0;
			ehcc_statistics.novelty_timer.stop();
			if (!novel) {
				// state will be pruned by novelty pruning
				bfs_cutoff = true;
			}
			return novel;
		};

		assert(novelty_heuristic->is_basic());
		if (!is_novel())
			continue;

		++num_expansions_this_ehc_phase;

		//const auto h = subgoal_heuristic.compute_result(node.get_parent_state_id(), state);
		if (h < best_state_h) {
			best_state_h = h;
			best_state_id = state.get_id();
		}
		expand(node, current_search_space, current_state_registry);
	}

	if (best_state_id != StateID::no_state) {
		auto state = current_state_registry.lookup_state(best_state_id);
		auto global_state = state_registry.import_state(state);
		auto eval_context = EvaluationContext(global_state, &statistics);
		if (!eval_context.is_heuristic_infinite(heuristic) && eval_context.get_heuristic_value(heuristic) < current_eval_context.get_heuristic_value(heuristic)) {
			learning_stagnation_counter = 0;
			excluded_states.clear();

			auto local_plan = Plan();
			current_search_space.trace_path(state, local_plan);

			auto current_state = current_eval_context.get_state();
			for (const auto op : local_plan) {
				auto current_parent_node = search_space.get_node(current_state);
				auto successor = state_registry.get_successor_state(current_state, *op);
				auto successor_node = search_space.get_node(successor);
				if (successor_node.is_new())
					successor_node.open(current_parent_node, op);
				current_state = successor;
				current_real_g = successor_node.get_real_g();
			}

			current_eval_context = eval_context;
			open_list = create_open_list(weight);

			return IN_PROGRESS;
		} else {
			return escape_local_minimum(-1);
		}
	}

	if (!bfs_cutoff)
		// state is considered to be a safe dead end
		return handle_safe_dead_end();
	if (no_learning)
		return handle_search_space_exhaustion();
	ehcc_statistics.num_expansions_in_ehc_phases_with_refinement += num_expansions_this_ehc_phase;
	return escape_local_minimum(-1);
}

auto EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch::evaluate_if_neccessary(EvaluationContext &eval_context, const GlobalState &parent_state, const GlobalOperator &last_op) -> int {
	reach_state(parent_state, last_op, eval_context.get_state());
	const auto &result = eval_context.get_result(heuristic);
	return result.get_h_value();
}

auto EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch::evaluate_if_neccessary(EvaluationContext &eval_context) -> int {
	const auto &result = eval_context.get_result(heuristic);
	return result.get_h_value();
}

// attempt to escape local minimum by generating conjunctions
auto EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch::escape_local_minimum(int target_h) -> SearchStatus {
	++online_learning_statistics.num_learning_calls;
	ehcc_statistics.total_stagnation_count += learning_stagnation_counter;
	ehcc_statistics.max_stagnation_count = std::max(ehcc_statistics.max_stagnation_count, learning_stagnation_counter);

	if (learning_stagnation_counter > 0)
		++ehcc_statistics.num_no_better_state_after_learning;

	// don't check for learning stagnation if we are going to restart/backjump due to search space exhaustion
	if (!(!bfs_cutoff && search_space_exhaustion != SearchSpaceExhaustion::CONTINUE) && learning_stagnation_counter >= learning_stagnation_threshold) {
		// learning is called in the same phase/state as the last time, i.e. we could not find a better state after improving the heuristic
		if (!learning_stagnation_restart)
			excluded_states.clear();
		if (current_eval_context.get_state().get_id() != state_registry.get_initial_state().get_id())
			return learning_stagnation_restart ? restart() : restart_in_parent();
	}

	++learning_stagnation_counter;
	auto modified = false;

	do {
		if (is_time_expired())
			return TIMEOUT;

		current_eval_context = EvaluationContext(current_eval_context.get_state(), &statistics);
		auto learning_result = generate_conjunctions(*heuristic, ConjunctionGenerationStrategy::Event::LOCAL_MINIMUM, current_eval_context, true, bound - current_real_g);
		if (learning_result == ConjunctionGenerationStrategy::Result::SOLVED && current_real_g + heuristic->get_last_bsg().get_real_cost() <= bound)
			return SOLVED;

		if (learning_result == ConjunctionGenerationStrategy::Result::MODIFIED) {
			modified = true;
			current_solved_unmodified.clear();
		}

		check_timer_and_print_intermediate_statistics(*heuristic);

		if (learning_result == ConjunctionGenerationStrategy::Result::DEAD_END || current_eval_context.is_heuristic_infinite(heuristic)) {
			++ehcc_statistics.num_dead_ends_during_learning;
			return handle_safe_dead_end();
		}

		assert(learning_result == ConjunctionGenerationStrategy::Result::MODIFIED
			|| (learning_result == ConjunctionGenerationStrategy::Result::SOLVED && current_real_g + heuristic->get_last_bsg().get_real_cost() > bound));

		if (learning_result == ConjunctionGenerationStrategy::Result::SOLVED && !modified) {
			auto insertion_result = current_solved_unmodified.insert(current_eval_context.get_state().get_id());
			if (!insertion_result.second)
				// fail if we arrived in this state more than once without making progress
				return FAILED;
		}
		if (!bfs_cutoff && search_space_exhaustion != SearchSpaceExhaustion::CONTINUE)
			return handle_search_space_exhaustion();
		if (learning_result == ConjunctionGenerationStrategy::Result::SOLVED)
			break;
	} while (!(always_reevaluate || conjunctions_strategy->deletes_conjunctions() || heuristic->get_counter_growth() > max_growth) && current_eval_context.get_heuristic_value(heuristic) <= target_h);

	return IN_PROGRESS;
}

auto EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch::handle_safe_dead_end() -> SearchStatus {
	++ehcc_statistics.num_dead_ends;
	auto node = search_space.get_node(current_eval_context.get_state());
	node.mark_as_dead_end();
	statistics.inc_dead_ends();
	if (node.get_state_id() == state_registry.get_initial_state().get_id()) {
		std::cout << "Initial state is a dead end, no solution" << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_UNSOLVABLE);
	}
	if (restart_in_dead_ends)
		return restart();
	return escape_dead_end(node);
}

auto EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch::handle_search_space_exhaustion() -> SearchStatus {
	++ehcc_statistics.num_search_space_exhaustion;
	if (search_space_exhaustion == SearchSpaceExhaustion::CONTINUE)
		return no_learning ? FAILED : IN_PROGRESS;
	assert(search_space_exhaustion == SearchSpaceExhaustion::RESTART || search_space_exhaustion == SearchSpaceExhaustion::BACKJUMP);
	return search_space_exhaustion == SearchSpaceExhaustion::RESTART ? restart() : escape_potential_dead_end();
}

auto EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch::escape_dead_end(const SearchNode &node) -> SearchStatus {
	++ehcc_statistics.total_dead_end_backjump_length;
	assert(node.is_dead_end());
	learning_stagnation_counter = 0;
	const auto parent_id = node.get_parent_state_id();
	if (parent_id == StateID::no_state) {
		std::cout << "Initial state is a dead end, no solution" << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_UNSOLVABLE);
	}
	current_eval_context = EvaluationContext(state_registry.lookup_state(parent_id), &statistics);
	auto parent_node = search_space.get_node(current_eval_context.get_state());
	current_real_g = parent_node.get_real_g();
	auto h = evaluate_if_neccessary(current_eval_context);
	if (h == EvaluationResult::INFTY) {
		parent_node.mark_as_dead_end();
		statistics.inc_dead_ends();
		return escape_dead_end(parent_node);
	}
	return IN_PROGRESS;
}

auto EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch::escape_potential_dead_end() -> SearchStatus {
	auto node = search_space.get_node(current_eval_context.get_state());
	assert(!node.is_dead_end());
	learning_stagnation_counter = 0;
	const auto parent_id = node.get_parent_state_id();
	if (parent_id == StateID::no_state)
		return IN_PROGRESS;
	current_eval_context = EvaluationContext(state_registry.lookup_state(parent_id), &statistics);
	auto h = evaluate_if_neccessary(current_eval_context);
	if (h == EvaluationResult::INFTY) {
		node.mark_as_dead_end();
		statistics.inc_dead_ends();
		return handle_safe_dead_end();
	}
	current_real_g = search_space.get_node(current_eval_context.get_state()).get_real_g();
	return IN_PROGRESS;
}

auto EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch::restart() -> SearchStatus {
	learning_stagnation_counter = 0;
	current_eval_context = EvaluationContext(state_registry.get_initial_state(), &statistics);
	current_real_g = 0;
	auto h = evaluate_if_neccessary(current_eval_context);
	if (h == EvaluationResult::INFTY) {
		std::cout << "Initial state is a dead end, no solution" << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_UNSOLVABLE);
	}
	return IN_PROGRESS;
}

auto EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch::restart_in_parent() -> SearchStatus {
	assert(current_eval_context.get_state().get_id() != state_registry.get_initial_state().get_id());
	learning_stagnation_counter = 0;
	auto node = search_space.get_node(current_eval_context.get_state());
	excluded_states.insert(node.get_state_id());
	const auto parent_id = node.get_parent_state_id();
	assert(parent_id != StateID::no_state);
	const auto &parent_state = state_registry.lookup_state(parent_id);
	current_eval_context = EvaluationContext(parent_state, &statistics);
	auto h = evaluate_if_neccessary(current_eval_context);
	if (h == EvaluationResult::INFTY)
		return handle_safe_dead_end();
	current_real_g = search_space.get_node(current_eval_context.get_state()).get_real_g();
	return IN_PROGRESS;
}

void EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch::print_ehcc_statistics() const {
	std::cout << "EHC Phases: " << ehcc_statistics.num_ehc_phases << std::endl;
	std::cout << "Average expansions per EHC phase: " << ehcc_statistics.get_avg_expansions_per_ehc_phase(statistics.get_expanded()) << std::endl;
	std::cout << "Average expansions per EHC phase with refinement: " << ehcc_statistics.get_avg_expansions_per_ehc_phase_with_refinement(online_learning_statistics.num_learning_calls) << std::endl;
	std::cout << "Dead ends during learning: " << ehcc_statistics.num_dead_ends_during_learning << std::endl;
	std::cout << "Dead ends during learning or backjumping: " << ehcc_statistics.num_dead_ends << std::endl;
	std::cout << "No better state after learning: " << ehcc_statistics.num_no_better_state_after_learning << std::endl;
	std::cout << "Max learning stagnation count: " << ehcc_statistics.max_stagnation_count << std::endl;
	std::cout << "Average learning stagnation count: " << ehcc_statistics.get_avg_stagnation_count(online_learning_statistics.num_learning_calls) << std::endl;
	std::cout << "Exhausted search space: " << ehcc_statistics.num_search_space_exhaustion << std::endl;
	std::cout << "Average dead end backjump length: " << (ehcc_statistics.num_dead_ends != 0 ? ehcc_statistics.total_dead_end_backjump_length / static_cast<double>(ehcc_statistics.num_dead_ends) : 0.0) << std::endl;
	std::cout << "Total time spent on evaluating novelty: " << ehcc_statistics.novelty_timer() << std::endl;
}

void EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch::print_intermediate_statistics(const ConjunctionsHeuristic &heuristic) const {
	OnlineLearningSearchEngine::print_intermediate_statistics(heuristic);
	print_ehcc_statistics();
}


void EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch::print_statistics() const {
	statistics.print_detailed_statistics();
	print_intermediate_statistics(*heuristic);

	std::cout << "EHC phases: " << ehcc_statistics.num_ehc_phases << std::endl;
	if (ehcc_statistics.num_ehc_phases != 0)
		std::cout << "Average expansions per EHC phase: " << ehcc_statistics.get_avg_expansions_per_ehc_phase(statistics.get_expanded()) << std::endl;
}

static auto _parse(OptionParser &parser) -> SearchEngine * {
	parser.document_synopsis("Lazy enforced hill-climbing", "");
	parser.add_option<Heuristic *>("h", "heuristic used for search, must be of type ConjunctionsHeuristic");
	parser.add_option<Heuristic *>("novelty", "novelty heuristic used for pruning in BFS explorations, must be of type NoveltyHeuristic", "novelty(cache_estimates=false)");
	parser.add_option<bool>("no_learning", "exit instead of learning", "false");
	parser.add_option<bool>("restart_in_dead_ends", "Do a full restart if the current state is safely recognized as a dead end; "
		"otherwise just go back to the last state along the path that is not the current state and does not have heuristic value infinity. This can happen in two ways: "
		"1. Without learning, the entire local search space is exhausted around the current best state. "
		"2. The current best state might have heuristic value infinity after learning new conjunctions.", "true");
	parser.add_option<int>("bfs_bound", "lookahead bound for breadth first search", "infinity");
	parser.add_option<bool>("always_reevaluate", "always reevaluate the local minimum neighborhood after adding a conjunction", "false");
	parser.add_option<bool>("randomize_successors", "randomize successors before inserting them into the open list", "true");
	// random seed (only relevant with enabled successor randomization)
	parser.add_option<int>("seed", "Random seed (for successor randomization). If this is set to -1, an arbitrary seed is used.", "-1");
	parser.add_option<int>("learning_stagnation_threshold", "how often learning must be repeated in the same state to trigger a learning stagnation", "infinity", Bounds("0", "infinity"));
	parser.add_option<bool>("learning_stagnation_restart", "do a full restart when learning stagnation is triggered", "false");
	parser.add_enum_option("search_space_exhaustion", {"CONTINUE", "RESTART", "BACKJUMP"},
		"search behavior whenever the BFS search space is fully explored but not caused by cutting off at k or the state on which learning is called is recognized as a dead end", "RESTART",
		{"With learning, continue with the next BFS phase at the current state after learning one conjunction. Without learning, search fails in this case.",
		 "Restart search from the initial state.",
		 "Go back along the current path to the most recent state that does not have an infinite heuristic value.",
		 "Like backjump, but mark this state as an unsafe dead end. Unsafe dead ends are handled like dead ends (i.e. never explored), but may cause the search to fail although there is a solution, in which case the unsafe dead ends are reset and search is restarted from the initial state."});
	parser.add_option<double>("max_growth", "fail when reaching this growth bound in the heuristic", "infinity");
	parser.add_enum_option("subgoal_aggregation_method", {"COUNT", "SUM", "MAX"}, "achieved relaxed plan subgoals aggregation method", "COUNT");
	parser.add_option<bool>("path_dependent_subgoals", "consider all subgoals reached on the path to each node", "true");
	parser.add_option<int>("w", "heuristic weight", "0", Bounds("0", "infinity"));
	OnlineLearningSearchEngine::add_options_to_parser(parser);
	SearchEngine::add_options_to_parser(parser);
	auto opts = parser.parse();

	if (parser.help_mode() || parser.dry_run())
		return nullptr;
	return new EnforcedHillClimbingNoveltyRelaxedSubgoalsSearch(opts);
}

static Plugin<SearchEngine> _plugin("ehc_cnsg", _parse);
}
