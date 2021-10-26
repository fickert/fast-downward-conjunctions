#include "gbfs_relaxed_subgoals_lookahead.h"

#include "../open_lists/open_list_factory.h"
#include "../task_utils/successor_generator.h"
#include "../options/option_parser.h"
#include "../options/plugin.h"
#include "search_common.h"
#include "../utils/rng.h"

namespace gbfs_rsl {
static const int DEFAULT_LAZY_BOOST = 1000;

auto LazySearchRelaxedSubgoalsLookahead::create_open_list(int w) -> open_list_t {
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

LazySearchRelaxedSubgoalsLookahead::LazySearchRelaxedSubgoalsLookahead(const options::Options &opts)
	: SearchEngine(opts),
	  open_list(opts.get<std::shared_ptr<OpenListFactory>>("open")->create_edge_open_list()),
	  reopen_closed_nodes(false),
	  randomize_successors(opts.get<bool>("randomize_successors")),
	  preferred_successors_first(opts.get<bool>("preferred_successors_first")),
	  current_state(state_registry.get_initial_state()),
	  current_predecessor_id(StateID::no_state),
	  current_operator(nullptr),
	  current_g(0),
	  current_real_g(0),
	  current_eval_context(current_state, 0, true, &statistics),
	  heuristic(opts.get<Heuristic *>("relaxed_plan_heuristic")),
	  novelty_heuristic(static_cast<novelty::NoveltyHeuristic *>(opts.get<Heuristic *>("novelty"))),
	  subgoal_heuristic(subgoal_count_heuristic::SubgoalHeuristic::SubgoalAggregationMethod(opts.get_enum("subgoal_aggregation_method")), opts.get<bool>("path_dependent_subgoals")),
	  use_base_heuristic(opts.get<bool>("use_base_heuristic")),
	  lookahead_weight(opts.get<int>("lookahead_weight")) {}

void LazySearchRelaxedSubgoalsLookahead::set_pref_operator_heuristics(std::vector<Heuristic *> &heur) {
	preferred_operator_heuristics = heur;
}

void LazySearchRelaxedSubgoalsLookahead::initialize() {
	utils::Timer initialization_timer;
	std::cout << "Conducting lazy best first search with online learning of conjunctions, (real) bound = " << bound << std::endl;
	std::cout << "Using lookahead heuristic weight " << lookahead_weight << std::endl;

	assert(open_list && "open list should have been set during _parse");
	std::set<Heuristic *> hset;
	open_list->get_involved_heuristics(hset);

	// Add heuristics that are used for preferred operators (in case they are
	// not also used in the open list).
	hset.insert(preferred_operator_heuristics.begin(),
				preferred_operator_heuristics.end());

	heuristics.assign(hset.begin(), hset.end());
	assert(!heuristics.empty());

	heuristic->initialize_if_necessary(state_registry);

	const auto &initial_state = state_registry.get_initial_state();
	for (auto heuristic : heuristics)
		heuristic->notify_initial_state(initial_state);

	std::cout << "Finished initialization, t = " << initialization_timer << std::endl;
}

void LazySearchRelaxedSubgoalsLookahead::get_successor_operators(std::vector<const GlobalOperator *> &ops) {
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
		g_rng()->shuffle(all_operators);
		// Note that preferred_operators can contain duplicates that are
		// only filtered out later, which gives operators "preferred
		// multiple times" a higher chance to be ordered early.
		g_rng()->shuffle(preferred_operators);
	}

	if (preferred_successors_first) {
		for (const auto op : preferred_operators) {
			if (!op->is_marked()) {
				ops.push_back(op);
				op->mark();
			}
		}

		for (const auto op : all_operators)
			if (!op->is_marked())
				ops.push_back(op);
	} else {
		for (const auto op : preferred_operators)
			if (!op->is_marked())
				op->mark();
		ops.swap(all_operators);
	}
}

void LazySearchRelaxedSubgoalsLookahead::lookahead_expand(SearchNode &node) {
	node.close();
	++lookahead_statistics.num_lookahead_expansions;
	const auto state = node.get_state();
	assert(!state_registry.lookup_state(state) || !search_space.get_node(*state_registry.lookup_state(state)).is_dead_end());
	auto applicable_ops = std::vector<const GlobalOperator *>();
	g_successor_generator->generate_applicable_ops(state, applicable_ops);

	for (const auto *op : applicable_ops) {
		const auto succ_g = node.get_g() + get_adjusted_cost(*op);
		if (current_real_g + node.get_real_g() + op->get_cost() > bound)
			continue;
		const auto succ = lookahead_state_registry->get_successor_state(state, *op);
		statistics.inc_generated();
		++lookahead_statistics.num_lookahead_generated;
		auto global_succ = state_registry.lookup_state(succ);
		if (global_succ && search_space.get_node(*global_succ).is_dead_end())
			continue;
		auto succ_node = lookahead_search_space->get_node(succ);
		if (!succ_node.is_new()) {
			if (succ_node.get_g() > succ_g)
				succ_node.update_parent(node, op);
			continue;
		}
		succ_node.open(node, op);
		if (!use_base_heuristic)
			lookahead_open_list.emplace(open_list_entry{succ.get_id(), succ_g, subgoal_heuristic.compute_result(state.get_id(), succ)});
		else {
			auto eval_context = EvaluationContext(succ, succ_g, true, nullptr);
			if (!eval_context.is_heuristic_infinite(heuristic)) {
				lookahead_open_list.emplace(open_list_entry{succ.get_id(), succ_g, eval_context.get_heuristic_value(heuristic)});
			} else {
				if (!global_succ)
					global_succ.emplace(state_registry.import_state(succ));
				search_space.get_node(*global_succ).mark_as_dead_end();
			}
		}
	}
}

auto LazySearchRelaxedSubgoalsLookahead::lookahead() -> std::pair<SearchStatus, StateID> {
	++lookahead_statistics.num_lookahead;
	lookahead_state_registry = std::make_unique<StateRegistry>(*g_root_task(), *g_state_packer, *g_axiom_evaluator, g_initial_state_data);
	lookahead_search_space = std::make_unique<SearchSpace>(*lookahead_state_registry, cost_type);
	lookahead_open_list = create_open_list(lookahead_weight);
	const auto lookahead_initial_state = lookahead_state_registry->import_state(current_state);
	if (!use_base_heuristic)
		subgoal_heuristic.initialize(heuristic->get_last_subgoals_and_costs(), lookahead_initial_state.get_id());
	auto initial_node = lookahead_search_space->get_node(lookahead_initial_state);
	initial_node.open_initial();
	lookahead_expand(initial_node);

	novelty_heuristic->reset();
	// evaluate novelty to mark currently true facts non-novel for other states
	if (current_eval_context.get_heuristic_value(novelty_heuristic) == 1) {
		assert(false && "should be unreachable");
		utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
	}

	auto best_state_id = StateID::no_state;
	auto best_state_h = std::numeric_limits<int>::max();

	while (!lookahead_open_list.empty()) {
		auto [state_id, g, h] = lookahead_open_list.top();
		lookahead_open_list.pop();
		const auto state = lookahead_state_registry->lookup_state(state_id);
		auto eval_context = EvaluationContext(state, &statistics);
		auto node = lookahead_search_space->get_node(state);
		assert(node.is_open());

		auto is_novel = [this, &eval_context]() {
			lookahead_statistics.novelty_timer.resume();
			const auto novel = eval_context.get_heuristic_value(novelty_heuristic) == 0;
			lookahead_statistics.novelty_timer.stop();
			return novel;
		};

		assert(novelty_heuristic->is_basic());
		if (!is_novel())
			continue;

		if (h < best_state_h) {
			if (h == 0)
				return {test_goal(state) ? SOLVED : IN_PROGRESS, state.get_id()};
			best_state_h = h;
			best_state_id = state.get_id();
		}
		lookahead_expand(node);
	}

	return {IN_PROGRESS, best_state_id};
}

void LazySearchRelaxedSubgoalsLookahead::generate_successors() {
	auto operators = std::vector<const GlobalOperator *>();
	get_successor_operators(operators);
	statistics.inc_generated(operators.size());

	for (const auto op : operators) {
		const auto new_g = current_g + get_adjusted_cost(*op);
		const auto new_real_g = current_real_g + op->get_cost();
		const auto is_preferred = op->is_marked();
		if (is_preferred)
			op->unmark();
		if (new_real_g <= bound) {
			auto new_eval_context = EvaluationContext(current_eval_context.get_cache(), new_g, is_preferred, nullptr);
			open_list->insert(new_eval_context, std::make_pair(current_state.get_id(), op));
		}
	}
}

SearchStatus LazySearchRelaxedSubgoalsLookahead::fetch_next_state() {
	if (next_evaluation_context) {
		current_eval_context = std::move(*next_evaluation_context);
		next_evaluation_context.reset();
		current_state = current_eval_context.get_state();
		current_predecessor_id = current_state.get_id();
		current_operator = nullptr;
		const auto current_node = search_space.get_node(current_state);
		current_g = current_node.get_g();
		current_real_g = current_node.get_real_g();
		return IN_PROGRESS;
	}

	if (open_list->empty()) {
		std::cout << "Completely explored state space -- no solution!" << std::endl;
		return FAILED;
	}

	const auto next = open_list->remove_min();

	current_predecessor_id = next.first;
	current_operator = next.second;
	assert(current_operator);
	const auto current_predecessor = state_registry.lookup_state(current_predecessor_id);
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

SearchStatus LazySearchRelaxedSubgoalsLookahead::step() {
	// Invariants:
	// - current_state is the next state for which we want to compute the heuristic.
	// - current_predecessor is a permanent pointer to the predecessor of that state.
	// - current_operator is the operator which leads to current_state from predecessor.
	// - current_g is the g value of the current state according to the cost_type
	// - current_real_g is the g value of the current state (using real costs)

	auto node = search_space.get_node(current_state);
	const auto reopen = reopen_closed_nodes && !node.is_new() &&
		!node.is_dead_end() && (current_g < node.get_g());

	if (node.is_new() || node.is_open() || reopen) {
		auto dummy_id = current_predecessor_id;
		// HACK! HACK! we do this because SearchNode has no default/copy constructor
		if (dummy_id == StateID::no_state) {
			const auto &initial_state = state_registry.get_initial_state();
			dummy_id = initial_state.get_id();
		}
		auto parent_state = state_registry.lookup_state(dummy_id);
		auto parent_node = search_space.get_node(parent_state);

		if (current_operator)
			for (auto *heuristic : heuristics)
				heuristic->notify_state_transition(parent_state, *current_operator, current_state);

		if (current_predecessor_id == StateID::no_state)
			print_initial_h_values(current_eval_context);

		statistics.inc_evaluated_states();
		if (!open_list->is_dead_end(current_eval_context)) {
			if (node.is_new()) {
				if (reopen) {
					node.reopen(parent_node, current_operator);
					statistics.inc_reopened();
				} else if (current_predecessor_id == StateID::no_state) {
					node.open_initial();
					if (search_progress.check_progress(current_eval_context))
						print_checkpoint_line(current_g);
				} else {
					node.open(parent_node, current_operator);
				}
			} else {
				if (current_operator && node.get_g() > current_g) {
					node.update_parent(parent_node, current_operator);
				} else {
					assert(current_g >= node.get_g());
					current_g = node.get_g();
					current_real_g = node.get_real_g();
				}
			}
			assert(current_g == node.get_g());
			assert(current_real_g == node.get_real_g());

			node.close();
			if (check_goal_and_set_plan(current_state))
				return SOLVED;
			if (search_progress.check_progress(current_eval_context)) {
				print_checkpoint_line(current_g);
				reward_progress();
			}

			if (heuristic->found_solution()) {
				auto plan = Plan();
				if (current_state.get_id() != state_registry.get_initial_state().get_id())
					search_space.trace_path(current_state, plan);
				const auto suffix = heuristic->get_solution();
				plan.insert(std::end(plan), std::begin(suffix), std::end(suffix));
				set_plan(plan);
				return SOLVED;
			}

			lookahead_statistics.lookahead_timer.resume();
			const auto [lookahead_status, lookahead_state_id] = lookahead();
			lookahead_statistics.lookahead_timer.stop();
			if (lookahead_state_id == StateID::no_state) {
				node.mark_as_dead_end();
				statistics.inc_dead_ends();
				return fetch_next_state();
			}
			const auto original_lookahead_state = lookahead_state_registry->lookup_state(lookahead_state_id);
			const auto lookahead_state = state_registry.import_state(original_lookahead_state);

			if (lookahead_status == SOLVED) {
				++lookahead_statistics.num_lookahead_is_selected;
				lookahead_statistics.notify_lookahead_is_new_or_open(current_eval_context.get_heuristic_value(heuristic));
				assert(test_goal(lookahead_state));
				auto lookahead_plan = Plan();
				lookahead_search_space->trace_path(original_lookahead_state, lookahead_plan);
				auto overall_plan = Plan();
				search_space.trace_path(current_state, overall_plan);
				overall_plan.insert(std::end(overall_plan), std::begin(lookahead_plan), std::end(lookahead_plan));
				set_plan(overall_plan);
				return SOLVED;
			}

			auto lookahead_node = lookahead_search_space->get_node(original_lookahead_state);
			auto lookahead_eval_context = EvaluationContext(lookahead_state, current_g + lookahead_node.get_g(), true, &statistics);

			auto global_lookahead_node = search_space.get_node(lookahead_state);

			if (global_lookahead_node.is_new() || global_lookahead_node.is_open()) {
				if (lookahead_eval_context.is_heuristic_infinite(heuristic)) {
					global_lookahead_node.mark_as_dead_end();
					statistics.inc_dead_ends();
					++lookahead_statistics.num_lookahead_is_dead_end;
				} else {
					lookahead_statistics.notify_lookahead_is_new_or_open(current_eval_context.get_heuristic_value(heuristic) - lookahead_eval_context.get_heuristic_value(heuristic));
					if (lookahead_eval_context.get_heuristic_value(heuristic) < current_eval_context.get_heuristic_value(heuristic)) {
						auto lookahead_plan = Plan();
						lookahead_search_space->trace_path(original_lookahead_state, lookahead_plan);

						auto new_g = current_g;
						auto new_real_g = current_real_g;
						assert(current_state.get_id() == current_eval_context.get_state().get_id());
						auto s = current_eval_context.get_state();
						for (const auto op : lookahead_plan) {
							auto current_parent_node = search_space.get_node(s);
							auto successor = state_registry.get_successor_state(s, *op);
							auto successor_node = search_space.get_node(successor);
							new_g += get_adjusted_cost(*op);
							new_real_g += op->get_cost();
							if (successor_node.is_new())
								successor_node.open(current_parent_node, op);
							else if (new_g < successor_node.get_g())
								successor_node.update_parent(current_parent_node, op);
							s = successor;
						}
						++lookahead_statistics.num_lookahead_is_selected;
						next_evaluation_context.emplace(std::move(lookahead_eval_context));
					}
				}
			} else if (global_lookahead_node.is_dead_end()) {
				++lookahead_statistics.num_lookahead_is_dead_end;
			} else {
				assert(global_lookahead_node.is_closed());
				++lookahead_statistics.num_lookahead_is_closed;
			}

			generate_successors();
			statistics.inc_expanded();
		} else {
			node.mark_as_dead_end();
			statistics.inc_dead_ends();
		}
	}
	return fetch_next_state();
}

void LazySearchRelaxedSubgoalsLookahead::reward_progress() {
	open_list->boost_preferred();
}

void LazySearchRelaxedSubgoalsLookahead::print_checkpoint_line(int g) const {
	std::cout << "[g=" << g << ", ";
	statistics.print_basic_statistics();
	std::cout << "]" << std::endl;
}

void LazySearchRelaxedSubgoalsLookahead::print_statistics() const {
	statistics.print_detailed_statistics();
	search_space.print_statistics();

	const auto print_lookahead_ratio_statistic = [this](auto value) {
		if (lookahead_statistics.num_lookahead > 0)
			std::cout << " (ratio: " << value / static_cast<double>(lookahead_statistics.num_lookahead) << ")";
		std::cout << std::endl;
	};

	std::cout << "Lookahead phases: " << lookahead_statistics.num_lookahead << std::endl;
	std::cout << "Lookahead expansions: " << lookahead_statistics.num_lookahead_expansions << std::endl;
	std::cout << "Lookahead generated states: " << lookahead_statistics.num_lookahead_generated << std::endl;
	std::cout << "Total time spent on lookahead: " << lookahead_statistics.lookahead_timer() << "s" << std::endl;
	std::cout << "Total time spent on evaluating novelty: " << lookahead_statistics.novelty_timer() << "s" << std::endl;
	std::cout << "Successful lookaheads: " << lookahead_statistics.num_lookahead_is_selected;
	print_lookahead_ratio_statistic(lookahead_statistics.num_lookahead_is_selected);
	std::cout << "Dead ends after lookahead: " << lookahead_statistics.num_lookahead_is_dead_end;
	print_lookahead_ratio_statistic(lookahead_statistics.num_lookahead_is_dead_end);
	std::cout << "Closed nodes after lookahead: " << lookahead_statistics.num_lookahead_is_closed;
	print_lookahead_ratio_statistic(lookahead_statistics.num_lookahead_is_closed);
	std::cout << "Average heuristic value improvement after lookahead: " << lookahead_statistics.average_heuristic_difference << std::endl;
}


static void _add_lookahead_options(options::OptionParser &parser) {
	parser.add_option<int>("lookahead_weight", "heuristic weight in the lookahead search", "infinity", options::Bounds("0", "infinity"));
	parser.add_option<Heuristic *>("novelty", "novelty heuristic used for pruning in BFS explorations, must be of type NoveltyHeuristic", "novelty(cache_estimates=false)");
	parser.add_enum_option("subgoal_aggregation_method", {"COUNT", "SUM", "MAX"}, "achieved relaxed plan subgoals aggregation method", "COUNT");
	parser.add_option<bool>("path_dependent_subgoals", "consider all subgoals reached on the path to each node", "true");
	parser.add_option<bool>("use_base_heuristic", "use base heuristic for lookahead instead of relaxed subgoal counting", "false");
}

static void _add_succ_order_options(options::OptionParser &parser) {
	std::vector<std::string> options;
	parser.add_option<bool>(
		"randomize_successors",
		"randomize the order in which successors are generated",
		"false");
	parser.add_option<bool>(
		"preferred_successors_first",
		"consider preferred operators first",
		"false");
	parser.document_note(
		"Successor ordering",
		"When using randomize_successors=true and "
		"preferred_successors_first=true, randomization happens before "
		"preferred operators are moved to the front.");
}

static SearchEngine *_parse_greedy(options::OptionParser &parser) {
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
	parser.add_option<bool>("reopen_closed",
		"reopen closed nodes", "false");
	parser.add_option<int>(
		"boost",
		"boost value for alternation queues that are restricted "
		"to preferred operator nodes",
		options::OptionParser::to_str(DEFAULT_LAZY_BOOST));
	_add_lookahead_options(parser);
	_add_succ_order_options(parser);
	SearchEngine::add_options_to_parser(parser);
	parser.add_option<Heuristic *>("relaxed_plan_heuristic", "relaxed-plan heuristic");
	auto opts = parser.parse();

	LazySearchRelaxedSubgoalsLookahead *engine = nullptr;
	if (!parser.dry_run()) {
		opts.set("open", search_common::create_greedy_open_list_factory(opts));
		engine = new LazySearchRelaxedSubgoalsLookahead(opts);
		auto preferred_list = opts.get_list<Heuristic *>("preferred");
		engine->set_pref_operator_heuristics(preferred_list);
	}
	return engine;
}

static options::Plugin<SearchEngine> _plugin_greedy("lazy_greedy_rsl_rainbow", _parse_greedy);
}
