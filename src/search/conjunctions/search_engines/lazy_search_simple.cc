#include "lazy_search_simple.h"

#include "../../open_lists/open_list_factory.h"
#include "../../task_utils/successor_generator.h"
#include "../../options/option_parser.h"
#include "../../options/plugin.h"
#include "../../search_engines/search_common.h"
#include "../utils.h"

namespace conjunctions {
static const int DEFAULT_LAZY_BOOST = 1000;

auto LazySearchSimple::get_detect_local_minimum(const options::Options &opts) -> std::unique_ptr<DetectLocalMinimum> {
	switch (opts.get_enum("local_min_detection")) {
	case 1:
		return std::make_unique<DetectLocalMinimumByTime>(opts.get<double>("local_min_detection_value"));
	case 2:
		return std::make_unique<DetectLocalMinimumByStates>(static_cast<int>(opts.get<double>("local_min_detection_value")));
	default:
		assert(opts.get_enum("local_min_detection") <= 2 && "unknown local minimum detection method");
		return std::make_unique<DetectLocalMinimum>();
	}
}

LazySearchSimple::LazySearchSimple(const options::Options &opts) :
	OnlineLearningSearchEngine(opts),
	open_list(opts.get<std::shared_ptr<OpenListFactory>>("open")->create_edge_open_list()),
	randomize_successors(opts.get<bool>("randomize_successors")),
	preferred_successors_first(opts.get<bool>("preferred_successors_first")),
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
	detect_local_minimum(get_detect_local_minimum(opts)),
	rp_sequence(RPSequence(opts.get_enum("rp_sequence"))),
	prioritize_rp_sequence(opts.get<bool>("prioritize_rp_sequence")) {}

void LazySearchSimple::set_pref_operator_heuristics(std::vector<Heuristic *> &heur) {
	preferred_operator_heuristics = heur;
}

void LazySearchSimple::initialize() {
	utils::Timer initialization_timer;
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
		solved |= (generate_conjunctions(*conjunctions_heuristic, ConjunctionGenerationStrategy::Event::INITIALIZATION, current_eval_context, true, bound) == ConjunctionGenerationStrategy::Result::SOLVED
			&& conjunctions_heuristic->get_last_bsg().get_real_cost() <= bound);
		conjunctions_heuristic->print_statistics();
	}

	std::cout << "Finished initialization, t = " << initialization_timer << std::endl;
	print_intermediate_statistics(*conjunctions_heuristics.front());

	start_search_timer();

	if (!conjunctions_heuristics.front()->is_last_bsg_valid_for_state(current_eval_context.get_state()))
		current_eval_context = EvaluationContext(current_eval_context.get_state(), &statistics);
}

void LazySearchSimple::get_successor_operators(std::vector<const GlobalOperator *> &ops) {
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

void LazySearchSimple::generate_successors() {
	const auto &bsg = conjunctions_heuristics.front()->get_last_bsg();
	auto enqueue_state_from_bsg_sequence = [this, &bsg](auto length) {
		assert(conjunctions_heuristics.front()->is_last_bsg_valid_for_state(current_state));
		if (length == bsg.nodes.size()) {
			set_solution(conjunctions_heuristics.front()->get_last_relaxed_plan(), current_state);
			solved = true;
			return;
		}
		auto resulting_g = current_g;
		auto resulting_parent_state = current_state;
		for (auto bsg_it = std::rbegin(bsg.nodes); bsg_it != std::rbegin(bsg.nodes) + length - 1; ++bsg_it) {
			const auto &op = *bsg_it->action->op->get_global_operator();
			auto parent_node = search_space.get_node(resulting_parent_state);
			assert(!parent_node.is_new() && !parent_node.is_dead_end());
			resulting_parent_state = state_registry.get_successor_state(resulting_parent_state, op);
			resulting_g += get_adjusted_cost(op);
			auto node = search_space.get_node(resulting_parent_state);
			if (node.is_new())
				node.open(parent_node, &op);
			else if (node.is_open() && node.get_real_g() > resulting_g)
				node.reopen(parent_node, &op);
		}
		const auto &last_op = *(std::rbegin(bsg.nodes) + length - 1)->action->op->get_global_operator();
		resulting_g += get_adjusted_cost(last_op);
		auto new_eval_context = EvaluationContext(current_eval_context.get_cache(), resulting_g, true, nullptr);
		open_list->insert(new_eval_context, {resulting_parent_state.get_id(), &last_op});
	};

	auto rp_sequence_length = 0u;
	switch (rp_sequence) {
	case RPSequence::NONE:
		break;
	case RPSequence::ALL:
		rp_sequence_length = get_applicable_sequence_length(bsg, current_state.get_values(), *g_root_task());
		break;
	case RPSequence::ALL_INTENDED:
		rp_sequence_length = get_applicable_sequence_length(bsg, current_state.get_values(), *g_root_task());
		break;
	case RPSequence::FIRST_DELETER:
		rp_sequence_length = get_applicable_sequence_length(bsg, current_state.get_values(), *g_root_task());
		break;
	default:
		std::cerr << "Unknown PO sequence option: " << static_cast<int>(rp_sequence) << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
	}

	auto operators = std::vector<const GlobalOperator *>();
	get_successor_operators(operators);
	statistics.inc_generated(operators.size());

	auto rp_sequence_index = static_cast<std::size_t>(prioritize_rp_sequence ? 0 : g_rng()->operator()(operators.size()) + 1);

	for (const auto op : operators) {
		if (rp_sequence_length > 1u && rp_sequence_index < operators.size() && op == operators[rp_sequence_index])
			enqueue_state_from_bsg_sequence(rp_sequence_length);
		auto new_g = current_g + get_adjusted_cost(*op);
		auto new_real_g = current_real_g + op->get_cost();
		auto is_preferred = op->is_marked();
		if (is_preferred)
			op->unmark();
		if (new_real_g <= bound) {
			auto new_eval_context = EvaluationContext(current_eval_context.get_cache(), new_g, is_preferred, nullptr);
			open_list->insert(new_eval_context, std::make_pair(current_state.get_id(), op));
		}
	}
	if (rp_sequence_length > 1u && rp_sequence_index == operators.size())
		enqueue_state_from_bsg_sequence(rp_sequence_length);
}

SearchStatus LazySearchSimple::fetch_next_state() {
	if (open_list->empty()) {
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

SearchStatus LazySearchSimple::step() {
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

	if (node.is_new() || (rp_sequence != RPSequence::NONE && node.is_open())) {
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
		for (auto conjunctions_heuristic : conjunctions_heuristics)
			check_timer_and_print_intermediate_statistics(*conjunctions_heuristic);

		statistics.inc_evaluated_states();
		if (!open_list->is_dead_end(current_eval_context)) {
			if (node.is_new()) {
				if (current_predecessor_id == StateID::no_state) {
					node.open_initial();
					if (search_progress.check_progress(current_eval_context))
						print_checkpoint_line(current_g);
				}
				else {
					node.open(parent_node, current_operator);
				}
			} else {
				assert(rp_sequence != RPSequence::NONE);
			}
			assert(node.is_open());

			assert(conjunctions_heuristics.front()->is_last_bsg_valid_for_state(current_state));
			assert(current_real_g == node.get_real_g());
			if (check_relaxed_plans
				&& is_valid_plan_in_the_original_task(conjunctions_heuristics.front()->get_last_bsg(), current_state.get_values(), *g_root_task())
				&& current_real_g + conjunctions_heuristics.front()->get_last_bsg().get_real_cost() <= bound) {
				set_solution(conjunctions_heuristics.front()->get_last_relaxed_plan(), current_state);
				return SOLVED;
			}

			// generate conjunctions according to the selected strategy for this step
			for (auto conjunctions_heuristic : conjunctions_heuristics) {
				auto result = generate_conjunctions(*conjunctions_heuristic, ConjunctionGenerationStrategy::Event::STEP, current_eval_context, true, bound - current_real_g);
				if (result == ConjunctionGenerationStrategy::Result::SOLVED && current_real_g + conjunctions_heuristics.front()->get_last_bsg().get_real_cost() <= bound)
					return SOLVED;
				if (result == ConjunctionGenerationStrategy::Result::DEAD_END) {
					node.mark_as_dead_end();
					statistics.inc_dead_ends();
					return fetch_next_state();
				}
			}

			// generate conjunctions according to the selected strategy if this is a local minimum
			auto improved = detect_local_minimum->get_best_h() > current_eval_context.get_heuristic_value(conjunctions_heuristics.front());
			detect_local_minimum->new_state(current_eval_context.get_heuristic_value(conjunctions_heuristics.front()));
			if ((*detect_local_minimum)()) {
				++online_learning_statistics.num_learning_calls;
				detect_local_minimum->reset();
				for (auto conjunctions_heuristic : conjunctions_heuristics) {
					auto result = generate_conjunctions(*conjunctions_heuristic, ConjunctionGenerationStrategy::Event::LOCAL_MINIMUM, current_eval_context, true, bound - current_real_g);
					if (result == ConjunctionGenerationStrategy::Result::SOLVED && current_real_g + conjunctions_heuristics.front()->get_last_bsg().get_real_cost() <= bound)
						return SOLVED;
					if (result == ConjunctionGenerationStrategy::Result::DEAD_END) {
						node.mark_as_dead_end();
						statistics.inc_dead_ends();
						return fetch_next_state();
					}
				}
			}

			node.close();
			if (check_goal_and_set_plan(current_state))
				return SOLVED;
			if (search_progress.check_progress(current_eval_context)) {
				print_checkpoint_line(current_g);
				reward_progress();
			}

			// if we are using additional search nodes from relaxed plan sequences, we need to recompute the heuristic after refinement
			if (rp_sequence != RPSequence::NONE && !conjunctions_heuristics.front()->is_last_bsg_valid_for_state(current_state)) {
				auto &cached_result = const_cast<HeuristicCache &>(current_eval_context.get_cache())[conjunctions_heuristics.front()];
				cached_result = EvaluationResult();
				if (current_eval_context.is_heuristic_infinite(conjunctions_heuristics.front())) {
					node.mark_as_dead_end();
					statistics.inc_dead_ends();
					return fetch_next_state();
				}
			}

			generate_successors();
			statistics.inc_expanded();
			if (improved) {
				// generate conjunctions according to the selected strategy after heuristic improvement
				for (auto conjunctions_heuristic : conjunctions_heuristics) {
					auto result = generate_conjunctions(*conjunctions_heuristic, ConjunctionGenerationStrategy::Event::NEW_BEST_H, current_eval_context, true, bound - current_real_g);
					if (result == ConjunctionGenerationStrategy::Result::SOLVED && current_real_g + conjunctions_heuristics.front()->get_last_bsg().get_real_cost() <= bound)
						return SOLVED;
					if (result == ConjunctionGenerationStrategy::Result::DEAD_END) {
						node.mark_as_dead_end();
						statistics.inc_dead_ends();
						return fetch_next_state();
					}
				}
			}
		} else {
			detect_local_minimum->dead_end();
			node.mark_as_dead_end();
			statistics.inc_dead_ends();
		}
	}
	return fetch_next_state();
}

void LazySearchSimple::reward_progress() {
	open_list->boost_preferred();
}

void LazySearchSimple::print_checkpoint_line(int g) const {
	std::cout << "[g=" << g << ", ";
	statistics.print_basic_statistics();
	std::cout << "]" << std::endl;
}

void LazySearchSimple::print_statistics() const {
	statistics.print_detailed_statistics();
	print_intermediate_statistics(*conjunctions_heuristics.front());
	search_space.print_statistics();
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

static SearchEngine *_parse(options::OptionParser &parser) {
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
	parser.add_enum_option("local_min_detection", {"NONE", "TIME", "STATES"},
		"Metric to detect local minima to be used for the conjunction generation: "
		"time elapsed or number of states since the currently best state was found.", "TIME");
	parser.add_option<double>("local_min_detection_value",
		"If the TIME (STATES) local minimum detection method is used, this is the amount "
		" of time (number of states) since the currently best state to consider it a local minimum.", "10");
	parser.add_enum_option("rp_sequence", {"NONE", "ALL", "ALL_INTENDED", "FIRST_DELETER"},
		"Use leading operator sequence from relaxed plan to generate additional open list entries.", "NONE");
	parser.add_option<bool>("prioritize_rp_sequence", "Insert search nodes generated by the relaxed plan sequence at the front of the open list.", "true");
	auto opts = parser.parse();

	LazySearchSimple *engine = nullptr;
	if (!parser.dry_run()) {
		opts.set("open", search_common::create_greedy_open_list_factory(opts));
		engine = new LazySearchSimple(opts);
		auto preferred_list = opts.get_list<Heuristic *>("preferred");
		engine->set_pref_operator_heuristics(preferred_list);
	}
	return engine;
}

static options::Plugin<SearchEngine> _plugin("lazy_greedy_c_simple", _parse);
}
