#include "yahsp_lookahead.h"

#include <map>
#include <numeric>

#include "../../heuristic.h"
#include "../../global_operator.h"
#include "../../search_progress.h"
#include "../../search_space.h"
#include "../../options/options.h"
#include "../../options/option_parser.h"
#include "../../task_utils/successor_generator.h"
#include "../../evaluation_context.h"

YahspLookahead::YahspLookahead(const options::Options &opts)
	: do_repair(opts.get<bool>("do_repair")) {}

YahspLookahead::YahspLookahead(bool do_repair)
	: do_repair(do_repair) {}

auto YahspLookahead::lookahead(EvaluationContext &eval_context, Heuristic &heuristic, StateRegistry &state_registry,
	successor_generator::SuccessorGenerator &successor_generator) const -> std::pair<GlobalState, std::vector<const GlobalOperator *>> {

	auto plan = std::vector<const GlobalOperator *>();

	auto relaxed_plan = heuristic.get_last_relaxed_plan();
	assert(!relaxed_plan.empty());

	auto state = eval_context.get_state();
	auto loop = true;


	auto applicable_ops_state_id = StateID::no_state;
	auto applicable_ops = std::vector<const GlobalOperator *>();

	auto update_applicable_ops = [&state, &applicable_ops_state_id, &applicable_ops, &successor_generator]() {
		if (state.get_id() != applicable_ops_state_id) {
			applicable_ops.clear();
			successor_generator.generate_applicable_ops(state, applicable_ops);
			applicable_ops_state_id = state.get_id();
		}
	};

	while (loop) {
		loop = false;
		for (auto it = std::begin(relaxed_plan); it != std::end(relaxed_plan); ++it) {
			if ((**it).is_applicable(state)) {
				loop = true;
				state = state_registry.get_successor_state(state, **it);
				plan.push_back(*it);
				relaxed_plan.erase(it);
				break;
			}
		}
		if (!do_repair)
			continue;
		for (auto it1 = std::begin(relaxed_plan); !loop && it1 != std::end(relaxed_plan); ++it1) {
			for (auto it2 = std::begin(relaxed_plan); !loop && it2 != std::end(relaxed_plan); ++it2) {
				if (it1 == it2)
					continue;
				// find intersecting preconditions and effects
				auto effects = std::vector<FactPair>();
				effects.reserve((**it1).get_effects().size());
				std::transform(std::begin((**it1).get_effects()), std::end((**it1).get_effects()), std::back_inserter(effects),
					[](const auto &effect) -> FactPair { return { effect.var, effect.val }; });
				std::sort(std::begin(effects), std::end(effects));

				auto preconditions = std::vector<FactPair>();
				preconditions.reserve((**it2).get_preconditions().size());
				std::transform(std::begin((**it2).get_preconditions()), std::end((**it2).get_preconditions()), std::back_inserter(preconditions),
					[](const auto &precondition) -> FactPair { return { precondition.var, precondition.val }; });
				std::sort(std::begin(preconditions), std::end(preconditions));

				auto intersection = std::vector<FactPair>();
				std::set_intersection(std::begin(preconditions), std::end(preconditions), std::begin(effects), std::end(effects), std::back_inserter(intersection));

				if (intersection.empty())
					continue;

				update_applicable_ops();
				auto candidates = applicable_ops;
				candidates.erase(std::remove_if(std::begin(candidates), std::end(candidates), [&intersection](const auto *op) {
					return std::none_of(std::begin(op->get_effects()), std::end(op->get_effects()), [&intersection](const auto &effect) {
						return std::binary_search(std::begin(intersection), std::end(intersection), FactPair(effect.var, effect.val));
					});
				}), std::end(candidates));

				if (candidates.empty())
					continue;

				loop = true;
				auto min_cost = std::numeric_limits<int>::max();
				auto min_cost_op = static_cast<const GlobalOperator *>(nullptr);

				for (const auto *op : candidates) {
					const auto cost = std::accumulate(std::begin(op->get_preconditions()), std::end(op->get_preconditions()), 0, [&heuristic](const auto cost, const auto &precondition) {
						return cost + heuristic.get_cost({ precondition.var, precondition.val });
					});
					if (cost < min_cost) {
						min_cost = cost;
						min_cost_op = op;
					}
				}

				*it1 = min_cost_op;
			}
		}
	}
	assert(!plan.empty());
	return {state, std::move(plan)};
}

void YahspLookahead::add_options_to_parser(options::OptionParser &parser) {
	parser.add_option<bool>("do_repair", "apply repair on lookahead plans", "true");
}
