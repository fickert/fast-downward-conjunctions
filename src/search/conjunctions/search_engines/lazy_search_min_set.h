#ifndef CONJUNCTIONS_LAZY_SEARCH_MIN_SET_H
#define CONJUNCTIONS_LAZY_SEARCH_MIN_SET_H

#include "online_learning_search_engine.h"

#include "../../open_lists/open_list.h"
#include "../options/option_parser.h"
#include "../../search_engines/search_common.h"

#include <memory>
#include <vector>


class GlobalOperator;
class Heuristic;

namespace options {
class Options;
}

namespace conjunctions {

namespace detail {
template<bool generalized, bool individual_min>
class LazySearchMinSetBase;

template<>
class LazySearchMinSetBase<false, false> {
protected:
	LazySearchMinSetBase(const options::Options &opts);
};

template<>
class LazySearchMinSetBase<false, true> {
protected:
	LazySearchMinSetBase(const options::Options &opts);

	// state -> successors
	std::unordered_map<StateID, std::unordered_set<StateID>> open_successors;
	StateID learning_state;
};

template<>
class LazySearchMinSetBase<true, false> {
protected:
	LazySearchMinSetBase(const options::Options &opts);

	// successor -> distance
	std::unordered_map<StateID, int> open_successors;
	std::unordered_map<StateID, int> all_successors;

	const int k;
};

template<>
class LazySearchMinSetBase<true, true> {
protected:
	LazySearchMinSetBase(const options::Options &opts);

	// state -> successor -> distance
	std::unordered_map<StateID, std::unordered_map<StateID, int>> open_successors;
	std::unordered_map<StateID, std::unordered_map<StateID, int>> all_successors;
	StateID learning_state;

	const int k;
};
}

template<bool generalized = false, bool individual_min = false>
class LazySearchMinSet : public OnlineLearningSearchEngine, public detail::LazySearchMinSetBase<generalized, individual_min> {
protected:
	std::unique_ptr<EdgeOpenList> open_list;

	// Search behavior parameters
	const bool randomize_successors;

	// random number generator for successor randomization
	std::mt19937 urng;

	enum class PreferredUsage {
		ALTERNATING,
		PRUNE_BY_PREFERRED,
		RANK_PREFERRED_FIRST
	} const preferred_usage;

	std::vector<Heuristic *> heuristics;
	std::vector<Heuristic *> preferred_operator_heuristics;
	std::vector<ConjunctionsHeuristic *> conjunctions_heuristics;

	GlobalState current_state;
	StateID current_predecessor_id;
	const GlobalOperator *current_operator;
	int current_g;
	int current_real_g;
	EvaluationContext current_eval_context;

	const std::shared_ptr<ConjunctionGenerationStrategy> strategy;

	bool solved;

	void initialize() override;
	auto step() -> SearchStatus override;

	void generate_successors();
	virtual auto fetch_next_state() -> SearchStatus;

	void reward_progress();

	void get_successor_operators(std::vector<const GlobalOperator *> &ops);

	void print_checkpoint_line(int g) const;

	void print_min_set_statistics() const;
	void print_intermediate_statistics(const ConjunctionsHeuristic &) const override;

	auto update_min_set() -> bool;
	auto do_learning() -> bool;
	auto get_learning_target() const -> int;
	void remove_current_state_from_open_successors(const SearchNode &);

	template<typename = std::enable_if<!generalized && individual_min>>
	void add_new_successors(const std::vector<StateID> &);

	template<typename = std::enable_if<generalized>>
	void add_new_successors(const std::vector<StateID> &, const std::vector<StateID> &);

	auto get_learning_states() const -> std::conditional_t<individual_min, std::vector<StateID>, const std::vector<StateID> &>;
	void notify_escape_local_minimum();
	virtual auto escape_local_minimum(int target_h) -> SearchStatus = 0;

	void reset_min();

	PerStateInformation<int> h_cache;
	std::vector<StateID> min_states;
	int min_h;

	struct MinSetStatistics {
		MinSetStatistics(const OnlineLearningStatistics &online_learning_statistics) :
			online_learning_statistics(online_learning_statistics),
			total_min_states_size(0),
			max_min_states_size(0) {}

		const OnlineLearningStatistics &online_learning_statistics;
		int total_min_states_size;
		int max_min_states_size;

		auto get_avg_min_states_size() const -> double {
			return online_learning_statistics.num_learning_calls == 0 ? 0. : total_min_states_size / static_cast<double>(online_learning_statistics.num_learning_calls);
		}
	} min_set_statistics;

	enum class LearningTarget {
		PLUS_ONE,
		NEXT_BEST_IN_OPEN_LIST,
		ONE_CONJUNCTION
	} const learning_target;

public:
	explicit LazySearchMinSet(const options::Options &opts);
	virtual ~LazySearchMinSet() = default;

	void set_pref_operator_heuristics(std::vector<Heuristic *> &heur);

	void print_statistics() const override;

	static void _parse_base(options::OptionParser &parser);

	template<typename LazySearchMinSetDerived, typename = std::enable_if<std::is_base_of<LazySearchMinSet, LazySearchMinSetDerived>::value>>
	static auto _set_open_list_and_create_search_engine(options::OptionParser &parser) -> SearchEngine * {
		auto opts = parser.parse();
		if (parser.help_mode() || parser.dry_run())
			return nullptr;
		auto evals = opts.get_list<ScalarEvaluator *>("evals");
		auto preferred = opts.get_list<Heuristic *>("preferred");
		if (PreferredUsage(opts.get_enum("preferred_usage")) == PreferredUsage::PRUNE_BY_PREFERRED
			&& evals.size() == 1 && preferred.size() == 1 && evals.front() == preferred.front()) {
			opts.set("open", search_common::create_standard_scalar_open_list_factory(evals.front(), false));
		} else {
			opts.set("open", search_common::create_greedy_open_list_factory(opts));
		}
		auto engine = new LazySearchMinSetDerived(opts);
		auto preferred_list = opts.get_list<Heuristic *>("preferred");
		engine->set_pref_operator_heuristics(preferred_list);
		return engine;
	}
};
}

#endif
