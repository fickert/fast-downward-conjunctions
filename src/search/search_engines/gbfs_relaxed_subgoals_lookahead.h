#ifndef SEARCH_ENGINES_GBFS_RELAXED_SUBGOALS_LOOKAHEAD_H
#define SEARCH_ENGINES_GBFS_RELAXED_SUBGOALS_LOOKAHEAD_H

#include "../conjunctions/novelty_heuristic.h"
#include "../heuristics/subgoal_count_heuristic.h"
#include "../open_lists/open_list.h"
#include "../search_engine.h"
#include "../utils/timer.h"

#include <memory>
#include <queue>
#include <vector>
#include <optional>


class GlobalOperator;
class Heuristic;

namespace options {
class Options;
}

namespace gbfs_rsl {
class LazySearchRelaxedSubgoalsLookahead : public SearchEngine {
protected:
	std::unique_ptr<EdgeOpenList> open_list;

	// Search behavior parameters
	bool reopen_closed_nodes;
	bool randomize_successors;
	bool preferred_successors_first;

	std::vector<Heuristic *> heuristics;
	std::vector<Heuristic *> preferred_operator_heuristics;

	GlobalState current_state;
	StateID current_predecessor_id;
	const GlobalOperator *current_operator;
	int current_g;
	int current_real_g;
	EvaluationContext current_eval_context;

	std::optional<EvaluationContext> next_evaluation_context;

	Heuristic *heuristic;

	struct LookaheadStatistics {
		LookaheadStatistics()
			: num_lookahead_expansions(0),
			  num_lookahead_generated(0),
			  num_lookahead(0),
			  num_lookahead_is_selected(0),
			  num_lookahead_is_new_or_open(0),
			  num_lookahead_is_dead_end(0),
			  num_lookahead_is_closed(0),
			  average_heuristic_difference(0.) {
			lookahead_timer.stop();
			lookahead_timer.reset();
			novelty_timer.stop();
			novelty_timer.reset();
		}

		void notify_lookahead_is_new_or_open(int heuristic_difference) {
			++num_lookahead_is_new_or_open;
			average_heuristic_difference += (heuristic_difference - average_heuristic_difference) / num_lookahead_is_new_or_open;
		}

		int num_lookahead_expansions;
		int num_lookahead_generated;
		int num_lookahead;
		int num_lookahead_is_selected;
		int num_lookahead_is_new_or_open;
		int num_lookahead_is_dead_end;
		int num_lookahead_is_closed;
		double average_heuristic_difference;
		utils::Timer novelty_timer;
		utils::Timer lookahead_timer;
	} lookahead_statistics;

	novelty::NoveltyHeuristic *novelty_heuristic;
	subgoal_count_heuristic::SubgoalHeuristic subgoal_heuristic;
	const bool use_base_heuristic;

	void initialize() override;
	SearchStatus step() override;

	struct open_list_entry {
		StateID state_id;
		int g;
		int h;
	};

	const int lookahead_weight;

	using open_list_t = std::priority_queue<open_list_entry, std::vector<open_list_entry>, std::function<bool(const open_list_entry &, const open_list_entry &)>>;
	open_list_t lookahead_open_list;
	static auto create_open_list(int w) -> open_list_t;

	std::unique_ptr<StateRegistry> lookahead_state_registry;
	std::unique_ptr<SearchSpace> lookahead_search_space;

	void lookahead_expand(SearchNode &node);
	auto lookahead() -> std::pair<SearchStatus, StateID>;

	void generate_successors();
	SearchStatus fetch_next_state();

	void reward_progress();

	void get_successor_operators(std::vector<const GlobalOperator *> &ops);

	void print_checkpoint_line(int g) const;

public:
	explicit LazySearchRelaxedSubgoalsLookahead(const options::Options &opts);
	virtual ~LazySearchRelaxedSubgoalsLookahead() = default;

	void set_pref_operator_heuristics(std::vector<Heuristic *> &heur);

	void print_statistics() const override;
};

}

#endif
