#ifndef SEARCH_ENGINES_GBFS_YAHSP_LOOKAHEAD_H
#define SEARCH_ENGINES_GBFS_YAHSP_LOOKAHEAD_H

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
class YahspLookahead;

namespace options {
class Options;
}

namespace gbfs_yahsp {
class LazySearchYahspLookahead : public SearchEngine {
protected:
	std::unique_ptr<EdgeOpenList> open_list;

	// Search behavior parameters
	bool reopen_closed_nodes;
	bool randomize_successors;
	bool preferred_successors_first;

	std::vector<Heuristic *> heuristics;
	std::vector<Heuristic *> preferred_operator_heuristics;

	std::unique_ptr<YahspLookahead> yahsp_lookahead;

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
			: num_lookahead(0),
			  num_lookahead_is_selected(0),
			  num_lookahead_is_new_or_open(0),
			  num_lookahead_is_dead_end(0),
			  num_lookahead_is_closed(0),
			  average_heuristic_difference(0.) {
			lookahead_timer.stop();
			lookahead_timer.reset();
		}

		void notify_lookahead_is_new_or_open(int heuristic_difference) {
			++num_lookahead_is_new_or_open;
			average_heuristic_difference += (heuristic_difference - average_heuristic_difference) / num_lookahead_is_new_or_open;
		}

		int num_lookahead;
		int num_lookahead_is_selected;
		int num_lookahead_is_new_or_open;
		int num_lookahead_is_dead_end;
		int num_lookahead_is_closed;
		double average_heuristic_difference;
		utils::Timer lookahead_timer;
	} lookahead_statistics;

	void initialize() override;
	SearchStatus step() override;

	struct open_list_entry {
		StateID state_id;
		int g;
		int h;
	};

	void generate_successors();
	SearchStatus fetch_next_state();

	void reward_progress();

	void get_successor_operators(std::vector<const GlobalOperator *> &ops);

	void print_checkpoint_line(int g) const;

public:
	explicit LazySearchYahspLookahead(const options::Options &opts);
	virtual ~LazySearchYahspLookahead() = default;

	void set_pref_operator_heuristics(std::vector<Heuristic *> &heur);

	void print_statistics() const override;
};

}

#endif
