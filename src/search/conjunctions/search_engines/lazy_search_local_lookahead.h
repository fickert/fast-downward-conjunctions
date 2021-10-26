
#ifndef CONJUNCTIONS_LAZY_SEARCH_LOCAL_LOOKAHEAD_H
#define CONJUNCTIONS_LAZY_SEARCH_LOCAL_LOOKAHEAD_H

#include "online_learning_search_engine.h"

#include "../../open_lists/open_list.h"

#include <memory>
#include <vector>
#include <queue>
#include <optional>


class GlobalOperator;
class Heuristic;

namespace options {
class Options;
}

namespace conjunctions {
class LazySearchLocalLookahead : public OnlineLearningSearchEngine {
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

	ConjunctionsHeuristic *conjunctions_heuristic;
	const std::shared_ptr<ConjunctionGenerationStrategy> strategy;

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

	bool solved;

	enum class ConjunctionGenerationStatus { SOLVED, DEAD_END, DEFAULT };
	auto generate_conjunction(SearchNode &node, SearchNode *global_lookahead_node = nullptr) -> ConjunctionGenerationStatus;

	void initialize() override;
	SearchStatus step() override;

	auto get_evaluators(int w, bool include_pref_eval) -> std::vector<ScalarEvaluator *>;
	std::unique_ptr<EdgeOpenList> lookahead_open_list;


	std::unique_ptr<SearchSpace> lookahead_search_space;

	enum class LookaheadClosedLearning { YES, NO, MAYBE } const do_learning_if_lookahead_closed;
	const bool exhaustive_lookahead;
	const bool no_learning;

	void lookahead_expand(EvaluationContext &eval_context, SearchNode &node);
	auto lookahead() -> std::pair<SearchStatus, StateID>;

	void generate_successors();
	SearchStatus fetch_next_state();

	void reward_progress();

	void get_successor_operators(std::vector<const GlobalOperator *> &ops);

	void print_checkpoint_line(int g) const;

	struct IteratedWeightsOptions {
		std::vector<ScalarEvaluator *> evals;
		std::vector<Heuristic *> preferred;
		int boost;
		std::vector<int> weights;
		int current_weight_index;
		bool repeat_last;
		bool currently_repeating;
	};

	std::unique_ptr<IteratedWeightsOptions> iterated_weights_options;

	const bool enable_heuristic_cache;
	std::unordered_map<StateID, std::pair<int, std::vector<int>>> heuristic_cache;
	void update_eval_context(EvaluationContext &eval_context, const decltype(heuristic_cache)::mapped_type &cache_entry);

	auto evaluate_if_neccessary(EvaluationContext &eval_context, const GlobalState &parent_state, const GlobalOperator &last_op) -> int;
	auto evaluate_if_neccessary(EvaluationContext &eval_context) -> int;

public:
	explicit LazySearchLocalLookahead(const options::Options &opts);
	virtual ~LazySearchLocalLookahead() = default;

	void set_pref_operator_heuristics(std::vector<Heuristic *> &heur);
	void set_iterated_weights_options(const options::Options &opts);
	auto has_another_phase() const -> bool;
	void restart_with_next_weight();

	void print_statistics() const override;
};

}

#endif