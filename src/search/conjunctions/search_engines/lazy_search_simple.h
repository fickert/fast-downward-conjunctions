#ifndef CONJUNCTIONS_LAZY_SEARCH_SIMPLE_H
#define CONJUNCTIONS_LAZY_SEARCH_SIMPLE_H

#include "online_learning_search_engine.h"

#include "../../open_lists/open_list.h"

#include <memory>
#include <vector>


class GlobalOperator;
class Heuristic;

namespace options {
class Options;
}

namespace conjunctions {
class LazySearchSimple : public OnlineLearningSearchEngine {
protected:
	std::unique_ptr<EdgeOpenList> open_list;

	// Search behavior parameters
	bool randomize_successors;
	bool preferred_successors_first;

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

	class DetectLocalMinimum {
	public:
		DetectLocalMinimum() :
			best_h(std::numeric_limits<int>::max()) {}

		virtual ~DetectLocalMinimum() = default;

		virtual auto operator()() -> bool {
			return false;
		}

		virtual void new_state(int) {}

		virtual void dead_end() {}

		virtual void reset() {}

		auto get_best_h() const -> int {
			return best_h;
		}

	protected:
		int best_h;
	};

	class DetectLocalMinimumByTime : public DetectLocalMinimum {
	public:
		DetectLocalMinimumByTime(double local_minimum_time) :
			DetectLocalMinimum(),
			time_since_current_minimum(),
			local_minimum_time(local_minimum_time) {}

		~DetectLocalMinimumByTime() override = default;

		auto operator()() -> bool override {
			return time_since_current_minimum() > local_minimum_time;
		}

		void new_state(int h) override {
			if (h < best_h) {
				best_h = h;
				reset();
			}
		}

		void reset() override {
			time_since_current_minimum.reset();
		}

	private:
		utils::Timer time_since_current_minimum;
		const double local_minimum_time;
	};

	class DetectLocalMinimumByStates : public DetectLocalMinimum {
	public:
		DetectLocalMinimumByStates(int local_minimum_states) :
			DetectLocalMinimum(),
			states_since_current_minimum(0),
			local_minimum_states(local_minimum_states) {}

		~DetectLocalMinimumByStates() override = default;

		auto operator()() -> bool override {
			return states_since_current_minimum > local_minimum_states;
		}

		void new_state(int h) override {
			if (h < best_h) {
				best_h = h;
				states_since_current_minimum = 0;
			} else {
				++states_since_current_minimum;
			}
		}

		void dead_end() override {
			++states_since_current_minimum;
		}

		void reset() override {
			best_h = std::numeric_limits<int>::max();
			states_since_current_minimum = 0;
		}

	private:
		int states_since_current_minimum;
		const int local_minimum_states;
	};

	static auto get_detect_local_minimum(const options::Options &opts) -> std::unique_ptr<DetectLocalMinimum>;

	const std::unique_ptr<DetectLocalMinimum> detect_local_minimum;

	enum class RPSequence {
		NONE,
		ALL,
		ALL_INTENDED,
		FIRST_DELETER
	} const rp_sequence;

	const bool prioritize_rp_sequence;

	void initialize() override;
	SearchStatus step() override;

	void generate_successors();
	SearchStatus fetch_next_state();

	void reward_progress();

	void get_successor_operators(std::vector<const GlobalOperator *> &ops);

	void print_checkpoint_line(int g) const;

public:
	explicit LazySearchSimple(const options::Options &opts);
	virtual ~LazySearchSimple() = default;

	void set_pref_operator_heuristics(std::vector<Heuristic *> &heur);

	void print_statistics() const override;
};
}

#endif
