#ifndef YAHSP_LOOKAHEAD_H
#define YAHSP_LOOKAHEAD_H

#include <vector>

class EvaluationContext;
class SearchStatistics;

namespace successor_generator {
class SuccessorGenerator;
}

class GlobalState;
class SearchSpace;
class StateRegistry;
class GlobalOperator;
class SearchProgress;
class Heuristic;

namespace options {
	class OptionParser;
	class Options;
}

class YahspLookahead {
public:
	explicit YahspLookahead(const options::Options &opts);
	explicit YahspLookahead(bool do_repair);
	~YahspLookahead() = default;

	auto lookahead(EvaluationContext &eval_context, Heuristic &heuristic, StateRegistry &state_registry, successor_generator::SuccessorGenerator &successor_generator) const -> std::pair<GlobalState, std::vector<const GlobalOperator *>>;

	static void add_options_to_parser(options::OptionParser &parser);

private:
	const bool do_repair;
};

#endif // YAHSP_LOOKAHEAD_H
