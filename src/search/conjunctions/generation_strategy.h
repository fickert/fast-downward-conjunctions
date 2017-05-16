#pragma once

#include "conjunctions.h"
#include "../evaluation_context.h"
#include "utils.h"

#include <memory>
#include <type_traits>
#include <vector>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(default: 4800 4512 4706 4100 4127 4702 4239 4996 4456 4458 4505)
#endif

namespace conjunctions {

class ConjunctionsHeuristic;
class ConflictExtraction;

class ConjunctionGenerationStrategy {
public:
	ConjunctionGenerationStrategy(const options::Options &opts);
	virtual ~ConjunctionGenerationStrategy();

	enum class Event {
		INITIALIZATION,
		STEP,
		LOCAL_MINIMUM,
		START_EHC_PHASE
	};

	enum class Result {
		MODIFIED,
		DEAD_END,
		UNMODIFIED,
		SOLVED
	};

	// returns true if the set of conjunctions was modified
	auto modify_conjunctions(ConjunctionsHeuristic &heuristic, Event event, const AbstractTask &task, EvaluationContext &eval_context) -> Result;

	virtual auto deletes_conjunctions() const -> bool {
		return false;
	}

	virtual void dump_options() const = 0;

protected:
	virtual auto modify_conjunctions_init(ConjunctionsHeuristic &, const AbstractTask &, EvaluationContext &) -> Result {
		return Result::UNMODIFIED;
	}

	virtual auto modify_conjunctions_step(ConjunctionsHeuristic &, const AbstractTask &, EvaluationContext &) -> Result {
		return Result::UNMODIFIED;
	}

	virtual auto modify_conjunctions_local_minimum(ConjunctionsHeuristic &, const AbstractTask &, EvaluationContext &) -> Result {
		return Result::UNMODIFIED;
	}

	virtual auto modify_conjunctions_start_ehc_phase(ConjunctionsHeuristic &, const AbstractTask &, EvaluationContext &) -> Result {
		return Result::UNMODIFIED;
	}
};


class ConflictExtractionStrategy : public ConjunctionGenerationStrategy {
public:
	ConflictExtractionStrategy(const options::Options &opts);
	virtual ~ConflictExtractionStrategy();

	void dump_options() const override;
	static void add_options(options::OptionParser &);

protected:
	const std::shared_ptr<ConflictExtraction> conflict_extraction;

	const int conjunctions_per_iteration;
	const bool check_relaxed_plan;

	auto generate_conjunctions(ConjunctionsHeuristic &, const AbstractTask &, EvaluationContext &, int) -> std::pair<Result, std::vector<FactSet>>;
};


// only generate conjunctions during initialization
class GenerateInitially : public ConflictExtractionStrategy {
public:
	GenerateInitially(const options::Options &opts);
	~GenerateInitially();

	static void add_options(options::OptionParser &);
	void dump_options() const override;

protected:
	auto modify_conjunctions_init(ConjunctionsHeuristic &, const AbstractTask &, EvaluationContext &) -> Result override;

private:
	const double conjunction_growth_bound;
	const double counter_growth_bound;
	const double counter_sum_growth_bound;
	const int learning_time;
};


// generate all conjunctions up to size m
class GenerateAllBoundedSize : public ConjunctionGenerationStrategy {
public:
	GenerateAllBoundedSize(const options::Options &opts);
	~GenerateAllBoundedSize();

	void dump_options() const override;

private:
	auto modify_conjunctions_init(ConjunctionsHeuristic &, const AbstractTask &, EvaluationContext &) -> Result override;

	void get_all_combinations(std::vector<FactSet> &combinations, const FactSet &base, const int max_size, const AbstractTask &task) const;

	const int m;
};


// add conjunctions from file during initialization
class LoadFromFile : public ConjunctionGenerationStrategy {
public:
	LoadFromFile(const options::Options &opts);
	~LoadFromFile();

	void dump_options() const override;

private:
	auto modify_conjunctions_init(ConjunctionsHeuristic &, const AbstractTask &, EvaluationContext &) -> Result override;

	const std::string file_name;
};


// generate a specific number of conjunctions that is read from a file
class ReadCountFromFile : public ConflictExtractionStrategy {
public:
	ReadCountFromFile(const options::Options &opts);
	~ReadCountFromFile();

	void dump_options() const override;

private:
	auto modify_conjunctions_init(ConjunctionsHeuristic &, const AbstractTask &, EvaluationContext &) -> Result override;

	static auto read_count(const std::string &file_name) -> int;

	const int count;
};


// helper functions
namespace detail {

enum class RemovalStrategy {
	OLDEST,
	LEAST_FREQUENTLY_IN_RELAXED_PLANS,
	LEAST_EFFICIENT,
	MOST_COUNTERS,
	RANDOM
};

auto operator<<(std::ostream &, const RemovalStrategy) -> std::ostream &;
void remove_conjunctions(ConjunctionsHeuristic &heuristic, int count, RemovalStrategy strategy, int min_evaluations);
void add_removal_strategy_options(options::OptionParser &parser);
void print_removal_strategy_options(RemovalStrategy, int);

}


// generate conjunctions up to a given bound during initialization, then maintain this size but some conjunctions with new ones
class MaintainFixedSize : public GenerateInitially {
public:
	MaintainFixedSize(const options::Options &opts);
	~MaintainFixedSize();

	void dump_options() const override;

	auto deletes_conjunctions() const -> bool override {
		return removed_conjunction;
	}

private:
	auto modify_conjunctions_step(ConjunctionsHeuristic &, const AbstractTask &, EvaluationContext &) -> Result override;

	const detail::RemovalStrategy removal_strategy;
	const int min_evaluations;
	const int replacement_frequency;
	const int replacement_count;
	int states_counter;
	bool removed_conjunction;
};


// generate conjunctions in local minima
class EscapeLocalMinima : public ConflictExtractionStrategy {
public:
	EscapeLocalMinima(const options::Options &opts);
	~EscapeLocalMinima();

	void dump_options() const override;

	static void add_options(options::OptionParser &);

protected:
	auto modify_conjunctions_local_minimum(ConjunctionsHeuristic &, const AbstractTask &, EvaluationContext &) -> Result override;

private:
	const double conjunction_growth_bound;
	const double counter_growth_bound;
	const int learning_time;
};


// generate conjunctions in local minima, if a fixed growth bound is reached replace conjunctions instead
class EscapeLocalMinimaBounded : public EscapeLocalMinima {
public:
	EscapeLocalMinimaBounded(const options::Options &opts);
	~EscapeLocalMinimaBounded();

	void dump_options() const override;

	auto deletes_conjunctions() const -> bool override {
		return reached_threshold;
	}

private:
	auto modify_conjunctions_local_minimum(ConjunctionsHeuristic &, const AbstractTask &, EvaluationContext &) -> Result override;

	const detail::RemovalStrategy removal_strategy;
	const int min_evaluations;
	const double replacement_threshold;
	bool reached_threshold;
};


// generate conjunctions in local minima, then forget the conjunctions upon the start of the next ehc phase
class EscapeLocalMinimaForgetAll : public EscapeLocalMinima {
public:
	EscapeLocalMinimaForgetAll(const options::Options &opts);
	~EscapeLocalMinimaForgetAll();

	void dump_options() const override;

	static void add_options(options::OptionParser &);

protected:
	auto modify_conjunctions_start_ehc_phase(ConjunctionsHeuristic &, const AbstractTask &, EvaluationContext &) -> Result override;
};

}

#ifdef _MSC_VER
#pragma warning(pop)
#endif
