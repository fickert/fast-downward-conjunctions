#include "generation_strategy.h"

#include "../options/option_parser.h"
#include "../options/plugin.h"

#include "conjunctions_heuristic.h"
#include "conflict_extraction.h"
#include "utils.h"

#include <chrono>
#include <fstream>


#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(default: 4800 4512 4706 4100 4127 4702 4239 4996 4456 4458 4505)
#endif

namespace conjunctions {

// strategy base class

ConjunctionGenerationStrategy::ConjunctionGenerationStrategy(const options::Options &) {}

ConjunctionGenerationStrategy::~ConjunctionGenerationStrategy() {}

auto ConjunctionGenerationStrategy::modify_conjunctions(ConjunctionsHeuristic &heuristic, Event event, const AbstractTask &task, EvaluationContext &eval_context) -> Result {
	switch (event) {
	case Event::INITIALIZATION:
		return modify_conjunctions_init(heuristic, task, eval_context);
	case Event::STEP:
		return modify_conjunctions_step(heuristic, task, eval_context);
	case Event::LOCAL_MINIMUM:
		return modify_conjunctions_local_minimum(heuristic, task, eval_context);
	case Event::START_EHC_PHASE:
		return modify_conjunctions_start_ehc_phase(heuristic, task, eval_context);
	default:
		assert(false && "unknown event");
		exit(1);
	}
}

// conflict extraction strategy base class

ConflictExtractionStrategy::ConflictExtractionStrategy(const options::Options &opts) :
	ConjunctionGenerationStrategy(opts),
	conflict_extraction(opts.get<std::shared_ptr<ConflictExtraction>>("conflict_extraction")),
	conjunctions_per_iteration(opts.get<int>("conjunctions_per_iteration")),
	check_relaxed_plan(opts.get<bool>("check_relaxed_plan")) {}

ConflictExtractionStrategy::~ConflictExtractionStrategy() {}

void ConflictExtractionStrategy::add_options(options::OptionParser &parser) {
	parser.add_option<std::shared_ptr<ConflictExtraction>>("conflict_extraction", "Conflict extraction method.", "conflict_extraction()");
	parser.add_option<int>("conjunctions_per_iteration", "maximum number of conjunctions to be added in each iteration", "1");
	parser.add_option<bool>("check_relaxed_plan", "check if relaxed plan is valid before attempting to learn conjunctions", "true");
}

auto ConflictExtractionStrategy::generate_conjunctions(ConjunctionsHeuristic &heuristic, const AbstractTask &task, EvaluationContext &eval_context, int count) -> std::pair<Result, std::vector<FactSet>> {
	auto &cached_result = const_cast<HeuristicCache &>(eval_context.get_cache())[&heuristic];
	if (!cached_result.is_uninitialized())
		cached_result = EvaluationResult();
	if (!heuristic.is_last_bsg_valid_for_state(eval_context.get_state()))
		eval_context.get_result(&heuristic);
	if (eval_context.is_heuristic_infinite(&heuristic))
		return {Result::DEAD_END, {}};
	auto &bsg = heuristic.get_last_bsg();

	assert(!bsg.nodes.empty() && "The last bsg should not be empty here. Make sure the 'cache_estimates' option of the heuristic is disabled.");

	if (check_relaxed_plan && is_valid_plan_in_the_original_task(bsg, eval_context.get_state().get_values(), task))
		return {Result::SOLVED, {}};

	auto conjunctions_to_add = conflict_extraction->generate_candidate_conjunctions(task, bsg, heuristic, count);
	assert(!conjunctions_to_add.empty() || is_valid_plan_in_the_original_task(bsg, eval_context.get_state().get_values(), task));
	return {conjunctions_to_add.empty() ? Result::SOLVED : Result::MODIFIED, std::move(conjunctions_to_add)};
}

void ConflictExtractionStrategy::dump_options() const {
	std::cout << "conjunctions per iteration: " << conjunctions_per_iteration << std::endl;
	std::cout << "check relaxed plan: " << check_relaxed_plan << std::endl;
}


// generate all conjunctions up to size m

GenerateAllBoundedSize::GenerateAllBoundedSize(const options::Options &opts) :
	ConjunctionGenerationStrategy(opts),
	m(opts.get<int>("m")) {}

GenerateAllBoundedSize::~GenerateAllBoundedSize() {}

auto GenerateAllBoundedSize::modify_conjunctions_init(ConjunctionsHeuristic &heuristic, const AbstractTask &task, EvaluationContext &) -> Result {
	auto all_combinations = std::vector<FactSet>();
	get_all_combinations(all_combinations, {}, m, task);
	assert(std::is_sorted(std::begin(all_combinations), std::end(all_combinations)));
	assert(std::unique(std::begin(all_combinations), std::end(all_combinations)) == std::end(all_combinations));
	all_combinations.erase(std::remove_if(std::begin(all_combinations), std::end(all_combinations),
		[&task](const auto &facts) { return contains_mutex(task, facts); }), std::end(all_combinations));
	heuristic.add_conjunctions(all_combinations);
	return all_combinations.empty() ? Result::UNMODIFIED : Result::MODIFIED;
}

void GenerateAllBoundedSize::get_all_combinations(std::vector<FactSet> &combinations, const FactSet &base, const int max_size, const AbstractTask &task) const{
	auto next_fact = [&task](const FactPair &f) -> FactPair {
		if (f.value + 1 < task.get_variable_domain_size(f.var))
			return {f.var, f.value + 1};
		if (f.var + 1 < task.get_num_variables())
			return {f.var + 1, 0};
		return {-1, -1};
	};
	assert(base.empty() || base.back().var + 1 < task.get_num_variables());
	for (auto f = FactPair(base.empty() ? 0 : base.back().var + 1, 0); f.var != -1; f = next_fact(f)) {
		auto new_combination = base;
		new_combination.push_back(f);
		assert(std::is_sorted(std::begin(new_combination), std::end(new_combination)));
		if (!contains_mutex(task, new_combination)) {
			assert(std::find(std::begin(combinations), std::end(combinations), new_combination) == std::end(combinations));
			if (new_combination.size() > 1)
				combinations.push_back(new_combination);
			if (static_cast<int>(new_combination.size()) < m && f.var + 1 < task.get_num_variables())
				get_all_combinations(combinations, new_combination, max_size, task);
		}
	}
}

void GenerateAllBoundedSize::dump_options() const {
	std::cout << "m: " << m << std::endl;
}

static auto _parse_generate_all_bounded_size(options::OptionParser &parser) -> std::shared_ptr<ConjunctionGenerationStrategy> {
	parser.add_option<int>("m", "maximum number of facts per conjunction", "2");
	if (parser.help_mode() || parser.dry_run())
		return nullptr;
	return std::make_shared<GenerateAllBoundedSize>(parser.parse());
}


// generate conjunctions during initialization only

GenerateInitially::GenerateInitially(const options::Options &opts) :
	ConflictExtractionStrategy(opts),
	conjunction_growth_bound(opts.get<double>("conjunction_growth_bound")),
	counter_growth_bound(opts.get<double>("counter_growth_bound")),
	counter_sum_growth_bound(opts.get<double>("counter_sum_growth_bound")),
	learning_time(opts.get<int>("learning_time")) {}

GenerateInitially::~GenerateInitially() {}

auto GenerateInitially::modify_conjunctions_init(ConjunctionsHeuristic &heuristic, const AbstractTask &task, EvaluationContext &eval_context) -> Result {
	auto modified = false;
	auto end_time = std::chrono::steady_clock::now() + std::chrono::seconds(learning_time);
	while (std::chrono::steady_clock::now() < end_time
		&& heuristic.get_conjunction_growth() < conjunction_growth_bound
		&& heuristic.get_counter_growth() < counter_growth_bound
		&& heuristic.get_counter_size_growth() < counter_sum_growth_bound) {

		auto result = generate_conjunctions(heuristic, task, eval_context, conjunctions_per_iteration);
		if (result.first != Result::MODIFIED)
			return result.first;
		heuristic.add_conjunctions(result.second);
		modified = true;
#ifndef NDEBUG
		std::cout << "Current problem size factor in the number of conjunctions/counters: " << heuristic.get_conjunction_growth() << "/" << heuristic.get_counter_growth() << std::endl;
		std::cout << "Current problem size factor in the sum of all counters: " << heuristic.get_counter_size_growth() << std::endl;
#endif
	}
	return modified ? Result::MODIFIED : Result::UNMODIFIED;
}

void GenerateInitially::dump_options() const {
	ConflictExtractionStrategy::dump_options();
	std::cout << "growth bound in the number of conjunctions (including singletons): " << conjunction_growth_bound << std::endl;
	std::cout << "growth bound in the number of counters: " << counter_growth_bound << std::endl;
	std::cout << "growth bound in the sum of counters: " << counter_sum_growth_bound << std::endl;
	std::cout << "learning time bound: " << learning_time << std::endl;
}

void GenerateInitially::add_options(options::OptionParser &parser) {
	ConflictExtractionStrategy::add_options(parser);
	parser.add_option<double>("conjunction_growth_bound", "maximum growth in the number of conjunctions (including singletons) as a factor of the original", std::to_string(std::numeric_limits<double>::max()));
	parser.add_option<double>("counter_growth_bound", "maximum growth in the number of counters as a factor of the original", "1.5");
	parser.add_option<double>("counter_sum_growth_bound", "maximum growth in the sum of counters as a factor of the original", std::to_string(std::numeric_limits<double>::max()));
	parser.add_option<int>("learning_time", "bound on learning time in seconds", "900");
}

static auto _parse_generate_initially(options::OptionParser &parser) -> std::shared_ptr<ConjunctionGenerationStrategy> {
	GenerateInitially::add_options(parser);
	if (parser.help_mode() || parser.dry_run())
		return nullptr;
	return std::make_shared<GenerateInitially>(parser.parse());
}


// load conjunctions from file during initialization

LoadFromFile::LoadFromFile(const options::Options &opts) :
	ConjunctionGenerationStrategy(opts),
	file_name(opts.get<std::string>("file_name")) {
	if (!std::experimental::filesystem::exists(std::experimental::filesystem::path(file_name))) {
		std::cout << "File not found: " << file_name << std::endl;
		utils::exit_with(utils::ExitCode::INPUT_ERROR);
	}
}

LoadFromFile::~LoadFromFile() {}

auto LoadFromFile::modify_conjunctions_init(ConjunctionsHeuristic &heuristic, const AbstractTask &, EvaluationContext &) -> Result {
	auto parse_fact = [](const std::string &line, int begin_index) {
		assert(line[begin_index] == '(');
		auto sep_index = line.find(',', begin_index);
		auto var = std::stoi(line.substr(begin_index + 1, sep_index));
		auto value = std::stoi(line.substr(sep_index + 2, line.find(')', sep_index + 2)));
		return FactPair(var, value);
	};

	auto parse_fact_set = [parse_fact](const std::string &line, int begin_index) {
		assert(line[begin_index] == '(');
		++begin_index;
		const auto num_facts = (std::count(std::begin(line) + begin_index, std::end(line), ',') + 1) / 2;
		assert(num_facts > 0);
		auto facts = FactSet();
		facts.reserve(num_facts);
		do {
			facts.emplace_back(parse_fact(line, begin_index));
			const auto var_value_sep = line.find(',', begin_index);
			if (var_value_sep == std::string::npos)
				break;
			const auto fact_sep = line.find(',', var_value_sep + 1);
			if (fact_sep == std::string::npos)
				break;
			begin_index = fact_sep + 2;
		} while (true);
		return facts;
	};

	auto in = std::ifstream(file_name);
	auto line = std::string();
	auto conjunctions = std::vector<FactSet>();
	while (!in.eof()) {
		std::getline(in, line);
		if (!line.empty()) {
			assert(line.find("Conjunction(") == 0u);
			conjunctions.push_back(parse_fact_set(line, 11));
		}
	}
	in.close();
	heuristic.add_conjunctions(conjunctions);
	return conjunctions.empty() ? Result::UNMODIFIED : Result::MODIFIED;
}

void LoadFromFile::dump_options() const {
	std::cout << "conjunctions file: " << file_name << std::endl;
}

static auto _parse_load_from_file(options::OptionParser &parser) -> std::shared_ptr<ConjunctionGenerationStrategy> {
	parser.add_option<std::string>("file_name", "file containing conjunctions", "conjunctions");
	if (parser.help_mode() || parser.dry_run())
		return nullptr;
	return std::make_shared<LoadFromFile>(parser.parse());
}


// generate a specific number of conjunctions that is read from a file

auto ReadCountFromFile::read_count(const std::string &file_name) -> int {
	if (!std::experimental::filesystem::exists(std::experimental::filesystem::path(file_name))) {
		std::cout << "File not found: " << file_name << std::endl;
		utils::exit_with(utils::ExitCode::INPUT_ERROR);
	}
	auto in = std::ifstream(file_name);
	auto count = 0;
	in >> count;
	return count;
}

ReadCountFromFile::ReadCountFromFile(const options::Options &opts) :
	ConflictExtractionStrategy(opts),
	count(read_count(opts.get<std::string>("file_name"))) {}

ReadCountFromFile::~ReadCountFromFile() {}

auto ReadCountFromFile::modify_conjunctions_init(ConjunctionsHeuristic &heuristic, const AbstractTask &task, EvaluationContext &eval_context) -> Result {
	auto num_added = 0;
	while (num_added < count) {
		auto result = generate_conjunctions(heuristic, task, eval_context, conjunctions_per_iteration);
		if (result.first != Result::MODIFIED)
			return result.first;
		heuristic.add_conjunctions(result.second);
		num_added += result.second.size();
#ifndef NDEBUG
		std::cout << "Current problem size factor in the number of conjunctions/counters: " << heuristic.get_conjunction_growth() << "/" << heuristic.get_counter_growth() << std::endl;
		std::cout << "Current problem size factor in the sum of all counters: " << heuristic.get_counter_size_growth() << std::endl;
#endif
	}
	return num_added != 0 ? Result::MODIFIED : Result::UNMODIFIED;
}

void ReadCountFromFile::dump_options() const {
	ConflictExtractionStrategy::dump_options();
	std::cout << "count: " << count << std::endl;
}

static auto _parse_read_count_from_file(options::OptionParser &parser) -> std::shared_ptr<ConjunctionGenerationStrategy> {
	ConflictExtractionStrategy::add_options(parser);
	parser.add_option<std::string>("file_name", "file containing the number of conjunctions to generate", "count");
	if (parser.help_mode() || parser.dry_run())
		return nullptr;
	return std::make_shared<ReadCountFromFile>(parser.parse());
}


namespace detail {

auto operator<<(std::ostream &out, const RemovalStrategy rs) -> std::ostream & {
	switch (rs) {
	case RemovalStrategy::OLDEST:
		return out << "OLDEST";
	case RemovalStrategy::LEAST_FREQUENTLY_IN_RELAXED_PLANS:
		return out << "LEAST_FREQUENTLY_IN_RELAXED_PLANS";
	case RemovalStrategy::LEAST_EFFICIENT:
		return out << "LEAST_EFFICIENT";
	case RemovalStrategy::MOST_COUNTERS:
		return out << "MOST_COUNTERS";
	case RemovalStrategy::RANDOM:
		return out << "RANDOM";
	default:
		std::cerr << "Unknown removal strategy option:" << rs << std::endl;
		utils::exit_with(utils::ExitCode::CRITICAL_ERROR);
	}
}

void remove_conjunctions(ConjunctionsHeuristic &heuristic, int count, RemovalStrategy removal_strategy, int min_evaluations) {
	switch (removal_strategy) {
	case RemovalStrategy::OLDEST:
		heuristic.remove_oldest_conjunctions(count, min_evaluations);
		break;
	case RemovalStrategy::LEAST_FREQUENTLY_IN_RELAXED_PLANS:
		heuristic.remove_least_frequently_in_relaxed_plans_conjunctions(count, min_evaluations);
		break;
	case RemovalStrategy::LEAST_EFFICIENT:
		heuristic.remove_least_efficient_conjunctions(count, min_evaluations);
		break;
	case RemovalStrategy::MOST_COUNTERS:
		heuristic.remove_conjunctions_with_most_counters(count, min_evaluations);
		break;
	case RemovalStrategy::RANDOM:
		heuristic.remove_random_conjunctions(count, min_evaluations);
		break;
	default:
		std::cerr << "Unknown replacement strategy option:" << static_cast<int>(removal_strategy) << std::endl;
		utils::exit_with(utils::ExitCode::CRITICAL_ERROR);
	}
}

void add_removal_strategy_options(options::OptionParser &parser) {
	parser.add_enum_option("removal_strategy", {"OLDEST", "LEAST_FREQUENTLY_IN_RELAXED_PLANS", "LEAST_EFFICIENT", "MOST_COUNTERS", "RANDOM"},
		"strategy describing which conjunctions to replace", "OLDEST");
	parser.add_option<int>("min_evaluations", "minimum number of evaluations a conjunction has to be part of before it can be replaced", "0");
}

void print_removal_strategy_options(RemovalStrategy removal_strategy, int min_evaluations) {
	std::cout << "Removal strategy: " << removal_strategy << std::endl;
	std::cout << "min evaluations: " << min_evaluations << std::endl;
}

}

// learn conjunctions until a bound x is reached, then replace conjunctions periodically during search

MaintainFixedSize::MaintainFixedSize(const options::Options &opts)
	: GenerateInitially(opts),
	removal_strategy(detail::RemovalStrategy(opts.get_enum("removal_strategy"))),
	min_evaluations(opts.get<int>("min_evaluations")),
	replacement_frequency(opts.get<int>("replacement_frequency")),
	replacement_count(opts.get<int>("replacement_count")),
	states_counter(0),
	removed_conjunction(false) {}

MaintainFixedSize::~MaintainFixedSize() {}

auto MaintainFixedSize::modify_conjunctions_step(ConjunctionsHeuristic &heuristic, const AbstractTask &task, EvaluationContext &eval_context) -> Result {
	if (++states_counter % replacement_frequency != 0)
		return Result::UNMODIFIED;
	remove_conjunctions(heuristic, replacement_count, removal_strategy, min_evaluations);
	removed_conjunction = true;
	auto result = generate_conjunctions(heuristic, task, eval_context, conjunctions_per_iteration);
	if (result.first != Result::MODIFIED)
		return result.first;
	heuristic.add_conjunctions(result.second);
#ifndef NDEBUG
	std::cout << "Current problem size factor in the number of conjunctions/counters: " << heuristic.get_conjunction_growth() << "/" << heuristic.get_counter_growth() << std::endl;
#endif
	return Result::MODIFIED;
}

void MaintainFixedSize::dump_options() const {
	GenerateInitially::dump_options();
	detail::print_removal_strategy_options(removal_strategy, min_evaluations);
	std::cout << "replacement frequency: " << replacement_frequency << std::endl;
	std::cout << "replacement count: " << replacement_count << std::endl;
}

static auto _parse_maintain_fixed_size(options::OptionParser &parser) -> std::shared_ptr<ConjunctionGenerationStrategy> {
	GenerateInitially::add_options(parser);
	detail::add_removal_strategy_options(parser);
	parser.add_option<int>("replacement_frequency", "replace conjunctions every X states", "10");
	parser.add_option<int>("replacement_count", "number of conjunctions to replace in each iteration", "1");
	if (parser.help_mode() || parser.dry_run())
		return nullptr;
	return std::make_shared<MaintainFixedSize>(parser.parse());
}


// learn conjunctions every time a local minimum is reached

EscapeLocalMinima::EscapeLocalMinima(const options::Options &opts) :
	ConflictExtractionStrategy(opts),
	conjunction_growth_bound(opts.get<double>("conjunction_growth_bound")),
	counter_growth_bound(opts.get<double>("counter_growth_bound")),
	learning_time(opts.get<int>("learning_time")) {}

EscapeLocalMinima::~EscapeLocalMinima() {}

auto EscapeLocalMinima::modify_conjunctions_local_minimum(ConjunctionsHeuristic &heuristic, const AbstractTask &task, EvaluationContext &eval_context) -> Result {
	auto next_conjunction_growth_bound = heuristic.get_conjunction_growth() * conjunction_growth_bound;
	auto next_counter_growth_bound = heuristic.get_counter_growth() * counter_growth_bound;
	auto end_time = std::chrono::steady_clock::now() + std::chrono::seconds(learning_time);
	auto modified = false;
	while (std::chrono::steady_clock::now() < end_time
		&& heuristic.get_conjunction_growth() < next_conjunction_growth_bound
		&& heuristic.get_counter_growth() < next_counter_growth_bound) {

		auto result = generate_conjunctions(heuristic, task, eval_context, conjunctions_per_iteration);
		if (result.first != Result::MODIFIED)
			return result.first;
		heuristic.add_conjunctions(result.second);
		modified = true;
#ifndef NDEBUG
		std::cout << "Current problem size factor in the number of conjunctions/counters: " << heuristic.get_conjunction_growth() << "/" << heuristic.get_counter_growth() << std::endl;
#endif
	}
	return modified ? Result::MODIFIED : Result::UNMODIFIED;
}

void EscapeLocalMinima::dump_options() const {
	ConflictExtractionStrategy::dump_options();
	std::cout << "growth bound in the number of conjunctions (including singletons): " << conjunction_growth_bound << std::endl;
	std::cout << "growth bound in the number of counters: " << counter_growth_bound << std::endl;
	std::cout << "learning time bound: " << learning_time << std::endl;
}


void EscapeLocalMinima::add_options(options::OptionParser &parser) {
	ConflictExtractionStrategy::add_options(parser);
	parser.add_option<double>("conjunction_growth_bound", "maximum growth in the number of conjunctions (including singletons) as a factor of the original", "1.000001");
	parser.add_option<double>("counter_growth_bound", "maximum growth in the number of counters as a factor of the original", std::to_string(std::numeric_limits<double>::max()));
	parser.add_option<int>("learning_time", "bound on learning time in seconds", "60");
}

static auto _parse_escape_local_minima(options::OptionParser &parser) -> std::shared_ptr<ConjunctionGenerationStrategy> {
	EscapeLocalMinima::add_options(parser);
	if (parser.help_mode() || parser.dry_run())
		return nullptr;
	return std::make_shared<EscapeLocalMinima>(parser.parse());
}


// learn a single conjunction every time a local minimum is reached, if the number of conjunctions exceeds a given bound replace an existing conjunction instead

EscapeLocalMinimaBounded::EscapeLocalMinimaBounded(const options::Options &opts) :
	EscapeLocalMinima(opts),
	removal_strategy(detail::RemovalStrategy(opts.get_enum("removal_strategy"))),
	min_evaluations(opts.get<int>("min_evaluations")),
	replacement_threshold(opts.get<double>("replacement_threshold")),
	reached_threshold(false) {}

EscapeLocalMinimaBounded::~EscapeLocalMinimaBounded() {}

auto EscapeLocalMinimaBounded::modify_conjunctions_local_minimum(ConjunctionsHeuristic &heuristic, const AbstractTask &task, EvaluationContext &eval_context) -> Result {
	auto removed = false;
	if (heuristic.get_counter_growth() > replacement_threshold) {
		detail::remove_conjunctions(heuristic, conjunctions_per_iteration, removal_strategy, min_evaluations);
		removed = true;
		reached_threshold = true;
	}
	auto result = EscapeLocalMinima::modify_conjunctions_local_minimum(heuristic, task, eval_context);
	return result == Result::UNMODIFIED ? (removed ? Result::MODIFIED : Result::UNMODIFIED) : result;
}

void EscapeLocalMinimaBounded::dump_options() const {
	EscapeLocalMinima::dump_options();
	detail::print_removal_strategy_options(removal_strategy, min_evaluations);
}

static auto _parse_escape_local_minima_bounded(options::OptionParser &parser) -> std::shared_ptr<ConjunctionGenerationStrategy> {
	EscapeLocalMinima::add_options(parser);
	detail::add_removal_strategy_options(parser);
	parser.add_option<double>("replacement_threshold", "Growth bound size in the number of counters. After this threshold is reached, conjunctions are replaced instead of added.", "1.5");
	if (parser.help_mode() || parser.dry_run())
		return nullptr;
	return std::make_shared<EscapeLocalMinimaBounded>(parser.parse());
}


// learn a single conjunction every time a local minimum is reached, and forget all conjunctions on the start of each EHC phase

EscapeLocalMinimaForgetAll::EscapeLocalMinimaForgetAll(const options::Options &opts) :
	EscapeLocalMinima(opts) {}

EscapeLocalMinimaForgetAll::~EscapeLocalMinimaForgetAll() {}

auto EscapeLocalMinimaForgetAll::modify_conjunctions_start_ehc_phase(ConjunctionsHeuristic &heuristic, const AbstractTask &, EvaluationContext &eval_context) -> Result {
	if (heuristic.get_num_added_conjunctions() == 0)
		return Result::UNMODIFIED;
	heuristic.remove_all_conjunctions();
	auto &cached_result = const_cast<HeuristicCache &>(eval_context.get_cache())[&heuristic];
	if (!cached_result.is_uninitialized())
		cached_result = EvaluationResult();
	eval_context.get_result(&heuristic);
	return eval_context.is_heuristic_infinite(&heuristic) ? Result::DEAD_END : Result::MODIFIED;
}

void EscapeLocalMinimaForgetAll::dump_options() const {
	EscapeLocalMinima::dump_options();
}

static auto _parse_escape_local_minima_forget_all(options::OptionParser &parser) -> std::shared_ptr<ConjunctionGenerationStrategy> {
	EscapeLocalMinima::add_options(parser);
	if (parser.help_mode() || parser.dry_run())
		return nullptr;
	return std::make_shared<EscapeLocalMinimaForgetAll>(parser.parse());
}


// strategy plugins

static options::PluginShared<ConjunctionGenerationStrategy> _plugin_generate_initially("generate_initially", _parse_generate_initially);
static options::PluginShared<ConjunctionGenerationStrategy> _plugin_generate_all_bounded_size("generate_all_bounded_size", _parse_generate_all_bounded_size);
static options::PluginShared<ConjunctionGenerationStrategy> _plugin_load_from_file("load_from_file", _parse_load_from_file);
static options::PluginShared<ConjunctionGenerationStrategy> _plugin_read_count_from_file("read_count_from_file", _parse_read_count_from_file);
static options::PluginShared<ConjunctionGenerationStrategy> _plugin_maintain_fixed_size("maintain_fixed_size", _parse_maintain_fixed_size);
static options::PluginShared<ConjunctionGenerationStrategy> _plugin_escape_local_minima("escape_local_minima", _parse_escape_local_minima);
static options::PluginShared<ConjunctionGenerationStrategy> _plugin_escape_local_minima_bounded("escape_local_minima_bounded", _parse_escape_local_minima_bounded);
static options::PluginShared<ConjunctionGenerationStrategy> _plugin_escape_local_minima_forget_all("escape_local_minima_forget_all", _parse_escape_local_minima_forget_all);

static options::PluginTypePlugin<ConjunctionGenerationStrategy> _type_plugin("Conjunction Generation Strategy",
	"Strategies to generate conjunctions for h^CFF and related heuristics before and during search.");

}

#ifdef _MSC_VER
#pragma warning(pop)
#endif
