#include "novelty_heuristic.h"

#include "../option_parser.h"
#include "../plugin.h"
#include "../evaluation_context.h"
#include "conjunctions_heuristic.h"


namespace novelty {
NoveltyHeuristic::NoveltyHeuristic(const Options &opts)
    : Heuristic(opts),
	  conjunctions(),
	  heuristics(opts.get_list<Heuristic *>("heuristics")),
	  conjunctions_heuristic(nullptr) {
    std::cout << "Initializing novelty heuristic..." << std::endl;

    // Initializing fact sets
	for (auto i = 0; i < task->get_num_variables(); ++i)
		for (auto j = 0; j < task->get_variable_domain_size(i); ++j)
			conjunctions.emplace_back(std::make_unique<Conjunction>(FactSet{FactPair(i, j)}, std::max(static_cast<decltype(heuristics.size())>(1), heuristics.size())));
	num_singletons = conjunctions.size();
	auto conjunction_pointers = std::vector<Conjunction *>();
	conjunction_pointers.reserve(conjunctions.size());
	std::transform(std::begin(conjunctions), std::end(conjunctions), std::back_inserter(conjunction_pointers), [](const auto &conjunction) { return conjunction.get(); });
	subset_generator = std::make_unique<conjunctions::ConjunctionSubsetGenerator<Conjunction>>(conjunction_pointers);
}

NoveltyHeuristic::~NoveltyHeuristic() {
	if (conjunctions_heuristic)
		conjunctions_heuristic->unsubscribe(*this);
}

void NoveltyHeuristic::add_conjunction(const FactSet &facts) {
	conjunctions.emplace_back(std::make_unique<Conjunction>(facts, std::max(static_cast<decltype(heuristics.size())>(1), heuristics.size())));
	subset_generator->add_conjunction(*conjunctions.back());
}

void NoveltyHeuristic::remove_conjunction(const FactSet &facts) {
	assert(facts.size() > 1);
	auto pos = std::find_if(std::begin(conjunctions) + num_singletons, std::end(conjunctions), [&facts](const auto &conjunction) { return conjunction->facts == facts; });
	assert(pos != std::end(conjunctions));
	subset_generator->remove_conjunction(**pos);
	conjunctions.erase(pos);
}

void NoveltyHeuristic::remove_all_conjunctions() {
	conjunctions.erase(std::begin(conjunctions) + num_singletons, std::end(conjunctions));
}

void NoveltyHeuristic::reset() {
	for (auto &conjunction : conjunctions)
		conjunction->h_values.assign(conjunction->h_values.size(), -1);
}

void NoveltyHeuristic::update_conjunction(const GlobalState &, Conjunction &conjunction, std::size_t heuristic_index, int value) {
	conjunction.h_values[heuristic_index] = value;
}

auto NoveltyHeuristic::get_h_values(const GlobalState &state) const -> std::vector<int> {
	if (heuristics.empty())
		return {0};
	auto h_values = std::vector<int>();
	h_values.reserve(heuristics.size());
	for (const auto heuristic : heuristics) {
		auto eval_context = EvaluationContext(state);
		if (eval_context.is_heuristic_infinite(heuristic))
			return {};
		h_values.push_back(eval_context.get_heuristic_value(heuristic));
	}
	return h_values;
}

int NoveltyHeuristic::compute_heuristic(const GlobalState &global_state) {
	auto h_values = get_h_values(global_state);
	if (h_values.empty())
		return 1;
	auto novel = false;
	auto subset_conjunctions = subset_generator->generate_conjunction_subset(global_state);
#ifndef NDEBUG
	for (auto &c : conjunctions)
		assert(conjunctions::is_subset(c->facts, global_state) == (std::find(std::begin(subset_conjunctions), std::end(subset_conjunctions), c.get()) != std::end(subset_conjunctions)));
#endif
	for (auto *conjunction : subset_conjunctions) {
		for (auto i = 0u; i < h_values.size(); ++i) {
			if (conjunction->h_values[i] == -1 || conjunction->h_values[i] < h_values[i]) {
				update_conjunction(global_state, *conjunction, i, h_values[i]);
				novel = true;
			}
		}
	}
	return novel ? 0 : 1;
}


MNoveltyHeuristic::MNoveltyHeuristic(const Options &opts)
	: NoveltyHeuristic(opts) {
	add_all_combinations({}, opts.get<int>("m"));
}

void MNoveltyHeuristic::add_all_combinations(const FactSet &base, int max_size) {
	const auto next_fact = [this](const FactPair &f) -> FactPair {
		if (f.value + 1 < task->get_variable_domain_size(f.var))
			return {f.var, f.value + 1};
		if (f.var + 1 < task->get_num_variables())
			return {f.var + 1, 0};
		return {-1, -1};
	};

	assert(base.empty() || base.back().var + 1 < task->get_num_variables());
	for (auto f = FactPair(base.empty() ? 0 : base.back().var + 1, 0); f.var != -1; f = next_fact(f)) {
		auto new_combination = base;
		new_combination.push_back(f);
		assert(std::is_sorted(std::begin(new_combination), std::end(new_combination)));
		if (!conjunctions::contains_mutex(*task, new_combination)) {
			if (new_combination.size() > 1)
				add_conjunction(new_combination);
			if (static_cast<int>(new_combination.size()) < max_size && f.var + 1 < task->get_num_variables())
				add_all_combinations(new_combination, max_size);
		}
	}
}


QuantifiedBothNoveltyHeuristic::QuantifiedBothNoveltyHeuristic(const Options &opts)
	: NoveltyHeuristic(opts) {}

int QuantifiedBothNoveltyHeuristic::compute_heuristic(const GlobalState &global_state) {
	auto h_values = get_h_values(global_state);
	if (h_values.empty())
		return 1;
	// slight redefinition compared to the original paper by Katz et al.:
	// a conjunction counts as novel in this state, if it is novel for any of the heuristics
	// a conjunction counts as non-novel in this state, if it is not novel for all of the heuristics and non-novel (i.e. strictly worse) for any of the heuristics
	auto num_novel = 0;
	auto num_non_novel = 0;
	auto subset_conjunctions = subset_generator->generate_conjunction_subset(global_state);
	for (auto *conjunction : subset_conjunctions) {
		auto is_novel = false;
		auto is_non_novel = false;
		for (auto i = 0u; i < h_values.size(); ++i) {
			if (conjunction->h_values[i] == -1 || conjunction->h_values[i] < h_values[i]) {
				update_conjunction(global_state, *conjunction, i, h_values[i]);
				is_novel = true;
			} else if (conjunction->h_values[i] > h_values[i])
				is_non_novel = true;
		}
		if (is_novel)
			++num_novel;
		else if (is_non_novel)
			++num_non_novel;
	}
	return num_novel > 0 ? static_cast<int>(conjunctions.size()) - num_novel : static_cast<int>(conjunctions.size()) + num_non_novel;
}


static Heuristic *_parse(OptionParser &parser) {
    parser.document_synopsis("Novelty heuristic", "");
    parser.document_property("admissible", "no");
    parser.document_property("consistent", "no");
    parser.document_property("safe", "yes");
    parser.document_property("preferred operators", "no");

	parser.add_list_option<Heuristic *>("heuristics",
		"List of heuristics for novelty calculation", "[]");

    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run())
        return nullptr;
    return new NoveltyHeuristic(opts);
}

static Heuristic *_parse_m(OptionParser &parser) {
    parser.document_synopsis("m-Novelty heuristic", "");
    parser.document_property("admissible", "no");
    parser.document_property("consistent", "no");
    parser.document_property("safe", "yes");
    parser.document_property("preferred operators", "no");

	parser.add_list_option<Heuristic *>("heuristics",
		"List of heuristics for novelty calculation", "[]");

	parser.add_option<int>("m", "Size of tuples to consider for novelty", "2", Bounds("1", "infinity"));

    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run())
        return nullptr;
    return new MNoveltyHeuristic(opts);

}

static Heuristic *_parse_qb(OptionParser &parser) {
	parser.document_synopsis("QB Novelty heuristic", "");
	parser.document_property("admissible", "no");
	parser.document_property("consistent", "no");
	parser.document_property("safe", "yes");
	parser.document_property("preferred operators", "no");

	parser.add_list_option<Heuristic *>("heuristics",
		"List of heuristics for novelty calculation", "[]");

	Heuristic::add_options_to_parser(parser);
	Options opts = parser.parse();
	if (parser.dry_run())
		return nullptr;
	return new QuantifiedBothNoveltyHeuristic(opts);
}

static Plugin<Heuristic> _plugin("novelty", _parse);
static Plugin<Heuristic> _plugin_m("m_novelty", _parse_m);
static Plugin<Heuristic> _plugin_qb("qb_novelty", _parse_qb);
}
