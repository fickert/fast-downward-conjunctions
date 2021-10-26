#include "conjunctions_subset_generator.h"

#include "../globals.h"
#include "../utils/collections.h"
#include "novelty_heuristic.h"

namespace conjunctions {

static constexpr auto HASH_SET_WEIGHT = 2;

template <class Conjunction>
ConjunctionSubsetGenerator<Conjunction>::ConjunctionSubsetGenerator(const std::vector<Conjunction *> &conjunctions)
	: root(ConjunctionSubsetGeneratorFactory<Conjunction>(conjunctions).create()) {}

template <class Conjunction>
ConjunctionSubsetGenerator<Conjunction>::~ConjunctionSubsetGenerator() = default;

template <class Conjunction>
void ConjunctionSubsetGenerator<Conjunction>::add_conjunction(Conjunction &conjunction) {
	root->add_conjunction(conjunction, 0);
}

template <class Conjunction>
void ConjunctionSubsetGenerator<Conjunction>::remove_conjunction(Conjunction &conjunction) {
	root->remove_conjunction(conjunction, 0);
}

template <class Conjunction>
auto ConjunctionSubsetGenerator<Conjunction>::generate_conjunction_subset(const State &state) const -> std::vector<Conjunction *> {
	auto conjunctions = std::vector<Conjunction *>();
	root->generate_conjunction_subset(state, conjunctions);
	return conjunctions;
}

template <class Conjunction>
auto ConjunctionSubsetGenerator<Conjunction>::generate_conjunction_subset(const GlobalState &state) const -> std::vector<Conjunction *> {
	auto conjunctions = std::vector<Conjunction *>();
	root->generate_conjunction_subset(state, conjunctions);
	return conjunctions;
}

template <class Conjunction>
auto ConjunctionSubsetGenerator<Conjunction>::generate_conjunction_subset(const std::vector<FactPair> &facts) const -> std::vector<Conjunction *> {
	auto conjunctions = std::vector<Conjunction *>();
	root->generate_conjunction_subset(facts, conjunctions);
	return conjunctions;
}

template class ConjunctionSubsetGenerator<Conjunction>;
template class ConjunctionSubsetGenerator<novelty::Conjunction>;

template<class Conjunction>
static auto create_recursive(Conjunction &conjunction, int depth) -> std::unique_ptr<GeneratorBase<Conjunction>> {
	if (depth == static_cast<int>(conjunction.facts.size()))
		return std::make_unique<GeneratorLeafSingle<Conjunction>>(conjunction);
	return std::make_unique<GeneratorSwitchSingle<Conjunction>>(conjunction.facts[depth].var, conjunction.facts[depth].value, create_recursive(conjunction, depth + 1));
}

template<class Conjunction>
static void add_and_update_if_necessary(std::unique_ptr<GeneratorBase<Conjunction>> &generator, Conjunction &conjunction, int depth) {
	auto [changed, updated_generator] = generator->add_conjunction(conjunction, depth);
	if (changed) {
		assert(updated_generator);
		generator = std::move(updated_generator);
	}
	assert(generator);
}

template<class Conjunction>
static auto remove_and_update_if_necessary(std::unique_ptr<GeneratorBase<Conjunction>> &generator, Conjunction &conjunction, int depth) -> bool {
	auto [changed, updated_generator] = generator->remove_conjunction(conjunction, depth);
	if (!changed)
		return false;
	if (updated_generator) {
		generator = std::move(updated_generator);
		return false;
	}
	return true;
}

template<class Conjunction>
GeneratorForkBinary<Conjunction>::GeneratorForkBinary(GeneratorForVariable<Conjunction> generator1, GeneratorForVariable<Conjunction> generator2)
	: generator1(move(generator1)),
	  generator2(move(generator2)) {
	/* There is no reason to use a fork if only one of the generators exists.
	   Use the existing generator directly if one of them exists or a nullptr
	   otherwise. */
	assert(this->generator1.second);
	assert(this->generator2.second);
	assert(this->generator1.first != -1);
	assert(this->generator2.first == -1 || this->generator2.first > this->generator1.first);
}

template<class Conjunction>
auto GeneratorForkBinary<Conjunction>::add_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> {
	const auto build_combined_generators = [this, &conjunction, depth](int var) {
		auto generators = std::vector<GeneratorForVariable<Conjunction>>();
		generators.reserve(3);
		generators.emplace_back(std::move(generator1));
		generators.emplace_back(std::move(generator2));
		generators.emplace_back(var, create_recursive(conjunction, depth));
		std::sort(std::begin(generators), std::end(generators), [](const auto &lhs, const auto &rhs) {
			return rhs.first == -1 || (lhs.first != -1 && lhs.first < rhs.first);
		});
		return generators;
	};

	if (depth == static_cast<int>(conjunction.facts.size())) {
		assert(generator2.first != -1);
		return {true, std::make_unique<GeneratorForkMulti<Conjunction>>(build_combined_generators(-1))};
	}

	if (conjunction.facts[depth].var == generator1.first) {
		add_and_update_if_necessary(generator1.second, conjunction, depth);
		return {false, nullptr};
	}
	if (conjunction.facts[depth].var == generator2.first) {
		add_and_update_if_necessary(generator2.second, conjunction, depth);
		return {false, nullptr};
	}

	return {true, std::make_unique<GeneratorForkMulti<Conjunction>>(build_combined_generators(conjunction.facts[depth].var))};
}

template<class Conjunction>
auto GeneratorForkBinary<Conjunction>::remove_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> {
	assert(generator1.first != -1);
	assert(generator2.first == -1 || generator2.first > generator1.first);
	if (depth == static_cast<int>(conjunction.facts.size())) {
		assert(generator2.first == -1);
		assert(dynamic_cast<GeneratorLeafSingle<Conjunction> *>(generator2.second.get()));
		return {true, std::move(generator1.second)};
	}

	if (generator1.first == conjunction.facts[depth].var) {
		if (remove_and_update_if_necessary(generator1.second, conjunction, depth))
			return {true, std::move(generator2.second)};
	} else {
		assert(generator2.first == conjunction.facts[depth].var);
		if (remove_and_update_if_necessary(generator2.second, conjunction, depth))
			return {true, std::move(generator1.second)};
	}
	return {false, nullptr};
}

template<class Conjunction>
void GeneratorForkBinary<Conjunction>::generate_conjunction_subset(const State &state, std::vector<Conjunction *> &conjunctions) const {
	generator1.second->generate_conjunction_subset(state, conjunctions);
	generator2.second->generate_conjunction_subset(state, conjunctions);
}

template<class Conjunction>
void GeneratorForkBinary<Conjunction>::generate_conjunction_subset(const GlobalState &state, std::vector<Conjunction *> &conjunctions) const {
	generator1.second->generate_conjunction_subset(state, conjunctions);
	generator2.second->generate_conjunction_subset(state, conjunctions);
}

template<class Conjunction>
void GeneratorForkBinary<Conjunction>::generate_conjunction_subset(const std::vector<FactPair> &facts, std::vector<Conjunction *> &conjunctions) const {
	generator1.second->generate_conjunction_subset(facts, conjunctions);
	generator2.second->generate_conjunction_subset(facts, conjunctions);
}

template class GeneratorForkBinary<Conjunction>;
template class GeneratorForkBinary<novelty::Conjunction>;

template<class Conjunction>
GeneratorForkMulti<Conjunction>::GeneratorForkMulti(std::vector<GeneratorForVariable<Conjunction>> children)
	: children(move(children)) {
	/* Note that we permit 0-ary forks as a way to define empty
	   successor generators (for tasks with no operators). It is
	   the responsibility of the factory code to make sure they
	   are not generated in other circumstances. */
	assert(this->children.empty() || this->children.size() >= 2);
	assert(std::is_sorted(std::begin(this->children), std::end(this->children), [](const auto &lhs, const auto &rhs) {
		return rhs.first == -1 || (lhs.first != -1 && lhs.first < rhs.first);
	}));
	assert(std::unique(std::begin(this->children), std::end(this->children), [](const auto &lhs, const auto &rhs) {
		return rhs.first ==  lhs.first;
	}) == std::end(this->children));
}

template<class Conjunction>
auto GeneratorForkMulti<Conjunction>::add_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> {
	if (depth == static_cast<int>(conjunction.facts.size())) {
		assert(children.empty() || children.back().first != -1);
		children.emplace_back(-1, std::make_unique<GeneratorLeafSingle<Conjunction>>(conjunction));
		return {false, nullptr};
	}

	assert(!children.empty() && "not implemented");
	auto pos = std::lower_bound(std::begin(children), std::end(children), conjunction.facts[depth].var, [](const auto &child, const auto &var) {
		assert(var != -1);
		return child.first != -1 && child.first < var;
	});
	if (pos != std::end(children) && pos->first == conjunction.facts[depth].var) {
		// child should be a switch node
		add_and_update_if_necessary(pos->second, conjunction, depth);
		return {false, nullptr};
	}

	children.emplace(pos, std::make_pair(conjunction.facts[depth].var, create_recursive(conjunction, depth)));
	return {false, nullptr};
}

template<class Conjunction>
auto GeneratorForkMulti<Conjunction>::remove_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> {
	const auto transform_if_necessary = [this]() -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> {
		if (children.size() == 1)
			return {true, std::move(children.front().second)};
		if (children.size() == 2)
			return {true, std::make_unique<GeneratorForkBinary<Conjunction>>(std::move(children.front()), std::move(children.back()))};
		return {false, nullptr};
	};

	if (depth == static_cast<int>(conjunction.facts.size())) {
		assert(!children.empty() && children.back().first == -1);
		assert(dynamic_cast<GeneratorLeafSingle<Conjunction> *>(children.back().second.get()));
		children.pop_back();
		assert(!children.empty());
		return transform_if_necessary();
	}

	auto pos = std::lower_bound(std::begin(children), std::end(children), conjunction.facts[depth].var, [](const auto &child, const auto &var) {
		assert(var != -1);
		return child.first != -1 && child.first < var;
	});
	assert(pos != std::end(children) && pos->first == conjunction.facts[depth].var);
	if (!remove_and_update_if_necessary(pos->second, conjunction, depth))
		return {false, nullptr};

	children.erase(pos);
	return transform_if_necessary();
}

template<class Conjunction>
void GeneratorForkMulti<Conjunction>::generate_conjunction_subset(const State &state, std::vector<Conjunction *> &conjunctions) const {
	for (const auto &[variable, generator] : children)
		generator->generate_conjunction_subset(state, conjunctions);
}

template<class Conjunction>
void GeneratorForkMulti<Conjunction>::generate_conjunction_subset(const GlobalState &state, std::vector<Conjunction *> &conjunctions) const {
	for (const auto &[variable, generator] : children)
		generator->generate_conjunction_subset(state, conjunctions);
}

template<class Conjunction>
void GeneratorForkMulti<Conjunction>::generate_conjunction_subset(const std::vector<FactPair> &facts, std::vector<Conjunction *> &conjunctions) const {
	for (const auto &[variable, generator] : children)
		generator->generate_conjunction_subset(facts, conjunctions);
}

template class GeneratorForkMulti<Conjunction>;
template class GeneratorForkMulti<novelty::Conjunction>;

template<class Conjunction>
GeneratorSwitchVector<Conjunction>::GeneratorSwitchVector(
	int switch_var_id, std::vector<std::unique_ptr<GeneratorBase<Conjunction>>> &&generator_for_value)
	: switch_var_id(switch_var_id),
	  generator_for_value(move(generator_for_value)) {}

template<class Conjunction>
auto GeneratorSwitchVector<Conjunction>::add_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> {
	if (depth == static_cast<int>(conjunction.facts.size()))
		// a conjunction that is a subset of this generator's children is added
		return {true, std::make_unique<GeneratorForkBinary<Conjunction>>(
			std::make_pair(switch_var_id, std::make_unique<GeneratorSwitchVector<Conjunction>>(switch_var_id, std::move(generator_for_value))),
			std::make_pair(-1, std::make_unique<GeneratorLeafSingle<Conjunction>>(conjunction)))};
	assert(depth < static_cast<int>(conjunction.facts.size()));
	if (conjunction.facts[depth].var != switch_var_id) {
		auto original_generator = std::make_pair(switch_var_id, std::make_unique<GeneratorSwitchVector<Conjunction>>(switch_var_id, std::move(generator_for_value)));
		auto new_generator = std::make_pair(conjunction.facts[depth].var, create_recursive(conjunction, depth));
		return {true, original_generator.first < new_generator.first ?
			std::make_unique<GeneratorForkBinary<Conjunction>>(std::move(original_generator), std::move(new_generator)) :
			std::make_unique<GeneratorForkBinary<Conjunction>>(std::move(new_generator), std::move(original_generator))};
	}
	assert(conjunction.facts[depth].var == switch_var_id);

	auto &generator = generator_for_value[conjunction.facts[depth].value];
	if (generator)
		add_and_update_if_necessary(generator, conjunction, depth + 1);
	else
		generator = create_recursive(conjunction, depth + 1);
	return {false, nullptr};
}

template<class Conjunction>
auto GeneratorSwitchVector<Conjunction>::remove_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> {
	assert(depth < static_cast<int>(conjunction.facts.size()));
	assert(conjunction.facts[depth].var == switch_var_id);

	auto &generator = generator_for_value[conjunction.facts[depth].value];
	assert(generator);

	if (!remove_and_update_if_necessary(generator, conjunction, depth + 1))
		return {false, nullptr};

	generator = nullptr;
	const auto num_generators = std::count_if(std::begin(generator_for_value), std::end(generator_for_value), [](const auto &generator) { return static_cast<bool>(generator); });
	if (num_generators == 1u) {
		const auto generator_it = std::find_if(std::begin(generator_for_value), std::end(generator_for_value), [](const auto &generator) { return static_cast<bool>(generator); });
		assert(generator_it != std::end(generator_for_value));
		return {true, std::make_unique<GeneratorSwitchSingle<Conjunction>>(switch_var_id, std::distance(std::begin(generator_for_value), generator_it), std::move(*generator_it))};
	}

	const auto var_domain = g_root_task()->get_variable_domain_size(switch_var_id);
	const auto vector_bytes = utils::estimate_vector_bytes<std::unique_ptr<GeneratorBase<Conjunction>>>(var_domain);
	const auto hash_bytes = utils::estimate_unordered_map_bytes<int, std::unique_ptr<GeneratorBase<Conjunction>>>(generator_for_value.size() + 1);

	if (HASH_SET_WEIGHT * hash_bytes >= vector_bytes)
		return {false, nullptr};

	auto generator_by_value = std::unordered_map<int, std::unique_ptr<GeneratorBase<Conjunction>>>();
	generator_by_value.reserve(num_generators);
	for (auto i = 0u; i < generator_for_value.size(); ++i)
		if (generator_for_value[i])
			generator_by_value[i] = std::move(generator_for_value[i]);
	assert(static_cast<int>(generator_by_value.size()) == num_generators);
	return {true, std::make_unique<GeneratorSwitchHash<Conjunction>>(switch_var_id, std::move(generator_by_value))};
}

template<class Conjunction>
void GeneratorSwitchVector<Conjunction>::generate_conjunction_subset(const State &state, std::vector<Conjunction *> &conjunctions) const {
	int val = state[switch_var_id].get_value();
	const std::unique_ptr<GeneratorBase<Conjunction>> &generator_for_val = generator_for_value[val];
	if (generator_for_val) {
		generator_for_val->generate_conjunction_subset(state, conjunctions);
	}
}

template<class Conjunction>
void GeneratorSwitchVector<Conjunction>::generate_conjunction_subset(const GlobalState &state, std::vector<Conjunction *> &conjunctions) const {
	int val = state[switch_var_id];
	const std::unique_ptr<GeneratorBase<Conjunction>> &generator_for_val = generator_for_value[val];
	if (generator_for_val) {
		generator_for_val->generate_conjunction_subset(state, conjunctions);
	}
}

template<class Conjunction>
void GeneratorSwitchVector<Conjunction>::generate_conjunction_subset(const std::vector<FactPair> &facts, std::vector<Conjunction *> &conjunctions) const {
	const auto fact_it = std::lower_bound(std::begin(facts), std::end(facts), switch_var_id, [](const auto &fact, const auto &switch_var) { return fact.var < switch_var; });
	if (fact_it == std::end(facts) || fact_it->var != switch_var_id)
		return;
	int val = fact_it->value;
	const std::unique_ptr<GeneratorBase<Conjunction>> &generator_for_val = generator_for_value[val];
	if (generator_for_val) {
		generator_for_val->generate_conjunction_subset(facts, conjunctions);
	}
}

template class GeneratorSwitchVector<Conjunction>;
template class GeneratorSwitchVector<novelty::Conjunction>;

template<class Conjunction>
GeneratorSwitchHash<Conjunction>::GeneratorSwitchHash(
	int switch_var_id,
	std::unordered_map<int, std::unique_ptr<GeneratorBase<Conjunction>>> &&generator_for_value)
	: switch_var_id(switch_var_id),
	  generator_for_value(move(generator_for_value)) {}

template<class Conjunction>
auto GeneratorSwitchHash<Conjunction>::add_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> {
	if (depth == static_cast<int>(conjunction.facts.size()))
		// a conjunction that is a subset of this generator's children is added
		return {true, std::make_unique<GeneratorForkBinary<Conjunction>>(
			std::make_pair(switch_var_id, std::make_unique<GeneratorSwitchHash<Conjunction>>(switch_var_id, std::move(generator_for_value))),
			std::make_pair(-1, std::make_unique<GeneratorLeafSingle<Conjunction>>(conjunction)))};
	assert(depth < static_cast<int>(conjunction.facts.size()));
	if (conjunction.facts[depth].var != switch_var_id) {
		auto original_generator = std::make_pair(switch_var_id, std::make_unique<GeneratorSwitchHash<Conjunction>>(switch_var_id, std::move(generator_for_value)));
		auto new_generator = std::make_pair(conjunction.facts[depth].var, create_recursive(conjunction, depth));
		return {true, original_generator.first < new_generator.first ?
			std::make_unique<GeneratorForkBinary<Conjunction>>(std::move(original_generator), std::move(new_generator)) :
			std::make_unique<GeneratorForkBinary<Conjunction>>(std::move(new_generator), std::move(original_generator))};
	}
	assert(conjunction.facts[depth].var == switch_var_id);

	auto value_it = generator_for_value.find(conjunction.facts[depth].value);
	if (value_it != std::end(generator_for_value)) {
		add_and_update_if_necessary(value_it->second, conjunction, depth + 1);
		return {false, nullptr};
	}

	const auto var_domain = g_root_task()->get_variable_domain_size(switch_var_id);
	const auto vector_bytes = utils::estimate_vector_bytes<std::unique_ptr<GeneratorBase<Conjunction>>>(var_domain);
	const auto hash_bytes = utils::estimate_unordered_map_bytes<int, std::unique_ptr<GeneratorBase<Conjunction>>>(generator_for_value.size() + 1);
	if (HASH_SET_WEIGHT * hash_bytes < vector_bytes) {
		generator_for_value.emplace(conjunction.facts[depth].value, create_recursive(conjunction, depth + 1));
		return {false, nullptr};
	}
	std::vector<std::unique_ptr<GeneratorBase<Conjunction>>> generator_by_value(var_domain);
	for (auto &&[value, generator] : generator_for_value)
		generator_by_value[value] = std::move(generator);
	generator_by_value[conjunction.facts[depth].value] = create_recursive(conjunction, depth + 1);
	return {true, std::make_unique<GeneratorSwitchVector<Conjunction>>(switch_var_id, std::move(generator_by_value))};
}

template<class Conjunction>
auto GeneratorSwitchHash<Conjunction>::remove_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> {
	assert(depth < static_cast<int>(conjunction.facts.size()));
	assert(conjunction.facts[depth].var == switch_var_id);

	auto value_it = generator_for_value.find(conjunction.facts[depth].value);
	assert(value_it != std::end(generator_for_value));

	if (!remove_and_update_if_necessary(value_it->second, conjunction, depth + 1))
		return {false, nullptr};

	generator_for_value.erase(value_it);
	assert(!generator_for_value.empty());
	if (generator_for_value.size() > 1u)
		return {false, nullptr};

	return {true, std::make_unique<GeneratorSwitchSingle<Conjunction>>(switch_var_id, std::begin(generator_for_value)->first, std::move(std::begin(generator_for_value)->second))};
}

template<class Conjunction>
void GeneratorSwitchHash<Conjunction>::generate_conjunction_subset(
	const State &state, std::vector<Conjunction *> &conjunctions) const {
	int val = state[switch_var_id].get_value();
	const auto &child = generator_for_value.find(val);
	if (child != generator_for_value.end()) {
		const std::unique_ptr<GeneratorBase<Conjunction>> &generator_for_val = child->second;
		generator_for_val->generate_conjunction_subset(state, conjunctions);
	}
}

template<class Conjunction>
void GeneratorSwitchHash<Conjunction>::generate_conjunction_subset(
	const GlobalState &state, std::vector<Conjunction *> &conjunctions) const {
	int val = state[switch_var_id];
	const auto &child = generator_for_value.find(val);
	if (child != generator_for_value.end()) {
		const std::unique_ptr<GeneratorBase<Conjunction>> &generator_for_val = child->second;
		generator_for_val->generate_conjunction_subset(state, conjunctions);
	}
}

template<class Conjunction>
void GeneratorSwitchHash<Conjunction>::generate_conjunction_subset(
	const std::vector<FactPair> &facts, std::vector<Conjunction *> &conjunctions) const {
	const auto fact_it = std::lower_bound(std::begin(facts), std::end(facts), switch_var_id, [](const auto &fact, const auto &switch_var) { return fact.var < switch_var; });
	if (fact_it == std::end(facts) || fact_it->var != switch_var_id)
		return;
	int val = fact_it->value;
	const auto &child = generator_for_value.find(val);
	if (child != generator_for_value.end()) {
		const std::unique_ptr<GeneratorBase<Conjunction>> &generator_for_val = child->second;
		generator_for_val->generate_conjunction_subset(facts, conjunctions);
	}
}

template class GeneratorSwitchHash<Conjunction>;
template class GeneratorSwitchHash<novelty::Conjunction>;

template<class Conjunction>
GeneratorSwitchSingle<Conjunction>::GeneratorSwitchSingle(
	int switch_var_id, int value, std::unique_ptr<GeneratorBase<Conjunction>> generator_for_value)
	: switch_var_id(switch_var_id),
	  value(value),
	  generator_for_value(move(generator_for_value)) {
	assert(this->generator_for_value);
}

template<class Conjunction>
auto GeneratorSwitchSingle<Conjunction>::add_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> {
	assert(generator_for_value);
	if (depth == static_cast<int>(conjunction.facts.size()))
		// a conjunction that is a subset of this generator's children is added
		return {true, std::make_unique<GeneratorForkBinary<Conjunction>>(
			std::make_pair(switch_var_id, std::make_unique<GeneratorSwitchSingle<Conjunction>>(switch_var_id, value, std::move(generator_for_value))),
			std::make_pair(-1, std::make_unique<GeneratorLeafSingle<Conjunction>>(conjunction)))};
	assert(depth < static_cast<int>(conjunction.facts.size()));
	if (conjunction.facts[depth].var != switch_var_id) {
		auto original_generator = std::make_pair(switch_var_id, std::make_unique<GeneratorSwitchSingle<Conjunction>>(switch_var_id, value, std::move(generator_for_value)));
		auto new_generator = std::make_pair(conjunction.facts[depth].var, create_recursive(conjunction, depth));
		return {true, original_generator.first < new_generator.first ?
			std::make_unique<GeneratorForkBinary<Conjunction>>(std::move(original_generator), std::move(new_generator)) :
			std::make_unique<GeneratorForkBinary<Conjunction>>(std::move(new_generator), std::move(original_generator))};
	}
	assert(conjunction.facts[depth].var == switch_var_id);
	if (conjunction.facts[depth].value == value) {
		add_and_update_if_necessary(generator_for_value, conjunction, depth + 1);
		assert(generator_for_value);
		return {false, nullptr};
	}

	// variable value doesn't match the single one for this switch, create new switch node for both
	const auto var_domain = g_root_task()->get_variable_domain_size(switch_var_id);
	const auto vector_bytes = utils::estimate_vector_bytes<std::unique_ptr<GeneratorBase<Conjunction>>>(var_domain);
	const auto hash_bytes = utils::estimate_unordered_map_bytes<int, std::unique_ptr<GeneratorBase<Conjunction>>>(2);
	if (HASH_SET_WEIGHT * hash_bytes < vector_bytes) {
		auto generator_by_value = std::unordered_map<int, std::unique_ptr<GeneratorBase<Conjunction>>>();
		generator_by_value.emplace(value, std::move(generator_for_value));
		generator_by_value.emplace(conjunction.facts[depth].value, create_recursive(conjunction, depth + 1));
		return {true, std::make_unique<GeneratorSwitchHash<Conjunction>>(switch_var_id, std::move(generator_by_value))};
	}
	std::vector<std::unique_ptr<GeneratorBase<Conjunction>>> generator_by_value(var_domain);
	generator_by_value[value] = std::move(generator_for_value);
	generator_by_value[conjunction.facts[depth].value] = create_recursive(conjunction, depth + 1);
	return {true, std::make_unique<GeneratorSwitchVector<Conjunction>>(switch_var_id, std::move(generator_by_value))};
}

template<class Conjunction>
auto GeneratorSwitchSingle<Conjunction>::remove_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> {
	assert(depth < static_cast<int>(conjunction.facts.size()));
	assert(conjunction.facts[depth].var == switch_var_id);
	assert(conjunction.facts[depth].value == value);
	auto [changed, updated_generator] = generator_for_value->remove_conjunction(conjunction, depth + 1);
	if (changed) {
		if (!updated_generator)
			return {true, nullptr};
		generator_for_value = std::move(updated_generator);
	}
	return {false, nullptr};
}

template<class Conjunction>
void GeneratorSwitchSingle<Conjunction>::generate_conjunction_subset(
	const State &state, std::vector<Conjunction *> &conjunctions) const {
	if (value == state[switch_var_id].get_value()) {
		generator_for_value->generate_conjunction_subset(state, conjunctions);
	}
}

template<class Conjunction>
void GeneratorSwitchSingle<Conjunction>::generate_conjunction_subset(
	const GlobalState &state, std::vector<Conjunction *> &conjunctions) const {
	if (value == state[switch_var_id]) {
		generator_for_value->generate_conjunction_subset(state, conjunctions);
	}
}

template<class Conjunction>
void GeneratorSwitchSingle<Conjunction>::generate_conjunction_subset(
	const std::vector<FactPair> &facts, std::vector<Conjunction *> &conjunctions) const {
	const auto fact_it = std::lower_bound(std::begin(facts), std::end(facts), switch_var_id, [](const auto &fact, const auto &switch_var) { return fact.var < switch_var; });
	if (fact_it == std::end(facts) || fact_it->var != switch_var_id)
		return;
	if (value == fact_it->value) {
		generator_for_value->generate_conjunction_subset(facts, conjunctions);
	}
}

template class GeneratorSwitchSingle<Conjunction>;
template class GeneratorSwitchSingle<novelty::Conjunction>;

template<class Conjunction>
GeneratorLeafVector<Conjunction>::GeneratorLeafVector(std::vector<Conjunction *> &&conjunctions)
	: conjunctions(move(conjunctions)) {}

template<class Conjunction>
auto GeneratorLeafVector<Conjunction>::add_conjunction(Conjunction &, int) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> {
	std::cerr << "GeneratorLeafVector not implemented for conjunctions" << std::endl;
	utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
}

template<class Conjunction>
auto GeneratorLeafVector<Conjunction>::remove_conjunction(Conjunction &, int) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> {
	std::cerr << "GeneratorLeafVector not implemented for conjunctions" << std::endl;
	utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
}

template<class Conjunction>
void GeneratorLeafVector<Conjunction>::generate_conjunction_subset(
	const State &, std::vector<Conjunction *> &conjunctions) const {
	conjunctions.insert(std::end(conjunctions), std::begin(this->conjunctions), std::end(this->conjunctions));
}

template<class Conjunction>
void GeneratorLeafVector<Conjunction>::generate_conjunction_subset(
	const GlobalState &, std::vector<Conjunction *> &conjunctions) const {
	conjunctions.insert(std::end(conjunctions), std::begin(this->conjunctions), std::end(this->conjunctions));
}

template<class Conjunction>
void GeneratorLeafVector<Conjunction>::generate_conjunction_subset(
	const std::vector<FactPair> &, std::vector<Conjunction *> &conjunctions) const {
	conjunctions.insert(std::end(conjunctions), std::begin(this->conjunctions), std::end(this->conjunctions));
}

template class GeneratorLeafVector<Conjunction>;
template class GeneratorLeafVector<novelty::Conjunction>;

template<class Conjunction>
GeneratorLeafSingle<Conjunction>::GeneratorLeafSingle(Conjunction &conjunction)
	: conjunction(&conjunction) {}

template<class Conjunction>
auto GeneratorLeafSingle<Conjunction>::add_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> {
	assert(depth < static_cast<int>(conjunction.facts.size()));
	return {true, std::make_unique<GeneratorForkBinary<Conjunction>>(
		std::make_pair(conjunction.facts[depth].var, create_recursive(conjunction, depth)),
		std::make_pair(-1, std::make_unique<GeneratorLeafSingle<Conjunction>>(*this->conjunction)))};
}

template<class Conjunction>
#ifndef NDEBUG
auto GeneratorLeafSingle<Conjunction>::remove_conjunction(Conjunction &conjunction, int) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> {
#else
auto GeneratorLeafSingle<Conjunction>::remove_conjunction(Conjunction &, int) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> {
#endif
	assert(conjunction.facts == this->conjunction->facts);
	assert(&conjunction == this->conjunction);
	return {true, nullptr};
}

template<class Conjunction>
void GeneratorLeafSingle<Conjunction>::generate_conjunction_subset(
	const State &, std::vector<Conjunction *> &conjunctions) const {
	conjunctions.push_back(conjunction);
}

template<class Conjunction>
void GeneratorLeafSingle<Conjunction>::generate_conjunction_subset(
	const GlobalState &, std::vector<Conjunction *> &conjunctions) const {
	conjunctions.push_back(conjunction);
}

template<class Conjunction>
void GeneratorLeafSingle<Conjunction>::generate_conjunction_subset(
	const std::vector<FactPair> &, std::vector<Conjunction *> &conjunctions) const {
	conjunctions.push_back(conjunction);
}

template class GeneratorLeafSingle<Conjunction>;
template class GeneratorLeafSingle<novelty::Conjunction>;


struct ConjunctionRange {
	int begin;
	int end;

	ConjunctionRange(int begin, int end)
		: begin(begin), end(end) {}

	bool empty() const {
		return begin == end;
	}

	int span() const {
		return end - begin;
	}
};

enum class GroupConjunctionsBy {
	VAR,
	VALUE
};

template<class Conjunction>
class ConjunctionGrouper {
	const std::vector<Conjunction *> &conjunctions;
	const int depth;
	const GroupConjunctionsBy group_by;
	ConjunctionRange range;

	int get_current_group_key() const {
		const auto &facts = conjunctions[range.begin]->facts;
		if (group_by == GroupConjunctionsBy::VAR) {
			return depth == static_cast<int>(facts.size()) ? -1 : facts[depth].var;
		} else {
			assert(group_by == GroupConjunctionsBy::VALUE);
			return facts[depth].value;
		}
	}
public:
	explicit ConjunctionGrouper(
		const std::vector<Conjunction *> &conjunctions,
		int depth,
		GroupConjunctionsBy group_by,
		ConjunctionRange range)
		: conjunctions(conjunctions), depth(depth), group_by(group_by), range(range) {}

	bool done() const {
		return range.empty();
	}

	std::pair<int, ConjunctionRange> next() {
		assert(!range.empty());
		int key = get_current_group_key();
		int group_begin = range.begin;
		do {
			++range.begin;
		} while (!range.empty() && get_current_group_key() == key);
		ConjunctionRange group_range(group_begin, range.begin);
		return std::make_pair(key, group_range);
	}
};

template class ConjunctionGrouper<Conjunction>;
template class ConjunctionGrouper<novelty::Conjunction>;


template<class Conjunction>
ConjunctionSubsetGeneratorFactory<Conjunction>::ConjunctionSubsetGeneratorFactory(const std::vector<Conjunction *> &conjunctions)
	: conjunctions(conjunctions) {}

template<class Conjunction>
auto ConjunctionSubsetGeneratorFactory<Conjunction>::construct_fork(
	std::vector<GeneratorForVariable<Conjunction>> nodes) const -> std::unique_ptr<GeneratorBase<Conjunction>> {
	int size = nodes.size();
	if (size == 1) {
		return move(nodes.at(0).second);
	} else if (size == 2) {
		return std::make_unique<GeneratorForkBinary<Conjunction>>(
			move(nodes.at(0)), move(nodes.at(1)));
	} else {
		/* This general case includes the case size == 0, which can
		   (only) happen for the root for tasks with no operators. */
		return std::make_unique<GeneratorForkMulti<Conjunction>>(move(nodes));
	}
}

template<class Conjunction>
auto ConjunctionSubsetGeneratorFactory<Conjunction>::construct_leaf(
	ConjunctionRange range) const -> std::unique_ptr<GeneratorBase<Conjunction>> {
	assert(!range.empty());
	auto current_conjunctions = std::vector<Conjunction *>(
		std::next(std::begin(conjunctions), range.begin),
		std::next(std::begin(conjunctions), range.end));

	assert(current_conjunctions.size() == 1 && "there should never be duplicate conjunctions");
	if (current_conjunctions.size() == 1) {
		return std::make_unique<GeneratorLeafSingle<Conjunction>>(*current_conjunctions.front());
	} else {
		return std::make_unique<GeneratorLeafVector<Conjunction>>(std::move(current_conjunctions));
	}
}

template<class Conjunction>
auto ConjunctionSubsetGeneratorFactory<Conjunction>::construct_switch(
	int switch_var_id, ValuesAndGenerators values_and_generators) const -> std::unique_ptr<GeneratorBase<Conjunction>> {
	VariablesProxy variables = TaskProxy(*g_root_task()).get_variables();
	int var_domain = variables[switch_var_id].get_domain_size();
	int num_children = values_and_generators.size();

	assert(num_children > 0);

	if (num_children == 1) {
		int value = values_and_generators[0].first;
		std::unique_ptr<GeneratorBase<Conjunction>> generator = move(values_and_generators[0].second);
		return std::make_unique<GeneratorSwitchSingle<Conjunction>>(
			switch_var_id, value, move(generator));
	}

	int vector_bytes = utils::estimate_vector_bytes<std::unique_ptr<GeneratorBase<Conjunction>>>(var_domain);
	int hash_bytes = utils::estimate_unordered_map_bytes<int, std::unique_ptr<GeneratorBase<Conjunction>>>(num_children);
	if (hash_bytes < vector_bytes) {
		std::unordered_map<int, std::unique_ptr<GeneratorBase<Conjunction>>> generator_by_value;
		for (auto &item : values_and_generators)
			generator_by_value[item.first] = move(item.second);
		return std::make_unique<GeneratorSwitchHash<Conjunction>>(
			switch_var_id, move(generator_by_value));
	} else {
		std::vector<std::unique_ptr<GeneratorBase<Conjunction>>> generator_by_value(var_domain);
		for (auto &item : values_and_generators)
			generator_by_value[item.first] = move(item.second);
		return std::make_unique<GeneratorSwitchVector<Conjunction>>(
			switch_var_id, move(generator_by_value));
	}
}

template<class Conjunction>
auto ConjunctionSubsetGeneratorFactory<Conjunction>::construct_recursive(
	int depth, ConjunctionRange range) const -> std::unique_ptr<GeneratorBase<Conjunction>> {
	std::vector<GeneratorForVariable<Conjunction>> nodes;
	ConjunctionGrouper<Conjunction> grouper_by_var(
		conjunctions, depth, GroupConjunctionsBy::VAR, range);
	while (!grouper_by_var.done()) {
		auto var_group = grouper_by_var.next();
		int var = var_group.first;
		ConjunctionRange var_range = var_group.second;

		if (var == -1) {
			// Handle a group of immediately applicable operators.
			nodes.emplace_back(var, construct_leaf(var_range));
		} else {
			// Handle a group of operators sharing the first precondition variable.
			ValuesAndGenerators values_and_generators;
			ConjunctionGrouper<Conjunction> grouper_by_value(
				conjunctions, depth, GroupConjunctionsBy::VALUE, var_range);
			while (!grouper_by_value.done()) {
				auto value_group = grouper_by_value.next();
				int value = value_group.first;
				ConjunctionRange value_range = value_group.second;

				values_and_generators.emplace_back(
					value, construct_recursive(depth + 1, value_range));
			}

			nodes.emplace_back(var, construct_switch(var, move(values_and_generators)));
		}
	}
	return construct_fork(move(nodes));
}

template<class Conjunction>
auto ConjunctionSubsetGeneratorFactory<Conjunction>::create() -> std::unique_ptr<GeneratorBase<Conjunction>> {
	std::sort(std::begin(conjunctions), std::end(conjunctions), [](const auto &lhs, const auto &rhs) {
		assert(lhs->facts != rhs->facts);
		return lhs->facts < rhs->facts;
	});

	ConjunctionRange full_range(0, conjunctions.size());
	std::unique_ptr<GeneratorBase<Conjunction>> root = construct_recursive(0, full_range);
	return root;
}

template class ConjunctionSubsetGeneratorFactory<Conjunction>;
template class ConjunctionSubsetGeneratorFactory<novelty::Conjunction>;
}
