#ifndef CONJUNCTIONS_SUBSET_GENERATOR_H
#define CONJUNCTIONS_SUBSET_GENERATOR_H

#pragma once

#include "conjunctions.h"

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(default: 4800 4512 4706 4100 4127 4702 4239 4996 4456 4458 4505)
#endif

namespace conjunctions {
struct ConjunctionRange;

template<class Conjunction>
class GeneratorBase;


// Generator

template<class Conjunction>
class ConjunctionSubsetGenerator {
	std::unique_ptr<GeneratorBase<Conjunction>> root;

public:
	explicit ConjunctionSubsetGenerator(const std::vector<Conjunction *> &conjunctions);
	/*
	  We cannot use the default destructor (implicitly or explicitly)
	  here because GeneratorBase is a forward declaration and the
	  incomplete type cannot be destroyed.
	*/
	~ConjunctionSubsetGenerator();

	void add_conjunction(Conjunction &conjunction);
	void remove_conjunction(Conjunction &conjunction);

	auto generate_conjunction_subset(const State &state) const -> std::vector<Conjunction *>;
	auto generate_conjunction_subset(const GlobalState &state) const -> std::vector<Conjunction *>;
	auto generate_conjunction_subset(const std::vector<FactPair> &facts) const -> std::vector<Conjunction *>;
};


// Generator Node classes

template<class Conjunction>
class GeneratorBase {
public:
	virtual ~GeneratorBase() = default;

	virtual auto add_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> = 0;
	virtual auto remove_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> = 0;

	virtual void generate_conjunction_subset(const State &state, std::vector<Conjunction *> &conjunctions) const = 0;
	virtual void generate_conjunction_subset(const GlobalState &state, std::vector<Conjunction *> &conjunctions) const = 0;
	virtual void generate_conjunction_subset(const std::vector<FactPair> &facts, std::vector<Conjunction *> &conjunctions) const = 0;
};

template<class Conjunction>
using GeneratorForVariable = std::pair<int, std::unique_ptr<GeneratorBase<Conjunction>>>;

template<class Conjunction>
class GeneratorForkBinary : public GeneratorBase<Conjunction> {
	GeneratorForVariable<Conjunction> generator1;
	GeneratorForVariable<Conjunction> generator2;
public:
	GeneratorForkBinary(GeneratorForVariable<Conjunction> generator1, GeneratorForVariable<Conjunction> generator2);
	virtual ~GeneratorForkBinary() = default;

	auto add_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> override;
	auto remove_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> override;

	void generate_conjunction_subset(const State &state, std::vector<Conjunction *> &conjunctions) const override;
	void generate_conjunction_subset(const GlobalState &state, std::vector<Conjunction *> &conjunctions) const override;
	void generate_conjunction_subset(const std::vector<FactPair> &facts, std::vector<Conjunction *> &conjunctions) const override;
};

template<class Conjunction>
class GeneratorForkMulti : public GeneratorBase<Conjunction> {
	std::vector<GeneratorForVariable<Conjunction>> children;
public:
	explicit GeneratorForkMulti(std::vector<GeneratorForVariable<Conjunction>> children);
	virtual ~GeneratorForkMulti() = default;

	auto add_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> override;
	auto remove_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> override;

	void generate_conjunction_subset(const State &state, std::vector<Conjunction *> &conjunctions) const override;
	void generate_conjunction_subset(const GlobalState &state, std::vector<Conjunction *> &conjunctions) const override;
	void generate_conjunction_subset(const std::vector<FactPair> &facts, std::vector<Conjunction *> &conjunctions) const override;
};

template<class Conjunction>
class GeneratorSwitchVector : public GeneratorBase<Conjunction> {
	int switch_var_id;
	std::vector<std::unique_ptr<GeneratorBase<Conjunction>>> generator_for_value;
public:
	GeneratorSwitchVector(
		int switch_var_id,
		std::vector<std::unique_ptr<GeneratorBase<Conjunction>>> &&generator_for_value);
	virtual ~GeneratorSwitchVector() = default;

	auto add_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> override;
	auto remove_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> override;

	void generate_conjunction_subset(const State &state, std::vector<Conjunction *> &conjunctions) const override;
	void generate_conjunction_subset(const GlobalState &state, std::vector<Conjunction *> &conjunctions) const override;
	void generate_conjunction_subset(const std::vector<FactPair> &facts, std::vector<Conjunction *> &conjunctions) const override;
};

template<class Conjunction>
class GeneratorSwitchHash : public GeneratorBase<Conjunction> {
	int switch_var_id;
	std::unordered_map<int, std::unique_ptr<GeneratorBase<Conjunction>>> generator_for_value;
public:
	GeneratorSwitchHash(
		int switch_var_id,
		std::unordered_map<int, std::unique_ptr<GeneratorBase<Conjunction>>> &&generator_for_value);
	virtual ~GeneratorSwitchHash() = default;

	auto add_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> override;
	auto remove_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> override;

	void generate_conjunction_subset(const State &state, std::vector<Conjunction *> &conjunctions) const override;
	void generate_conjunction_subset(const GlobalState &state, std::vector<Conjunction *> &conjunctions) const override;
	void generate_conjunction_subset(const std::vector<FactPair> &facts, std::vector<Conjunction *> &conjunctions) const override;
};

template<class Conjunction>
class GeneratorSwitchSingle : public GeneratorBase<Conjunction> {
	int switch_var_id;
	int value;
	std::unique_ptr<GeneratorBase<Conjunction>> generator_for_value;
public:
	GeneratorSwitchSingle(
		int switch_var_id, int value,
		std::unique_ptr<GeneratorBase<Conjunction>> generator_for_value);
	virtual ~GeneratorSwitchSingle() = default;

	auto add_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> override;
	auto remove_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> override;

	void generate_conjunction_subset(const State &state, std::vector<Conjunction *> &conjunctions) const override;
	void generate_conjunction_subset(const GlobalState &state, std::vector<Conjunction *> &conjunctions) const override;
	void generate_conjunction_subset(const std::vector<FactPair> &facts, std::vector<Conjunction *> &conjunctions) const override;
};

template<class Conjunction>
class GeneratorLeafVector : public GeneratorBase<Conjunction> {
	std::vector<Conjunction *> conjunctions;
public:
	explicit GeneratorLeafVector(std::vector<Conjunction *> &&conjunctions);
	virtual ~GeneratorLeafVector() = default;

	auto add_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> override;
	auto remove_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> override;

	void generate_conjunction_subset(const State &state, std::vector<Conjunction *> &conjunctions) const override;
	void generate_conjunction_subset(const GlobalState &state, std::vector<Conjunction *> &conjunctions) const override;
	void generate_conjunction_subset(const std::vector<FactPair> &facts, std::vector<Conjunction *> &conjunctions) const override;
};

template<class Conjunction>
class GeneratorLeafSingle : public GeneratorBase<Conjunction> {
	Conjunction * const conjunction;
public:
	explicit GeneratorLeafSingle(Conjunction &conjunction);
	virtual ~GeneratorLeafSingle() = default;

	auto add_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> override;
	auto remove_conjunction(Conjunction &conjunction, int depth) -> std::pair<bool, std::unique_ptr<GeneratorBase<Conjunction>>> override;

	void generate_conjunction_subset(const State &state, std::vector<Conjunction *> &conjunctions) const override;
	void generate_conjunction_subset(const GlobalState &state, std::vector<Conjunction *> &conjunctions) const override;
	void generate_conjunction_subset(const std::vector<FactPair> &facts, std::vector<Conjunction *> &conjunctions) const override;
};


// Factory

template<class Conjunction>
class ConjunctionSubsetGeneratorFactory {
	using ValuesAndGenerators = std::vector<std::pair<int, std::unique_ptr<GeneratorBase<Conjunction>>>>;

	std::vector<Conjunction *> conjunctions;

	std::unique_ptr<GeneratorBase<Conjunction>> construct_fork(std::vector<GeneratorForVariable<Conjunction>> nodes) const;
	std::unique_ptr<GeneratorBase<Conjunction>> construct_leaf(ConjunctionRange range) const;
	std::unique_ptr<GeneratorBase<Conjunction>> construct_switch(int switch_var_id, ValuesAndGenerators values_and_generators) const;
	std::unique_ptr<GeneratorBase<Conjunction>> construct_recursive(int depth, ConjunctionRange range) const;
public:
	explicit ConjunctionSubsetGeneratorFactory(const std::vector<Conjunction *> &conjunctions);
	~ConjunctionSubsetGeneratorFactory() = default;
	std::unique_ptr<GeneratorBase<Conjunction>> create();
};

}

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif
