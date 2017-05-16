#ifndef CONJUNCTIONS_H
#define CONJUNCTIONS_H

#pragma once

#include <algorithm>
#include <unordered_map>
#include <boost/range/adaptor/reversed.hpp>

#include "../abstract_task.h"
#include "../task_proxy.h"

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(default: 4800 4512 4706 4100 4127 4702 4239 4996 4456 4458 4505)
#endif

namespace conjunctions {

using FactSet = std::vector<FactPair>;

struct Conjunction;
struct BSGNode;

struct Action {
	Action(int id, const OperatorProxy &op, const FactSet &pre, const FactSet &eff, int cost) :
		id(id),
		op(new OperatorProxy(op)),
		pre(pre),
		eff(eff),
		cost(cost) {}

	// constructor for end action only
	Action(const FactSet &pre, const FactSet &eff) :
		id(-1),
		op(nullptr),
		pre(pre),
		eff(eff),
		cost(0) {}

	const int id;
	const std::unique_ptr<OperatorProxy> op;

	const FactSet pre;
	const FactSet eff;

	const int cost;
};

inline auto operator<<(std::ostream &out, const Action &action) -> std::ostream & {
	return out << "Action(" << action.id << ")";
}

struct CounterGroup {
	CounterGroup(const FactSet &regression) :
		group(), regression(regression), regression_conjunctions(), value(-1), cost(0) {}

	CounterGroup(FactSet &&regression) :
		group(), regression(std::move(regression)), regression_conjunctions(), value(-1), cost(0) {}

	std::vector<std::pair<const Action *, Conjunction *>> group;

	FactSet regression;

	// all c in C where c subseteq R(g, a) and c is not dominated by any other conjunction
	std::vector<Conjunction *> regression_conjunctions;

	// counts the number of unsatisfied regression conjunctions
	int value;

	int cost;

	void reset() {
		value = regression_conjunctions.size();
		cost = 0;
	}
};

using CounterGroupIndex = std::vector<CounterGroup>::size_type;

struct Conjunction {

	Conjunction(const FactSet &facts, bool is_subgoal) :
		id(id_counter++),
		facts(facts),
		counter_groups(),
		regression_of(),
		supporters(),
		is_subgoal(is_subgoal),
		initially_true(false),
		cost(-1),
		required_by(),
		supporter_pos(-1) {}

	const int id;

	// facts contained in this conjunction
	FactSet facts;

	// the set of counters attached to this conjunction
	std::unordered_map<const Action *, CounterGroupIndex> counter_groups;

	// the set of counters attached to a pair (g, a) where this conjunction is part of the regression R(g, a)
	std::vector<CounterGroupIndex> regression_of;

	// actions through which this conjunction can be achieved (best supporters)
	std::vector<const Action *> supporters;

	// this is consistent with facts being a subset of the goal, except for the empty conjunction which is used as a dummy precondition for actions without preconditions
	const bool is_subgoal;

	// true in the current state
	bool initially_true;

	int cost;

	// successors in the BSG
	std::vector<int> required_by;

	// predecessor in the BSG
	int supporter_pos;

	auto has_supporter() const -> bool {
		return supporter_pos != -1;
	}

	void reset() {
		supporters.clear();
		initially_true = false;
		cost = -1;
	}

private:
	static int id_counter;
};

inline auto operator<<(std::ostream &out, const Conjunction &conjunction) -> std::ostream & {
	out << "Conjunction(";
	if (!conjunction.facts.empty()) {
		for (size_t i = 0; i < conjunction.facts.size() - 1; ++i)
			out << "(" << conjunction.facts[i].var << ", " << conjunction.facts[i].value << "), ";
		out << "(" << conjunction.facts.back().var << ", " << conjunction.facts.back().value << ")";
	}
	return out << ")";
}

struct BSGNode {

	BSGNode(const Action *action, const std::vector<Conjunction *> &supported_conjunctions, const std::vector<Conjunction *> &precondition_conjunctions, const FactSet &precondition_facts) :
		action(action),
		supported_conjunctions(supported_conjunctions),
		precondition_conjunctions(precondition_conjunctions),
		precondition_facts(precondition_facts),
		visited(false),
		branch_preconditions(),
		branch_deleted_facts() {}

	const Action * const action;
	const std::vector<Conjunction *> supported_conjunctions;
	const std::vector<Conjunction *> precondition_conjunctions;
	const FactSet precondition_facts;

	// stuff for parallel conflict extraction
	bool visited;
	std::unordered_map<FactPair, BSGNode *> branch_preconditions;
	std::unordered_map<FactPair, BSGNode *> branch_deleted_facts;
};

inline auto operator<<(std::ostream &out, const BSGNode &bsg_node) -> std::ostream & {
	out << *bsg_node.action;
	for (auto supported_conjunction : bsg_node.supported_conjunctions)
		out << "\n  " << *supported_conjunction;
	return out;
}

struct BestSupporterGraph {

	std::vector<BSGNode> nodes;
	using size_type = decltype(nodes)::size_type;

	auto operator[](size_type pos) -> BSGNode & {
		return nodes[pos];
	}

	auto operator[](size_type pos) const -> const BSGNode &{
		return nodes[pos];
	}

	// add a new node and return the position where it was inserted
	template<class... Args>
	auto add_node(Args&&... args) -> size_type {
		nodes.emplace_back(std::forward<Args>(args)...);
		return nodes.size() - 1;
	}

	void clear() {
		nodes.clear();
	}

	auto get_end_node() const -> const BSGNode &{
		return nodes.front();
	}

	auto get_end_node() -> BSGNode & {
		return nodes.front();
	}

	auto get_plan_length() const -> int {
		return nodes.size() - 1;
	}

	auto get_num_unique_actions() const -> int {
		auto all_actions = std::vector<const Action *>();
		all_actions.reserve(nodes.size());
		for (const auto &bsg_node : nodes)
			all_actions.push_back(bsg_node.action);
		std::sort(std::begin(all_actions), std::end(all_actions));
		return std::distance(std::begin(all_actions), std::unique(std::begin(all_actions), std::end(all_actions)));
	}

};

inline auto operator<<(std::ostream &out, const BestSupporterGraph &bsg) -> std::ostream & {
	out << "##### RelaxedPlan: #####" << std::endl;
	for (const auto &node : boost::adaptors::reverse(bsg.nodes))
		out << node << "\n";
	return out << "########################" << std::endl;
}

inline auto is_regressable(const Action &action, const FactSet &g) -> bool {
	const auto &pre = action.pre;
	const auto &eff = action.eff;

	assert(std::is_sorted(pre.begin(), pre.end()) && "fact sets should always be sorted!");
	assert(std::is_sorted(pre.begin(), pre.end()) && "fact sets should always be sorted!");
	assert(std::is_sorted(g.begin(), g.end()) && "fact sets should always be sorted!");

	// check conditions (i) and (ii)
	auto g_iterator = g.begin();
	auto a_achieves_part_of_g = false;
	for (const auto &fact : eff) {
		while (g_iterator != g.end() && g_iterator->var < fact.var)
			++g_iterator;
		if (g_iterator == g.end())
			break;
		if (fact.var == g_iterator->var) {
			// if eff contradicts g, the regression is undefined
			if (fact.value != g_iterator->value)
				return false;
			a_achieves_part_of_g = true;
		}
	}
	if (!a_achieves_part_of_g)
		return false;

	// check condition (iii)
	g_iterator = g.begin();
	for (const auto &fact : pre) {
		while (g_iterator != g.end() && g_iterator->var < fact.var)
			++g_iterator;
		if (g_iterator == g.end())
			break;
		if (fact.var == g_iterator->var && fact.value != g_iterator->value) {
			auto eff_defined = false;
			for (const auto &eff_fact : eff) {
				if (eff_fact.var == fact.var) {
					eff_defined = true;
					break;
				}
			}
			if (!eff_defined)
				return false;
		}
	}
	return true;
}

inline auto compute_regression(const Action &action, const FactSet &g) -> FactSet {
	const auto &pre = action.pre;
	const auto &eff = action.eff;

	assert(std::is_sorted(pre.begin(), pre.end()) && "fact sets should always be sorted!");
	assert(std::is_sorted(pre.begin(), pre.end()) && "fact sets should always be sorted!");
	assert(std::is_sorted(g.begin(), g.end()) && "fact sets should always be sorted!");

	auto tmp = FactSet();
	auto regression = FactSet();
	std::set_difference(g.begin(), g.end(), eff.begin(), eff.end(), std::back_inserter(tmp));
	std::set_union(tmp.begin(), tmp.end(), pre.begin(), pre.end(), std::back_inserter(regression));

	assert(std::is_sorted(regression.begin(), regression.end()) && "fact sets should always be sorted!");

	return regression;
}

#ifdef _MSC_VER
#pragma warning(pop)
#endif
}

#endif
