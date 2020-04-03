#ifndef RED_BLACK_RED_BLACK_OPERATOR_H
#define RED_BLACK_RED_BLACK_OPERATOR_H


#include <vector>
#include <set>

#include "../global_operator.h"

namespace red_black {

typedef std::pair<int, int> assignment;
typedef std::set<assignment> partial_assignment;
typedef std::pair<partial_assignment, partial_assignment> sas_action;

class RedBlackOperator {
	partial_assignment red_precondition, black_precondition, red_effect, black_effect;
	int op_no;
public:
	RedBlackOperator(int _op_no);
	virtual ~RedBlackOperator();

	void set_black_pre_eff(const std::vector<bool>& black_vars);
	const partial_assignment& get_red_precondition() const { return red_precondition;}
	const partial_assignment& get_black_precondition() const { return black_precondition;}
	const partial_assignment& get_red_effect() const { return red_effect;}
	const partial_assignment& get_black_effect() const { return black_effect;}

	// these functions don't really make sense here... but they don't really make sense anywhere
	static auto get_precondition_for_variable(const GlobalOperator &op, int var) -> int {
		for (const auto &precondition : op.get_preconditions())
			if (precondition.var == var)
				return precondition.val;
		return -1;
	}
	static auto get_effect_for_variable(const GlobalOperator &op, int var) -> int {
		for (const auto &effect : op.get_effects())
			if (effect.var == var)
				return effect.val;
		return -1;
	}

	bool is_red_applicable(const std::vector<int>&  curr_state_buffer) const;
	bool is_applicable(const std::vector<int>&  curr_state_buffer) const;
	bool is_applicable(const GlobalState& state) const;
	void apply(std::vector<int>&  curr_state_buffer) const;
	void dump() const;
	int get_op_no() const { return op_no; }
};

typedef const RedBlackOperator* sas_operator;

}

#endif
