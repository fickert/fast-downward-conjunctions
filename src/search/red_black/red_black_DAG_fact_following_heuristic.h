#ifndef RED_BLACK_RED_BLACK_DAG_FACT_FOLLOWING_HEURISTIC_H
#define RED_BLACK_RED_BLACK_DAG_FACT_FOLLOWING_HEURISTIC_H

#include <vector>
#include <list>
#include "dtg_operators.h"
#include "../heuristics/additive_heuristic.h"
using namespace std;

#include "red_black_operator.h"

namespace red_black {

enum ActionApplicationResult {
    ACTION_APPLICABLE,
    ACTION_NOT_APPLICABLE,
    ACTION_SELF_LOOP
};

enum PreferredOpsType {
    RELAXED,
    EXTRA_BLACK,
    PRECS_UNCHANGED
};

class RedBlackDAGFactFollowingHeuristic : public additive_heuristic::AdditiveHeuristic {
	// from FF heuristic
	typedef std::vector<bool> RelaxedPlan;
	RelaxedPlan relaxed_plan;
	// Patrick: copied from Micheal:
	typedef std::vector<std::vector<int> > ParallelRelaxedPlan;
	ParallelRelaxedPlan parallel_relaxed_plan;

	std::vector<int> curr_state_buffer;
	bool solution_found;
	bool extract_plan;

	std::vector<const GlobalOperator *> suffix_plan;
	std::vector<std::pair<FactPair, int>> subgoals_and_costs;
    std::vector<const GlobalOperator *> red_black_plan;
    vector<DtgOperators *> dtgs_by_transition;
    vector<bool> invertible_vars;  // Keeps invertible variables until black variables are set

    bool applicability_status;
    int num_invertible_vars;
    bool shortest_paths_calculated;
    bool use_black_dag;

    const bool ignore_invertibility;
    const PreferredOpsType preferred_type;
    const bool paint_roots_black;
    const bool applicable_paths_first; // Try to find applicable paths for black variables
    bool use_connected;
    const bool next_red_action_test;
    const bool extract_plan_no_blacks;

	std::vector<int> connected_state_buffer;
	std::vector<int>  black_state_buffer;

    // Keeping operators by pre for red variables only.
    vector<vector<vector<int> > > ops_by_pre;

    // For calculation of the number of reached red preconditions;
    vector<int> ops_num_reached_red_preconditions;

    void create_extended_DTGs();

    void free_red_data();
    void free_initial_data();
    void check_invertibility();
    vector<ConnectivityStatus> connectivity_status;
    void check_connectivity();
    vector<bool> almost_roots;
    // Keeping sas operators for faster checks
    vector<RedBlackOperator*> red_black_sas_operators;

    bool is_already_DAG(const vector<bool>& blacks) const;

    vector<partial_assignment> black_var_deletes;
    void black_DAG_initialize_data_structures();
    bool achieving_black_pre_may_delete_achieved_red_sufficient(int op_no) const;

    int get_best_index(vector<int>& order, int* elements);

    int get_index_of_leftmost_nonzero(vector<int>& order, int* elements) const;

    void paint_red_by_vertex_cover(vector<int>& order, int* elements);

	void print_statistics() const;
    void get_relaxed_plan(const GlobalState &state, relaxation_heuristic::Proposition *goal);

    int get_semi_relaxed_plan_cost(const GlobalState &state);
    int add_red_black_plan_suffix(const GlobalState &state, int h_val);
    int get_next_action(bool skip_currently_red_inapplicable = false);

    void prepare_for_red_fact_following();

    bool is_semi_relaxed_achieved(int var, int val) const;
    void mark_red_precondition(int var, int val);

    void clear_black_marks();

    void dump_current_semi_relaxed_state(bool dump_fact = false) const;

    void remove_all_operators_from_parallel_relaxed_plan();

    void dump_options() const;
    void prepare_DTGs_for_invertibility_check();

    void precalculate_shortest_paths_for_var(size_t var);

    void set_op_as_preferred(const GlobalState &state, int op_no, bool relaxed);
    bool currently_op_prec_unchanged(int op_no) const;
    void apply_action_to_current_state(int op_no);
    ActionApplicationResult apply_action_to_semi_relaxed_state(int op_no, bool check_applicability = true);

    bool are_black_variables_connected();
//    bool are_black_variables_singly_connected();

    void set_black_variables();
    void set_black_variables_vertex_cover();

    void set_red_black_indices();
    void update_marks();
    void update_marks(int );
    int get_black_prv(int op_no, int var);

/////////////////////////////////////////////////////////////////////////////////////////////////////
/// From RedBlackFactFollowingHeuristic

    // Keeping operators by effect for red variables only (used for following the relaxed facts).
    vector<vector<vector<int> > > ops_by_eff;

    // For fast update of the black vars in the red fact following option
    vector<vector<int> > blacks_by_ops;

    void keep_operators_by_effects();
    void set_black_successors_by_ops();

    void mark_red_sufficient(int op_no);

    list<int> red_sufficient_unachieved;
    int get_operator_estimated_conflict_cost_black_reachability(int op_no) const;
/////////////////////////////////////////////////////////////////////////////////////////////////////
    int resolve_conflicts_disconnected(const GlobalState &state);
    int resolve_conflicts_DAG(const GlobalState &state);

    void add_path_for_var_from_to(int var, int from, int to, vector<int>& curr_sequence);
    const vector<int>& get_path_for_var_from_to(int var, int from, int to) const;
    const vector<int>& get_path_for_var(int var);
    vector<int> current_applicable_sequence;

    bool op_all_black_preconditions_reachable(int op_no) const;

protected:
    virtual void initialize();
    virtual int compute_heuristic(const GlobalState &state);

private:
    vector<bool> black_vars;
    vector<int> black_indices;
    vector<int> red_indices;
    DtgOperators* get_dtg(size_t v) const { return dtgs_by_transition[v]; }
    void free_mem();
    void prepare_operators_for_counting_achieved_preconditions();
    void reset_all_marks();
    void set_new_marks_for_state(const GlobalState &state);
    bool is_semi_relaxed_goal_reached();
    int resolve_conflicts(const GlobalState &state);

    RedBlackOperator* get_rb_sas_operator(size_t op_no) const { return red_black_sas_operators[op_no]; }

    const vector<int>& get_ops_by_pre(int var, int val) const { return ops_by_pre[var][val]; }
    void precalculate_variables();

    bool is_shortest_paths_calculated() const { return shortest_paths_calculated; }

    // Getting the number of red and black preconditions from the sas operators, no need to store them
    int get_num_black_preconditions(int op_no) const { return get_rb_sas_operator(op_no)->get_black_precondition().size(); }
    int get_num_red_preconditions(int op_no) const { return get_rb_sas_operator(op_no)->get_red_precondition().size(); }

    bool is_red_effects_only_action(int op_no) const { return get_num_black_preconditions(op_no) == 0; }

    bool is_black_effects_only_action(int op_no) const { return get_num_red_preconditions(op_no) == 0; }
    bool is_mixed_effects_action(int op_no) const { return (!is_red_effects_only_action(op_no) && !is_black_effects_only_action(op_no)); }
    bool is_currently_mixed_effects(int op_no) const;
    void increment_number_reached_red_preconditions(int op_no) { ops_num_reached_red_preconditions[op_no]++; }
    int get_num_reached_red_preconditions(int op_no) const { return ops_num_reached_red_preconditions[op_no]; }
    bool op_all_red_preconditions_reached(int op_no) const { return get_num_reached_red_preconditions(op_no) == get_num_red_preconditions(op_no); }

	static bool test_goal_for_int_vector(const std::vector<int>& state) {
		for (size_t i = 0; i < g_goal.size(); ++i) {
			if (state[g_goal[i].first] != g_goal[i].second) {
				return false;
			}
		}
		return true;
	}
	void check_goal_via_state() {
		if (test_goal_for_int_vector(curr_state_buffer)) {
			// The goal is found!
			std::cout << "Goal found by the heuristic!!" << std::endl;
			solution_found = true;
		}
	}
	static bool is_op_applicable(const GlobalOperator *op, const std::vector<int>& state) {
		for (size_t i = 0; i < op->get_preconditions().size(); ++i) {
			const GlobalCondition &precondition = op->get_preconditions()[i];
			// return false if not applicable		
			if (precondition.val != state[precondition.var])
				return false;
		}
		return true;
	}
	void apply_op(const GlobalOperator *op, std::vector<int>& state) const {
		for (size_t i = 0; i < op->get_effects().size(); i++)
			state[op->get_effects()[i].var] = op->get_effects()[i].val;
	}

    static auto get_cg_neighbours(const CausalGraph &cg, int node) -> vector<int>;

	void mark_preferred_operators_and_relaxed_plan(const GlobalState &state, relaxation_heuristic::Proposition *goal);
	auto ff_compute_heuristic(const GlobalState &state) -> int;
	void apply_while_possible();

	void add_subgoal(int var, int val);
public:
    RedBlackDAGFactFollowingHeuristic(const options::Options &options);
    ~RedBlackDAGFactFollowingHeuristic();

	auto get_last_subgoals_and_costs() const -> std::vector<std::pair<FactPair, int>> override {
		return subgoals_and_costs;
	}

	auto get_last_relaxed_plan() const->std::vector<const GlobalOperator *> override {
		if (!black_indices.empty()) {
			assert(!red_black_plan.empty());
			return red_black_plan;
		}
		auto plan = std::vector<const GlobalOperator *>();
		for (size_t op_no = 0; op_no < relaxed_plan.size(); ++op_no)
			if (relaxed_plan[op_no])
				plan.push_back(task_proxy.get_operators()[op_no].get_global_operator());
		return plan;
	}

	void initialize_if_necessary(StateRegistry &) override {
		initialize();
	}

	auto found_solution() const -> bool override {
		return solution_found;
	}

	auto get_solution() const -> std::vector<const GlobalOperator *> override {
		return suffix_plan;
	}

    static void add_options_to_parser(options::OptionParser &parser);

    bool op_is_enabled(int op_no) const;
    bool is_currently_applicable(const vector<int>& ops, bool skip_black=false);
};

}

#endif
