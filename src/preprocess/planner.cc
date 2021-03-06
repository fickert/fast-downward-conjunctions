/* Main file, keeps all important variables.
 * Calls functions from "helper_functions" to read in input (variables, operators,
 * goals, initial state),
 * then calls functions to build causal graph, domain_transition_graphs and
 * successor generator
 * finally prints output to file "output"
 */

#include "helper_functions.h"
#include "successor_generator.h"
#include "causal_graph.h"
#include "domain_transition_graph.h"
#include "state.h"
#include "mutex_group.h"
#include "operator.h"
#include "axiom.h"
#include "variable.h"
#include <iostream>
using namespace std;

int main(int argc, const char **) {
    bool metric;
    vector<Variable *> variables;
    vector<Variable> internal_variables;
    State initial_state;
    vector<pair<Variable *, int>> goals;
    vector<MutexGroup> mutexes;
    vector<Operator> operators;
    vector<Axiom> axioms;

    if (argc != 1) {
        cout << "*** do not perform relevance analysis ***" << endl;
        g_do_not_prune_variables = true;
    }

    read_preprocessed_problem_description
        (cin, metric, internal_variables, variables, mutexes, initial_state, goals, operators, axioms);
    //dump_preprocessed_problem_description
    //  (variables, initial_state, goals, operators, axioms);

    cout << "Building causal graph..." << endl;
    CausalGraph causal_graph(variables, operators, axioms, goals);
    const vector<Variable *> &ordering = causal_graph.get_variable_ordering();

    // Remove unnecessary effects from operators and axioms, then remove
    // operators and axioms without effects.
    strip_mutexes(mutexes);
    strip_operators(operators);
    strip_axioms(axioms);

    // Output some task statistics
    int facts = 0;
    int derived_vars = 0;
    for (Variable *var : ordering) {
        facts += var->get_range();
        if (var->is_derived())
            derived_vars++;
    }
    cout << "Preprocessor facts: " << facts << endl;
    cout << "Preprocessor derived variables: " << derived_vars << endl;

    // Calculate the problem size
    int task_size = ordering.size() + facts + goals.size();

    for (const MutexGroup &mutex : mutexes)
        task_size += mutex.get_encoding_size();

    for (const Operator &op : operators)
        task_size += op.get_encoding_size();

    for (const Axiom &axiom : axioms)
        task_size += axiom.get_encoding_size();

    cout << "Preprocessor task size: " << task_size << endl;

    cout << "Writing output..." << endl;
    generate_cpp_input(ordering, metric,
                       mutexes, initial_state, goals,
                       operators, axioms);
    cout << "done" << endl;
}
