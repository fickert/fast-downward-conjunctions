#include "successor_generator.h"

#include "successor_generator_factory.h"
#include "successor_generator_internals.h"

#include "../abstract_task.h"
#include "../global_state.h"
#include "../task_proxy.h"

using namespace std;

namespace successor_generator {
SuccessorGenerator::SuccessorGenerator(const TaskProxy &task_proxy)
    : root(SuccessorGeneratorFactory(task_proxy).create()) {
}

SuccessorGenerator::~SuccessorGenerator() = default;

void SuccessorGenerator::generate_applicable_ops(
    const State &state, vector<const GlobalOperator *> &applicable_ops) const {
    root->generate_applicable_ops(state, applicable_ops);
}

void SuccessorGenerator::generate_applicable_ops(
    const GlobalState &state, vector<const GlobalOperator *> &applicable_ops) const {
    root->generate_applicable_ops(state, applicable_ops);
}
}
