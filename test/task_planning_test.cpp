#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <common_robotics_utilities/print.hpp>
#include <common_robotics_utilities/simple_task_planner.hpp>

#include <gtest/gtest.h>

namespace common_robotics_utilities
{
namespace simple_task_planner
{
namespace
{
class PutInBoxState
{
public:
  PutInBoxState() {}

  PutInBoxState(const int32_t objects_available,
                const bool object_put_away,
                const bool box_open)
    : objects_available_(objects_available),
      object_put_away_(object_put_away),
      box_open_(box_open) {}

  int32_t ObjectsAvailable() const { return objects_available_; }

  bool ObjectPutAway() const { return object_put_away_; }

  bool BoxOpen() const { return box_open_; }

  std::string Print() const
  {
    std::string rep
        = "Objects available: " + std::to_string(objects_available_)
          + " Object put away: "
          + common_robotics_utilities::print::Print(object_put_away_)
          + " Box open: "
          + common_robotics_utilities::print::Print(box_open_);
    return rep;
  }

  bool operator==(const PutInBoxState& other) const
  {
    if (ObjectPutAway() == other.ObjectPutAway() &&
        BoxOpen() == other.BoxOpen())
    {
      return ((ObjectsAvailable() > 0 && other.ObjectsAvailable() > 0) ||
              (ObjectsAvailable() == 0 && other.ObjectsAvailable() == 0) ||
              (ObjectsAvailable() < 0 && other.ObjectsAvailable() < 0));
    }
    else
    {
      return false;
    }
  }

private:
  int32_t objects_available_ = -1;
  bool object_put_away_ = false;
  bool box_open_ = false;
};

struct PutInBoxStateHasher
{
  size_t operator()(const PutInBoxState& state) const
  {
    const uint64_t BOX_OPEN = 0x01;
    const uint64_t BOX_CLOSED = 0x02;
    const uint64_t OBJECT_NOT_PUT_AWAY = 0x04;
    const uint64_t OBJECT_PUT_AWAY = 0x08;
    const uint64_t OBJECTS_AVAILABLE_NONE = 0x10;
    const uint64_t OBJECTS_AVAILABLE_SOME = 0x20;

    uint64_t state_identifier = 0;

    if (state.BoxOpen())
    {
      state_identifier |= BOX_OPEN;
    }
    else
    {
      state_identifier |= BOX_CLOSED;
    }

    if (state.ObjectPutAway())
    {
      state_identifier |= OBJECT_PUT_AWAY;
    }
    else
    {
      state_identifier |= OBJECT_NOT_PUT_AWAY;
    }

    if (state.ObjectsAvailable() > 0)
    {
      state_identifier |= OBJECTS_AVAILABLE_SOME;
    }
    else if (state.ObjectsAvailable() == 0)
    {
      state_identifier |= OBJECTS_AVAILABLE_NONE;
    }

    return static_cast<size_t>(state_identifier);
  }
};

std::ostream& operator<<(std::ostream& strm, const PutInBoxState& state)
{
  strm << state.Print();
  return strm;
}

using PutInBoxStatesWithCosts = OutcomesWithCosts<PutInBoxState>;
using PutInBoxStateContainer = std::vector<PutInBoxState>;
using PutInBoxStatePrimitiveType =
    ActionPrimitiveInterface<PutInBoxState, PutInBoxStateContainer>;
using PutInBoxStatePrimitiveCollection =
    ActionPrimitiveCollection<PutInBoxState, PutInBoxStateContainer>;

class OpenBoxPrimitive : public PutInBoxStatePrimitiveType
{
public:
  bool IsCandidate(const PutInBoxState& state) const override
  {
    return (!state.BoxOpen());
  }

  const std::string& Name() const override { return name_; }

  PutInBoxStatesWithCosts GetOutcomes(const PutInBoxState& state) const override
  {
    if (IsCandidate(state))
    {
      PutInBoxStatesWithCosts outcomes;
      outcomes.emplace_back(
          PutInBoxState(state.ObjectsAvailable(), state.ObjectPutAway(), true),
          1.0);
      return outcomes;
    }
    else
    {
      throw std::invalid_argument("State is not a candidate for primitive");
    }
  }

  PutInBoxStateContainer Execute(const PutInBoxState& state) override
  {
    if (IsCandidate(state))
    {
      return {PutInBoxState(
          state.ObjectsAvailable(), state.ObjectPutAway(), true)};
    }
    else
    {
      throw std::invalid_argument("State is not a candidate for primitive");
    }
  }

private:
  const std::string name_ = "OpenBoxPrimitive";
};

class CloseBoxPrimitive : public PutInBoxStatePrimitiveType
{
public:
  bool IsCandidate(const PutInBoxState& state) const override
  {
    return (state.BoxOpen());
  }

  const std::string& Name() const override { return name_; }

  PutInBoxStatesWithCosts GetOutcomes(const PutInBoxState& state) const override
  {
    if (IsCandidate(state))
    {
      PutInBoxStatesWithCosts outcomes;
      outcomes.emplace_back(
          PutInBoxState(state.ObjectsAvailable(), state.ObjectPutAway(), false),
          1.0);
      return outcomes;
    }
    else
    {
      throw std::invalid_argument("State is not a candidate for primitive");
    }
  }

  PutInBoxStateContainer Execute(const PutInBoxState& state) override
  {
    if (IsCandidate(state))
    {
      return {PutInBoxState(
          state.ObjectsAvailable(), state.ObjectPutAway(), false)};
    }
    else
    {
      throw std::invalid_argument("State is not a candidate for primitive");
    }
  }

private:
  const std::string name_ = "CloseBoxPrimitive";
};

class CheckIfObjectAvailablePrimitive : public PutInBoxStatePrimitiveType
{
public:
  bool IsCandidate(const PutInBoxState& state) const override
  {
    return (state.ObjectsAvailable() < 0);
  }

  const std::string& Name() const override { return name_; }

  PutInBoxStatesWithCosts GetOutcomes(const PutInBoxState& state) const override
  {
    if (IsCandidate(state))
    {
      const PutInBoxState object_available(
          1, state.ObjectPutAway(), state.BoxOpen());
      const PutInBoxState none_available(
          0, state.ObjectPutAway(), state.BoxOpen());

      PutInBoxStatesWithCosts outcomes;
      outcomes.emplace_back(object_available, 0.5);
      outcomes.emplace_back(none_available, 0.5);
      return outcomes;
    }
    else
    {
      throw std::invalid_argument("State is not a candidate for primitive");
    }
  }

  PutInBoxStateContainer Execute(const PutInBoxState& state) override
  {
    if (IsCandidate(state))
    {
      return {PutInBoxState(
          std::abs(state.ObjectsAvailable()), state.ObjectPutAway(),
          state.BoxOpen())};
    }
    else
    {
      throw std::invalid_argument("State is not a candidate for primitive");
    }
  }

private:
  const std::string name_ = "CheckIfObjectAvailablePrimitive";
};

class PutObjectInBoxPrimitive : public PutInBoxStatePrimitiveType
{
public:
  bool IsCandidate(const PutInBoxState& state) const override
  {
    return ((state.ObjectsAvailable() > 0) && (state.ObjectPutAway() == false)
        && state.BoxOpen());
  }

  const std::string& Name() const override { return name_; }

  PutInBoxStatesWithCosts GetOutcomes(const PutInBoxState& state) const override
  {
    if (IsCandidate(state))
    {
      const PutInBoxState object_remaining(1, true, true);
      const PutInBoxState task_done(0, true, true);

      PutInBoxStatesWithCosts outcomes;
      outcomes.emplace_back(object_remaining, 1.0);
      outcomes.emplace_back(task_done, 1.0);
      return outcomes;
    }
    else
    {
      throw std::invalid_argument("State is not a candidate for primitive");
    }
  }

  PutInBoxStateContainer Execute(const PutInBoxState& state) override
  {
    if (IsCandidate(state))
    {
      return {PutInBoxState(state.ObjectsAvailable() - 1, true, true)};
    }
    else
    {
      throw std::invalid_argument("State is not a candidate for primitive");
    }
  }

private:
  const std::string name_ = "PutObjectInBoxPrimitive";
};

bool IsTaskComplete(const PutInBoxState& state)
{
  return ((state.ObjectsAvailable() == 0) && (state.BoxOpen() == false));
}

bool IsSingleExecutionComplete(const PutInBoxState& state)
{
  if (IsTaskComplete(state))
  {
    return true;
  }
  else
  {
    return ((state.ObjectsAvailable() > 0) && (state.ObjectPutAway() == true));
  }
}

GTEST_TEST(TaskPlanningTest, Test)
{
  // Collect the primitives
  PutInBoxStatePrimitiveCollection primitive_collection;
  primitive_collection.RegisterPrimitive(std::make_shared<OpenBoxPrimitive>());
  primitive_collection.RegisterPrimitive(std::make_shared<CloseBoxPrimitive>());
  primitive_collection.RegisterPrimitive(
      std::make_shared<CheckIfObjectAvailablePrimitive>());
  primitive_collection.RegisterPrimitive(
      std::make_shared<PutObjectInBoxPrimitive>());

  // Plan a sequence for each possible starting state
  const PutInBoxStateContainer possible_starting_states = {
      PutInBoxState(-1, false, false), PutInBoxState(-1, false, true),
      PutInBoxState(-1, true, false), PutInBoxState(-1, true, true),
      PutInBoxState(0, false, false), PutInBoxState(0, false, true),
      PutInBoxState(0, true, false), PutInBoxState(0, true, true),
      PutInBoxState(1, false, false), PutInBoxState(1, false, true),
      PutInBoxState(1, true, false), PutInBoxState(1, true, true)};

  for (const auto& starting_state : possible_starting_states)
  {
    std::cout << "++++++++++\nStarting state:\n" << starting_state << std::endl;
    const auto task_sequence_plan = PlanTaskStateSequence
        <PutInBoxState, PutInBoxStateContainer, PutInBoxStateHasher>(
            primitive_collection, IsSingleExecutionComplete, {starting_state});
    std::cout << "Task sequence plan:" << std::endl;
    for (const auto& state : task_sequence_plan.Path())
    {
      std::cout << print::Print(state.Outcome()) << std::endl;
    }
    ASSERT_TRUE(!std::isinf(task_sequence_plan.PathCost()));
    ASSERT_GT(task_sequence_plan.Path().size(), 0u);
    const auto& final_state = task_sequence_plan.Path().back();
    ASSERT_TRUE(IsSingleExecutionComplete(final_state.Outcome()));
  }

  // Execute the task
  PutInBoxStateContainer task_execution_trace;
  bool task_complete = false;
  int32_t num_task_executions = 0;
  while (!task_complete)
  {
    num_task_executions++;
    std::cout << "++++++++++++++++++++\nStarting task execution "
              << num_task_executions << std::endl;
    PutInBoxState execution_start_state;
    if (task_execution_trace.empty())
    {
      // Start with five objects to put away.
      execution_start_state = PutInBoxState(-5, false, false);
    }
    else
    {
      const auto& last = task_execution_trace.back();
      execution_start_state = PutInBoxState(
          -last.ObjectsAvailable(), false, last.BoxOpen());
    }
    const auto execution_trace = PerformSingleTaskExecution
        <PutInBoxState, PutInBoxStateContainer, PutInBoxStateHasher>(
            primitive_collection, IsSingleExecutionComplete,
            execution_start_state, -1);
    std::cout << "Single execution of task in " << execution_trace.size()
              << " states" << std::endl;
    ASSERT_GT(execution_trace.size(), 0u);
    const auto& last_state = execution_trace.back();
    ASSERT_TRUE(IsSingleExecutionComplete(last_state));
    if (IsTaskComplete(last_state))
    {
      task_complete = true;
    }
    task_execution_trace.insert(
        task_execution_trace.end(), execution_trace.begin(),
        execution_trace.end());
  }
  std::cout << "Completed task in " << num_task_executions << " executions"
            << std::endl;
  ASSERT_GT(task_execution_trace.size(), 0u);
  ASSERT_TRUE(IsTaskComplete(task_execution_trace.back()));
}
}  // namespace
}  // namespace simple_task_planner
}  // namespace common_robotics_utilities

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
