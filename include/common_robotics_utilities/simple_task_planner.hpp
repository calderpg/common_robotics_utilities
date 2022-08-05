#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/maybe.hpp>
#include <common_robotics_utilities/simple_astar_search.hpp>
#include <common_robotics_utilities/simple_knearest_neighbors.hpp>
#include <common_robotics_utilities/utility.hpp>

namespace common_robotics_utilities
{
namespace simple_task_planner
{
template<typename State>
using OutcomesWithCosts = simple_astar_search::StatesWithCosts<State>;

/// Base type for all action primitives
template<typename State, typename Container=std::vector<State>>
class ActionPrimitiveInterface
{
public:
  virtual ~ActionPrimitiveInterface() {}

  /// Returns true if the provided state is a candidare for the primitive
  virtual bool IsCandidate(const State& state) const = 0;

  /// Returns all possible outcomes of executing the primitive, including the
  /// real/estimated cost of performing the primitive.
  virtual OutcomesWithCosts<State> GetOutcomes(const State& state) const = 0;

  /// Executes the primitive and returns the resulting state(s)
  /// Multiple returned states are only valid if *all* states are real,
  /// and you want the policy system to select easiest outcome to pursue.
  /// For example, if your actions operate at the object-level,
  /// a sensing primitive might return multiple states, each corresponding
  /// to a different object in the scene, so that the policy can select the
  /// easiest object to manipulate first.
  virtual Container Execute(const State& state) = 0;

  /// Returns the name of the primitive
  virtual const std::string& Name() const = 0;
};

/// Typedef of shared pointer to primitive.
template<typename State, typename Container=std::vector<State>>
using ActionPrimitiveSharedPtr =
    std::shared_ptr<ActionPrimitiveInterface<State, Container>>;

/// Wrapper type to generate action primitive types from std::functions
/// Use this if you want to assemble primitives from a number of existing
/// functions or members and don't want to define a new type each time.
template<typename State, typename Container=std::vector<State>>
class ActionPrimitiveWrapper
    : public ActionPrimitiveInterface<State, Container>
{
public:
  using IsCandidateFunction = std::function<bool(const State&)>;
  using GetOutcomesFunction =
      std::function<OutcomesWithCosts<State>(const State&)>;
  using ExecuteFunction = std::function<Container(const State&)>;

  ActionPrimitiveWrapper(
      const IsCandidateFunction& is_candidate_fn,
      const GetOutcomesFunction& get_outcomes_fn,
      const ExecuteFunction& execute_fn,
      const std::string& name)
      : is_candidate_fn_(is_candidate_fn),
        get_outcomes_fn_(get_outcomes_fn),
        execute_fn_(execute_fn), name_(name) {}

  bool IsCandidate(const State& state) const override
  {
    return is_candidate_fn_(state);
  }

  OutcomesWithCosts<State> GetOutcomes(const State& state) const override
  {
    return get_outcomes_fn_(state);
  }

  Container Execute(const State& state) override { return execute_fn_(state); }

  const std::string& Name() const override { return name_; }

private:
  IsCandidateFunction is_candidate_fn_;
  GetOutcomesFunction get_outcomes_fn_;
  ExecuteFunction execute_fn_;
  std::string name_;
};

/// Stores a collection of action primitives, enforcing name uniqueness and
/// providing useful helpers.
template<typename State, typename Container=std::vector<State>>
class ActionPrimitiveCollection
{
public:
  explicit ActionPrimitiveCollection(
      const utility::LoggingFunction& logging_fn = {})
      : logging_fn_(logging_fn) {}

  void SetLoggingFunction(const utility::LoggingFunction& logging_fn)
  {
    logging_fn_ = logging_fn;
  }

  void Log(const std::string& message) const
  {
    if (logging_fn_)
    {
      logging_fn_(message);
    }
  }

  void RegisterPrimitive(
      const ActionPrimitiveSharedPtr<State, Container>& new_primitive)
  {
    for (const auto& primitive : primitives_)
    {
      if (primitive->Name() == new_primitive->Name())
      {
        throw std::invalid_argument("New planning primitive with name ["
                                    + new_primitive->Name()
                                    + "] cannot share name with existing ["
                                    + primitive->Name() + "]");
      }
    }
    Log("Registering primitive [" + new_primitive->Name() + "]");
    primitives_.push_back(new_primitive);
  }

  void UnregisterPrimitivesByName(const std::unordered_set<std::string>& names)
  {
    std::vector<ActionPrimitiveSharedPtr<State, Container>> primitives_to_keep;
    for (const auto& primitive : primitives_)
    {
      if (names.count(primitive->Name()) == 0)
      {
        primitives_to_keep.push_back(primitive);
      }
      else
      {
        Log("Unregistering primitive [" + primitive->Name() + "]");
      }
    }
    primitives_ = primitives_to_keep;
  }

  void UnregisterPrimitiveByName(const std::string& name)
  {
    UnregisterPrimitivesByName({name});
  }

  void ClearPrimitives() { primitives_.clear(); }

  const std::vector<ActionPrimitiveSharedPtr<State, Container>>&
  Primitives() const { return primitives_; }

private:
  std::vector<ActionPrimitiveSharedPtr<State, Container>> primitives_;
  utility::LoggingFunction logging_fn_;
};

/// Return a heuristic value for the provided state.
template<typename State>
using StateHeuristicFunction = std::function<double(const State&)>;

/// Returns true if the provided state completes a task sequence. For a cyclic
/// task, this means either (a) a single cycle of the task has been completed,
/// or (b) the entire task is completed.
template<typename State>
using TaskSequenceCompleteFunction = std::function<bool(const State&)>;

/// Returns true if the provided state completes the entire task.
template<typename State>
using TaskExecutionCompleteFunction = std::function<bool(const State&)>;

/// Wrapper for an outcome and the index of the primitive that produced it.
template<typename State>
class OutcomeWithPrimitiveIndex
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OutcomeWithPrimitiveIndex(const State& outcome, const int64_t primitive_index)
      : outcome_(outcome), primitive_index_(primitive_index) {}

  explicit OutcomeWithPrimitiveIndex(const State& outcome)
      : OutcomeWithPrimitiveIndex(outcome, -1) {}

  const State& Outcome() const { return outcome_; }

  int64_t PrimitiveIndex() const { return primitive_index_; }

private:
  State outcome_;
  int64_t primitive_index_ = -1;
};

template<typename State>
using OutcomeWithPrimitiveIndexAllocator =
    Eigen::aligned_allocator<OutcomeWithPrimitiveIndex<State>>;

template<typename State>
using OutcomeWithPrimitiveIndexContainer = std::vector
    <OutcomeWithPrimitiveIndex<State>,
     OutcomeWithPrimitiveIndexAllocator<State>>;

/// Wrapper for hash function on OutcomeWithPrimitiveIndex<State>.
template<typename State, typename StateHash>
class OutcomeWithPrimitiveIndexHash
{
public:
  explicit OutcomeWithPrimitiveIndexHash(const StateHash& state_hasher)
      : state_hasher_(state_hasher) {}

  bool operator()(const OutcomeWithPrimitiveIndex<State>& outcome) const
  {
    return state_hasher_(outcome.Outcome());
  }

private:
  StateHash state_hasher_;
};

/// Wrapper for equal function on OutcomeWithPrimitiveIndex<State>.
template<typename State, typename StateEqual>
class OutcomeWithPrimitiveIndexEqual
{
public:
  explicit OutcomeWithPrimitiveIndexEqual(const StateEqual& state_equaler)
      : state_equaler_(state_equaler) {}

  bool operator()(
      const OutcomeWithPrimitiveIndex<State>& first,
      const OutcomeWithPrimitiveIndex<State>& second) const
  {
    return ((first.PrimitiveIndex() == second.PrimitiveIndex()) &&
            state_equaler_(first.Outcome(), second.Outcome()));
  }

private:
  StateEqual state_equaler_;
};

/// Return type for planning task state sequences.
template<typename State>
using TaskStateAStarResult = simple_astar_search::AstarResult
    <OutcomeWithPrimitiveIndex<State>,
     OutcomeWithPrimitiveIndexContainer<State>>;

template<typename State, typename Container=std::vector<State>,
         typename StateHash=std::hash<State>,
         typename StateEqual=std::equal_to<State>>
TaskStateAStarResult<State> PlanTaskStateSequence(
    const ActionPrimitiveCollection<State, Container>& primitive_collection,
    const TaskSequenceCompleteFunction<State>& task_sequence_complete_fn,
    const Container& start_states,
    const StateHeuristicFunction<State>& state_heuristic_fn = {},
    const StateHash& state_hasher = StateHash(),
    const StateEqual& state_equaler = StateEqual())
{
  using PrimitiveIndexStatesWithCosts =
      simple_astar_search::StatesWithCosts<OutcomeWithPrimitiveIndex<State>>;

  // Assemble helper functions.
  const TaskSequenceCompleteFunction<OutcomeWithPrimitiveIndex<State>>
      goal_reached_function = [&] (
          const OutcomeWithPrimitiveIndex<State>& current_outcome)
  {
    return task_sequence_complete_fn(current_outcome.Outcome());
  };

  const std::function<PrimitiveIndexStatesWithCosts(
      const OutcomeWithPrimitiveIndex<State>&)> generate_children_function =
          [&] (const OutcomeWithPrimitiveIndex<State>& current_outcome)
  {
    const State& current_state = current_outcome.Outcome();

    PrimitiveIndexStatesWithCosts child_states_with_costs;

    for (int64_t index = 0;
         index < static_cast<int64_t>(primitive_collection.Primitives().size());
         index++)
    {
      const auto& primitive =
          primitive_collection.Primitives().at(static_cast<size_t>(index));
      if (primitive->IsCandidate(current_state))
      {
        const auto outcomes = primitive->GetOutcomes(current_state);
        for (const auto& outcome : outcomes)
        {
          child_states_with_costs.emplace_back(
              OutcomeWithPrimitiveIndex<State>(outcome.State(), index),
              outcome.Cost());
        }
      }
    }

    return child_states_with_costs;
  };

  // Wrap the heuristic function, if provided.
  const std::function<double(const OutcomeWithPrimitiveIndex<State>&)>
      heuristic_function = [&] (
          const OutcomeWithPrimitiveIndex<State>& current_outcome)
  {
    if (state_heuristic_fn)
    {
      return state_heuristic_fn(current_outcome.Outcome());
    }
    else
    {
      return 0.0;
    }
  };

  // Package the initial states.
  PrimitiveIndexStatesWithCosts astar_start_states;
  for (const auto& start_state : start_states)
  {
    astar_start_states.emplace_back(
        OutcomeWithPrimitiveIndex<State>(start_state), 0.0);
  }

  const bool limit_pqueue_duplicates = true;

  return simple_astar_search::PerformAstarSearch
      <OutcomeWithPrimitiveIndex<State>,
       OutcomeWithPrimitiveIndexContainer<State>,
       OutcomeWithPrimitiveIndexHash<State, StateHash>,
       OutcomeWithPrimitiveIndexEqual<State, StateEqual>>(
          astar_start_states, goal_reached_function, generate_children_function,
          heuristic_function, limit_pqueue_duplicates,
          OutcomeWithPrimitiveIndexHash<State, StateHash>(state_hasher),
          OutcomeWithPrimitiveIndexEqual<State, StateEqual>(state_equaler));
}

/// Wrapper for an action primitive and the index of the best (selected)
/// outcome.
template<typename State, typename Container=std::vector<State>>
class NextPrimitiveToExecute
{
public:
  NextPrimitiveToExecute(
      const ActionPrimitiveSharedPtr<State, Container>& action_primitive,
      const int64_t selected_outcome_index)
      : action_primitive_(action_primitive),
        selected_outcome_index_(selected_outcome_index)
  {
    if (selected_outcome_index_ < 0)
    {
      throw std::invalid_argument("selected_outcome_index must be >= 0");
    }
  }

  explicit NextPrimitiveToExecute(const int64_t selected_outcome_index)
      : NextPrimitiveToExecute({}, selected_outcome_index) {}

  const ActionPrimitiveSharedPtr<State, Container>& ActionPrimitive() const
  {
    return action_primitive_;
  }

  int64_t SelectedOutcomeIndex() const { return selected_outcome_index_; }

private:
  ActionPrimitiveSharedPtr<State, Container> action_primitive_;
  int64_t selected_outcome_index_ = -1;
};

template<typename State, typename Container=std::vector<State>,
         typename StateHash=std::hash<State>,
         typename StateEqual=std::equal_to<State>>
NextPrimitiveToExecute<State, Container> GetNextPrimitiveToExecute(
    const ActionPrimitiveCollection<State, Container>& primitive_collection,
    const TaskSequenceCompleteFunction<State>& task_sequence_complete_fn,
    const Container& potential_outcome_states,
    const StateHeuristicFunction<State>& state_heuristic_fn = {},
    const StateHash& state_hasher = StateHash(),
    const StateEqual& state_equaler = StateEqual())
{
  if (potential_outcome_states.empty())
  {
    throw std::invalid_argument("potential_outcome_states cannot be empty");
  }

  // Plan for each potential outcome to get a sense of how expensive it is.
  const auto task_sequence =
      PlanTaskStateSequence<State, Container, StateHash, StateEqual>(
          primitive_collection, task_sequence_complete_fn,
          potential_outcome_states, state_heuristic_fn, state_hasher,
          state_equaler);

  // Identify the best primitive to perform next.
  if (!std::isinf(task_sequence.PathCost()))
  {
    const State& outcome_state = task_sequence.Path().at(0).Outcome();

    int64_t best_outcome_index = -1;
    for (size_t index = 0; index < potential_outcome_states.size(); index++)
    {
      if (state_equaler(outcome_state, potential_outcome_states.at(index)))
      {
        best_outcome_index = static_cast<int64_t>(index);
        break;
      }
    }
    if (best_outcome_index < 0)
    {
      throw std::runtime_error("Could not identify best outcome state");
    }

    if (task_sequence.Path().size() > 1)
    {
      const auto& next_state_and_index = task_sequence.Path().at(1);
      const auto& next_primitive = primitive_collection.Primitives().at(
          static_cast<size_t>(next_state_and_index.PrimitiveIndex()));

      return NextPrimitiveToExecute<State, Container>(
          next_primitive, best_outcome_index);
    }
    else
    {
      // Path.size() == 1 means that the outcome completes a task sequence and
      // thus there is no next primitive to execute.
      return NextPrimitiveToExecute<State, Container>(best_outcome_index);
    }
  }
  else
  {
    throw std::runtime_error(
        "Could not identify task state sequence for any potential outcomes");
  }
}

template<typename State, typename Container=std::vector<State>,
         typename StateHash=std::hash<State>,
         typename StateEqual=std::equal_to<State>>
Container PerformSingleTaskExecution(
    const ActionPrimitiveCollection<State, Container>& primitive_collection,
    const TaskSequenceCompleteFunction<State>& task_sequence_complete_fn,
    const Container& start_states,
    const int32_t max_primitive_executions = -1,
    const StateHeuristicFunction<State>& state_heuristic_fn = {},
    const StateHash& state_hasher = StateHash(),
    const StateEqual& state_equaler = StateEqual(),
    const std::function<void(
        const State&, const ActionPrimitiveSharedPtr<State, Container>&)>&
            user_pre_action_callback_fn = {},
    const std::function<void(const Container&)>&
        user_post_execution_callback = {},
    const std::function<void(const Container&, int64_t)>&
        user_post_outcome_callback_fn = {})
{
  Container current_outcomes = start_states;

  Container task_state_trace;

  bool done_task = false;
  int32_t num_primitive_executions = 0;
  while (!done_task && (num_primitive_executions < max_primitive_executions ||
                        max_primitive_executions < 0))
  {
    // Identify the selected outcome state and the next primitive to execute.
    const auto next_to_execute =
        GetNextPrimitiveToExecute<State, Container, StateHash, StateEqual>(
            primitive_collection, task_sequence_complete_fn, current_outcomes,
            state_heuristic_fn, state_hasher, state_equaler);

    // Get the outcome state and add it to the execution trace.
    const State& selected_outcome = current_outcomes.at(
        static_cast<size_t>(next_to_execute.SelectedOutcomeIndex()));
    task_state_trace.push_back(selected_outcome);

    // Call the user-provided callback.
    if (user_post_outcome_callback_fn)
    {
      user_post_outcome_callback_fn(
          current_outcomes, next_to_execute.SelectedOutcomeIndex());
    }

    // Get the next primitive.
    const auto& next_primitive = next_to_execute.ActionPrimitive();

    // No next primitive means the task is done.
    if (!next_primitive)
    {
      done_task = true;
      break;
    }

    // Call the user-provided callback.
    if (user_pre_action_callback_fn)
    {
      user_pre_action_callback_fn(selected_outcome, next_primitive);
    }

    primitive_collection.Log(
        "Performing primitive [" + next_primitive->Name() + "]");

    // Execute the primitive.
    num_primitive_executions++;
    current_outcomes = next_primitive->Execute(selected_outcome);

    // Call the user-provided callback.
    if (user_post_execution_callback)
    {
      user_post_execution_callback(current_outcomes);
    }
  }

  return task_state_trace;
}

template<typename State, typename Container=std::vector<State>,
         typename StateHash=std::hash<State>,
         typename StateEqual=std::equal_to<State>>
Container PerformSingleTaskExecution(
    const ActionPrimitiveCollection<State, Container>& primitive_collection,
    const TaskSequenceCompleteFunction<State>& task_sequence_complete_fn,
    const State& start_state,
    const int32_t max_primitive_executions = -1,
    const StateHeuristicFunction<State>& state_heuristic_fn = {},
    const StateHash& state_hasher = StateHash(),
    const StateEqual& state_equaler = StateEqual(),
    const std::function<void(
        const State&, const ActionPrimitiveSharedPtr<State, Container>&)>&
            user_pre_action_callback_fn = {},
    const std::function<void(const Container&)>&
        user_post_execution_callback = {},
    const std::function<void(const Container&, int64_t)>&
        user_post_outcome_callback_fn = {})
{
  Container start_states;
  start_states.push_back(start_state);

  return PerformSingleTaskExecution<State, Container, StateHash, StateEqual>(
      primitive_collection, task_sequence_complete_fn, start_states,
      max_primitive_executions, state_heuristic_fn, state_hasher, state_equaler,
      user_pre_action_callback_fn, user_post_execution_callback,
      user_post_outcome_callback_fn);
}
}  // namespace simple_task_planner
}  // namespace common_robotics_utilities
