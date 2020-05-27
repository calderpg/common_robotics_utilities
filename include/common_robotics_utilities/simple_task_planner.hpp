#pragma once

#include <omp.h>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <common_robotics_utilities/simple_astar_search.hpp>
#include <common_robotics_utilities/simple_knearest_neighbors.hpp>

namespace common_robotics_utilities
{
namespace simple_task_planner
{
/// Base type for all action primitives
template<typename State, typename Container=std::vector<State>>
class ActionPrimitiveInterface
{
public:
  virtual ~ActionPrimitiveInterface() {}

  /// Returns true if the provided state is a candidare for the primitive
  virtual bool IsCandidate(const State& state) const = 0;

  /// Returns all possible outcomes of executing the primitive
  virtual Container GetOutcomes(const State& state) const = 0;

  /// Estimates the cost of performing the action from `current` to `target`.
  /// `current` must be a candidate for the primitive, and `target` must be a
  /// possible outcome, and a primitive should enforce this if necessary.
  virtual double EstimateActionCost(
      const State& current, const State& target) const = 0;

  /// Executes the primitive and returns the resulting state(s)
  /// Multiple returned states are only valid if *all* states are real,
  /// and you want the policy system to select easiest outcome to pursue.
  /// For example, if your actions operate at the object-level,
  /// a sensing primitive might return multiple states, each corresponding
  /// to a different object in the scene, so that the policy can select the
  /// easiest object to manipulate first.
  virtual Container Execute(const State& state) = 0;

  /// Returns the ranking of the primitive
  /// When multiple primitives can be applied to a given state, the planner
  /// will select the highest-ranked primitive. If multiple primitives with
  /// the same ranking are available, the planner will select the most
  /// recently added primitive.
  virtual double Ranking() const = 0;

  /// Returns the name of the primitive
  virtual const std::string& Name() const = 0;
};

/// Typedef of shared pointer to primitive.
template<typename State, typename Container=std::vector<State>>
using ActionPrimitivePtr
  = std::shared_ptr<ActionPrimitiveInterface<State, Container>>;

/// Wrapper type to generate action primitive types from std::functions
/// Use this if you want to assemble primitives from a number of existing
/// functions or members and don't want to define a new type each time.
template<typename State, typename Container=std::vector<State>>
class ActionPrimitiveWrapper
    : public ActionPrimitiveInterface<State, Container>
{
public:
  using IsCandidateFunction = std::function<bool(const State&)>;
  using GetOutcomesFunction = std::function<Container(const State&)>;
  using EstimateActionCostFunction =
      std::function<double(const State&, const State&)>;
  using ExecuteFunction = std::function<Container(const State&)>;

  ActionPrimitiveWrapper(
      const IsCandidateFunction& is_candidate_fn,
      const GetOutcomesFunction& get_outcomes_fn,
      const EstimateActionCostFunction& estimate_action_cost_fn,
      const ExecuteFunction& execute_fn,
      const double ranking, const std::string& name)
      : is_candidate_fn_(is_candidate_fn),
        get_outcomes_fn_(get_outcomes_fn),
        estimate_action_cost_fn_(estimate_action_cost_fn),
        execute_fn_(execute_fn),
        ranking_(ranking), name_(name) {}

  bool IsCandidate(const State& state) const override
  {
    return is_candidate_fn_(state);
  }

  Container GetOutcomes(const State& state) const override
  {
    return get_outcomes_fn_(state);
  }

  double EstimateActionCost(
      const State& current, const State& target) const override
  {
    return estimate_action_cost_fn_(current, target);
  }

  Container Execute(const State& state) override { return execute_fn_(state); }

  double Ranking() const override { return ranking_; }

  const std::string& Name() const override { return name_; }

private:
  IsCandidateFunction is_candidate_fn_;
  GetOutcomesFunction get_outcomes_fn_;
  EstimateActionCostFunction estimate_action_cost_fn_;
  std::function<Container(const State&)> execute_fn_;
  double ranking_;
  std::string name_;
};

/// Generate a distinct state identifier. Two states with the same identifier
/// must be equal and, likewise, two distinct states must not share the same
/// identifier.
template<typename State>
using StateIdentifierFunction = std::function<int64_t(const State&)>;

/// Wrapper for an action primitive and the estimated cost of applying it for a
/// given transition.
template<typename State, typename Container=std::vector<State>>
class ActionPrimitiveAndEstimatedCost
{
public:
  ActionPrimitiveAndEstimatedCost(
      const ActionPrimitivePtr<State, Container>& action_primitive,
      const double estimated_cost)
      : action_primitive_(action_primitive), estimated_cost_(estimated_cost)
  {
    if (estimated_cost_ < 0.0)
    {
      throw std::invalid_argument("estimated_cost must be >= 0.0");
    }
  }

  ActionPrimitiveAndEstimatedCost() {}

  const ActionPrimitivePtr<State, Container>& ActionPrimitive() const
  {
    return action_primitive_;
  }

  double EstimatedCost() const { return estimated_cost_; }

private:
  ActionPrimitivePtr<State, Container> action_primitive_;
  double estimated_cost_ = std::numeric_limits<double>::infinity();
};

/// Stores a collection of action primitives, enforcing name uniqueness and
/// providing useful helpers.
template<typename State, typename Container=std::vector<State>>
class ActionPrimitiveCollection
{
public:
  using ActionPrimitivePtrType = ActionPrimitivePtr<State, Container>;
  using ActionPrimitivePtrTypeVector = std::vector<ActionPrimitivePtrType>;
  using ActionPrimitiveAndEstimatedCostType =
      ActionPrimitiveAndEstimatedCost<State, Container>;

  explicit ActionPrimitiveCollection(
      const StateIdentifierFunction<State>& state_identifier_fn)
      : state_identifier_fn_(state_identifier_fn) {}

  int64_t GetStateIdentifier(const State& state) const
  {
    return state_identifier_fn_(state);
  }

  void RegisterPrimitive(const ActionPrimitivePtrType& new_primitive)
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
    std::cout << "Registering primitive [" << new_primitive->Name() << "]"
              << std::endl;
    primitives_.push_back(new_primitive);
  }

  void UnregisterPrimitivesByName(const std::unordered_set<std::string>& names)
  {
    ActionPrimitivePtrTypeVector primitives_to_keep;
    for (const auto& primitive : primitives_)
    {
      if (names.count(primitive->Name()) == 0)
      {
        primitives_to_keep.push_back(primitive);
      }
      else
      {
        std::cout << "Unregistering primitive [" << primitive->Name() << "]"
                  << std::endl;
      }
    }
    primitives_ = primitives_to_keep;
  }

  void UnregisterPrimitiveByName(const std::string& name)
  {
    UnregisterPrimitivesByName({name});
  }

  void ClearPrimitives() { primitives_.clear(); }

  const ActionPrimitivePtrTypeVector& Primitives() const { return primitives_; }

  // Return best primitive and estimated cost (since determining "best" will
  // involve estimating the cost).
  ActionPrimitiveAndEstimatedCostType SelectBestPrimitiveForTransition(
      const State& start_state, const State& target_state) const
  {
    const int64_t target_state_identifier = GetStateIdentifier(target_state);

    // Get the cost for applying each primitive. If the primitive cannot be
    // applied to the starting state, or does not produce the target state, it
    // receives a cost of infinity.
    std::vector<simple_knearest_neighbors::IndexAndDistance> primitive_costs(
        primitives_.size());

#pragma omp parallel for
    for (size_t idx = 0; idx < primitives_.size(); idx++)
    {
      double primitive_cost = std::numeric_limits<double>::infinity();
      const auto& primitive = primitives_.at(idx);
      if (primitive->IsCandidate(start_state))
      {
        const auto outcomes = primitive->GetOutcomes(start_state);
        for (const auto& outcome : outcomes)
        {
          const int64_t outcome_state_identifier = GetStateIdentifier(outcome);
          if (outcome_state_identifier == target_state_identifier)
          {
            const double estimated_action_cost =
                primitive->EstimateActionCost(start_state, target_state);
            primitive_cost = estimated_action_cost;
            break;
          }
        }
      }
      primitive_costs.at(idx) = simple_knearest_neighbors::IndexAndDistance(
          static_cast<int64_t>(idx), primitive_cost);
    }

    // Sort all primitives by cost
    std::sort(
        primitive_costs.begin(), primitive_costs.end(),
        simple_knearest_neighbors::IndexAndDistanceCompare);

    const double best_cost = primitive_costs.at(0).Distance();
    if (!std::isinf(best_cost))
    {
      // Identify the "best" primitives, of which there may be multiple.
      std::vector<simple_knearest_neighbors::IndexAndDistance> best_primitives;
      for (const auto& primitive_cost : primitive_costs)
      {
        if (primitive_cost.Distance() == best_cost)
        {
          best_primitives.push_back(primitive_cost);
        }
      }

      // Select the single "best" primitive by ranking
      simple_knearest_neighbors::IndexAndDistance
          best_primitive_index_and_cost;
      double best_primitive_ranking = -std::numeric_limits<double>::infinity();

      for (const auto& primitive_cost : best_primitives)
      {
        const auto& primitive = primitives_.at(primitive_cost.Index());
        const double primitive_ranking = primitive->Ranking();
        if (primitive_ranking > best_primitive_ranking)
        {
          best_primitive_ranking = primitive_ranking;
          best_primitive_index_and_cost = primitive_cost;
        }
      }

      const int64_t best_primitive_index =
          best_primitive_index_and_cost.Index();
      const double best_primitive_cost =
          best_primitive_index_and_cost.Distance();
      const auto& best_primitive = primitives_.at(best_primitive_index);

      // Warn about cases with arbitrary primitive selection.
      for (const auto& primitive_cost : best_primitives)
      {
        if (primitive_cost.Index() != best_primitive_index)
        {
          const auto& primitive = primitives_.at(primitive_cost.Index());
          if (primitive->Ranking() == best_primitive_ranking)
          {
            std::cerr << "*WARNING* Alternative [" << primitive->Name()
                      << "] has the same cost and ranking as selected ["
                      << best_primitive->Name()
                      << "] - primitive selection was arbitrary" << std::endl;
          }
        }
      }

      return ActionPrimitiveAndEstimatedCostType(
          best_primitive, best_primitive_cost);
    }
    else
    {
      std::cerr << "No primitive produces the desired transition" << std::endl;
      return ActionPrimitiveAndEstimatedCostType();
    }
  }

private:
  StateIdentifierFunction<State> state_identifier_fn_;
  ActionPrimitivePtrTypeVector primitives_;
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

/// Return type for planning task state sequences.
template<typename State, typename Container=std::vector<State>>
using TaskStateAStarResult = simple_astar_search::AstarResult<State, Container>;

template<typename State, typename Container=std::vector<State>>
TaskStateAStarResult<State, Container> PlanTaskStateSequence(
    const ActionPrimitiveCollection<State, Container>& primitive_collection,
    const TaskSequenceCompleteFunction<State>& task_sequence_complete_fn,
    const State& start_state,
    const StateHeuristicFunction<State>& state_heuristic_fn = {})
{
  std::unordered_map<int64_t, State> state_map;
  const int64_t start_id = primitive_collection.GetStateIdentifier(start_state);
  state_map[start_id] = start_state;

  // Assemble helper functions.
  const std::function<bool(const int64_t)> goal_check_function =
      [&](const int64_t current_state_id)
  {
    const State& current_state = state_map.at(current_state_id);
    return task_sequence_complete_fn(current_state);
  };

  const std::function<std::vector<int64_t>(const int64_t)>
      generate_children_function = [&] (const int64_t current_state_id)
  {
    const State& current_state = state_map.at(current_state_id);

    std::vector<int64_t> child_state_ids;

    for (const auto& primitive : primitive_collection.Primitives())
    {
      if (primitive->IsCandidate(current_state))
      {
        const auto outcomes = primitive->GetOutcomes(current_state);
        for (const State& outcome_state : outcomes)
        {
          const int64_t outcome_state_id =
              primitive_collection.GetStateIdentifier(outcome_state);
          state_map[outcome_state_id] = outcome_state;
          child_state_ids.push_back(outcome_state_id);
        }
      }
    }

    return child_state_ids;
  };

  // Child state generation only produces valid transitions.
  const std::function<bool(int64_t, int64_t)> edge_validity_check_function =
      [] (const int64_t, const int64_t) { return true; };

  const std::function<double(int64_t, int64_t)> state_distance_function =
      [&] (const int64_t starting_state_id, const int64_t target_state_id)
  {
    const State& starting_state = state_map.at(starting_state_id);
    const State& target_state = state_map.at(target_state_id);

    const auto best_primitive =
        primitive_collection.SelectBestPrimitiveForTransition(
            starting_state, target_state);
    return best_primitive.EstimatedCost();
  };

  const std::function<double(const int64_t)> heuristic_function =
      [&] (const int64_t current_state_id)
  {
    if (state_heuristic_fn)
    {
      const State& current_state = state_map.at(current_state_id);
      return state_heuristic_fn(current_state);
    }
    else
    {
      return 0.0;
    }
  };

  const bool limit_pqueue_duplicates = true;
  const auto astar_solution = simple_astar_search::PerformGenericAstarSearch(
      start_id, goal_check_function, generate_children_function,
      edge_validity_check_function, state_distance_function,
      heuristic_function, limit_pqueue_duplicates);

  Container solution_path;
  solution_path.reserve(astar_solution.Path().size());
  for (const int64_t state_id : astar_solution.Path()) {
    const State& solution_path_state = state_map.at(state_id);
    solution_path.push_back(solution_path_state);
  }
  solution_path.shrink_to_fit();
  return TaskStateAStarResult<State, Container>(
      solution_path, astar_solution.PathCost());
}

/// Wrapper for an action primitive, the estimated cost of applying it for a
/// given transition, and the index of the best (selected) outcome.
template<typename State, typename Container=std::vector<State>>
class NextPrimitiveToExecute
{
public:
  NextPrimitiveToExecute(
      const ActionPrimitivePtr<State, Container>& action_primitive,
      const double estimated_cost, const int64_t selected_outcome_index)
      : action_primitive_(action_primitive), estimated_cost_(estimated_cost),
        selected_outcome_index_(selected_outcome_index)
  {
    if (estimated_cost_ < 0.0)
    {
      throw std::invalid_argument("estimated_cost must be >= 0.0");
    }
    if (selected_outcome_index_ < 0)
    {
      throw std::invalid_argument("selected_outcome_index must be >= 0");
    }
  }

  explicit NextPrimitiveToExecute(const int64_t selected_outcome_index)
      : NextPrimitiveToExecute({}, 0.0, selected_outcome_index) {}

  const ActionPrimitivePtr<State, Container>& ActionPrimitive() const
  {
    return action_primitive_;
  }

  double EstimatedCost() const { return estimated_cost_; }

  int64_t SelectedOutcomeIndex() const { return selected_outcome_index_; }

private:
  ActionPrimitivePtr<State, Container> action_primitive_;
  double estimated_cost_ = std::numeric_limits<double>::infinity();
  int64_t selected_outcome_index_ = -1;
};

template<typename State, typename Container=std::vector<State>>
NextPrimitiveToExecute<State, Container> GetNextPrimitiveToExecute(
    const ActionPrimitiveCollection<State, Container>& primitive_collection,
    const TaskSequenceCompleteFunction<State>& task_sequence_complete_fn,
    const Container& potential_outcome_states,
    const StateHeuristicFunction<State>& state_heuristic_fn = {})
{
  if (potential_outcome_states.empty())
  {
    throw std::invalid_argument("potential_outcome_states cannot be empty");
  }

  // Plan for each potential outcome to get a sense of how expensive it is.
  std::vector<TaskStateAStarResult<State, Container>> outcome_task_sequences(
      potential_outcome_states.size());

#pragma omp parallel for
  for (size_t idx = 0; idx < potential_outcome_states.size(); idx++) {
    outcome_task_sequences.at(idx) = PlanTaskStateSequence<State, Container>(
        primitive_collection, task_sequence_complete_fn,
        potential_outcome_states.at(idx), state_heuristic_fn);
  }

  // Get the cheapest outcome in terms of task sequence cost.
  const std::function<double(
      const TaskStateAStarResult<State, Container>&, const int&)>
          outcome_cost_function = [&] (
              const TaskStateAStarResult<State, Container>& task_state_sequence,
              const int&)
  {
    return task_state_sequence.PathCost();
  };

  const auto best_outcome = common_robotics_utilities::simple_knearest_neighbors
      ::GetKNearestNeighbors(
          outcome_task_sequences, 0, outcome_cost_function, 1, false).at(0);

  // Identify the best primitive to perform next.
  if (!std::isinf(best_outcome.Distance())) {
    if (best_outcome.Distance() > 0.0) {
      const State& best_outcome_state =
          potential_outcome_states.at(best_outcome.Index());
      const auto& best_task_state_sequence =
          outcome_task_sequences.at(best_outcome.Index());
      const State& target_next_state = best_task_state_sequence.Path().at(1);

      const auto best_primitive_and_cost =
          primitive_collection.SelectBestPrimitiveForTransition(
              best_outcome_state, target_next_state);

      return NextPrimitiveToExecute<State, Container>(
          best_primitive_and_cost.ActionPrimitive(),
          best_primitive_and_cost.EstimatedCost(), best_outcome.Index());
    } else {
      // 0 distance means the outcome completes a task sequence and thus there
      // is no next primitive to execute.
      return NextPrimitiveToExecute<State, Container>(best_outcome.Index());
    }
  } else {
    throw std::runtime_error(
        "Could not identify task state sequence for any potential outcomes");
  }
}

template<typename State, typename Container=std::vector<State>>
Container PerformSingleTaskExecution(
    const ActionPrimitiveCollection<State, Container>& primitive_collection,
    const TaskSequenceCompleteFunction<State>& task_sequence_complete_fn,
    const Container& start_states,
    const int32_t max_primitive_executions = -1,
    const bool single_step = false,
    const StateHeuristicFunction<State>& state_heuristic_fn = {},
    const std::function<void(
        const State&, const ActionPrimitivePtr<State, Container>&)>&
            user_pre_action_callback_fn = {},
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
    const auto next_to_execute = GetNextPrimitiveToExecute<State, Container>(
        primitive_collection, task_sequence_complete_fn, current_outcomes,
        state_heuristic_fn);

    // Get the outcome state and add it to the execution trace.
    const State& selected_outcome =
        current_outcomes.at(next_to_execute.SelectedOutcomeIndex());
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

    // Prompt to continue/notify.
    if (single_step)
    {
      std::cout << "Press ENTER to perform primitive ["
                << next_primitive->Name() << "]" << std::endl;
      std::cin.get();
    }
    else
    {
      std::cout << "Performing primitive [" << next_primitive->Name() << "]"
                << std::endl;
    }

    // Execute the primitive.
    num_primitive_executions++;
    const auto primitive_outcomes = next_primitive->Execute(selected_outcome);

    current_outcomes = primitive_outcomes;
  }

  return task_state_trace;
}

template<typename State, typename Container=std::vector<State>>
Container PerformSingleTaskExecution(
    const ActionPrimitiveCollection<State, Container>& primitive_collection,
    const TaskSequenceCompleteFunction<State>& task_sequence_complete_fn,
    const State& start_state,
    const int32_t max_primitive_executions = -1,
    const bool single_step = false,
    const StateHeuristicFunction<State>& state_heuristic_fn = {},
    const std::function<void(
        const State&, const ActionPrimitivePtr<State, Container>&)>&
            user_pre_action_callback_fn = {},
    const std::function<void(const Container&, int64_t)>&
        user_post_outcome_callback_fn = {})
{
  Container start_states;
  start_states.push_back(start_state);

  return PerformSingleTaskExecution<State, Container>(
      primitive_collection, task_sequence_complete_fn, start_states,
      max_primitive_executions, single_step, state_heuristic_fn,
      user_pre_action_callback_fn, user_post_outcome_callback_fn);
}
}  // namespace simple_task_planner
}  // namespace common_robotics_utilities
