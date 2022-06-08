#pragma once

#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/maybe.hpp>

namespace common_robotics_utilities
{
namespace simple_astar_search
{
/// Wrapper for P-Queue elements used in A* search. Contains the unique node_id,
/// "backpointer" to the parent node_id, cost to come, and state value.
template<typename T>
class AstarPQueueElement
{
protected:

  T state_;
  double cost_to_come_ = 0.0;
  double value_ = 0.0;
  OwningMaybe<T> back_pointer_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AstarPQueueElement(const T& state, const T& back_pointer,
                     const double cost_to_come, const double value)
      : state_(state), cost_to_come_(cost_to_come), value_(value),
        back_pointer_(OwningMaybe<T>(back_pointer)) {}

  AstarPQueueElement(const T& state, const double cost_to_come,
                     const double value)
      : state_(state), cost_to_come_(cost_to_come), value_(value) {}

  const T& State() const { return state_; }

  const T& BackPointer() const { return back_pointer_.Value(); }

  bool HasBackPointer() const { return back_pointer_.HasValue(); }

  const OwningMaybe<T>& RawBackPointer() const { return back_pointer_; }

  double CostToCome() const { return cost_to_come_; }

  double Value() const { return value_; }
};

/// Comparator for AstarPQueueElements for use in std::priority_queue.
template<typename T>
class CompareAstarPQueueElementFn
{
public:

  bool operator()(const AstarPQueueElement<T>& lhs,
                  const AstarPQueueElement<T>& rhs) const
  {
    return lhs.Value() > rhs.Value();
  }
};

/// Search result is both path and path cost.
/// Path is a vector of values, and cost is the computed cost-to-come of the
/// goal node.
template<typename T, typename Container=std::vector<T>>
class AstarResult
{
private:
  Container path_;
  double path_cost_ = std::numeric_limits<double>::infinity();

public:
  AstarResult() {}

  AstarResult(const Container& path, const double path_cost)
      : path_(path), path_cost_(path_cost)
  {
    if (path_.size() > 0 && std::isinf(path_cost_))
    {
      throw std::invalid_argument(
          "Cannot create AstarResult with non-empty path and infinite cost");
    }
  }

  const Container& Path() const { return path_; }

  double PathCost() const { return path_cost_; }
};

/// Class to wrap the backpointer and cost-to-come, as used in the explored
/// list in A*.
template<typename T>
class BackPointerAndCostToCome
{
private:
  double cost_to_come_ = std::numeric_limits<double>::infinity();
  OwningMaybe<T> back_pointer_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // TODO(calderpg) Once C++17 is required, remove the default constructor and
  // replace map[] with map.insert_or_assign().
  BackPointerAndCostToCome() {}

  explicit BackPointerAndCostToCome(const AstarPQueueElement<T>& pqueue_element)
      : cost_to_come_(pqueue_element.CostToCome()),
        back_pointer_(pqueue_element.RawBackPointer()) {}

  const T& BackPointer() const { return back_pointer_.Value(); }

  bool HasBackPointer() const { return back_pointer_.HasValue(); }

  const OwningMaybe<T>& RawBackPointer() const { return back_pointer_; }

  double CostToCome() const { return cost_to_come_; }
};

template<typename T, typename StateHash=std::hash<T>,
         typename StateEqual=std::equal_to<T>>
using ExploredList =
    std::unordered_map<T, BackPointerAndCostToCome<T>, StateHash, StateEqual>;

/// Extract the result from the explored node map @param explored produced by A*
/// search, with @param goal_id the node_id of the goal node.
template<typename T, typename Container=std::vector<T>,
         typename StateHash=std::hash<T>, typename StateEqual=std::equal_to<T>>
inline AstarResult<T, Container> ExtractAstarResult(
    const ExploredList<T, StateHash, StateEqual>& explored, const T& goal_state)
{
  // Check if a solution was found
  const auto goal_state_itr = explored.find(goal_state);
  // If no solution found
  if (goal_state_itr == explored.end())
  {
    return AstarResult<T, Container>();
  }
  // If a solution was found
  else
  {
    // Extract the path states in reverse order
    Container solution_path;
    solution_path.push_back(goal_state);

    OwningMaybe<T> current_back_pointer =
        goal_state_itr->second.RawBackPointer();

    while (current_back_pointer)
    {
      const T& backpointer_state = current_back_pointer.Value();
      // Using map.at(key) throws an exception if key not found
      // This provides bounds safety check
      const auto& current_state = explored.at(backpointer_state);
      solution_path.push_back(backpointer_state);
      current_back_pointer = current_state.RawBackPointer();
    }

    // Reverse
    std::reverse(solution_path.begin(), solution_path.end());

    // Get the cost of the path
    const double solution_path_cost = goal_state_itr->second.CostToCome();

    return AstarResult<T, Container>(solution_path, solution_path_cost);
  }
}

/// Wrapper for state with cost.
template<typename T>
class StateWithCost
{
private:
  T state_;
  double cost_ = 0.0;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StateWithCost() {}

  explicit StateWithCost(const T& state) : state_(state) {}

  StateWithCost(const T& state, const double cost)
      : state_(state), cost_(cost)
  {
    if (cost_ < 0.0)
    {
      throw std::invalid_argument("cost cannot be negative");
    }
  }

  const T& State() const { return state_; }

  double Cost() const { return cost_; }
};

template<typename T>
using StateWithCostAllocator = Eigen::aligned_allocator<StateWithCost<T>>;

template<typename T>
using StatesWithCosts =
    std::vector<StateWithCost<T>, StateWithCostAllocator<T>>;

/// Perform A* search.
/// @return Solution path between one of the starting states in
/// @param start_states and the goal (as defined by @param goal_check_fn), if
/// one exists, or an empty path.
/// @param generate_valid_children_fn returns the valid child states for the
/// provided state together with the cost of moving from provided state to the
/// child state.
/// @param heuristic_fn returns the heuristic value for the provided state.
/// @param limit_pqueue_duplicates selects whether to use a second hashtable to
/// track elements in the pqueue and reduce duplicates added to the pqueue. This
/// usually improves performance.
template<typename T, typename Container=std::vector<T>,
         typename StateHash=std::hash<T>, typename StateEqual=std::equal_to<T>>
inline AstarResult<T, Container> PerformAstarSearch(
    const StatesWithCosts<T>& start_states,
    const std::function<bool(const T&)>& goal_check_fn,
    const std::function<StatesWithCosts<T>(const T&)>&
        generate_valid_children_fn,
    const std::function<double(const T&)>& heuristic_fn,
    const bool limit_pqueue_duplicates = true,
    const StateHash& hasher = StateHash(),
    const StateEqual& equaler = StateEqual())
{
  // Sanity check
  if (start_states.empty())
  {
    throw std::invalid_argument("start_states cannot be empty");
  }

  // Check if a provided start state meets goal conditions
  int64_t best_start_meeting_goal_index = -1;
  double best_start_meeting_goal_cost = std::numeric_limits<double>::infinity();

  for (size_t idx = 0; idx < start_states.size(); idx++)
  {
    const T& start_state = start_states.at(idx).State();
    const double start_cost = start_states.at(idx).Cost();
    if (goal_check_fn(start_state))
    {
      if (start_cost < best_start_meeting_goal_cost)
      {
        best_start_meeting_goal_index = static_cast<int64_t>(idx);
        best_start_meeting_goal_cost = start_cost;
      }
    }
  }

  // Return early if we already have a solution
  if (best_start_meeting_goal_index >= 0)
  {
    const auto& best_start_meeting_goal =
        start_states.at(static_cast<size_t>(best_start_meeting_goal_index));

    Container solution;
    solution.push_back(best_start_meeting_goal.State());

    return AstarResult<T, Container>(solution, best_start_meeting_goal.Cost());
  }

  // Setup
  std::priority_queue<AstarPQueueElement<T>,
                      std::vector<AstarPQueueElement<T>>,
                      CompareAstarPQueueElementFn<T>> queue;

  // We must specific an initial bucket count to unordered_map constructors.
  const size_t initial_bucket_count = 100;

  // Optional map to reduce the number of duplicate items added to the pqueue
  // Key is the node ID
  // Value is cost-to-come
  std::unordered_map<T, double, StateHash, StateEqual> queue_members_map(
      initial_bucket_count, hasher, equaler);

  // Key is the node ID
  // Value is backpointer + cost-to-come
  // backpointer is the parent node ID
  ExploredList<T, StateHash, StateEqual> explored(
      initial_bucket_count, hasher, equaler);

  // Initialize
  for (const auto& start_state_and_cost : start_states)
  {
    const T& start_state = start_state_and_cost.State();
    const double start_cost = start_state_and_cost.Cost();
    queue.push(AstarPQueueElement<T>(
        start_state, start_cost, heuristic_fn(start_state)));
    if (limit_pqueue_duplicates)
    {
      queue_members_map[start_state] = start_cost;
    }
  }

  // Storage for the goal state (once found)
  OwningMaybe<T> goal_state;

  // Search
  while (queue.size() > 0)
  {
    // Get the top of the priority queue
    const AstarPQueueElement<T> top_node = queue.top();
    queue.pop();

    // Remove from queue map if necessary
    if (limit_pqueue_duplicates)
    {
      queue_members_map.erase(top_node.State());
    }

    // Check if the node has already been discovered
    const auto node_explored_find_itr = explored.find(top_node.State());
    // We have not been here before, or it is cheaper now
    const bool node_in_explored = (node_explored_find_itr != explored.end());
    const bool node_explored_is_better
        = (node_in_explored)
          ? (top_node.CostToCome()
              >= node_explored_find_itr->second.CostToCome())
          : false;

    if (!node_explored_is_better)
    {
      // Add to the explored list
      explored[top_node.State()] = BackPointerAndCostToCome<T>(top_node);

      // Check if we have reached the goal
      if (goal_check_fn(top_node.State()))
      {
        goal_state = OwningMaybe<T>(top_node.State());
        break;
      }

      // Generate valid children
      const StatesWithCosts<T> children_with_costs
          = generate_valid_children_fn(top_node.State());

      for (const auto& child_with_cost : children_with_costs)
      {
        const T& child_state = child_with_cost.State();
        const double parent_to_child_cost = child_with_cost.Cost();

        // Compute the cost-to-come for the new child
        const double parent_cost_to_come = top_node.CostToCome();
        const double child_cost_to_come
            = parent_cost_to_come + parent_to_child_cost;

        // Check if the child state has already been explored
        const auto child_explored_find_itr = explored.find(child_state);
        // Check if it has already been explored with lower cost
        const bool child_in_explored
            = (child_explored_find_itr != explored.end());
        const bool explored_child_is_better
            = (child_in_explored)
              ? (child_cost_to_come
                  >= child_explored_find_itr->second.CostToCome())
              : false;

        // Check if the child state is already in the queue
        bool queue_is_better = false;
        if (limit_pqueue_duplicates)
        {
          const auto queue_members_map_itr
              = queue_members_map.find(child_state);
          const bool in_queue
              = (queue_members_map_itr != queue_members_map.end());
          queue_is_better
              = (in_queue)
                ? (child_cost_to_come >= queue_members_map_itr->second)
                : false;
        }

        // Only add the new state if we need to
        if (!explored_child_is_better && !queue_is_better)
        {
          // Compute the heuristic for the child
          const double child_heuristic = heuristic_fn(child_state);
          // Compute the child value
          const double child_value = child_cost_to_come + child_heuristic;
          // Push onto the pqueue
          queue.push(AstarPQueueElement<T>(
              child_state, top_node.State(), child_cost_to_come, child_value));
          // Add to the queue member map
          if (limit_pqueue_duplicates)
          {
            queue_members_map[child_state] = child_cost_to_come;
          }
        }
      }
    }
  }

  // Package search result
  if (goal_state)
  {
    return ExtractAstarResult<T, Container, StateHash, StateEqual>(
        explored, goal_state.Value());
  }
  else
  {
    return AstarResult<T, Container>();
  }
}

/// Perform A* search.
/// @return Solution path between one of the starting states in
/// @param start_states and the goal (as defined by @param goal_check_fn), if
/// one exists, or an empty path.
/// @param generate_children_fn returns the child states for the provided state
/// together with the cost of moving from provided state to the child state.
/// @param edge_validity_check_fn returns true if the edge between the provided
/// states is valid.
/// @param distance_fn returns the distance between the provided states.
/// @param heuristic_fn returns the heuristic value for the provided state.
/// @param limit_pqueue_duplicates selects whether to use a second hashtable to
/// track elements in the pqueue and reduce duplicates added to the pqueue. This
/// usually improves performance.
/// Note that @param generate_children_fn and @param edge_validity_check overlap
/// in functionality and that @param edge_validity_check can always return true
/// if @param generate_children_fn always generates valid children. In such
/// cases, consider using the above function instead, which takes valid child
/// nodes plus cost.
template<typename T, typename Container=std::vector<T>,
         typename StateHash=std::hash<T>, typename StateEqual=std::equal_to<T>>
inline AstarResult<T, Container> PerformAstarSearch(
    const StatesWithCosts<T>& start_states,
    const std::function<bool(const T&)>& goal_check_fn,
    const std::function<Container(const T&)>& generate_children_fn,
    const std::function<bool(const T&, const T&)>&
        edge_validity_check_fn,
    const std::function<double(const T&, const T&)>& distance_fn,
    const std::function<double(const T&)>& heuristic_fn,
    const bool limit_pqueue_duplicates = true,
    const StateHash& hasher = StateHash(),
    const StateEqual& equaler = StateEqual())
{
  // Make a valid-child-states-only function
  const auto valid_child_state_function = [&] (const T& current_state)
  {
    StatesWithCosts<T> valid_child_states_and_costs;

    // Generate possible children
    const Container candidate_children = generate_children_fn(current_state);

    // Loop through potential child nodes
    for (const T& child_state : candidate_children)
    {
      // Check if the top node->child edge is valid
      if (edge_validity_check_fn(current_state, child_state))
      {
        const double parent_to_child_cost
              = distance_fn(current_state, child_state);
        valid_child_states_and_costs.emplace_back(
            child_state, parent_to_child_cost);
      }
    }
    return valid_child_states_and_costs;
  };

  return PerformAstarSearch<T, Container, StateHash, StateEqual>(
      start_states, goal_check_fn, valid_child_state_function, heuristic_fn,
      limit_pqueue_duplicates, hasher, equaler);
}

/// Perform A* search.
/// @return Solution path between the start state and goal state, if one exists,
/// or an empty path.
/// @param generate_children_fn returns the child states for the provided state
/// together with the cost of moving from provided state to the child state.
/// @param edge_validity_check_fn returns true if the edge between the provided
/// states is valid.
/// @param distance_fn returns the distance between the provided states.
/// @param heuristic_fn returns the heuristic value for the provided state.
/// @param limit_pqueue_duplicates selects whether to use a second hashtable to
/// track elements in the pqueue and reduce duplicates added to the pqueue. This
/// usually improves performance.
/// Note that @param generate_children_fn and @param edge_validity_check overlap
/// in functionality and that @param edge_validity_check can always return true
/// if @param generate_children_fn always generates valid children. In such
/// cases, consider using the above function instead, which takes valid child
/// nodes plus cost.
template<typename T, typename Container=std::vector<T>,
         typename StateHash=std::hash<T>, typename StateEqual=std::equal_to<T>>
inline AstarResult<T, Container> PerformAstarSearch(
    const T& start_state, const T& goal_state,
    const std::function<Container(const T&)>& generate_children_fn,
    const std::function<bool(const T&, const T&)>&
        edge_validity_check_fn,
    const std::function<double(const T&, const T&)>& distance_fn,
    const std::function<double(const T&, const T&)>& heuristic_fn,
    const bool limit_pqueue_duplicates = true,
    const StateHash& hasher = StateHash(),
    const StateEqual& equaler = StateEqual())
{
  // Make goal check function
  const auto goal_check_function = [&] (const T& current_state)
  {
    return equaler(current_state, goal_state);
  };

  // Make heuristic helper function
  const auto heuristic_function = [&] (const T& current_state)
  {
    return heuristic_fn(current_state, goal_state);
  };

  StatesWithCosts<T> start_states;
  start_states.emplace_back(start_state);

  return PerformAstarSearch<T, Container, StateHash, StateEqual>(
      start_states, goal_check_function, generate_children_fn,
      edge_validity_check_fn, distance_fn, heuristic_function,
      limit_pqueue_duplicates, hasher, equaler);
}
}  // namespace simple_astar_search
}  // namespace common_robotics_utilities
