#pragma once

#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include <common_robotics_utilities/maybe.hpp>

namespace common_robotics_utilities
{
namespace simple_astar_search
{
/// Wrapper for P-Queue elements used in A* search. Contains the unique node_id,
/// "backpointer" to the parent node_id, cost to come, and state value.
class AstarPQueueElement
{
protected:

  int64_t node_id_ = 0;
  double cost_to_come_ = 0.0;
  double value_ = 0.0;
  OwningMaybe<int64_t> back_pointer_;

public:

  AstarPQueueElement(const int64_t node_id, const int64_t back_pointer,
                     const double cost_to_come, const double value)
      : node_id_(node_id), cost_to_come_(cost_to_come), value_(value),
        back_pointer_(OwningMaybe<int64_t>(back_pointer)) {}

  AstarPQueueElement(const int64_t node_id, const double value)
      : node_id_(node_id), cost_to_come_(0.0), value_(value) {}

  int64_t NodeID() const { return node_id_; }

  int64_t BackPointer() const { return back_pointer_.Value(); }

  bool HasBackPointer() const { return back_pointer_.HasValue(); }

  const OwningMaybe<int64_t>& RawBackPointer() const { return back_pointer_; }

  double CostToCome() const { return cost_to_come_; }

  double Value() const { return value_; }
};

/// Comparator for AstarPQueueElements for use in std::priority_queue.
class CompareAstarPQueueElementFn
{
public:

  bool operator()(const AstarPQueueElement& lhs,
                  const AstarPQueueElement& rhs) const
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

/// Internally, A* plans over node ids, rather than node values.
/// Search result is both path and path cost.
/// Path is a vector of node_ids, and cost is the computed cost-to-come of the
/// goal node.
using AstarIndexResult = AstarResult<int64_t, std::vector<int64_t>>;

/// Class to wrap the backpointer and cost-to-come, as used in the explored
/// list in A*.
class BackPointerAndCostToCome
{
private:
  double cost_to_come_ = std::numeric_limits<double>::infinity();
  OwningMaybe<int64_t> back_pointer_;

public:
  // TODO(calderpg) Once C++17 is required, remove the default constructor and
  // replace map[] with map.insert_or_assign().
  BackPointerAndCostToCome() {}

  explicit BackPointerAndCostToCome(const AstarPQueueElement& pqueue_element)
      : cost_to_come_(pqueue_element.CostToCome()),
        back_pointer_(pqueue_element.RawBackPointer()) {}

  int64_t BackPointer() const { return back_pointer_.Value(); }

  bool HasBackPointer() const { return back_pointer_.HasValue(); }

  const OwningMaybe<int64_t>& RawBackPointer() const { return back_pointer_; }

  double CostToCome() const { return cost_to_come_; }
};

using ExploredList = std::unordered_map<int64_t, BackPointerAndCostToCome>;

/// Extract the result from the explored node map @param explored produced by A*
/// search, with @param start_id the node_id of the start node and @param
/// goal_id the node_id of the goal node.
inline AstarIndexResult ExtractAstarResult(
    const ExploredList& explored, const int64_t start_id, const int64_t goal_id)
{
  // Check if a solution was found
  const auto goal_state_itr = explored.find(goal_id);
  // If no solution found
  if (goal_state_itr == explored.end())
  {
    return AstarIndexResult();
  }
  // If a solution was found
  else
  {
    // Extract the path node ids in reverse order
    std::vector<int64_t> solution_path_ids;
    solution_path_ids.push_back(goal_id);
    OwningMaybe<int64_t> current_back_pointer =
        goal_state_itr->second.RawBackPointer();
    while (current_back_pointer)
    {
      const int64_t current_id = current_back_pointer.Value();
      // Using map.at(key) throws an exception if key not found
      // This provides bounds safety check
      const auto& current_state = explored.at(current_id);
      solution_path_ids.push_back(current_id);
      if (current_id == start_id)
      {
        break;
      }
      else
      {
        current_back_pointer = current_state.RawBackPointer();
      }
    }
    // Reverse
    std::reverse(solution_path_ids.begin(), solution_path_ids.end());
    // Get the cost of the path
    const double solution_path_cost = goal_state_itr->second.CostToCome();
    return AstarIndexResult(solution_path_ids, solution_path_cost);
  }
}

/// Perform A* search. This implementation is somewhat different structurally
/// from many implementations - it operates excusively on "node IDs" that
/// uniquely and reproducibly identify a single state. This is used because A*
/// needs to insert states into std::unordered_map and instead of refering to
/// states and their hashes, we only refer to them by their IDs. This means you,
/// the caller, need to maintain a mapping between IDs and values so that you
/// can translate the IDs used here with the real values of the state. See the
/// implementation below in PerformAstarSearch to see how this may be performed.
/// @return Solution path between @param start_id and the goal (as defined by
/// @goal_check_fn), if one exists, or an empty path.
/// @param generate_children_fn returns the node IDs of child states for the
/// provided node ID, @param edge_validity_check_fn returns true if the edge
/// between provided node IDs is valid, @param distance_fn returns the distance
/// between provided node IDs, and @param heuristic_fn returns the heuristic
/// value for the provided node ID. @param limit_pqueue_duplicates selects
/// whether to use a second hashtable to track elements in the pqueue and reduce
/// duplicates added to the pqueue. This usually improves performance.
/// Note that @param generate_children_fn and @param edge_validity_check overlap
/// in functionality and that @param edge_validity_check can always return true
/// if @param generate_children_fn always generates valid children.
inline AstarIndexResult PerformGenericAstarSearch(
    const int64_t start_id,
    const std::function<bool(const int64_t)>& goal_check_fn,
    const std::function<std::vector<int64_t>(
      const int64_t)>& generate_children_fn,
    const std::function<bool(const int64_t,
                             const int64_t)>& edge_validity_check_fn,
    const std::function<double(const int64_t, const int64_t)>& distance_fn,
    const std::function<double(const int64_t)>& heuristic_fn,
    const bool limit_pqueue_duplicates)
{
  // Enforced sanity checks
  if (goal_check_fn(start_id))
  {
    return AstarIndexResult({start_id}, 0.0);
  }
  // Setup
  std::priority_queue<AstarPQueueElement,
                      std::vector<AstarPQueueElement>,
                      CompareAstarPQueueElementFn> queue;
  // Optional map to reduce the number of duplicate items added to the pqueue
  // Key is the node ID
  // Value is cost-to-come
  std::unordered_map<int64_t, double> queue_members_map;
  // Key is the node ID
  // Value is backpointer + cost-to-come
  // backpointer is the parent node ID
  ExploredList explored;
  // Initialize
  queue.push(AstarPQueueElement(start_id, heuristic_fn(start_id)));
  if (limit_pqueue_duplicates)
  {
    queue_members_map[start_id] = 0.0;
  }
  // Storage for the goal state (once found)
  OwningMaybe<int64_t> goal_id;
  // Search
  while (queue.size() > 0)
  {
    // Get the top of the priority queue
    const AstarPQueueElement top_node = queue.top();
    queue.pop();
    // Remove from queue map if necessary
    if (limit_pqueue_duplicates)
    {
      queue_members_map.erase(top_node.NodeID());
    }
    // Check if the node has already been discovered
    const auto node_explored_find_itr = explored.find(top_node.NodeID());
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
      explored[top_node.NodeID()] = BackPointerAndCostToCome(top_node);
      // Check if we have reached the goal
      if (goal_check_fn(top_node.NodeID()))
      {
        goal_id = OwningMaybe<int64_t>(top_node.NodeID());
        break;
      }
      // Generate possible children
      const std::vector<int64_t> candidate_children
          = generate_children_fn(top_node.NodeID());
      // Loop through potential child nodes
      for (const int64_t child_node_id : candidate_children)
      {
        // Check if the top node->child edge is valid
        if (edge_validity_check_fn(top_node.NodeID(), child_node_id))
        {
          // Compute the cost-to-come for the new child
          const double parent_cost_to_come = top_node.CostToCome();
          const double parent_to_child_cost
              = distance_fn(top_node.NodeID(), child_node_id);
          const double child_cost_to_come
              = parent_cost_to_come + parent_to_child_cost;
          // Check if the child state has already been explored
          const auto child_explored_find_itr = explored.find(child_node_id);
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
                = queue_members_map.find(child_node_id);
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
            const double child_heuristic = heuristic_fn(child_node_id);
            // Compute the child value
            const double child_value = child_cost_to_come + child_heuristic;
            // Push onto the pqueue
            queue.push(AstarPQueueElement(child_node_id, top_node.NodeID(),
                                          child_cost_to_come, child_value));
            // Add to the queue member map
            if (limit_pqueue_duplicates)
            {
              queue_members_map[child_node_id] = child_cost_to_come;
            }
          }
        }
      }
    }
  }
  if (goal_id)
  {
    return ExtractAstarResult(explored, start_id, goal_id.Value());
  }
  else
  {
    return AstarIndexResult();
  }
}

/// Perform A* search. This implementation is somewhat different structurally
/// from many implementations - it operates excusively on "node IDs" that
/// uniquely and reproducibly identify a single state. This is used because A*
/// needs to insert states into std::unordered_map and instead of refering to
/// states and their hashes, we only refer to them by their IDs. This means you,
/// the caller, need to maintain a mapping between IDs and values so that you
/// can translate the IDs used here with the real values of the state. See the
/// implementation below in PerformAstarSearch to see how this may be performed.
/// @return Solution path between @param start_id and @param goal_id, if one
/// exists, or an empty path. @param generate_children_fn returns the node IDs
/// of child states for the provided node ID, @param edge_validity_check_fn
/// returns true if the edge between provided node IDs is valid, @param
/// distance_fn returns the distance between provided node IDs, and @param
/// heuristic_fn returns the heuristic value between provided node IDs.
/// @param limit_pqueue_duplicates selects whether to use a second hashtable to
/// track elements in the pqueue and reduce duplicates added to the pqueue. This
/// usually improves performance.
/// Note that @param generate_children_fn and @param edge_validity_check overlap
/// in functionality and that @param edge_validity_check can always return true
/// if @param generate_children_fn always generates valid children.
inline AstarIndexResult PerformGenericAstarSearch(
    const int64_t start_id, const int64_t goal_id,
    const std::function<std::vector<int64_t>(
      const int64_t)>& generate_children_fn,
    const std::function<bool(const int64_t,
                             const int64_t)>& edge_validity_check_fn,
    const std::function<double(const int64_t, const int64_t)>& distance_fn,
    const std::function<double(const int64_t, const int64_t)>& heuristic_fn,
    const bool limit_pqueue_duplicates)
{
  // Make goal check function
  const auto goal_check_function = [&] (const int64_t node_id)
  {
    return (node_id == goal_id);
  };
  // Make heuristic helper function
  const auto heuristic_function = [&] (const int64_t node_id)
  {
    return heuristic_fn(node_id, goal_id);
  };
  return PerformGenericAstarSearch(
      start_id, goal_check_function, generate_children_fn,
      edge_validity_check_fn, distance_fn, heuristic_function,
      limit_pqueue_duplicates);
}

/// Perform A* search.
/// @return Solution path between @param start and @param goal, if one
/// exists, or an empty path. @param generate_children_fn returns the child
/// states for the provided state, @param edge_validity_check_fn returns true if
/// the edge between provided states is valid, @param distance_fn returns the
///  distance between provided states, and @param heuristic_fn returns the
///  heuristic value between provided states. @param state_id_fn produces
/// distinct state IDs (for all states, two states are identical if their IDs
/// are the same, and if two states are different, their IDs must be different).
///  @param limit_pqueue_duplicates selects whether to use a second hashtable to
/// track elements in the pqueue and reduce duplicates added to the pqueue. This
/// usually improves performance.
/// Note that @param generate_children_fn and @param edge_validity_check overlap
/// in functionality and that @param edge_validity_check can always return true
/// if @param generate_children_fn always generates valid children.
template<typename T, typename Container=std::vector<T>>
inline AstarResult<T, Container> PerformAstarSearch(
    const T& start, const T& goal,
    const std::function<Container(const T&)>& generate_children_fn,
    const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
    const std::function<double(const T&, const T&)>& distance_fn,
    const std::function<double(const T&, const T&)>& heuristic_fn,
    const std::function<int64_t(const T&)>& state_id_fn,
    const bool limit_pqueue_duplicates)
{
  // Make the state storage and add the start & goal states
  std::unordered_map<int64_t, T> state_map;
  const int64_t start_id = state_id_fn(start);
  state_map[start_id] = start;
  const int64_t goal_id = state_id_fn(goal);
  state_map[goal_id] = goal;
  // Bind new helper functions
  const std::function<std::vector<int64_t>(
    const int64_t)> generate_children_function = [&] (const int64_t state_id)
  {
    const T& state = state_map.at(state_id);
    const auto child_states = generate_children_fn(state);
    std::vector<int64_t> child_state_ids(child_states.size());
    for (size_t idx = 0; idx < child_states.size(); idx++)
    {
      const T& child_state = child_states.at(idx);
      const int64_t child_state_id = state_id_fn(child_state);
      state_map[child_state_id] = child_state;
      child_state_ids.at(idx) = child_state_id;
    }
    return child_state_ids;
  };
  const std::function<bool(const int64_t,
                           const int64_t)> edge_validity_check_function
      = [&] (const int64_t start_state_id, const int64_t end_state_id)
  {
    const T& start_state = state_map.at(start_state_id);
    const T& end_state = state_map.at(end_state_id);
    return edge_validity_check_fn(start_state, end_state);
  };
  const std::function<double(const int64_t, const int64_t)> distance_function
      = [&] (const int64_t start_state_id, const int64_t end_state_id)
  {
    const T& start_state = state_map.at(start_state_id);
    const T& end_state = state_map.at(end_state_id);
    return distance_fn(start_state, end_state);
  };
  const std::function<double(const int64_t, const int64_t)> heuristic_function
      = [&] (const int64_t start_state_id, const int64_t end_state_id)
  {
    const T& start_state = state_map.at(start_state_id);
    const T& end_state = state_map.at(end_state_id);
    return heuristic_fn(start_state, end_state);
  };
  // Call A*
  const auto astar_solution
      = PerformGenericAstarSearch(
          start_id, goal_id, generate_children_function,
          edge_validity_check_function, distance_function, heuristic_function,
          limit_pqueue_duplicates);
  // Extract the solution path
  Container solution_path;
  solution_path.reserve(astar_solution.Path().size());
  for (const int64_t state_id : astar_solution.Path())
  {
    const T& solution_path_state = state_map.at(state_id);
    solution_path.push_back(solution_path_state);
  }
  solution_path.shrink_to_fit();
  return AstarResult<T, Container>(solution_path, astar_solution.PathCost());
}
}  // namespace simple_astar_search
}  // namespace common_robotics_utilities
