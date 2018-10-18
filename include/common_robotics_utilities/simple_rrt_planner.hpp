#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/serialization.hpp>
#include <common_robotics_utilities/simple_knearest_neighbors.hpp>
#include <common_robotics_utilities/utility.hpp>

namespace common_robotics_utilities
{
namespace simple_rrt_planner
{
/// Basic templated tree node for use in RRT planners. Node stores a templated
/// value, parent index, and child indices.
template<typename T>
class SimpleRRTPlannerState
{
private:
  T value_;
  std::vector<int64_t> child_indices_;
  int64_t parent_index_ = -1;
  bool initialized_ = false;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static uint64_t Serialize(
      const SimpleRRTPlannerState<T>& state,
      std::vector<uint8_t>& buffer,
      const std::function<uint64_t(
          const T&, std::vector<uint8_t>&)>& value_serializer)
  {
    return state.SerializeSelf(buffer, value_serializer);
  }

  static std::pair<SimpleRRTPlannerState<T>, uint64_t> Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset,
      const std::function<std::pair<T, uint64_t>(
          const std::vector<uint8_t>&, const uint64_t)>& value_deserializer)
  {
    SimpleRRTPlannerState<T> temp_state;
    const uint64_t bytes_read
        = temp_state.DeserializeSelf(buffer, starting_offset,
                                     value_deserializer);
    return std::make_pair(temp_state, bytes_read);
  }

  SimpleRRTPlannerState() : parent_index_(-1), initialized_(false) {}

  SimpleRRTPlannerState(const T& value,
                        const int64_t parent_index,
                        const std::vector<int64_t>& child_indices)
  {
    value_ = value;
    child_indices_ = child_indices;
    parent_index_ = parent_index;
    initialized_ = true;
  }

  SimpleRRTPlannerState(const T& value, const int64_t parent_index)
  {
    value_ = value;
    parent_index_ = parent_index;
    initialized_ = true;
  }

  explicit SimpleRRTPlannerState(const T& value)
  {
    value_ = value;
    parent_index_ = -1;
    initialized_ = true;
  }

  uint64_t SerializeSelf(
      std::vector<uint8_t>& buffer,
      const std::function<uint64_t(
          const T&, std::vector<uint8_t>&)>& value_serializer) const
  {
    const uint64_t start_buffer_size = buffer.size();
    // Serialize the value
    value_serializer(value_, buffer);
    // Serialize the child indices
    serialization::SerializeMemcpyableVectorLike<int64_t>(
        child_indices_, buffer);
    // Serialize the parent index
    serialization::SerializeMemcpyable<int64_t>(parent_index_, buffer);
    // Serialize the initialized
    serialization::SerializeMemcpyable<uint8_t>(
        static_cast<uint8_t>(initialized_), buffer);
    // Figure out how many bytes were written
    const uint64_t end_buffer_size = buffer.size();
    const uint64_t bytes_written = end_buffer_size - start_buffer_size;
    return bytes_written;
  }

  uint64_t DeserializeSelf(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset,
      const std::function<std::pair<T, uint64_t>(
          const std::vector<uint8_t>&, const uint64_t)>& value_deserializer)
  {
    uint64_t current_position = starting_offset;
    // Deserialize the value
    const std::pair<T, uint64_t> value_deserialized
        = value_deserializer(buffer, current_position);
    value_ = value_deserialized.first;
    current_position += value_deserialized.second;
    // Deserialize the child indices
    const std::pair<std::vector<int64_t>, uint64_t> child_indices_deserialized
        = serialization::DeserializeMemcpyableVectorLike<int64_t>(
            buffer, current_position);
    child_indices_ = child_indices_deserialized.first;
    current_position += child_indices_deserialized.second;
    // Deserialize the parent index
    const std::pair<int64_t, uint64_t> parent_index_deserialized
        = serialization::DeserializeMemcpyable<int64_t>(buffer,
                                                        current_position);
    parent_index_ = parent_index_deserialized.first;
    current_position += parent_index_deserialized.second;
    // Deserialize the initialized
    const std::pair<uint8_t, uint64_t> initialized_deserialized
        = serialization::DeserializeMemcpyable<uint8_t>(buffer,
                                                        current_position);
    initialized_ = static_cast<bool>(initialized_deserialized.first);
    current_position += initialized_deserialized.second;
    // Figure out how many bytes were read
    const uint64_t bytes_read = current_position - starting_offset;
    return bytes_read;
  }

  bool IsInitialized() const { return initialized_; }

  const T& GetValueImmutable() const { return value_; }

  T& GetValueMutable() { return value_; }

  int64_t GetParentIndex() const { return parent_index_; }

  void SetParentIndex(const int64_t parent_index)
  {
    parent_index_ = parent_index;
  }

  const std::vector<int64_t>& GetChildIndices() const
  {
    return child_indices_;
  }

  void ClearChildIndicies() { child_indices_.clear(); }

  void AddChildIndex(const int64_t child_index)
  {
    for (size_t idx = 0; idx < child_indices_.size(); idx++)
    {
      if (child_indices_[idx] == child_index)
      {
        return;
      }
    }
    child_indices_.push_back(child_index);
  }

  void RemoveChildIndex(const int64_t child_index)
  {
    std::vector<int64_t> new_child_indices;
    for (size_t idx = 0; idx < child_indices_.size(); idx++)
    {
      if (child_indices_[idx] != child_index)
      {
        new_child_indices.push_back(child_indices_[idx]);
      }
    }
    child_indices_ = new_child_indices;
  }
};

/// Checks tree @param nodes to make sure the parent-child linkages are correct.
/// This conservatively enforces that the tree does not conatin cycles by
/// enforcing parent_index < state_index < child_indices. This is sufficient to
/// ensure that the tree does not contain cycles, but it does not work for all
/// trees generally. however, this is enough for all trees produced by the RRT
/// planners. Note that this does *NOT* ensure that all nodes in @param nodes
/// form a single connected tree, since it can be useful to store multiple
/// trees in the same container.
template<typename T, typename Allocator=std::allocator<T>>
bool CheckTreeLinkage(const std::vector<SimpleRRTPlannerState<T>>& nodes)
{
  // Step through each state in the tree.
  // Make sure that the linkage to the parent and child states are correct.
  for (size_t current_index = 0; current_index < nodes.size(); current_index++)
  {
    // For every state, make sure all the parent<->child linkages are valid
    const SimpleRRTPlannerState<T>& current_state = nodes.at(current_index);
    if (!current_state.IsInitialized())
    {
      std::cerr << "Tree contains uninitialized node(s): "
                << current_index << std::endl;
      return false;
    }
    // Check the linkage to the parent state
    const int64_t parent_index = current_state.GetParentIndex();
    if (parent_index > static_cast<int64_t>(current_index))
    {
      std::cerr << "Invalid parent index " << parent_index << " for state "
                << current_index
                << " [Parent index cannot be greater than our own index]"
                << std::endl;
      return false;
    }
    if ((parent_index >= 0)
        && (parent_index < static_cast<int64_t>(nodes.size())))
    {
      if (parent_index != static_cast<int64_t>(current_index))
      {
        const SimpleRRTPlannerState<T>& parent_state = nodes.at(parent_index);
        // Make sure the parent state is initialized.
        if (!parent_state.IsInitialized())
        {
          std::cerr << "Tree contains uninitialized node(s) "
                    << parent_index << std::endl;
          return false;
        }
        // Make sure the corresponding parent contains the current node in its
        // list of child indices.
        const std::vector<int64_t>& parent_child_indices
            = parent_state.GetChildIndices();
        auto index_found = std::find(parent_child_indices.begin(),
                                     parent_child_indices.end(),
                                     static_cast<int64_t>(current_index));
        if (index_found == parent_child_indices.end())
        {
          std::cerr << "Parent state " << parent_index
                    << " does not contain child index for current node "
                    << current_index << std::endl;
          return false;
        }
      }
      else
      {
        std::cerr << "Invalid parent index " << parent_index << " for state "
                  << current_index << " [Parent index cannot be our own index]"
                  << std::endl;
        return false;
      }
    }
    else if (parent_index < -1)
    {
      std::cerr << "Invalid parent index " << parent_index << " for state "
                << current_index << " [Parent index < -1]" << std::endl;
      return false;
    }
    // Check the linkage to the child states
    const std::vector<int64_t>& current_child_indices
        = current_state.GetChildIndices();
    for (const int64_t current_child_index : current_child_indices)
    {
      if ((current_child_index > 0)
          && (current_child_index < static_cast<int64_t>(nodes.size())))
      {
        if (current_child_index > static_cast<int64_t>(current_index))
        {
          const SimpleRRTPlannerState<T>& child_state
              = nodes.at(current_child_index);
          if (!child_state.IsInitialized())
          {
            std::cerr << "Tree contains uninitialized node(s) "
                      << current_child_index << std::endl;
            return false;
          }
          // Make sure the child node points to us as the parent index
          const int64_t child_parent_index = child_state.GetParentIndex();
          if (child_parent_index != (int64_t)current_index)
          {
            std::cerr << "Parent index " << child_parent_index
                      << " for current child state " << current_child_index
                      << " does not match index " << current_index
                      << " for current node " << std::endl;
            return false;
          }
        }
        else if (current_child_index == static_cast<int64_t>(current_index))
        {
          std::cerr << "Invalid child index " << current_child_index
                    << " for state " << current_index
                    << " [Child index cannot be our own index]" << std::endl;
          return false;
        }
        else
        {
          std::cerr << "Invalid child index " << current_child_index
                    << " for state " << current_index
                    << " [Child index cannot be less than our own index]"
                    << std::endl;
          return false;
        }
      }
      else
      {
        std::cerr << "Invalid child index " << current_child_index
                  << " for state " << current_index << std::endl;
        return false;
      }
    }
  }
  return true;
}

/// Extracts a single solution path corresponding to the provided goal state.
/// @param nodes tree produced by RRT planner.
/// @param goal_state_index index of goal state in @param nodes.
template<typename T, typename Allocator=std::allocator<T>>
std::vector<T, Allocator> ExtractSolutionPath(
    const std::vector<SimpleRRTPlannerState<T>>& nodes,
    const int64_t goal_state_index)
{
  std::vector<T, Allocator> solution_path;
  const SimpleRRTPlannerState<T>& goal_state = nodes.at(goal_state_index);
  solution_path.push_back(goal_state.GetValueImmutable());
  int64_t parent_index = goal_state.GetParentIndex();
  while (parent_index >= 0)
  {
    const SimpleRRTPlannerState<T>& parent_state = nodes.at(parent_index);
    const T& parent = parent_state.GetValueImmutable();
    solution_path.push_back(parent);
    parent_index = parent_state.GetParentIndex();
  }
  // Put it in the right order
  std::reverse(solution_path.begin(), solution_path.end());
  return solution_path;
}

/// Extracts the solution paths corresponding to each of the provided goal
/// states.
/// /// @param nodes tree produced by RRT planner.
/// @param goal_state_indices indices of goal states in @param nodes.
template<typename T, typename Allocator=std::allocator<T>>
std::vector<std::vector<T, Allocator>> ExtractSolutionPaths(
    const std::vector<SimpleRRTPlannerState<T>>& nodes,
    const std::vector<int64_t>& goal_state_indices)
{
  std::vector<std::vector<T, Allocator>> solution_paths(
      goal_state_indices.size());
  for (size_t idx = 0; idx < goal_state_indices.size(); idx++)
  {
    std::vector<T, Allocator> solution_path
        = ExtractSolutionPath<T, Allocator>(nodes, goal_state_indices.at(idx));
    solution_paths.at(idx) = solution_path;
  }
  return solution_paths;
}

/// Plan multiple paths using a single-direction RRT planner. This planner will
/// continue to explore and add new solution paths until terminated. Note that
/// StateType and SampleType are different; this makes implementation of some
/// kinodynamic planning problems easier.
/// @param tree existing tree. This tree must contain at least one node, but can
/// contain multiple nodes as well. All nodes in @param tree are assumed to be
/// valid starting nodes, and if they are linked together, they must have valid
/// parent-child linkage.
/// @param sampling_fn function to sample a new state.
/// @param nearest_neighbor_fn function to compute the nearest neighbor index
/// in the provided tree given the provided sample. If a negative index
/// (showing that no valid nearest neighbor could be found) planning is
/// terminated.
/// @param forward_propagation_fn function to forward propagate states from
/// the provided state to the provided sample. The propagated states are
/// returned as a vector<pair<state, relative_index>> in which state is the new
/// propagated state and relative_index is a relative index used to connect the
/// new state to the correct parent state. This relative_index is the index of
/// the parent state, relative to the vector of propagated nodes. A negative
/// value means the nearest neighbor in the tree, zero means the first
/// propagated node, and so on. NOTE - the relative parent index *must* be lower
/// than the index in the list of prograted nodes i.e. the first node must have
/// a negative value, and so on. While complicated, this structure allows @param
/// forward_propagation_fn to return an entire subtree at once.
/// @param state_added_callback_fn callback function called once a state is
/// added to the tree, providing a mutable reference to the tree and the index
/// of the newly-added state. This can be used, for example, to update a KD-tree
/// used for nearest neighbors. You can leave this default-constructed ({}) if
/// you do not need it.
/// @param check_goal_reached_fn function to check if the provided state meets
/// goal conditions.
/// @param goal_reached_callback_fn callback function called once a state meets
/// goal conditions, providing a mutable reference to the tree and the index of
/// the new goal state. You can leave this default-constructed ({}) if you do
/// not need it.
/// @param termination_check_fn Returns true if planning has been
/// terminated. The provided int64_t is the current size of the planner tree,
/// which may be useful for a size-limited planning problem.
/// @return pair<paths, statistics> where paths is a vector of solution paths
/// and statistics is a map<string, double> of useful statistics collected while
/// planning.
template<typename StateType,
         typename SampleType=StateType,
         typename StateAllocator=std::allocator<StateType>>
std::pair<std::vector<std::vector<StateType, StateAllocator>>,
          std::map<std::string, double>> RRTPlanMultiPath(
    std::vector<SimpleRRTPlannerState<StateType>>& tree,
    const std::function<SampleType(void)>& sampling_fn,
    const std::function<int64_t(
        const std::vector<SimpleRRTPlannerState<StateType>>&,
        const SampleType&)>& nearest_neighbor_fn,
    const std::function<std::vector<std::pair<StateType, int64_t>>(
        const StateType&, const SampleType&)>& forward_propagation_fn,
    const std::function<void(
        std::vector<SimpleRRTPlannerState<StateType>>&,
        const int64_t)>& state_added_callback_fn,
    const std::function<bool(const StateType&)>& check_goal_reached_fn,
    const std::function<void(
        std::vector<SimpleRRTPlannerState<StateType>>&,
        const int64_t)>& goal_reached_callback_fn,
    const std::function<bool(const int64_t)>& termination_check_fn)
{
  // Make sure we've been given a start state
  if (tree.empty())
  {
    throw std::invalid_argument(
        "Must be called with at least one node in tree");
  }
  // Keep track of statistics
  std::map<std::string, double> statistics;
  statistics["total_samples"] = 0.0;
  statistics["successful_samples"] = 0.0;
  statistics["failed_samples"] = 0.0;
  // Storage for the goal states we reach
  std::vector<int64_t> goal_state_indices;
  // Safety check before doing real work
  for (size_t idx = 0; idx < tree.size(); idx++)
  {
    if (check_goal_reached_fn(tree[idx].GetValueImmutable()))
    {
      std::cerr << "Starting node " << idx
                << " meets goal conditions, adding to goal states" << std::endl;
      goal_state_indices.push_back(static_cast<int64_t>(idx));
      goal_reached_callback_fn(tree, static_cast<int64_t>(idx));
    }
  }
  // Update the start time
  const std::chrono::time_point<std::chrono::steady_clock> start_time
      = std::chrono::steady_clock::now();
  // Plan
  while (!termination_check_fn(static_cast<int64_t>(tree.size())))
  {
    // Sample a random goal
    const StateType random_target = sampling_fn();
    statistics["total_samples"] += 1.0;
    // Get the nearest neighbor
    const int64_t nearest_neighbor_index
        = nearest_neighbor_fn(tree, random_target);
    // nearest_neighbor_index < 0 is handled as a special case of early
    // termination.
    if (UNLIKELY(nearest_neighbor_index < 0))
    {
      break;
    }
    const StateType& nearest_neighbor
        = tree.at(nearest_neighbor_index).GetValueImmutable();
    // Forward propagate towards the goal
    const std::vector<std::pair<StateType, int64_t>> propagated
        = forward_propagation_fn(nearest_neighbor, random_target);
    if (!propagated.empty())
    {
      statistics["successful_samples"] += 1.0;
      for (size_t idx = 0; idx < propagated.size(); idx++)
      {
        const std::pair<StateType, int64_t>& current_propagation
            = propagated.at(idx);
        // Determine the parent index of the new state
        // This process deserves some explanation
        // The "current relative parent index" is the index of the parent,
        // relative to the list of propagated nodes. A negative value means the
        // nearest neighbor in the tree, zero means the first propagated node,
        // and so on. NOTE - the relative parent index *must* be lower than the
        // index in the list of prograted nodes i.e. the first node must have a
        // negative value, and so on.
        const int64_t& current_relative_parent_index
            = current_propagation.second;
        int64_t node_parent_index = nearest_neighbor_index;
        if (current_relative_parent_index >= 0)
        {
          const int64_t current_relative_index = static_cast<int64_t>(idx);
          if (current_relative_parent_index >= current_relative_index)
          {
            throw std::invalid_argument(
                "Linkage with relative parent index >="
                " current relative index is invalid");
          }
          const int64_t current_relative_offset
              = current_relative_parent_index - current_relative_index;
          const int64_t current_nodes_size = static_cast<int64_t>(tree.size());
          // Remember that current_relative_offset is negative!
          node_parent_index = current_nodes_size + current_relative_offset;
        }
        else
        {
          // Negative relative parent index means our parent index is the
          // nearest neighbor index.
          node_parent_index = nearest_neighbor_index;
        }
        // Build the new state
        const StateType& current_propagated = current_propagation.first;
        // Add the state to the tree
        tree.emplace_back(
            SimpleRRTPlannerState<StateType>(current_propagated,
                                             node_parent_index));
        const int64_t new_node_index = static_cast<int64_t>(tree.size() - 1);
        tree.at(node_parent_index).AddChildIndex(new_node_index);
        // Call the state added callback
        if (state_added_callback_fn)
        {
          state_added_callback_fn(tree, new_node_index);
        }
        // Check if we've reached the goal
        if (check_goal_reached_fn(tree.at(new_node_index).GetValueImmutable()))
        {
          goal_state_indices.push_back(new_node_index);
          if (goal_reached_callback_fn)
          {
            goal_reached_callback_fn(tree, new_node_index);
          }
        }
      }
    }
    else
    {
      statistics["failed_samples"] += 1.0;
    }
  }
  // Put together the results
  const std::vector<std::vector<StateType, StateAllocator>> planned_paths
      = ExtractSolutionPaths<StateType, StateAllocator>(
          tree, goal_state_indices);
  const std::chrono::time_point<std::chrono::steady_clock> cur_time
      = std::chrono::steady_clock::now();
  const std::chrono::duration<double> planning_time(cur_time - start_time);
  statistics["planning_time"] = planning_time.count();
  statistics["total_states"] = static_cast<double>(tree.size());
  statistics["solutions"] = static_cast<double>(planned_paths.size());
  return std::make_pair(planned_paths, statistics);
}

/// Plan multiple paths using a bidirectional RRT planner. This planner will
/// continue to explore and add new solution paths until terminated.
/// @param start_tree existing starting tree. This tree must contain at least
/// one node, but can contain multiple nodes as well. All nodes in @param
/// start_tree are assumed to be valid starting nodes, and if they are linked
/// together, they must have valid parent-child linkage.
/// @param goal_tree existing goal tree. This tree must contain at least one
/// node, but can contain multiple nodes as well. All nodes in @param
/// goal_tree are assumed to be valid goal nodes, and if they are linked
/// together, they must have valid parent-child linkage.
/// @param state_sampling_fn function to sample a new state.
/// @param nearest_neighbor_fn function to compute the nearest neighbor index
/// in the provided tree given the provided sample. If a negative index
/// (showing that no valid nearest neighbor could be found) planning is
/// terminated.
/// @param forward_propagation_fn function to forward propagate states from
/// the provided state to the provided sample. The propagated states are
/// returned as a vector<pair<state, relative_index>> in which state is the new
/// propagated state and relative_index is a relative index used to connect the
/// new state to the correct parent state. This relative_index is the index of
/// the parent state, relative to the vector of propagated nodes. A negative
/// value means the nearest neighbor in the tree, zero means the first
/// propagated node, and so on. NOTE - the relative parent index *must* be lower
/// than the index in the list of prograted nodes i.e. the first node must have
/// a negative value, and so on. While complicated, this structure allows @param
/// forward_propagation_fn to return an entire subtree at once.
/// @param state_added_callback_fn callback function called once a state is
/// added to the tree, providing a mutable reference to the tree and the index
/// of the newly-added state. This can be used, for example, to update a KD-tree
/// used for nearest neighbors. You can leave this default-constructed ({}) if
/// you do not need it.
/// @param states_connected_fn function to check if the two provided states, one
/// from each tree, are connected, and a solution has been found.
/// @param goal_bridge_callback_fn callback function called once a solution
/// "goal bridge", that is a pair of nodes, one from each tree, has been found.
/// It provides a mutable reference to the start tree, the goal bridge index in
/// the start tree, a mutable reference to the goal tree, the goal bridge
/// index in the goal tree, and a bool flag that specifies which tree is the
/// active tree. If this flag is true, start_tree is the active tree. You can
/// leave this default-constructed ({}) if you do not need it.
/// @param tree_sampling_bias probability that the next sample should be from
/// the target tree, rather than from calling @param state_sampling_fn.
/// @param p_switch_tree probability at each iteration that the active tree
/// should be swapped.
/// @param termination_check_fn Returns true if planning has been
/// terminated. The provided int64_t values are the current size of the start
/// and goal tree, respectively. These may be useful for a size-limited planning
/// problem.
/// @param rng a PRNG for use in internal sampling and tree swaps.
/// @return pair<paths, statistics> where paths is a vector of solution paths
/// and statistics is a map<string, double> of useful statistics collected while
/// planning.
template<typename RNG, typename StateType,
         typename Allocator=std::allocator<StateType>>
std::pair<std::vector<std::vector<StateType, Allocator>>,
          std::map<std::string, double>> BidirectionalRRTPlanMultiPath(
    std::vector<SimpleRRTPlannerState<StateType>>& start_tree,
    std::vector<SimpleRRTPlannerState<StateType>>& goal_tree,
    const std::function<StateType(void)>& state_sampling_fn,
    const std::function<int64_t(
        const std::vector<SimpleRRTPlannerState<StateType>>&,
        const StateType&)>& nearest_neighbor_fn,
    const std::function<std::vector<std::pair<StateType, int64_t>>(
        const StateType&, const StateType&)>& forward_propagation_fn,
    const std::function<void(
        std::vector<SimpleRRTPlannerState<StateType>>&,
        const int64_t)>& state_added_callback_fn,
    const std::function<bool(const StateType&,
                             const StateType&)>& states_connected_fn,
    const std::function<void(
        std::vector<SimpleRRTPlannerState<StateType>>&, const int64_t,
        std::vector<SimpleRRTPlannerState<StateType>>&, const int64_t,
        const bool)>& goal_bridge_callback_fn,
    const double tree_sampling_bias,
    const double p_switch_tree,
    const std::function<bool(const int64_t,
                             const int64_t)>& termination_check_fn,
    RNG& rng)
{
  if ((tree_sampling_bias < 0.0) || (tree_sampling_bias > 1.0))
  {
    throw std::invalid_argument(
        "tree_sampling_bias is not a valid probability");
  }
  if ((p_switch_tree < 0.0) || (p_switch_tree > 1.0))
  {
    throw std::invalid_argument(
        "p_switch_tree is not a valid probability");
  }
  if (start_tree.empty())
  {
    throw std::invalid_argument(
        "Must be called with at least one node in start tree");
  }
  if (goal_tree.empty())
  {
    throw std::invalid_argument(
        "Must be called with at least one node in goal tree");
  }
  // Keep track of the "goal bridges" between the trees
  std::vector<std::pair<int64_t, int64_t>> goal_bridges;
  // Keep track of the active treee
  bool start_tree_active = true;
  // Distribution to control sampling type
  std::uniform_real_distribution<double> unit_real_distribution(0.0, 1.0);
  // Keep track of statistics
  std::map<std::string, double> statistics;
  statistics["total_samples"] = 0.0;
  statistics["successful_samples"] = 0.0;
  statistics["failed_samples"] = 0.0;
  statistics["active_tree_swaps"] = 0.0;
  // Safety check before doing real work
  for (size_t start_tree_idx = 0; start_tree_idx < start_tree.size();
       start_tree_idx++)
  {
    for (size_t goal_tree_idx = 0; goal_tree_idx < goal_tree.size();
         goal_tree_idx++)
    {
      if (states_connected_fn(start_tree.at(start_tree_idx).GetValueImmutable(),
                              goal_tree.at(goal_tree_idx).GetValueImmutable()))
      {
        std::cerr << "Starting pair (" << start_tree_idx << ", "
                  << goal_tree_idx
                  << ") meets goal conditions, adding to goal states"
                  << std::endl;
        goal_bridges.push_back(
              std::pair<int64_t, int64_t>(
                static_cast<int64_t>(start_tree_idx),
                static_cast<int64_t>(goal_tree_idx)));
        goal_bridge_callback_fn(start_tree,
                                static_cast<int64_t>(start_tree_idx),
                                goal_tree,
                                static_cast<int64_t>(goal_tree_idx), true);
      }
    }
  }
  // Update the start time
  const std::chrono::time_point<std::chrono::steady_clock> start_time
      = std::chrono::steady_clock::now();
  // Plan
  while (!termination_check_fn(static_cast<int64_t>(start_tree.size()),
                               static_cast<int64_t>(goal_tree.size())))
  {
    // Get the current active/target trees
    std::vector<SimpleRRTPlannerState<StateType>>& active_tree
        = (start_tree_active) ? start_tree : goal_tree;
    std::vector<SimpleRRTPlannerState<StateType>>& target_tree
        = (start_tree_active) ? goal_tree : start_tree;
    // Select our sampling type
    const bool sample_from_tree
        = (unit_real_distribution(rng) <= tree_sampling_bias);
    int64_t target_tree_node_index = -1;
    if (sample_from_tree)
    {
      std::uniform_int_distribution<int64_t> tree_sampling_distribution(
          0, static_cast<int64_t>(target_tree.size() - 1));
      target_tree_node_index = tree_sampling_distribution(rng);
    }
    // Sample a target state
    const StateType target_state
        = (sample_from_tree)
          ? target_tree.at(target_tree_node_index).GetValueImmutable()
          : state_sampling_fn();
    // Get the nearest neighbor
    const int64_t nearest_neighbor_index
        = nearest_neighbor_fn(active_tree, target_state);
    // nearest_neighbor_index < 0 is handled as a sepecial case of early
    // termination.
    if (UNLIKELY(nearest_neighbor_index < 0))
    {
        break;
    }
    statistics["total_samples"] += 1.0;
    const StateType& nearest_neighbor
        = active_tree.at(nearest_neighbor_index).GetValueImmutable();
    // Forward propagate towards the goal
    const std::vector<std::pair<StateType, int64_t>> propagated
        = forward_propagation_fn(nearest_neighbor, target_state);
    if (!propagated.empty())
    {
      statistics["successful_samples"] += 1.0;
      for (size_t idx = 0; idx < propagated.size(); idx++)
      {
        const std::pair<StateType, int64_t>& current_propagation
            = propagated.at(idx);
        // Determine the parent index of the new state
        // This process deserves some explanation
        // The "current relative parent index" is the index of the parent,
        // relative to the list of propagated nodes. A negative value means the
        // nearest neighbor in the tree, zero means the first propagated node,
        // and so on. NOTE - the relative parent index *must* be lower than the
        // index in the list of prograted nodes i.e. the first node must have a
        // negative value, and so on.
        const int64_t& current_relative_parent_index
            = current_propagation.second;
        int64_t node_parent_index = nearest_neighbor_index;
        if (current_relative_parent_index >= 0)
        {
          const int64_t current_relative_index = static_cast<int64_t>(idx);
          if (current_relative_parent_index >= current_relative_index)
          {
            throw std::invalid_argument(
                  "Linkage with relative parent index >="
                  " current relative index is invalid");
          }
          const int64_t current_relative_offset
              = current_relative_parent_index - current_relative_index;
          const int64_t current_nodes_size
              = static_cast<int64_t>(active_tree.size());
          // Remember that current_relative_offset is negative!
          node_parent_index = current_nodes_size + current_relative_offset;
        }
        else
        {
          // Negative relative parent index means our parent index is the
          // nearest neighbor index.
          node_parent_index = nearest_neighbor_index;
        }
        // Build the new state
        const StateType& current_propagated = current_propagation.first;
        // Add the state to the tree
        active_tree.emplace_back(SimpleRRTPlannerState<StateType>(
                                     current_propagated, node_parent_index));
        const int64_t new_node_index
            = static_cast<int64_t>(active_tree.size() - 1);
        active_tree.at(node_parent_index).AddChildIndex(new_node_index);
        // Call the state added callback
        if (state_added_callback_fn)
        {
          state_added_callback_fn(active_tree, new_node_index);
        }
        // If we sampled from the other tree
        if (sample_from_tree)
        {
          // Check if we have connected the trees
          if (states_connected_fn(
                active_tree.at(new_node_index).GetValueImmutable(),
                target_state))
          {
            if (start_tree_active)
            {
              goal_bridges.push_back(
                  std::pair<int64_t, int64_t>(new_node_index,
                                              target_tree_node_index));
              goal_bridge_callback_fn(
                  active_tree, new_node_index, target_tree,
                  target_tree_node_index, start_tree_active);
            }
            else
            {
              goal_bridges.push_back(
                  std::pair<int64_t, int64_t>(target_tree_node_index,
                                              new_node_index));
              if (goal_bridge_callback_fn)
              {
                goal_bridge_callback_fn(
                    target_tree, target_tree_node_index, active_tree,
                    new_node_index, start_tree_active);
              }
            }
          }
        }
      }
    }
    else
    {
      statistics["failed_samples"] += 1.0;
    }
    // Decide if we should switch the active tree
    if (unit_real_distribution(rng) <= p_switch_tree)
    {
      start_tree_active = !start_tree_active;
      statistics["active_tree_swaps"] += 1.0;
    }
  }
  // Put together the results
  std::vector<std::vector<StateType, Allocator>> planned_paths;
  // Extract the solution paths
  for (const std::pair<int64_t, int64_t>& goal_bridge : goal_bridges)
  {
    // Extract the portion in the start tree
    std::vector<StateType, Allocator> start_path
        = ExtractSolutionPath<StateType, Allocator>(
            start_tree, goal_bridge.first);
    // Extract the portion in the goal tree
    std::vector<StateType, Allocator> goal_path
        = ExtractSolutionPath<StateType, Allocator>(
            goal_tree, goal_bridge.second);
    // Reverse the goal tree part
    std::reverse(goal_path.begin(), goal_path.end());
    // Combine
    start_path.insert(start_path.end(), goal_path.begin(), goal_path.end());
    planned_paths.push_back(start_path);
  }
  const std::chrono::time_point<std::chrono::steady_clock> cur_time
      = std::chrono::steady_clock::now();
  const std::chrono::duration<double> planning_time(cur_time - start_time);
  statistics["planning_time"] = planning_time.count();
  statistics["total_states"]
      = static_cast<double>(start_tree.size() + goal_tree.size());
  statistics["solutions"] = static_cast<double>(planned_paths.size());
  return std::make_pair(planned_paths, statistics);
}

/// Plan a single path using a single-direction RRT planner. Note that
/// StateType and SampleType are different; this makes implementation of some
/// kinodynamic planning problems easier.
/// @param tree existing tree. This tree must contain at least one node, but can
/// contain multiple nodes as well. All nodes in @param tree are assumed to be
/// valid starting nodes, and if they are linked together, they must have valid
/// parent-child linkage.
/// @param sampling_fn function to sample a new state.
/// @param nearest_neighbor_fn function to compute the nearest neighbor index
/// in the provided tree given the provided sample. If a negative index
/// (showing that no valid nearest neighbor could be found) planning is
/// terminated.
/// @param forward_propagation_fn function to forward propagate states from
/// the provided state to the provided sample. The propagated states are
/// returned as a vector<pair<state, relative_index>> in which state is the new
/// propagated state and relative_index is a relative index used to connect the
/// new state to the correct parent state. This relative_index is the index of
/// the parent state, relative to the vector of propagated nodes. A negative
/// value means the nearest neighbor in the tree, zero means the first
/// propagated node, and so on. NOTE - the relative parent index *must* be lower
/// than the index in the list of prograted nodes i.e. the first node must have
/// a negative value, and so on. While complicated, this structure allows @param
/// forward_propagation_fn to return an entire subtree at once.
/// @param state_added_callback_fn callback function called once a state is
/// added to the tree, providing a mutable reference to the tree and the index
/// of the newly-added state. This can be used, for example, to update a KD-tree
/// used for nearest neighbors.
/// @param check_goal_reached_fn function to check if the provided state meets
/// goal conditions.
/// @param goal_reached_callback_fn callback function called once a state meets
/// goal conditions, providing a mutable reference to the tree and the index of
/// the new goal state.
/// @param termination_check_fn Returns true if planning has been
/// terminated. The provided int64_t is the current size of the planner tree,
/// which may be useful for a size-limited planning problem.
/// @return pair<path, statistics> where path is the solution path and
/// statistics is a map<string, double> of useful statistics collected while
/// planning.
template<typename StateType,
         typename SampleType=StateType,
         typename StateAllocator=std::allocator<StateType>>
std::pair<std::vector<StateType, StateAllocator>,
          std::map<std::string, double>> RRTPlanSinglePath(
    std::vector<SimpleRRTPlannerState<StateType>>& tree,
    const std::function<SampleType(void)>& sampling_fn,
    const std::function<int64_t(
        const std::vector<SimpleRRTPlannerState<StateType>>&,
        const SampleType&)>& nearest_neighbor_fn,
    const std::function<std::vector<std::pair<StateType, int64_t>>(
        const StateType&, const SampleType&)>& forward_propagation_fn,
    const std::function<void(
        std::vector<SimpleRRTPlannerState<StateType>>&,
        const int64_t)>& state_added_callback_fn,
    const std::function<bool(const StateType&)>& check_goal_reached_fn,
    const std::function<void(
        std::vector<SimpleRRTPlannerState<StateType>>&,
        const int64_t)>& goal_reached_callback_fn,
    const std::function<bool(const int64_t)>& termination_check_fn)
{
  bool solution_found = false;
  const std::function<void(
      std::vector<SimpleRRTPlannerState<StateType>>&,
      const int64_t)> goal_reached_callback_function
      = [&] (std::vector<SimpleRRTPlannerState<StateType>>& planning_tree,
             const int64_t goal_node_index)
  {
    solution_found = true;
    if (goal_reached_callback_fn)
    {
      goal_reached_callback_fn(planning_tree, goal_node_index);
    }
  };
  const std::function<bool(const int64_t)> termination_check_function
      = [&] (const int64_t current_tree_size)
  {
    return (solution_found || termination_check_fn(current_tree_size));
  };
  const auto rrt_result = RRTPlanMultiPath(
      tree, sampling_fn, nearest_neighbor_fn, forward_propagation_fn,
      state_added_callback_fn, check_goal_reached_fn,
      goal_reached_callback_function, termination_check_function);
  if (rrt_result.first.size() > 0)
  {
    return std::make_pair(rrt_result.first.at(0), rrt_result.second);
  }
  else
  {
    return std::make_pair(
        std::vector<StateType, StateAllocator>(), rrt_result.second);
  }
}

/// Plan a single path using a bidirectional RRT planner.
/// @param start_tree existing starting tree. This tree must contain at least
/// one node, but can contain multiple nodes as well. All nodes in @param
/// start_tree are assumed to be valid starting nodes, and if they are linked
/// together, they must have valid parent-child linkage.
/// @param goal_tree existing goal tree. This tree must contain at least one
/// node, but can contain multiple nodes as well. All nodes in @param
/// goal_tree are assumed to be valid goal nodes, and if they are linked
/// together, they must have valid parent-child linkage.
/// @param state_sampling_fn function to sample a new state.
/// @param nearest_neighbor_fn function to compute the nearest neighbor index
/// in the provided tree given the provided sample. If a negative index
/// (showing that no valid nearest neighbor could be found) planning is
/// terminated.
/// @param forward_propagation_fn function to forward propagate states from
/// the provided state to the provided sample. The propagated states are
/// returned as a vector<pair<state, relative_index>> in which state is the new
/// propagated state and relative_index is a relative index used to connect the
/// new state to the correct parent state. This relative_index is the index of
/// the parent state, relative to the vector of propagated nodes. A negative
/// value means the nearest neighbor in the tree, zero means the first
/// propagated node, and so on. NOTE - the relative parent index *must* be lower
/// than the index in the list of prograted nodes i.e. the first node must have
/// a negative value, and so on. While complicated, this structure allows @param
/// forward_propagation_fn to return an entire subtree at once.
/// @param state_added_callback_fn callback function called once a state is
/// added to the tree, providing a mutable reference to the tree and the index
/// of the newly-added state. This can be used, for example, to update a KD-tree
/// used for nearest neighbors.
/// @param states_connected_fn function to check if the two provided states, one
/// from each tree, are connected, and a solution has been found.
/// @param goal_bridge_callback_fn callback function called once a solution
/// "goal bridge", that is a pair of nodes, one from each tree, has been found.
/// It provides a mutable reference to the start tree, the goal bridge index in
/// the start tree, a mutable reference to the goal tree, the goal bridge
/// index in the goal tree, and a bool flag that specifies which tree is the
/// active tree. If this flag is true, start_tree is the active tree.
/// @param tree_sampling_bias probability that the next sample should be from
/// the target tree, rather than from calling @param state_sampling_fn.
/// @param p_switch_tree probability at each iteration that the active tree
/// should be swapped.
/// @param termination_check_fn Returns true if planning has been
/// terminated. The provided int64_t values are the current size of the start
/// and goal tree, respectively. These may be useful for a size-limited planning
/// problem.
/// @param rng a PRNG for use in internal sampling and tree swaps.
/// @return pair<paths, statistics> where paths is a vector of solution paths
/// and statistics is a map<string, double> of useful statistics collected while
/// planning.
template<typename RNG, typename StateType,
         typename Allocator=std::allocator<StateType>>
std::pair<std::vector<StateType, Allocator>,
          std::map<std::string, double>> BidirectionalRRTPlanSinglePath(
    std::vector<SimpleRRTPlannerState<StateType>>& start_tree,
    std::vector<SimpleRRTPlannerState<StateType>>& goal_tree,
    const std::function<StateType(void)>& state_sampling_fn,
    const std::function<int64_t(
        const std::vector<SimpleRRTPlannerState<StateType>>&,
        const StateType&)>& nearest_neighbor_fn,
    const std::function<std::vector<std::pair<StateType, int64_t>>(
        const StateType&, const StateType&)>& forward_propagation_fn,
    const std::function<void(
        std::vector<SimpleRRTPlannerState<StateType>>&,
        const int64_t)>& state_added_callback_fn,
    const std::function<bool(const StateType&,
                             const StateType&)>& states_connected_fn,
    const std::function<void(
        std::vector<SimpleRRTPlannerState<StateType>>&, const int64_t,
        std::vector<SimpleRRTPlannerState<StateType>>&, const int64_t,
        const bool)>& goal_bridge_callback_fn,
    const double tree_sampling_bias,
    const double p_switch_tree,
    const std::function<bool(const int64_t,
                             const int64_t)>& termination_check_fn,
    RNG& rng)
{
  bool solution_found = false;
  const std::function<void(
        std::vector<SimpleRRTPlannerState<StateType>>&, const int64_t,
        std::vector<SimpleRRTPlannerState<StateType>>&, const int64_t,
        const bool)> goal_bridge_callback_function
      = [&] (std::vector<SimpleRRTPlannerState<StateType>>& planning_start_tree,
             const int64_t start_tree_index,
             std::vector<SimpleRRTPlannerState<StateType>>& planning_goal_tree,
             const int64_t goal_tree_index, const bool was_start_tree_active)
  {
    solution_found = true;
    if (goal_bridge_callback_fn)
    {
      goal_bridge_callback_fn(planning_start_tree, start_tree_index,
                              planning_goal_tree, goal_tree_index,
                              was_start_tree_active);
    }
  };
  const std::function<bool(const int64_t, const int64_t)>
      termination_check_function = [&] (const int64_t current_start_tree_size,
                                        const int64_t current_goal_tree_size)
  {
    return (solution_found || termination_check_fn(current_start_tree_size,
                                                   current_goal_tree_size));
  };
  const auto birrt_result = BidirectionalRRTPlanMultiPath(
      start_tree, goal_tree, state_sampling_fn, nearest_neighbor_fn,
      forward_propagation_fn, state_added_callback_fn, states_connected_fn,
      goal_bridge_callback_function, tree_sampling_bias, p_switch_tree,
      termination_check_function, rng);
  if (birrt_result.first.size() > 0)
  {
    return std::make_pair(birrt_result.first.at(0), birrt_result.second);
  }
  else
  {
    return std::make_pair(
        std::vector<StateType, Allocator>(), birrt_result.second);
  }
}

/// Helper function to build a termination check function for the
/// single-direction RRT planner that checks if the provided timeout has been
/// exceeded. The timeout function starts keeping track of elapsed time after
/// the first call, so you can create this function well before using it.
/// However, it can only be used once!
std::function<bool(const int64_t)> MakeRRTTimeoutTerminationFunction(
    const double planning_timeout)
{
  class TimeoutTerminationFunction
  {
  private:
    std::chrono::duration<double> timeout_;
    std::chrono::time_point<std::chrono::steady_clock> first_called_time_;
    bool already_called_ = false;

  public:
    TimeoutTerminationFunction(const double timeout)
    {
      if (timeout <= 0.0)
      {
        throw std::runtime_error("timeout <= 0.0");
      }
      timeout_ = std::chrono::duration<double>(timeout);
      already_called_ = false;
    }

    bool Check()
    {
      if (already_called_)
      {
        return ((std::chrono::steady_clock::now() - first_called_time_)
                > timeout_);
      }
      else
      {
        first_called_time_ = std::chrono::steady_clock::now();
        return false;
      }
    }
  };
  TimeoutTerminationFunction termination_fn_helper(planning_timeout);
  std::function<bool(const int64_t)> termination_function
      = [termination_fn_helper] (const int64_t) mutable
  {
    return termination_fn_helper.Check();
  };
  return termination_function;
}

/// Helper function to build a termination check function for the
/// bi-directional RRT planner that checks if the provided timeout has been
/// exceeded. The timeout function starts keeping track of elapsed time after
/// the first call, so you can create this function well before using it.
/// However, it can only be used once!
std::function<bool(const int64_t, const int64_t)>
MakeBiRRTTimeoutTerminationFunction(const double planning_timeout)
{
  class TimeoutTerminationFunction
  {
  private:
    std::chrono::duration<double> timeout_;
    std::chrono::time_point<std::chrono::steady_clock> first_called_time_;
    bool already_called_ = false;

  public:
    TimeoutTerminationFunction(const double timeout)
    {
      if (timeout <= 0.0)
      {
        throw std::runtime_error("timeout <= 0.0");
      }
      timeout_ = std::chrono::duration<double>(timeout);
      already_called_ = false;
    }

    bool Check()
    {
      if (already_called_)
      {
        return ((std::chrono::steady_clock::now() - first_called_time_)
                > timeout_);
      }
      else
      {
        first_called_time_ = std::chrono::steady_clock::now();
        return false;
      }
    }
  };
  TimeoutTerminationFunction termination_fn_helper(planning_timeout);
  std::function<bool(const int64_t, const int64_t)> termination_function
      = [termination_fn_helper] (const int64_t, const int64_t) mutable
  {
    return termination_fn_helper.Check();
  };
  return termination_function;
}

/// Helper function to create a sampling function that wraps a state sampling
/// function @param state_sampling_fn and goal state @param goal_state and
/// samples states and goal according to @param goal_bias. In the basic single-
/// direction RRT with a fixed goal state, you interleave sampling random states
/// (accomplished here by calling @param state_sampling_fn) and "sampling" the
/// known goal state (here, @param goal_state) with probablity @param goal_bias.
/// This helper function copies the provided @param state_sampling_fn, @param
/// goal_state, and @param goal_bias, but passes @param rng by reference. Thus,
/// the lifetime of @param rng must cover the entire lifetime of the
/// std::function this returns!
template<typename SampleType, typename PRNG>
std::function<SampleType(void)> MakeStateAndGoalSamplingFunction(
    const std::function<SampleType(void)>& state_sampling_fn,
    const SampleType& goal_state, const double goal_bias,
    PRNG& rng)
{
  class StateAndGoalSamplingFunction
  {
  private:
    SampleType goal_sample_;
    const double goal_bias_ = 0.0;
    std::uniform_real_distribution<double> unit_real_dist_;
    const std::function<SampleType(void)> state_sampling_fn_;

  public:
    StateAndGoalSamplingFunction(
          const SampleType& goal_sample, const double goal_bias,
          const std::function<SampleType(void)>& state_sampling_fn)
        : goal_sample_(goal_sample), goal_bias_(goal_bias),
          unit_real_dist_(0.0, 1.0), state_sampling_fn_(state_sampling_fn)
    {
      if ((goal_bias_ < 0.0) || (goal_bias_ > 1.0))
      {
        throw std::invalid_argument(
            "goal_bias_ is not a valid probability");
      }
    }

    SampleType Sample(PRNG& rng)
    {
      if (unit_real_dist_(rng) > goal_bias_)
      {
        return state_sampling_fn_();
      }
      else
      {
        return goal_sample_;
      }
    }
  };
  StateAndGoalSamplingFunction sampling_fn_helper(
      goal_state, goal_bias, state_sampling_fn);
  std::function<SampleType(void)> sampling_function
      = [sampling_fn_helper, &rng] (void) mutable
  {
    return sampling_fn_helper.Sample(rng);
  };
  return sampling_function;
}

/// Helper function to create a serial/parallel linear nearest neighbors
/// function for use in RRT and BiRRT planners given the provided state-to-state
/// distance function @param distance_fn and flag @param use_parallel which
/// selects if parallel linear nearest neighbors should be performed.
template<typename StateType>
std::function<int64_t(
    const std::vector<SimpleRRTPlannerState<StateType>>&,
    const StateType&)>
MakeLinearNearestNeighborsFunction(
    const std::function<double(const StateType&,
                               const StateType&)>& distance_fn,
    const bool use_parallel = true)
{
  std::function<int64_t(
      const std::vector<SimpleRRTPlannerState<StateType>>&,
      const StateType&)> nearest_neighbors_function
      = [=] (const std::vector<SimpleRRTPlannerState<StateType>>& tree,
             const StateType& sampled)
  {
    std::function<double(const SimpleRRTPlannerState<StateType>&,
                         const StateType&)>
        real_distance_fn =
            [&](const SimpleRRTPlannerState<StateType>& tree_state,
                const StateType& state) {
              const StateType& candidate_q = tree_state.GetValueImmutable();
              return distance_fn(candidate_q, state);
            };
    const std::vector<std::pair<int64_t, double>> neighbors =
        simple_knearest_neighbors::GetKNearestNeighbors(
            tree, sampled, real_distance_fn, 1, use_parallel);
    if (neighbors.size() > 0) {
      const std::pair<int64_t, double>& nearest_neighbor = neighbors.front();
      return nearest_neighbor.first;
    } else {
      throw std::runtime_error("NN check produced no neighbors");
    }
  };
  return nearest_neighbors_function;
}

/// Helper function to create an RRT-Extend forward propagation function for
/// holonomic kinematic planning problems. Given the provided distance function
/// @param distance_fn which computes state-to-state distance, @param
/// state_interpolation_fn which interpolates a state between the provided start
/// and end states and interpolation ratio, @param edge_validity_check peforms
/// collision checks on the edge between the provided start and end states and
/// returns true if the edge is valid. @param step_size is the maximum length
/// edge to consider at once; if the sampled state is farther away, the forward
/// propagation function will take one step (of @param step_size length) towards
/// it.
template<typename StateType>
std::function<std::vector<std::pair<StateType, int64_t>>(
    const StateType&, const StateType&)>
MakeKinematicRRTExtendPropagationFunction(
    const std::function<double(const StateType&,
                               const StateType&)>& distance_fn,
    const std::function<StateType(const StateType&,
                                  const StateType&,
                                  const double)>& state_interpolation_fn,
    const std::function<bool(const StateType&,
                             const StateType&)>& edge_validity_check_fn,
    const double step_size)
{
  if (step_size <= 0.0)
  {
    throw std::invalid_argument("step_size <= 0.0");
  }
  std::function<std::vector<std::pair<StateType, int64_t>>(
      const StateType&, const StateType&)> forward_propagation_fn
         = [=] (const StateType& nearest, const StateType& sampled)
  {
    const double distance = distance_fn(nearest, sampled);
    const double ratio = (distance > step_size) ? (step_size / distance) : 1.0;
    const StateType extend_state =
        (ratio < 1.0) ? state_interpolation_fn(nearest, sampled, ratio)
                      : sampled;
    if (edge_validity_check_fn(nearest, extend_state)) {
      // -1 is the parent offset used in adding the new node to the tree.
      return std::vector<std::pair<StateType, int64_t>>{
          std::make_pair(extend_state, -1)};
    } else {
      return std::vector<std::pair<StateType, int64_t>>();
    }
  };
  return forward_propagation_fn;
}

/// Helper function to create an RRT-Connect forward propagation function for
/// holonomic kinematic planning problems. Given the provided distance function
/// @param distance_fn which computes state-to-state distance, @param
/// state_interpolation_fn which interpolates a state between the provided start
/// and end states and interpolation ratio, @param edge_validity_check peforms
/// collision checks on the edge between the provided start and end states and
/// returns true if the edge is valid. @param step_size is the maximum length
/// edge to consider at once; if the sampled state is farther away, the forward
/// propagation function will take a sequence of steps (of @param step_size
/// length) towards it.
template<typename StateType>
std::function<std::vector<std::pair<StateType, int64_t>>(
    const StateType&, const StateType&)>
MakeKinematicRRTConnectPropagationFunction(
    const std::function<double(const StateType&,
                               const StateType&)>& distance_fn,
    const std::function<StateType(const StateType&,
                                  const StateType&,
                                  const double)>& state_interpolation_fn,
    const std::function<bool(const StateType&,
                             const StateType&)>& edge_validity_check_fn,
    const double step_size)
{
  if (step_size <= 0.0)
  {
    throw std::invalid_argument("step_size <= 0.0");
  }
  std::function<std::vector<std::pair<StateType, int64_t>>(
      const StateType&, const StateType&)> forward_propagation_fn
         = [=] (const StateType& nearest, const StateType& sampled)
  {
    std::vector<std::pair<StateType, int64_t>> propagated_states;
    int64_t parent_offset = -1;
    // Compute a maximum number of steps to take
    const double total_distance = distance_fn(nearest, sampled);
    const int32_t total_steps =
        static_cast<int32_t>(
            std::ceil(total_distance / step_size));
    StateType current = nearest;
    int32_t steps = 0;
    bool completed = false;
    while (!completed && (steps < total_steps)) {
      // Compute the next intermediate target state
      StateType current_target = sampled;
      const double target_distance = distance_fn(current, current_target);
      if (target_distance > step_size) {
        const double step_fraction = step_size / target_distance;
        const StateType interpolated_target =
            state_interpolation_fn(current, sampled, step_fraction);
        current_target = interpolated_target;
      } else if (std::abs(target_distance) <=
                 std::numeric_limits<double>::epsilon()) {
        // If we've reached the target state, stop
        completed = true;
        break;
      } else {
        // If we're less than step size away, this is our last step
        completed = true;
      }
      // If the current edge is valid, we keep going
      if (edge_validity_check_fn(current, current_target)) {
        propagated_states.emplace_back(
            std::make_pair(current_target, parent_offset));
        current = current_target;
        parent_offset++;
        steps++;
      } else {
        completed = true;
      }
    }
    return propagated_states;
  };
  return forward_propagation_fn;
}
}  // namespace simple_rrt_planner
}  // namespace common_robotics_utilities
