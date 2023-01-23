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
#include <common_robotics_utilities/maybe.hpp>
#include <common_robotics_utilities/serialization.hpp>
#include <common_robotics_utilities/simple_knearest_neighbors.hpp>
#include <common_robotics_utilities/utility.hpp>

namespace common_robotics_utilities
{
namespace simple_rrt_planner
{
/// Basic templated tree node for use in RRT planners. Node stores a templated
/// value, parent index, and child indices.
template<typename StateType>
class SimpleRRTPlannerState
{
private:
  StateType value_;
  std::vector<int64_t> child_indices_;
  int64_t parent_index_ = -1;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static uint64_t Serialize(
      const SimpleRRTPlannerState<StateType>& state,
      std::vector<uint8_t>& buffer,
      const serialization::Serializer<StateType>& value_serializer)
  {
    const uint64_t start_buffer_size = buffer.size();
    // Serialize the value
    value_serializer(state.GetValueImmutable(), buffer);
    // Serialize the child indices
    serialization::SerializeMemcpyableVectorLike<int64_t>(
        state.GetChildIndices(), buffer);
    // Serialize the parent index
    serialization::SerializeMemcpyable<int64_t>(state.GetParentIndex(), buffer);
    // Figure out how many bytes were written
    const uint64_t end_buffer_size = buffer.size();
    const uint64_t bytes_written = end_buffer_size - start_buffer_size;
    return bytes_written;
  }

  static serialization::Deserialized<SimpleRRTPlannerState<StateType>>
  Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset,
      const serialization::Deserializer<StateType>& value_deserializer)
  {
    uint64_t current_position = starting_offset;
    // Deserialize the value
    const auto value_deserialized
        = value_deserializer(buffer, current_position);
    current_position += value_deserialized.BytesRead();
    // Deserialize the child indices
    const auto child_indices_deserialized
        = serialization::DeserializeMemcpyableVectorLike<int64_t>(
            buffer, current_position);
    current_position += child_indices_deserialized.BytesRead();
    // Deserialize the parent index
    const auto parent_index_deserialized
        = serialization::DeserializeMemcpyable<int64_t>(
            buffer, current_position);
    current_position += parent_index_deserialized.BytesRead();
    // Figure out how many bytes were read
    const uint64_t bytes_read = current_position - starting_offset;
    return serialization::MakeDeserialized(
        SimpleRRTPlannerState<StateType>(
            value_deserialized.Value(),
            parent_index_deserialized.Value(),
            child_indices_deserialized.Value()),
        bytes_read);
  }

  SimpleRRTPlannerState(const StateType& value,
                        const int64_t parent_index,
                        const std::vector<int64_t>& child_indices)
      : value_(value), child_indices_(child_indices),
        parent_index_(parent_index) {}

  SimpleRRTPlannerState(const StateType& value, const int64_t parent_index)
      : value_(value), parent_index_(parent_index) {}

  explicit SimpleRRTPlannerState(const StateType& value)
      : value_(value), parent_index_(-1) {}

  const StateType& GetValueImmutable() const { return value_; }

  StateType& GetValueMutable() { return value_; }

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
      if (child_indices_.at(idx) == child_index)
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
      if (child_indices_.at(idx) != child_index)
      {
        new_child_indices.push_back(child_indices_.at(idx));
      }
    }
    child_indices_ = new_child_indices;
  }
};

template<typename StateType>
using SimpleRRTPlannerStateAllocator =
    Eigen::aligned_allocator<SimpleRRTPlannerState<StateType>>;

template<typename StateType>
using SimpleRRTPlannerStateVector =
    std::vector<SimpleRRTPlannerState<StateType>,
                SimpleRRTPlannerStateAllocator<StateType>>;

/// Basic templated tree type used in RRT planners.
template<typename StateType>
class SimpleRRTPlannerTree
{
private:
  SimpleRRTPlannerStateVector<StateType> nodes_;

public:
  using NodeType = SimpleRRTPlannerState<StateType>;

  static uint64_t Serialize(
      const SimpleRRTPlannerTree<StateType>& tree,
      std::vector<uint8_t>& buffer,
      const serialization::Serializer<StateType>& value_serializer)
  {
    const serialization::Serializer<SimpleRRTPlannerState<StateType>>
        element_serializer = [&] (
            const SimpleRRTPlannerState<StateType>& state,
            std::vector<uint8_t>& serialize_buffer)
    {
      return SimpleRRTPlannerState<StateType>::Serialize(
          state, serialize_buffer, value_serializer);
    };

    return serialization::SerializeVectorLike<SimpleRRTPlannerState<StateType>>(
        tree.GetNodesImmutable(), buffer, element_serializer);
  }

  static serialization::Deserialized<SimpleRRTPlannerTree<StateType>>
  Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset,
      const serialization::Deserializer<StateType>& value_deserializer)
  {
    const serialization::Deserializer<SimpleRRTPlannerState<StateType>>
        element_deserializer = [&] (
            const std::vector<uint8_t>& deserialize_buffer,
            const uint64_t element_starting_offset)
    {
      return SimpleRRTPlannerState<StateType>::Deserialize(
          deserialize_buffer, element_starting_offset, value_deserializer);
    };

    const serialization::Deserialized<SimpleRRTPlannerStateVector<StateType>>
        deserialized_nodes = serialization::DeserializeVectorLike
            <SimpleRRTPlannerState<StateType>,
             SimpleRRTPlannerStateVector<StateType>>(
                buffer, starting_offset, element_deserializer);

    const SimpleRRTPlannerTree<StateType> tree(deserialized_nodes.Value());

    return serialization::MakeDeserialized(
        tree, deserialized_nodes.BytesRead());
  }

  SimpleRRTPlannerTree() {}

  explicit SimpleRRTPlannerTree(const int64_t anticipated_size)
  {
    nodes_.reserve(static_cast<size_t>(anticipated_size));
  }

  explicit SimpleRRTPlannerTree(
      const SimpleRRTPlannerStateVector<StateType>& nodes)
  {
    if (!SetNodes(nodes))
    {
      throw std::invalid_argument("Provided nodes have invalid tree linkage");
    }
  }

  int64_t Size() const { return static_cast<int64_t>(nodes_.size()); }

  bool Empty() const { return nodes_.empty(); }

  void Clear() { nodes_.clear(); }

  const SimpleRRTPlannerState<StateType>& GetNodeImmutable(
      const int64_t index) const
  {
    return nodes_.at(static_cast<size_t>(index));
  }

  SimpleRRTPlannerState<StateType>& GetNodeMutable(const int64_t index)
  {
    return nodes_.at(static_cast<size_t>(index));
  }

  int64_t AddNode(const StateType& value)
  {
    nodes_.emplace_back(value);
    return static_cast<int64_t>(nodes_.size() - 1);
  }

  int64_t AddNodeAndConnect(const StateType& value, const int64_t parent_index)
  {
    nodes_.emplace_back(value, parent_index);
    const int64_t new_node_index = static_cast<int64_t>(nodes_.size() - 1);
    GetNodeMutable(parent_index).AddChildIndex(new_node_index);
    return new_node_index;
  }

  const SimpleRRTPlannerStateVector<StateType>& GetNodesImmutable() const
  {
    return nodes_;
  }

  SimpleRRTPlannerStateVector<StateType>& GetNodesMutable() { return nodes_; }

  bool SetNodes(const SimpleRRTPlannerStateVector<StateType>& nodes)
  {
    if (CheckNodeLinkage(nodes))
    {
      nodes_ = nodes;
      return true;
    }
    else
    {
      return false;
    }
  }

  /// Checks tree @param nodes to make sure the parent-child linkages are valid.
  /// This conservatively enforces that the tree does not conatin cycles by
  /// enforcing parent_index < state_index < child_indices. This is sufficient
  /// to ensure that the tree does not contain cycles, but it does not work for
  /// all trees generally. However, this is enough for all trees produced by the
  /// RRT planners. Note that this does *NOT* ensure that all nodes in
  /// @param nodes form a single connected tree, since it can be useful to store
  /// multiple trees in the same container.
  static bool CheckNodeLinkage(
      const SimpleRRTPlannerStateVector<StateType>& nodes)
  {
    // Step through each state in nodes.
    // Make sure that the linkage to the parent and child states are correct.
    for (size_t current_index = 0; current_index < nodes.size();
         current_index++)
    {
      // For every state, make sure all the parent<->child linkages are valid
      const auto& current_state = nodes.at(current_index);
      // Check the linkage to the parent state
      const int64_t parent_index = current_state.GetParentIndex();
      if (parent_index > static_cast<int64_t>(current_index))
      {
        return false;
      }
      if ((parent_index >= 0)
          && (parent_index < static_cast<int64_t>(nodes.size())))
      {
        if (parent_index != static_cast<int64_t>(current_index))
        {
          const auto& parent_state =
              nodes.at(static_cast<size_t>(parent_index));
          // Make sure the corresponding parent contains the current node in its
          // list of child indices.
          const std::vector<int64_t>& parent_child_indices
              = parent_state.GetChildIndices();
          auto index_found = std::find(parent_child_indices.begin(),
                                       parent_child_indices.end(),
                                       static_cast<int64_t>(current_index));
          if (index_found == parent_child_indices.end())
          {
            return false;
          }
        }
        else
        {
          return false;
        }
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
            const auto& child_state =
                nodes.at(static_cast<size_t>(current_child_index));
            // Make sure the child node points to us as the parent index
            const int64_t child_parent_index = child_state.GetParentIndex();
            if (child_parent_index != static_cast<int64_t>(current_index))
            {
              return false;
            }
          }
          else
          {
            return false;
          }
        }
        else
        {
          return false;
        }
      }
    }
    return true;
  }
};

/// Helper function and type definitions

/// State sampling function signature.
template<typename SampleType>
using SamplingFunction = std::function<SampleType(void)>;

/// Select the nearest neighbor state in the tree, returning the index of the
/// nearest neighbor state.
template<typename StateType, typename TreeType=SimpleRRTPlannerTree<StateType>,
         typename SampleType=StateType>
using RRTNearestNeighborFunction =
    std::function<int64_t(
        const TreeType&,  // Planner tree
        const SampleType&)>;  // Sampled state

template<typename StateType>
class PropagatedState
{
private:
  StateType state_;
  int64_t relative_parent_index_ = -1;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PropagatedState(const StateType& state, const int64_t relative_parent_index)
      : state_(state), relative_parent_index_(relative_parent_index) {}

  PropagatedState(StateType&& state, const int64_t relative_parent_index)
      : state_(std::move(state)), relative_parent_index_(relative_parent_index)
  {}

  const StateType& State() const { return state_; }

  StateType& MutableState() { return state_; }

  int64_t RelativeParentIndex() const { return relative_parent_index_; }

  void SetRelativeParentIndex(const int64_t relative_parent_index)
  {
    relative_parent_index_ = relative_parent_index;
  }
};

template<typename StateType>
using PropagatedStateAllocator =
    Eigen::aligned_allocator<PropagatedState<StateType>>;

template<typename StateType>
using ForwardPropagation = std::vector<PropagatedState<StateType>,
                                       PropagatedStateAllocator<StateType>>;

/// Perform forward propagation from the nearest state towards the target state.
template<typename StateType, typename SampleType=StateType>
using RRTForwardPropagationFunction =
    std::function<ForwardPropagation<StateType>(
        const StateType&,  // Nearest state
        const SampleType&)>;  // Sampled state

/// Callback function called after state added in RRT planner.
template<typename StateType, typename TreeType=SimpleRRTPlannerTree<StateType>>
using RRTStateAddedCallbackFunction =
    std::function<void(
        TreeType&,  // Planner tree
        const int64_t)>;  // Index of added state in planner tree

/// Check if the provided state satisfies goal conditions.
template<typename StateType>
using RRTCheckGoalReachedFunction = std::function<bool(const StateType&)>;

/// Callback function called after goal in reached in RRT planner.
template<typename StateType, typename TreeType=SimpleRRTPlannerTree<StateType>>
using RRTGoalReachedCallbackFunction =
    std::function<void(
        TreeType&,  // Planner tree
        const int64_t)>;  // Index of goal state in planner tree

/// Termination check function for RRT planning.
using RRTTerminationCheckFunction =
    std::function<bool(const int64_t)>;  // Size of planner tree

/// Typedef for multiple paths.
template<typename StateType, typename Container=std::vector<StateType>>
using PlannedPaths = std::vector<Container>;

/// Typedef for planning statistics, of the form map<name, value>.
using PlanningStatistics = std::map<std::string, double>;

/// Container for the results of multi-solution planning.
template<typename StateType, typename Container=std::vector<StateType>>
class MultipleSolutionPlanningResults
{
private:
  PlannedPaths<StateType, Container> paths_;
  PlanningStatistics statistics_;

public:
  MultipleSolutionPlanningResults() {}

  explicit MultipleSolutionPlanningResults(
      const PlanningStatistics& statistics) : statistics_(statistics) {}

  MultipleSolutionPlanningResults(
      const PlannedPaths<StateType, Container>& paths,
      const PlanningStatistics& statistics)
      : paths_(paths), statistics_(statistics) {}

  MultipleSolutionPlanningResults(
      PlannedPaths<StateType, Container>&& paths,
      const PlanningStatistics& statistics)
      : paths_(std::move(paths)), statistics_(statistics) {}

  const PlannedPaths<StateType, Container>& Paths() const { return paths_; }

  const PlanningStatistics& Statistics() const { return statistics_; }
};

/// Container for the results of single-solution planning.
template<typename StateType, typename Container=std::vector<StateType>>
class SingleSolutionPlanningResults
{
private:
  Container path_;
  PlanningStatistics statistics_;

public:
  SingleSolutionPlanningResults() {}

  explicit SingleSolutionPlanningResults(
      const PlanningStatistics& statistics) : statistics_(statistics) {}

  SingleSolutionPlanningResults(
      const Container& path, const PlanningStatistics& statistics)
      : path_(path), statistics_(statistics) {}

  SingleSolutionPlanningResults(
      Container&& path, const PlanningStatistics& statistics)
      : path_(std::move(path)), statistics_(statistics) {}

  const Container& Path() const { return path_; }

  const PlanningStatistics& Statistics() const { return statistics_; }
};

/// Distinguish between sampling from the target tree or state sampling via the
/// state sampling function.
enum class BiRRTSampleType : uint8_t {TREE_SAMPLE, STATE_SAMPLE};

/// Identify which tree (i.e. start or goal tree) is the current "active" tree.
enum class BiRRTActiveTreeType : uint8_t {START_TREE, GOAL_TREE};

/// Select what type of sample should be sampled.
template<typename StateType, typename TreeType=SimpleRRTPlannerTree<StateType>>
using BiRRTSelectSampleTypeFunction =
    std::function<BiRRTSampleType(
        const TreeType&,  // Start tree
        const TreeType&,  // Goal tree
        const BiRRTActiveTreeType)>;  // Which tree is the active tree?

/// Sample a state from the provided target tree, returning the index of the
/// sampled state.
template<typename StateType, typename TreeType=SimpleRRTPlannerTree<StateType>>
using BiRRTTreeSamplingFunction =
    std::function<int64_t(
        const TreeType&,  // Target tree
        const BiRRTActiveTreeType)>;  // Which tree is the active tree?

/// Select which tree should be the new active tree.
template<typename StateType, typename TreeType=SimpleRRTPlannerTree<StateType>>
using BiRRTSelectActiveTreeFunction =
    std::function<BiRRTActiveTreeType(
        const TreeType&,  // Start tree
        const TreeType&,  // Goal tree
        const BiRRTActiveTreeType)>;  // Which tree is the active tree?

/// Select the nearest neighbor state in the active tree, returning the index of
/// the nearest neighbor state.
template<typename StateType, typename TreeType=SimpleRRTPlannerTree<StateType>>
using BiRRTNearestNeighborFunction =
    std::function<int64_t(
        const TreeType&,  // Active tree
        const StateType&,  // Sampled state
        const BiRRTActiveTreeType)>;  // Which tree is the active tree?

/// Perform forward propagation from the nearest state towards the target state.
template<typename StateType>
using BiRRTPropagationFunction =
    std::function<ForwardPropagation<StateType>(
        const StateType&,  // Nearest state (from the active tree)
        const StateType&,  // Target state
        const BiRRTActiveTreeType)>;  // Which tree is the active tree?

/// Callback function called after a new state is added by the BiRRT planner.
template<typename StateType, typename TreeType=SimpleRRTPlannerTree<StateType>>
using BiRRTStateAddedCallbackFunction =
    std::function<void(
        TreeType&,  // Active tree
        const int64_t, // Index of added state in active tree
        const BiRRTActiveTreeType)>;  // Which tree is the active tree?

/// Check if the provided source and target states are connected.
template<typename StateType>
using BiRRTStatesConnectedFunction =
    std::function<bool(
        const StateType&,  // Source state (from the active tree)
        const StateType&,  // Target state
        const BiRRTActiveTreeType)>;  // Which tree is the active tree?

/// Callback function called after a "goal bridge" connecting start and goal
/// trees is found.
template<typename StateType, typename TreeType=SimpleRRTPlannerTree<StateType>>
using BiRRTGoalBridgeCallbackFunction =
    std::function<void(
        TreeType&, const int64_t,  // Start tree + index
        TreeType&, const int64_t,  // Goal tree + index
        const BiRRTActiveTreeType)>;  // Which tree is the active tree?

/// Termination check function for BiRRT planning.
using BiRRTTerminationCheckFunction =
    std::function<bool(
        const int64_t,  // Size of start tree
        const int64_t)>;  // Size of goal tree

/// Extracts a single solution path corresponding to the provided goal state.
/// @param tree tree produced by RRT planner.
/// @param goal_state_index index of goal state in @param tree.
template<typename StateType, typename TreeType=SimpleRRTPlannerTree<StateType>,
         typename Container=std::vector<StateType>>
Container ExtractSolutionPath(
    const TreeType& tree, const int64_t goal_state_index)
{
  Container solution_path;
  const auto& goal_state = tree.GetNodeImmutable(goal_state_index);
  solution_path.push_back(goal_state.GetValueImmutable());
  int64_t parent_index = goal_state.GetParentIndex();
  while (parent_index >= 0)
  {
    const auto& parent_state = tree.GetNodeImmutable(parent_index);
    const StateType& parent = parent_state.GetValueImmutable();
    solution_path.push_back(parent);
    parent_index = parent_state.GetParentIndex();
  }
  // Put it in the right order
  std::reverse(solution_path.begin(), solution_path.end());
  return solution_path;
}

/// Extracts the solution paths corresponding to each of the provided goal
/// states.
/// @param tree tree produced by RRT planner.
/// @param goal_state_indices indices of goal states in @param tree.
template<typename StateType, typename TreeType=SimpleRRTPlannerTree<StateType>,
         typename Container=std::vector<StateType>>
PlannedPaths<StateType, Container> ExtractSolutionPaths(
    const TreeType& tree, const std::vector<int64_t>& goal_state_indices)
{
  PlannedPaths<StateType, Container> solution_paths(
      goal_state_indices.size());
  for (size_t idx = 0; idx < goal_state_indices.size(); idx++)
  {
    solution_paths.at(idx) =
        ExtractSolutionPath<StateType, TreeType, Container>(
            tree, goal_state_indices.at(idx));
  }
  return solution_paths;
}

/// Plan multiple paths using a single-direction RRT planner. This planner will
/// continue to explore and add new solution paths until terminated. Note that
/// StateType and SampleType are different; this makes implementation of some
/// kinodynamic planning problems easier.
///
/// @param tree existing tree. This tree must contain at least one node, but can
///     contain multiple nodes as well. All nodes in @param tree are assumed to
///     be valid starting nodes, and if they are linked together, they must have
///     valid parent-child linkage.
/// @param sampling_fn function to sample a new state.
/// @param nearest_neighbor_fn function to compute the nearest neighbor index
///     in the provided tree given the provided sample. If a negative index
///     (showing that no valid nearest neighbor could be found) planning is
///     terminated.
/// @param forward_propagation_fn function to forward propagate states from
///     the provided state to the provided sample. The propagated states are
///     returned as a vector<pair<state, relative_index>> in which state is the
///     new propagated state and relative_index is a relative index used to
///     connect the new state to the correct parent state. This relative_index
///     is the index of the parent state, relative to the vector of propagated
///     nodes. A negative value means the nearest neighbor in the tree, zero
///     means the first propagated node, and so on. NOTE - the relative parent
///     index *must* be lower than the index in the list of propagated nodes
///     i.e. the first node must have a negative value, and so on. While
///     complicated, this structure allows @param forward_propagation_fn to
///     return an entire subtree at once, which is used for certain
///     multi-outcome planners.
/// @param state_added_callback_fn callback function called once a state is
///     added to the tree, providing a mutable reference to the tree and the
///     index of the newly-added state. This can be used, for example, to update
///     a KD-tree used for nearest neighbors. You can leave this
///     default-constructed ({}) if you do not need it.
/// @param check_goal_reached_fn function to check if the provided state meets
///     goal conditions.
/// @param goal_reached_callback_fn callback function called once a state meets
///     goal conditions, providing a mutable reference to the tree and the index
///     of the new goal state. You can leave this default-constructed ({}) if
///     you do not need it.
/// @param termination_check_fn Returns true if planning has been
///     terminated. The provided int64_t is the current size of the planner
///     tree, which may be useful for a size-limited planning problem.
/// @return paths + statistics where paths is the vector of solution paths
///     and statistics is a map<string, double> of useful statistics collected
///     while planning.
template<typename StateType,
         typename TreeType=SimpleRRTPlannerTree<StateType>,
         typename SampleType=StateType,
         typename Container=std::vector<StateType>>
MultipleSolutionPlanningResults<StateType, Container> RRTPlanMultiPath(
    TreeType& tree,
    const SamplingFunction<SampleType>& sampling_fn,
    const RRTNearestNeighborFunction<StateType, TreeType, SampleType>&
        nearest_neighbor_fn,
    const RRTForwardPropagationFunction<StateType, SampleType>&
        forward_propagation_fn,
    const RRTStateAddedCallbackFunction<StateType, TreeType>&
        state_added_callback_fn,
    const RRTCheckGoalReachedFunction<StateType>& check_goal_reached_fn,
    const RRTGoalReachedCallbackFunction<StateType, TreeType>&
        goal_reached_callback_fn,
    const RRTTerminationCheckFunction& termination_check_fn)
{
  // Make sure we've been given a start state
  if (tree.Empty())
  {
    throw std::invalid_argument(
        "Must be called with at least one node in tree");
  }

  // Keep track of statistics
  PlanningStatistics statistics;
  statistics["total_samples"] = 0.0;
  statistics["successful_samples"] = 0.0;
  statistics["failed_samples"] = 0.0;

  // Storage for the goal states we reach
  std::vector<int64_t> goal_state_indices;

  // Safety check before doing real work
  for (int64_t idx = 0; idx < tree.Size(); idx++)
  {
    if (check_goal_reached_fn(tree.GetNodeImmutable(idx).GetValueImmutable()))
    {
      goal_state_indices.push_back(idx);
      goal_reached_callback_fn(tree, idx);
    }
  }

  // Get the start time
  const std::chrono::time_point<std::chrono::steady_clock> start_time
      = std::chrono::steady_clock::now();

  // Plan
  while (!termination_check_fn(tree.Size()))
  {
    // Sample a random goal
    const SampleType random_target = sampling_fn();
    statistics["total_samples"] += 1.0;

    // Get the nearest neighbor
    const int64_t nearest_neighbor_index =
        nearest_neighbor_fn(tree, random_target);
    const StateType& nearest_neighbor =
        tree.GetNodeImmutable(nearest_neighbor_index).GetValueImmutable();

    // Forward propagate towards the goal
    const ForwardPropagation<StateType> propagated
        = forward_propagation_fn(nearest_neighbor, random_target);
    if (!propagated.empty())
    {
      statistics["successful_samples"] += 1.0;

      // Keep track of the mapping from propagated node to tree index.
      // We rely on the bounds check of propagated_index_to_tree_index.at() to
      // enforce the invariant that relative parent indices refer to prior
      // nodes, thus this cannot use resize() or equivalent.
      std::vector<int64_t> propagated_index_to_tree_index;
      propagated_index_to_tree_index.reserve(propagated.size());

      // Go through propagated states
      for (size_t idx = 0; idx < propagated.size(); idx++)
      {
        const PropagatedState<StateType>& current_propagation
            = propagated.at(idx);
        // Determine the parent index of the new state
        // This process deserves some explanation
        // The "current relative parent index" is the index of the parent,
        // relative to the list of propagated nodes. A negative value means the
        // nearest neighbor in the tree, zero means the first propagated node,
        // and so on. NOTE - the relative parent index *must* be lower than the
        // index in the list of propagated nodes i.e. the first node must have a
        // negative value, and so on.
        const int64_t& current_relative_parent_index
            = current_propagation.RelativeParentIndex();

        // Note: all negative indices correspond to the same nearest neighbor
        // node; further indexing back into the tree is not supported.
        const int64_t node_parent_index =
            (current_relative_parent_index >= 0)
                ? propagated_index_to_tree_index.at(
                    static_cast<size_t>(current_relative_parent_index))
                : nearest_neighbor_index;

        // Build the new state
        const StateType& current_propagated = current_propagation.State();

        // Add the state to the tree
        const int64_t new_node_index =
            tree.AddNodeAndConnect(current_propagated, node_parent_index);

        // Add the new index to the mapping
        propagated_index_to_tree_index.emplace_back(new_node_index);

        // Call the state added callback
        if (state_added_callback_fn)
        {
          state_added_callback_fn(tree, new_node_index);
        }

        // Check if we've reached the goal
        if (check_goal_reached_fn(
                tree.GetNodeImmutable(new_node_index).GetValueImmutable()))
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
  PlannedPaths<StateType, Container> planned_paths
      = ExtractSolutionPaths<StateType, TreeType, Container>(
          tree, goal_state_indices);

  // Collect final statistics
  const std::chrono::time_point<std::chrono::steady_clock> cur_time
      = std::chrono::steady_clock::now();
  const std::chrono::duration<double> planning_time(cur_time - start_time);
  statistics["planning_time"] = planning_time.count();
  statistics["total_states"] = static_cast<double>(tree.Size());
  statistics["solutions"] = static_cast<double>(planned_paths.size());
  return MultipleSolutionPlanningResults<StateType, Container>(
      std::move(planned_paths), statistics);
}

/// Plan multiple paths using a bidirectional RRT planner. This planner will
/// continue to explore and add new solution paths until terminated.
///
/// @param start_tree existing starting tree. This tree must contain at least
///     one node, but can contain multiple nodes as well. All nodes in @param
///     start_tree are assumed to be valid starting nodes, and if they are
///     linked together, they must have valid parent-child linkage.
/// @param goal_tree existing goal tree. This tree must contain at least one
///     node, but can contain multiple nodes as well. All nodes in @param
///     goal_tree are assumed to be valid goal nodes, and if they are linked
///     together, they must have valid parent-child linkage.
/// @param select_sample_type_fn function to select sample type (i.e. between
///     tree sample or state sample.
/// @param state_sampling_fn function to sample a new state.
/// @param tree_sampling_fn function to sample a state from the target tree.
/// @param nearest_neighbor_fn function to compute the nearest neighbor index
///     in the provided tree given the provided sample. If a negative index
///     (showing that no valid nearest neighbor could be found) planning is
///     terminated.
/// @param propagation_fn function to forward propagate states from the provided
///     state to the provided sample. The propagated states are returned as a
///     vector<pair<state, relative_index>> in which state is the new propagated
///     state and relative_index is a relative index used to connect the new
///     state to the correct parent state. This relative_index is the index of
///     the parent state, relative to the vector of propagated nodes. A negative
///     value means the nearest neighbor in the tree, zero means the first
///     propagated node, and so on. NOTE - the relative parent index *must* be
///     lower than the index in the list of propagated nodes i.e. the first node
///     must have a negative value, and so on. While complicated, this structure
///     allows @param forward_propagation_fn to return an entire subtree at
///     once, which is used for certain multi-outcome planners.
/// @param state_added_callback_fn callback function called once a state is
///     added to the tree, providing a mutable reference to the tree and the
///     index of the newly-added state. This can be used, for example, to update
///     a KD-tree used for nearest neighbors. You can leave this
///     default-constructed ({}) if you do not need it.
/// @param states_connected_fn function to check if the two provided states, one
///     from each tree, are connected, and a solution has been found.
/// @param goal_bridge_callback_fn callback function called once a solution
///     "goal bridge", that is a pair of nodes, one from each tree, has been
///     found. It provides a mutable reference to the start tree, the goal
///     bridge index in the start tree, a mutable reference to the goal tree,
///     the goal bridge index in the goal tree, and a bool flag that specifies
///     which tree is the current active tree. If this flag is true, start_tree
///     is the active tree. You can leave this default-constructed ({}) if you
///     do not need it.
/// @param select_active_tree_fn function to select the active tree for the next
///     planning iteration.
/// @param termination_check_fn Returns true if planning has been
///     terminated. The provided int64_t values are the current size of the
///     start and goal tree, respectively. These may be useful for a
///     size-limited planning problem.
/// @return paths + statistics where paths is the vector of solution paths
///     and statistics is a map<string, double> of useful statistics collected
///     while planning.
template<typename StateType,
         typename TreeType=SimpleRRTPlannerTree<StateType>,
         typename Container=std::vector<StateType>>
MultipleSolutionPlanningResults<StateType, Container> BiRRTPlanMultiPath(
    TreeType& start_tree,
    TreeType& goal_tree,
    const BiRRTSelectSampleTypeFunction<StateType, TreeType>&
        select_sample_type_fn,
    const SamplingFunction<StateType>& state_sampling_fn,
    const BiRRTTreeSamplingFunction<StateType, TreeType>& tree_sampling_fn,
    const BiRRTNearestNeighborFunction<StateType, TreeType>&
        nearest_neighbor_fn,
    const BiRRTPropagationFunction<StateType>& propagation_fn,
    const BiRRTStateAddedCallbackFunction<StateType, TreeType>&
        state_added_callback_fn,
    const BiRRTStatesConnectedFunction<StateType>& states_connected_fn,
    const BiRRTGoalBridgeCallbackFunction<StateType, TreeType>&
        goal_bridge_callback_fn,
    const BiRRTSelectActiveTreeFunction<StateType, TreeType>&
        select_active_tree_fn,
    const BiRRTTerminationCheckFunction& termination_check_fn)
{
  if (start_tree.Empty())
  {
    throw std::invalid_argument(
        "Must be called with at least one node in start tree");
  }
  if (goal_tree.Empty())
  {
    throw std::invalid_argument(
        "Must be called with at least one node in goal tree");
  }

  // Keep track of the "goal bridges" between the trees
  class GoalBridge
  {
  private:
    int64_t start_tree_index_ = -1;
    int64_t goal_tree_index_ = -1;

  public:
    GoalBridge(const int64_t start_tree_index, const int64_t goal_tree_index)
        : start_tree_index_(start_tree_index), goal_tree_index_(goal_tree_index)
    {
      if (start_tree_index_ < 0)
      {
        throw std::invalid_argument("start_tree_index_ < 0");
      }
      if (goal_tree_index_ < 0)
      {
        throw std::invalid_argument("goal_tree_index_ < 0");
      }
    }

    GoalBridge(const size_t start_tree_index, const size_t goal_tree_index)
        : start_tree_index_(static_cast<int64_t>(start_tree_index)),
          goal_tree_index_(static_cast<int64_t>(goal_tree_index)) {}

    int64_t StartTreeIndex() const { return start_tree_index_; }

    int64_t GoalTreeIndex() const { return goal_tree_index_; }
  };
  std::vector<GoalBridge> goal_bridges;

  // Keep track of the active tree
  BiRRTActiveTreeType active_tree_type = BiRRTActiveTreeType::START_TREE;

  // Keep track of statistics
  PlanningStatistics statistics;
  statistics["start_tree_active_iterations"] = 0.0;
  statistics["goal_tree_active_iterations"] = 0.0;
  statistics["total_samples"] = 0.0;
  statistics["successful_samples"] = 0.0;
  statistics["failed_samples"] = 0.0;
  statistics["active_tree_swaps"] = 0.0;

  // Safety check before doing real work
  for (int64_t start_tree_idx = 0; start_tree_idx < start_tree.Size();
       start_tree_idx++)
  {
    for (int64_t goal_tree_idx = 0; goal_tree_idx < goal_tree.Size();
         goal_tree_idx++)
    {
      if (states_connected_fn(
              start_tree.GetNodeImmutable(start_tree_idx).GetValueImmutable(),
              goal_tree.GetNodeImmutable(goal_tree_idx).GetValueImmutable(),
              active_tree_type))
      {
        goal_bridges.emplace_back(start_tree_idx, goal_tree_idx);
        goal_bridge_callback_fn(
            start_tree, start_tree_idx,
            goal_tree, goal_tree_idx,
            active_tree_type);
      }
    }
  }

  // Get the start time
  const std::chrono::time_point<std::chrono::steady_clock> start_time
      = std::chrono::steady_clock::now();

  // Plan
  while (!termination_check_fn(start_tree.Size(), goal_tree.Size()))
  {
    // Get the current active/target trees
    TreeType& active_tree =
        (active_tree_type == BiRRTActiveTreeType::START_TREE)
            ? start_tree : goal_tree;
    TreeType& target_tree =
        (active_tree_type == BiRRTActiveTreeType::START_TREE)
            ? goal_tree : start_tree;

    if (active_tree_type == BiRRTActiveTreeType::START_TREE)
    {
      statistics["start_tree_active_iterations"] += 1.0;
    }
    else
    {
      statistics["goal_tree_active_iterations"] += 1.0;
    }

    // Select our sampling type
    const BiRRTSampleType sample_type =
        select_sample_type_fn(start_tree, goal_tree, active_tree_type);

    // Sample a target state
    const int64_t target_tree_node_index =
        (sample_type == BiRRTSampleType::TREE_SAMPLE)
            ? tree_sampling_fn(target_tree, active_tree_type)
            : -1;
    const StateType& target_state =
        (sample_type == BiRRTSampleType::TREE_SAMPLE)
            ? target_tree.GetNodeImmutable(
                target_tree_node_index).GetValueImmutable()
            : state_sampling_fn();
    statistics["total_samples"] += 1.0;

    // Get the nearest neighbor
    const int64_t nearest_neighbor_index =
        nearest_neighbor_fn(active_tree, target_state, active_tree_type);
    const StateType& nearest_neighbor = active_tree.GetNodeImmutable(
        nearest_neighbor_index).GetValueImmutable();

    // Forward propagate towards the goal
    const ForwardPropagation<StateType> propagated =
        propagation_fn(nearest_neighbor, target_state, active_tree_type);
    if (!propagated.empty())
    {
      statistics["successful_samples"] += 1.0;

      // Keep track of the mapping from propagated node to tree index.
      // We rely on the bounds check of propagated_index_to_tree_index.at() to
      // enforce the invariant that relative parent indices refer to prior
      // nodes, thus this cannot use resize() or equivalent.
      std::vector<int64_t> propagated_index_to_tree_index;
      propagated_index_to_tree_index.reserve(propagated.size());

      // Go through propagated states
      for (size_t idx = 0; idx < propagated.size(); idx++)
      {
        const PropagatedState<StateType>& current_propagation
            = propagated.at(idx);
        // Determine the parent index of the new state
        // This process deserves some explanation
        // The "current relative parent index" is the index of the parent,
        // relative to the list of propagated nodes. A negative value means the
        // nearest neighbor in the tree, zero means the first propagated node,
        // and so on. NOTE - the relative parent index *must* be lower than the
        // index in the list of propagated nodes i.e. the first node must have a
        // negative value, and so on.
        const int64_t& current_relative_parent_index
            = current_propagation.RelativeParentIndex();

        // Note: all negative indices correspond to the same nearest neighbor
        // node; further indexing back into the tree is not supported.
        const int64_t node_parent_index =
            (current_relative_parent_index >= 0)
                ? propagated_index_to_tree_index.at(
                    static_cast<size_t>(current_relative_parent_index))
                : nearest_neighbor_index;

        // Build the new state
        const StateType& current_propagated = current_propagation.State();

        // Add the state to the tree
        const int64_t new_node_index = active_tree.AddNodeAndConnect(
            current_propagated, node_parent_index);

        // Add the new index to the mapping
        propagated_index_to_tree_index.emplace_back(new_node_index);

        // Call the state added callback
        if (state_added_callback_fn)
        {
          state_added_callback_fn(
              active_tree, new_node_index, active_tree_type);
        }

        // If we sampled from the other tree
        if (sample_type == BiRRTSampleType::TREE_SAMPLE)
        {
          // Check if we have connected the trees
          if (states_connected_fn(
                  active_tree.GetNodeImmutable(
                      new_node_index).GetValueImmutable(),
                  target_state, active_tree_type))
          {
            // Add the goal bridge
            if (active_tree_type == BiRRTActiveTreeType::START_TREE)
            {
              goal_bridges.emplace_back(new_node_index, target_tree_node_index);
            }
            else
            {
              goal_bridges.emplace_back(target_tree_node_index, new_node_index);
            }

            // Call the goal bridge callback (if provided)
            if (goal_bridge_callback_fn)
            {
              const GoalBridge& latest_goal_bridge = goal_bridges.back();
              goal_bridge_callback_fn(
                  start_tree, latest_goal_bridge.StartTreeIndex(),
                  goal_tree, latest_goal_bridge.GoalTreeIndex(),
                  active_tree_type);
            }
          }
        }
      }
    }
    else
    {
      statistics["failed_samples"] += 1.0;
    }

    // Select the new active tree
    const BiRRTActiveTreeType new_active_tree_type =
        select_active_tree_fn(start_tree, goal_tree, active_tree_type);
    if (new_active_tree_type != active_tree_type)
    {
      statistics["active_tree_swaps"] += 1.0;
    }
    active_tree_type = new_active_tree_type;
  }

  // Put together the results
  PlannedPaths<StateType, Container> planned_paths;

  // Extract the solution paths
  for (const GoalBridge& goal_bridge : goal_bridges)
  {
    // Extract the portion in the start tree
    Container start_path = ExtractSolutionPath<StateType, TreeType, Container>(
        start_tree, goal_bridge.StartTreeIndex());
    // Extract the portion in the goal tree
    Container goal_path = ExtractSolutionPath<StateType, TreeType, Container>(
        goal_tree, goal_bridge.GoalTreeIndex());
    // Reverse the goal tree part
    std::reverse(goal_path.begin(), goal_path.end());
    // Combine
    start_path.insert(start_path.end(), goal_path.begin(), goal_path.end());
    planned_paths.push_back(start_path);
  }

  // Collect final statistics
  const std::chrono::time_point<std::chrono::steady_clock> cur_time
      = std::chrono::steady_clock::now();
  const std::chrono::duration<double> planning_time(cur_time - start_time);
  statistics["planning_time"] = planning_time.count();
  statistics["total_states"]
      = static_cast<double>(start_tree.Size() + goal_tree.Size());
  statistics["solutions"] = static_cast<double>(planned_paths.size());
  return MultipleSolutionPlanningResults<StateType, Container>(
      std::move(planned_paths), statistics);
}

/// Plan a single path using a single-direction RRT planner. Note that
/// StateType and SampleType are different; this makes implementation of some
/// kinodynamic planning problems easier.
///
/// @param tree existing tree. This tree must contain at least one node, but can
///     contain multiple nodes as well. All nodes in @param tree are assumed to
///     be valid starting nodes, and if they are linked together, they must have
///     valid parent-child linkage.
/// @param sampling_fn function to sample a new state.
/// @param nearest_neighbor_fn function to compute the nearest neighbor index
///     in the provided tree given the provided sample. If a negative index
///     (showing that no valid nearest neighbor could be found) planning is
///     terminated.
/// @param forward_propagation_fn function to forward propagate states from
///     the provided state to the provided sample. The propagated states are
///     returned as a vector<pair<state, relative_index>> in which state is the
///     new propagated state and relative_index is a relative index used to
///     connect the new state to the correct parent state. This relative_index
///     is the index of the parent state, relative to the vector of propagated
///     nodes. A negative value means the nearest neighbor in the tree, zero
///     means the first propagated node, and so on. NOTE - the relative parent
///     index *must* be lower than the index in the list of propagated nodes
///     i.e. the first node must have a negative value, and so on. While
///     complicated, this structure allows @param forward_propagation_fn to
///     return an entire subtree at once, which is used for certain
///     multi-outcome planners.
/// @param state_added_callback_fn callback function called once a state is
///     added to the tree, providing a mutable reference to the tree and the
///     index of the newly-added state. This can be used, for example, to update
///     a KD-tree used for nearest neighbors. You can leave this
///     default-constructed ({}) if you do not need it.
/// @param check_goal_reached_fn function to check if the provided state meets
///     goal conditions.
/// @param goal_reached_callback_fn callback function called once a state meets
///     goal conditions, providing a mutable reference to the tree and the index
///     of the new goal state. You can leave this default-constructed ({}) if
///     you do not need it.
/// @param termination_check_fn Returns true if planning has been
///     terminated. The provided int64_t is the current size of the planner
///     tree, which may be useful for a size-limited planning problem.
/// @return path + statistics where path is the solution path and
///     statistics is a map<string, double> of useful statistics collected while
///     planning.
template<typename StateType,
         typename TreeType=SimpleRRTPlannerTree<StateType>,
         typename SampleType=StateType,
         typename Container=std::vector<StateType>>
SingleSolutionPlanningResults<StateType, Container> RRTPlanSinglePath(
    TreeType& tree,
    const SamplingFunction<SampleType>& sampling_fn,
    const RRTNearestNeighborFunction<StateType, TreeType, SampleType>&
        nearest_neighbor_fn,
    const RRTForwardPropagationFunction<StateType, SampleType>&
        forward_propagation_fn,
    const RRTStateAddedCallbackFunction<StateType, TreeType>&
        state_added_callback_fn,
    const RRTCheckGoalReachedFunction<StateType>& check_goal_reached_fn,
    const RRTGoalReachedCallbackFunction<StateType, TreeType>&
        goal_reached_callback_fn,
    const RRTTerminationCheckFunction& termination_check_fn)
{
  bool solution_found = false;
  const RRTGoalReachedCallbackFunction<StateType, TreeType>
      internal_goal_reached_callback_fn
          = [&] (TreeType& planning_tree, const int64_t goal_node_index)
  {
    solution_found = true;
    if (goal_reached_callback_fn)
    {
      goal_reached_callback_fn(planning_tree, goal_node_index);
    }
  };
  const RRTTerminationCheckFunction internal_termination_check_fn
      = [&] (const int64_t current_tree_size)
  {
    return (solution_found || termination_check_fn(current_tree_size));
  };
  const auto rrt_result =
      RRTPlanMultiPath<StateType, TreeType, SampleType, Container>(
          tree, sampling_fn, nearest_neighbor_fn, forward_propagation_fn,
          state_added_callback_fn, check_goal_reached_fn,
          internal_goal_reached_callback_fn, internal_termination_check_fn);
  if (rrt_result.Paths().size() > 0)
  {
    return SingleSolutionPlanningResults<StateType, Container>(
        rrt_result.Paths().at(0), rrt_result.Statistics());
  }
  else
  {
    return SingleSolutionPlanningResults<StateType, Container>(
        rrt_result.Statistics());
  }
}

/// Plan a single path using a bidirectional RRT planner.
///
/// @param start_tree existing starting tree. This tree must contain at least
///     one node, but can contain multiple nodes as well. All nodes in @param
///     start_tree are assumed to be valid starting nodes, and if they are
///     linked together, they must have valid parent-child linkage.
/// @param goal_tree existing goal tree. This tree must contain at least one
///     node, but can contain multiple nodes as well. All nodes in @param
///     goal_tree are assumed to be valid goal nodes, and if they are linked
///     together, they must have valid parent-child linkage.
/// @param select_sample_type_fn function to select sample type (i.e. between
///     tree sample or state sample.
/// @param state_sampling_fn function to sample a new state.
/// @param tree_sampling_fn function to sample a state from the target tree.
/// @param nearest_neighbor_fn function to compute the nearest neighbor index
///     in the provided tree given the provided sample. If a negative index
///     (showing that no valid nearest neighbor could be found) planning is
///     terminated.
/// @param propagation_fn function to forward propagate states from the provided
///     state to the provided sample. The propagated states are returned as a
///     vector<pair<state, relative_index>> in which state is the new propagated
///     state and relative_index is a relative index used to connect the new
///     state to the correct parent state. This relative_index is the index of
///     the parent state, relative to the vector of propagated nodes. A negative
///     value means the nearest neighbor in the tree, zero means the first
///     propagated node, and so on. NOTE - the relative parent index *must* be
///     lower than the index in the list of propagated nodes i.e. the first node
///     must have a negative value, and so on. While complicated, this structure
///     allows @param forward_propagation_fn to return an entire subtree at
///     once, which is used for certain multi-outcome planners.
/// @param state_added_callback_fn callback function called once a state is
///     added to the tree, providing a mutable reference to the tree and the
///     index of the newly-added state. This can be used, for example, to update
///     a KD-tree used for nearest neighbors. You can leave this
///     default-constructed ({}) if you do not need it.
/// @param states_connected_fn function to check if the two provided states, one
///     from each tree, are connected, and a solution has been found.
/// @param goal_bridge_callback_fn callback function called once a solution
///     "goal bridge", that is a pair of nodes, one from each tree, has been
///     found. It provides a mutable reference to the start tree, the goal
///     bridge index in the start tree, a mutable reference to the goal tree,
///     the goal bridge index in the goal tree, and a bool flag that specifies
///     which tree is the current active tree. If this flag is true, start_tree
///     is the active tree. You can leave this default-constructed ({}) if you
///     do not need it.
/// @param select_active_tree_fn function to select the active tree for the next
///     planning iteration.
/// @param termination_check_fn Returns true if planning has been
///     terminated. The provided int64_t values are the current size of the
///     start and goal tree, respectively. These may be useful for a
///     size-limited planning problem.
/// @return path + statistics where path is the solution path and
///     statistics is a map<string, double> of useful statistics collected while
///     planning.
template<typename StateType,
         typename TreeType=SimpleRRTPlannerTree<StateType>,
         typename Container=std::vector<StateType>>
SingleSolutionPlanningResults<StateType, Container> BiRRTPlanSinglePath(
    TreeType& start_tree,
    TreeType& goal_tree,
    const BiRRTSelectSampleTypeFunction<StateType, TreeType>&
        select_sample_type_fn,
    const SamplingFunction<StateType>& state_sampling_fn,
    const BiRRTTreeSamplingFunction<StateType, TreeType>& tree_sampling_fn,
    const BiRRTNearestNeighborFunction<StateType, TreeType>&
        nearest_neighbor_fn,
    const BiRRTPropagationFunction<StateType>& propagation_fn,
    const BiRRTStateAddedCallbackFunction<StateType, TreeType>&
        state_added_callback_fn,
    const BiRRTStatesConnectedFunction<StateType>& states_connected_fn,
    const BiRRTGoalBridgeCallbackFunction<StateType, TreeType>&
        goal_bridge_callback_fn,
    const BiRRTSelectActiveTreeFunction<StateType, TreeType>&
        select_active_tree_fn,
    const BiRRTTerminationCheckFunction& termination_check_fn)
{
  bool solution_found = false;
  const BiRRTGoalBridgeCallbackFunction<StateType, TreeType>
      internal_goal_bridge_callback_fn
          = [&] (TreeType& planning_start_tree, const int64_t start_tree_index,
                 TreeType& planning_goal_tree, const int64_t goal_tree_index,
                 const BiRRTActiveTreeType active_tree_type)
  {
    solution_found = true;
    if (goal_bridge_callback_fn)
    {
      goal_bridge_callback_fn(planning_start_tree, start_tree_index,
                              planning_goal_tree, goal_tree_index,
                              active_tree_type);
    }
  };
  const BiRRTTerminationCheckFunction internal_termination_check_fn
      = [&] (const int64_t current_start_tree_size,
             const int64_t current_goal_tree_size)
  {
    return (solution_found || termination_check_fn(current_start_tree_size,
                                                   current_goal_tree_size));
  };
  const auto birrt_result =
      BiRRTPlanMultiPath<StateType, TreeType, Container>(
          start_tree, goal_tree, select_sample_type_fn, state_sampling_fn,
          tree_sampling_fn, nearest_neighbor_fn, propagation_fn,
          state_added_callback_fn, states_connected_fn,
          internal_goal_bridge_callback_fn, select_active_tree_fn,
          internal_termination_check_fn);
  if (birrt_result.Paths().size() > 0)
  {
    return SingleSolutionPlanningResults<StateType, Container>(
        birrt_result.Paths().at(0), birrt_result.Statistics());
  }
  else
  {
    return SingleSolutionPlanningResults<StateType, Container>(
        birrt_result.Statistics());
  }
}

/// Helper type to manage a timeout using std::chrono::steady_clock to measure
/// elapsed time.
class TimeoutCheckHelper
{
private:
  std::chrono::duration<double> timeout_;
  OwningMaybe<std::chrono::time_point<std::chrono::steady_clock>> start_time_;

public:
  explicit TimeoutCheckHelper(const double timeout)
  {
    if (timeout <= 0.0)
    {
      throw std::runtime_error("timeout <= 0.0");
    }
    timeout_ = std::chrono::duration<double>(timeout);
  }

  bool Check() const
  {
    if (start_time_.HasValue())
    {
      return
          ((std::chrono::steady_clock::now() - start_time_.Value()) > timeout_);
    }
    else
    {
      throw std::runtime_error(
          "Check() called on TimeoutCheckHelper that has not been started");
    }
  }

  bool CheckOrStart()
  {
    if (start_time_.HasValue())
    {
      return Check();
    }
    else
    {
      Start();
      return false;
    }
  }

  void Start()
  {
    start_time_ =
        OwningMaybe<std::chrono::time_point<std::chrono::steady_clock>>(
            std::chrono::steady_clock::now());
  }

  void Reset()
  {
    start_time_ =
        OwningMaybe<std::chrono::time_point<std::chrono::steady_clock>>();
  }
};

/// Helper function to build a termination check function for the
/// single-direction RRT planner that checks if the provided timeout has been
/// exceeded. The timeout function starts keeping track of elapsed time after
/// the first call, so you can create this function well before using it.
/// However, it can only be used once!
inline RRTTerminationCheckFunction MakeRRTTimeoutTerminationFunction(
    const double planning_timeout)
{
  TimeoutCheckHelper termination_fn_helper(planning_timeout);
  RRTTerminationCheckFunction termination_function
      = [termination_fn_helper] (const int64_t) mutable
  {
    return termination_fn_helper.CheckOrStart();
  };
  return termination_function;
}

/// Helper function to build a termination check function for the
/// bi-directional RRT planner that checks if the provided timeout has been
/// exceeded. The timeout function starts keeping track of elapsed time after
/// the first call, so you can create this function well before using it.
/// However, it can only be used once!
inline BiRRTTerminationCheckFunction MakeBiRRTTimeoutTerminationFunction(
    const double planning_timeout)
{
  TimeoutCheckHelper termination_fn_helper(planning_timeout);
  BiRRTTerminationCheckFunction termination_function
      = [termination_fn_helper] (const int64_t, const int64_t) mutable
  {
    return termination_fn_helper.CheckOrStart();
  };
  return termination_function;
}

/// Make a uniform random sample type selection function.
template<typename StateType, typename TreeType=SimpleRRTPlannerTree<StateType>>
BiRRTSelectSampleTypeFunction<StateType, TreeType>
MakeUniformRandomBiRRTSelectSampleTypeFunction(
    const utility::UniformUnitRealFunction& uniform_unit_real_fn,
    const double tree_sampling_bias)
{
  if (tree_sampling_bias >= 0.0 && tree_sampling_bias <= 1.0)
  {
    BiRRTSelectSampleTypeFunction<StateType, TreeType>
        select_sample_type_function =
            [uniform_unit_real_fn, tree_sampling_bias] (
                const TreeType&, const TreeType&, const BiRRTActiveTreeType)
    {
      const bool sample_from_tree =
          uniform_unit_real_fn() <= tree_sampling_bias;
      return (sample_from_tree)
          ? BiRRTSampleType::TREE_SAMPLE : BiRRTSampleType::STATE_SAMPLE;
    };
    return select_sample_type_function;
  }
  else
  {
    throw std::invalid_argument(
        "tree_sampling_bias is not a valid probability");
  }
}

/// Make a uniform random BiRRT active tree selection function.
template<typename StateType, typename TreeType=SimpleRRTPlannerTree<StateType>>
BiRRTSelectActiveTreeFunction<StateType, TreeType>
MakeUniformRandomBiRRTSelectActiveTreeFunction(
    const utility::UniformUnitRealFunction& uniform_unit_real_fn,
    const double p_switch_tree)
{
  if (p_switch_tree >= 0.0 && p_switch_tree <= 1.0)
  {
    BiRRTSelectActiveTreeFunction<StateType, TreeType>
        switch_active_tree_function = [uniform_unit_real_fn, p_switch_tree] (
            const TreeType&, const TreeType&,
            const BiRRTActiveTreeType active_tree_type)
    {
      const bool switch_tree = uniform_unit_real_fn() <= p_switch_tree;
      if (switch_tree)
      {
        return (active_tree_type == BiRRTActiveTreeType::START_TREE)
            ? BiRRTActiveTreeType::GOAL_TREE : BiRRTActiveTreeType::START_TREE;
      }
      else
      {
        return active_tree_type;
      }
    };
    return switch_active_tree_function;
  }
  else
  {
    throw std::invalid_argument("p_switch_tree is not a valid probability");
  }
}

/// Helper function to create a sampling function that wraps a state sampling
/// function @param state_sampling_fn and goal states @param goal_states and
/// samples states and goals according to @param goal_bias. In the basic single-
/// direction RRT with fixed goal states, you interleave sampling random states
/// (accomplished here by calling @param state_sampling_fn) and "sampling" the
/// known goal states (here, @param goal_states) with probablity
/// @param goal_bias. @param uniform_unit_real_fn Returns a uniformly
/// distributed double from [0.0, 1.0).
template<typename SampleType, typename Container=std::vector<SampleType>>
SamplingFunction<SampleType> MakeStateAndGoalsSamplingFunction(
    const SamplingFunction<SampleType>& state_sampling_fn,
    const Container& goal_states,
    const double goal_bias,
    const utility::UniformUnitRealFunction& uniform_unit_real_fn)
{
  class StateAndGoalsSamplingFunction
  {
  private:
    Container goal_samples_;
    const double goal_bias_ = 0.0;
    utility::UniformUnitRealFunction uniform_unit_real_fn_;
    const SamplingFunction<SampleType> state_sampling_fn_;

  public:
    StateAndGoalsSamplingFunction(
        const Container& goal_samples,
        const double goal_bias,
        const utility::UniformUnitRealFunction& uniform_unit_real_fn,
        const SamplingFunction<SampleType>& state_sampling_fn)
        : goal_samples_(goal_samples), goal_bias_(goal_bias),
          uniform_unit_real_fn_(uniform_unit_real_fn),
          state_sampling_fn_(state_sampling_fn)
    {
      if ((goal_bias_ < 0.0) || (goal_bias_ > 1.0))
      {
        throw std::invalid_argument(
            "goal_bias is not a valid probability");
      }
      if (goal_samples_.empty())
      {
        throw std::invalid_argument("goal_samples is empty");
      }
    }

    SampleType Sample()
    {
      if (uniform_unit_real_fn_() > goal_bias_)
      {
        return state_sampling_fn_();
      }
      else
      {
        if (goal_samples_.size() == 1)
        {
          return goal_samples_.at(0);
        }
        else
        {
          return goal_samples_.at(utility::GetUniformRandomIndex(
              uniform_unit_real_fn_, goal_samples_.size()));
        }
      }
    }
  };

  StateAndGoalsSamplingFunction sampling_fn_helper(
      goal_states, goal_bias, uniform_unit_real_fn, state_sampling_fn);
  SamplingFunction<SampleType> sampling_function
      = [sampling_fn_helper] (void) mutable
  {
    return sampling_fn_helper.Sample();
  };
  return sampling_function;
}

/// Make a uniform random BiRRT tree sampling function.
template<typename StateType, typename TreeType=SimpleRRTPlannerTree<StateType>>
BiRRTTreeSamplingFunction<StateType, TreeType>
MakeUniformRandomBiRRTTreeSamplingFunction(
    const utility::UniformUnitRealFunction& uniform_unit_real_fn)
{
  BiRRTTreeSamplingFunction<StateType, TreeType> tree_sample_function
      = [uniform_unit_real_fn] (
          const TreeType& target_tree, const BiRRTActiveTreeType)
  {
    const int64_t target_node_index = utility::GetUniformRandomIndex(
        uniform_unit_real_fn, target_tree.Size());
    return target_node_index;
  };
  return tree_sample_function;
}

/// Helper function to create a serial/parallel linear nearest neighbors
/// function for use in RRT planner given the provided state-to-state distance
/// function @param distance_fn and flag @param use_parallel which selects if
/// parallel linear nearest neighbors should be performed.
template<typename StateType, typename TreeType=SimpleRRTPlannerTree<StateType>,
         typename SampleType=StateType>
RRTNearestNeighborFunction<StateType, TreeType, SampleType>
MakeLinearRRTNearestNeighborsFunction(
    const std::function<double(const StateType&,
                               const SampleType&)>& distance_fn,
    const bool use_parallel = true)
{
  RRTNearestNeighborFunction<StateType, TreeType, SampleType>
      nearest_neighbors_function = [=] (
          const TreeType& tree, const SampleType& sampled)
  {
    std::function<double(const typename TreeType::NodeType&,
                         const SampleType&)>
        real_distance_fn =
            [&](const typename TreeType::NodeType& tree_node,
                const SampleType& sample) {
              const StateType& candidate_q = tree_node.GetValueImmutable();
              return distance_fn(candidate_q, sample);
            };
    const auto neighbors = simple_knearest_neighbors::GetKNearestNeighbors(
        tree.GetNodesImmutable(), sampled, real_distance_fn, 1, use_parallel);
    if (neighbors.size() > 0)
    {
      const auto& nearest_neighbor = neighbors.at(0);
      return nearest_neighbor.Index();
    }
    else
    {
      throw std::runtime_error("NN check produced no neighbors");
    }
  };
  return nearest_neighbors_function;
}

/// Helper function to create a serial/parallel linear nearest neighbors
/// function for use in RRT planner given the provided state-to-state distance
/// function @param distance_fn and flag @param use_parallel which selects if
/// parallel linear nearest neighbors should be performed. Use this helper for
/// kinematic planning problems, in which state type and sample type are the
/// same.
template<typename StateType, typename TreeType=SimpleRRTPlannerTree<StateType>>
RRTNearestNeighborFunction<StateType, TreeType, StateType>
MakeKinematicLinearRRTNearestNeighborsFunction(
    const std::function<double(const StateType&,
                               const StateType&)>& distance_fn,
    const bool use_parallel = true)
{
  return MakeLinearRRTNearestNeighborsFunction<StateType, TreeType, StateType>(
      distance_fn, use_parallel);
}

/// Helper function to create a serial/parallel linear nearest neighbors
/// function for use in BiRRT planners given the provided state-to-state
/// distance function @param distance_fn and flag @param use_parallel which
/// selects if parallel linear nearest neighbors should be performed. This is
/// best used for kinematic planning problems, where nearest neighbors is the
/// same for both start and goal tree.
template<typename StateType, typename TreeType=SimpleRRTPlannerTree<StateType>>
BiRRTNearestNeighborFunction<StateType, TreeType>
MakeKinematicLinearBiRRTNearestNeighborsFunction(
    const std::function<double(const StateType&,
                               const StateType&)>& distance_fn,
    const bool use_parallel = true)
{
  BiRRTNearestNeighborFunction<StateType, TreeType> nearest_neighbors_function
      = [=] (
          const TreeType& tree, const StateType& sampled,
          const BiRRTActiveTreeType)
  {
    std::function<double(const typename TreeType::NodeType&,
                         const StateType&)>
        real_distance_fn =
            [&](const typename TreeType::NodeType& tree_node,
                const StateType& sample) {
              const StateType& candidate_q = tree_node.GetValueImmutable();
              return distance_fn(candidate_q, sample);
            };
    const auto neighbors = simple_knearest_neighbors::GetKNearestNeighbors(
        tree.GetNodesImmutable(), sampled, real_distance_fn, 1, use_parallel);
    if (neighbors.size() > 0)
    {
      const auto& nearest_neighbor = neighbors.at(0);
      return nearest_neighbor.Index();
    }
    else
    {
      throw std::runtime_error("NN check produced no neighbors");
    }
  };
  return nearest_neighbors_function;
}

/// Helper function to create a RRT-Extend forward propagation function for
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
RRTForwardPropagationFunction<StateType>
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
  RRTForwardPropagationFunction<StateType> forward_propagation_fn
      = [=] (const StateType& nearest, const StateType& sampled)
  {
    const double distance = distance_fn(nearest, sampled);
    const double ratio = (distance > step_size) ? (step_size / distance) : 1.0;
    StateType extend_state = (ratio < 1.0)
        ? state_interpolation_fn(nearest, sampled, ratio) : sampled;
    ForwardPropagation<StateType> forward_propagation;
    if (edge_validity_check_fn(nearest, extend_state))
    {
      // -1 is the parent offset used in adding the new node to the tree.
      forward_propagation.emplace_back(std::move(extend_state), -1);
    }
    return forward_propagation;
  };
  return forward_propagation_fn;
}

/// Helper function to create a BiRRT-Extend forward propagation function for
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
BiRRTPropagationFunction<StateType>
MakeKinematicBiRRTExtendPropagationFunction(
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
  BiRRTPropagationFunction<StateType> forward_propagation_fn
      = [=] (
          const StateType& nearest, const StateType& sampled,
          const BiRRTActiveTreeType)
  {
    const double distance = distance_fn(nearest, sampled);
    const double ratio = (distance > step_size) ? (step_size / distance) : 1.0;
    StateType extend_state = (ratio < 1.0)
        ? state_interpolation_fn(nearest, sampled, ratio) : sampled;
    ForwardPropagation<StateType> forward_propagation;
    if (edge_validity_check_fn(nearest, extend_state))
    {
      // -1 is the parent offset used in adding the new node to the tree.
      forward_propagation.emplace_back(std::move(extend_state), -1);
    }
    return forward_propagation;
  };
  return forward_propagation_fn;
}

/// Helper function to create a RRT-Connect forward propagation function for
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
RRTForwardPropagationFunction<StateType>
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
  RRTForwardPropagationFunction<StateType> forward_propagation_fn
      = [=] (const StateType& nearest, const StateType& sampled)
  {
    ForwardPropagation<StateType> propagated_states;
    int64_t parent_offset = -1;
    // Compute a maximum number of steps to take
    const double total_distance = distance_fn(nearest, sampled);
    const int32_t total_steps =
        static_cast<int32_t>(std::ceil(total_distance / step_size));
    StateType current = nearest;
    int32_t steps = 0;
    bool completed = false;
    while (!completed && (steps < total_steps))
    {
      // Compute the next intermediate target state
      StateType current_target = sampled;
      const double target_distance = distance_fn(current, current_target);
      if (target_distance > step_size)
      {
        const double step_fraction = step_size / target_distance;
        current_target =
            state_interpolation_fn(current, sampled, step_fraction);
      }
      else if (std::abs(target_distance) <=
                 std::numeric_limits<double>::epsilon())
      {
        // If we've reached the target state, stop
        completed = true;
        break;
      }
      else
      {
        // If we're less than step size away, this is our last step
        completed = true;
      }
      // If the current edge is valid, we keep going
      if (edge_validity_check_fn(current, current_target))
      {
        current = current_target;
        propagated_states.emplace_back(
            std::move(current_target), parent_offset);
        parent_offset++;
        steps++;
      }
      else
      {
        completed = true;
      }
    }
    return propagated_states;
  };
  return forward_propagation_fn;
}

/// Helper function to create a BiRRT-Connect forward propagation function for
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
BiRRTPropagationFunction<StateType>
MakeKinematicBiRRTConnectPropagationFunction(
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
  BiRRTPropagationFunction<StateType> forward_propagation_fn
      = [=] (
          const StateType& nearest, const StateType& sampled,
          const BiRRTActiveTreeType)
  {
    ForwardPropagation<StateType> propagated_states;
    int64_t parent_offset = -1;
    // Compute a maximum number of steps to take
    const double total_distance = distance_fn(nearest, sampled);
    const int32_t total_steps =
        static_cast<int32_t>(std::ceil(total_distance / step_size));
    StateType current = nearest;
    int32_t steps = 0;
    bool completed = false;
    while (!completed && (steps < total_steps))
    {
      // Compute the next intermediate target state
      StateType current_target = sampled;
      const double target_distance = distance_fn(current, current_target);
      if (target_distance > step_size)
      {
        const double step_fraction = step_size / target_distance;
        current_target =
            state_interpolation_fn(current, sampled, step_fraction);
      }
      else if (std::abs(target_distance) <=
                 std::numeric_limits<double>::epsilon())
      {
        // If we've reached the target state, stop
        completed = true;
        break;
      }
      else
      {
        // If we're less than step size away, this is our last step
        completed = true;
      }
      // If the current edge is valid, we keep going
      if (edge_validity_check_fn(current, current_target))
      {
        current = current_target;
        propagated_states.emplace_back(
            std::move(current_target), parent_offset);
        parent_offset++;
        steps++;
      }
      else
      {
        completed = true;
      }
    }
    return propagated_states;
  };
  return forward_propagation_fn;
}
}  // namespace simple_rrt_planner
}  // namespace common_robotics_utilities
