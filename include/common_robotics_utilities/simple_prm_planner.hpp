#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

#include <common_robotics_utilities/maybe.hpp>
#include <common_robotics_utilities/openmp_helpers.hpp>
#include <common_robotics_utilities/simple_graph.hpp>
#include <common_robotics_utilities/simple_graph_search.hpp>
#include <common_robotics_utilities/simple_knearest_neighbors.hpp>

namespace common_robotics_utilities
{
namespace simple_prm_planner
{
/// Enum to select the direction of node-to-graph connection. If the "distance"
/// function is not symmetrical (i.e. an edge can have different "distance" in
/// different directions), then the direction is needed to call the distance
/// function correctly.
enum class NNDistanceDirection {ROADMAP_TO_NEW_STATE, NEW_STATE_TO_ROADMAP};

/// Attempt to add a single state as a new node to the roadmap.
/// @param state new state to add.
/// @param roadmap existing roadmap.
/// @param distance_fn distance function for state-to-state distance. If
/// use_parallel is true, this must be thread-safe.
/// @param edge_validity_check_fn edge validity checking function. If
/// use_parallel is true, this must be thread-safe.
/// @param K number of K neighboring states in the roadmap to attempt to connect
/// the new state to.
/// @param max_node_index_for_knn maximum node index to use for KNN. To consider
/// all nodes in roadmap, provide roadmap.Size().
/// @param use_parallel use parallel operations when possible.
/// @param connection_is_symmetric are distance and edge validity symmetric
/// (i.e. distance_fn(a, b) == distance_fn(b, a) and
/// edge_validity_check_fn(a, b) == edge_validity_check_fn(b, a))? Asymmetric
/// distance and edge validity functions are more expensive to work with and
/// should be avoided when possible.
/// @param add_duplicate_states should @param state be added to the roadmap if
/// it is a duplicate of an existing state? @param state is considered a
/// duplicate if one of the K neighboring states has distance zero from @param
/// state.
/// @return the index of the newly-added node OR the index of an existing
/// duplicate node in the roadmap. You can check which happended by querying the
/// size of the roadmap before and after calling AddNodeToRoadmap.
template<typename T, typename GraphType>
inline int64_t AddNodeToRoadmap(
    const T& state,
    const NNDistanceDirection nn_distance_direction,
    GraphType& roadmap,
    const std::function<double(const T&, const T&)>& distance_fn,
    const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
    const int64_t K,
    const int64_t max_node_index_for_knn,
    const bool use_parallel,
    const bool connection_is_symmetric,
    const bool add_duplicate_states)
{
  // Make the node->graph or graph->node distance function as needed
  std::function<double(const typename GraphType::NodeType&, const T&)>
      graph_distance_fn = nullptr;
  if (nn_distance_direction == NNDistanceDirection::ROADMAP_TO_NEW_STATE)
  {
    graph_distance_fn = [&] (const typename GraphType::NodeType& node,
                             const T& query_state)
    {
      return distance_fn(node.GetValueImmutable(), query_state);
    };
  }
  else
  {
    graph_distance_fn = [&] (const typename GraphType::NodeType& node,
                             const T& query_state)
    {
      return distance_fn(query_state, node.GetValueImmutable());
    };
  }

  // Call KNN with the distance function.
  const auto nearest_neighbors =
      simple_knearest_neighbors::GetKNearestNeighbors(
          simple_graph::GraphKNNAdapter<GraphType>(
              roadmap, max_node_index_for_knn),
          state, graph_distance_fn, K, use_parallel);

  // Check if we already have this state in the roadmap
  // (and we don't want to add duplicates)
  if (add_duplicate_states == false)
  {
    for (const auto& neighbor : nearest_neighbors)
    {
      if (neighbor.Distance() == 0.0)
      {
        return neighbor.Index();
      }
    }
  }

  // Add the new node AFTER KNN is performed
  const int64_t new_node_index = roadmap.AddNode(state);

  // Parallelize the collision-checking and distance computation
  // Because we don't need any special caching here, we define the loop contents
  // as a lambda and then call branch between parfor/for loops that call it.
  std::vector<std::pair<double, double>> nearest_neighbors_distances(
      nearest_neighbors.size());

  CRU_OMP_PARALLEL_FOR_IF(use_parallel)
  for (size_t idx = 0; idx < nearest_neighbors.size(); idx++)
  {
    const auto& nearest_neighbor = nearest_neighbors.at(idx);
    const int64_t nearest_neighbor_index = nearest_neighbor.Index();
    const double graph_to_node_distance = nearest_neighbor.Distance();
    const T& nearest_neighbor_state
        = roadmap.GetNodeImmutable(nearest_neighbor_index).GetValueImmutable();
    const bool graph_to_node_edge_validity
        = edge_validity_check_fn(nearest_neighbor_state, state);
    if (graph_to_node_edge_validity && connection_is_symmetric)
    {
      // Distance is symmetric and the edge is valid
      nearest_neighbors_distances.at(idx)
          = std::make_pair(graph_to_node_distance,
                           graph_to_node_distance);
    }
    else if (!connection_is_symmetric)
    {
      const bool node_to_graph_edge_validity
          = edge_validity_check_fn(state, nearest_neighbor_state);
      const double node_to_graph_distance
          = distance_fn(state,
                        roadmap.GetNodeImmutable(nearest_neighbor_index)
                            .GetValueImmutable());
      // We use -1 as a signaling value of an infeasible edge
      const double real_graph_to_node_distance
          = (graph_to_node_edge_validity) ? graph_to_node_distance : -1.0;
      const double real_node_to_graph_distance
          = (node_to_graph_edge_validity) ? node_to_graph_distance : -1.0;
      // Set the distance values depending on direction
      if (nn_distance_direction == NNDistanceDirection::ROADMAP_TO_NEW_STATE)
      {
        nearest_neighbors_distances.at(idx)
            = std::make_pair(real_graph_to_node_distance,
                             real_node_to_graph_distance);
      }
      else
      {
        nearest_neighbors_distances.at(idx)
            = std::make_pair(real_node_to_graph_distance,
                             real_graph_to_node_distance);
      }
    }
    else
    {
      // Distance is symmetric, but the edge is not valid!
      nearest_neighbors_distances.at(idx) = std::make_pair(-1.0, -1.0);
    }
  };

  // THIS MUST BE SERIAL - add edges to roadmap
  for (size_t idx = 0; idx < nearest_neighbors.size(); idx++)
  {
    const auto& nearest_neighbor = nearest_neighbors.at(idx);
    const int64_t nearest_neighbor_index = nearest_neighbor.Index();
    const std::pair<double, double>& nearest_neighbor_distances
        = nearest_neighbors_distances.at(idx);
    // Add the edges individually to allow for different "distances" in each
    // direction - for example, if the "distance" is a probability of the edge
    // being traversable, the probability may not be symmetric.
    if (nearest_neighbor_distances.first >= 0.0)
    {
      roadmap.AddEdgeBetweenNodes(nearest_neighbor_index, new_node_index,
                                  nearest_neighbor_distances.first);
    }
    if (nearest_neighbor_distances.second >= 0.0)
    {
      roadmap.AddEdgeBetweenNodes(new_node_index, nearest_neighbor_index,
                                  nearest_neighbor_distances.second);
    }
  }

  return new_node_index;
}

/// Attempt to grow a roadmap consisting of a graph of states.
/// @param roadmap roadmap to grow. This may be empty to start with.
/// @param sampling_fn function to sample new states.
/// @param distance_fn distance function for state-to-state distance. If
/// use_parallel is true, this must be thread-safe.
/// @param state_validity_check function to check if a sampled state is valid.
/// @param edge_validity_check_fn edge validity checking function. If
/// use_parallel is true, this must be thread-safe.
/// @param termination_check_fn function to check if we have finished building
/// the roadmap. Returns true when roadmap construction is done. The provided
/// int64_t is the current size of the roadmap, since roadmap size is a common
/// termination condition.
/// @param K number of K neighboring states in the roadmap to attempt to connect
/// the new state to.
/// @param use_parallel use parallel operations when possible.
/// @param connection_is_symmetric are distance and edge validity symmetric
/// (i.e. distance_fn(a, b) == distance_fn(b, a) and
/// edge_validity_check_fn(a, b) == edge_validity_check_fn(b, a))? Asymmetric
/// distance and edge validity functions are more expensive to work with and
/// should be avoided when possible.
/// @param add_duplicate_states should a new state be added to the roadmap if
/// it is a duplicate of an existing state? A new state is considered a
/// duplicate if one of the K neighboring states has distance zero from the
/// state.
/// @return statistics as a map<string, double> of useful statistics collected
/// while growing the roadmap.
template<typename T, typename GraphType>
inline std::map<std::string, double> GrowRoadMap(
    GraphType& roadmap,
    const std::function<T(void)>& sampling_fn,
    const std::function<double(const T&, const T&)>& distance_fn,
    const std::function<bool(const T&)>& state_validity_check_fn,
    const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
    const std::function<bool(const int64_t)>& termination_check_fn,
    const int64_t K,
    const bool use_parallel = true,
    const bool connection_is_symmetric = true,
    const bool add_duplicate_states = false)
{
  std::map<std::string, double> statistics;
  statistics["total_samples"] = 0.0;
  statistics["successful_samples"] = 0.0;
  statistics["duplicate_samples"] = 0.0;
  statistics["failed_samples"] = 0.0;
  // Update the start time
  const std::chrono::time_point<std::chrono::steady_clock> start_time
      = std::chrono::steady_clock::now();
  while (!termination_check_fn(roadmap.Size()))
  {
    const T random_state = sampling_fn();
    statistics["total_samples"] += 1.0;
    if (state_validity_check_fn(random_state))
    {
      const int64_t pre_size = roadmap.Size();
      AddNodeToRoadmap<T, GraphType>(
          random_state, NNDistanceDirection::ROADMAP_TO_NEW_STATE, roadmap,
          distance_fn, edge_validity_check_fn, K, pre_size, use_parallel,
          connection_is_symmetric, add_duplicate_states);
      const int64_t post_size = roadmap.Size();
      if (post_size > pre_size)
      {
        statistics["successful_samples"] += 1.0;
      }
      else
      {
        statistics["duplicate_samples"] += 1.0;
      }
    }
    else
    {
      statistics["failed_samples"] += 1.0;
    }
  }
  const std::chrono::time_point<std::chrono::steady_clock> cur_time
      = std::chrono::steady_clock::now();
  const std::chrono::duration<double> growing_time(cur_time - start_time);
  statistics["growing_time"] = growing_time.count();
  return statistics;
}

/// Build a roadmap consisting of a graph of states.
/// @param roadmap_size size of roadmap to build.
/// @param sampling_fn function to sample new states.
/// @param distance_fn distance function for state-to-state distance. If
/// use_parallel is true, this must be thread-safe.
/// @param state_validity_check function to check if a sampled state is valid.
/// @param edge_validity_check_fn edge validity checking function. If
/// use_parallel is true, this must be thread-safe.
/// @param K number of K neighboring states in the roadmap to attempt to connect
/// the new state to.
/// @param max_valid_sample_tries maximum number of calls to sampling_fn to
/// produce a valid sample, throws if exceeded.
/// @param use_parallel use parallel operations when possible.
/// @param connection_is_symmetric are distance and edge validity symmetric
/// (i.e. distance_fn(a, b) == distance_fn(b, a) and
/// edge_validity_check_fn(a, b) == edge_validity_check_fn(b, a))? Asymmetric
/// distance and edge validity functions are more expensive to work with and
/// should be avoided when possible.
/// @param add_duplicate_states should a new state be added to the roadmap if
/// it is a duplicate of an existing state? A new state is considered a
/// duplicate if one of the K neighboring states has distance zero from the
/// state.
template<typename T, typename GraphType>
GraphType BuildRoadMap(
    const int64_t roadmap_size,
    const std::function<T(void)>& sampling_fn,
    const std::function<double(const T&, const T&)>& distance_fn,
    const std::function<bool(const T&)>& state_validity_check_fn,
    const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
    const int64_t K,
    const int32_t max_valid_sample_tries,
    const bool use_parallel = true,
    const bool connection_is_symmetric = true,
    const bool add_duplicate_states = false)
{
  if (roadmap_size < 0)
  {
    throw std::runtime_error("roadmap_size < 0");
  }
  if (K < 1)
  {
    throw std::runtime_error("K < 1");
  }
  if (max_valid_sample_tries < 1)
  {
    throw std::runtime_error("max_valid_sample_tries < 1");
  }

  std::vector<OwningMaybe<T>, OwningMaybeAllocator<T>> roadmap_states(
      static_cast<size_t>(roadmap_size));

  const std::function<double(const OwningMaybe<T>&, const T&)>
      roadmap_states_distance_fn = [&](
          const OwningMaybe<T>& roadmap_state, const T& sample)
  {
    if (roadmap_state.HasValue())
    {
      return distance_fn(roadmap_state.Value(), sample);
    }
    else
    {
      return std::numeric_limits<double>::infinity();
    }
  };

  // Define a helper function to sample valid states, or record that sampling
  // failed. This method can't throw, as throwing inside an OpenMP loop is not
  // recoverable.
  const std::function<OwningMaybe<T>(void)> valid_sample_fn = [&](void)
  {
    int32_t tries = 0;
    while (tries < max_valid_sample_tries)
    {
      tries++;
      const T sample = sampling_fn();
      const bool state_valid = state_validity_check_fn(sample);
      if (state_valid && add_duplicate_states)
      {
        return OwningMaybe<T>(sample);
      }
      else if (state_valid)
      {
        const auto nearest_state =
            simple_knearest_neighbors::GetKNearestNeighbors(
                roadmap_states, sample, roadmap_states_distance_fn, 1,
                use_parallel).at(0);
        if (nearest_state.Distance() > 0.0)
        {
          return OwningMaybe<T>(sample);
        }
      }
    }
    return OwningMaybe<T>();
  };

  // Sample roadmap_size valid configurations. This can only be parallelized if
  // add_duplicate_states is true, since the check for duplicate states would
  // be a race condition otherwise.
  const bool use_parallel_sampling = use_parallel && add_duplicate_states;
  CRU_OMP_PARALLEL_FOR_IF(use_parallel_sampling)
  for (size_t index = 0; index < roadmap_states.size(); index++)
  {
    roadmap_states.at(index) = valid_sample_fn();
  }

  // Populate the roadmap from the sampled valid configurations.
  GraphType roadmap(roadmap_size);
  for (const OwningMaybe<T>& maybe_state : roadmap_states)
  {
    if (maybe_state.HasValue())
    {
      roadmap.AddNode(maybe_state.Value());
    }
    else
    {
      throw std::runtime_error("Failed to sample valid state");
    }
  }

  // Distance function for KNN checks.
  const std::function<double(const typename GraphType::NodeType&, const T&)>
      roadmap_to_state_distance_fn = [&](
          const typename GraphType::NodeType& node, const T& state)
  {
    return distance_fn(node.GetValueImmutable(), state);
  };

  // Perform edge validity and distance checks for all nodes, optionally in
  // parallel.
  CRU_OMP_PARALLEL_FOR_IF(use_parallel)
  for (int64_t node_index = 0; node_index < roadmap.Size(); ++node_index)
  {
    auto& node = roadmap.GetNodeMutable(node_index);
    const T& state = node.GetValueImmutable();

    // Find K+1 nearest neighbors, since KNN will find the current node too.
    const auto nearest_neighbors =
        simple_knearest_neighbors::GetKNearestNeighbors(
            simple_graph::GraphKNNAdapter<GraphType>(roadmap), state,
            roadmap_to_state_distance_fn, K + 1, false);

    for (const auto& neighbor : nearest_neighbors)
    {
      const int64_t other_node_index = neighbor.Index();
      // Don't try to connect the node to itself.
      if (other_node_index != node_index)
      {
        const T& other_state =
            roadmap.GetNodeImmutable(other_node_index).GetValueImmutable();
        const bool other_state_to_state_valid =
            edge_validity_check_fn(other_state, state);

        if (other_state_to_state_valid)
        {
          const double distance = neighbor.Distance();
          node.AddInEdge(typename GraphType::EdgeType(
              other_node_index, node_index, distance));
        }

        if (connection_is_symmetric && other_state_to_state_valid)
        {
          const double distance = neighbor.Distance();
          node.AddOutEdge(typename GraphType::EdgeType(
              node_index, other_node_index, distance));
        }
        else if (edge_validity_check_fn(state, other_state))
        {
          const double distance = distance_fn(state, other_state);
          node.AddOutEdge(typename GraphType::EdgeType(
              node_index, other_node_index, distance));
        }
      }
    }
  }

  // Helpers for checking if an edge is present in another node.
  const auto has_in_edge_from = [](
      const typename GraphType::NodeType& node, const int64_t other_node_index)
  {
    for (const auto& in_edge : node.GetInEdgesImmutable())
    {
      if (in_edge.GetFromIndex() == other_node_index)
      {
        return true;
      }
    }
    return false;
  };

  const auto has_out_edge_to = [](
      const typename GraphType::NodeType& node, const int64_t other_node_index)
  {
    for (const auto& out_edge : node.GetOutEdgesImmutable())
    {
      if (out_edge.GetToIndex() == other_node_index)
      {
        return true;
      }
    }
    return false;
  };

  // Go through the roadmap and make node linkages are consistent.
  for (int64_t node_index = 0; node_index < roadmap.Size(); ++node_index)
  {
    const auto& node = roadmap.GetNodeImmutable(node_index);

    for (const auto& in_edge : node.GetInEdgesImmutable())
    {
      const int64_t other_node_index = in_edge.GetFromIndex();
      auto& other_node = roadmap.GetNodeMutable(other_node_index);
      if (!has_out_edge_to(other_node, node_index))
      {
        other_node.AddOutEdge(typename GraphType::EdgeType(
            other_node_index, node_index, in_edge.GetWeight()));
      }
    }

    for (const auto& out_edge : node.GetOutEdgesImmutable())
    {
      const int64_t other_node_index = out_edge.GetToIndex();
      auto& other_node = roadmap.GetNodeMutable(other_node_index);
      if (!has_in_edge_from(other_node, node_index))
      {
        other_node.AddInEdge(typename GraphType::EdgeType(
            node_index, other_node_index, out_edge.GetWeight()));
      }
    }
  }

  return roadmap;
}

/// Update edge distances in a roadmap.
/// @param roadmap existing roadmap to update.
/// @param edge_validity_check_fn edge validity checking function. If
/// use_parallel is true, this must be thread-safe.
/// @param distance_fn distance function for state-to-state distance. If
/// use_parallel is true, this must be thread-safe.
/// @param use_parallel use parallel operations when possible.
template<typename T, typename GraphType>
inline void UpdateRoadMapEdges(
    GraphType& roadmap,
    const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
    const std::function<double(const T&, const T&)>& distance_fn,
    const bool use_parallel = true)
{
  if (roadmap.CheckGraphLinkage() == false)
  {
    throw std::invalid_argument("Provided roadmap has invalid linkage");
  }

  CRU_OMP_PARALLEL_FOR_IF(use_parallel)
  for (int64_t current_node_index = 0; current_node_index < roadmap.Size();
       current_node_index++)
  {
    auto& current_node = roadmap.GetNodeMutable(current_node_index);
    auto& current_node_out_edges = current_node.GetOutEdgesMutable();
    for (auto& current_out_edge : current_node_out_edges)
    {
      const int64_t other_node_idx = current_out_edge.GetToIndex();
      auto& other_node = roadmap.GetNodeMutable(other_node_idx);
      auto& other_node_in_edges = other_node.GetInEdgesMutable();
      const double updated_weight
          = (edge_validity_check_fn(current_node.GetValueImmutable(),
                                    other_node.GetValueImmutable()))
            ? distance_fn(current_node.GetValueImmutable(),
                          other_node.GetValueImmutable())
            : std::numeric_limits<double>::infinity();
      // Update our out edge
      current_out_edge.SetWeight(updated_weight);
      // Update the other node's in edges
      for (auto& other_in_edge : other_node_in_edges)
      {
        if (other_in_edge.GetFromIndex() == current_node_index)
        {
          other_in_edge.SetWeight(updated_weight);
        }
      }
    }
  };
}

/// Extracts the solution path from the roadmap.
/// @param roadmap roadmap used to find solution path.
/// @param astar_index_solution A* planned path, in terms of node indices in the
/// provided roadmap.
template<typename T, typename Container, typename GraphType>
inline simple_astar_search::AstarResult<T, Container> ExtractSolution(
    const GraphType& roadmap,
    const simple_graph_search::AstarIndexResult& astar_index_solution)
{
  Container solution_path;
  solution_path.reserve(astar_index_solution.Path().size());
  for (const int64_t solution_path_index : astar_index_solution.Path())
  {
    const T& solution_path_state
        = roadmap.GetNodeImmutable(solution_path_index).GetValueImmutable();
    solution_path.push_back(solution_path_state);
  }
  solution_path.shrink_to_fit();
  return simple_astar_search::AstarResult<T, Container>(
      solution_path, astar_index_solution.PathCost());
}

/// Find the best path from a start state to a goal state in a lazy manner, i.e.
/// the edge validity and distances of the graph are not trusted and instead the
/// provided @param edge_validity_check_fn and @param distance_fn are used. This
/// is used in "lazy-PRM" where collision checking is deferred from roadmap
/// construction to query time. Note that this can be much more expensive than
/// normal PRM queries, so it should only be used when the roadmap is frequently
/// changing or vastly larger than the region needed for most queries.
/// Starting and goal states are added to the roadmap.
/// @param starts multiple start states.
/// @param goals multiple goal states.
/// @param roadmap existing roadmap.
/// @param distance_fn distance function for state-to-state distance. If
/// use_parallel is true, this must be thread-safe.
/// @param edge_validity_check_fn edge validity checking function. If
/// use_parallel is true, this must be thread-safe.
/// @param K number of K neighboring states in the roadmap to attempt to connect
/// the new state to.
/// @param use_parallel use parallel operations when possible.
/// @param connection_is_symmetric are distance and edge validity symmetric
/// (i.e. distance_fn(a, b) == distance_fn(b, a) and
/// edge_validity_check_fn(a, b) == edge_validity_check_fn(b, a))? Asymmetric
/// distance and edge validity functions are more expensive to work with and
/// should be avoided when possible.
/// @param add_duplicate_states should @param state be added to the roadmap if
/// it is a duplicate of an existing state? @param state is considered a
/// duplicate if one of the K neighboring states has distance zero from @param
/// state.
/// @param limit_astar_pqueue_duplicates use an additional map to limit the
/// duplicate states added to the A* pqueue?
/// @return path + length of the best path from a single start to the goal.
/// If no solution exists, path is empty and length is infinity.
template<typename T, typename Container, typename GraphType>
inline simple_astar_search::AstarResult<T, Container>
LazyQueryPathAndAddNodes(
    const Container& starts,
    const Container& goals,
    GraphType& roadmap,
    const std::function<double(const T&, const T&)>& distance_fn,
    const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
    const int64_t K,
    const bool use_parallel = true,
    const bool connection_is_symmetric = true,
    const bool add_duplicate_states = false,
    const bool limit_astar_pqueue_duplicates = true)
{
  if (starts.empty())
  {
    throw std::runtime_error("starts is empty");
  }
  if (goals.empty())
  {
    throw std::runtime_error("goals is empty");
  }
  // Add the start nodes to the roadmap
  const int64_t pre_starts_size = roadmap.Size();
  std::vector<int64_t> start_node_indices(starts.size());
  for (size_t start_idx = 0; start_idx < starts.size(); start_idx++)
  {
    const T& start = starts.at(start_idx);
    start_node_indices.at(start_idx) = AddNodeToRoadmap<T, GraphType>(
        start, NNDistanceDirection::NEW_STATE_TO_ROADMAP, roadmap, distance_fn,
        edge_validity_check_fn, K, pre_starts_size, use_parallel,
        connection_is_symmetric, add_duplicate_states);
  }
  // Add the goal nodes to the roadmap
  const int64_t pre_goals_size = roadmap.Size();
  std::vector<int64_t> goal_node_indices(goals.size());
  for (size_t goal_idx = 0; goal_idx < goals.size(); goal_idx++)
  {
    const T& goal = goals.at(goal_idx);
    goal_node_indices.at(goal_idx) = AddNodeToRoadmap<T, GraphType>(
        goal, NNDistanceDirection::ROADMAP_TO_NEW_STATE, roadmap, distance_fn,
        edge_validity_check_fn, K, pre_goals_size, use_parallel,
        connection_is_symmetric, add_duplicate_states);
  }
  // Call graph A*
  const auto astar_result =
      simple_graph_search::PerformLazyAstarSearch<T, GraphType>(
          roadmap, start_node_indices, goal_node_indices,
          edge_validity_check_fn, distance_fn, distance_fn,
          limit_astar_pqueue_duplicates);
  // Convert the solution path from A* provided as indices into real states
  return ExtractSolution<T, Container, GraphType>(roadmap, astar_result);
}

/// Find the best path from one of a set of starting states to one of a set of
/// goal states. Starting and goal states are added to the roadmap.
/// @param starts multiple start states.
/// @param goals multiple goal states.
/// @param roadmap existing roadmap.
/// @param distance_fn distance function for state-to-state distance. If
/// use_parallel is true, this must be thread-safe.
/// @param edge_validity_check_fn edge validity checking function. If
/// use_parallel is true, this must be thread-safe.
/// @param K number of K neighboring states in the roadmap to attempt to connect
/// the new state to.
/// @param use_parallel use parallel operations when possible.
/// @param connection_is_symmetric are distance and edge validity symmetric
/// (i.e. distance_fn(a, b) == distance_fn(b, a) and
/// edge_validity_check_fn(a, b) == edge_validity_check_fn(b, a))? Asymmetric
/// distance and edge validity functions are more expensive to work with and
/// should be avoided when possible.
/// @param add_duplicate_states should @param state be added to the roadmap if
/// it is a duplicate of an existing state? @param state is considered a
/// duplicate if one of the K neighboring states has distance zero from @param
/// state.
/// @param limit_astar_pqueue_duplicates use an additional map to limit the
/// duplicate states added to the A* pqueue?
/// @return path + length of the best path from a single start to the goal.
/// If no solution exists, path is empty and length is infinity.
template<typename T, typename Container, typename GraphType>
inline simple_astar_search::AstarResult<T, Container>
QueryPathAndAddNodes(
    const Container& starts,
    const Container& goals,
    GraphType& roadmap,
    const std::function<double(const T&, const T&)>& distance_fn,
    const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
    const int64_t K,
    const bool use_parallel = true,
    const bool connection_is_symmetric = true,
    const bool add_duplicate_states = false,
    const bool limit_astar_pqueue_duplicates = true)
{
  if (starts.empty())
  {
    throw std::runtime_error("starts is empty");
  }
  if (goals.empty())
  {
    throw std::runtime_error("goals is empty");
  }
  // Add the start nodes to the roadmap
  const int64_t pre_starts_size = roadmap.Size();
  std::vector<int64_t> start_node_indices(starts.size());
  for (size_t start_idx = 0; start_idx < starts.size(); start_idx++)
  {
    const T& start = starts.at(start_idx);
    start_node_indices.at(start_idx) = AddNodeToRoadmap<T, GraphType>(
        start, NNDistanceDirection::NEW_STATE_TO_ROADMAP, roadmap, distance_fn,
        edge_validity_check_fn, K, pre_starts_size, use_parallel,
        connection_is_symmetric, add_duplicate_states);
  }
  // Add the goal nodes to the roadmap
  const int64_t pre_goals_size = roadmap.Size();
  std::vector<int64_t> goal_node_indices(goals.size());
  for (size_t goal_idx = 0; goal_idx < goals.size(); goal_idx++)
  {
    const T& goal = goals.at(goal_idx);
    goal_node_indices.at(goal_idx) = AddNodeToRoadmap<T, GraphType>(
        goal, NNDistanceDirection::ROADMAP_TO_NEW_STATE, roadmap, distance_fn,
        edge_validity_check_fn, K, pre_goals_size, use_parallel,
        connection_is_symmetric, add_duplicate_states);
  }
  const auto astar_result =
      simple_graph_search::PerformAstarSearch<T, GraphType>(
          roadmap, start_node_indices, goal_node_indices, distance_fn,
          limit_astar_pqueue_duplicates);
  // Convert the solution path from A* provided as indices into real states
  return ExtractSolution<T, Container, GraphType>(roadmap, astar_result);
}

/// Find the best path from a start state to a goal state in a lazy manner, i.e.
/// the edge validity and distances of the graph are not trusted and instead the
/// provided @param edge_validity_check_fn and @param distance_fn are used. This
/// is used in "lazy-PRM" where collision checking is deferred from roadmap
/// construction to query time. Note that this can be much more expensive than
/// normal PRM queries, so it should only be used when the roadmap is frequently
/// changing or vastly larger than the region needed for most queries.
/// @param starts multiple start states.
/// @param goals multiple goal states.
/// @param roadmap existing roadmap.
/// @param distance_fn distance function for state-to-state distance. If
/// use_parallel is true, this must be thread-safe.
/// @param edge_validity_check_fn edge validity checking function. If
/// use_parallel is true, this must be thread-safe.
/// @param K number of K neighboring states in the roadmap to attempt to connect
/// the new state to.
/// @param use_parallel use parallel operations when possible.
/// @param connection_is_symmetric are distance and edge validity symmetric
/// (i.e. distance_fn(a, b) == distance_fn(b, a) and
/// edge_validity_check_fn(a, b) == edge_validity_check_fn(b, a))? Asymmetric
/// distance and edge validity functions are more expensive to work with and
/// should be avoided when possible.
/// @param add_duplicate_states should @param state be added to the roadmap if
/// it is a duplicate of an existing state? @param state is considered a
/// duplicate if one of the K neighboring states has distance zero from @param
/// state.
/// @param limit_astar_pqueue_duplicates use an additional map to limit the
/// duplicate states added to the A* pqueue?
/// @return path + length of the best path from a single start to the goal.
/// If no solution exists, path is empty and length is infinity.
template<typename T, typename Container, typename GraphType>
inline simple_astar_search::AstarResult<T, Container>
LazyQueryPath(
    const Container& starts,
    const Container& goals,
    const GraphType& roadmap,
    const std::function<double(const T&, const T&)>& distance_fn,
    const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
    const int64_t K,
    const bool use_parallel = true,
    const bool connection_is_symmetric = true,
    const bool add_duplicate_states = false,
    const bool limit_astar_pqueue_duplicates = true,
    const bool use_roadmap_overlay = true)
{
  if (use_roadmap_overlay)
  {
    using OverlaidType = simple_graph::NonOwningGraphOverlay<T, GraphType>;
    OverlaidType overlaid_roadmap(roadmap);
    return LazyQueryPathAndAddNodes<T, Container, OverlaidType>(
        starts, goals, overlaid_roadmap, distance_fn, edge_validity_check_fn, K,
        use_parallel, connection_is_symmetric, add_duplicate_states,
        limit_astar_pqueue_duplicates);
  }
  else
  {
    auto working_copy = roadmap;
    return LazyQueryPathAndAddNodes<T, Container, GraphType>(
       starts, goals, working_copy, distance_fn, edge_validity_check_fn, K,
       use_parallel, connection_is_symmetric, add_duplicate_states,
       limit_astar_pqueue_duplicates);
  }
}

/// Find the best path from one of a set of starting states to one of a set of
/// goal states.
/// @param starts multiple start states.
/// @param goals multiple goal states.
/// @param roadmap existing roadmap.
/// @param distance_fn distance function for state-to-state distance. If
/// use_parallel is true, this must be thread-safe.
/// @param edge_validity_check_fn edge validity checking function. If
/// use_parallel is true, this must be thread-safe.
/// @param K number of K neighboring states in the roadmap to attempt to connect
/// the new state to.
/// @param use_parallel use parallel operations when possible.
/// @param connection_is_symmetric are distance and edge validity symmetric
/// (i.e. distance_fn(a, b) == distance_fn(b, a) and
/// edge_validity_check_fn(a, b) == edge_validity_check_fn(b, a))? Asymmetric
/// distance and edge validity functions are more expensive to work with and
/// should be avoided when possible.
/// @param add_duplicate_states should @param state be added to the roadmap if
/// it is a duplicate of an existing state? @param state is considered a
/// duplicate if one of the K neighboring states has distance zero from @param
/// state.
/// @param limit_astar_pqueue_duplicates use an additional map to limit the
/// duplicate states added to the A* pqueue?
/// @return path + length of the best path from a single start to the goal.
/// If no solution exists, path is empty and length is infinity.
template<typename T, typename Container, typename GraphType>
inline simple_astar_search::AstarResult<T, Container>
QueryPath(
    const Container& starts,
    const Container& goals,
    const GraphType& roadmap,
    const std::function<double(const T&, const T&)>& distance_fn,
    const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
    const int64_t K,
    const bool use_parallel = true,
    const bool connection_is_symmetric = true,
    const bool add_duplicate_states = false,
    const bool limit_astar_pqueue_duplicates = true,
    const bool use_roadmap_overlay = true)
{
  if (use_roadmap_overlay)
  {
    using OverlaidType = simple_graph::NonOwningGraphOverlay<T, GraphType>;
    OverlaidType overlaid_roadmap(roadmap);
    return QueryPathAndAddNodes<T, Container, OverlaidType>(
        starts, goals, overlaid_roadmap, distance_fn, edge_validity_check_fn, K,
        use_parallel, connection_is_symmetric, add_duplicate_states,
        limit_astar_pqueue_duplicates);
  }
  else
  {
    auto working_copy = roadmap;
    return QueryPathAndAddNodes<T, Container, GraphType>(
        starts, goals, working_copy, distance_fn, edge_validity_check_fn, K,
        use_parallel, connection_is_symmetric, add_duplicate_states,
        limit_astar_pqueue_duplicates);
  }
}
}  // namespace simple_prm_planner
}  // namespace common_robotics_utilities
