#pragma once

#include <omp.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

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
/// @param use_parallel use parallel operations when possible.
/// @param distance_is_symmetric is the distance symmetric
/// (i.e. distance_fn(a, b) == distance_fn(b, a))? Asymmetric distance functions
/// are more expensive to work with and should be avoided when possible.
/// @param add_duplicate_states should @param state be added to the roadmap if
/// it is a duplicate of an existing state? @param state is considered a
/// duplicate if one of the K neighboring states has distance zero from @param
/// state.
/// @return the index of the newly-added node OR the index of an existing
/// duplicate node in the roadmap. You can check which happended by querying the
/// size of the roadmap before and after calling AddNodeToRoadmap.
template<typename T>
inline int64_t AddNodeToRoadmap(
    const T& state, const NNDistanceDirection nn_distance_direction,
    simple_graph::Graph<T>& roadmap,
    const std::function<double(const T&, const T&)>& distance_fn,
    const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
    const size_t K, const bool use_parallel = true,
    const bool distance_is_symmetric = true,
    const bool add_duplicate_states = false)
{
  // Make the node->graph or graph->node distance function as needed
  std::function<double(const simple_graph::GraphNode<T>&, const T&)>
      graph_distance_fn = nullptr;
  if (nn_distance_direction == NNDistanceDirection::ROADMAP_TO_NEW_STATE)
  {
    graph_distance_fn = [&] (const simple_graph::GraphNode<T>& node,
                             const T& query_state)
    {
      return distance_fn(node.GetValueImmutable(), query_state);
    };
  }
  else
  {
    graph_distance_fn = [&] (const simple_graph::GraphNode<T>& node,
                             const T& query_state)
    {
      return distance_fn(query_state, node.GetValueImmutable());
    };
  }
  // Call KNN with the distance function
  const std::vector<std::pair<int64_t, double>> nearest_neighbors =
      simple_knearest_neighbors::GetKNearestNeighbors(
          roadmap.GetNodesImmutable(), state, graph_distance_fn, K,
          use_parallel);
  // Check if we already have this state in the roadmap
  // (and we don't want to add duplicates)
  if (add_duplicate_states == false)
  {
    for (const auto& neighbor : nearest_neighbors)
    {
      if (neighbor.second == 0.0)
      {
        return neighbor.first;
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
  auto collision_check_and_distance_fn = [&] (const size_t idx)
  {
    const std::pair<int64_t, double>& nearest_neighbor = nearest_neighbors[idx];
    const int64_t nearest_neighbor_index = nearest_neighbor.first;
    const double graph_to_node_distance = nearest_neighbor.second;
    const T& nearest_neighbor_state
        = roadmap.GetNodeImmutable(nearest_neighbor_index).GetValueImmutable();
    const bool graph_to_node_edge_validity
        = edge_validity_check_fn(nearest_neighbor_state, state);
    if (graph_to_node_edge_validity && distance_is_symmetric)
    {
      // Distance is symmetric and the edge is valid
      nearest_neighbors_distances[idx]
          = std::make_pair(graph_to_node_distance,
                           graph_to_node_distance);
    }
    else if (!distance_is_symmetric)
    {
      const bool node_to_graph_edge_validity
          = edge_validity_check_fn(nearest_neighbor_state, state);
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
        nearest_neighbors_distances[idx]
            = std::make_pair(real_graph_to_node_distance,
                             real_node_to_graph_distance);
      }
      else
      {
        nearest_neighbors_distances[idx]
            = std::make_pair(real_node_to_graph_distance,
                             real_graph_to_node_distance);
      }
    }
    else
    {
      // Distance is symmetric, but the edge is not valid!
      nearest_neighbors_distances[idx] = std::make_pair(-1.0, -1.0);
    }
  };
  if (use_parallel)
  {
#if defined(_OPENMP)
#pragma omp parallel for
#endif
    for (size_t idx = 0; idx < nearest_neighbors.size(); idx++)
    {
      collision_check_and_distance_fn(idx);
    }
  }
  else
  {
    for (size_t idx = 0; idx < nearest_neighbors.size(); idx++)
    {
      collision_check_and_distance_fn(idx);
    }
  }
  // THIS MUST BE SERIAL - add edges to roadmap
  for (size_t idx = 0; idx < nearest_neighbors.size(); idx++)
  {
    const std::pair<int64_t, double>& nearest_neighbor = nearest_neighbors[idx];
    const int64_t nearest_neighbor_index = nearest_neighbor.first;
    const std::pair<double, double>& nearest_neighbor_distances
        = nearest_neighbors_distances[idx];
    if (nearest_neighbor_distances.first >= 0.0
        && nearest_neighbor_distances.second >= 0.0)
    {
      // Add the edges individually to allow for different "distances" in each
      // direction - for example, if the "distance" is a probability of the edge
      // being traversable, the probability may not be symmetric.
      roadmap.AddEdgeBetweenNodes(nearest_neighbor_index, new_node_index,
                                  nearest_neighbor_distances.first);
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
/// @param distance_is_symmetric is the distance symmetric
/// (i.e. distance_fn(a, b) == distance_fn(b, a))? Asymmetric distance functions
/// are more expensive to work with and should be avoided when possible.
/// @param add_duplicate_states should a new state be added to the roadmap if
/// it is a duplicate of an existing state? A new state is considered a
/// duplicate if one of the K neighboring states has distance zero from the
/// state.
/// @return statistics as a map<string, double> of useful statistics collected
/// while growing the roadmap.
template<typename T>
inline std::map<std::string, double> GrowRoadMap(
    simple_graph::Graph<T>& roadmap,
    const std::function<T(void)>& sampling_fn,
    const std::function<double(const T&, const T&)>& distance_fn,
    const std::function<bool(const T&)>& state_validity_check_fn,
    const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
    const std::function<bool(const int64_t)>& termination_check_fn,
    const size_t K,
    const bool use_parallel = true,
    const bool distance_is_symmetric = true,
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
  while (!termination_check_fn(static_cast<int64_t>(roadmap.Size())))
  {
    const T random_state = sampling_fn();
    statistics["total_samples"] += 1.0;
    if (state_validity_check_fn(random_state))
    {
      const size_t pre_size = roadmap.Size();
      AddNodeToRoadmap(random_state, NNDistanceDirection::ROADMAP_TO_NEW_STATE,
                       roadmap, distance_fn, edge_validity_check_fn, K,
                       use_parallel, distance_is_symmetric,
                       add_duplicate_states);
      const size_t post_size = roadmap.Size();
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

/// Update edge distances in a roadmap.
/// @param roadmap existing roadmap to update.
/// @param edge_validity_check_fn edge validity checking function. If
/// use_parallel is true, this must be thread-safe.
/// @param distance_fn distance function for state-to-state distance. If
/// use_parallel is true, this must be thread-safe.
/// @param use_parallel use parallel operations when possible.
template<typename T>
inline void UpdateRoadMapEdges(
    simple_graph::Graph<T>& roadmap,
    const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
    const std::function<double(const T&, const T&)>& distance_fn,
    const bool use_parallel = true)
{
  if (roadmap.CheckGraphLinkage() == false)
  {
    throw std::invalid_argument("Provided roadmap has invalid linkage");
  }
  // Because we don't need any special caching here, we define the loop contents
  // as a lambda and then call branch between parfor/for loops that call it.
  auto update_node_fn = [&] (const size_t current_node_index)
  {
    simple_graph::GraphNode<T>& current_node
        = roadmap.GetNodeMutable(current_node_index);
    std::vector<simple_graph::GraphEdge>& current_node_out_edges
        = current_node.GetOutEdgesMutable();
    for (auto& current_out_edge : current_node_out_edges)
    {
      const int64_t other_node_idx = current_out_edge.GetToIndex();
      simple_graph::GraphNode<T>& other_node
          = roadmap.GetNodeMutable(other_node_idx);
      std::vector<simple_graph::GraphEdge>& other_node_in_edges
          = other_node.GetInEdgesMutable();
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
        if (other_in_edge.GetFromIndex()
            == static_cast<int64_t>(current_node_index))
        {
          other_in_edge.SetWeight(updated_weight);
        }
      }
    }
  };
  if (use_parallel)
  {
#if defined(_OPENMP)
#pragma omp parallel for
#endif
    for (size_t current_node_index = 0; current_node_index < roadmap.Size();
         current_node_index++)
    {
      update_node_fn(current_node_index);
    }
  }
  else
  {
    for (size_t current_node_index = 0; current_node_index < roadmap.Size();
         current_node_index++)
    {
      update_node_fn(current_node_index);
    }
  }
}

/// Extracts the solution path from the roadmap.
/// @param roadmap roadmap used to find solution path.
/// @param solution_path_indices indices of the solution nodes in the graph, in
/// order of the solution path.
template<typename T, typename Allocator=std::allocator<T>>
inline std::vector<T, Allocator> ExtractSolutionPath(
    const simple_graph::Graph<T>& roadmap,
    const std::vector<int64_t>& solution_path_indices)
{
  std::vector<T, Allocator> solution_path;
  solution_path.reserve(solution_path_indices.size());
  for (const int64_t solution_path_index : solution_path_indices)
  {
    const T& solution_path_state
        = roadmap.GetNodeImmutable(solution_path_index).GetValueImmutable();
    solution_path.push_back(solution_path_state);
  }
  solution_path.shrink_to_fit();
  return solution_path;
}

/// Find the best path from one of a set of starting states to a single goal.
/// This implementation uses a much more expensive search method optimized for
/// large numbers of starting states. If the number of starting states is low,
/// you may be better off with multiple calls to
/// QueryPathAndAddNodesSingleStartSingleGoal instead.
/// Starting and goal states are added to the roadmap.
/// @param starts multiple start states.
/// @param goal goal state.
/// @param roadmap existing roadmap.
/// @param distance_fn distance function for state-to-state distance. If
/// use_parallel is true, this must be thread-safe.
/// @param edge_validity_check_fn edge validity checking function. If
/// use_parallel is true, this must be thread-safe.
/// @param K number of K neighboring states in the roadmap to attempt to connect
/// the new state to.
/// @param use_parallel use parallel operations when possible.
/// @param distance_is_symmetric is the distance symmetric
/// (i.e. distance_fn(a, b) == distance_fn(b, a))? Asymmetric distance functions
/// are more expensive to work with and should be avoided when possible.
/// @param add_duplicate_states should @param state be added to the roadmap if
/// it is a duplicate of an existing state? @param state is considered a
/// duplicate if one of the K neighboring states has distance zero from @param
/// state.
/// @return pair<path, length> of the best path from a single start to the goal.
/// If no solution exists, path is empty and length is infinity.
template<typename T, typename Allocator=std::allocator<T>>
inline std::pair<std::vector<T, Allocator>, double>
QueryPathAndAddNodesMultiStartSingleGoal(
    const std::vector<T, Allocator>& starts, const T& goal,
    simple_graph::Graph<T>& roadmap,
    const std::function<double(const T&, const T&)>& distance_fn,
    const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
    const size_t K, const bool use_parallel = true,
    const bool distance_is_symmetric = true,
    const bool add_duplicate_states = false)
{
  if (starts.empty())
  {
    throw std::runtime_error("starts is empty");
  }
  // Add the multiple start nodes to the roadmap
  std::vector<int64_t> start_node_indices(starts.size());
  for (size_t start_idx = 0; start_idx < starts.size(); start_idx++)
  {
    const T& start = starts[start_idx];
    start_node_indices[start_idx]
        = AddNodeToRoadmap<T>(start, NNDistanceDirection::NEW_STATE_TO_ROADMAP,
                              roadmap, distance_fn, edge_validity_check_fn, K,
                              use_parallel, distance_is_symmetric,
                              add_duplicate_states);
  }
  // Add the goal node to the roadmap
  const int64_t goal_node_index
      = AddNodeToRoadmap<T>(goal, NNDistanceDirection::ROADMAP_TO_NEW_STATE,
                            roadmap, distance_fn, edge_validity_check_fn, K,
                            use_parallel, distance_is_symmetric,
                            add_duplicate_states);
  // Call Dijkstra's
  const auto dijkstras_solution
      = simple_graph_search::PerformDijkstrasAlgorithm<T>(roadmap,
                                                          goal_node_index);
  // Identify the lowest-distance starting state
  double best_start_node_distance = std::numeric_limits<double>::infinity();
  int64_t best_start_node_index = -1;
  for (const int64_t start_node_index : start_node_indices)
  {
    const double start_node_distance
        = dijkstras_solution.GetNodeDistance(start_node_index);
    if (start_node_distance < best_start_node_distance)
    {
      best_start_node_distance = start_node_distance;
      best_start_node_index = start_node_index;
    }
  }
  const int64_t start_node_index = best_start_node_index;
  const double start_node_distance = best_start_node_distance;
  // Extract solution path
  if (std::isinf(start_node_distance))
  {
    return std::make_pair(std::vector<T, Allocator>(),
                          std::numeric_limits<double>::infinity());
  }
  else
  {
    std::vector<int64_t> solution_path_indices;
    solution_path_indices.push_back(start_node_index);
    int64_t previous_index
        = dijkstras_solution.GetPreviousIndex(start_node_index);
    while (previous_index >= 0)
    {
      const int64_t current_index = previous_index;
      solution_path_indices.push_back(current_index);
      if (current_index == goal_node_index)
      {
        break;
      }
      else
      {
        previous_index = dijkstras_solution.GetPreviousIndex(current_index);
      }
    }
    return std::make_pair(ExtractSolutionPath<T, Allocator>(
        roadmap, solution_path_indices), start_node_distance);
  }
}

/// Find the best path from one of a set of starting states to a single goal.
/// @param starts multiple start states.
/// @param goal goal state.
/// @param roadmap existing roadmap.
/// @param distance_fn distance function for state-to-state distance. If
/// use_parallel is true, this must be thread-safe.
/// @param edge_validity_check_fn edge validity checking function. If
/// use_parallel is true, this must be thread-safe.
/// @param K number of K neighboring states in the roadmap to attempt to connect
/// the new state to.
/// @param use_parallel use parallel operations when possible.
/// @param distance_is_symmetric is the distance symmetric
/// (i.e. distance_fn(a, b) == distance_fn(b, a))? Asymmetric distance functions
/// are more expensive to work with and should be avoided when possible.
/// @param add_duplicate_states should @param state be added to the roadmap if
/// it is a duplicate of an existing state? @param state is considered a
/// duplicate if one of the K neighboring states has distance zero from @param
/// state.
/// @return pair<path, length> of the best path from a single start to the goal.
/// If no solution exists, path is empty and length is infinity.
template<typename T, typename Allocator=std::allocator<T>>
inline std::pair<std::vector<T, Allocator>, double>
QueryPathMultiStartSingleGoal(
    const std::vector<T, Allocator>& starts, const T& goal,
    const simple_graph::Graph<T>& roadmap,
    const std::function<double(const T&, const T&)>& distance_fn,
    const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
    const size_t K, const bool use_parallel = true,
    const bool distance_is_symmetric = true,
    const bool add_duplicate_states = false)
{
  auto working_copy = roadmap;
  return QueryPathAndAddNodesMultiStartSingleGoal<T, Allocator>(
      starts, goal, working_copy, distance_fn, edge_validity_check_fn, K,
      use_parallel, distance_is_symmetric, add_duplicate_states);
}

/// Find the best path from start state to goal state.
/// Start and goal states are added to the roadmap.
/// @param start start state.
/// @param goal goal state.
/// @param roadmap existing roadmap.
/// @param distance_fn distance function for state-to-state distance. If
/// use_parallel is true, this must be thread-safe.
/// @param edge_validity_check_fn edge validity checking function. If
/// use_parallel is true, this must be thread-safe.
/// @param K number of K neighboring states in the roadmap to attempt to connect
/// the new state to.
/// @param use_parallel use parallel operations when possible.
/// @param distance_is_symmetric is the distance symmetric
/// (i.e. distance_fn(a, b) == distance_fn(b, a))? Asymmetric distance functions
/// are more expensive to work with and should be avoided when possible.
/// @param add_duplicate_states should @param state be added to the roadmap if
/// it is a duplicate of an existing state? @param state is considered a
/// duplicate if one of the K neighboring states has distance zero from @param
/// state.
/// @return pair<path, length> of the best path from start to goal.
/// If no solution exists, path is empty and length is infinity.
template<typename T, typename Allocator=std::allocator<T>>
inline std::pair<std::vector<T, Allocator>, double>
QueryPathAndAddNodesSingleStartSingleGoal(
    const T& start, const T& goal, simple_graph::Graph<T>& roadmap,
    const std::function<double(const T&, const T&)>& distance_fn,
    const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
    const size_t K, const bool use_parallel = true,
    const bool distance_is_symmetric = true,
    const bool add_duplicate_states = false,
    const bool limit_astar_pqueue_duplicates=true)
{
  // Add the start node to the roadmap
  const int64_t start_node_index
      = AddNodeToRoadmap<T>(start, NNDistanceDirection::NEW_STATE_TO_ROADMAP,
                            roadmap, distance_fn, edge_validity_check_fn, K,
                            use_parallel, distance_is_symmetric,
                            add_duplicate_states);
  // Add the goal node to the roadmap
  const int64_t goal_node_index
      = AddNodeToRoadmap<T>(goal, NNDistanceDirection::ROADMAP_TO_NEW_STATE,
                            roadmap, distance_fn, edge_validity_check_fn, K,
                            use_parallel, distance_is_symmetric,
                            add_duplicate_states);
  // Call graph A*
  const std::pair<std::vector<int64_t>, double> astar_result
      = simple_graph_search::PerformAstarSearch<T>(
          roadmap, start_node_index, goal_node_index, distance_fn,
          limit_astar_pqueue_duplicates);
  // Convert the solution path from A* provided as indices into real states
  return std::make_pair(ExtractSolutionPath<T, Allocator>(
      roadmap, astar_result.first), astar_result.second);
}

/// Find the best path from start state to goal state.
/// @param start start state.
/// @param goal goal state.
/// @param roadmap existing roadmap.
/// @param distance_fn distance function for state-to-state distance. If
/// use_parallel is true, this must be thread-safe.
/// @param edge_validity_check_fn edge validity checking function. If
/// use_parallel is true, this must be thread-safe.
/// @param K number of K neighboring states in the roadmap to attempt to connect
/// the new state to.
/// @param use_parallel use parallel operations when possible.
/// @param distance_is_symmetric is the distance symmetric
/// (i.e. distance_fn(a, b) == distance_fn(b, a))? Asymmetric distance functions
/// are more expensive to work with and should be avoided when possible.
/// @param add_duplicate_states should @param state be added to the roadmap if
/// it is a duplicate of an existing state? @param state is considered a
/// duplicate if one of the K neighboring states has distance zero from @param
/// state.
/// @return pair<path, length> of the best path from start to goal.
/// If no solution exists, path is empty and length is infinity.
template<typename T, typename Allocator=std::allocator<T>>
inline std::pair<std::vector<T, Allocator>, double>
QueryPathSingleStartSingleGoal(
    const T& start, const T& goal, const simple_graph::Graph<T>& roadmap,
    const std::function<double(const T&, const T&)>& distance_fn,
    const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
    const size_t K, const bool use_parallel = true,
    const bool distance_is_symmetric = true,
    const bool add_duplicate_states = false,
    const bool limit_astar_pqueue_duplicates=true)
{
  auto working_copy = roadmap;
  return QueryPathAndAddNodesSingleStartSingleGoal<T, Allocator>(
      start, goal, working_copy, distance_fn, edge_validity_check_fn, K,
      use_parallel, distance_is_symmetric, add_duplicate_states,
        limit_astar_pqueue_duplicates);
}

/// Find the best path from start state to goal state in a lazy manner, i.e.
/// the edge validity and distances of the graph are not trusted and instead the
/// provided @param edge_validity_check_fn and @param distance_fn are used. This
/// is used in "lazy-PRM" where collision checking is deferred from roadmap
/// construction to query time. Note that this can be much more expensive than
/// normal PRM queries, so it sohuld only be used when the roadmap is frequently
/// changing or vastly larger than the region needed for most queries.
/// Start and goal states are added to the roadmap.
/// @param start start state.
/// @param goal goal state.
/// @param roadmap existing roadmap.
/// @param distance_fn distance function for state-to-state distance. If
/// use_parallel is true, this must be thread-safe.
/// @param edge_validity_check_fn edge validity checking function. If
/// use_parallel is true, this must be thread-safe.
/// @param K number of K neighboring states in the roadmap to attempt to connect
/// the new state to.
/// @param use_parallel use parallel operations when possible.
/// @param distance_is_symmetric is the distance symmetric
/// (i.e. distance_fn(a, b) == distance_fn(b, a))? Asymmetric distance functions
/// are more expensive to work with and should be avoided when possible.
/// @param add_duplicate_states should @param state be added to the roadmap if
/// it is a duplicate of an existing state? @param state is considered a
/// duplicate if one of the K neighboring states has distance zero from @param
/// state.
/// @return pair<path, length> of the best path from start to goal.
/// If no solution exists, path is empty and length is infinity.
template<typename T, typename Allocator=std::allocator<T>>
inline std::pair<std::vector<T, Allocator>, double>
LazyQueryPathAndAddNodesSingleStartSingleGoal(
    const T& start, const T& goal, simple_graph::Graph<T>& roadmap,
    const std::function<double(const T&, const T&)>& distance_fn,
    const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
    const size_t K, const bool use_parallel = true,
    const bool distance_is_symmetric = true,
    const bool add_duplicate_states = false,
    const bool limit_astar_pqueue_duplicates=true)
{
  // Add the start node to the roadmap
  const int64_t start_node_index
      = AddNodeToRoadmap<T>(start, NNDistanceDirection::NEW_STATE_TO_ROADMAP,
                            roadmap, distance_fn, edge_validity_check_fn, K,
                            use_parallel, distance_is_symmetric,
                            add_duplicate_states);
  // Add the goal node to the roadmap
  const int64_t goal_node_index
      = AddNodeToRoadmap<T>(goal, NNDistanceDirection::ROADMAP_TO_NEW_STATE,
                            roadmap, distance_fn, edge_validity_check_fn, K,
                            use_parallel, distance_is_symmetric,
                            add_duplicate_states);
  // Call graph A*
  const std::pair<std::vector<int64_t>, double> astar_result
      = simple_graph_search::PerformLazyAstarSearch<T>(
          roadmap, start_node_index, goal_node_index, edge_validity_check_fn,
          distance_fn, distance_fn, limit_astar_pqueue_duplicates);
  // Convert the solution path from A* provided as indices into real states
  return std::make_pair(ExtractSolutionPath<T, Allocator>(
      roadmap, astar_result.first), astar_result.second);
}

/// Find the best path from start state to goal state in a lazy manner, i.e.
/// the edge validity and distances of the graph are not trusted and instead the
/// provided @param edge_validity_check_fn and @param distance_fn are used. This
/// is used in "lazy-PRM" where collision checking is deferred from roadmap
/// construction to query time. Note that this can be much more expensive than
/// normal PRM queries, so it sohuld only be used when the roadmap is frequently
/// changing or vastly larger than the region needed for most queries.
/// @param start start state.
/// @param goal goal state.
/// @param roadmap existing roadmap.
/// @param distance_fn distance function for state-to-state distance. If
/// use_parallel is true, this must be thread-safe.
/// @param edge_validity_check_fn edge validity checking function. If
/// use_parallel is true, this must be thread-safe.
/// @param K number of K neighboring states in the roadmap to attempt to connect
/// the new state to.
/// @param use_parallel use parallel operations when possible.
/// @param distance_is_symmetric is the distance symmetric
/// (i.e. distance_fn(a, b) == distance_fn(b, a))? Asymmetric distance functions
/// are more expensive to work with and should be avoided when possible.
/// @param add_duplicate_states should @param state be added to the roadmap if
/// it is a duplicate of an existing state? @param state is considered a
/// duplicate if one of the K neighboring states has distance zero from @param
/// state.
/// @return pair<path, length> of the best path from start to goal.
/// If no solution exists, path is empty and length is infinity.
template<typename T, typename Allocator=std::allocator<T>>
inline std::pair<std::vector<T, Allocator>, double>
LazyQueryPathSingleStartSingleGoal(
    const T& start, const T& goal, const simple_graph::Graph<T>& roadmap,
    const std::function<double(const T&, const T&)>& distance_fn,
    const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
    const size_t K, const bool use_parallel = true,
    const bool distance_is_symmetric = true,
    const bool add_duplicate_states = false,
    const bool limit_astar_pqueue_duplicates=true)
{
  auto working_copy = roadmap;
  return LazyQueryPathAndAddNodesSingleStartSingleGoal<T, Allocator>(
      start, goal, working_copy, distance_fn, edge_validity_check_fn, K,
      use_parallel, distance_is_symmetric, add_duplicate_states,
        limit_astar_pqueue_duplicates);
}

/// Find the best path from one of a set of starting states to one of a set of
/// goal states. This is the most expensive search possible, and should only be
/// used if there are a large number of starting states. If @param
/// distance_is_symmetric is true, you can swap starts and goals to ensure that
/// |starts| > |goals|.
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
/// @param distance_is_symmetric is the distance symmetric
/// (i.e. distance_fn(a, b) == distance_fn(b, a))? Asymmetric distance functions
/// are more expensive to work with and should be avoided when possible.
/// @param add_duplicate_states should @param state be added to the roadmap if
/// it is a duplicate of an existing state? @param state is considered a
/// duplicate if one of the K neighboring states has distance zero from @param
/// state.
/// @return pair<path, length> of the best path from a single start to a single
/// goal. If no solution exists, path is empty and length is infinity.
template<typename T, typename Allocator=std::allocator<T>>
inline std::pair<std::vector<T, Allocator>, double>
QueryPathMultiStartMultiGoal(
    const std::vector<T, Allocator>& starts,
    const std::vector<T, Allocator>& goals,
    const simple_graph::Graph<T>& roadmap,
    const std::function<double(const T&, const T&)>& distance_fn,
    const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
    const size_t K, const bool use_parallel = true,
    const bool distance_is_symmetric = true,
    const bool add_duplicate_states = false)
{
  std::vector<std::pair<std::vector<T, Allocator>, double>>
      possible_solutions(goals.size());
  for (size_t goal_idx = 0; goal_idx < goals.size(); goal_idx++)
  {
    possible_solutions.at(goal_idx)
        = QueryPathMultiStartSingleGoal<T, Allocator>(
            starts, goals.at(goal_idx), roadmap, distance_fn,
            edge_validity_check_fn, K, use_parallel, distance_is_symmetric,
            add_duplicate_states);
  }
  double best_solution_distance = std::numeric_limits<double>::infinity();
  int64_t best_solution_index = -1;
  for (size_t goal_idx = 0; goal_idx < goals.size(); goal_idx++)
  {
    const double solution_distance = possible_solutions[goal_idx].second;
    if (solution_distance < best_solution_distance)
    {
      best_solution_distance = solution_distance;
      best_solution_index = goal_idx;
    }
  }
  if ((best_solution_index >= 0)
      && (best_solution_distance < std::numeric_limits<double>::infinity()))
  {
    return possible_solutions[best_solution_index];
  }
  else
  {
    return std::make_pair(std::vector<T, Allocator>(),
                          std::numeric_limits<double>::infinity());
  }
}
}  // namespace simple_prm_planner
}  // namespace common_robotics_utilities
