#pragma once

#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <common_robotics_utilities/simple_astar_search.hpp>
#include <common_robotics_utilities/simple_graph.hpp>
#include <common_robotics_utilities/utility.hpp>

namespace common_robotics_utilities
{
namespace simple_graph_search
{
/// Wrapper class to store the results of performing Dijkstra's search on a
/// graph: the "previous index map" which returns the previous index (leading to
/// the origin/goal of the search) for the current index, and the "node
/// distance" the distance from the current index to the origin/goal of the
/// search.
class DijkstrasResult
{
private:
  std::vector<int64_t> previous_index_map_;
  std::vector<double> node_distances_;

public:
  static uint64_t Serialize(const DijkstrasResult& result,
                            std::vector<uint8_t>& buffer)
  {
    return result.SerializeSelf(buffer);
  }

  static serialization::Deserialized<DijkstrasResult> Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
  {
    DijkstrasResult temp_result;
    const uint64_t bytes_read
        = temp_result.DeserializeSelf(buffer, starting_offset);
    return serialization::MakeDeserialized(temp_result, bytes_read);
  }

  DijkstrasResult() {}

  DijkstrasResult(const std::vector<int64_t>& previous_index_map,
                  const std::vector<double>& node_distances)
      : previous_index_map_(previous_index_map), node_distances_(node_distances)
  {
    if (previous_index_map_.size() != node_distances_.size())
    {
      throw std::invalid_argument(
          "previous_index_map.size() != node_distances.size()");
    }
  }

  uint64_t SerializeSelf(std::vector<uint8_t>& buffer) const
  {
    const uint64_t start_buffer_size = buffer.size();
    // Serialize the previous index map
    serialization::SerializeMemcpyableVectorLike<int64_t>(
        previous_index_map_, buffer);
    // Serialize the node distances
    serialization::SerializeMemcpyableVectorLike<double>(
        node_distances_, buffer);
    // Figure out how many bytes were written
    const uint64_t end_buffer_size = buffer.size();
    const uint64_t bytes_written = end_buffer_size - start_buffer_size;
    return bytes_written;
  }

  uint64_t DeserializeSelf(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
  {
    uint64_t current_position = starting_offset;
    // Deserialize the previous index map
    const auto previous_index_map_deserialized
        = serialization::DeserializeMemcpyableVectorLike<int64_t>(
            buffer, current_position);
    previous_index_map_ = previous_index_map_deserialized.Value();
    current_position += previous_index_map_deserialized.BytesRead();
    // Deserialize the node distances
    const auto node_distances_deserialized
        = serialization::DeserializeMemcpyableVectorLike<double>(
            buffer, current_position);
    node_distances_ = node_distances_deserialized.Value();
    current_position += node_distances_deserialized.BytesRead();
    if (previous_index_map_.size() != node_distances_.size())
    {
      throw std::invalid_argument(
          "Deserialized previous_index_map.size() != node_distances.size()");
    }
    // Figure out how many bytes were read
    const uint64_t bytes_read = current_position - starting_offset;
    return bytes_read;
  }

  size_t Size() const { return previous_index_map_.size(); }

  int64_t GetPreviousIndex(const int64_t node_index) const
  {
    return previous_index_map_.at(static_cast<size_t>(node_index));
  }

  double GetNodeDistance(const int64_t node_index) const
  {
    return node_distances_.at(static_cast<size_t>(node_index));
  }
};

class IndexAndDistance
{
private:
  int64_t index_ = -1;
  double distance_ = std::numeric_limits<double>::infinity();

public:
  IndexAndDistance() {}

  IndexAndDistance(const int64_t index, const double distance)
      : index_(index), distance_(distance) {}

  int64_t Index() const { return index_; }

  double Distance() const { return distance_; }
};

class CompareIndexDistancePairFn
{
public:
  bool operator()(const IndexAndDistance& lhs,
                  const IndexAndDistance& rhs) const
  {
    return lhs.Distance() > rhs.Distance();
  }
};

template<typename NodeValueType>
inline DijkstrasResult PerformDijkstrasAlgorithm(
    const simple_graph::Graph<NodeValueType>& graph, const int64_t start_index)
{
  if (!graph.IndexInRange(start_index))
  {
    throw std::invalid_argument("Start index out of range");
  }
  // Setup
  std::vector<int64_t> previous_index_map(graph.Size(), -1);
  std::vector<double> distances(
      graph.Size(), std::numeric_limits<double>::infinity());
  std::priority_queue<IndexAndDistance,
                      std::vector<IndexAndDistance>,
                      CompareIndexDistancePairFn> queue;
  std::unordered_set<int64_t> explored(graph.Size());
  previous_index_map.at(static_cast<size_t>(start_index)) = start_index;
  distances.at(static_cast<size_t>(start_index)) = 0.0;
  queue.push(IndexAndDistance(start_index, 0.0));
  while (queue.size() > 0)
  {
    const IndexAndDistance top_node = queue.top();
    queue.pop();
    if (explored.count(top_node.Index()) > 0)
    {
      // We've already been here
      continue;
    }
    else
    {
      // Note that we've been here
      explored.insert(top_node.Index());
      // Get our neighbors
      const std::vector<simple_graph::GraphEdge>& neighbor_edges
          = graph.GetNodeImmutable(top_node.Index()).GetInEdgesImmutable();
      // Go through our neighbors
      for (const simple_graph::GraphEdge& neighbor_edge : neighbor_edges)
      {
        const int64_t neighbor_index = neighbor_edge.GetFromIndex();
        const double neighbor_edge_weight = neighbor_edge.GetWeight();
        const double new_neighbor_distance
            = top_node.Distance() + neighbor_edge_weight;
        // Check against the neighbor
        const double stored_neighbor_distance
            = distances.at(static_cast<size_t>(neighbor_index));
        if (new_neighbor_distance < stored_neighbor_distance)
        {
          // We've found a better way to get to this node
          // Check if it's already been explored
          if (explored.count(neighbor_index) == 0)
          {
            // If it hasn't been explored, we need to add it to the queue
            // even though it may already be in the queue, we have found it with
            // a lower distance.
            queue.push(IndexAndDistance(neighbor_index, new_neighbor_distance));
          }
          // Update that we're the best previous node
          previous_index_map.at(static_cast<size_t>(neighbor_index))
              = top_node.Index();
          distances.at(static_cast<size_t>(neighbor_index))
              = new_neighbor_distance;
        }
        else
        {
          // Do nothing
          continue;
        }
      }
    }
  }
  return DijkstrasResult(previous_index_map, distances);
}

template<typename NodeValueType>
inline simple_astar_search::AstarIndexResult PerformLazyAstarSearch(
    const simple_graph::Graph<NodeValueType>& graph,
    const std::map<int64_t, double>& start_indices,
    const std::function<bool(
        const simple_graph::Graph<NodeValueType>&,
        const int64_t)>& goal_check_fn,
    const std::function<bool(
        const simple_graph::Graph<NodeValueType>&,
        const simple_graph::GraphEdge&)>& edge_validity_check_fn,
    const std::function<double(
        const simple_graph::Graph<NodeValueType>&,
        const simple_graph::GraphEdge&)>& distance_fn,
    const std::function<double(
        const simple_graph::Graph<NodeValueType>&,
        const int64_t)>& heuristic_fn,
    const bool limit_pqueue_duplicates)
{
  // Enforced sanity checks
  int64_t best_start_meeting_goal_index = -1;
  double best_start_meeting_goal_cost = std::numeric_limits<double>::infinity();
  for (const auto& start_index_and_cost : start_indices)
  {
    const int64_t start_index = start_index_and_cost.first;
    const double start_cost = start_index_and_cost.second;
    if (!graph.IndexInRange(start_index))
    {
      throw std::invalid_argument("Start index out of range");
    }
    if (goal_check_fn(graph, start_index))
    {
      if (start_cost < best_start_meeting_goal_cost)
      {
        best_start_meeting_goal_index = start_index;
        best_start_meeting_goal_cost = start_cost;
      }
    }
  }
  if (best_start_meeting_goal_index >= 0)
  {
    return simple_astar_search::AstarIndexResult(
        {best_start_meeting_goal_index}, best_start_meeting_goal_cost);
  }
  // Setup
  std::priority_queue<simple_astar_search::AstarPQueueElement,
                      std::vector<simple_astar_search::AstarPQueueElement>,
                      simple_astar_search::CompareAstarPQueueElementFn> queue;
  // Optional map to reduce the number of duplicate items added to the pqueue
  // Key is the node index in the provided graph
  // Value is cost-to-come
  std::unordered_map<int64_t, double> queue_members_map;
  // Key is the node index in the provided graph
  // Value is backpointer + cost-to-come
  // backpointer is the parent index in the provided graph
  simple_astar_search::ExploredList explored;
  // Initialize
  for (const auto& start_index_and_cost : start_indices)
  {
    const int64_t start_index = start_index_and_cost.first;
    const double start_cost = start_index_and_cost.second;
    queue.push(simple_astar_search::AstarPQueueElement(
        start_index, start_cost, heuristic_fn(graph, start_index)));
    if (limit_pqueue_duplicates)
    {
      queue_members_map[start_index] = start_cost;
    }
  }
  // Storage for the goal state (once found)
  OwningMaybe<int64_t> goal_index;
  // Search
  while (queue.size() > 0)
  {
    // Get the top of the priority queue
    const simple_astar_search::AstarPQueueElement top_node = queue.top();
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
    const bool explored_node_is_better
        = (node_in_explored)
          ? (top_node.CostToCome()
              >= node_explored_find_itr->second.CostToCome())
          : false;
    if (!explored_node_is_better)
    {
      // Add to the explored list
      explored[top_node.NodeID()]
          = simple_astar_search::BackPointerAndCostToCome(top_node);
      // Check if we have reached the goal
      if (goal_check_fn(graph, top_node.NodeID()))
      {
        goal_index = OwningMaybe<int64_t>(top_node.NodeID());
        break;
      }
      // Explore and add the children
      const std::vector<simple_graph::GraphEdge>& out_edges
          = graph.GetNodeImmutable(top_node.NodeID()).GetOutEdgesImmutable();
      for (const auto& current_out_edge : out_edges)
      {
        // Get the next potential child node
        const int64_t child_node_index = current_out_edge.GetToIndex();
        // Check if the top node->child edge is valid
        if (edge_validity_check_fn(graph, current_out_edge))
        {
          // Compute the cost-to-come for the new child
          const double parent_cost_to_come = top_node.CostToCome();
          const double parent_to_child_cost
              = distance_fn(graph, current_out_edge);
          const double child_cost_to_come
              = parent_cost_to_come + parent_to_child_cost;
          // Check if the child state has already been explored
          const auto child_explored_find_itr
              = explored.find(child_node_index);
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
                = queue_members_map.find(child_node_index);
            const bool in_queue
                = (queue_members_map_itr != queue_members_map.end());
            queue_is_better =
                (in_queue)
                ? (child_cost_to_come >= queue_members_map_itr->second)
                : false;
          }
          // Only add the new state if we need to
          if (!explored_child_is_better && !queue_is_better)
          {
            // Compute the heuristic for the child
            const double child_heuristic =
                heuristic_fn(graph, child_node_index);
            // Compute the child value
            const double child_value = child_cost_to_come + child_heuristic;
            // Push onto the pqueue
            queue.push(simple_astar_search::AstarPQueueElement(
                child_node_index, top_node.NodeID(), child_cost_to_come,
                child_value));
            // Add to the queue member map
            if (limit_pqueue_duplicates)
            {
              queue_members_map[child_node_index] = child_cost_to_come;
            }
          }
        }
      }
    }
  }
  if (goal_index)
  {
    return simple_astar_search::ExtractAstarResult(
        explored, goal_index.Value());
  }
  else
  {
    return simple_astar_search::AstarIndexResult();
  }
}

template<typename NodeValueType>
inline simple_astar_search::AstarIndexResult PerformLazyAstarSearch(
    const simple_graph::Graph<NodeValueType>& graph,
    const std::vector<int64_t>& start_indices,
    const std::vector<int64_t>& goal_indices,
    const std::function<bool(
        const simple_graph::Graph<NodeValueType>&,
        const simple_graph::GraphEdge&)>& edge_validity_check_fn,
    const std::function<double(
        const simple_graph::Graph<NodeValueType>&,
        const simple_graph::GraphEdge&)>& distance_fn,
    const std::function<double(const simple_graph::Graph<NodeValueType>&,
                               const int64_t, const int64_t)>& heuristic_fn,
    const bool limit_pqueue_duplicates)
{
  // Enforced sanity checks
  for (const int64_t start_index : start_indices)
  {
    if (!graph.IndexInRange(start_index))
    {
      throw std::invalid_argument("Start index out of range");
    }
  }
  for (const int64_t goal_index : goal_indices)
  {
    if (!graph.IndexInRange(goal_index))
    {
      throw std::invalid_argument("Goal index out of range");
    }
  }
  // Make goal check function
  const auto goal_check_function = [&] (
      const simple_graph::Graph<NodeValueType>& search_graph,
      const int64_t node_index)
  {
    UNUSED(search_graph);
    for (const int64_t goal_index : goal_indices)
    {
      if (node_index == goal_index)
      {
        return true;
      }
    }
    return false;
  };
  // Make heuristic helper function
  const auto heuristic_function = [&] (
      const simple_graph::Graph<NodeValueType>& search_graph,
      const int64_t node_index)
  {
    double best_heuristic_value = std::numeric_limits<double>::infinity();
    for (const int64_t goal_index : goal_indices)
    {
      const double heuristic_value =
          heuristic_fn(search_graph, node_index, goal_index);
      if (heuristic_value < best_heuristic_value)
      {
        best_heuristic_value = heuristic_value;
      }
    }
    return best_heuristic_value;
  };
  std::map<int64_t, double> start_indices_map;
  for (const int64_t start_index : start_indices)
  {
    start_indices_map[start_index] = 0.0;
  }
  return PerformLazyAstarSearch<NodeValueType>(
      graph, start_indices_map, goal_check_function, edge_validity_check_fn,
      distance_fn, heuristic_function, limit_pqueue_duplicates);
}

template<typename NodeValueType>
inline simple_astar_search::AstarIndexResult PerformLazyAstarSearch(
    const simple_graph::Graph<NodeValueType>& graph,
    const std::vector<int64_t>& start_indices,
    const std::vector<int64_t>& goal_indices,
    const std::function<bool(const NodeValueType&,
                             const NodeValueType&)>& edge_validity_check_fn,
    const std::function<double(const NodeValueType&,
                               const NodeValueType&)>& distance_fn,
    const std::function<double(const NodeValueType&,
                               const NodeValueType&)>& heuristic_fn,
    const bool limit_pqueue_duplicates)
{
  // Wrap the helper functions
  const auto edge_validity_check_function
      = [&] (const simple_graph::Graph<NodeValueType>& search_graph,
             const simple_graph::GraphEdge& edge)
  {
    return edge_validity_check_fn(
        search_graph.GetNodeImmutable(edge.GetFromIndex()).GetValueImmutable(),
        search_graph.GetNodeImmutable(edge.GetToIndex()).GetValueImmutable());
  };
  const auto distance_function
      = [&] (const simple_graph::Graph<NodeValueType>& search_graph,
             const simple_graph::GraphEdge& edge)
  {
    return distance_fn(
        search_graph.GetNodeImmutable(edge.GetFromIndex()).GetValueImmutable(),
        search_graph.GetNodeImmutable(edge.GetToIndex()).GetValueImmutable());
  };
  const auto heuristic_function
      = [&] (const simple_graph::Graph<NodeValueType>& search_graph,
             const int64_t from_index, const int64_t to_index)
  {
    return heuristic_fn(
        search_graph.GetNodeImmutable(from_index).GetValueImmutable(),
        search_graph.GetNodeImmutable(to_index).GetValueImmutable());
  };
  return PerformLazyAstarSearch<NodeValueType>(
      graph, start_indices, goal_indices, edge_validity_check_function,
      distance_function, heuristic_function, limit_pqueue_duplicates);
}

template<typename NodeValueType>
inline simple_astar_search::AstarIndexResult PerformAstarSearch(
    const simple_graph::Graph<NodeValueType>& graph,
    const std::vector<int64_t>& start_indices,
    const std::vector<int64_t>& goal_indices,
    const std::function<double(const NodeValueType&,
                               const NodeValueType&)>& heuristic_fn,
    const bool limit_pqueue_duplicates)
{
  const auto edge_validity_check_function
      = [&] (const simple_graph::Graph<NodeValueType>& search_graph,
             const simple_graph::GraphEdge& edge)
  {
    UNUSED(search_graph);
    if (edge.GetWeight() < std::numeric_limits<double>::infinity())
    {
      return true;
    }
    else
    {
      return false;
    }
  };
  const auto distance_function
      = [&] (const simple_graph::Graph<NodeValueType>& search_graph,
             const simple_graph::GraphEdge& edge)
  {
    UNUSED(search_graph);
    return edge.GetWeight();
  };
  const auto heuristic_function
      = [&] (const simple_graph::Graph<NodeValueType>& search_graph,
             const int64_t from_index, const int64_t to_index)
  {
    return heuristic_fn(
        search_graph.GetNodeImmutable(from_index).GetValueImmutable(),
        search_graph.GetNodeImmutable(to_index).GetValueImmutable());
  };
  return PerformLazyAstarSearch<NodeValueType>(
      graph, start_indices, goal_indices, edge_validity_check_function,
      distance_function, heuristic_function, limit_pqueue_duplicates);
}
}  // namespace simple_graph_search
}  // namespace common_robotics_utilities
