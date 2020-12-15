#pragma once

#if defined(_OPENMP)
#include <omp.h>
#endif

#include <algorithm>
#include <cstdint>
#include <unordered_set>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/print.hpp>
#include <common_robotics_utilities/serialization.hpp>
#include <common_robotics_utilities/utility.hpp>

namespace common_robotics_utilities
{
namespace simple_graph
{
/// Directed weighted graph edge.
/// Edge stores from and to node indices into the graph.
class GraphEdge
{
private:
  int64_t from_index_ = -1;
  int64_t to_index_ = -1;
  double weight_ = 0.0;
  uint64_t scratchpad_ = 0;

public:

  static uint64_t Serialize(const GraphEdge& edge, std::vector<uint8_t>& buffer)
  {
    return edge.SerializeSelf(buffer);
  }

  static serialization::Deserialized<GraphEdge> Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
  {
    GraphEdge temp_edge;
    const uint64_t bytes_read
        = temp_edge.DeserializeSelf(buffer, starting_offset);
    return serialization::MakeDeserialized(temp_edge, bytes_read);
  }

  GraphEdge(
      const int64_t from_index, const int64_t to_index, const double weight,
      const uint64_t scratchpad)
      : from_index_(from_index), to_index_(to_index), weight_(weight),
        scratchpad_(scratchpad)
  {}

  GraphEdge(
      const int64_t from_index, const int64_t to_index, const double weight)
      : from_index_(from_index), to_index_(to_index), weight_(weight),
        scratchpad_(0)
  {}

  GraphEdge() : from_index_(-1), to_index_(-1), weight_(0.0), scratchpad_(0) {}

  uint64_t SerializeSelf(std::vector<uint8_t>& buffer) const
  {
    const uint64_t start_buffer_size = buffer.size();
    serialization::SerializeMemcpyable<int64_t>(from_index_, buffer);
    serialization::SerializeMemcpyable<int64_t>(to_index_, buffer);
    serialization::SerializeMemcpyable<double>(weight_, buffer);
    serialization::SerializeMemcpyable<uint64_t>(scratchpad_, buffer);
    // Figure out how many bytes were written
    const uint64_t end_buffer_size = buffer.size();
    const uint64_t bytes_written = end_buffer_size - start_buffer_size;
    return bytes_written;
  }

  uint64_t DeserializeSelf(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
  {
    uint64_t current_position = starting_offset;
    const auto deserialized_from_index
        = serialization::DeserializeMemcpyable<int64_t>(buffer,
                                                        current_position);
    from_index_ = deserialized_from_index.Value();
    current_position += deserialized_from_index.BytesRead();
    const auto deserialized_to_index
        = serialization::DeserializeMemcpyable<int64_t>(buffer,
                                                        current_position);
    to_index_ = deserialized_to_index.Value();
    current_position += deserialized_to_index.BytesRead();
    const auto deserialized_weight
        = serialization::DeserializeMemcpyable<double>(buffer,
                                                       current_position);
    weight_ = deserialized_weight.Value();
    current_position += deserialized_weight.BytesRead();
    const auto deserialized_scratchpad
        = serialization::DeserializeMemcpyable<uint64_t>(buffer,
                                                         current_position);
    scratchpad_ = deserialized_scratchpad.Value();
    current_position += deserialized_scratchpad.BytesRead();
    // Figure out how many bytes were read
    const uint64_t bytes_read = current_position - starting_offset;
    return bytes_read;
  }

  bool operator==(const GraphEdge& other) const
  {
    return (from_index_ == other.GetFromIndex()
            && to_index_ == other.GetToIndex()
            && weight_ == other.GetWeight()
            && scratchpad_ == other.GetScratchpad());
  }

  std::string Print() const
  {
    return "(" + std::to_string(from_index_) + "->" + std::to_string(to_index_)
           + ") : " + std::to_string(weight_) + " ["
           + std::to_string(scratchpad_) + "]";
  }

  int64_t GetFromIndex() const { return from_index_; }

  int64_t GetToIndex() const { return to_index_; }

  double GetWeight() const { return weight_; }

  uint64_t GetScratchpad() const { return scratchpad_; }

  void SetFromIndex(const int64_t new_from_index)
  {
    from_index_ = new_from_index;
  }

  void SetToIndex(const int64_t new_to_index) { to_index_ = new_to_index; }

  void SetWeight(const double new_weight) { weight_ = new_weight; }

  void SetScratchpad(const uint64_t new_scratchpad)
  {
    scratchpad_ = new_scratchpad;
  }
};

/// Node in a graph, which stores directed in and out edges as well as value.
template<typename NodeValueType>
class GraphNode
{
private:
  using GraphNodeType = GraphNode<NodeValueType>;

  NodeValueType value_;
  std::vector<GraphEdge> in_edges_;
  std::vector<GraphEdge> out_edges_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static uint64_t Serialize(
      const GraphNodeType& node, std::vector<uint8_t>& buffer,
      const serialization::Serializer<NodeValueType>& value_serializer)
  {
    return node.SerializeSelf(buffer, value_serializer);
  }

  static serialization::Deserialized<GraphNodeType> Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset,
      const serialization::Deserializer<NodeValueType>& value_deserializer)
  {
    GraphNodeType temp_node;
    const uint64_t bytes_read
        = temp_node.DeserializeSelf(buffer, starting_offset,
                                    value_deserializer);
    return serialization::MakeDeserialized(temp_node, bytes_read);
  }

  GraphNode(const NodeValueType& value,
            const std::vector<GraphEdge>& new_in_edges,
            const std::vector<GraphEdge>& new_out_edges)
      : value_(value), in_edges_(new_in_edges), out_edges_(new_out_edges) {}

  explicit GraphNode(const NodeValueType& value) : value_(value) {}

  GraphNode() {}

  uint64_t SerializeSelf(
      std::vector<uint8_t>& buffer,
      const serialization::Serializer<NodeValueType>& value_serializer) const
  {
    const uint64_t start_buffer_size = buffer.size();
    // Serialize the value
    value_serializer(value_, buffer);
    // Serialize the in edges
    serialization::SerializeVectorLike<GraphEdge>(
        in_edges_, buffer, GraphEdge::Serialize);
    // Serialize the out edges
    serialization::SerializeVectorLike<GraphEdge>(
        out_edges_, buffer, GraphEdge::Serialize);
    // Figure out how many bytes were written
    const uint64_t end_buffer_size = buffer.size();
    const uint64_t bytes_written = end_buffer_size - start_buffer_size;
    return bytes_written;
  }

  uint64_t DeserializeSelf(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset,
      const serialization::Deserializer<NodeValueType>& value_deserializer)
  {
    uint64_t current_position = starting_offset;
    // Deserialize the value
    const auto value_deserialized
        = value_deserializer(buffer, current_position);
    value_ = value_deserialized.Value();
    current_position += value_deserialized.BytesRead();
    // Deserialize the in edges
    const auto in_edges_deserialized
        = serialization::DeserializeVectorLike<GraphEdge>(
            buffer, current_position, GraphEdge::Deserialize);
    in_edges_ = in_edges_deserialized.Value();
    current_position += in_edges_deserialized.BytesRead();
    // Deserialize the out edges
    const auto out_edges_deserialized
        = serialization::DeserializeVectorLike<GraphEdge>(
            buffer, current_position, GraphEdge::Deserialize);
    out_edges_ = out_edges_deserialized.Value();
    current_position += out_edges_deserialized.BytesRead();
    // Figure out how many bytes were read
    const uint64_t bytes_read = current_position - starting_offset;
    return bytes_read;
  }

  std::string Print() const
  {
    std::ostringstream strm;
    strm << "Node : " << print::Print(value_) << " In Edges : [";
    if (in_edges_.size() > 0)
    {
      strm << in_edges_.at(0).Print();
      for (size_t idx = 1; idx < in_edges_.size(); idx++)
      {
        strm << ", " << in_edges_.at(idx).Print();
      }
    }
    strm << "] Out Edges : [";
    if (out_edges_.size() > 0)
    {
      strm << out_edges_.at(0).Print();
      for (size_t idx = 1; idx < out_edges_.size(); idx++)
      {
        strm << ", " << out_edges_.at(idx).Print();
      }
    }
    strm << "]";
    return strm.str();
  }

  const NodeValueType& GetValueImmutable() const { return value_; }

  NodeValueType& GetValueMutable() { return value_; }

  void AddInEdge(const GraphEdge& new_in_edge)
  {
    in_edges_.push_back(new_in_edge);
  }

  void AddOutEdge(const GraphEdge& new_out_edge)
  {
    out_edges_.push_back(new_out_edge);
  }

  void AddEdgePair(const GraphEdge& new_in_edge, const GraphEdge& new_out_edge)
  {
    AddInEdge(new_in_edge);
    AddOutEdge(new_out_edge);
  }

  const std::vector<GraphEdge>& GetInEdgesImmutable() const
  {
    return in_edges_;
  }

  std::vector<GraphEdge>& GetInEdgesMutable()
  {
    return in_edges_;
  }

  const std::vector<GraphEdge>& GetOutEdgesImmutable() const
  {
    return out_edges_;
  }

  std::vector<GraphEdge>& GetOutEdgesMutable()
  {
    return out_edges_;
  }

  void SetInEdges(const std::vector<GraphEdge>& new_in_edges)
  {
    in_edges_ = new_in_edges;
  }

  void SetOutEdges(const std::vector<GraphEdge>& new_out_edges)
  {
    out_edges_ = new_out_edges;
  }
};

template<typename NodeValueType>
using GraphNodeAllocator = Eigen::aligned_allocator<GraphNode<NodeValueType>>;

template<typename NodeValueType>
class Graph
{
private:
  using GraphType = Graph<NodeValueType>;
  using GraphNodeType = GraphNode<NodeValueType>;
  using GraphNodeAllocatorType = GraphNodeAllocator<NodeValueType>;
  using GraphNodeVector = std::vector<GraphNodeType, GraphNodeAllocatorType>;

  GraphNodeVector nodes_;

public:
  static uint64_t Serialize(
      const Graph<NodeValueType>& graph, std::vector<uint8_t>& buffer,
      const serialization::Serializer<NodeValueType>& value_serializer)
  {
    return graph.SerializeSelf(buffer, value_serializer);
  }

  static serialization::Deserialized<GraphType> Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset,
      const serialization::Deserializer<NodeValueType>& value_deserializer)
  {
    Graph<NodeValueType> temp_graph;
    const uint64_t bytes_read
        = temp_graph.DeserializeSelf(
            buffer, starting_offset, value_deserializer);
    return serialization::MakeDeserialized(temp_graph, bytes_read);
  }

  static GraphType PruneGraph(
      const GraphType& graph, const std::unordered_set<int64_t>& nodes_to_prune,
      const bool use_parallel)
  {
    // By making a sorted copy of the indices in nodes_to_prune, the two
    // for-loops that update edge to/from indices can terminate early once the
    // node_to_prune is greater than the original to/from index.
    std::vector<int64_t> vector_nodes_to_prune
        = utility::GetKeysFromSetLike<int64_t>(nodes_to_prune);
    std::sort(vector_nodes_to_prune.begin(), vector_nodes_to_prune.end());
    // Make the pruned graph, initialized with the number of nodes we expect it
    // to contain.
    GraphType pruned_graph(graph.Size() - nodes_to_prune.size());
    // First, serial pass through to copy nodes to be kept
    for (int64_t node_index = 0;
         node_index < static_cast<int64_t>(graph.Size());
         node_index++)
    {
      if (nodes_to_prune.count(node_index) == 0)
      {
        // Copy the node to the new graph
        pruned_graph.AddNode(graph.GetNodeImmutable(node_index));
      }
    }
    pruned_graph.ShrinkToFit();
    // Loop body for second pass to update edges for the kept nodes
    const auto update_edge_fn = [&] (const int64_t kept_node_index)
    {
      GraphNodeType& kept_graph_node
          = pruned_graph.GetNodeMutable(kept_node_index);
      // Make space for the updated in edges
      std::vector<GraphEdge> new_in_edges;
      new_in_edges.reserve(kept_graph_node.GetInEdgesImmutable().size());
      // Go through the in edges
      for (const auto& in_edge : kept_graph_node.GetInEdgesImmutable())
      {
        const int64_t original_from_index = in_edge.GetFromIndex();
        // Only update edges we keep
        if (nodes_to_prune.count(original_from_index) == 0)
        {
          // Make a copy of the existing edge
          GraphEdge new_in_edge = in_edge;
          // Update the "to index" to our node's index
          new_in_edge.SetToIndex(kept_node_index);
          // Update the "from index" to account for pruned nodes
          int64_t new_from_index = original_from_index;
          for (const int64_t index_to_prune : vector_nodes_to_prune)
          {
            // For each pruned node before the "from node", decrement the index
            if (index_to_prune < original_from_index)
            {
              new_from_index--;
            }
            // We can terminate early because we know the indices are sorted
            else if (index_to_prune > original_from_index)
            {
              break;
            }
            else
            {
              throw std::runtime_error(
                  "index_to_prune cannot equal original_from_index");
            }
          }
          new_in_edge.SetFromIndex(new_from_index);
          // Copy the new edge
          new_in_edges.push_back(new_in_edge);
        }
      }
      new_in_edges.shrink_to_fit();
      // Make space for the updated out edges
      std::vector<GraphEdge> new_out_edges;
      new_out_edges.reserve(kept_graph_node.GetOutEdgesImmutable().size());
      // Go through the out edges
      for (const auto& out_edge : kept_graph_node.GetOutEdgesImmutable())
      {
        const int64_t original_to_index = out_edge.GetToIndex();
        // Only update edges we keep
        if (nodes_to_prune.count(original_to_index) == 0)
        {
          // Make a copy of the existing edge
          GraphEdge new_out_edge = out_edge;
          // Update the "from index" to our node's index
          new_out_edge.SetFromIndex(kept_node_index);
          // Update the "from index" to account for pruned nodes
          int64_t new_to_index = original_to_index;
          for (const int64_t index_to_prune : vector_nodes_to_prune)
          {
            // For each pruned node before the "from node", decrement the index
            if (index_to_prune < original_to_index)
            {
              new_to_index--;
            }
            // We can terminate early because we know the indices are sorted
            else if (index_to_prune > original_to_index)
            {
              break;
            }
            else
            {
              throw std::runtime_error(
                  "index_to_prune cannot equal original_to_index");
            }
          }
          new_out_edge.SetToIndex(new_to_index);
          // Copy the new edge
          new_out_edges.push_back(new_out_edge);
        }
      }
      new_out_edges.shrink_to_fit();
      // Copy in the updated edges
      kept_graph_node.SetInEdges(new_in_edges);
      kept_graph_node.SetOutEdges(new_out_edges);
    };
    // Second, optionally parallel pass to update edges for the kept nodes
    if (use_parallel)
    {
#if defined(_OPENMP)
#pragma omp parallel for
#endif
      for (int64_t kept_node_index = 0;
          kept_node_index < static_cast<int64_t>(pruned_graph.Size());
          kept_node_index++)
      {
        update_edge_fn(kept_node_index);
      }
    }
    else
    {
      for (int64_t kept_node_index = 0;
          kept_node_index < static_cast<int64_t>(pruned_graph.Size());
          kept_node_index++)
      {
        update_edge_fn(kept_node_index);
      }
    }
    return pruned_graph;
  }

  Graph(const GraphNodeVector& nodes)
  {
    if (CheckGraphLinkage(nodes))
    {
      nodes_ = nodes;
    }
    else
    {
      throw std::invalid_argument("Invalid graph linkage");
    }
  }

  Graph(const size_t expected_size) { nodes_.reserve(expected_size); }

  Graph() {}

  uint64_t SerializeSelf(
      std::vector<uint8_t>& buffer,
      const serialization::Serializer<NodeValueType>& value_serializer) const
  {
    const uint64_t start_buffer_size = buffer.size();
    const serialization::Serializer<GraphNodeType> graph_node_serializer =
        [&] (const GraphNodeType& node,
             std::vector<uint8_t>& serialize_buffer)
    {
      return GraphNodeType::Serialize(
          node, serialize_buffer, value_serializer);
    };
    serialization::SerializeVectorLike<GraphNodeType, GraphNodeVector>(
        nodes_, buffer, graph_node_serializer);
    // Figure out how many bytes were written
    const uint64_t end_buffer_size = buffer.size();
    const uint64_t bytes_written = end_buffer_size - start_buffer_size;
    return bytes_written;
  }

  uint64_t DeserializeSelf(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset,
      const serialization::Deserializer<NodeValueType>& value_deserializer)
  {
    const serialization::Deserializer<GraphNodeType> graph_node_deserializer =
        [&] (const std::vector<uint8_t>& deserialize_buffer,
             const uint64_t offset)
    {
      return GraphNodeType::Deserialize(
          deserialize_buffer, offset, value_deserializer);
    };
    const auto deserialized_nodes
        = serialization::DeserializeVectorLike<GraphNodeType, GraphNodeVector>(
            buffer, starting_offset, graph_node_deserializer);
    if (CheckGraphLinkage(deserialized_nodes.Value()))
    {
      nodes_ = deserialized_nodes.Value();
    }
    else
    {
      throw std::invalid_argument("Deserialized invalid graph linkage");
    }
    return deserialized_nodes.BytesRead();
  }

  std::string Print() const
  {
    std::ostringstream strm;
    strm << "Graph - Nodes : ";
    if (nodes_.size() > 0)
    {
      strm << nodes_.at(0).Print();
      for (size_t idx = 1; idx < nodes_.size(); idx++)
      {
        strm << "\n" << nodes_.at(idx).Print();
      }
    }
    return strm.str();
  }

  GraphType MakePrunedCopy(
      const std::unordered_set<int64_t>& nodes_to_prune,
      const bool use_parallel) const
  {
    return PruneGraph(*this, nodes_to_prune, use_parallel);
  }

  void ShrinkToFit()
  {
    nodes_.shrink_to_fit();
  }

  size_t Size() const { return nodes_.size(); }

  bool IndexInRange(const int64_t index) const
  {
    if ((index >= 0) && (index < static_cast<int64_t>(nodes_.size())))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  bool CheckGraphLinkage() const
  {
    return CheckGraphLinkage(GetNodesImmutable());
  }

  static bool CheckGraphLinkage(const Graph<NodeValueType>& graph)
  {
    return CheckGraphLinkage(graph.GetNodesImmutable());
  }

  static bool CheckGraphLinkage(
      const GraphNodeVector& nodes)
  {
    // Go through every node and make sure the edges are valid
    for (size_t idx = 0; idx < nodes.size(); idx++)
    {
      const GraphNodeType& current_node = nodes.at(idx);
      // Check the in edges first
      const std::vector<GraphEdge>& in_edges
          = current_node.GetInEdgesImmutable();
      for (const GraphEdge& current_edge : in_edges)
      {
        // Check from index to make sure it's in bounds
        const int64_t from_index = current_edge.GetFromIndex();
        if (from_index < 0 || from_index >= static_cast<int64_t>(nodes.size()))
        {
          std::cerr << "From index out of bounds" << std::endl;
          return false;
        }
        // Check to index to make sure it matches our own index
        const int64_t to_index = current_edge.GetToIndex();
        if (to_index != static_cast<int64_t>(idx))
        {
          std::cerr << "To index does not point to current node" << std::endl;
          return false;
        }
        // Check edge validity (edges to ourself are not allowed)
        if (from_index == to_index)
        {
          std::cerr << "From index == to index not allowed" << std::endl;
          return false;
        }
        // Check to make sure that the from index node is linked to us
        const GraphNodeType& from_node
            = nodes.at(static_cast<size_t>(from_index));
        const std::vector<GraphEdge>& from_node_out_edges
            = from_node.GetOutEdgesImmutable();
        bool from_node_connection_valid = false;
        // Make sure at least one out edge of the from index node corresponds
        // to the current node
        for (const GraphEdge& current_from_node_out_edge : from_node_out_edges)
        {
          if (current_from_node_out_edge.GetToIndex()
              == static_cast<int64_t>(idx))
          {
            from_node_connection_valid = true;
          }
        }
        if (from_node_connection_valid == false)
        {
          std::cerr << "From index connection is invalid" << std::endl;
          return false;
        }
      }
      // Check the out edges second
      const std::vector<GraphEdge>& out_edges
          = current_node.GetOutEdgesImmutable();
      for (const GraphEdge& current_edge : out_edges)
      {
        // Check from index to make sure it matches our own index
        const int64_t from_index = current_edge.GetFromIndex();
        if (from_index != static_cast<int64_t>(idx))
        {
          std::cerr << "From index does not point to current node" << std::endl;
          return false;
        }
        // Check to index to make sure it's in bounds
        const int64_t to_index = current_edge.GetToIndex();
        if (to_index < 0 || to_index >= static_cast<int64_t>(nodes.size()))
        {
          std::cerr << "To index out of bounds" << std::endl;
          return false;
        }
        // Check edge validity (edges to ourself are not allowed)
        if (from_index == to_index)
        {
          std::cerr << "From index == to index not allowed" << std::endl;
          return false;
        }
        // Check to make sure that the to index node is linked to us
        const GraphNodeType& to_node
            = nodes.at(static_cast<size_t>(to_index));
        const std::vector<GraphEdge>& to_node_in_edges
            = to_node.GetInEdgesImmutable();
        bool to_node_connection_valid = false;
        // Make sure at least one in edge of the to index node corresponds to
        // the current node
        for (const GraphEdge& current_to_node_in_edge : to_node_in_edges)
        {
          if (current_to_node_in_edge.GetFromIndex()
              == static_cast<int64_t>(idx))
          {
            to_node_connection_valid = true;
          }
        }
        if (to_node_connection_valid == false)
        {
          std::cerr << "To index connection is invalid" << std::endl;
          return false;
        }
      }
    }
    return true;
  }

  const GraphNodeVector& GetNodesImmutable() const { return nodes_; }

  GraphNodeVector& GetNodesMutable() { return nodes_; }

  const GraphNodeType& GetNodeImmutable(const int64_t index) const
  {
    return nodes_.at(static_cast<size_t>(index));
  }

  GraphNodeType& GetNodeMutable(const int64_t index)
  {
    return nodes_.at(static_cast<size_t>(index));
  }

  int64_t AddNode(const GraphNodeType& new_node)
  {
    nodes_.push_back(new_node);
    return static_cast<int64_t>(nodes_.size() - 1);
  }

  int64_t AddNode(const NodeValueType& new_value)
  {
    nodes_.push_back(GraphNodeType(new_value));
    return static_cast<int64_t>(nodes_.size() - 1);
  }

  void AddEdgeBetweenNodes(const int64_t from_index, const int64_t to_index,
                           const double edge_weight)
  {
    // We retrieve the nodes first, since retrieval performs bounds checks first
    GraphNodeType& from_node = GetNodeMutable(from_index);
    GraphNodeType& to_node = GetNodeMutable(to_index);
    if (from_index == to_index)
    {
      throw std::invalid_argument(
          "Invalid circular edge from == to not allowed");
    }
    const GraphEdge new_edge(from_index, to_index, edge_weight);
    from_node.AddOutEdge(new_edge);
    to_node.AddInEdge(new_edge);
  }

  void AddEdgesBetweenNodes(const int64_t first_index,
                            const int64_t second_index,
                            const double edge_weight)
  {
    // We retrieve the nodes first, since retrieval performs bounds checks first
    GraphNodeType& first_node = GetNodeMutable(first_index);
    GraphNodeType& second_node = GetNodeMutable(second_index);
    if (first_index == second_index)
    {
      throw std::invalid_argument(
          "Invalid circular edge first == second not allowed");
    }
    const GraphEdge first_edge(first_index, second_index, edge_weight);
    first_node.AddOutEdge(first_edge);
    second_node.AddInEdge(first_edge);
    const GraphEdge second_edge(second_index, first_index, edge_weight);
    second_node.AddOutEdge(second_edge);
    first_node.AddInEdge(second_edge);
  }
};

inline std::ostream& operator<< (std::ostream& stream, const GraphEdge& edge)
{
  stream << edge.Print();
  return stream;
}

template<typename NodeValueType>
inline std::ostream& operator<< (
    std::ostream& stream, const GraphNode<NodeValueType>& node)
{
  stream << node.Print();
  return stream;
}

template<typename NodeValueType>
inline std::ostream& operator<< (
    std::ostream& stream, const Graph<NodeValueType>& graph)
{
  stream << graph.Print();
  return stream;
}
}  // namespace simple_graph
}  // namespace common_robotics_utilities

