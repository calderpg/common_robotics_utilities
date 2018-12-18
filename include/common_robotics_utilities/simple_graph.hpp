#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/print.hpp>
#include <common_robotics_utilities/serialization.hpp>

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

  static std::pair<GraphEdge, uint64_t> Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
  {
    GraphEdge temp_edge;
    const uint64_t bytes_read
        = temp_edge.DeserializeSelf(buffer, starting_offset);
    return std::make_pair(temp_edge, bytes_read);
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
    const std::pair<int64_t, uint64_t> deserialized_from_index
        = serialization::DeserializeMemcpyable<int64_t>(buffer,
                                                        current_position);
    from_index_ = deserialized_from_index.first;
    current_position += deserialized_from_index.second;
    const std::pair<int64_t, uint64_t> deserialized_to_index
        = serialization::DeserializeMemcpyable<int64_t>(buffer,
                                                        current_position);
    to_index_ = deserialized_to_index.first;
    current_position += deserialized_to_index.second;
    const std::pair<double, uint64_t> deserialized_weight
        = serialization::DeserializeMemcpyable<double>(buffer,
                                                       current_position);
    weight_ = deserialized_weight.first;
    current_position += deserialized_weight.second;
    const std::pair<uint64_t, uint64_t> deserialized_scratchpad
        = serialization::DeserializeMemcpyable<uint64_t>(buffer,
                                                         current_position);
    scratchpad_ = deserialized_scratchpad.first;
    current_position += deserialized_scratchpad.second;
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
  NodeValueType value_;
  std::vector<GraphEdge> in_edges_;
  std::vector<GraphEdge> out_edges_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static uint64_t Serialize(
      const GraphNode<NodeValueType>& node, std::vector<uint8_t>& buffer,
      const std::function<uint64_t(
          const NodeValueType&, std::vector<uint8_t>&)>& value_serializer)
  {
    return node.SerializeSelf(buffer, value_serializer);
  }

  static std::pair<GraphNode<NodeValueType>, uint64_t> Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset,
      const std::function<std::pair<NodeValueType, uint64_t>(
          const std::vector<uint8_t>&, const uint64_t)>& value_deserializer)
  {
    GraphNode<NodeValueType> temp_node;
    const uint64_t bytes_read
        = temp_node.DeserializeSelf(buffer, starting_offset,
                                    value_deserializer);
    return std::make_pair(temp_node, bytes_read);
  }

  GraphNode(const NodeValueType& value,
            const std::vector<GraphEdge>& new_in_edges,
            const std::vector<GraphEdge>& new_out_edges)
      : value_(value), in_edges_(new_in_edges), out_edges_(new_out_edges) {}

  explicit GraphNode(const NodeValueType& value) : value_(value) {}

  GraphNode() {}

  uint64_t SerializeSelf(
      std::vector<uint8_t>& buffer,
      const std::function<uint64_t(
          const NodeValueType&, std::vector<uint8_t>&)>& value_serializer) const
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
      const std::function<std::pair<NodeValueType, uint64_t>(
          const std::vector<uint8_t>&, const uint64_t)>& value_deserializer)
  {
    uint64_t current_position = starting_offset;
    // Deserialize the value
    const std::pair<NodeValueType, uint64_t> value_deserialized
        = value_deserializer(buffer, current_position);
    value_ = value_deserialized.first;
    current_position += value_deserialized.second;
    // Deserialize the in edges
    const std::pair<std::vector<GraphEdge>, uint64_t> in_edges_deserialized
        = serialization::DeserializeVectorLike<GraphEdge>(
            buffer, current_position, GraphEdge::Deserialize);
    in_edges_ = in_edges_deserialized.first;
    current_position += in_edges_deserialized.second;
    // Deserialize the out edges
    const std::pair<std::vector<GraphEdge>, uint64_t> out_edges_deserialized
        = serialization::DeserializeVectorLike<GraphEdge>(
            buffer, current_position, GraphEdge::Deserialize);
    out_edges_ = out_edges_deserialized.first;
    current_position += out_edges_deserialized.second;
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
      strm << in_edges_[0].Print();
      for (size_t idx = 1; idx < in_edges_.size(); idx++)
      {
        strm << ", " << in_edges_[idx].Print();
      }
    }
    strm << "] Out Edges : [";
    if (out_edges_.size() > 0)
    {
      strm << out_edges_[0].Print();
      for (size_t idx = 1; idx < out_edges_.size(); idx++)
      {
        strm << ", " << out_edges_[idx].Print();
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
class Graph
{
private:
  std::vector<GraphNode<NodeValueType>> nodes_;

public:
  static uint64_t Serialize(
      const Graph<NodeValueType>& graph, std::vector<uint8_t>& buffer,
      const std::function<uint64_t(
          const NodeValueType&, std::vector<uint8_t>&)>& value_serializer)
  {
    return graph.SerializeSelf(buffer, value_serializer);
  }

  static std::pair<Graph<NodeValueType>, uint64_t> Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset,
      const std::function<std::pair<NodeValueType, uint64_t>(
          const std::vector<uint8_t>&, const uint64_t)>& value_deserializer)
  {
    Graph<NodeValueType> temp_graph;
    const uint64_t bytes_read
        = temp_graph.DeserializeSelf(
            buffer, starting_offset, value_deserializer);
    return std::make_pair(temp_graph, bytes_read);
  }

  Graph(const std::vector<GraphNode<NodeValueType>>& nodes)
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
      const std::function<uint64_t(
          const NodeValueType&, std::vector<uint8_t>&)>& value_serializer) const
  {
    const uint64_t start_buffer_size = buffer.size();
    std::function<uint64_t(const GraphNode<NodeValueType>&,
                           std::vector<uint8_t>&)> graph_state_serializer =
        [&] (const GraphNode<NodeValueType>& node,
             std::vector<uint8_t>& serialize_buffer)
    {
      return GraphNode<NodeValueType>::Serialize(
          node, serialize_buffer, value_serializer);
    };
    serialization::SerializeVectorLike<GraphNode<NodeValueType>>(
        nodes_, buffer, graph_state_serializer);
    // Figure out how many bytes were written
    const uint64_t end_buffer_size = buffer.size();
    const uint64_t bytes_written = end_buffer_size - start_buffer_size;
    return bytes_written;
  }

  uint64_t DeserializeSelf(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset,
      const std::function<std::pair<NodeValueType, uint64_t>(
          const std::vector<uint8_t>&, const uint64_t)>& value_deserializer)
  {
    const std::function<std::pair<GraphNode<NodeValueType>, uint64_t>(
        const std::vector<uint8_t>&, const uint64_t)> graph_state_deserializer =
        [&] (const std::vector<uint8_t>& deserialize_buffer,
             const uint64_t offset)
    {
      return GraphNode<NodeValueType>::Deserialize(
          deserialize_buffer, offset, value_deserializer);
    };
    const std::pair<std::vector<GraphNode<NodeValueType>>, uint64_t>
        deserialized_nodes
        = serialization::DeserializeVectorLike<GraphNode<NodeValueType>>(
            buffer, starting_offset, graph_state_deserializer);
    nodes_ = deserialized_nodes.first;
    if (!CheckGraphLinkage())
    {
      throw std::invalid_argument("Deserialized invalid graph linkage");
    }
    return deserialized_nodes.second;
  }

  std::string Print() const
  {
    std::ostringstream strm;
    strm << "Graph - Nodes : ";
    if (nodes_.size() > 0)
    {
      strm << nodes_[0].Print();
      for (size_t idx = 1; idx < nodes_.size(); idx++)
      {
        strm << "\n" << nodes_[idx].Print();
      }
    }
    return strm.str();
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
      const std::vector<GraphNode<NodeValueType>>& nodes)
  {
    // Go through every node and make sure the edges are valid
    for (size_t idx = 0; idx < nodes.size(); idx++)
    {
      const GraphNode<NodeValueType>& current_node = nodes[idx];
      // Check the in edges first
      const std::vector<GraphEdge>& in_edges
          = current_node.GetInEdgesImmutable();
      for (size_t in_edge_idx = 0; in_edge_idx < in_edges.size(); in_edge_idx++)
      {
        const GraphEdge& current_edge = in_edges[in_edge_idx];
        // Check from index to make sure it's in bounds
        const int64_t from_index = current_edge.GetFromIndex();
        if (from_index < 0 || from_index >= static_cast<int64_t>(nodes.size()))
        {
          return false;
        }
        // Check to index to make sure it matches our own index
        const int64_t to_index = current_edge.GetToIndex();
        if (to_index != static_cast<int64_t>(idx))
        {
          return false;
        }
        // Check edge validity (edges to ourself are not allowed)
        if (from_index == to_index)
        {
          return false;
        }
        // Check to make sure that the from index node is linked to us
        const GraphNode<NodeValueType>& from_node
            = nodes[static_cast<size_t>(from_index)];
        const std::vector<GraphEdge>& from_node_out_edges
            = from_node.GetOutEdgesImmutable();
        bool from_node_connection_valid = false;
        // Make sure at least one out edge of the from index node corresponds
        // to the current node
        for (size_t from_node_out_edge_idx = 0;
             from_node_out_edge_idx < from_node_out_edges.size();
             from_node_out_edge_idx++)
        {
          const GraphEdge& current_from_node_out_edge
              = from_node_out_edges[from_node_out_edge_idx];
          if (current_from_node_out_edge.GetToIndex()
              == static_cast<int64_t>(idx))
          {
            from_node_connection_valid = true;
          }
        }
        if (from_node_connection_valid == false)
        {
          return false;
        }
      }
      // Check the out edges second
      const std::vector<GraphEdge>& out_edges
          = current_node.GetOutEdgesImmutable();
      for (size_t out_edge_idx = 0; out_edge_idx < out_edges.size();
           out_edge_idx++)
      {
        const GraphEdge& current_edge = out_edges[out_edge_idx];
        // Check from index to make sure it matches our own index
        const int64_t from_index = current_edge.GetFromIndex();
        if (from_index != static_cast<int64_t>(idx))
        {
          return false;
        }
        // Check to index to make sure it's in bounds
        const int64_t to_index = current_edge.GetToIndex();
        if (to_index < 0 || to_index >= static_cast<int64_t>(nodes.size()))
        {
          return false;
        }
        // Check edge validity (edges to ourself are not allowed)
        if (from_index == to_index)
        {
          return false;
        }
        // Check to make sure that the to index node is linked to us
        const GraphNode<NodeValueType>& to_node
            = nodes[static_cast<size_t>(to_index)];
        const std::vector<GraphEdge>& to_node_in_edges
            = to_node.GetInEdgesImmutable();
        bool to_node_connection_valid = false;
        // Make sure at least one in edge of the to index node corresponds to
        // the current node
        for (size_t to_node_in_edge_idx = 0;
             to_node_in_edge_idx < to_node_in_edges.size();
             to_node_in_edge_idx++)
        {
          const GraphEdge& current_to_node_in_edge
              = to_node_in_edges[to_node_in_edge_idx];
          if (current_to_node_in_edge.GetFromIndex()
              == static_cast<int64_t>(idx))
          {
            to_node_connection_valid = true;
          }
        }
        if (to_node_connection_valid == false)
        {
          return false;
        }
      }
    }
    return true;
  }

  const std::vector<GraphNode<NodeValueType>>& GetNodesImmutable() const
  {
    return nodes_;
  }

  std::vector<GraphNode<NodeValueType>>& GetNodesMutable() { return nodes_; }

  const GraphNode<NodeValueType>& GetNodeImmutable(const int64_t index) const
  {
    return nodes_.at(static_cast<size_t>(index));
  }

  GraphNode<NodeValueType>& GetNodeMutable(const int64_t index)
  {
    return nodes_.at(static_cast<size_t>(index));
  }

  int64_t AddNode(const GraphNode<NodeValueType>& new_node)
  {
    nodes_.push_back(new_node);
    return static_cast<int64_t>(nodes_.size() - 1);
  }

  int64_t AddNode(const NodeValueType& new_value)
  {
    nodes_.push_back(GraphNode<NodeValueType>(new_value));
    return static_cast<int64_t>(nodes_.size() - 1);
  }

  void AddEdgeBetweenNodes(const int64_t from_index, const int64_t to_index,
                           const double edge_weight)
  {
    // We retrieve the nodes first, since retrieval performs bounds checks first
    GraphNode<NodeValueType>& from_node = GetNodeMutable(from_index);
    GraphNode<NodeValueType>& to_node = GetNodeMutable(to_index);
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
    GraphNode<NodeValueType>& first_node = GetNodeMutable(first_index);
    GraphNode<NodeValueType>& second_node = GetNodeMutable(second_index);
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
}  // namespace simple_graph
}  // namespace common_robotics_utilities

inline std::ostream& operator<< (
    std::ostream& stream,
    const common_robotics_utilities::simple_graph::GraphEdge& edge)
{
    stream << edge.Print();
    return stream;
}

template<typename NodeValueType>
inline std::ostream& operator<< (
    std::ostream& stream,
    const common_robotics_utilities::simple_graph
        ::GraphNode<NodeValueType>& node)
{
    stream << node.Print();
    return stream;
}

template<typename NodeValueType>
inline std::ostream& operator<< (
    std::ostream& stream,
    const common_robotics_utilities::simple_graph
        ::Graph<NodeValueType>& graph)
{
    stream << graph.Print();
    return stream;
}
