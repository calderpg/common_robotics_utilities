#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/math.hpp>
#include <common_robotics_utilities/openmp_helpers.hpp>

namespace common_robotics_utilities
{
/// Implementation of hierarchical clustering, in both Single-link and
/// Complete-link forms. Unlike some implementations, this does not produce the
/// dendrogram of clusters - it only returns the accumulated clusters up to the
/// provided cluster distance bound. In complete-link clustering, the maximum
/// point-to-point distance in any cluster must be <= distance bound, and
/// clustering is performed by combining two items (each could be a point or an
/// existing cluster) if the maximum distance is less than distance bound.
/// Single-link clustering is produced by combining two items (each could be a
/// point or an existing cluster) if the minimum distance is less than distance
/// bound.
/// Complete-link clustering produces dense clusters, while single-link
/// clustering produces long "thin" clusters.
namespace simple_hierarchical_clustering
{
enum class ClusterStrategy { SINGLE_LINK, COMPLETE_LINK };

/// Storage for a single "item" - either an index to a single value or an index
/// to an existing cluster.
class Item
{
private:
  int64_t index_ = -1;
  bool is_cluster_ = false;

public:
  Item() : index_(-1), is_cluster_(false) {}

  Item(const size_t index, const bool is_cluster)
      : Item(static_cast<int64_t>(index), is_cluster) {}

  Item(const int64_t index, const bool is_cluster)
      : index_(index), is_cluster_(is_cluster)
  {
    if (index_ < 0)
    {
      throw std::invalid_argument("index < 0");
    }
  }

  int64_t Index() const { return index_; }

  bool IsValue() const { return !is_cluster_; }

  bool IsCluster() const { return is_cluster_; }

  bool IsValid() const { return (index_ >= 0); }
};

/// Storage for a pair of "items" and the distance between them
class ClosestPair
{
private:
  Item first_item_;
  Item second_item_;
  double distance_ = std::numeric_limits<double>::infinity();

public:
  ClosestPair() : distance_(std::numeric_limits<double>::infinity()) {}

  ClosestPair(const Item& first_item, const Item& second_item,
              const double distance)
      : first_item_(first_item), second_item_(second_item), distance_(distance)
  {
    if (distance < 0.0)
    {
      throw std::invalid_argument("distance < 0.0");
    }
    if (!first_item_.IsValid())
    {
      throw std::invalid_argument("first_item is not valid");
    }
    if (!second_item_.IsValid())
    {
      throw std::invalid_argument("second_item is not valid");
    }
    if (first_item_.IsValue() == second_item_.IsValue())
    {
      if (first_item_.Index() == second_item_.Index())
      {
        throw std::invalid_argument("first and second items are the same");
      }
    }
  }

  const Item& FirstItem() const { return first_item_; }

  const Item& SecondItem() const { return second_item_; }

  double Distance() const { return distance_; }

  bool IsValid() const
  {
    return first_item_.IsValid() && second_item_.IsValid();
  }
};

/// Find the closest existing clusters in @param clusters, using the pairwise
/// element-to-element distances in @param distance_matrix and the strategy
/// specified by @param strategy. @return closest pair of clusters.
/// Search is performed in parallel.
inline ClosestPair GetClosestClustersParallel(
    const Eigen::MatrixXd& distance_matrix,
    const std::vector<std::vector<int64_t>>& clusters,
    const ClusterStrategy strategy)
{
  std::vector<ClosestPair> per_thread_closest_clusters(
      static_cast<size_t>(openmp_helpers::GetNumOmpThreads()), ClosestPair());
  CRU_OMP_PARALLEL_FOR
  for (size_t first_cluster_idx = 0; first_cluster_idx < clusters.size();
       first_cluster_idx++)
  {
    // Skip empty clusters
    const std::vector<int64_t>& first_cluster = clusters.at(first_cluster_idx);
    if (first_cluster.size() > 0)
    {
      // Only compare against remaining clusters
      for (size_t second_cluster_idx = first_cluster_idx + 1;
           second_cluster_idx < clusters.size();
           second_cluster_idx++)
      {
        // Skip empty clusters
        const std::vector<int64_t>& second_cluster
            = clusters.at(second_cluster_idx);
        if (second_cluster.size() > 0)
        {
          // Compute cluster-cluster distance
          double minimum_distance = std::numeric_limits<double>::infinity();
          double maximum_distance = 0.0;
          for (const int64_t& cluster1_index : first_cluster)
          {
            for (const int64_t& cluster2_index : second_cluster)
            {
              const double distance = distance_matrix(
                  static_cast<ssize_t>(cluster1_index),
                  static_cast<ssize_t>(cluster2_index));
              minimum_distance = std::min(minimum_distance, distance);
              maximum_distance = std::max(maximum_distance, distance);
            }
          }
          const double cluster_distance
              = (strategy == ClusterStrategy::COMPLETE_LINK) ? maximum_distance
                                                             : minimum_distance;
          const size_t thread_num =
              static_cast<size_t>(openmp_helpers::GetContextOmpThreadNum());
          const double current_closest_distance
              = per_thread_closest_clusters.at(thread_num).Distance();
          if (cluster_distance < current_closest_distance)
          {
            per_thread_closest_clusters.at(thread_num)
                = ClosestPair(Item(first_cluster_idx, true),
                              Item(second_cluster_idx, true),
                              cluster_distance);
          }
        }
      }
    }
  }
  ClosestPair closest_clusters;
  for (const ClosestPair& per_thread_closest_cluster_pair
       : per_thread_closest_clusters)
  {
    if (per_thread_closest_cluster_pair.Distance()
        < closest_clusters.Distance())
    {
      closest_clusters = per_thread_closest_cluster_pair;
    }
  }
  return closest_clusters;
}

/// Find the closest existing clusters in @param clusters, using the pairwise
/// element-to-element distances in @param distance_matrix and the strategy
/// specified by @param strategy. @return closest pair of clusters.
inline ClosestPair GetClosestClustersSerial(
    const Eigen::MatrixXd& distance_matrix,
    const std::vector<std::vector<int64_t>>& clusters,
    const ClusterStrategy strategy)
{
  ClosestPair closest_clusters;
  for (size_t first_cluster_idx = 0; first_cluster_idx < clusters.size();
       first_cluster_idx++)
  {
    // Skip empty clusters
    const std::vector<int64_t>& first_cluster = clusters.at(first_cluster_idx);
    if (first_cluster.size() > 0)
    {
      // Only compare against remaining clusters
      for (size_t second_cluster_idx = first_cluster_idx + 1;
           second_cluster_idx < clusters.size();
           second_cluster_idx++)
      {
        // Skip empty clusters
        const std::vector<int64_t>& second_cluster
            = clusters.at(second_cluster_idx);
        if (second_cluster.size() > 0)
        {
          // Compute cluster-cluster distance
          double minimum_distance = std::numeric_limits<double>::infinity();
          double maximum_distance = 0.0;
          for (const int64_t& cluster1_index : first_cluster)
          {
            for (const int64_t& cluster2_index : second_cluster)
            {
              const double distance = distance_matrix(
                  static_cast<ssize_t>(cluster1_index),
                  static_cast<ssize_t>(cluster2_index));
              minimum_distance = std::min(minimum_distance, distance);
              maximum_distance = std::max(maximum_distance, distance);
            }
          }
          const double cluster_distance
              = (strategy == ClusterStrategy::COMPLETE_LINK) ? maximum_distance
                                                             : minimum_distance;
          if (cluster_distance < closest_clusters.Distance())
          {
            closest_clusters = ClosestPair(Item(first_cluster_idx, true),
                                           Item(second_cluster_idx, true),
                                           cluster_distance);
          }
        }
      }
    }
  }
  return closest_clusters;
}

/// Find the closest existing clusters in @param clusters, using the pairwise
/// element-to-element distances in @param distance_matrix and the strategy
/// specified by @param strategy. @return closest pair of clusters.
/// @param use_parallel selects if the search should be performed in parallel.
inline ClosestPair GetClosestClusters(
    const Eigen::MatrixXd& distance_matrix,
    const std::vector<std::vector<int64_t>>& clusters,
    const ClusterStrategy strategy,
    const bool use_parallel)
{
  if (use_parallel)
  {
    return GetClosestClustersParallel(distance_matrix, clusters, strategy);
  }
  else
  {
    return GetClosestClustersSerial(distance_matrix, clusters, strategy);
  }
}

/// Find the closest value-{value, cluster} pair, using @param datapoint_mask to
/// ignore values that have already been clustered, @param distance_matrix to
/// provide pairwise value-to-value distances, existing clusters provided by
/// @param clusters, and strategy specified by @param strategy.
/// Search is performed in parallel. @return closest pair.
inline ClosestPair GetClosestValueToOtherParallel(
    const std::vector<uint8_t>& datapoint_mask,
    const Eigen::MatrixXd& distance_matrix,
    const std::vector<std::vector<int64_t>>& clusters,
    const ClusterStrategy strategy)
{
  std::vector<ClosestPair> per_thread_closest_value_other(
      static_cast<size_t>(openmp_helpers::GetNumOmpThreads()), ClosestPair());
  CRU_OMP_PARALLEL_FOR
  for (size_t value_idx = 0; value_idx < datapoint_mask.size(); value_idx++)
  {
    // Make sure we're not already clustered
    if (datapoint_mask.at(value_idx) == 0x00)
    {
      const size_t thread_num =
          static_cast<size_t>(openmp_helpers::GetContextOmpThreadNum());
      // Check against other values
      for (size_t other_value_idx = value_idx + 1;
           other_value_idx < datapoint_mask.size(); other_value_idx++)
      {
        // Make sure it's not already clustered
        if (datapoint_mask.at(other_value_idx) == 0x00)
        {
          const double distance = distance_matrix(
              static_cast<ssize_t>(value_idx),
              static_cast<ssize_t>(other_value_idx));
          const double current_closest_distance
              = per_thread_closest_value_other.at(thread_num).Distance();
          if (distance < current_closest_distance)
          {
            per_thread_closest_value_other.at(thread_num)
                = ClosestPair(Item(value_idx, false),
                              Item(other_value_idx, false),
                              distance);
          }
        }
      }
      // Check against clusters
      for (size_t cluster_idx = 0; cluster_idx < clusters.size(); cluster_idx++)
      {
        const std::vector<int64_t>& cluster = clusters.at(cluster_idx);
        // Skip empty clusters
        if (cluster.size() > 0)
        {
          // Compute cluster-cluster distance
          double minimum_distance = std::numeric_limits<double>::infinity();
          double maximum_distance = 0.0;
          for (const int64_t& cluster_element_idx : cluster)
          {
            const double distance = distance_matrix(
                static_cast<ssize_t>(value_idx),
                static_cast<ssize_t>(cluster_element_idx));
            minimum_distance = std::min(minimum_distance, distance);
            maximum_distance = std::max(maximum_distance, distance);
          }
          const double cluster_distance
              = (strategy == ClusterStrategy::COMPLETE_LINK) ? maximum_distance
                                                             : minimum_distance;
          const double current_closest_distance
              = per_thread_closest_value_other.at(thread_num).Distance();
          if (cluster_distance < current_closest_distance)
          {
            per_thread_closest_value_other.at(thread_num)
                = ClosestPair(Item(value_idx, false),
                              Item(cluster_idx, true),
                              cluster_distance);
          }
        }
      }
    }
  }
  ClosestPair closest_value_other;
  for (const ClosestPair& value_other : per_thread_closest_value_other)
  {
    if (value_other.Distance() < closest_value_other.Distance())
    {
      closest_value_other = value_other;
    }
  }
  return closest_value_other;
}

/// Find the closest value-{value, cluster} pair, using @param datapoint_mask to
/// ignore values that have already been clustered, @param distance_matrix to
/// provide pairwise value-to-value distances, existing clusters provided by
/// @param clusters, and strategy specified by @param strategy. @return closest
/// pair.
inline ClosestPair GetClosestValueToOtherSerial(
    const std::vector<uint8_t>& datapoint_mask,
    const Eigen::MatrixXd& distance_matrix,
    const std::vector<std::vector<int64_t>>& clusters,
    const ClusterStrategy strategy)
{
  ClosestPair closest_value_other;
  for (size_t value_idx = 0; value_idx < datapoint_mask.size(); value_idx++)
  {
    // Make sure we're not already clustered
    if (datapoint_mask.at(value_idx) == 0x00)
    {
      // Check against other values
      for (size_t other_value_idx = value_idx + 1;
           other_value_idx < datapoint_mask.size(); other_value_idx++)
      {
        // Make sure it's not already clustered
        if (datapoint_mask.at(other_value_idx) == 0x00)
        {
          const double distance = distance_matrix(
              static_cast<ssize_t>(value_idx),
              static_cast<ssize_t>(other_value_idx));
          if (distance < closest_value_other.Distance())
          {
            closest_value_other = ClosestPair(Item(value_idx, false),
                                              Item(other_value_idx, false),
                                              distance);
          }
        }
      }
      // Check against clusters
      for (size_t cluster_idx = 0; cluster_idx < clusters.size(); cluster_idx++)
      {
        const std::vector<int64_t>& cluster = clusters.at(cluster_idx);
        // Skip empty clusters
        if (cluster.size() > 0)
        {
          // Compute cluster-cluster distance
          double minimum_distance = std::numeric_limits<double>::infinity();
          double maximum_distance = 0.0;
          for (const int64_t& cluster_element_idx : cluster)
          {
            const double distance = distance_matrix(
                static_cast<ssize_t>(value_idx),
                static_cast<ssize_t>(cluster_element_idx));
            minimum_distance = std::min(minimum_distance, distance);
            maximum_distance = std::max(maximum_distance, distance);
          }
          const double cluster_distance
              = (strategy == ClusterStrategy::COMPLETE_LINK) ? maximum_distance
                                                             : minimum_distance;
          if (cluster_distance < closest_value_other.Distance())
          {
            closest_value_other = ClosestPair(Item(value_idx, false),
                                              Item(cluster_idx, true),
                                              cluster_distance);
          }
        }
      }
    }
  }
  return closest_value_other;
}

/// Find the closest value-{value, cluster} pair, using @param datapoint_mask to
/// ignore values that have already been clustered, @param distance_matrix to
/// provide pairwise value-to-value distances, existing clusters provided by
/// @param clusters, and strategy specified by @param strategy.
/// @param use_parallel selects if the search should be performed in parallel.
/// @return closest pair.
inline ClosestPair GetClosestValueToOther(
    const std::vector<uint8_t>& datapoint_mask,
    const Eigen::MatrixXd& distance_matrix,
    const std::vector<std::vector<int64_t>>& clusters,
    const ClusterStrategy strategy,
    const bool use_parallel)
{
  if (use_parallel)
  {
    return GetClosestValueToOtherParallel(
        datapoint_mask, distance_matrix, clusters, strategy);
  }
  else
  {
    return GetClosestValueToOtherSerial(
        datapoint_mask, distance_matrix, clusters, strategy);
  }
}

/// Find the closest {value, cluster}-{value, cluster} pair, using @param
/// datapoint_mask to ignore values that have already been clustered, @param
/// distance_matrix to provide pairwise value-to-value distances, existing
/// clusters provided by @param clusters, and strategy specified by @param
/// strategy. @param use_parallel selects if the search should be performed in
/// parallel. @return closest pair.
inline ClosestPair GetClosestPair(
    const std::vector<uint8_t>& datapoint_mask,
    const Eigen::MatrixXd& distance_matrix,
    const std::vector<std::vector<int64_t>>& clusters,
    const ClusterStrategy strategy,
    const bool use_parallel)
{
  const ClosestPair closest_value_to_other
      = GetClosestValueToOther(datapoint_mask, distance_matrix, clusters,
                               strategy, use_parallel);
  const ClosestPair closest_clusters
      = GetClosestClusters(distance_matrix, clusters, strategy, use_parallel);
  if (closest_value_to_other.IsValid() && closest_clusters.IsValid())
  {
    if (closest_value_to_other.Distance() < closest_clusters.Distance())
    {
      return closest_value_to_other;
    }
    else
    {
      return closest_clusters;
    }
  }
  else if (closest_value_to_other.IsValid())
  {
    return closest_value_to_other;
  }
  else if (closest_clusters.IsValid())
  {
    return closest_clusters;
  }
  else
  {
    return ClosestPair();
  }
}

template<typename DataType, typename Container=std::vector<DataType>>
class ClusteringResult
{
private:
  std::vector<Container> clusters_;
  double final_closest_distance_ = 0.0;

public:
  ClusteringResult() {}

  ClusteringResult(const std::vector<Container>& clusters,
                   const double final_closest_distance)
      : clusters_(clusters), final_closest_distance_(final_closest_distance) {}

  const std::vector<Container>& Clusters() const { return clusters_; }

  double FinalClosestDistance() const { return final_closest_distance_; }
};

using IndexClusteringResult = ClusteringResult<int64_t, std::vector<int64_t>>;

/// Perform hierarchical index clustering of @param data up to @param
/// max_cluster_distance, using @param distance_matrix to provide pairwise
/// value-to-value distance for values in @param data. Strategy to use is
/// specified by @param strategy, and @param use_parallel selects if parallel
/// search should be used internally.
inline IndexClusteringResult IndexClusterWithDistanceMatrix(
    const Eigen::MatrixXd& distance_matrix, const double max_cluster_distance,
    const ClusterStrategy strategy, const bool use_parallel = false)
{
  if (distance_matrix.rows() != distance_matrix.cols())
  {
    throw std::invalid_argument("distance_matrix is not square");
  }
  else if (distance_matrix.rows() == 0)
  {
    throw std::invalid_argument("distance_matrix is empty");
  }
  std::vector<uint8_t> datapoint_mask(
      static_cast<size_t>(distance_matrix.rows()), 0u);
  std::vector<std::vector<int64_t>> cluster_indices;
  double closest_distance = 0.0;
  bool complete = false;
  while (!complete)
  {
    // Get closest pair of items (an element can be a cluster or single value!)
    const ClosestPair closest_element_pair
        = GetClosestPair(datapoint_mask, distance_matrix, cluster_indices,
                         strategy, use_parallel);
    if (closest_element_pair.IsValid()
        && closest_element_pair.Distance() <= max_cluster_distance)
    {
      closest_distance = closest_element_pair.Distance();
      const auto& first_item = closest_element_pair.FirstItem();
      const auto& second_item = closest_element_pair.SecondItem();
      // If both elements are values, create a new cluster
      if (first_item.IsValue() && second_item.IsValue())
      {
        // Add a cluster
        cluster_indices.push_back(std::vector<int64_t>{first_item.Index(),
                                                       second_item.Index()});
        // Mask out the indices (this way we know they are already clustered)
        datapoint_mask.at(static_cast<size_t>(first_item.Index())) = 1u;
        datapoint_mask.at(static_cast<size_t>(second_item.Index())) = 1u;
      }
      // If both elements are clusters, merge the clusters
      else if (first_item.IsCluster() && second_item.IsCluster())
      {
        // Merge the second cluster into the first
        std::vector<int64_t>& first_cluster
            = cluster_indices.at(static_cast<size_t>(first_item.Index()));
        std::vector<int64_t>& second_cluster
            = cluster_indices.at(static_cast<size_t>(second_item.Index()));
        first_cluster.insert(
            first_cluster.end(), second_cluster.begin(), second_cluster.end());
        // Empty the second cluster
        // (we don't remove, because this would trigger a move and reallocation)
        second_cluster.clear();
      }
      // If one of the elements is a cluster and the other is a point, add the
      // point to the existing cluster
      else
      {
        const bool first_is_cluster = first_item.IsCluster();
        const int64_t cluster_index
            = (first_is_cluster) ? first_item.Index() : second_item.Index();
        const int64_t value_index
            = (first_is_cluster) ? second_item.Index() : first_item.Index();
        // Add the element to the cluster
        std::vector<int64_t>& cluster
            = cluster_indices.at(static_cast<size_t>(cluster_index));
        cluster.push_back(value_index);
        // Mask out the index (this way we know it is are already clustered)
        datapoint_mask.at(static_cast<size_t>(value_index)) = 1u;
      }
    }
    else
    {
      complete = true;
    }
  }
  // Get rid of empty clusters left over from cluster-cluster merges
  std::vector<std::vector<int64_t>> index_clusters;
  index_clusters.reserve(cluster_indices.size());
  for (const auto& index_cluster : cluster_indices)
  {
    if (index_cluster.size() > 0)
    {
      index_clusters.push_back(index_cluster);
    }
  }
  // Add any points that we haven't clustered into their own clusters
  for (size_t idx = 0; idx < datapoint_mask.size(); idx++)
  {
    // If an element hasn't been clustered at all yet, make a new cluster
    if (datapoint_mask.at(idx) == 0)
    {
      index_clusters.push_back(
          std::vector<int64_t>(1, static_cast<int64_t>(idx)));
    }
  }
  index_clusters.shrink_to_fit();
  return IndexClusteringResult(index_clusters, closest_distance);
}

template<typename DataType, typename Container=std::vector<DataType>>
inline ClusteringResult<DataType, Container>
MakeElementClusteringFromIndexClustering(
    const Container& data,
    const IndexClusteringResult& index_clustering)
{
  std::vector<Container> clusters(index_clustering.Clusters().size());
  for (size_t idx = 0; idx < clusters.size(); idx++)
  {
    const std::vector<int64_t>& current_index_cluster
        = index_clustering.Clusters().at(idx);
    Container& current_cluster = clusters.at(idx);
    // Use reserve + shrink_to_fit for cases where DataType is not
    // default-constructible.
    current_cluster.reserve(current_index_cluster.size());
    for (const int64_t data_index : current_index_cluster)
    {
      current_cluster.push_back(data.at(static_cast<size_t>(data_index)));
    }
    current_cluster.shrink_to_fit();
  }
  return ClusteringResult<DataType, Container>(
      clusters, index_clustering.FinalClosestDistance());
}

/// Perform hierarchical clustering of @param data up to @param
/// max_cluster_distance, using @param distance_matrix to provide pairwise
/// value-to-value distance for values in @param data. Strategy to use is
/// specified by @param strategy, and @param use_parallel selects if parallel
/// search should be used internally.
template<typename DataType, typename Container=std::vector<DataType>>
inline ClusteringResult<DataType, Container> ClusterWithDistanceMatrix(
    const Container& data, const Eigen::MatrixXd& distance_matrix,
    const double max_cluster_distance, const ClusterStrategy strategy,
    const bool use_parallel = false)
{
  // Safety check the input
  if (data.empty())
  {
    throw std::invalid_argument("data is empty");
  }
  if (static_cast<size_t>(distance_matrix.rows()) != data.size()
      || static_cast<size_t>(distance_matrix.cols()) != data.size())
  {
    throw std::invalid_argument("distance_matrix is the wrong size");
  }
  // Perform index clustering
  const auto index_clustering = IndexClusterWithDistanceMatrix(
      distance_matrix, max_cluster_distance, strategy, use_parallel);
  // Extract the actual cluster data from index clusters
  return MakeElementClusteringFromIndexClustering<DataType, Container>(
      data, index_clustering);
}

/// Perform hierarchical index clustering of @param data up to @param
/// max_cluster_distance, using @param distance_fn to compute pairwise
/// value-to-value distance for values in @param data. Strategy to use is
/// specified by @param strategy, and @param use_parallel selects if parallel
/// search should be used internally.
template<typename DataType, typename Container=std::vector<DataType>>
inline IndexClusteringResult IndexCluster(
    const Container& data,
    const std::function<double(const DataType&, const DataType&)>& distance_fn,
    const double max_cluster_distance, const ClusterStrategy strategy,
    const bool use_parallel = false)
{
  const Eigen::MatrixXd distance_matrix
      = math::BuildPairwiseDistanceMatrix<DataType, Container>(
          data, distance_fn, use_parallel);
  return IndexClusterWithDistanceMatrix(
      distance_matrix, max_cluster_distance, strategy, use_parallel);
}

/// Perform hierarchical clustering of @param data up to @param
/// max_cluster_distance, using @param distance_fn to compute pairwise
/// value-to-value distance for values in @param data. Strategy to use is
/// specified by @param strategy, and @param use_parallel selects if parallel
/// search should be used internally.
template<typename DataType, typename Container=std::vector<DataType>>
inline ClusteringResult<DataType, Container> Cluster(
    const Container& data,
    const std::function<double(const DataType&, const DataType&)>& distance_fn,
    const double max_cluster_distance, const ClusterStrategy strategy,
    const bool use_parallel = false)
{
  const Eigen::MatrixXd distance_matrix
      = math::BuildPairwiseDistanceMatrix<DataType, Container>(
          data, distance_fn, use_parallel);
  return ClusterWithDistanceMatrix<DataType, Container>(
      data, distance_matrix, max_cluster_distance, strategy, use_parallel);
}
}  // namespace simple_hierarchical_clustering
}  // namespace common_robotics_utilities
