#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <common_robotics_utilities/openmp_helpers.hpp>
#include <common_robotics_utilities/simple_knearest_neighbors.hpp>
#include <common_robotics_utilities/utility.hpp>

namespace common_robotics_utilities
{
/// Simple implementation of K-means clustering.
namespace simple_kmeans_clustering
{
/// Performs a single clustering iteration in which all elements of @param data
/// are assigned a label based on the closest cluster center in @param
/// current_cluster_centers. Cluster center-to-element distance is computed by
/// @param distance_fn. @return labels for all elements.
/// @param use_parallel selects if clustering should be performed in parallel.
template<typename DataType, typename Container=std::vector<DataType>>
inline std::vector<int32_t> PerformSingleClusteringIteration(
    const Container& data, const Container& current_cluster_centers,
    const std::function<double(const DataType&, const DataType&)>& distance_fn,
    const bool use_parallel)
{
  std::vector<int32_t> new_cluster_labels(data.size());

  CRU_OMP_PARALLEL_FOR_IF(use_parallel)
  for (size_t idx = 0; idx < data.size(); idx++)
  {
    const DataType& datapoint = data.at(idx);
    const int64_t closest_cluster_index
        = simple_knearest_neighbors::GetKNearestNeighborsSerial(
            current_cluster_centers, datapoint, distance_fn, 1).at(0).Index();
    new_cluster_labels.at(idx) = static_cast<int32_t>(closest_cluster_index);
  };

  return new_cluster_labels;
}

/// Recomputes cluster centers for elements in @param data and cluster labels
/// @param cluster_labels. For a given cluster, the cluster center is computed
/// by @param weighted_average_fn, which performs weighted averaging of the
/// provided Container of data and corresponding weights from @param
/// data_weights. @param num_clusters specifies the number of clusters, and is
/// used to preallocate storage. @return the new cluster centers.
/// @param use_parallel selects if cluster centers should be computed in
/// parallel.
template<typename DataType, typename Container=std::vector<DataType>>
inline Container ComputeClusterCentersWeighted(
    const Container& data,  const std::vector<double>& data_weights,
    const std::vector<int32_t>& cluster_labels,
    const std::function<DataType(
        const Container&, const std::vector<double>&)>& weighted_average_fn,
    const int32_t num_clusters, const bool use_parallel)
{
  if (data.size() == cluster_labels.size())
  {
    // TODO: improve this to avoid copies. It's possible right now, but it would
    // result in the average functions being more complicated and slower.
    // Separate the datapoints into their clusters
    std::vector<Container> clustered_data(static_cast<size_t>(num_clusters));
    std::vector<std::vector<double>> clustered_data_weights(
        static_cast<size_t>(num_clusters));
    for (size_t idx = 0; idx < data.size(); idx++)
    {
      const DataType& datapoint = data[idx];
      const int32_t label = cluster_labels.at(idx);
      clustered_data.at(static_cast<size_t>(label)).push_back(datapoint);
      clustered_data_weights.at(
          static_cast<size_t>(label)).push_back(data_weights.at(idx));
    }

    // Compute the center of each cluster
    Container cluster_centers(static_cast<size_t>(num_clusters));

    CRU_OMP_PARALLEL_FOR_IF(use_parallel)
    for (size_t cluster = 0; cluster < clustered_data.size(); cluster++)
    {
      const Container& cluster_data = clustered_data.at(cluster);
      const std::vector<double>& cluster_data_weights
          = clustered_data_weights.at(cluster);
      cluster_centers.at(cluster)
          = weighted_average_fn(cluster_data, cluster_data_weights);
    };

    return cluster_centers;
  }
  else
  {
    throw std::invalid_argument("data.size() != cluster_labels.size()");
  }
}

/// Checks for convergence - i.e. if @param old_labels and @param new_labels are
/// the same. @returns if the labels are the same.
inline bool CheckForConvergence(
    const std::vector<int32_t>& old_labels,
    const std::vector<int32_t>& new_labels)
{
  if (old_labels.size() == new_labels.size())
  {
    for (size_t idx = 0; idx < old_labels.size(); idx++)
    {
      if (old_labels.at(idx) != new_labels.at(idx))
      {
          return false;
      }
    }
    return true;
  }
  else
  {
    throw std::invalid_argument("old_labels.size() != new_labels.size()");
  }
}

/// Perform K-means clustering of @param data with corresponding weights @param
/// data_weightd, using @param distance_fn to compute element-to-element
/// distances and @param weighted_average_fn to compute the center/mean of a set
/// of elements with corresponding weights. @param num_clusters specifies the
/// desired number of clusters, and @param prng_seed specifies the seed for the
/// internal PRNG used to draw from @param data.
/// @param do_preliminary_clustering selects if a first pass of clustering using
/// 1/8 of the elements in @param data should be performed first, and
/// @param use_parallel selects if internal operations should be parallelized.
template<typename DataType, typename Container=std::vector<DataType>>
inline std::vector<int32_t> ClusterWeighted(
    const Container& data, const std::vector<double>& data_weights,
    const std::function<double(const DataType&, const DataType&)>& distance_fn,
    const std::function<DataType(const Container&, const std::vector<double>&)>&
        weighted_average_fn,
    const int32_t num_clusters, const int64_t prng_seed,
    const bool do_preliminary_clustering, const bool use_parallel = false,
    const utility::LoggingFunction& logging_fn = {})
{
  if (data.empty())
  {
    return std::vector<int32_t>();
  }
  if (num_clusters == 1)
  {
    if (logging_fn)
    {
      logging_fn("[K-means clustering] Provided num_clusters = 1");
    }
    return std::vector<int32_t>(data.size(), 0u);
  }
  else if (num_clusters == 0)
  {
    throw std::invalid_argument("num_clusters == 0");
  }
  // Make sure we have enough datapoints to do meaningful preliminary clustering
  bool enable_preliminary_clustering = do_preliminary_clustering;
  if (enable_preliminary_clustering)
  {
    const int64_t subset_size
        = static_cast<int64_t>(std::ceil(
            static_cast<double>(data.size()) * 0.125));
    if (subset_size >= (num_clusters * 5))
    {
      enable_preliminary_clustering = true;
      if (logging_fn)
      {
        logging_fn(
            "[K-means clustering] Preliminary clustering enabled, using subset "
            "of " + std::to_string(subset_size) + " datapoints from " +
            std::to_string(data.size()) + " total");
      }
    }
    else
    {
      enable_preliminary_clustering = false;
      if (logging_fn)
      {
        logging_fn(
            "[K-means clustering] Preliminary clustering disabled as input data"
            " is too small w.r.t. number of clusters");
      }
    }
  }
  // Prepare an RNG for cluster initialization
  std::mt19937_64 prng(
      static_cast<typename std::mt19937_64::result_type>(prng_seed));
  std::uniform_int_distribution<size_t> initialization_distribution(
      0, data.size() - 1);
  // Initialize cluster centers
  Container cluster_centers;
  if (enable_preliminary_clustering)
  {
    // Select a random 1/8th of the input data
    const size_t subset_size
        = static_cast<size_t>(std::ceil(
            static_cast<double>(data.size()) * 0.125));
    // This makes sure we don't get duplicates
    std::unordered_map<size_t, uint8_t> index_map;
    while (index_map.size() < subset_size)
    {
      const size_t random_index = initialization_distribution(prng);
      index_map[random_index] = 1u;
    }
    Container random_subset;
    random_subset.reserve(subset_size);
    std::vector<double> random_subset_weights;
    random_subset_weights.reserve(subset_size);
    for (auto itr = index_map.begin(); itr != index_map.end(); ++itr)
    {
      if (itr->second == 1u)
      {
        const size_t random_index = itr->first;
        const DataType& random_element = data.at(random_index);
        random_subset.push_back(random_element);
        random_subset_weights.push_back(data_weights.at(random_index));
      }
    }
    random_subset.shrink_to_fit();
    random_subset_weights.shrink_to_fit();
    // Run clustering on the subset
    std::vector<int32_t> random_subset_labels
        = ClusterWeighted<DataType, Container>(
            random_subset, random_subset_weights, distance_fn,
            weighted_average_fn, num_clusters, false, use_parallel);
    // Now we use the centers of the clusters to form the cluster centers
    cluster_centers
        = ComputeClusterCentersWeighted(
            random_subset, random_subset_weights, random_subset_labels,
            weighted_average_fn, num_clusters, use_parallel);
  }
  else
  {
    // This makes sure we don't get duplicates
    std::unordered_map<size_t, uint8_t> index_map;
    while (static_cast<int64_t>(index_map.size()) < num_clusters)
    {
      const size_t random_index = initialization_distribution(prng);
      index_map[random_index] = 1u;
    }
    cluster_centers.reserve(static_cast<size_t>(num_clusters));
    for (auto itr = index_map.begin(); itr != index_map.end(); ++itr)
    {
      if (itr->second == 1u)
      {
        const size_t random_index = itr->first;
        const DataType& random_element = data[random_index];
        cluster_centers.push_back(random_element);
      }
    }
    cluster_centers.shrink_to_fit();
  }
  if (static_cast<int64_t>(cluster_centers.size()) != num_clusters)
  {
    throw std::runtime_error("cluster_centers.size() != num_clusters");
  }
  // Run the first iteration of clustering
  std::vector<int32_t> cluster_labels
      = PerformSingleClusteringIteration(
          data, cluster_centers, distance_fn, use_parallel);
  bool converged = false;
  int32_t iteration = 1;
  // Run until convergence
  while (!converged)
  {
    iteration++;
    // Update cluster centers
    cluster_centers
        = ComputeClusterCentersWeighted(
            data, data_weights, cluster_labels, weighted_average_fn,
            num_clusters, use_parallel);
    // Cluster with the new centers
    const std::vector<int32_t> new_cluster_labels
        = PerformSingleClusteringIteration(
            data, cluster_centers, distance_fn, use_parallel);
    // Check for convergence
    converged = CheckForConvergence(cluster_labels, new_cluster_labels);
    cluster_labels = new_cluster_labels;
  }
  if (logging_fn)
  {
    logging_fn(
        "[K-means clustering] Clustering converged after " +
        std::to_string(iteration) + " iterations");
  }
  return cluster_labels;
}

/// Perform K-means clustering of @param data, using @param distance_fn to
/// compute element-to-element distances and @param average_fn to compute the
/// center/mean of a set of elements. @param num_clusters specifies the desired
/// number of clusters, and @param prng_seed specifies the seed for the internal
/// PRNG used to draw from @param data. @param do_preliminary_clustering selects
/// if a first pass of clustering using 1/8 of the elements in @param data
/// should be performed first, and @param use_parallel selects if internal
/// operations should be parallelized.
template<typename DataType, typename Container=std::vector<DataType>>
inline std::vector<int32_t> Cluster(
    const Container& data,
    const std::function<double(const DataType&, const DataType&)>& distance_fn,
    const std::function<DataType(const Container&)>& average_fn,
    const int32_t num_clusters, const int64_t prng_seed,
    const bool do_preliminary_clustering, const bool use_parallel = false,
    const utility::LoggingFunction& logging_fn = {})
{
  // Make a dummy set of uniform weights
  const std::vector<double> data_weights(data.size(), 1.0);
  // Wrap the non-weighted average function to make the weighted average
  // function signature.
  const std::function<DataType(const Container&, const std::vector<double>&)>
      weighted_average_fn = [&] (const Container& cluster_members,
                                 const std::vector<double>&)
  {
    return average_fn(cluster_members);
  };
  return ClusterWeighted<DataType, Container>(
      data, data_weights, distance_fn, weighted_average_fn, num_clusters,
      prng_seed, do_preliminary_clustering, use_parallel, logging_fn);
}
}  // namespace simple_kmeans_clustering
}  // namespace common_robotics_utilities
