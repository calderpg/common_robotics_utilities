#include <chrono>
#include <fstream>
#include <iostream>
#include <random>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/math.hpp>
#include <common_robotics_utilities/simple_hierarchical_clustering.hpp>
#include <common_robotics_utilities/simple_kmeans_clustering.hpp>

using PointVector = common_robotics_utilities::math::VectorVector4d;
using IndexClusteringResult
    = common_robotics_utilities::simple_hierarchical_clustering
        ::IndexClusteringResult;
using PointClusteringResult
    = common_robotics_utilities::simple_hierarchical_clustering
        ::ClusteringResult<Eigen::Vector4d, PointVector>;

void SaveClustering(
    const PointClusteringResult& clusters,
    const std::string& filename)
{
  // Use the cluster index to set z-coordinates for easy visualization
  std::ofstream clustering_output_file(filename, std::ios_base::out);
  if (!clustering_output_file.is_open())
  {
    throw std::invalid_argument(
        "Log file " + filename + " must be write-openable");
  }
  std::cout << "Saving clustering to " << filename << std::endl;
  for (size_t idx = 0; idx < clusters.Clusters().size(); idx++)
  {
    const PointVector& cluster = clusters.Clusters().at(idx);
    const double cluster_num = 1.0 + static_cast<double>(idx);
    for (const Eigen::Vector4d& point : cluster)
    {
      clustering_output_file << point(0) << "," << point(1) << ","
                             << cluster_num << std::endl;
    }
  }
  clustering_output_file.close();
}

void SaveIndexClustering(
    const PointVector& raw_points,
    const IndexClusteringResult& index_clusters,
    const std::string& filename)
{
  SaveClustering(
      common_robotics_utilities::simple_hierarchical_clustering
          ::MakeElementClusteringFromIndexClustering<
              Eigen::Vector4d, PointVector>(raw_points, index_clusters),
      filename);
}

int main(int argc, char** argv)
{
  const size_t num_points
      = (argc >= 2) ? static_cast<size_t>(atoi(argv[1])) : 1000;
  const double cluster_threshold
      = (argc >= 3) ? atof(argv[2]) : 0.1;
  if (cluster_threshold < 0.0)
  {
    throw std::invalid_argument("cluster_threshold < 0.0");
  }
  std::cout << "Generating " << num_points << " points..." << std::endl;
  const int64_t seed = 42;
  std::mt19937_64 prng(seed);
  std::uniform_real_distribution<double> dist(0.0, 10.0);
  PointVector random_points(num_points);
  for (size_t idx = 0; idx < num_points; idx++)
  {
    const double x = dist(prng);
    const double y = dist(prng);
    random_points.at(idx) = Eigen::Vector4d(x, y, 0.0, 0.0);
  }
  std::function<double(const Eigen::Vector4d&, const Eigen::Vector4d&)>
      distance_fn = [] (const Eigen::Vector4d& v1, const Eigen::Vector4d& v2)
  {
    return (v1 - v2).norm();
  };
  const std::vector<bool> use_parallel_options = {false, true};
  for (const bool use_parallel : use_parallel_options)
  {
    // Cluster using index-cluster methods
    // Single-link clustering
    std::cout << "Single-link hierarchical index clustering " << num_points
              << " points..." << std::endl;
    const auto slhic_start = std::chrono::steady_clock::now();
    const auto single_link_index_clusters
        = common_robotics_utilities::simple_hierarchical_clustering
            ::IndexCluster(
                random_points, distance_fn, 1.0,
                common_robotics_utilities::simple_hierarchical_clustering
                    ::ClusterStrategy::SINGLE_LINK, use_parallel);
    const auto slhic_end = std::chrono::steady_clock::now();
    const double slhic_elapsed =
        std::chrono::duration<double>(slhic_end - slhic_start).count();
    std::cout << "...took " << slhic_elapsed << " seconds" << std::endl;
    // Save
    SaveIndexClustering(
        random_points, single_link_index_clusters, (use_parallel)
            ? "/tmp/test_parallel_single_link_index_clustering.csv"
            : "/tmp/test_serial_single_link_index_clustering.csv");
    // Complete-link clustering
    std::cout << "Complete-link hierarchical index clustering " << num_points
              << " points..." << std::endl;
    const auto clhic_start = std::chrono::steady_clock::now();
    const auto complete_link_index_clusters
        = common_robotics_utilities::simple_hierarchical_clustering
            ::IndexCluster(
                random_points, distance_fn, 1.0,
                common_robotics_utilities::simple_hierarchical_clustering
                    ::ClusterStrategy::COMPLETE_LINK, use_parallel);
    const auto clhic_end = std::chrono::steady_clock::now();
    const double clhic_elapsed =
        std::chrono::duration<double>(clhic_end - clhic_start).count();
    std::cout << "...took " << clhic_elapsed << " seconds" << std::endl;
    // Save
    SaveIndexClustering(
        random_points, complete_link_index_clusters, (use_parallel)
            ? "/tmp/test_parallel_complete_link_index_clustering.csv"
            : "/tmp/test_serial_complete_link_index_clustering.csv");
    // Cluster using value-cluster methods
    // Single-link clustering
    std::cout << "Single-link hierarchical clustering " << num_points
              << " points..." << std::endl;
    const auto slhc_start = std::chrono::steady_clock::now();
    const auto single_link_clusters
        = common_robotics_utilities::simple_hierarchical_clustering::Cluster(
            random_points, distance_fn, 1.0,
            common_robotics_utilities::simple_hierarchical_clustering
                ::ClusterStrategy::SINGLE_LINK, use_parallel);
    const auto slhc_end = std::chrono::steady_clock::now();
    const double slhc_elapsed =
        std::chrono::duration<double>(slhc_end - slhc_start).count();
    std::cout << "...took " << slhc_elapsed << " seconds" << std::endl;
    // Save
    SaveClustering(single_link_clusters, (use_parallel)
        ? "/tmp/test_parallel_single_link_clustering.csv"
        : "/tmp/test_serial_single_link_clustering.csv");
    // Complete-link clustering
    std::cout << "Complete-link hierarchical clustering " << num_points
              << " points..." << std::endl;
    const auto clhc_start = std::chrono::steady_clock::now();
    const auto complete_link_clusters
        = common_robotics_utilities::simple_hierarchical_clustering::Cluster(
            random_points, distance_fn, 1.0,
            common_robotics_utilities::simple_hierarchical_clustering
                ::ClusterStrategy::COMPLETE_LINK, use_parallel);
    const auto clhc_end = std::chrono::steady_clock::now();
    const double clhc_elapsed =
        std::chrono::duration<double>(clhc_end - clhc_start).count();
    std::cout << "...took " << clhc_elapsed << " seconds" << std::endl;
    // Save
    SaveClustering(complete_link_clusters, (use_parallel)
        ? "/tmp/test_parallel_complete_link_clustering.csv"
        : "/tmp/test_serial_complete_link_clustering.csv");
    // Cluster using K-means
    const int32_t num_clusters = 50;
    std::cout << "K-means clustering " << num_points << " points into "
              << num_clusters << " clusters..." << std::endl;
    std::function<Eigen::Vector4d(const PointVector&)>
        average_fn = [] (const PointVector& cluster)
    {
      return common_robotics_utilities::math::AverageEigenVector4d(cluster);
    };
    const auto logging_fn = [] (const std::string& message)
    {
      std::cout << message << std::endl;
    };
    const auto kmeans_start = std::chrono::steady_clock::now();
    const std::vector<int32_t> kmeans_labels
        = common_robotics_utilities::simple_kmeans_clustering::Cluster(
            random_points, distance_fn, average_fn, num_clusters, 42, true,
            use_parallel, logging_fn);
    const auto kmeans_end = std::chrono::steady_clock::now();
    const double kmeans_elapsed =
        std::chrono::duration<double>(kmeans_end - kmeans_start).count();
    std::cout << "...took " << kmeans_elapsed << " seconds" << std::endl;
    // Get value clusters from the K-means labels
    std::vector<PointVector> kmeans_clusters(num_clusters);
    for (size_t idx = 0; idx < kmeans_labels.size(); idx++)
    {
      const size_t cluster_num = static_cast<size_t>(kmeans_labels.at(idx));
      kmeans_clusters.at(cluster_num).push_back(random_points.at(idx));
    }
    // Save
    SaveClustering(complete_link_clusters, (use_parallel)
        ? "/tmp/test_parallel_kmeans_clustering.csv"
        : "/tmp/test_serial_kmeans_clustering.csv");
  }
  std::cout << "Done saving, you can plot as a 3d scatterplot to see clustering"
            << std::endl;
  return 0;
}
