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

void SaveClustering(
    const std::vector<std::vector<Eigen::VectorXd>>& clusters,
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
  for (size_t idx = 0; idx < clusters.size(); idx++)
  {
    const std::vector<Eigen::VectorXd>& cluster = clusters.at(idx);
    const double cluster_num = 1.0 + static_cast<double>(idx);
    for (const Eigen::VectorXd& point : cluster)
    {
      clustering_output_file << point(0) << "," << point(1) << ","
                             << cluster_num << std::endl;
    }
  }
  clustering_output_file.close();
}

void SaveIndexClustering(
    const std::vector<Eigen::VectorXd>& raw_points,
    const std::vector<std::vector<int64_t>>& index_clusters,
    const std::string& filename)
{
  SaveClustering(
      common_robotics_utilities::simple_hierarchical_clustering
          ::MakeElementClustersFromIndexClusters<Eigen::VectorXd>(
              raw_points, index_clusters),
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
  std::vector<Eigen::VectorXd> random_points(num_points);
  for (size_t idx = 0; idx < num_points; idx++)
  {
    const double x = dist(prng);
    const double y = dist(prng);
    Eigen::VectorXd random_point(3);
    random_point << x, y, 0.0;
    random_points.at(idx) = random_point;
  }
  std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&)>
      distance_fn = [] (const Eigen::VectorXd& v1, const Eigen::VectorXd& v2)
  {
    return common_robotics_utilities::math::Distance(v1, v2);
  };
  const std::vector<bool> use_parallel_options = {false, true};
  for (const bool use_parallel : use_parallel_options)
  {
    // Cluster using index-cluster methods
    // Single-link clustering
    std::cout << "Single-link hierarchical index clustering " << num_points
              << " points..." << std::endl;
    const std::vector<std::vector<int64_t>> single_link_index_clusters
        = common_robotics_utilities::simple_hierarchical_clustering
            ::IndexCluster(
                random_points, distance_fn, 1.0,
                common_robotics_utilities::simple_hierarchical_clustering
                    ::ClusterStrategy::SINGLE_LINK, use_parallel).first;
    // Save
    SaveIndexClustering(
        random_points, single_link_index_clusters, (use_parallel)
            ? "/tmp/test_parallel_single_link_index_clustering.csv"
            : "/tmp/test_serial_single_link_index_clustering.csv");
    // Complete-link clustering
    std::cout << "Complete-link hierarchical index clustering " << num_points
              << " points..." << std::endl;
    const std::vector<std::vector<int64_t>> complete_link_index_clusters
        = common_robotics_utilities::simple_hierarchical_clustering
            ::IndexCluster(
                random_points, distance_fn, 1.0,
                common_robotics_utilities::simple_hierarchical_clustering
                    ::ClusterStrategy::COMPLETE_LINK, use_parallel).first;
    // Save
    SaveIndexClustering(
        random_points, complete_link_index_clusters, (use_parallel)
            ? "/tmp/test_parallel_complete_link_index_clustering.csv"
            : "/tmp/test_serial_complete_link_index_clustering.csv");
    // Cluster using value-cluster methods
    // Single-link clustering
    std::cout << "Single-link hierarchical clustering " << num_points
              << " points..." << std::endl;
    const std::vector<std::vector<Eigen::VectorXd>> single_link_clusters
        = common_robotics_utilities::simple_hierarchical_clustering::Cluster(
            random_points, distance_fn, 1.0,
            common_robotics_utilities::simple_hierarchical_clustering
                ::ClusterStrategy::SINGLE_LINK, use_parallel).first;
    // Save
    SaveClustering(single_link_clusters, (use_parallel)
        ? "/tmp/test_parallel_single_link_clustering.csv"
        : "/tmp/test_serial_single_link_clustering.csv");
    // Complete-link clustering
    std::cout << "Complete-link hierarchical clustering " << num_points
              << " points..." << std::endl;
    const std::vector<std::vector<Eigen::VectorXd>> complete_link_clusters
        = common_robotics_utilities::simple_hierarchical_clustering::Cluster(
            random_points, distance_fn, 1.0,
            common_robotics_utilities::simple_hierarchical_clustering
                ::ClusterStrategy::COMPLETE_LINK, use_parallel).first;
    // Save
    SaveClustering(complete_link_clusters, (use_parallel)
        ? "/tmp/test_parallel_complete_link_clustering.csv"
        : "/tmp/test_serial_complete_link_clustering.csv");
    // Cluster using K-means
    const int32_t num_clusters = 50;
    std::cout << "K-means clustering " << num_points << " points into "
              << num_clusters << " clusters..." << std::endl;
    std::function<Eigen::VectorXd(const std::vector<Eigen::VectorXd>&)>
        average_fn = [] (const std::vector<Eigen::VectorXd>& cluster)
    {
      return common_robotics_utilities::math::AverageEigenVectorXd(cluster);
    };
    const std::vector<int32_t> kmeans_labels
        = common_robotics_utilities::simple_kmeans_clustering::Cluster(
            random_points, distance_fn, average_fn, num_clusters, 42, true,
            use_parallel);
    // Get value clusters from the K-means labels
    std::vector<std::vector<Eigen::VectorXd>> kmeans_clusters(num_clusters);
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
