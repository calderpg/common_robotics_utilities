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

int main(int argc, char** argv)
{
  const size_t num_points
      = (argc >= 2) ? static_cast<size_t>(atoi(argv[1])) : 1000;
  std::cout << "Generating " << num_points << " points..." << std::endl;
  const int64_t seed = 42;
  std::mt19937_64 prng(seed);
  std::uniform_real_distribution<double> dist(0.0, 10.0);
  std::vector<Eigen::VectorXd> random_points(num_points);
  std::vector<size_t> indices(num_points);
  for (size_t idx = 0; idx < num_points; idx++)
  {
    const double x = dist(prng);
    const double y = dist(prng);
    Eigen::VectorXd random_point(3);
    random_point << x, y, 0.0;
    random_points[idx] = random_point;
    indices[idx] = idx;
  }
  const std::vector<bool> use_parallel_options = {false, true};
  for (const bool use_parallel : use_parallel_options)
  {
    std::cout << "Complete-link hierarchical clustering " << num_points
              << " points..." << std::endl;
    std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&)>
        distance_fn = [] (const Eigen::VectorXd& v1, const Eigen::VectorXd& v2)
    {
      return common_robotics_utilities::math::Distance(v1, v2);
    };
    const Eigen::MatrixXd distance_matrix
        = common_robotics_utilities::math
            ::BuildPairwiseDistanceMatrixParallel(random_points, distance_fn);
    const std::vector<std::vector<size_t>> hierarchical_index_clusters
        = common_robotics_utilities::simple_hierarchical_clustering
            ::ClusterWithDistanceMatrix<int64_t>(
                indices, distance_matrix, 1.0,
                common_robotics_utilities::simple_hierarchical_clustering
                    ::ClusterStrategy::COMPLETE_LINK, use_parallel).first;
    std::vector<Eigen::VectorXd> hierarchical_clustered_points = random_points;
    for (size_t cluster_idx = 0;
         cluster_idx < hierarchical_index_clusters.size(); cluster_idx++)
    {
      const std::vector<size_t>& current_cluster
          = hierarchical_index_clusters[cluster_idx];
      const double cluster_num = 1.0 + (double)cluster_idx;
      for (size_t element_idx = 0; element_idx < current_cluster.size();
           element_idx++)
      {
        const size_t index = current_cluster[element_idx];
        hierarchical_clustered_points.at(index)(2) = cluster_num;
      }
    }
    std::cout << "K-means clustering " << num_points << " points..."
              << std::endl;
    std::function<Eigen::VectorXd(const std::vector<Eigen::VectorXd>&)>
        average_fn = [] (const std::vector<Eigen::VectorXd>& cluster)
    {
      return common_robotics_utilities::math::AverageEigenVectorXd(cluster);
    };
    const std::vector<int32_t> kmeans_labels
        = common_robotics_utilities::simple_kmeans_clustering::Cluster(
            random_points, distance_fn, average_fn, 50, 42, true, use_parallel);
    std::vector<Eigen::VectorXd> kmeans_clustered_points = random_points;
    for (size_t idx = 0; idx < kmeans_clustered_points.size(); idx++)
    {
      kmeans_clustered_points.at(idx)(2) = kmeans_labels.at(idx);
    }
    std::cout << "Saving to CSV..." << std::endl;
    const std::string hierarchical_log_file_name
        = (use_parallel) ? "/tmp/test_parallel_hierarchical_clustering.csv"
                         : "/tmp/test_hierarchical_clustering.csv";
    std::ofstream hierarchical_log_file(
        hierarchical_log_file_name, std::ios_base::out);
    if (!hierarchical_log_file.is_open())
    {
      std::cerr << "\x1b[31;1m Unable to create folder/file to log to: "
                << hierarchical_log_file_name << "\x1b[0m \n";
      throw std::invalid_argument("Log filename must be write-openable");
    }
    for (size_t idx = 0; idx < num_points; idx++)
    {
      const Eigen::VectorXd& point = hierarchical_clustered_points.at(idx);
      hierarchical_log_file << point(0) << "," << point(1) << "," << point(2)
                            << std::endl;
    }
    hierarchical_log_file.close();
    const std::string kmeans_log_file_name
        = (use_parallel) ? "/tmp/test_parallel_kmeans_clustering.csv"
                         : "/tmp/test_kmeans_clustering.csv";
    std::ofstream kmeans_log_file(kmeans_log_file_name, std::ios_base::out);
    if (!kmeans_log_file.is_open())
    {
      std::cerr << "\x1b[31;1m Unable to create folder/file to log to: "
                << kmeans_log_file_name << "\x1b[0m \n";
      throw std::invalid_argument("Log filename must be write-openable");
    }
    for (size_t idx = 0; idx < num_points; idx++)
    {
      const Eigen::VectorXd& point = kmeans_clustered_points.at(idx);
      kmeans_log_file << point(0) << "," << point(1) << "," << point(2)
                      << std::endl;
    }
    kmeans_log_file.close();
  }
  std::cout << "Done saving, you can plot as a 3d scatterplot to see clustering"
            << std::endl;
  return 0;
}
