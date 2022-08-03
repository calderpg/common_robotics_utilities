#pragma once

#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/openmp_helpers.hpp>

namespace common_robotics_utilities
{
/// Compute Hausdorff distance between two distributions.
/// "The Hausdorff distance is the longest distance you can be forced to travel
/// by an adversary who chooses a point in one of the two sets, from where you
/// then must travel to the other set. In other words, it is the greatest of all
/// the distances from a point in one set to the closest point in the other
/// set." - from Wikipedia
namespace simple_hausdorff_distance
{
/// Compute Hausdorff distance between @param first_distribution and @param
/// second_distribution using @param distance_function to compute distance
/// between an element in @param first_distribution and an element in @param
/// second_distribution. @return distance between distributions.
/// Computation is performed in parallel, and the larger of the two input
/// distributions is automatically selected to be the outer (parallelized) loop.
template<typename FirstDatatype, typename SecondDatatype,
         typename FirstContainer=std::vector<FirstDatatype>,
         typename SecondContainer=std::vector<SecondDatatype>>
inline double ComputeDistanceParallel(
    const FirstContainer& first_distribution,
    const SecondContainer& second_distribution,
    const std::function<double(const FirstDatatype&,
                               const SecondDatatype&)>& distance_fn)
{
  if (first_distribution.empty())
  {
    throw std::invalid_argument("first_distribution is empty");
  }
  if (second_distribution.empty())
  {
    throw std::invalid_argument("second_distribution is empty");
  }
  // Swap if needed to make the outer parallel loop be larger
  const bool swap = (first_distribution.size() < second_distribution.size());
  const auto& outer_distribution
      = (swap) ? second_distribution : first_distribution;
  const auto& inner_distribution
      = (swap) ? first_distribution : second_distribution;
  // Make per-thread storage
  std::vector<double> per_thread_storage(
      openmp_helpers::GetNumOmpThreads(), 0.0);
  CRU_OMP_PARALLEL_FOR
  for (size_t idx = 0; idx < outer_distribution.size(); idx++)
  {
    const FirstDatatype& first = outer_distribution[idx];
    double minimum_distance = std::numeric_limits<double>::infinity();
    for (size_t jdx = 0; jdx < inner_distribution.size(); jdx++)
    {
      const SecondDatatype& second = inner_distribution[jdx];
      const double current_distance = distance_fn(first, second);
      if (current_distance < minimum_distance)
      {
        minimum_distance = current_distance;
      }
    }
    const auto current_thread_id = openmp_helpers::GetContextOmpThreadNum();
    if (minimum_distance > per_thread_storage.at(current_thread_id))
    {
      per_thread_storage.at(current_thread_id) = minimum_distance;
    }
  }
  double maximum_minimum_distance = 0.0;
  for (const double& temp_minimum_distance : per_thread_storage)
  {
    if (temp_minimum_distance > maximum_minimum_distance)
    {
      maximum_minimum_distance = temp_minimum_distance;
    }
  }
  return maximum_minimum_distance;
}

/// Compute Hausdorff distance between @param first_distribution and @param
/// second_distribution using @param distance_function to compute distance
/// between an element in @param first_distribution and an element in @param
/// second_distribution. @return distance between distributions.
template<typename FirstDatatype, typename SecondDatatype,
         typename FirstContainer=std::vector<FirstDatatype>,
         typename SecondContainer=std::vector<SecondDatatype>>
inline double ComputeDistanceSerial(
    const FirstContainer& first_distribution,
    const SecondContainer& second_distribution,
    const std::function<double(const FirstDatatype&,
                               const SecondDatatype&)>& distance_fn)
{
  if (first_distribution.empty())
  {
    throw std::invalid_argument("first_distribution is empty");
  }
  if (second_distribution.empty())
  {
    throw std::invalid_argument("second_distribution is empty");
  }
  double maximum_minimum_distance = 0.0;
  for (size_t idx = 0; idx < first_distribution.size(); idx++)
  {
    const FirstDatatype& first = first_distribution[idx];
    double minimum_distance = std::numeric_limits<double>::infinity();
    for (size_t jdx = 0; jdx < second_distribution.size(); jdx++)
    {
      const SecondDatatype& second = second_distribution[jdx];
      const double current_distance = distance_fn(first, second);
      if (current_distance < minimum_distance)
      {
        minimum_distance = current_distance;
      }
    }
    if (minimum_distance > maximum_minimum_distance)
    {
      maximum_minimum_distance = minimum_distance;
    }
  }
  return maximum_minimum_distance;
}

/// Compute Hausdorff distance between @param first_distribution and @param
/// second_distribution using @param distance_matrix which stores the pairwise
/// distance between all elements in @param first_distribution and @param
/// second_distribution. @return distance between distributions.
/// Computation is performed in parallel, and the larger of the two input
/// distributions is automatically selected to be the outer (parallelized) loop.
template<typename FirstDatatype, typename SecondDatatype,
         typename FirstContainer=std::vector<FirstDatatype>,
         typename SecondContainer=std::vector<SecondDatatype>>
inline double ComputeDistanceParallel(
    const FirstContainer& first_distribution,
    const SecondContainer& second_distribution,
    const Eigen::MatrixXd& distance_matrix)
{
  if (first_distribution.empty())
  {
    throw std::invalid_argument("first_distribution is empty");
  }
  if (second_distribution.empty())
  {
    throw std::invalid_argument("second_distribution is empty");
  }
  if (static_cast<size_t>(distance_matrix.rows()) != first_distribution.size()
      || static_cast<size_t>(distance_matrix.cols())
         != second_distribution.size())
  {
    throw std::invalid_argument("distance_matrix is the wrong size");
  }
  // Swap if needed to make the outer parallel loop be larger
  const bool swap = (first_distribution.size() < second_distribution.size());
  const auto& outer_distribution
      = (swap) ? second_distribution : first_distribution;
  const auto& inner_distribution
      = (swap) ? first_distribution : second_distribution;
  // Make per-thread storage
  std::vector<double> per_thread_storage(
      openmp_helpers::GetNumOmpThreads(), 0.0);
  CRU_OMP_PARALLEL_FOR
  for (size_t idx = 0; idx < outer_distribution.size(); idx++)
  {
    double minimum_distance = std::numeric_limits<double>::infinity();
    for (size_t jdx = 0; jdx < inner_distribution.size(); jdx++)
    {
      // Swap the lookup if we've swapped the loops
      const ssize_t row = (swap) ? static_cast<ssize_t>(jdx)
                                 : static_cast<ssize_t>(idx);
      const ssize_t col = (swap) ? static_cast<ssize_t>(idx)
                                 : static_cast<ssize_t>(jdx);
      const double current_distance = distance_matrix(row, col);
      if (current_distance < minimum_distance)
      {
        minimum_distance = current_distance;
      }
    }
    const auto current_thread_id = openmp_helpers::GetContextOmpThreadNum();
    if (minimum_distance > per_thread_storage.at(current_thread_id))
    {
      per_thread_storage.at(current_thread_id) = minimum_distance;
    }
  }
  double maximum_minimum_distance = 0.0;
  for (const double& temp_minimum_distance : per_thread_storage)
  {
    if (temp_minimum_distance > maximum_minimum_distance)
    {
      maximum_minimum_distance = temp_minimum_distance;
    }
  }
  return maximum_minimum_distance;
}

/// Compute Hausdorff distance between @param first_distribution and @param
/// second_distribution using @param distance_matrix which stores the pairwise
/// distance between all elements in @param first_distribution and @param
/// second_distribution. @return distance between distributions.
template<typename FirstDatatype, typename SecondDatatype,
         typename FirstContainer=std::vector<FirstDatatype>,
         typename SecondContainer=std::vector<SecondDatatype>>
inline double ComputeDistanceSerial(
    const FirstContainer& first_distribution,
    const SecondContainer& second_distribution,
    const Eigen::MatrixXd& distance_matrix)
{
  if (first_distribution.empty())
  {
    throw std::invalid_argument("first_distribution is empty");
  }
  if (second_distribution.empty())
  {
    throw std::invalid_argument("second_distribution is empty");
  }
  if (static_cast<size_t>(distance_matrix.rows()) != first_distribution.size()
      || static_cast<size_t>(distance_matrix.cols())
         != second_distribution.size())
  {
    throw std::invalid_argument("distance_matrix is the wrong size");
  }
  double maximum_minimum_distance = 0.0;
  for (size_t idx = 0; idx < first_distribution.size(); idx++)
  {
    double minimum_distance = std::numeric_limits<double>::infinity();
    for (size_t jdx = 0; jdx < second_distribution.size(); jdx++)
    {
      const double current_distance
          = distance_matrix(static_cast<ssize_t>(idx),
                            static_cast<ssize_t>(jdx));
      if (current_distance < minimum_distance)
      {
        minimum_distance = current_distance;
      }
    }
    if (minimum_distance > maximum_minimum_distance)
    {
      maximum_minimum_distance = minimum_distance;
    }
  }
  return maximum_minimum_distance;
}

/// Compute Hausdorff distance between @param first_distribution and @param
/// second_distribution using @param distance_function to compute distance
/// between an element in @param first_distribution and an element in @param
/// second_distribution. @return distance between distributions.
/// @param use_parallel selects if the computation should be performed in
/// parallel. If computation is performed in parallel, the larger of the two
/// input distributions is automatically selected to be the outer (parallelized)
/// loop.
template<typename FirstDatatype, typename SecondDatatype,
         typename FirstContainer=std::vector<FirstDatatype>,
         typename SecondContainer=std::vector<SecondDatatype>>
inline double ComputeDistance(
    const FirstContainer& first_distribution,
    const SecondContainer& second_distribution,
    const std::function<double(const FirstDatatype&,
                               const SecondDatatype&)>& distance_fn,
    const bool use_parallel = false)
{
  if (use_parallel)
  {
    return ComputeDistanceParallel
        <FirstDatatype, SecondDatatype, FirstContainer, SecondContainer>(
            first_distribution, second_distribution, distance_fn);
  }
  else
  {
    return ComputeDistanceSerial
        <FirstDatatype, SecondDatatype, FirstContainer, SecondContainer>(
            first_distribution, second_distribution, distance_fn);
  }
}

/// Compute Hausdorff distance between @param first_distribution and @param
/// second_distribution using @param distance_matrix which stores the pairwise
/// distance between all elements in @param first_distribution and @param
/// second_distribution. @return distance between distributions.
/// @param use_parallel selects if the computation should be performed in
/// parallel. If computation is performed in parallel, the larger of the two
/// input distributions is automatically selected to be the outer (parallelized)
/// loop.
template<typename FirstDatatype, typename SecondDatatype,
         typename FirstContainer=std::vector<FirstDatatype>,
         typename SecondContainer=std::vector<SecondDatatype>>
inline double ComputeDistance(
    const FirstContainer& first_distribution,
    const SecondContainer& second_distribution,
    const Eigen::MatrixXd& distance_matrix,
    const bool use_parallel = false)
{
  if (use_parallel)
  {
    return ComputeDistanceParallel
        <FirstDatatype, SecondDatatype, FirstContainer, SecondContainer>(
            first_distribution, second_distribution, distance_matrix);
  }
  else
  {
    return ComputeDistanceSerial
        <FirstDatatype, SecondDatatype, FirstContainer, SecondContainer>(
            first_distribution, second_distribution, distance_matrix);
  }
}
}  // namespace simple_hausdorff_distance
}  // namespace common_robotics_utilities
