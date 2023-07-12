#include <cmath>
#include <functional>
#include <iostream>
#include <vector>

#include <gtest/gtest.h>
#include <common_robotics_utilities/math.hpp>
#include <common_robotics_utilities/simple_hausdorff_distance.hpp>

namespace common_robotics_utilities
{
namespace
{
class HausdorffDistanceTestSuite
    : public testing::TestWithParam<openmp_helpers::DegreeOfParallelism> {};

TEST_P(HausdorffDistanceTestSuite, Test)
{
  const openmp_helpers::DegreeOfParallelism parallelism = GetParam();
  std::cout << "# of threads = " << parallelism.GetNumThreads() << std::endl;

  const std::vector<double> d1 = {1.5, 2.5, 3.5, 4.5, 5.5};
  const std::vector<double> d2 = {10.5, 11.5, 12.5, 13.5, 14.5, 15.5};

  const std::vector<int32_t> i1 = {1, 2, 3, 4, 5};
  const std::vector<int32_t> i2 = {10, 11, 12, 13, 14, 15};

  const std::function<double(const double&, const double&)> dd_dist_fn = [](
      const double& first, const double& second)
  {
    return std::abs(first - second);
  };

  const std::function<double(const double&, const int32_t&)> di_dist_fn = [](
      const double& first, const int32_t& second)
  {
    return std::abs(first - static_cast<double>(second));
  };

  const std::function<double(const int32_t&, const double&)> id_dist_fn = [](
      const int32_t& first, const double& second)
  {
    return std::abs(static_cast<double>(first) - second);
  };

  const std::function<double(const int32_t&, const int32_t&)> ii_dist_fn = [](
      const double& first, const double& second)
  {
    return static_cast<double>(std::abs(first - second));
  };

  // Compute pairwise distance matrices
  const Eigen::MatrixXd d1d2_dist_matrix = math::BuildPairwiseDistanceMatrix(
      d1, d2, dd_dist_fn, parallelism);
  const Eigen::MatrixXd d1i2_dist_matrix = math::BuildPairwiseDistanceMatrix(
      d1, i2, di_dist_fn, parallelism);
  const Eigen::MatrixXd i1d2_dist_matrix = math::BuildPairwiseDistanceMatrix(
      i1, d2, id_dist_fn, parallelism);
  const Eigen::MatrixXd i1i2_dist_matrix = math::BuildPairwiseDistanceMatrix(
      i1, i2, ii_dist_fn, parallelism);

  const Eigen::MatrixXd d1d1_dist_matrix = math::BuildPairwiseDistanceMatrix(
      d1, d1, dd_dist_fn, parallelism);
  const Eigen::MatrixXd i1i1_dist_matrix = math::BuildPairwiseDistanceMatrix(
      i1, i1, ii_dist_fn, parallelism);

  // Compute distribution-distribution distances
  const double func_dist_d1d2 = simple_hausdorff_distance::ComputeDistance(
      d1, d2, dd_dist_fn, parallelism);
  const double matrix_dist_d1d2 =
      simple_hausdorff_distance::ComputeDistance<double, double>(
          d1, d2, d1d2_dist_matrix, parallelism);
  EXPECT_EQ(func_dist_d1d2, matrix_dist_d1d2);
  EXPECT_EQ(func_dist_d1d2, 9.0);

  const double func_dist_d1i2 = simple_hausdorff_distance::ComputeDistance(
      d1, i2, di_dist_fn, parallelism);
  const double matrix_dist_d1i2 =
      simple_hausdorff_distance::ComputeDistance<double, int32_t>(
          d1, i2, d1i2_dist_matrix, parallelism);
  EXPECT_EQ(func_dist_d1i2, matrix_dist_d1i2);
  EXPECT_EQ(func_dist_d1i2, 8.5);

  const double func_dist_i1d2 = simple_hausdorff_distance::ComputeDistance(
      i1, d2, id_dist_fn, parallelism);
  const double matrix_dist_i1d2 =
      simple_hausdorff_distance::ComputeDistance<int32_t, double>(
          i1, d2, i1d2_dist_matrix, parallelism);
  EXPECT_EQ(func_dist_i1d2, matrix_dist_i1d2);
  EXPECT_EQ(func_dist_i1d2, 9.5);

  const double func_dist_i1i2 = simple_hausdorff_distance::ComputeDistance(
      i1, i2, ii_dist_fn, parallelism);
  const double matrix_dist_i1i2 =
      simple_hausdorff_distance::ComputeDistance<int32_t, int32_t>(
          i1, i2, i1i2_dist_matrix, parallelism);
  EXPECT_EQ(func_dist_i1i2, matrix_dist_i1i2);
  EXPECT_EQ(func_dist_i1i2, 9.0);

  const double func_dist_d1d1 = simple_hausdorff_distance::ComputeDistance(
      d1, d1, dd_dist_fn, parallelism);
  const double matrix_dist_d1d1 =
      simple_hausdorff_distance::ComputeDistance<double, double>(
          d1, d1, d1d1_dist_matrix, parallelism);
  EXPECT_EQ(func_dist_d1d1, matrix_dist_d1d1);
  EXPECT_EQ(func_dist_d1d1, 0.0);

  const double func_dist_i1i1 = simple_hausdorff_distance::ComputeDistance(
      i1, i1, ii_dist_fn, parallelism);
  const double matrix_dist_i1i1 =
      simple_hausdorff_distance::ComputeDistance<int32_t, int32_t>(
          i1, i1, i1i1_dist_matrix, parallelism);
  EXPECT_EQ(func_dist_i1i1, matrix_dist_i1i1);
  EXPECT_EQ(func_dist_i1i1, 0.0);
}

INSTANTIATE_TEST_SUITE_P(
    SerialHausdorffDistanceTest, HausdorffDistanceTestSuite,
    testing::Values(openmp_helpers::DegreeOfParallelism::None()));

INSTANTIATE_TEST_SUITE_P(
    ParallelHausdorffDistanceTest, HausdorffDistanceTestSuite,
    testing::Values(openmp_helpers::DegreeOfParallelism::FromOmp()));
}  // namespace
}  // namespace common_robotics_utilities

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

