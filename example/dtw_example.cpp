#include <chrono>
#include <functional>
#include <iostream>
#include <random>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/math.hpp>
#include <common_robotics_utilities/simple_dtw.hpp>

// We're using Vector4d to make sure alignment is handled properly for builds
// using -march=native and fixed-size-vectorizable Eigen types.
using PointVector = common_robotics_utilities::math::VectorVector4d;
using PointVectorDTW = common_robotics_utilities::simple_dtw::SimpleDTW<
    Eigen::Vector4d, Eigen::Vector4d, PointVector, PointVector>;

PointVector SampleRandomPointVector(
    const size_t num_points, std::mt19937_64& prng,
    std::uniform_real_distribution<double>& distribution)
{
  std::cout << "Generating " << num_points << " random points" << std::endl;
  PointVector random_points(num_points);
  for (size_t idx = 0; idx < num_points; idx++)
  {
    const double x = distribution(prng);
    const double y = distribution(prng);
    const double z = distribution(prng);
    random_points.at(idx) = Eigen::Vector4d(x, y, z, 1.0);
  }
  return random_points;
}

int main(int, char**)
{
  // Set up PRNG and sampling distribution.
  const int64_t seed = 42;
  std::mt19937_64 prng(seed);
  std::uniform_real_distribution<double> dist(0.0, 10.0);

  // Set up test vectors.
  const PointVector test_vector1 = SampleRandomPointVector(12500, prng, dist);
  const PointVector test_vector2 = SampleRandomPointVector(15000, prng, dist);
  const PointVector test_vector3 = SampleRandomPointVector(17500, prng, dist);
  const PointVector test_vector4 = SampleRandomPointVector(20000, prng, dist);

  // Set up the distance function.
  const std::function<double(const Eigen::Vector4d&, const Eigen::Vector4d&)>
      distance_fn = [] (const Eigen::Vector4d& v1, const Eigen::Vector4d& v2)
  {
    return (v1 - v2).norm();
  };

  // Run a number of DTW queries without reusing the DTW matrix memory.
  const auto uncached_start_time = std::chrono::steady_clock::now();

  {
    const auto point_vector_dtw_distance_fn
        = [&] (const PointVector& vec1, const PointVector& vec2)
    {
      return common_robotics_utilities::simple_dtw::EvaluateWarpingCost<
          Eigen::Vector4d, Eigen::Vector4d, PointVector, PointVector>(
              vec1, vec2, distance_fn);
    };

    const auto vector1_self_distance =
        point_vector_dtw_distance_fn(test_vector1, test_vector1);
    std::cout << "vector1 self-distance = " << vector1_self_distance
              << std::endl;

    const auto vector2_self_distance =
        point_vector_dtw_distance_fn(test_vector2, test_vector2);
    std::cout << "vector2 self-distance = " << vector2_self_distance
              << std::endl;

    const auto vector3_self_distance =
        point_vector_dtw_distance_fn(test_vector3, test_vector3);
    std::cout << "vector3 self-distance = " << vector3_self_distance
              << std::endl;

    const auto vector4_self_distance =
        point_vector_dtw_distance_fn(test_vector4, test_vector4);
    std::cout << "vector4 self-distance = " << vector4_self_distance
              << std::endl;

    const auto vector1_vector2_distance =
        point_vector_dtw_distance_fn(test_vector1, test_vector2);
    std::cout << "vector1-vector2 distance = " << vector1_vector2_distance
              << std::endl;

    const auto vector2_vector3_distance =
        point_vector_dtw_distance_fn(test_vector2, test_vector3);
    std::cout << "vector2-vector3 distance = " << vector2_vector3_distance
              << std::endl;

    const auto vector3_vector4_distance =
        point_vector_dtw_distance_fn(test_vector3, test_vector4);
    std::cout << "vector3-vector4 distance = " << vector3_vector4_distance
              << std::endl;

    const auto vector4_vector1_distance =
        point_vector_dtw_distance_fn(test_vector4, test_vector1);
    std::cout << "vector4-vector1 distance = " << vector4_vector1_distance
              << std::endl;
  }

  const auto uncached_end_time = std::chrono::steady_clock::now();
  const double uncached_elapsed = std::chrono::duration<double>(
      uncached_end_time - uncached_start_time).count();

  std::cout << "DTW queries runtime: " << uncached_elapsed
            << " seconds (without memory reuse)" << std::endl;

  // Run the same set of DTW queries reusing the DTW matrix memory.
  const auto cached_start_time = std::chrono::steady_clock::now();

  {
    // Create a DTW evaluator with the known maximum sequence size.
    PointVectorDTW dtw_evaluator(2000, 2000);

    const auto vector1_self_distance = dtw_evaluator.EvaluateWarpingCost(
        test_vector1, test_vector1, distance_fn);
    std::cout << "vector1 self-distance = " << vector1_self_distance
              << std::endl;

    const auto vector2_self_distance = dtw_evaluator.EvaluateWarpingCost(
        test_vector2, test_vector2, distance_fn);
    std::cout << "vector2 self-distance = " << vector2_self_distance
              << std::endl;

    const auto vector3_self_distance = dtw_evaluator.EvaluateWarpingCost(
        test_vector3, test_vector3, distance_fn);
    std::cout << "vector3 self-distance = " << vector3_self_distance
              << std::endl;

    const auto vector4_self_distance = dtw_evaluator.EvaluateWarpingCost(
        test_vector4, test_vector4, distance_fn);
    std::cout << "vector4 self-distance = " << vector4_self_distance
              << std::endl;

    const auto vector1_vector2_distance = dtw_evaluator.EvaluateWarpingCost(
        test_vector1, test_vector2, distance_fn);
    std::cout << "vector1-vector2 distance = " << vector1_vector2_distance
              << std::endl;

    const auto vector2_vector3_distance = dtw_evaluator.EvaluateWarpingCost(
        test_vector2, test_vector3, distance_fn);
    std::cout << "vector2-vector3 distance = " << vector2_vector3_distance
              << std::endl;

    const auto vector3_vector4_distance = dtw_evaluator.EvaluateWarpingCost(
        test_vector3, test_vector4, distance_fn);
    std::cout << "vector3-vector4 distance = " << vector3_vector4_distance
              << std::endl;

    const auto vector4_vector1_distance = dtw_evaluator.EvaluateWarpingCost(
        test_vector4, test_vector1, distance_fn);
    std::cout << "vector4-vector1 distance = " << vector4_vector1_distance
              << std::endl;
  }

  const auto cached_end_time = std::chrono::steady_clock::now();
  const double cached_elapsed = std::chrono::duration<double>(
      cached_end_time - cached_start_time).count();

  std::cout << "DTW queries runtime: " << cached_elapsed
            << " seconds (with memory reuse)" << std::endl;

  return 0;
}
