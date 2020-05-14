#pragma once

#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>

namespace common_robotics_utilities
{
namespace simple_dtw
{
template<typename FirstDatatype, typename SecondDatatype,
         typename FirstContainer=std::vector<FirstDatatype>,
         typename SecondContainer=std::vector<SecondDatatype>>
class SimpleDTW
{
protected:
  void InitializeMatrix(const ssize_t first_sequence_size,
                        const ssize_t second_sequence_size)
  {
    const ssize_t rows = first_sequence_size + 1;
    const ssize_t cols = second_sequence_size + 1;
    if (dtw_matrix_.rows() < rows || dtw_matrix_.cols() < cols)
    {
      dtw_matrix_ = Eigen::MatrixXd::Zero(rows, cols);
      if (rows > 1 && cols > 1)
      {
        for (ssize_t row = 1; row < rows; row++)
        {
          dtw_matrix_(row, 0) = std::numeric_limits<double>::infinity();
        }
        for (ssize_t col = 1; col < cols; col++)
        {
          dtw_matrix_(0, col) = std::numeric_limits<double>::infinity();
        }
      }
    }
  }

  Eigen::MatrixXd dtw_matrix_;

public:
  SimpleDTW()
  {
    InitializeMatrix(0, 0);
  }

  SimpleDTW(const ssize_t first_sequence_size,
            const ssize_t second_sequence_size)
  {
    InitializeMatrix(first_sequence_size, second_sequence_size);
  }

  double EvaluateWarpingCost(
      const FirstContainer& first_sequence,
      const SecondContainer& second_sequence,
      const std::function<double(const FirstDatatype&,
                                 const SecondDatatype&)>& distance_fn)
  {
    if (first_sequence.empty())
    {
      throw std::invalid_argument("first_sequence is empty");
    }
    if (second_sequence.empty())
    {
      throw std::invalid_argument("second_sequence is empty");
    }
    const ssize_t first_sequence_size
        = static_cast<ssize_t>(first_sequence.size());
    const ssize_t second_sequence_size
        = static_cast<ssize_t>(second_sequence.size());
    InitializeMatrix(first_sequence_size, second_sequence_size);
    // Compute DTW cost for the two sequences
    for (ssize_t i = 1; i <= first_sequence_size; i++)
    {
      const FirstDatatype& first_item
          = first_sequence[static_cast<size_t>(i - 1)];
      for (ssize_t j = 1; j <= second_sequence_size; j++)
      {
        const SecondDatatype& second_item
            = second_sequence[static_cast<size_t>(j - 1)];
        const double index_cost = distance_fn(first_item, second_item);
        double prev_cost = 0.0;
        // Get the next neighboring values from the matrix to use for the update
        const double im1j = dtw_matrix_(i - 1, j);
        const double im1jm1 = dtw_matrix_(i - 1, j - 1);
        const double ijm1 = dtw_matrix_(i, j - 1);
        // Start the update step
        if (im1j < im1jm1 && im1j < ijm1)
        {
          prev_cost = im1j;
        }
        else if (ijm1 < im1j && ijm1 < im1jm1)
        {
          prev_cost = ijm1;
        }
        else
        {
          prev_cost = im1jm1;
        }
        // Update the value in the matrix
        const double new_cost = index_cost + prev_cost;
        dtw_matrix_(i, j) = new_cost;
      }
    }
    //Return total path cost
    const double warping_cost
        = dtw_matrix_(first_sequence_size, second_sequence_size);
    return warping_cost;
  }
};

template<typename FirstDatatype, typename SecondDatatype,
         typename FirstContainer=std::vector<FirstDatatype>,
         typename SecondContainer=std::vector<SecondDatatype>>
inline double EvaluateWarpingCost(
    const FirstContainer& first_sequence,
    const SecondContainer& second_sequence,
    const std::function<double(const FirstDatatype&,
                               const SecondDatatype&)>& distance_fn)
{
  SimpleDTW<FirstDatatype, SecondDatatype,
            FirstContainer, SecondContainer> dtw_evaluator;
  return dtw_evaluator.EvaluateWarpingCost(
      first_sequence, second_sequence, distance_fn);
}
}  // namespace simple_dtw
}  // namespace common_robotics_utilities
