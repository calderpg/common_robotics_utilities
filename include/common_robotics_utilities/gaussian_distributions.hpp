#pragma once

#include <cmath>
#include <cstdint>
#include <random>

#include <Eigen/Geometry>

namespace common_robotics_utilities
{
namespace gaussian_distributions
{
/// See https://people.sc.fsu.edu/~jburkardt/presentations/truncated_normal.pdf
/// for details on the implementations here.

/// Evaluate the value of the CDF of the gaussian distribution specified by
/// @param mean and @param std_dev at @param val.
inline double EvaluateGaussianCDF(
    const double mean, const double std_dev, const double val)
{
  return 0.5 * (1.0 + std::erf(((val - mean) / std_dev) / std::sqrt(2.0)));
}

/// Evaluate the value of the PDF of the gaussian distribution specified by
/// @param mean and @param std_dev at @param val.
inline double EvaluateGaussianPDF(
    const double mean, const double std_dev, const double val)
{
  const double exponent = ((val - mean) * (val - mean))
                          / (2.0 * std_dev * std_dev);
  const double fraction = 1.0 / (std_dev * std::sqrt(2.0 * M_PI));
  const double pdf = fraction * std::exp(-exponent);
  return pdf;
}

/// Evaluate the value of the CDF of the truncated gaussian distribution
/// specified by @param mean and @param std_dev and bounds @param lower_bound
/// and @param upper_bound at @param val.
inline double EvaluateTruncatedGaussianCDF(
    const double mean, const double lower_bound, const double upper_bound,
    const double std_dev, const double val)
{
  if (lower_bound > upper_bound)
  {
    throw std::invalid_argument("lower_bound > upper_bound");
  }
  if (val <= lower_bound)
  {
    return 0.0;
  }
  else if (val >= upper_bound)
  {
    return 1.0;
  }
  else
  {
    const double cdf_lower_bound
        = EvaluateGaussianCDF(mean, std_dev, lower_bound);
    const double numerator
        = EvaluateGaussianCDF(mean, std_dev, val) - cdf_lower_bound;
    const double denominator
        = EvaluateGaussianCDF(mean, std_dev, upper_bound) - cdf_lower_bound;
    return numerator / denominator;
  }
}

/// Evaluate the value of the PDF of the truncated gaussian distribution
/// specified by @param mean and @param std_dev and bounds @param lower_bound
/// and @param upper_bound at @param val.
inline double EvaluateTruncatedGaussianPDF(
    const double mean, const double lower_bound, const double upper_bound,
    const double std_dev, const double val)
{
  if (lower_bound > upper_bound)
  {
    throw std::invalid_argument("lower_bound > upper_bound");
  }
  if (val <= lower_bound)
  {
    return 0.0;
  }
  else if (val >= upper_bound)
  {
    return 0.0;
  }
  else
  {
    const double cdf_upper = EvaluateGaussianCDF(mean, std_dev, upper_bound);
    const double cdf_lower = EvaluateGaussianCDF(mean, std_dev, lower_bound);
    const double probability_enclosed = cdf_upper - cdf_lower;
    const double gaussian_pdf = EvaluateGaussianPDF(mean, std_dev, val);
    const double pdf = gaussian_pdf / probability_enclosed;
    return pdf;
  }
}

/// Integrate the probability contained between @param lower_limit and @param
/// upper_limit for the gaussian distribution specified by @param mean and
/// @param std_dev.
inline double IntegrateGaussian(
    const double mean, const double std_dev, const double lower_limit,
    const double upper_limit)
{
  if (lower_limit > upper_limit)
  {
    throw std::invalid_argument("lower_limit > upper_limit");
  }
  const double upper_limit_cdf
      = EvaluateGaussianCDF(mean, std_dev, upper_limit);
  const double lower_limit_cdf
      = EvaluateGaussianCDF(mean, std_dev, lower_limit);
  const double probability = upper_limit_cdf - lower_limit_cdf;
  return probability;
}

/// Integrate the probability contained between @param lower_limit and @param
/// upper_limit for the truncated gaussian distribution specified by @param mean
/// and @param std_dev and bounds @param lower_bound and @param upper_bound.
inline double IntegrateTruncatedGaussian(
    const double mean, const double lower_bound, const double upper_bound,
    const double std_dev, const double lower_limit, const double upper_limit)
{
  if (lower_bound > upper_bound)
  {
    throw std::invalid_argument("lower_bound > upper_bound");
  }
  if (lower_limit > upper_limit)
  {
    throw std::invalid_argument("lower_limit > upper_limit");
  }
  const double lower_limit_cdf
      = EvaluateTruncatedGaussianCDF(
          mean, lower_bound, upper_bound, std_dev, lower_limit);
  const double upper_limit_cdf
      = EvaluateTruncatedGaussianCDF(
          mean, lower_bound, upper_bound, std_dev, upper_limit);
  const double probability = upper_limit_cdf - lower_limit_cdf;
  return probability;
}

/// Note that comments come from (and refer to) parts of the original R plugin
/// that this implementation was derived from.
class TruncatedGaussianDistribution
{
private:

  double mean_ = 0.0;
  double stddev_ = 0.0;
  double std_lower_bound_ = 0.0;
  double std_upper_bound_ = 0.0;

  enum class DrawCases {TYPE_1, TYPE_2, TYPE_3, TYPE_4, NONE};
  DrawCases case_ = NONE;
  std::uniform_real_distribution<double> uniform_unit_dist_;
  std::uniform_real_distribution<double> uniform_range_dist_;
  std::exponential_distribution<double> exponential_dist_;
  std::normal_distribution<double> normal_dist_;

  bool CheckSimple(const double lower_bound, const double upper_bound) const
  {
    // Init Values Used in Inequality of Interest
    const double val1
        = (2 * std::sqrt(std::exp(1)))
          / (lower_bound + std::sqrt(std::pow(lower_bound, 2) + 4));
    const double val2
        = std::exp((std::pow(lower_bound, 2)
                    - lower_bound * std::sqrt(std::pow(lower_bound, 2) + 4))
                   / (4));
    if (upper_bound > lower_bound + val1 * val2)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  // Naive Accept-Reject algorithm
  template<typename Generator>
  double NaiveAcceptReject(
      const double lower_bound, const double upper_bound, Generator& prng)
  {
    while (true)
    {
      // In the R plugin, this was a call to Rf_rnorm(0.0, 1.0), in pure C++ it
      // is a call to std::normal_distribution<double>.
      const double draw = normal_dist_(prng);
      if ((draw <= upper_bound) && (draw >= lower_bound))
      {
          return draw;
      }
    }
  }

  // Accept-Reject Algorithm
  template<typename Generator>
  double SimpleAcceptReject(const double lower_bound, Generator& prng)
  {
    // Init Values
    const double alpha
        = (lower_bound + std::sqrt(std::pow(lower_bound, 2) + 4.0)) / (2.0);
    while (true)
    {
      // In the R plugin, this was a call to Rf_rexp(1.0), in pure C++ it is a
      // call to std::exponential_distribution<double>.
      const double e = exponential_dist_(prng);
      const double z = lower_bound + e / alpha;
      const double rho = std::exp(-std::pow(alpha - z, 2) / 2);
      // In the R plugin, this was a call to Rf_rnorm(0.0, 1.0), in pure C++ it
      // is a call to std::normal_distribution<double>.
      const double u = uniform_unit_dist_(prng);
      if (u <= rho)
      {
        return z;
      }
    }
  }

  // Accept-Reject Algorithm
  template<typename Generator>
  double ComplexAcceptReject(
      const double lower_bound, const double upper_bound, Generator& prng)
  {
    while (true)
    {
      // In the R plugin, this was a call to Rf_rnorm(lower_bound, upper_bound),
      // in pure C++ it is a call to std::normal_distribution<double>.
      const double z = uniform_range_dist_(prng);
      double rho = 0.0;
      if (0 < lower_bound)
      {
        rho = std::exp((std::pow(lower_bound, 2) - std::pow(z, 2)) / 2);
      }
      else if (upper_bound < 0)
      {
        rho = std::exp((std::pow(upper_bound, 2) - std::pow(z, 2)) / 2);
      }
      else if (0 < upper_bound && lower_bound < 0)
      {
        rho = std::exp(-std::pow(z, 2) / 2);
      }
      // In the R plugin, this was a call to Rf_rnorm(0.0, 1.0), in pure C++ it
      // is a call to std::normal_distribution<double>.
      const double u = uniform_unit_dist_(prng);
      if (u <= rho)
      {
        return z;
      }
    }
  }

  template<typename Generator>
  double Sample(Generator& prng)
  {
    if (case_ == DrawCases::TYPE_1)
    {
      const double draw
          = NaiveAcceptReject(std_lower_bound_, std_upper_bound_, prng);
      return mean_ + stddev_ * draw;
    }
    else if (case_ == DrawCases::TYPE_2)
    {
      const double draw = SimpleAcceptReject(std_lower_bound_, prng);
      return mean_ + stddev_ * draw;
    }
    else if (case_ == DrawCases::TYPE_3)
    {
      while (true)
      {
        const double draw = SimpleAcceptReject(std_lower_bound_, prng);
        if (draw <= std_upper_bound_)
        {
          return mean_ + stddev_ * draw;
        }
      }
    }
    else if (case_ == DrawCases::TYPE_4)
    {
      const double draw
          = ComplexAcceptReject(std_lower_bound_, std_upper_bound_, prng);
      return mean_ + stddev_ * draw;
    }
    else
    {
      if (case_ == DrawCases::NONE)
      {
        return mean_;
      }
      else
      {
        throw std::runtime_error("Invalid case");
      }
    }
  }

public:

  TruncatedGaussianDistribution(
      const double mean, const double stddev, const double lower_bound,
      const double upper_bound)
    : uniform_unit_dist_(0.0, 1.0),
      uniform_range_dist_(lower_bound, upper_bound),
      exponential_dist_(1.0), normal_dist_(0.0, 1.0)
  {
    // Set operating parameters
    mean_ = mean;
    stddev_ = stddev;
    if (std::abs(stddev_) == 0.0)
    {
      case_ = DrawCases::NONE;
    }
    else
    {
      // Standardize the lower and upper bounds
      std_lower_bound_ = (lower_bound - mean_) / stddev_;
      std_upper_bound_ = (upper_bound - mean_) / stddev_;
      // Set the operating case - i.e. which sampling method we will use
      case_ = DrawCases::NONE;
      if (0.0 <= std_upper_bound_ && 0.0 >= std_lower_bound_)
      {
        case_ = DrawCases::TYPE_1;
      }
      if (0.0 < std_lower_bound_
          && std_upper_bound_ == std::numeric_limits<double>::infinity())
      {
        case_ = DrawCases::TYPE_2;
      }
      if (0.0 > std_upper_bound_
          && std_lower_bound_ == -std::numeric_limits<double>::infinity())
      {
        std_lower_bound_ = -1 * std_upper_bound_;
        std_upper_bound_ = std::numeric_limits<double>::infinity();
        stddev_ = -1 * stddev_;
        case_ = DrawCases::TYPE_2;
      }
      if ((0.0 > std_upper_bound_ || 0.0 < std_lower_bound_)
          && !(std_upper_bound_ == std::numeric_limits<double>::infinity()
               || std_lower_bound_ == -std::numeric_limits<double>::infinity()))
      {
        if (CheckSimple(std_lower_bound_, std_upper_bound_))
        {
          case_ = DrawCases::TYPE_3;
        }
        else
        {
          case_ = DrawCases::TYPE_4;
        }
      }
      if ((case_ != DrawCases::TYPE_1) && (case_ != DrawCases::TYPE_2)
          && (case_ != DrawCases::TYPE_3) && (case_ != DrawCases::TYPE_4))
      {
        throw std::invalid_argument("case cannot be NONE with stddev == 0");
      }
    }
  }

  template<typename Generator>
  double operator()(Generator& prng)
  {
    return Sample(prng);
  }
};

/// Simple multivariate gaussian distribution.
class MultivariteGaussianDistribution
{
private:
  const Eigen::VectorXd mean_;
  const Eigen::MatrixXd norm_transform_;

  std::normal_distribution<double> unit_gaussian_dist_;

  template<typename Generator>
  Eigen::VectorXd Sample(Generator& prng)
  {
    Eigen::VectorXd draw;
    draw.resize(mean_.rows());
    for (ssize_t idx = 0; idx < draw.rows(); idx++)
    {
      draw(idx) = unit_gaussian_dist_(prng);
    }
    return norm_transform_ * draw + mean_;
  }

  static Eigen::MatrixXd CalculateNormTransform(
      const Eigen::MatrixXd& covariance)
  {
    Eigen::MatrixXd norm_transform;
    Eigen::LLT<Eigen::MatrixXd> chol_solver(covariance);
    if (chol_solver.info() == Eigen::Success)
    {
      // Use cholesky solver
      norm_transform = chol_solver.matrixL();
    }
    else
    {
      // Use eigen solver
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(covariance);
      norm_transform
          = eigen_solver.eigenvectors()
            * eigen_solver.eigenvalues().cwiseMax(0.0).cwiseSqrt().asDiagonal();
    }
    return norm_transform;
  }

public:
  MultivariteGaussianDistribution(
      const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance)
      : mean_(mean), norm_transform_(CalculateNormTransform(covariance)),
        unit_gaussian_dist_(0.0, 1.0)
  {
    if (mean_.rows() != covariance.rows() || mean_.cols() != covariance.cols())
    {
      throw std::invalid_argument("mean and covariance are different sizes");
    }
    const auto nan_check = [] (const double& val) { return std::isnan(val); };
    if ((norm_transform_.unaryExpr(nan_check)).any())
    {
      throw std::runtime_error(
          "NaN Found in norm_transform in MultivariateGaussianDistribution");
    }
    const auto inf_check = [] (const double& val) { return std::isinf(val); };
    if ((norm_transform_.unaryExpr(inf_check)).any())
    {
      throw std::runtime_error(
          "Inf Found in norm_transform in MultivariateGaussianDistribution");
    }
  }

  template<typename Generator>
  Eigen::VectorXd operator()(Generator& prng)
  {
    return Sample(prng);
  }
};
}  // namespace gaussian_distributions
}  // namespace common_robotics_utilities
