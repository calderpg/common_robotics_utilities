#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>
#include <common_robotics_utilities/math.hpp>
#include <common_robotics_utilities/utility.hpp>

namespace common_robotics_utilities
{
namespace math
{
bool Equal3d(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
  if ((v1.x() == v2.x()) && (v1.y() == v2.y()) && (v1.z() == v2.z()))
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool Equal4d(const Eigen::Vector4d& v1, const Eigen::Vector4d& v2)
{
  if ((v1(0) == v2(0)) && (v1(1) == v2(1))
      && (v1(2) == v2(2)) && (v1(3) == v2(3)))
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool CloseEnough(const double p1, const double p2, const double threshold)
{
  const double real_threshold = std::abs(threshold);
  const double abs_delta = std::abs(p2 - p1);
  if (abs_delta <= real_threshold)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool CloseEnough(const Eigen::Vector3d& v1,
                 const Eigen::Vector3d& v2,
                 const double threshold)
{
  const double real_threshold = std::abs(threshold);
  if (std::abs(v1.x() - v2.x()) > real_threshold)
  {
    return false;
  }
  if (std::abs(v1.y() - v2.y()) > real_threshold)
  {
    return false;
  }
  if (std::abs(v1.z() - v2.z()) > real_threshold)
  {
    return false;
  }
  return true;
}

double EnforceContinuousRevoluteBounds(const double value)
{
  if ((value <= -M_PI) || (value > M_PI))
  {
    const double remainder = std::fmod(value, 2.0 * M_PI);
    if (remainder <= -M_PI)
    {
      return (remainder + (2.0 * M_PI));
    }
    else if (remainder > M_PI)
    {
      return (remainder - (2.0 * M_PI));
    }
    else
    {
      return remainder;
    }
  }
  else
  {
    return value;
  }
}

double SquaredNorm(const std::vector<double>& vec)
{
  double squared_norm = 0.0;
  for (size_t idx = 0; idx < vec.size(); idx++)
  {
    const double element = vec[idx];
    squared_norm += (element * element);
  }
  return squared_norm;
}

double Norm(const std::vector<double>& vec)
{
  return std::sqrt(SquaredNorm(vec));
}

std::vector<double> Abs(const std::vector<double>& vec)
{
  std::vector<double> absed(vec.size(), 0.0);
  for (size_t idx = 0; idx < absed.size(); idx++)
  {
    absed[idx] = std::abs(vec[idx]);
  }
  return absed;
}

std::vector<double> Multiply(const std::vector<double>& vec,
                             const double scalar)
{
  std::vector<double> multiplied(vec.size(), 0.0);
  for (size_t idx = 0; idx < multiplied.size(); idx++)
  {
    const double element = vec[idx];
    multiplied[idx] = element * scalar;
  }
  return multiplied;
}

std::vector<double> Multiply(const std::vector<double>& vec1,
                             const std::vector<double>& vec2)
{
  if (vec1.size() == vec2.size())
  {
    std::vector<double> multiplied(vec1.size(), 0.0);
    for (size_t idx = 0; idx < multiplied.size(); idx++)
    {
      const double element1 = vec1[idx];
      const double element2 = vec2[idx];
      multiplied[idx] = element1 * element2;
    }
    return multiplied;
  }
  else
  {
    throw std::invalid_argument("vec1.size() != vec2.size()");
  }
}

std::vector<double> Divide(const std::vector<double>& vec, const double scalar)
{
  const double inv_scalar = 1.0 / scalar;
  return Multiply(vec, inv_scalar);
}

std::vector<double> Divide(const std::vector<double>& vec1,
                           const std::vector<double>& vec2)
{
  if (vec1.size() == vec2.size())
  {
    std::vector<double> divided(vec1.size(), 0.0);
    for (size_t idx = 0; idx < divided.size(); idx++)
    {
      const double element1 = vec1[idx];
      const double element2 = vec2[idx];
      divided[idx] = element1 / element2;
    }
    return divided;
  }
  else
  {
    throw std::invalid_argument("vec1.size() != vec2.size()");
  }
}

std::vector<double> Add(const std::vector<double>& vec, const double scalar)
{
  std::vector<double> added(vec.size(), 0.0);
  for (size_t idx = 0; idx < added.size(); idx++)
  {
    added[idx] = vec[idx] + scalar;
  }
  return added;
}

std::vector<double> Add(const std::vector<double>& vec1,
                        const std::vector<double>& vec2)
{
  if (vec1.size() == vec2.size())
  {
    std::vector<double> added(vec1.size(), 0.0);
    for (size_t idx = 0; idx < added.size(); idx++)
    {
      const double element1 = vec1[idx];
      const double element2 = vec2[idx];
      added[idx] = element1 + element2;
    }
    return added;
  }
  else
  {
    throw std::invalid_argument("vec1.size() != vec2.size()");
  }
}

std::vector<double> Sub(const std::vector<double>& vec, const double scalar)
{
  std::vector<double> subed(vec.size(), 0.0);
  for (size_t idx = 0; idx < subed.size(); idx++)
  {
    subed[idx] = vec[idx] - scalar;
  }
  return subed;
}

std::vector<double> Sub(const std::vector<double>& vec1,
                        const std::vector<double>& vec2)
{
  if (vec1.size() == vec2.size())
  {
    std::vector<double> subed(vec1.size(), 0.0);
    for (size_t idx = 0; idx < subed.size(); idx++)
    {
      const double element1 = vec1[idx];
      const double element2 = vec2[idx];
      subed[idx] = element1 - element2;
    }
    return subed;
  }
  else
  {
    throw std::invalid_argument("vec1.size() != vec2.size()");
  }
}

double Sum(const std::vector<double>& vec)
{
  double sum = 0.0;
  for (size_t idx = 0; idx < vec.size(); idx++)
  {
    const double element = vec[idx];
    sum += element;
  }
  return sum;
}

Eigen::Matrix3d Skew(const Eigen::Vector3d& vector)
{
  Eigen::Matrix3d skewed;
  skewed << 0.0, -vector.z(), vector.y(),
        vector.z(), 0.0, -vector.x(),
        -vector.y(), vector.x(), 0.0;
  return skewed;
}

Eigen::Vector3d Unskew(const Eigen::Matrix3d& matrix)
{
  const Eigen::Matrix3d matrix_symetric = (matrix - matrix.transpose()) / 2.0;
  const Eigen::Vector3d unskewed(matrix_symetric(2, 1),
                                 matrix_symetric(0, 2),
                                 matrix_symetric(1, 0));
  return unskewed;
}

Eigen::Matrix4d TwistHat(const Eigen::Matrix<double, 6, 1>& twist)
{
  const Eigen::Vector3d trans_velocity = twist.segment<3>(0);
  const Eigen::Matrix3d hatted_rot_velocity = Skew(twist.segment<3>(3));
  Eigen::Matrix4d hatted_twist = Eigen::Matrix4d::Zero();
  hatted_twist.block<3, 3>(0, 0) = hatted_rot_velocity;
  hatted_twist.block<3, 1>(0, 3) = trans_velocity;
  return hatted_twist;
}

Eigen::Matrix<double, 6, 1> TwistUnhat(const Eigen::Matrix4d& hatted_twist)
{
   const Eigen::Vector3d trans_velocity = hatted_twist.block<3, 1>(0, 3);
   const Eigen::Vector3d rot_velocity = Unskew(hatted_twist.block<3, 3>(0, 0));
   Eigen::Matrix<double, 6, 1> twist;
   twist.segment<3>(0) = trans_velocity;
   twist.segment<3>(3) = rot_velocity;
   return twist;
}

Eigen::Matrix<double, 6, 6> AdjointFromTransform(
    const Eigen::Isometry3d& transform)
{
  const Eigen::Matrix3d rotation = transform.matrix().block<3, 3>(0, 0);
  const Eigen::Vector3d translation = transform.matrix().block<3, 1>(0, 3);
  const Eigen::Matrix3d translation_hat = Skew(translation);
  // Assemble the adjoint matrix
  Eigen::Matrix<double, 6, 6> adjoint;
  adjoint.block<3, 3>(0, 0) = rotation;
  adjoint.block<3, 3>(0, 3) = translation_hat * rotation;
  adjoint.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
  adjoint.block<3, 3>(3, 3) = rotation;
  return adjoint;
}

Eigen::Matrix<double, 6, 1> TransformTwist(
    const Eigen::Isometry3d& transform,
    const Eigen::Matrix<double, 6, 1>& initial_twist)
{
  return static_cast<Eigen::Matrix<double, 6, 1>>(
      AdjointFromTransform(transform) * initial_twist);
}

Eigen::Matrix<double, 6, 1> TwistBetweenTransforms(
    const Eigen::Isometry3d& start,
    const Eigen::Isometry3d& end)
{
  const Eigen::Isometry3d t_diff = start.inverse() * end;
  return TwistUnhat(t_diff.matrix().log());
}

Eigen::Matrix3d ExpMatrixExact(const Eigen::Matrix3d& hatted_rot_velocity,
                               const double delta_t)
{
  if (std::abs(Unskew(hatted_rot_velocity).norm() - 1.0) < 1e-10)
  {
    const Eigen::Matrix3d exp_matrix = Eigen::Matrix3d::Identity()
      + (hatted_rot_velocity * sin(delta_t))
      + (hatted_rot_velocity * hatted_rot_velocity * (1.0 - cos(delta_t)));
    return exp_matrix;
  }
  else
  {
    throw std::invalid_argument("Invalid hatted_rot_velocity: "
                                "std::abs(Unskew(hatted_rot_velocity).norm()"
                                " - 1.0) >= 1e-10");
  }
}

Eigen::Isometry3d ExpTwist(const Eigen::Matrix<double, 6, 1>& twist,
                           const double delta_t)
{
  const Eigen::Vector3d trans_velocity = twist.segment<3>(0);
  const Eigen::Vector3d rot_velocity = twist.segment<3>(3);
  const double trans_velocity_norm = trans_velocity.norm();
  const double rot_velocity_norm = rot_velocity.norm();
  Eigen::Matrix4d raw_transform = Eigen::Matrix4d::Identity();
  if (rot_velocity_norm >= 1e-100)
  {
    const double scaled_delta_t = delta_t * rot_velocity_norm;
    const Eigen::Vector3d scaled_trans_velocity
        = trans_velocity / rot_velocity_norm;
    const Eigen::Vector3d scaled_rot_velocity
        = rot_velocity / rot_velocity_norm;
    const Eigen::Matrix3d rotation_displacement
        = ExpMatrixExact(Skew(scaled_rot_velocity), scaled_delta_t);
    const Eigen::Vector3d translation_displacement
        = ((Eigen::Matrix3d::Identity() - rotation_displacement)
           * scaled_rot_velocity.cross(scaled_trans_velocity))
          + (scaled_rot_velocity * scaled_rot_velocity.transpose()
             * scaled_trans_velocity * scaled_delta_t);
    raw_transform.block<3, 3>(0, 0) = rotation_displacement;
    raw_transform.block<3, 1>(0, 3) = translation_displacement;
  }
  else
  {
    // NOTE: you may encounter numerical instability using ExpTwist(...) with
    // translation and rotational norm < 1e-100.
    if ((trans_velocity_norm >= 1e-100) || (rot_velocity_norm == 0.0))
    {
      raw_transform.block<3, 1>(0, 3) = trans_velocity * delta_t;
    }
    else
    {
      const double scaled_delta_t = delta_t * rot_velocity_norm;
      const Eigen::Vector3d scaled_trans_velocity
          = trans_velocity / rot_velocity_norm;
      const Eigen::Vector3d scaled_rot_velocity
          = rot_velocity / rot_velocity_norm;
      const Eigen::Matrix3d rotation_displacement
          = ExpMatrixExact(Skew(scaled_rot_velocity), scaled_delta_t);
      const Eigen::Vector3d translation_displacement
          = ((Eigen::Matrix3d::Identity() - rotation_displacement)
             * scaled_rot_velocity.cross(scaled_trans_velocity))
            + (scaled_rot_velocity * scaled_rot_velocity.transpose()
               * scaled_trans_velocity * scaled_delta_t);
      raw_transform.block<3, 3>(0, 0) = rotation_displacement;
      raw_transform.block<3, 1>(0, 3) = translation_displacement;
    }
  }
  Eigen::Isometry3d transform;
  transform = raw_transform;
  return transform;
}

double Interpolate(const double p1, const double p2, const double ratio)
{
  // Safety check ratio
  const double real_ratio = utility::ClampValue(ratio, 0.0, 1.0);
  // Interpolate
  // This is the numerically stable version,
  // rather than  (p1 + (p2 - p1) * real_ratio)
  return ((p1 * (1.0 - real_ratio)) + (p2 * real_ratio));
}

double InterpolateContinuousRevolute(const double p1,
                                     const double p2,
                                     const double ratio)
{
  // Safety check ratio
  const double real_ratio = utility::ClampValue(ratio, 0.0, 1.0);
  // Safety check args
  const double real_p1 = EnforceContinuousRevoluteBounds(p1);
  const double real_p2 = EnforceContinuousRevoluteBounds(p2);
  // Interpolate
  double interpolated = 0.0;
  double diff = real_p2 - real_p1;
  if (std::abs(diff) <= M_PI)
  {
    interpolated = real_p1 + diff * real_ratio;
  }
  else
  {
    if (diff > 0.0)
    {
      diff = 2.0 * M_PI - diff;
    }
    else
    {
      diff = -2.0 * M_PI - diff;
    }
    interpolated = real_p1 - diff * real_ratio;
    // Input states are within bounds, so the following check is sufficient
    if (interpolated > M_PI)
    {
      interpolated -= 2.0 * M_PI;
    }
    else
    {
      if (interpolated < -M_PI)
      {
        interpolated += 2.0 * M_PI;
      }
    }
  }
  return interpolated;
}

std::vector<double> Interpolate(const std::vector<double>& v1,
                                const std::vector<double>& v2,
                                const double ratio)
{
  // Safety check ratio
  const double real_ratio = utility::ClampValue(ratio, 0.0, 1.0);
  // Safety check inputs
  const size_t len = v1.size();
  if (len != v2.size())
  {
    throw std::invalid_argument("Vectors v1 and v2 must be the same size");
  }
  // Interpolate
  // This is the numerically stable version,
  // rather than  (p1 + (p2 - p1) * real_ratio)
  std::vector<double> interped(len, 0);
  for (size_t idx = 0; idx < len; idx++)
  {
    interped[idx] = ((v1[idx] * (1.0 - real_ratio)) + (v2[idx] * real_ratio));
  }
  return interped;
}

Eigen::Quaterniond Interpolate(const Eigen::Quaterniond& q1,
                               const Eigen::Quaterniond& q2,
                               const double ratio)
{
  // Safety check ratio
  const double real_ratio = utility::ClampValue(ratio, 0.0, 1.0);
  // Interpolate
  return q1.slerp(real_ratio, q2);
}

Eigen::VectorXd InterpolateXd(const Eigen::VectorXd& v1,
                              const Eigen::VectorXd& v2,
                              const double ratio)
{
  // Safety check sizes
  if (v1.size() != v2.size())
  {
    throw std::invalid_argument("Vectors v1 and v2 must be the same size");
  }
  // Safety check ratio
  const double real_ratio = utility::ClampValue(ratio, 0.0, 1.0);
  // Interpolate
  // This is the numerically stable version,
  // rather than  (p1 + (p2 - p1) * real_ratio)
  return ((v1 * (1.0 - real_ratio)) + (v2 * real_ratio));
}

Eigen::Vector3d Interpolate3d(const Eigen::Vector3d& v1,
                              const Eigen::Vector3d& v2,
                              const double ratio)
{
  // Safety check ratio
  const double real_ratio = utility::ClampValue(ratio, 0.0, 1.0);
  // Interpolate
  // This is the numerically stable version,
  // rather than  (p1 + (p2 - p1) * real_ratio)
  return ((v1 * (1.0 - real_ratio)) + (v2 * real_ratio));
}

Eigen::Vector4d Interpolate4d(const Eigen::Vector4d& v1,
                              const Eigen::Vector4d& v2,
                              const double ratio)
{
  // Safety check ratio
  const double real_ratio = utility::ClampValue(ratio, 0.0, 1.0);
  // Interpolate
  // This is the numerically stable version,
  // rather than  (p1 + (p2 - p1) * real_ratio)
  return ((v1 * (1.0 - real_ratio)) + (v2 * real_ratio));
}

Eigen::Isometry3d Interpolate(const Eigen::Isometry3d& t1,
                              const Eigen::Isometry3d& t2,
                              const double ratio)
{
  // Safety check ratio
  const double real_ratio = utility::ClampValue(ratio, 0.0, 1.0);
  // Interpolate
  const Eigen::Vector3d v1 = t1.translation();
  const Eigen::Quaterniond q1(t1.rotation());
  const Eigen::Vector3d v2 = t2.translation();
  const Eigen::Quaterniond q2(t2.rotation());
  const Eigen::Vector3d vint = Interpolate3d(v1, v2, real_ratio);
  const Eigen::Quaterniond qint = Interpolate(q1, q2, real_ratio);
  const Eigen::Isometry3d tint = static_cast<Eigen::Translation3d>(vint) * qint;
  return tint;
}

double SquaredDistance(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2)
{
  const double xd = v2.x() - v1.x();
  const double yd = v2.y() - v1.y();
  return ((xd * xd) + (yd * yd));
}

double Distance(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2)
{
  return std::sqrt(SquaredDistance(v1, v2));
}

double SquaredDistance(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
  const double xd = v2.x() - v1.x();
  const double yd = v2.y() - v1.y();
  const double zd = v2.z() - v1.z();
  return ((xd * xd) + (yd * yd) + (zd * zd));
}

double Distance(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
  return std::sqrt(SquaredDistance(v1, v2));
}

double SquaredDistance(const Eigen::VectorXd& v1, const Eigen::VectorXd& v2)
{
  if (v1.size() == v2.size())
  {
    return (v2 - v1).squaredNorm();
  }
  else
  {
    throw std::invalid_argument("v1.size() != v2.size()");
  }
}

double Distance(const Eigen::VectorXd& v1, const Eigen::VectorXd& v2)
{
  return std::sqrt(SquaredDistance(v1, v2));
}

double Distance(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2)
{
  const double dq = std::abs((q1.w() * q2.w())
                             + (q1.x() * q2.x())
                             + (q1.y() * q2.y())
                             + (q1.z() * q2.z()));
  if (dq < (1.0 - std::numeric_limits<double>::epsilon()))
  {
    return std::acos(2.0 * (dq * dq) - 1.0);
  }
  else
  {
    return 0.0;
  }
}

double Distance(const Eigen::Isometry3d& t1,
                const Eigen::Isometry3d& t2,
                const double alpha)
{
  const double real_alpha = utility::ClampValue(alpha, 0.0, 1.0);
  const Eigen::Vector3d v1 = t1.translation();
  const Eigen::Quaterniond q1(t1.rotation());
  const Eigen::Vector3d v2 = t2.translation();
  const Eigen::Quaterniond q2(t2.rotation());
  const double vdist = Distance(v1, v2) * (1.0 - real_alpha);
  const double qdist = Distance(q1, q2) * (real_alpha);
  return vdist + qdist;
}

double SquaredDistance(const std::vector<double>& p1,
                       const std::vector<double>& p2)
{
  if (p1.size() == p2.size())
  {
    double distance = 0.0;
    for (size_t idx = 0; idx < p1.size(); idx++)
    {
      distance += (p2[idx] - p1[idx]) * (p2[idx] - p1[idx]);
    }
    return distance;
  }
  else
  {
    throw std::invalid_argument("p1.size() != p2.size()");
  }
}

double Distance(const std::vector<double>& p1, const std::vector<double>& p2)
{
  if (p1.size() == p2.size())
  {
    return std::sqrt(SquaredDistance(p1, p2));
  }
  else
  {
    throw std::invalid_argument("p1.size() != p2.size()");
  }
}

double ContinuousRevoluteSignedDistance(const double p1, const double p2)
{
  // Safety check args
  const double real_p1 = EnforceContinuousRevoluteBounds(p1);
  const double real_p2 = EnforceContinuousRevoluteBounds(p2);
  const double raw_distance = real_p2 - real_p1;
  if ((raw_distance <= -M_PI) || (raw_distance > M_PI))
  {
    if (raw_distance <= -M_PI)
    {
      return (-(2.0 * M_PI) - raw_distance);
    }
    else if (raw_distance > M_PI)
    {
      return ((2.0 * M_PI) - raw_distance);
    }
    else
    {
      return raw_distance;
    }
  }
  else
  {
    return raw_distance;
  }
}

double ContinuousRevoluteDistance(const double p1, const double p2)
{
  return std::abs(ContinuousRevoluteSignedDistance(p1, p2));
}

double AddContinuousRevoluteValues(const double start, const double change)
{
  return EnforceContinuousRevoluteBounds(start + change);
}

double GetContinuousRevoluteRange(const double start, const double end)
{
  const double raw_range = ContinuousRevoluteSignedDistance(start, end);
  if (raw_range >= 0.0)
  {
    return raw_range;
  }
  else
  {
    return (2.0 * M_PI) + raw_range;
  }
}

bool CheckInContinuousRevoluteRange(const double start,
                                    const double range,
                                    const double val)
{
  const double real_val = EnforceContinuousRevoluteBounds(val);
  const double real_start = EnforceContinuousRevoluteBounds(start);
  const double delta = ContinuousRevoluteSignedDistance(real_start, real_val);
  if (delta >= 0.0)
  {
    if (delta <= range)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    const double real_delta = (2.0 * M_PI) + delta;
    if (real_delta <= range)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}

bool CheckInContinuousRevoluteBounds(const double start,
                                     const double end,
                                     const double val)
{
  const double range = GetContinuousRevoluteRange(start, end);
  return CheckInContinuousRevoluteRange(start, range, val);
}

double AverageStdVectorDouble(const std::vector<double>& values,
                              const std::vector<double>& weights)
{
  // Get the weights
  if (values.empty())
  {
    throw std::invalid_argument("Provided vector is empty");
  }
  if ((weights.size() != values.size()) && (weights.size() != 0))
  {
    throw std::invalid_argument("Provided weights must be empty"
                                " or same size to provided vector");
  }
  const bool use_weights = (weights.size() != 0);
  // Find the first element with non-zero weight
  size_t starting_idx = 0;
  while (starting_idx < weights.size()
         && std::abs(weights[starting_idx]) == 0.0)
  {
    starting_idx++;
  }
  // If all weights are zero, result is undefined
  if (starting_idx >= values.size())
  {
    throw std::invalid_argument("Provided weights are all zero");
  }
  // Start the recursive definition with the base case
  double average = values[starting_idx];
  const double starting_weight = use_weights ? std::abs(weights[starting_idx])
                                             : 1.0;
  double weights_running_sum = starting_weight;
  // Do the weighted averaging on the rest of the vectors
  for (size_t idx = starting_idx + 1; idx < values.size(); idx++)
  {
    const double weight = use_weights ? std::abs(weights[idx]) : 1.0;
    weights_running_sum += weight;
    const double effective_weight = weight / weights_running_sum;
    const double prev_average = average;
    const double current = values[idx];
    average = prev_average + (effective_weight * (current - prev_average));
  }
  return average;
}

double ComputeStdDevStdVectorDouble(const std::vector<double>& values,
                                    const double mean)
{
  if (values.empty())
  {
    throw std::invalid_argument("Provided vector is empty");
  }
  else if (values.size() == 1)
  {
    return 0.0;
  }
  else
  {
    const double inv_n_minus_1 = 1.0 / static_cast<double>(values.size() - 1);
    double stddev_sum = 0.0;
    for (size_t idx = 0; idx < values.size(); idx++)
    {
      const double delta = values[idx] - mean;
      stddev_sum += (delta * delta);
    }
    return std::sqrt(stddev_sum * inv_n_minus_1);
  }
}

double ComputeStdDevStdVectorDouble(const std::vector<double>& values)
{
  const double mean = AverageStdVectorDouble(values);
  return ComputeStdDevStdVectorDouble(values, mean);
}

double AverageContinuousRevolute(const std::vector<double>& angles,
                                 const std::vector<double>& weights)
{
  return AverageStdVectorDouble(angles, weights);
}

Eigen::Vector3d AverageEigenVector3d(const VectorVector3d& vectors,
                                     const std::vector<double>& weights)
{
  return AverageEigenVector(vectors, weights);
}

Eigen::Vector4d AverageEigenVector4d(const VectorVector4d& vectors,
                                     const std::vector<double>& weights)
{
  return AverageEigenVector(vectors, weights);
}

Eigen::VectorXd AverageEigenVectorXd(
    const std::vector<Eigen::VectorXd>& vectors,
    const std::vector<double>& weights)
{
  return AverageEigenVector(vectors, weights);
}

// Implementation of method described in (http://stackoverflow.com/a/27410865)
// See paper at (http://www.acsu.buffalo.edu/~johnc/ave_quat07.pdf) for more
Eigen::Quaterniond AverageEigenQuaterniond(const VectorQuaterniond& quaternions,
                                           const std::vector<double>& weights)
{
  // Get the weights
  const bool use_weights = weights.size() == quaternions.size() ? true : false;
  if (quaternions.empty())
  {
    throw std::invalid_argument("Provided vector is empty");
  }
  if ((weights.size() != quaternions.size()) && (weights.size() != 0))
  {
    throw std::invalid_argument("Provided weights must be empty"
                                " or same size to provided vector");
  }
  // Shortcut the process if there is only 1 quaternion
  if (quaternions.size() == 1)
  {
    if (weights.size() > 0)
    {
      if (std::abs(weights[0]) == 0.0)
      {
        throw std::invalid_argument("Single quaternion with zero weight");
      }
    }
    return quaternions[0];
  }
  // Build the averaging matrix
  Eigen::MatrixXd q_matrix(4, quaternions.size());
  for (size_t idx = 0; idx < quaternions.size(); idx++)
  {
    const double weight = use_weights ? std::abs(weights[idx]) : 1.0;
    const Eigen::Quaterniond& q = quaternions[idx];
    q_matrix.col(static_cast<ssize_t>(idx))
        << weight * q.w(), weight * q.x(), weight * q.y(), weight * q.z();
  }
  // Make the matrix square
  const Eigen::Matrix<double, 4, 4> qqtranspose_matrix
      = q_matrix * q_matrix.transpose();
  // Compute the eigenvectors and eigenvalues of the qqtranspose matrix
  const Eigen::EigenSolver<Eigen::Matrix<double, 4, 4>>
      solver(qqtranspose_matrix);
  const Eigen::EigenSolver<Eigen::Matrix<double, 4, 4>>::EigenvalueType
      eigen_values = solver.eigenvalues();
  const Eigen::EigenSolver<Eigen::Matrix<double, 4, 4>>::EigenvectorsType
      eigen_vectors = solver.eigenvectors();
  // Extract the eigenvector corresponding to the largest eigenvalue
  double max_eigenvalue = -std::numeric_limits<double>::infinity();
  int64_t max_eigenvector_index = -1;
  for (size_t idx = 0; idx < 4; idx++)
  {
    const double current_eigenvalue
        = eigen_values(static_cast<ssize_t>(idx)).real();
    if (current_eigenvalue > max_eigenvalue)
    {
      max_eigenvalue = current_eigenvalue;
      max_eigenvector_index = static_cast<int64_t>(idx);
    }
  }
  if (max_eigenvector_index < 0)
  {
    throw std::runtime_error("Failed to find max eigenvector");
  }
  // Note that these are already normalized!
  const Eigen::Vector4cd best_eigenvector
      = eigen_vectors.col(static_cast<ssize_t>(max_eigenvector_index));
  // Convert back into a quaternion
  const Eigen::Quaterniond average_q(best_eigenvector(0).real(),
                                     best_eigenvector(1).real(),
                                     best_eigenvector(2).real(),
                                     best_eigenvector(3).real());
  return average_q;
}

Eigen::Isometry3d AverageEigenIsometry3d(const VectorIsometry3d& transforms,
                                         const std::vector<double>& weights)
{
  if (transforms.empty())
  {
    throw std::invalid_argument("Provided vector is empty");
  }
  if ((weights.size() != transforms.size()) && (weights.size() != 0))
  {
    throw std::invalid_argument("Provided weights must be empty"
                                " or same size to provided vector");
  }
  // Shortcut the process if there is only 1 transform
  if (transforms.size() == 1)
  {
    if (weights.size() > 0)
    {
      if (std::abs(weights[0]) == 0.0)
      {
        throw std::invalid_argument("Single transform with zero weight");
      }
    }
    return transforms[0];
  }
  // Extract components
  VectorVector3d translations(transforms.size());
  VectorQuaterniond rotations(transforms.size());
  for (size_t idx = 0; idx < transforms.size(); idx++)
  {
    translations[idx] = transforms[idx].translation();
    rotations[idx] = Eigen::Quaterniond(transforms[idx].rotation());
  }
  // Average
  const Eigen::Vector3d average_translation
      = AverageEigenVector(translations, weights);
  const Eigen::Quaterniond average_rotation
      = AverageEigenQuaterniond(rotations, weights);
  // Make the average transform
  const Eigen::Isometry3d average_transform
      = static_cast<Eigen::Translation3d>(average_translation)
          * average_rotation;
  return average_transform;
}

double WeightedDotProduct(const Eigen::VectorXd& vec1,
                          const Eigen::VectorXd& vec2,
                          const Eigen::VectorXd& weights)
{
  return vec1.cwiseProduct(weights).dot(vec2);
}

double WeightedSquaredNorm(const Eigen::VectorXd& vec,
                           const Eigen::VectorXd weights)
{
  return WeightedDotProduct(vec, vec, weights);
}

double WeightedNorm(const Eigen::VectorXd& vec, const Eigen::VectorXd& weights)
{
  return std::sqrt(WeightedSquaredNorm(vec, weights));
}

double WeightedCosineAngleBetweenVectors(const Eigen::VectorXd& vec1,
                                         const Eigen::VectorXd& vec2,
                                         const Eigen::VectorXd& weights)
{
  const double vec1_norm = WeightedNorm(vec1, weights);
  const double vec2_norm = WeightedNorm(vec2, weights);
  if (vec1_norm > 0 && vec2_norm > 0)
  {
    const double result
        = WeightedDotProduct(vec1, vec2, weights) / (vec1_norm * vec2_norm);
    return std::max(-1.0, std::min(result, 1.0));;
  }
  else
  {
    throw std::invalid_argument("One or more input vectors has zero norm");
  }
}

double WeightedAngleBetweenVectors(const Eigen::VectorXd& vec1,
                                   const Eigen::VectorXd& vec2,
                                   const Eigen::VectorXd& weights)
{
  return std::acos(WeightedCosineAngleBetweenVectors(vec1, vec2, weights));
}

Hyperplane FitPlaneToPoints(const std::vector<Eigen::VectorXd>& points)
{
  // Subtract out the centroid
  const Eigen::VectorXd centroid = AverageEigenVectorXd(points);
  Eigen::MatrixXd centered_points(centroid.size(), points.size());
  for (size_t idx = 0; idx < points.size(); idx++)
  {
    const Eigen::VectorXd& current_point = points[idx];
    centered_points.block(0, static_cast<ssize_t>(idx), centroid.size(), 1) =
        (current_point - centroid);
  }
  // Compute SVD of the centered points
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      centered_points, Eigen::ComputeThinU | Eigen::ComputeThinV);
  // Get results of SVD
  const Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType& singular_values =
      svd.singularValues();
  const Eigen::JacobiSVD<Eigen::MatrixXd>::MatrixUType& u_matrix =
      svd.matrixU();
  // Get the left singular vector corresponding to the minimum singular value
  double minimum_singular_value = INFINITY;
  ssize_t best_singular_value_index = -1;
  for (ssize_t idx = 0; idx < singular_values.size(); idx++)
  {
    const std::complex<double> current_singular_value = singular_values(idx);
    if (current_singular_value.real() < minimum_singular_value)
    {
      minimum_singular_value = current_singular_value.real();
      best_singular_value_index = idx;
    }
  }
  if (best_singular_value_index < 0)
  {
    throw std::runtime_error("Could not find best singular value index");
  }
  // The corresponding left singular vector is the normal vector of the best-fit
  // hyperplane.
  const Eigen::VectorXd best_left_singular_vector =
      u_matrix.col(best_singular_value_index);
  const Eigen::VectorXd normal_vector =
      best_left_singular_vector.stableNormalized();
  return Hyperplane(centroid, normal_vector);
}
}  // namespace math
}  // namespace common_robotics_utilities
