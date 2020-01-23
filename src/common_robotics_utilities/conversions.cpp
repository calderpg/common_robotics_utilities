#include <common_robotics_utilities/conversions.hpp>

#include <vector>

#include <Eigen/Geometry>

namespace common_robotics_utilities
{
namespace conversions
{
Eigen::Quaterniond QuaternionFromRPY(const double R,
                                     const double P,
                                     const double Y)
{
  const Eigen::AngleAxisd roll(R, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(P, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(Y, Eigen::Vector3d::UnitZ());
  const Eigen::Quaterniond quat(roll * pitch * yaw);
  return quat;
}

// URDF RPY IS ACTUALLY APPLIED Y*P*R
Eigen::Quaterniond QuaternionFromUrdfRPY(const double R,
                                         const double P,
                                         const double Y)
{
  const Eigen::AngleAxisd roll(R, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(P, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(Y, Eigen::Vector3d::UnitZ());
  const Eigen::Quaterniond quat(yaw * pitch * roll);
  return quat;
}

// Returns XYZ Euler angles
Eigen::Vector3d EulerAnglesFromRotationMatrix(
    const Eigen::Matrix3d& rot_matrix)
{
  // Use XYZ angles
  const Eigen::Vector3d euler_angles = rot_matrix.eulerAngles(0, 1, 2);
  return euler_angles;
}

// Returns XYZ Euler angles
Eigen::Vector3d EulerAnglesFromQuaternion(const Eigen::Quaterniond& quat)
{
  return EulerAnglesFromRotationMatrix(quat.toRotationMatrix());
}

// Returns XYZ Euler angles
Eigen::Vector3d EulerAnglesFromIsometry3d(const Eigen::Isometry3d& trans)
{
  return EulerAnglesFromRotationMatrix(trans.rotation());
}

Eigen::Isometry3d TransformFromXYZRPY(const double x,
                                      const double y,
                                      const double z,
                                      const double roll,
                                      const double pitch,
                                      const double yaw)
{
  const Eigen::Isometry3d transform
      = Eigen::Translation3d(x, y, z) * QuaternionFromRPY(roll, pitch, yaw);
  return transform;
}

Eigen::Isometry3d TransformFromRPY(const Eigen::Vector3d& translation,
                                   const Eigen::Vector3d& rotation)
{
  const Eigen::Isometry3d transform
      = static_cast<Eigen::Translation3d>(translation)
            * QuaternionFromRPY(rotation.x(), rotation.y(), rotation.z());
  return transform;
}

Eigen::Isometry3d TransformFromRPY(const Eigen::VectorXd& components)
{
  if (components.size() == 6)
  {
    return Eigen::Translation3d(components(0),
                                components(1),
                                components(2))
           * QuaternionFromRPY(components(3),
                               components(4),
                               components(5));
  }
  else
  {
    throw std::invalid_argument(
          "VectorXd source vector is not 6 elements in size");
  }
}

// URDF RPY IS ACTUALLY APPLIED Y*P*R
Eigen::Isometry3d TransformFromUrdfXYZRPY(const double x,
                                          const double y,
                                          const double z,
                                          const double roll,
                                          const double pitch,
                                          const double yaw)
{
  const Eigen::Isometry3d transform = Eigen::Translation3d(x, y, z)
                                      * QuaternionFromUrdfRPY(roll, pitch, yaw);
  return transform;
}

// URDF RPY IS ACTUALLY APPLIED Y*P*R
Eigen::Isometry3d TransformFromUrdfRPY(const Eigen::Vector3d& translation,
                                       const Eigen::Vector3d& rotation)
{
  const Eigen::Isometry3d transform
      = static_cast<Eigen::Translation3d>(translation)
          * QuaternionFromUrdfRPY(rotation.x(), rotation.y(), rotation.z());
  return transform;
}

// URDF RPY IS ACTUALLY APPLIED Y*P*R
Eigen::Isometry3d TransformFromUrdfRPY(const Eigen::VectorXd& components)
{
  if (components.size() == 6)
  {
    return Eigen::Translation3d(components(0),
                                components(1),
                                components(2))
           * QuaternionFromUrdfRPY(components(3),
                                   components(4),
                                   components(5));
  }
  else
  {
    throw std::invalid_argument(
          "VectorXd source vector is not 6 elements in size");
  }
}

Eigen::VectorXd TransformToRPY(const Eigen::Isometry3d& transform)
{
  Eigen::VectorXd components = Eigen::VectorXd::Zero(6);
  const Eigen::Vector3d translation = transform.translation();
  const Eigen::Vector3d rotation
      = EulerAnglesFromRotationMatrix(transform.rotation());
  components << translation, rotation;
  return components;
}

Eigen::Vector3d StdVectorDoubleToEigenVector3d(
    const std::vector<double>& vector)
{
  if (vector.size() == 3)
  {
    return Eigen::Vector3d(vector.at(0), vector.at(1), vector.at(2));
  }
  else
  {
    throw std::invalid_argument(
          "Vector3d source vector is not 3 elements in size");
  }
}

Eigen::VectorXd StdVectorDoubleToEigenVectorXd(
    const std::vector<double>& vector)
{
  Eigen::VectorXd eigen_vector(vector.size());
  for (size_t idx = 0; idx < vector.size(); idx++)
  {
    const double val = vector.at(idx);
    eigen_vector(static_cast<ssize_t>(idx)) = val;
  }
  return eigen_vector;
}

std::vector<double> EigenVector3dToStdVectorDouble(
    const Eigen::Vector3d& point)
{
  return std::vector<double>{point.x(), point.y(), point.z()};
}

std::vector<double> EigenVectorXdToStdVectorDouble(
    const Eigen::VectorXd& eigen_vector)
{
  std::vector<double> vector(static_cast<size_t>(eigen_vector.size()));
  for (size_t idx = 0; idx < vector.size(); idx++)
  {
    const double val = eigen_vector(static_cast<ssize_t>(idx));
    vector.at(idx) = val;
  }
  return vector;
}

// Takes <x, y, z, w> as is the ROS custom!
Eigen::Quaterniond StdVectorDoubleToEigenQuaterniond(
    const std::vector<double>& vector)
{
  if (vector.size() == 4)
  {
    return Eigen::Quaterniond(
        vector.at(3), vector.at(0), vector.at(1), vector.at(2));
  }
  else
  {
    throw std::invalid_argument(
          "Quaterniond source vector is not 4 elements in size");
  }
}

// Returns <x, y, z, w> as is the ROS custom!
std::vector<double> EigenQuaterniondToStdVectorDouble(
    const Eigen::Quaterniond& quat)
{
  return std::vector<double>{quat.x(), quat.y(), quat.z(), quat.w()};
}
}  // namespace conversions
}  // namespace common_robotics_utilities
