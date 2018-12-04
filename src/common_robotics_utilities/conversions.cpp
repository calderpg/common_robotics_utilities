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
  const Eigen::Isometry3d transform = Eigen::Translation3d(x, y, z)
                                      * QuaternionFromRPY(roll, pitch, yaw);
  return transform;
}

Eigen::Isometry3d TransformFromRPY(const Eigen::Vector3d& translation,
                                   const Eigen::Vector3d& rotation)
{
  const Eigen::Isometry3d transform = (Eigen::Translation3d)translation
                                      * QuaternionFromRPY(rotation.x(),
                                                          rotation.y(),
                                                          rotation.z());
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
  const Eigen::Isometry3d transform = (Eigen::Translation3d)translation
                                      * QuaternionFromUrdfRPY(rotation.x(),
                                                              rotation.y(),
                                                              rotation.z());
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
    return Eigen::Vector3d(vector[0], vector[1], vector[2]);
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
    const double val = vector[idx];
    eigen_vector((ssize_t)idx) = val;
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
  std::vector<double> vector((size_t)eigen_vector.size());
  for (size_t idx = 0; idx < (size_t)eigen_vector.size(); idx++)
  {
    const double val = eigen_vector[(ssize_t)idx];
    vector[idx] = val;
  }
  return vector;
}

// Takes <x, y, z, w> as is the ROS custom!
Eigen::Quaterniond StdVectorDoubleToEigenQuaterniond(
    const std::vector<double>& vector)
{
  if (vector.size() == 4)
  {
    return Eigen::Quaterniond(vector[3], vector[0], vector[1], vector[2]);
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

Eigen::Vector3d GeometryPointToEigenVector3d(
    const geometry_msgs::Point& point)
{
  Eigen::Vector3d eigen_point(point.x, point.y, point.z);
  return eigen_point;
}

geometry_msgs::Point EigenVector3dToGeometryPoint(
    const Eigen::Vector3d& point)
{
  geometry_msgs::Point geom_point;
  geom_point.x = point.x();
  geom_point.y = point.y();
  geom_point.z = point.z();
  return geom_point;
}

Eigen::Vector4d GeometryPointToEigenVector4d(
    const geometry_msgs::Point& point)
{
  Eigen::Vector4d eigen_point(point.x, point.y, point.z, 1.0);
  return eigen_point;
}

geometry_msgs::Point EigenVector4dToGeometryPoint(
    const Eigen::Vector4d& point)
{
  geometry_msgs::Point geom_point;
  geom_point.x = point(0);
  geom_point.y = point(1);
  geom_point.z = point(2);
  return geom_point;
}

geometry_msgs::PointStamped EigenVector3dToGeometryPointStamped(
    const Eigen::Vector3d& point, const std::string& frame_id)
{
  geometry_msgs::PointStamped point_stamped;
  point_stamped.header.frame_id = frame_id;
  point_stamped.point = EigenVector3dToGeometryPoint(point);
  return point_stamped;
}

Eigen::Vector3d GeometryVector3ToEigenVector3d(
    const geometry_msgs::Vector3& vector)
{
  Eigen::Vector3d eigen_vector(vector.x, vector.y, vector.z);
  return eigen_vector;
}

geometry_msgs::Vector3 EigenVector3dToGeometryVector3(
    const Eigen::Vector3d& vector)
{
  geometry_msgs::Vector3 geom_vector;
  geom_vector.x = vector.x();
  geom_vector.y = vector.y();
  geom_vector.z = vector.z();
  return geom_vector;
}

Eigen::Vector4d GeometryVector3ToEigenVector4d(
    const geometry_msgs::Vector3& vector)
{
  Eigen::Vector4d eigen_vector(vector.x, vector.y, vector.z, 0.0);
  return eigen_vector;
}

geometry_msgs::Vector3 EigenVector4dToGeometryVector3(
    const Eigen::Vector4d& vector)
{
  geometry_msgs::Vector3 geom_vector;
  geom_vector.x = vector(0);
  geom_vector.y = vector(1);
  geom_vector.z = vector(2);
  return geom_vector;
}

Eigen::Quaterniond GeometryQuaternionToEigenQuaterniond(
    const geometry_msgs::Quaternion& quat)
{
  Eigen::Quaterniond eigen_quaternion(quat.w, quat.x, quat.y, quat.z);
  return eigen_quaternion;
}

geometry_msgs::Quaternion EigenQuaterniondToGeometryQuaternion(
    const Eigen::Quaterniond& quat)
{
  geometry_msgs::Quaternion geom_quaternion;
  geom_quaternion.w = quat.w();
  geom_quaternion.x = quat.x();
  geom_quaternion.y = quat.y();
  geom_quaternion.z = quat.z();
  return geom_quaternion;
}

Eigen::Isometry3d GeometryPoseToEigenIsometry3d(
    const geometry_msgs::Pose& pose)
{
  const Eigen::Translation3d trans(pose.position.x,
                                   pose.position.y,
                                   pose.position.z);
  const Eigen::Quaterniond quat(pose.orientation.w,
                                pose.orientation.x,
                                pose.orientation.y,
                                pose.orientation.z);
  const Eigen::Isometry3d eigen_pose = trans * quat;
  return eigen_pose;
}

geometry_msgs::Pose EigenIsometry3dToGeometryPose(
    const Eigen::Isometry3d& transform)
{
  const Eigen::Vector3d trans = transform.translation();
  const Eigen::Quaterniond quat(transform.rotation());
  geometry_msgs::Pose geom_pose;
  geom_pose.position.x = trans.x();
  geom_pose.position.y = trans.y();
  geom_pose.position.z = trans.z();
  geom_pose.orientation.w = quat.w();
  geom_pose.orientation.x = quat.x();
  geom_pose.orientation.y = quat.y();
  geom_pose.orientation.z = quat.z();
  return geom_pose;
}

geometry_msgs::PoseStamped EigenIsometry3dToGeometryPoseStamped(
    const Eigen::Isometry3d& transform, const std::string& frame_id)
{
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = frame_id;
  pose_stamped.pose = EigenIsometry3dToGeometryPose(transform);
  return pose_stamped;
}

Eigen::Isometry3d GeometryTransformToEigenIsometry3d(
    const geometry_msgs::Transform& transform)
{
  const Eigen::Translation3d trans(transform.translation.x,
                                   transform.translation.y,
                                   transform.translation.z);
  const Eigen::Quaterniond quat(transform.rotation.w,
                                transform.rotation.x,
                                transform.rotation.y,
                                transform.rotation.z);
  const Eigen::Isometry3d eigen_transform = trans * quat;
  return eigen_transform;
}

geometry_msgs::Transform EigenIsometry3dToGeometryTransform(
    const Eigen::Isometry3d& transform)
{
  const Eigen::Vector3d trans = transform.translation();
  const Eigen::Quaterniond quat(transform.rotation());
  geometry_msgs::Transform geom_transform;
  geom_transform.translation.x = trans.x();
  geom_transform.translation.y = trans.y();
  geom_transform.translation.z = trans.z();
  geom_transform.rotation.w = quat.w();
  geom_transform.rotation.x = quat.x();
  geom_transform.rotation.y = quat.y();
  geom_transform.rotation.z = quat.z();
  return geom_transform;
}

geometry_msgs::TransformStamped EigenIsometry3dToGeometryTransformStamped(
    const Eigen::Isometry3d& transform, const std::string& frame_id,
    const std::string& child_frame_id)
{
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform = EigenIsometry3dToGeometryTransform(transform);
  return transform_stamped;
}

Eigen::Matrix3Xd VectorGeometryPointToEigenMatrix3Xd(
    const std::vector<geometry_msgs::Point>& vector_geom)
{
  Eigen::Matrix3Xd eigen_matrix = Eigen::MatrixXd(3, vector_geom.size());
  for (size_t idx = 0; idx < vector_geom.size(); idx++)
  {
    eigen_matrix.block<3,1>(0, (ssize_t)idx)
        = GeometryPointToEigenVector3d(vector_geom[idx]);
  }
  return eigen_matrix;
}

std::vector<geometry_msgs::Point> EigenMatrix3XdToVectorGeometryPoint(
    const Eigen::Matrix3Xd& eigen_matrix)
{
  std::vector<geometry_msgs::Point> vector_geom((size_t)eigen_matrix.cols());
  for (size_t idx = 0; idx < vector_geom.size(); idx++)
  {
    vector_geom[idx]
        = EigenVector3dToGeometryPoint(
            eigen_matrix.block<3,1>(0, (ssize_t)idx));
  }
  return vector_geom;
}

std::vector<geometry_msgs::Point>
VectorEigenVector3dToVectorGeometryPoint(
    const common_robotics_utilities::math::VectorVector3d& vector_eigen)
{
  std::vector<geometry_msgs::Point> vector_geom(vector_eigen.size());
  for (size_t idx = 0; idx < vector_eigen.size(); idx++)
  {
    vector_geom[idx] = EigenVector3dToGeometryPoint(vector_eigen[idx]);
  }
  return vector_geom;
}

common_robotics_utilities::math::VectorVector3d
VectorGeometryPointToVectorEigenVector3d(
    const std::vector<geometry_msgs::Point>& vector_geom)
{
  common_robotics_utilities::math::VectorVector3d vector_eigen(
      vector_geom.size());
  for (size_t idx = 0; idx < vector_geom.size(); idx++)
  {
    vector_eigen[idx] = GeometryPointToEigenVector3d(vector_geom[idx]);
  }
  return vector_eigen;
}

common_robotics_utilities::math::VectorVector3d
VectorGeometryVector3ToEigenVector3d(
    const std::vector<geometry_msgs::Vector3>& vector_geom)
{
  common_robotics_utilities::math::VectorVector3d vector_eigen(
      vector_geom.size());
  for (size_t idx = 0; idx < vector_geom.size(); idx++)
  {
    vector_eigen[idx] = GeometryVector3ToEigenVector3d(vector_geom[idx]);
  }
  return vector_eigen;
}

common_robotics_utilities::math::VectorIsometry3d
VectorGeometryPoseToVectorIsometry3d(
    const std::vector<geometry_msgs::Pose>& vector_geom)
{
  common_robotics_utilities::math::VectorIsometry3d vector_eigen(
      vector_geom.size());
  for (size_t idx = 0; idx < vector_geom.size(); idx++)
  {
    vector_eigen[idx] = GeometryPoseToEigenIsometry3d(vector_geom[idx]);
  }
  return vector_eigen;
}

common_robotics_utilities::math::VectorIsometry3d
VectorGeometryPoseToVectorIsometry3d(
    const std::vector<geometry_msgs::Transform>& vector_geom)
{
  common_robotics_utilities::math::VectorIsometry3d vector_eigen(
      vector_geom.size());
  for (size_t idx = 0; idx < vector_geom.size(); idx++)
  {
    vector_eigen[idx] = GeometryTransformToEigenIsometry3d(vector_geom[idx]);
  }
  return vector_eigen;
}

std::vector<geometry_msgs::Pose> VectorIsometry3dToVectorGeometryPose(
    const common_robotics_utilities::math::VectorIsometry3d& vector_eigen)
{
  std::vector<geometry_msgs::Pose> vector_geom(vector_eigen.size());
  for (size_t idx = 0; idx < vector_eigen.size(); idx++)
  {
    vector_geom[idx] = EigenIsometry3dToGeometryPose(vector_eigen[idx]);
  }
  return vector_geom;
}

std::vector<geometry_msgs::Transform> VectorIsometry3dToVectorGeometryTransform(
    const common_robotics_utilities::math::VectorIsometry3d& vector_eigen)
{
  std::vector<geometry_msgs::Transform> vector_geom(vector_eigen.size());
  for (size_t idx = 0; idx < vector_eigen.size(); idx++)
  {
    vector_geom[idx] = EigenIsometry3dToGeometryTransform(vector_eigen[idx]);
  }
  return vector_geom;
}
}  // namespace conversions
}  // namespace common_robotics_utilities
