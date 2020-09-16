#include <common_robotics_utilities/ros_conversions.hpp>

#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/math.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace common_robotics_utilities
{
namespace ros_conversions
{
Eigen::Vector3d GeometryPointToEigenVector3d(
    const geometry_msgs::msg::Point& point)
{
  Eigen::Vector3d eigen_point(point.x, point.y, point.z);
  return eigen_point;
}

geometry_msgs::msg::Point EigenVector3dToGeometryPoint(
    const Eigen::Vector3d& point)
{
  geometry_msgs::msg::Point geom_point;
  geom_point.x = point.x();
  geom_point.y = point.y();
  geom_point.z = point.z();
  return geom_point;
}

Eigen::Vector4d GeometryPointToEigenVector4d(
    const geometry_msgs::msg::Point& point)
{
  Eigen::Vector4d eigen_point(point.x, point.y, point.z, 1.0);
  return eigen_point;
}

geometry_msgs::msg::Point EigenVector4dToGeometryPoint(
    const Eigen::Vector4d& point)
{
  geometry_msgs::msg::Point geom_point;
  geom_point.x = point(0);
  geom_point.y = point(1);
  geom_point.z = point(2);
  return geom_point;
}

geometry_msgs::msg::PointStamped EigenVector3dToGeometryPointStamped(
    const Eigen::Vector3d& point, const std::string& frame_id)
{
  geometry_msgs::msg::PointStamped point_stamped;
  point_stamped.header.frame_id = frame_id;
  point_stamped.point = EigenVector3dToGeometryPoint(point);
  return point_stamped;
}

Eigen::Vector3d GeometryVector3ToEigenVector3d(
    const geometry_msgs::msg::Vector3& vector)
{
  Eigen::Vector3d eigen_vector(vector.x, vector.y, vector.z);
  return eigen_vector;
}

geometry_msgs::msg::Vector3 EigenVector3dToGeometryVector3(
    const Eigen::Vector3d& vector)
{
  geometry_msgs::msg::Vector3 geom_vector;
  geom_vector.x = vector.x();
  geom_vector.y = vector.y();
  geom_vector.z = vector.z();
  return geom_vector;
}

Eigen::Vector4d GeometryVector3ToEigenVector4d(
    const geometry_msgs::msg::Vector3& vector)
{
  Eigen::Vector4d eigen_vector(vector.x, vector.y, vector.z, 0.0);
  return eigen_vector;
}

geometry_msgs::msg::Vector3 EigenVector4dToGeometryVector3(
    const Eigen::Vector4d& vector)
{
  geometry_msgs::msg::Vector3 geom_vector;
  geom_vector.x = vector(0);
  geom_vector.y = vector(1);
  geom_vector.z = vector(2);
  return geom_vector;
}

Eigen::Quaterniond GeometryQuaternionToEigenQuaterniond(
    const geometry_msgs::msg::Quaternion& quat)
{
  Eigen::Quaterniond eigen_quaternion(quat.w, quat.x, quat.y, quat.z);
  return eigen_quaternion;
}

geometry_msgs::msg::Quaternion EigenQuaterniondToGeometryQuaternion(
    const Eigen::Quaterniond& quat)
{
  geometry_msgs::msg::Quaternion geom_quaternion;
  geom_quaternion.w = quat.w();
  geom_quaternion.x = quat.x();
  geom_quaternion.y = quat.y();
  geom_quaternion.z = quat.z();
  return geom_quaternion;
}

Eigen::Isometry3d GeometryPoseToEigenIsometry3d(
    const geometry_msgs::msg::Pose& pose)
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

geometry_msgs::msg::Pose EigenIsometry3dToGeometryPose(
    const Eigen::Isometry3d& transform)
{
  const Eigen::Vector3d trans = transform.translation();
  const Eigen::Quaterniond quat(transform.rotation());
  geometry_msgs::msg::Pose geom_pose;
  geom_pose.position.x = trans.x();
  geom_pose.position.y = trans.y();
  geom_pose.position.z = trans.z();
  geom_pose.orientation.w = quat.w();
  geom_pose.orientation.x = quat.x();
  geom_pose.orientation.y = quat.y();
  geom_pose.orientation.z = quat.z();
  return geom_pose;
}

geometry_msgs::msg::PoseStamped EigenIsometry3dToGeometryPoseStamped(
    const Eigen::Isometry3d& transform, const std::string& frame_id)
{
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = frame_id;
  pose_stamped.pose = EigenIsometry3dToGeometryPose(transform);
  return pose_stamped;
}

Eigen::Isometry3d GeometryTransformToEigenIsometry3d(
    const geometry_msgs::msg::Transform& transform)
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

geometry_msgs::msg::Transform EigenIsometry3dToGeometryTransform(
    const Eigen::Isometry3d& transform)
{
  const Eigen::Vector3d trans = transform.translation();
  const Eigen::Quaterniond quat(transform.rotation());
  geometry_msgs::msg::Transform geom_transform;
  geom_transform.translation.x = trans.x();
  geom_transform.translation.y = trans.y();
  geom_transform.translation.z = trans.z();
  geom_transform.rotation.w = quat.w();
  geom_transform.rotation.x = quat.x();
  geom_transform.rotation.y = quat.y();
  geom_transform.rotation.z = quat.z();
  return geom_transform;
}

geometry_msgs::msg::TransformStamped EigenIsometry3dToGeometryTransformStamped(
    const Eigen::Isometry3d& transform, const std::string& frame_id,
    const std::string& child_frame_id)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform = EigenIsometry3dToGeometryTransform(transform);
  return transform_stamped;
}

Eigen::Matrix3Xd VectorGeometryPointToEigenMatrix3Xd(
    const std::vector<geometry_msgs::msg::Point>& vector_geom)
{
  Eigen::Matrix3Xd eigen_matrix = Eigen::MatrixXd(3, vector_geom.size());
  for (size_t idx = 0; idx < vector_geom.size(); idx++)
  {
    eigen_matrix.block<3,1>(0, static_cast<ssize_t>(idx))
        = GeometryPointToEigenVector3d(vector_geom.at(idx));
  }
  return eigen_matrix;
}

std::vector<geometry_msgs::msg::Point> EigenMatrix3XdToVectorGeometryPoint(
    const Eigen::Matrix3Xd& eigen_matrix)
{
  std::vector<geometry_msgs::msg::Point> vector_geom(
      static_cast<size_t>(eigen_matrix.cols()));
  for (size_t idx = 0; idx < vector_geom.size(); idx++)
  {
    vector_geom.at(idx)
        = EigenVector3dToGeometryPoint(
            eigen_matrix.block<3,1>(0, static_cast<ssize_t>(idx)));
  }
  return vector_geom;
}

std::vector<geometry_msgs::msg::Point>
VectorEigenVector3dToVectorGeometryPoint(
    const common_robotics_utilities::math::VectorVector3d& vector_eigen)
{
  std::vector<geometry_msgs::msg::Point> vector_geom(vector_eigen.size());
  for (size_t idx = 0; idx < vector_eigen.size(); idx++)
  {
    vector_geom.at(idx) = EigenVector3dToGeometryPoint(vector_eigen.at(idx));
  }
  return vector_geom;
}

common_robotics_utilities::math::VectorVector3d
VectorGeometryPointToVectorEigenVector3d(
    const std::vector<geometry_msgs::msg::Point>& vector_geom)
{
  common_robotics_utilities::math::VectorVector3d vector_eigen(
      vector_geom.size());
  for (size_t idx = 0; idx < vector_geom.size(); idx++)
  {
    vector_eigen.at(idx) = GeometryPointToEigenVector3d(vector_geom.at(idx));
  }
  return vector_eigen;
}

common_robotics_utilities::math::VectorVector3d
VectorGeometryVector3ToEigenVector3d(
    const std::vector<geometry_msgs::msg::Vector3>& vector_geom)
{
  common_robotics_utilities::math::VectorVector3d vector_eigen(
      vector_geom.size());
  for (size_t idx = 0; idx < vector_geom.size(); idx++)
  {
    vector_eigen.at(idx) = GeometryVector3ToEigenVector3d(vector_geom.at(idx));
  }
  return vector_eigen;
}

common_robotics_utilities::math::VectorIsometry3d
VectorGeometryPoseToVectorIsometry3d(
    const std::vector<geometry_msgs::msg::Pose>& vector_geom)
{
  common_robotics_utilities::math::VectorIsometry3d vector_eigen(
      vector_geom.size());
  for (size_t idx = 0; idx < vector_geom.size(); idx++)
  {
    vector_eigen.at(idx) = GeometryPoseToEigenIsometry3d(vector_geom.at(idx));
  }
  return vector_eigen;
}

common_robotics_utilities::math::VectorIsometry3d
VectorGeometryPoseToVectorIsometry3d(
    const std::vector<geometry_msgs::msg::Transform>& vector_geom)
{
  common_robotics_utilities::math::VectorIsometry3d vector_eigen(
      vector_geom.size());
  for (size_t idx = 0; idx < vector_geom.size(); idx++)
  {
    vector_eigen.at(idx) =
        GeometryTransformToEigenIsometry3d(vector_geom.at(idx));
  }
  return vector_eigen;
}

std::vector<geometry_msgs::msg::Pose> VectorIsometry3dToVectorGeometryPose(
    const common_robotics_utilities::math::VectorIsometry3d& vector_eigen)
{
  std::vector<geometry_msgs::msg::Pose> vector_geom(vector_eigen.size());
  for (size_t idx = 0; idx < vector_eigen.size(); idx++)
  {
    vector_geom.at(idx) = EigenIsometry3dToGeometryPose(vector_eigen.at(idx));
  }
  return vector_geom;
}

std::vector<geometry_msgs::msg::Transform> VectorIsometry3dToVectorGeometryTransform(
    const common_robotics_utilities::math::VectorIsometry3d& vector_eigen)
{
  std::vector<geometry_msgs::msg::Transform> vector_geom(vector_eigen.size());
  for (size_t idx = 0; idx < vector_eigen.size(); idx++)
  {
    vector_geom.at(idx) =
        EigenIsometry3dToGeometryTransform(vector_eigen.at(idx));
  }
  return vector_geom;
}
}  // namespace ros_conversions
}  // namespace common_robotics_utilities
