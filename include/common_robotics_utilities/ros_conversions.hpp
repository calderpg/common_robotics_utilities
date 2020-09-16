#pragma once

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
    const geometry_msgs::msg::Point& point);

geometry_msgs::msg::Point EigenVector3dToGeometryPoint(
    const Eigen::Vector3d& point);

Eigen::Vector4d GeometryPointToEigenVector4d(
    const geometry_msgs::msg::Point& point);

geometry_msgs::msg::Point EigenVector4dToGeometryPoint(
    const Eigen::Vector4d& point);

geometry_msgs::msg::PointStamped EigenVector3dToGeometryPointStamped(
    const Eigen::Vector3d& point, const std::string& frame_id);

Eigen::Vector3d GeometryVector3ToEigenVector3d(
    const geometry_msgs::msg::Vector3& vector);

geometry_msgs::msg::Vector3 EigenVector3dToGeometryVector3(
    const Eigen::Vector3d& vector);

Eigen::Vector4d GeometryVector3ToEigenVector4d(
    const geometry_msgs::msg::Vector3& vector);

geometry_msgs::msg::Vector3 EigenVector4dToGeometryVector3(
    const Eigen::Vector4d& vector);

Eigen::Quaterniond GeometryQuaternionToEigenQuaterniond(
    const geometry_msgs::msg::Quaternion& quat);

geometry_msgs::msg::Quaternion EigenQuaterniondToGeometryQuaternion(
    const Eigen::Quaterniond& quat);

Eigen::Isometry3d GeometryPoseToEigenIsometry3d(
    const geometry_msgs::msg::Pose& pose);

geometry_msgs::msg::Pose EigenIsometry3dToGeometryPose(
    const Eigen::Isometry3d& transform);

geometry_msgs::msg::PoseStamped EigenIsometry3dToGeometryPoseStamped(
    const Eigen::Isometry3d& transform, const std::string& frame_id);

Eigen::Isometry3d GeometryTransformToEigenIsometry3d(
    const geometry_msgs::msg::Transform& transform);

geometry_msgs::msg::Transform EigenIsometry3dToGeometryTransform(
    const Eigen::Isometry3d& transform);

geometry_msgs::msg::TransformStamped EigenIsometry3dToGeometryTransformStamped(
    const Eigen::Isometry3d& transform, const std::string& frame_id,
    const std::string& child_frame_id);

Eigen::Matrix3Xd VectorGeometryPointToEigenMatrix3Xd(
    const std::vector<geometry_msgs::msg::Point>& vector_geom);

std::vector<geometry_msgs::msg::Point> EigenMatrix3XdToVectorGeometryPoint(
    const Eigen::Matrix3Xd& eigen_matrix);

std::vector<geometry_msgs::msg::Point>
VectorEigenVector3dToVectorGeometryPoint(
    const common_robotics_utilities::math::VectorVector3d& vector_eigen);

common_robotics_utilities::math::VectorVector3d
VectorGeometryPointToVectorEigenVector3d(
    const std::vector<geometry_msgs::msg::Point>& vector_geom);

common_robotics_utilities::math::VectorVector3d
VectorGeometryVector3ToEigenVector3d(
    const std::vector<geometry_msgs::msg::Vector3>& vector_geom);

common_robotics_utilities::math::VectorIsometry3d
VectorGeometryPoseToVectorIsometry3d(
    const std::vector<geometry_msgs::msg::Pose>& vector_geom);

common_robotics_utilities::math::VectorIsometry3d
VectorGeometryPoseToVectorIsometry3d(
    const std::vector<geometry_msgs::msg::Transform>& vector_geom);

std::vector<geometry_msgs::msg::Pose> VectorIsometry3dToVectorGeometryPose(
    const common_robotics_utilities::math::VectorIsometry3d& vector_eigen);

std::vector<geometry_msgs::msg::Transform> VectorIsometry3dToVectorGeometryTransform(
    const common_robotics_utilities::math::VectorIsometry3d& vector_eigen);
}  // namespace ros_conversions
}  // namespace common_robotics_utilities
