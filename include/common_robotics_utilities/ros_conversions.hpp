#pragma once

#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/math.hpp>

#if COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION == 2
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#elif COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION == 1
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#else
#error "Undefined or unknown COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION"
#endif

namespace common_robotics_utilities
{
namespace ros_conversions
{

#if COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION == 2
using GeometryPoint = geometry_msgs::msg::Point;
using GeometryPointStamped = geometry_msgs::msg::PointStamped;
using GeometryPose = geometry_msgs::msg::Pose;
using GeometryPoseStamped = geometry_msgs::msg::PoseStamped;
using GeometryQuaternion = geometry_msgs::msg::Quaternion;
using GeometryTransform = geometry_msgs::msg::Transform;
using GeometryTransformStamped = geometry_msgs::msg::TransformStamped;
using GeometryVector3 = geometry_msgs::msg::Vector3;
#elif COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION == 1
using GeometryPoint = geometry_msgs::Point;
using GeometryPointStamped = geometry_msgs::PointStamped;
using GeometryPose = geometry_msgs::Pose;
using GeometryPoseStamped = geometry_msgs::PoseStamped;
using GeometryQuaternion = geometry_msgs::Quaternion;
using GeometryTransform = geometry_msgs::Transform;
using GeometryTransformStamped = geometry_msgs::TransformStamped;
using GeometryVector3 = geometry_msgs::Vector3;
#endif

Eigen::Vector3d GeometryPointToEigenVector3d(const GeometryPoint& point);

GeometryPoint EigenVector3dToGeometryPoint(const Eigen::Vector3d& point);

Eigen::Vector4d GeometryPointToEigenVector4d(const GeometryPoint& point);

GeometryPoint EigenVector4dToGeometryPoint(const Eigen::Vector4d& point);

GeometryPointStamped EigenVector3dToGeometryPointStamped(
    const Eigen::Vector3d& point, const std::string& frame_id);

Eigen::Vector3d GeometryVector3ToEigenVector3d(const GeometryVector3& vector);

GeometryVector3 EigenVector3dToGeometryVector3(const Eigen::Vector3d& vector);

Eigen::Vector4d GeometryVector3ToEigenVector4d(const GeometryVector3& vector);

GeometryVector3 EigenVector4dToGeometryVector3(const Eigen::Vector4d& vector);

Eigen::Quaterniond GeometryQuaternionToEigenQuaterniond(
    const GeometryQuaternion& quat);

GeometryQuaternion EigenQuaterniondToGeometryQuaternion(
    const Eigen::Quaterniond& quat);

Eigen::Isometry3d GeometryPoseToEigenIsometry3d(const GeometryPose& pose);

GeometryPose EigenIsometry3dToGeometryPose(const Eigen::Isometry3d& transform);

GeometryPoseStamped EigenIsometry3dToGeometryPoseStamped(
    const Eigen::Isometry3d& transform, const std::string& frame_id);

Eigen::Isometry3d GeometryTransformToEigenIsometry3d(
    const GeometryTransform& transform);

GeometryTransform EigenIsometry3dToGeometryTransform(
    const Eigen::Isometry3d& transform);

GeometryTransformStamped EigenIsometry3dToGeometryTransformStamped(
    const Eigen::Isometry3d& transform, const std::string& frame_id,
    const std::string& child_frame_id);

Eigen::Matrix3Xd VectorGeometryPointToEigenMatrix3Xd(
    const std::vector<GeometryPoint>& vector_geom);

std::vector<GeometryPoint> EigenMatrix3XdToVectorGeometryPoint(
    const Eigen::Matrix3Xd& eigen_matrix);

std::vector<GeometryPoint>
VectorEigenVector3dToVectorGeometryPoint(
    const common_robotics_utilities::math::VectorVector3d& vector_eigen);

common_robotics_utilities::math::VectorVector3d
VectorGeometryPointToVectorEigenVector3d(
    const std::vector<GeometryPoint>& vector_geom);

common_robotics_utilities::math::VectorVector3d
VectorGeometryVector3ToEigenVector3d(
    const std::vector<GeometryVector3>& vector_geom);

common_robotics_utilities::math::VectorIsometry3d
VectorGeometryPoseToVectorIsometry3d(
    const std::vector<GeometryPose>& vector_geom);

common_robotics_utilities::math::VectorIsometry3d
VectorGeometryPoseToVectorIsometry3d(
    const std::vector<GeometryTransform>& vector_geom);

std::vector<GeometryPose> VectorIsometry3dToVectorGeometryPose(
    const common_robotics_utilities::math::VectorIsometry3d& vector_eigen);

std::vector<GeometryTransform> VectorIsometry3dToVectorGeometryTransform(
    const common_robotics_utilities::math::VectorIsometry3d& vector_eigen);
}  // namespace ros_conversions
}  // namespace common_robotics_utilities
