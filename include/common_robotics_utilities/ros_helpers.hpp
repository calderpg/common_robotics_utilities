#pragma once

#include <vector>

#if COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION == 2
#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#elif COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION == 1
#include <ros/time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#else
#error "Undefined or unknown COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION"
#endif

namespace common_robotics_utilities
{
namespace ros_helpers
{

#if COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION == 2
using RosTime = rclcpp::Time;
#elif COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION == 1
using RosTime = ros::Time;
#endif

template <typename MessageType, typename Container = std::vector<MessageType>>
void SetMessageTimestamps(Container& messages, const RosTime& timestamp)
{
  for (auto& message : messages)
  {
    message.header.stamp = timestamp;
  }
}

#if COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION == 2
using VisualizationMarker = visualization_msgs::msg::Marker;
using VisualizationMarkerArray = visualization_msgs::msg::MarkerArray;
#elif COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION == 1
using VisualizationMarker = visualization_msgs::Marker;
using VisualizationMarkerArray = visualization_msgs::MarkerArray;
#endif

void SetMessageTimestamps(VisualizationMarkerArray& markers, const RosTime& timestamp)
{
  SetMessageTimestamps<VisualizationMarker>(markers.markers, timestamp);
}

}  // namespace ros_helpers
}  // namespace common_robotics_utilities
