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
using Time = rclcpp::Time;
#elif COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION == 1
using Time = ros::Time;
#endif

template <typename MessageType, typename Container = std::vector<MessageType>>
void SetMessageTimestamps(Container& messages, const Time& timestamp)
{
  for (auto& message : messages)
  {
    message.header.stamp = timestamp;
  }
}

#if COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION == 2
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
#elif COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION == 1
using Marker = visualization_msgs::Marker;
using MarkerArray = visualization_msgs::MarkerArray;
#endif

void SetMessageTimestamps(MarkerArray& markers, const Time& timestamp)
{
  SetMessageTimestamps<Marker>(markers.markers, timestamp);
}

}  // namespace ros_helpers
}  // namespace common_robotics_utilities
