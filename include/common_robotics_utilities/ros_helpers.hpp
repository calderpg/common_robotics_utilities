#pragma once

#include <ostream>
#include <type_traits>
#include <vector>

#include <common_robotics_utilities/cru_namespace.hpp>

// The rosidl_runtime_cpp/traits.hpp header is missing this include, so we
// include it first as a workaround.
#include <cstdint>
#include <rosidl_runtime_cpp/traits.hpp>
// On pre-Humble distributions, a single message header is included to ensure
// that rosidl_generator_traits::to_yaml is found by the compiler. The version
// header is only present in Humble or later, so we use the absence of the file
// to detect earlier distributions.
#if !__has_include(<rclcpp/version.h>)
#include <builtin_interfaces/msg/duration.hpp>
#endif
#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

/// Define an ostream operator for ROS 2 message types.
namespace std
{
template <typename T>
enable_if_t<rosidl_generator_traits::is_message<T>::value, ostream>&
operator<<(ostream& os, const T& message)
{
#if !__has_include(<rclcpp/version.h>)
  // ROS 2 Galactic only supports block-style output.
  os << rosidl_generator_traits::to_yaml(message);
#else
  // ROS 2 Humble and later support optional flow-style output found via ADL.
  constexpr bool use_flow_style = false;
  os << to_yaml(message, use_flow_style);
#endif
  return os;
}
}  // namespace std

namespace common_robotics_utilities
{
CRU_NAMESPACE_BEGIN
namespace ros_helpers
{
using RosTime = rclcpp::Time;

template <typename MessageType, typename Container = std::vector<MessageType>>
void SetMessageTimestamps(Container& messages, const RosTime& timestamp)
{
  for (auto& message : messages)
  {
    message.header.stamp = timestamp;
  }
}

using VisualizationMarker = visualization_msgs::msg::Marker;
using VisualizationMarkerArray = visualization_msgs::msg::MarkerArray;

inline void SetMessageTimestamps(
    VisualizationMarkerArray& markers, const RosTime& timestamp)
{
  SetMessageTimestamps<VisualizationMarker>(markers.markers, timestamp);
}

}  // namespace ros_helpers
CRU_NAMESPACE_END
}  // namespace common_robotics_utilities
