#include <common_robotics_utilities/ros_helpers.hpp>

#include <gtest/gtest.h>

namespace common_robotics_utilities
{
namespace ros_helpers
{
namespace
{

#if COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION == 2
using RosDuration = rclcpp::Duration;
#elif COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION == 1
using RosDuration = ros::Duration;
#endif

GTEST_TEST(ROSHelpersTest, SetMessageTimestamps)
{
  VisualizationMarkerArray marker_array;
  marker_array.markers.resize(5);
  // Note that the ROS time type depends on which ROS variant is in use.
  // Get marker base timestamp (ensuring clock sources match).
  const RosTime base_stamp = marker_array.markers[0].header.stamp;
  // Advance timestamps by one second.
  const RosTime expected_stamp = base_stamp + RosDuration(1, 0);
  SetMessageTimestamps(marker_array, expected_stamp);
  for (const auto& marker : marker_array.markers) {
    EXPECT_EQ(marker.header.stamp, expected_stamp);
  }
}

}  // namespace
}  // namespace ros_helpers_test
}  // namespace common_robotics_utilities

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
