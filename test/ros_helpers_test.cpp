#include <common_robotics_utilities/ros_helpers.hpp>

#include <gtest/gtest.h>

namespace common_robotics_utilities
{
namespace ros_helpers
{
namespace
{

GTEST_TEST(ROSHelpersTest, SetMessageTimestamps)
{
  VisualizationMarkerArray marker_array;
  marker_array.markers.resize(5);
  // Note that the ROS time type depends on which ROS variant is in use.
#if COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION == 2
  // Base time measured against ROS clock.
  const rclcpp::Time base_stamp(0LL, RCL_ROS_TIME);
  // Advance time by one second.
  const rclcpp::Time expected_stamp =
    base_stamp + rclcpp::Duration(1, 0);
#elif COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION == 1
  // Default constructed base time.
  const ros::Time base_stamp;
  // Advance time by one second.
  const ros::Time expected_stamp =
    base_stamp + ros::Duration(1.0);
#endif
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
