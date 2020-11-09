#include <common_robotics_utilities/ros_helpers.hpp>

#include <gtest/gtest.h>

namespace common_robotics_utilities
{
namespace ros_helpers
{
namespace
{

#if COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION == 2
using Duration = rclcpp::Duration;
#elif COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION == 1
using Duration = ros::Duration;
#endif

GTEST_TEST(ROSHelpersTest, SetMessageTimestamps)
{
  Marker marker;
  MarkerArray marker_array;
  marker_array.markers.push_back(marker);
  Time base_stamp = marker.header.stamp;  // handle clock types (indirectly)
  Time expected_stamp = base_stamp + Duration(1, 0);  // +1 second
  SetMessageTimestamps(marker_array, expected_stamp);
  EXPECT_EQ(expected_stamp, marker_array.markers[0].header.stamp);
}

}  // namespace
}  // namespace ros_helpers_test
}  // namespace common_robotics_utilities

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
