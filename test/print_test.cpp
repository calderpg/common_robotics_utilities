#include <vector>

#if COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION == 2
#include <geometry_msgs/msg/point.hpp>
#elif COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION == 1
#include <geometry_msgs/Point.h>
#else
#error "Undefined or unknown COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION"
#endif

#include <common_robotics_utilities/print.hpp>
#include <common_robotics_utilities/voxel_grid.hpp>

#include <gtest/gtest.h>

namespace
{
namespace test
{
struct DummyType
{
  int32_t value = 0;
};
}  // namespace test

std::ostream& operator<<(
    std::ostream& os, const test::DummyType& dummy)
{
  return os << dummy.value;
}
}  // namespace

namespace common_robotics_utilities
{
namespace print_test
{
GTEST_TEST(PrintTest, CanPrintIfADLCanFindStreamOperator)
{
  std::cout << print::Print(std::vector<voxel_grid::GridIndex>{{1, 1, 1}}) << std::endl;
  std::cout << print::Print(std::vector<test::DummyType>(3)) << std::endl;
}

GTEST_TEST(PrintTest, CanPrintROSMessages)
{
#if COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION == 2
  using Point = geometry_msgs::msg::Point;
#elif COMMON_ROBOTICS_UTILITIES__SUPPORTED_ROS_VERSION == 1
  using Point = geometry_msgs::Point;
#endif
  std::cout << print::Print(std::vector<Point>(3)) << std::endl;
}
}  // namespace print_test
}  // namespace common_robotics_utilities

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
