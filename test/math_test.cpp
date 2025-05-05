#include <cmath>
#include <iostream>
#include <vector>

#include <common_robotics_utilities/math.hpp>

#include <gtest/gtest.h>

namespace common_robotics_utilities
{
namespace
{
GTEST_TEST(MathTest, EnforceContinuousRevoluteBounds)
{
  EXPECT_DOUBLE_EQ(math::EnforceContinuousRevoluteBounds(0.0), 0.0);
  EXPECT_DOUBLE_EQ(math::EnforceContinuousRevoluteBounds(1.0), 1.0);
  EXPECT_DOUBLE_EQ(math::EnforceContinuousRevoluteBounds(-1.0), -1.0);
  EXPECT_DOUBLE_EQ(math::EnforceContinuousRevoluteBounds(M_PI), -M_PI);
  EXPECT_DOUBLE_EQ(math::EnforceContinuousRevoluteBounds(-M_PI), -M_PI);
  EXPECT_DOUBLE_EQ(
      math::EnforceContinuousRevoluteBounds(M_PI + 1.0), (-M_PI + 1.0));
  EXPECT_DOUBLE_EQ(
      math::EnforceContinuousRevoluteBounds(-M_PI - 1.0), (M_PI - 1.0));
  EXPECT_DOUBLE_EQ(math::EnforceContinuousRevoluteBounds(2.0 * M_PI), 0.0);
  EXPECT_DOUBLE_EQ(math::EnforceContinuousRevoluteBounds(2.0 * - M_PI), 0.0);

  const std::vector<double> angles =
      {0.0, 1.0, M_PI_4, M_PI_2, M_PI - 1e-6, -1.0, -M_PI_4, -M_PI_2, -M_PI};
  const std::vector<double> wraps = {0.0, 1.0, 2.0, 3.0, -1.0, -2.0, -3.0};
  constexpr double full_wrap = M_PI * 2.0;

  for (const double& angle : angles)
  {
    for (const double& wrap : wraps)
    {
      const double value = angle + (full_wrap * wrap);
      const double wrapped_angle = math::EnforceContinuousRevoluteBounds(value);
      EXPECT_NEAR(angle, wrapped_angle, 1e-9);
    }
  }
}

GTEST_TEST(MathTest, ContinuousRevoluteSignedDistance)
{
  const std::vector<double> starts =
      {0.0, 1.0, M_PI_4, M_PI_2, M_PI, -1.0, -M_PI_4, -M_PI_2, -M_PI};
  const std::vector<double> offsets =
      {0.0, 1.0, M_PI_4, M_PI_2, M_PI - 1e-6, -1.0, -M_PI_4, -M_PI_2, -M_PI};

  for (const double& start : starts)
  {
    for (const double& offset : offsets)
    {
      const double raw_end = start + offset;
      const double end = math::EnforceContinuousRevoluteBounds(raw_end);

      const double raw_signed_distance =
          math::ContinuousRevoluteSignedDistance(start, raw_end);
      const double signed_distance =
          math::ContinuousRevoluteSignedDistance(start, end);
      EXPECT_DOUBLE_EQ(raw_signed_distance, signed_distance);
      EXPECT_NEAR(raw_signed_distance, offset, 1e-9);
      EXPECT_NEAR(signed_distance, offset, 1e-9);

      const double raw_distance =
          math::ContinuousRevoluteDistance(start, raw_end);
      const double distance =
          math::ContinuousRevoluteDistance(start, end);
      EXPECT_DOUBLE_EQ(raw_distance, distance);
      EXPECT_NEAR(raw_distance, std::abs(offset), 1e-9);
      EXPECT_NEAR(distance, std::abs(offset), 1e-9);
    }
  }
}

GTEST_TEST(MathTest, InterpolateContinuousRevolute)
{
  const std::vector<double> starts =
      {0.0, 1.0, M_PI_4, M_PI_2, M_PI, -1.0, -M_PI_4, -M_PI_2, -M_PI};
  const std::vector<double> offsets =
      {0.0, 1.0, M_PI_4, M_PI_2, M_PI - 1e-6, -1.0, -M_PI_4, -M_PI_2, -M_PI};
  const std::vector<double> ratios = {0.0, 0.25, 0.5, 0.75, 1.0};

  for (const double& start : starts)
  {
    for (const double& offset : offsets)
    {
      const double raw_end = start + offset;
      const double end = math::EnforceContinuousRevoluteBounds(raw_end);

      for (const double& ratio : ratios)
      {
        const double interp =
            math::InterpolateContinuousRevolute(start, end, ratio);
        const double check =
            math::EnforceContinuousRevoluteBounds(start + (offset * ratio));
        EXPECT_NEAR(interp, check, 1e-9);
      }
    }
  }
}
}  // namespace
}  // namespace common_robotics_utilities

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
