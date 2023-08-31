#include <cstdint>
#include <iostream>
#include <vector>

#include <common_robotics_utilities/parallelism.hpp>

#include <gtest/gtest.h>

namespace common_robotics_utilities
{
namespace utility_test
{
GTEST_TEST(ParallelismTest, CalcStaticThreadRangeStartAndEndTest)
{
  // Parameter sanity testing
  // range_start > range_end throws
  EXPECT_THROW(
      parallelism::CalcStaticThreadRangeStartAndEnd(1, 0, 1, 1),
      std::invalid_argument);
  // num_threads < 1 throws
  EXPECT_THROW(
      parallelism::CalcStaticThreadRangeStartAndEnd(0, 1, 0, 1),
      std::invalid_argument);
  // thread_num < 0 throws
  EXPECT_THROW(
      parallelism::CalcStaticThreadRangeStartAndEnd(1, 0, 1, -1),
      std::invalid_argument);
  // thread_num >= num_threads throws
  EXPECT_THROW(
      parallelism::CalcStaticThreadRangeStartAndEnd(1, 0, 1, 1),
      std::invalid_argument);

  const std::vector<int64_t> range_starts = {0, 25, 100, 2000, 9001};
  const std::vector<int64_t> range_sizes = {0, 1, 5, 10, 15, 100, 1000, 9001};
  const std::vector<int64_t> thread_counts = {1, 2, 5, 10, 25, 100};

  for (const int64_t range_start : range_starts)
  {
    for (const int64_t range_size : range_sizes)
    {
      for (const int64_t num_threads : thread_counts)
      {
        int64_t previous_range_end = range_start;

        for (int64_t thread_num = 0; thread_num < num_threads; thread_num++)
        {
          const auto start_and_end =
              parallelism::CalcStaticThreadRangeStartAndEnd(
                  range_start, range_start + range_size, num_threads,
                  thread_num);
          const int64_t thread_range_start = start_and_end.first;
          const int64_t thread_range_end = start_and_end.second;

          EXPECT_EQ(previous_range_end, thread_range_start);
          EXPECT_GE(thread_range_end, thread_range_start);
          EXPECT_LE(thread_range_end, range_start + range_size);

          previous_range_end = thread_range_end;
        }

        EXPECT_EQ(previous_range_end, range_start + range_size);
      }
    }
  }
}
}  // namespace parallelism_test
}  // namespace common_robotics_utilities

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
