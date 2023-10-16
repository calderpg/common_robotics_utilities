#include <cstdint>
#include <iostream>
#include <mutex>
#include <random>
#include <set>
#include <vector>

#include <common_robotics_utilities/maybe.hpp>
#include <common_robotics_utilities/parallelism.hpp>

#include <gtest/gtest.h>

namespace common_robotics_utilities
{
namespace parallelism_test
{
constexpr size_t kNumTrackers = 1000000;
constexpr size_t kNumInts = 1000000;

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

class ThreadTracker
{
public:
  ThreadTracker() = default;

  void RecordThread(const int32_t thread_id)
  {
    std::lock_guard<std::mutex> lock(thread_set_mutex_);
    thread_set_.insert(thread_id);
  }

  size_t NumThreads() const
  {
    std::lock_guard<std::mutex> lock(thread_set_mutex_);
    return thread_set_.size();
  }

private:
  mutable std::mutex thread_set_mutex_;
  std::set<int32_t> thread_set_;
};

std::vector<int64_t> GenerateRandomInt64Sequence(const size_t length)
{
  std::mt19937_64 prng(42);
  std::uniform_int_distribution<int64_t> dist(-1000000, 1000000);
  std::vector<int64_t> values(length);

  for (size_t index = 0; index < length; index++)
  {
    values[index] = dist(prng);
  }

  return values;
}

void StaticParallelRecordThreads(
    std::vector<ThreadTracker>& trackers,
    const parallelism::DegreeOfParallelism& parallelism,
    const parallelism::ParallelForBackend backend)
{
  const auto per_thread_work = [&trackers](
      const parallelism::ThreadWorkRange& work_range)
  {
    for (int64_t index = work_range.GetRangeStart();
         index < work_range.GetRangeEnd();
         index++)
    {
      trackers.at(static_cast<size_t>(index)).RecordThread(
          work_range.GetThreadNum());
    }
  };

  parallelism::StaticParallelForLoop(
      parallelism, 0, static_cast<int64_t>(trackers.size()), per_thread_work,
      backend);
}

void DynamicParallelRecordThreads(
    std::vector<ThreadTracker>& trackers,
    const parallelism::DegreeOfParallelism& parallelism,
    const parallelism::ParallelForBackend backend)
{
  const auto per_thread_work = [&trackers](
      const parallelism::ThreadWorkRange& work_range)
  {
    for (int64_t index = work_range.GetRangeStart();
         index < work_range.GetRangeEnd();
         index++)
    {
      trackers.at(static_cast<size_t>(index)).RecordThread(
          work_range.GetThreadNum());
    }
  };

  parallelism::DynamicParallelForLoop(
      parallelism, 0, static_cast<int64_t>(trackers.size()), per_thread_work,
      backend);
}

int64_t StaticParallelSum(
    const std::vector<int64_t>& elements,
    const parallelism::DegreeOfParallelism& parallelism,
    const parallelism::ParallelForBackend backend)
{
  std::vector<int64_t> parallel_thread_sums(parallelism.GetNumThreads(), 0);

  const auto per_thread_work = [&elements, &parallel_thread_sums](
      const parallelism::ThreadWorkRange& work_range)
  {
    for (int64_t index = work_range.GetRangeStart();
         index < work_range.GetRangeEnd();
         index++)
    {
      parallel_thread_sums.at(static_cast<size_t>(work_range.GetThreadNum()))
          += elements.at(static_cast<size_t>(index));
    }
  };

  parallelism::StaticParallelForLoop(
      parallelism, 0, static_cast<int64_t>(elements.size()), per_thread_work,
      backend);

  int64_t parallel_sum = 0;
  for (const int64_t thread_sum : parallel_thread_sums)
  {
    parallel_sum += thread_sum;
  }

  return parallel_sum;
}

int64_t DynamicParallelSum(
    const std::vector<int64_t>& elements,
    const parallelism::DegreeOfParallelism& parallelism,
    const parallelism::ParallelForBackend backend)
{
  std::vector<int64_t> parallel_thread_sums(parallelism.GetNumThreads(), 0);

  const auto per_thread_work = [&elements, &parallel_thread_sums](
      const parallelism::ThreadWorkRange& work_range)
  {
    for (int64_t index = work_range.GetRangeStart();
         index < work_range.GetRangeEnd();
         index++)
    {
      parallel_thread_sums.at(static_cast<size_t>(work_range.GetThreadNum()))
          += elements.at(static_cast<size_t>(index));
    }
  };

  parallelism::DynamicParallelForLoop(
      parallelism, 0, static_cast<int64_t>(elements.size()), per_thread_work,
      backend);

  int64_t parallel_sum = 0;
  for (const int64_t thread_sum : parallel_thread_sums)
  {
    parallel_sum += thread_sum;
  }

  return parallel_sum;
}

class ParallelismTestSuite
    : public testing::TestWithParam<parallelism::DegreeOfParallelism> {};

TEST_P(ParallelismTestSuite, StaticParallelForTest)
{
  const parallelism::DegreeOfParallelism parallelism = GetParam();
  std::cout << "# of threads = " << parallelism.GetNumThreads() << std::endl;

  {
    std::vector<ThreadTracker> trackers(kNumTrackers);
    StaticParallelRecordThreads(
        trackers, parallelism, parallelism::ParallelForBackend::BEST_AVAILABLE);
    for (const auto& tracker : trackers)
    {
      EXPECT_EQ(tracker.NumThreads(), 1u);
    }
  }

  {
    std::vector<ThreadTracker> trackers(kNumTrackers);
    StaticParallelRecordThreads(
        trackers, parallelism, parallelism::ParallelForBackend::OPENMP);
    for (const auto& tracker : trackers)
    {
      EXPECT_EQ(tracker.NumThreads(), 1u);
    }
  }

  {
    std::vector<ThreadTracker> trackers(kNumTrackers);
    StaticParallelRecordThreads(
        trackers, parallelism, parallelism::ParallelForBackend::ASYNC);
    for (const auto& tracker : trackers)
    {
      EXPECT_EQ(tracker.NumThreads(), 1u);
    }
  }
}

TEST_P(ParallelismTestSuite, DynamicParallelForTest)
{
  const parallelism::DegreeOfParallelism parallelism = GetParam();
  std::cout << "# of threads = " << parallelism.GetNumThreads() << std::endl;

  {
    std::vector<ThreadTracker> trackers(kNumTrackers);
    DynamicParallelRecordThreads(
        trackers, parallelism, parallelism::ParallelForBackend::BEST_AVAILABLE);
    for (const auto& tracker : trackers)
    {
      EXPECT_EQ(tracker.NumThreads(), 1u);
    }
  }

  {
    std::vector<ThreadTracker> trackers(kNumTrackers);
    DynamicParallelRecordThreads(
        trackers, parallelism, parallelism::ParallelForBackend::OPENMP);
    for (const auto& tracker : trackers)
    {
      EXPECT_EQ(tracker.NumThreads(), 1u);
    }
  }

  {
    std::vector<ThreadTracker> trackers(kNumTrackers);
    DynamicParallelRecordThreads(
        trackers, parallelism, parallelism::ParallelForBackend::ASYNC);
    for (const auto& tracker : trackers)
    {
      EXPECT_EQ(tracker.NumThreads(), 1u);
    }
  }
}

TEST_P(ParallelismTestSuite, StaticParallelForSumTest)
{
  const parallelism::DegreeOfParallelism parallelism = GetParam();
  std::cout << "# of threads = " << parallelism.GetNumThreads() << std::endl;

  const auto elements = GenerateRandomInt64Sequence(kNumInts);

  int64_t serial_sum = 0;
  for (const int64_t value : elements)
  {
    serial_sum += value;
  }

  const int64_t best_backend_sum = StaticParallelSum(
      elements, parallelism, parallelism::ParallelForBackend::BEST_AVAILABLE);
  EXPECT_EQ(serial_sum, best_backend_sum);

  const int64_t openmp_backend_sum = StaticParallelSum(
      elements, parallelism, parallelism::ParallelForBackend::OPENMP);
  EXPECT_EQ(serial_sum, openmp_backend_sum);

  const int64_t async_backend_sum = StaticParallelSum(
      elements, parallelism, parallelism::ParallelForBackend::ASYNC);
  EXPECT_EQ(serial_sum, async_backend_sum);
}

TEST_P(ParallelismTestSuite, DynamicParallelForSumTest)
{
  const parallelism::DegreeOfParallelism parallelism = GetParam();
  std::cout << "# of threads = " << parallelism.GetNumThreads() << std::endl;

  const auto elements = GenerateRandomInt64Sequence(kNumInts);

  int64_t serial_sum = 0;
  for (const int64_t value : elements)
  {
    serial_sum += value;
  }

  const int64_t best_backend_sum = DynamicParallelSum(
      elements, parallelism, parallelism::ParallelForBackend::BEST_AVAILABLE);
  EXPECT_EQ(serial_sum, best_backend_sum);

  const int64_t openmp_backend_sum = DynamicParallelSum(
      elements, parallelism, parallelism::ParallelForBackend::OPENMP);
  EXPECT_EQ(serial_sum, openmp_backend_sum);

  const int64_t async_backend_sum = DynamicParallelSum(
      elements, parallelism, parallelism::ParallelForBackend::ASYNC);
  EXPECT_EQ(serial_sum, async_backend_sum);
}

INSTANTIATE_TEST_SUITE_P(
    SerialParallelismTest, ParallelismTestSuite,
    testing::Values(parallelism::DegreeOfParallelism::None()));

// For fallback testing on platforms with no OpenMP support, specify 2 threads.
int32_t GetNumThreads()
{
  if (openmp_helpers::IsOmpEnabledInBuild())
  {
    return openmp_helpers::GetNumOmpThreads();
  }
  else
  {
    return 2;
  }
}

INSTANTIATE_TEST_SUITE_P(
    ParallelParallelismTest, ParallelismTestSuite,
    testing::Values(parallelism::DegreeOfParallelism(GetNumThreads())));
}  // namespace parallelism_test
}  // namespace common_robotics_utilities

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
