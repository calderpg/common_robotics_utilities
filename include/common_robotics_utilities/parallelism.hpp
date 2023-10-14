#pragma once

#include <cstdint>
#include <future>
#include <list>
#include <stdexcept>

#include <common_robotics_utilities/cru_namespace.hpp>
#include <common_robotics_utilities/openmp_helpers.hpp>
#include <common_robotics_utilities/utility.hpp>

namespace common_robotics_utilities
{
CRU_NAMESPACE_BEGIN
namespace parallelism
{
/// Abstraction for level of parallelism to use, covering both OpenMP and
/// std::thread-based parallelism.
class DegreeOfParallelism
{
public:
  static DegreeOfParallelism None() { return DegreeOfParallelism(1); }

  static DegreeOfParallelism FromOmp()
  { return DegreeOfParallelism(openmp_helpers::GetNumOmpThreads()); }

  DegreeOfParallelism() : DegreeOfParallelism(1) {}

  explicit DegreeOfParallelism(const int32_t num_threads)
  {
    if (num_threads >= 1)
    {
      num_threads_ = num_threads;
    }
    else
    {
      throw std::invalid_argument(
          "Provided num_threads " + std::to_string(num_threads) +
          " cannot be less than 1");
    }
  }

  bool IsParallel() const { return num_threads_ > 1; }

  int32_t GetNumThreads() const { return num_threads_; }

private:
  int32_t num_threads_ = 1;
};

/// Given a range defined by @param range_start and @param range_end and the
/// total number of threads given by @param num_threads, calculates the start
/// and end of the range to assign to thread @param thread_num.
inline std::pair<int64_t, int64_t> CalcStaticThreadRangeStartAndEnd(
    const int64_t range_start, const int64_t range_end,
    const int64_t num_threads, const int64_t thread_num)
{
  if (range_end < range_start)
  {
    throw std::invalid_argument("range_end < range_start");
  }
  if (num_threads < 1)
  {
    throw std::invalid_argument("num_threads < 1");
  }
  if (thread_num < 0 || thread_num >= num_threads)
  {
    throw std::invalid_argument("thread_num not in [0, num_threads)");
  }

  // Every thread gets at least floor(range_size / num_threads) work, and the
  // remainder is distributed across the first range_size % num_threads as one
  // additional element each. Note that starting with per-thread range of
  // ceil(range_size / num_threads) will not leave enough work for all
  // threads.
  const int64_t range_size = range_end - range_start;
  const int64_t quotient = range_size / num_threads;
  const int64_t remainder = range_size % num_threads;

  const int64_t nominal_range = quotient;
  const int64_t remainder_range = nominal_range + 1;

  if (thread_num < remainder)
  {
    const int64_t thread_range = remainder_range;
    const int64_t thread_range_start =
        range_start + (thread_num * remainder_range);
    const int64_t thread_range_end = thread_range_start + thread_range;
    return std::make_pair(thread_range_start, thread_range_end);
  }
  else
  {
    const int64_t thread_range = nominal_range;
    const int64_t thread_range_start =
        range_start + (remainder * remainder_range) +
            ((thread_num - remainder) * nominal_range);
    const int64_t thread_range_end = thread_range_start + thread_range;
    return std::make_pair(thread_range_start, thread_range_end);
  }
}

class ThreadWorkRange
{
public:
  ThreadWorkRange() = default;

  ThreadWorkRange(
      const int64_t range_start, const int64_t range_end,
      const int32_t thread_num)
      : range_start_(range_start), range_end_(range_end),
        thread_num_(thread_num)
  {
    if (range_end < range_start)
    {
      throw std::invalid_argument("range_end < range_start");
    }
    if (thread_num < 0)
    {
      throw std::invalid_argument("thread_num < 0");
    }
  }

  int64_t GetRangeStart() const { return range_start_; }

  int64_t GetRangeEnd() const { return range_end_; }

  int32_t GetThreadNum() const { return thread_num_; }

private:
  int64_t range_start_ = 0;
  int64_t range_end_ = 0;
  int32_t thread_num_ = 0;
};

enum class ParallelForBackend { OPENMP, ASYNC, BEST_AVAILABLE };

/// Functor must have a signature compatible with
/// functor(const ThreadWorkRange& work_range).
template<typename Functor>
void StaticParallelForLoop(
    const DegreeOfParallelism& parallelism,
    const int64_t range_start, const int64_t range_end,
    const Functor& functor,
    const ParallelForBackend backend = ParallelForBackend::BEST_AVAILABLE)
{
  if (range_end < range_start)
  {
    throw std::runtime_error("range_end < range_start");
  }

  const int64_t total_range = range_end - range_start;
  if (total_range == 0)
  {
    return;
  }

  // If serial execution is specified, or the total range is 1, perform all work
  // in a single thread and work range.
  if (!parallelism.IsParallel() || (total_range == 1))
  {
    const ThreadWorkRange full_range(range_start, range_end, 0);
    functor(full_range);
    return;
  }

  // If there are fewer elements than the specified number of threads, don't
  // dispatch unnecessary threads.
  const int32_t real_num_threads = static_cast<int32_t>(std::min(
      static_cast<int64_t>(parallelism.GetNumThreads()), total_range));

  const auto thread_work = [&](const int32_t thread_num)
  {
    const auto thread_range_start_end = CalcStaticThreadRangeStartAndEnd(
        range_start, range_end, real_num_threads, thread_num);
    const ThreadWorkRange work_range(
        thread_range_start_end.first, thread_range_start_end.second,
        thread_num);
    functor(work_range);
  };

  if (backend == ParallelForBackend::OPENMP ||
      (backend == ParallelForBackend::BEST_AVAILABLE &&
       openmp_helpers::IsOmpEnabledInBuild()))
  {
#if defined(_OPENMP)
#pragma omp parallel for num_threads(real_num_threads) schedule(static)
#endif
    for (int32_t thread_num = 0; thread_num < real_num_threads; thread_num++)
    {
      thread_work(openmp_helpers::GetContextOmpThreadNum());
    }
  }
  else
  {
    // Dispatch worker threads.
    std::vector<std::future<void>> workers;
    for (int64_t thread_num = 0; thread_num < real_num_threads; thread_num++)
    {
      workers.emplace_back(
          std::async(std::launch::async, thread_work, thread_num));
    }

    // Wait for worker threads to complete. This also rethrows any exception
    // thrown in a worker thread.
    for (auto& worker : workers)
    {
      worker.get();
    }
  }
}

/// Functor must have a signature compatible with
/// functor(const ThreadWorkRange& work_range).
template<typename Functor>
void DynamicParallelForLoop(
    const DegreeOfParallelism& parallelism,
    const int64_t range_start, const int64_t range_end,
    const Functor& functor,
    const ParallelForBackend backend = ParallelForBackend::BEST_AVAILABLE)
{
  if (range_end < range_start)
  {
    throw std::runtime_error("range_end < range_start");
  }

  const int64_t total_range = range_end - range_start;
  if (total_range == 0)
  {
    return;
  }

  // If serial execution is specified, do that directly.
  if (!parallelism.IsParallel())
  {
    const ThreadWorkRange full_range(range_start, range_end, 0);
    functor(full_range);
    return;
  }

  // If there are fewer elements than the specified number of threads, don't
  // dispatch unnecessary threads.
  const int32_t real_num_threads = static_cast<int32_t>(std::min(
      static_cast<int64_t>(parallelism.GetNumThreads()), total_range));

  if (backend == ParallelForBackend::OPENMP ||
      (backend == ParallelForBackend::BEST_AVAILABLE &&
       openmp_helpers::IsOmpEnabledInBuild()))
  {
#if defined(_OPENMP)
#pragma omp parallel for num_threads(real_num_threads) schedule(dynamic)
#endif
    for (int64_t index = range_start; index < range_end; index++)
    {
      const ThreadWorkRange work_range(
          index, index + 1, openmp_helpers::GetContextOmpThreadNum());
      functor(work_range);
    }
  }
  else
  {
    // Dispatch worker threads.
    int64_t workers_dispatched = 0;
    size_t num_live_workers = 0;

    std::mutex cv_mutex;
    std::condition_variable cv;

    std::list<std::future<void>> active_workers;

    while (active_workers.size() > 0 || workers_dispatched < total_range)
    {
      // Check for completed workers.
      for (auto worker = active_workers.begin();
           worker != active_workers.end();)
      {
        if (utility::IsFutureReady(*worker))
        {
          // This call to future.get() is necessary to propagate any exception
          // thrown during simulation execution.
          worker->get();
          // Erase returns iterator to the next node in the list.
          worker = active_workers.erase(worker);
        }
        else
        {
          // Advance to next node in the list.
          ++worker;
        }
      }

      // Dispatch new workers.
      while (static_cast<int32_t>(active_workers.size()) < real_num_threads
             && workers_dispatched < total_range)
      {
        {
          std::lock_guard<std::mutex> lock(cv_mutex);
          num_live_workers++;
        }
        active_workers.emplace_back(std::async(
            std::launch::async,
            [&functor, &cv, &cv_mutex, &num_live_workers](
                const int64_t index, const int32_t thread_num)
            {
              const ThreadWorkRange work_range(index, index + 1, thread_num);
              functor(work_range);
              {
                std::lock_guard<std::mutex> lock(cv_mutex);
                num_live_workers--;
              }
              cv.notify_all();
            },
            workers_dispatched,
            static_cast<int32_t>(active_workers.size())));
        workers_dispatched++;
      }

      // Wait until a worker completes.
      if (active_workers.size() > 0)
      {
        std::unique_lock<std::mutex> wait_lock(cv_mutex);
        cv.wait(
            wait_lock,
            [&num_live_workers, &active_workers]()
            {
              return num_live_workers < active_workers.size();
            });
      }
    }
  }
}
}  // namespace openmp_helpers
CRU_NAMESPACE_END
}  // namespace common_robotics_utilities
