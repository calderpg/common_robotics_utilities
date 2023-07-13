#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <future>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <common_robotics_utilities/openmp_helpers.hpp>
#include <common_robotics_utilities/utility.hpp>

namespace common_robotics_utilities
{
namespace simple_knearest_neighbors
{
/// Type to wrap an index and its corresponding distance.
class IndexAndDistance
{
private:
  int64_t index_ = -1;
  double distance_ = std::numeric_limits<double>::infinity();

public:
  IndexAndDistance() {}

  IndexAndDistance(const int64_t index, const double distance)
      : index_(index), distance_(distance) {}

  IndexAndDistance(const size_t index, const double distance)
      : index_(static_cast<int64_t>(index)), distance_(distance) {}

  void SetIndexAndDistance(const int64_t index, const double distance)
  {
    index_ = index;
    distance_ = distance;
  }

  void SetIndexAndDistance(const size_t index, const double distance)
  {
    SetIndexAndDistance(static_cast<int64_t>(index), distance);
  }

  void SetFromOther(const IndexAndDistance& other)
  {
    SetIndexAndDistance(other.Index(), other.Distance());
  }

  int64_t Index() const { return index_; }

  double Distance() const { return distance_; }
};

/// Comparison function for std::max_element.
inline bool IndexAndDistanceCompare(
    const IndexAndDistance& index1, const IndexAndDistance& index2)
{
  return (index1.Distance() < index2.Distance());
}

/// @return vector<pair<index, distance>> of the nearest @param K neighbors to
/// @param current in the specified range [range_start, range_end) of
/// @param items, with distance computed by @param distance_fn.
template<typename Item, typename Value, typename Container=std::vector<Item>>
inline std::vector<IndexAndDistance> GetKNearestNeighborsInRangeSerial(
    const Container& items, const size_t range_start, const size_t range_end,
    const Value& current,
    const std::function<double(const Item&, const Value&)>& distance_fn,
    const int64_t K)
{
  if (range_end < range_start)
  {
    throw std::invalid_argument("range_end < range_start");
  }
  if (range_end > items.size())
  {
    throw std::invalid_argument("range_end > items.size()");
  }

  const size_t range_size = range_end - range_start;

  if (K < 0)
  {
    throw std::invalid_argument("K must be >= 0");
  }
  else if (K == 0 || range_size == 0)
  {
    return std::vector<IndexAndDistance>();
  }
  else if (range_size <= static_cast<size_t>(K))
  {
    std::vector<IndexAndDistance> k_nearests(range_size);
    for (size_t idx = range_start; idx < range_end; idx++)
    {
      const Item& item = items[idx];
      const double distance = distance_fn(item, current);
      k_nearests[idx - range_start].SetIndexAndDistance(idx, distance);
    }
    return k_nearests;
  }
  else
  {
    // Collect index + distance for all items.
    std::vector<IndexAndDistance> all_distances(range_size);
    for (size_t idx = range_start; idx < range_end; idx++)
    {
      const Item& item = items[idx];
      const double distance = distance_fn(item, current);
      all_distances[idx - range_start] = IndexAndDistance(idx, distance);
    }

    // Sort the K smallest elements by distance.
    // Uses an O(range_size * log(K)) approach.
    std::partial_sort(
        all_distances.begin(), all_distances.begin() + K, all_distances.end(),
        IndexAndDistanceCompare);

    // Return the first K elements by resizing down.
    all_distances.resize(static_cast<size_t>(K));
    return all_distances;
  }
}

/// @return vector<pair<index, distance>> of the nearest @param K neighbors to
/// @param current in the specified range [range_start, range_end) of
/// @param items, with distance computed by @param distance_fn and search
/// performed in parallel.
/// @param parallelism control if/how search is performed in parallel.
template<typename Item, typename Value, typename Container=std::vector<Item>>
inline std::vector<IndexAndDistance> GetKNearestNeighborsInRangeParallel(
    const Container& items, const size_t range_start, const size_t range_end,
    const Value& current,
    const std::function<double(const Item&, const Value&)>& distance_fn,
    const int64_t K, const openmp_helpers::DegreeOfParallelism& parallelism)
{
  if (range_end < range_start)
  {
    throw std::invalid_argument("range_end < range_start");
  }
  if (range_end > items.size())
  {
    throw std::invalid_argument("range_end > items.size()");
  }

  const size_t range_size = range_end - range_start;

  // Handle the easy cases where no comparisons must be performed.
  if (K < 0)
  {
    throw std::invalid_argument("K must be >= 0");
  }
  else if (K == 0 || range_size == 0)
  {
    return std::vector<IndexAndDistance>();
  }
  else
  {
    // Per-thread work calculation common to both range_size <= K and full case.
    const int64_t num_threads = parallelism.GetNumThreads();

    // Easy case where we only need to compute distance for each element.
    if (range_size <= static_cast<size_t>(K))
    {
      std::vector<IndexAndDistance> k_nearests(range_size);

      // Helper lambda for each thread's work.
      const auto per_thread_work = [&](const int64_t thread_num)
      {
        const auto thread_range_start_end = utility::CalcThreadRangeStartAndEnd(
            static_cast<int64_t>(range_start), static_cast<int64_t>(range_end),
            num_threads, thread_num);

        for (size_t index = static_cast<size_t>(thread_range_start_end.first);
             index < static_cast<size_t>(thread_range_start_end.second);
             index++)
        {
          const Item& item = items[index];
          const double distance = distance_fn(item, current);
          k_nearests[index - range_start].SetIndexAndDistance(index, distance);
        }
      };

      // Find the K nearest for each thread. Use OpenMP if available, if not
      // fall back to manual dispatch via std::async.
      if (openmp_helpers::IsOmpEnabledInBuild())
      {
        CRU_OMP_PARALLEL_FOR_DEGREE(parallelism)
        for (int64_t thread_num = 0; thread_num < num_threads; thread_num++)
        {
          per_thread_work(thread_num);
        }
      }
      else
      {
        // Dispatch worker threads.
        std::vector<std::future<void>> workers;
        for (int64_t thread_num = 0; thread_num < num_threads; thread_num++)
        {
          workers.emplace_back(
              std::async(std::launch::async, per_thread_work, thread_num));
        }

        // Wait for worker threads to complete. This also rethrows any exception
        // thrown in a worker thread.
        for (auto& worker : workers)
        {
          worker.get();
        }
      }

      return k_nearests;
    }
    // Where real work is required, divide items into per-thread blocks, find
    // the K nearest in each block (in parallel), then merge those results
    // together.
    else
    {
      // Allocate per-worker-thread storage.
      std::vector<std::vector<IndexAndDistance>>
          per_thread_nearests(num_threads);

      // Helper lambda for each thread's work.
      const auto per_thread_work = [&](const int64_t thread_num)
      {
        const auto thread_range_start_end = utility::CalcThreadRangeStartAndEnd(
            static_cast<int64_t>(range_start), static_cast<int64_t>(range_end),
            num_threads, thread_num);

        per_thread_nearests[thread_num] =
            GetKNearestNeighborsInRangeSerial<Item, Value, Container>(
                items, static_cast<size_t>(thread_range_start_end.first),
                static_cast<size_t>(thread_range_start_end.second), current,
                distance_fn, K);
      };

      // Find the K nearest for each thread. Use OpenMP if available, if not
      // fall back to manual dispatch via std::async.
      if (openmp_helpers::IsOmpEnabledInBuild())
      {
        CRU_OMP_PARALLEL_FOR_DEGREE(parallelism)
        for (int64_t thread_num = 0; thread_num < num_threads; thread_num++)
        {
          per_thread_work(thread_num);
        }
      }
      else
      {
        // Dispatch worker threads.
        std::vector<std::future<void>> workers;
        for (int64_t thread_num = 0; thread_num < num_threads; thread_num++)
        {
          workers.emplace_back(
              std::async(std::launch::async, per_thread_work, thread_num));
        }

        // Wait for worker threads to complete. This also rethrows any exception
        // thrown in a worker thread.
        for (auto& worker : workers)
        {
          worker.get();
        }
      }

      // Merge the per-thread K-nearest together.
      // Uses an O((K * num_threads) * log(K)) approach.
      std::vector<IndexAndDistance> all_nearests;
      all_nearests.reserve(static_cast<size_t>(K) * per_thread_nearests.size());
      for (const auto& thread_nearests : per_thread_nearests)
      {
        all_nearests.insert(
            all_nearests.end(), thread_nearests.begin(), thread_nearests.end());
      }

      std::partial_sort(
          all_nearests.begin(), all_nearests.begin() + K, all_nearests.end(),
          IndexAndDistanceCompare);

      // Return the first K elements by resizing down.
      all_nearests.resize(static_cast<size_t>(K));
      return all_nearests;
    }
  }
}

/// @return vector<pair<index, distance>> of the nearest @param K neighbors to
/// @param current in @param items, with distance computed by @param
/// distance_fn.
template<typename Item, typename Value, typename Container=std::vector<Item>>
inline std::vector<IndexAndDistance> GetKNearestNeighborsSerial(
    const Container& items, const Value& current,
    const std::function<double(const Item&, const Value&)>& distance_fn,
    const int64_t K)
{
  return GetKNearestNeighborsInRangeSerial<Item, Value, Container>(
      items, 0, items.size(), current, distance_fn, K);
}

/// @return vector<pair<index, distance>> of the nearest @param K neighbors to
/// @param current in @param items, with distance computed by @param distance_fn
/// and search performed in parallel.
template<typename Item, typename Value, typename Container=std::vector<Item>>
inline std::vector<IndexAndDistance> GetKNearestNeighborsParallel(
    const Container& items, const Value& current,
    const std::function<double(const Item&, const Value&)>& distance_fn,
    const int64_t K, const openmp_helpers::DegreeOfParallelism& parallelism)
{
  return GetKNearestNeighborsInRangeParallel<Item, Value, Container>(
      items, 0, items.size(), current, distance_fn, K, parallelism);
}

/// @return vector<pair<index, distance>> of the nearest @param K neighbors to
/// @param current in the specified range [range_start, range_end) of
/// @param items, with distance computed by @param distance_fn and
/// @param parallelism control if/how search is performed in parallel.
template<typename Item, typename Value, typename Container=std::vector<Item>>
inline std::vector<IndexAndDistance> GetKNearestNeighborsInRange(
    const Container& items, const size_t range_start, const size_t range_end,
    const Value& current,
    const std::function<double(const Item&, const Value&)>& distance_fn,
    const int64_t K, const openmp_helpers::DegreeOfParallelism& parallelism)
{
  if (parallelism.IsParallel())
  {
    return GetKNearestNeighborsInRangeParallel<Item, Value, Container>(
        items, range_start, range_end, current, distance_fn, K, parallelism);
  }
  else
  {
    return GetKNearestNeighborsInRangeSerial<Item, Value, Container>(
        items, range_start, range_end, current, distance_fn, K);
  }
}

/// @return vector<pair<index, distance>> of the nearest @param K neighbors to
/// @param current in @param items, with distance computed by @param distance_fn
/// @param parallelism control if/how search is performed in parallel.
template<typename Item, typename Value, typename Container=std::vector<Item>>
inline std::vector<IndexAndDistance> GetKNearestNeighbors(
    const Container& items, const Value& current,
    const std::function<double(const Item&, const Value&)>& distance_fn,
    const int64_t K, const openmp_helpers::DegreeOfParallelism& parallelism)
{
  return GetKNearestNeighborsInRange<Item, Value, Container>(
      items, 0, items.size(), current, distance_fn, K, parallelism);
}
}  // namespace simple_knearest_neighbors
}  // namespace common_robotics_utilities
