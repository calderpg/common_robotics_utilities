#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <common_robotics_utilities/openmp_helpers.hpp>

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
/// @param current in @param items, with distance computed by @param distance_fn
/// and search performed in parallel.
template<typename Item, typename Value, typename Container=std::vector<Item>>
inline std::vector<IndexAndDistance> GetKNearestNeighborsParallel(
    const Container& items, const Value& current,
    const std::function<double(const Item&, const Value&)>& distance_fn,
    const int64_t K)
{
  // Helper to merge per-thread K-nearests into a single K-nearests.
  const auto merge_per_thread_nearests = [K] (
      const std::vector<std::vector<IndexAndDistance>>& per_thread_nearests)
  {
    // For small K, use a num_threads * K^2 approach.
    if (static_cast<double>(K) < std::log2(per_thread_nearests.size()))
    {
      std::vector<IndexAndDistance> k_nearests(static_cast<size_t>(K));
      for (const auto& thread_nearests : per_thread_nearests)
      {
        for (const auto& current_ith_nearest : thread_nearests)
        {
          if (!std::isinf(current_ith_nearest.Distance())
              && current_ith_nearest.Index() != -1)
          {
            auto itr = std::max_element(
                k_nearests.begin(), k_nearests.end(), IndexAndDistanceCompare);
            if (itr->Distance() > current_ith_nearest.Distance())
            {
              itr->SetFromOther(current_ith_nearest);
            }
          }
        }
      }
      return k_nearests;
    }
    // For larger K, use a (K * num_threads) * log(K * num_threads) approach.
    else
    {
      std::vector<IndexAndDistance> all_nearests;
      all_nearests.reserve(static_cast<size_t>(K) * per_thread_nearests.size());
      for (const auto& thread_nearests : per_thread_nearests)
      {
        all_nearests.insert(
            all_nearests.end(), thread_nearests.begin(), thread_nearests.end());
      }

      std::sort(
          all_nearests.begin(), all_nearests.end(), IndexAndDistanceCompare);

      std::vector<IndexAndDistance> k_nearests(static_cast<size_t>(K));
      for (size_t idx = 0; idx < k_nearests.size(); idx++)
      {
        k_nearests.at(idx) = all_nearests.at(idx);
      }
      return k_nearests;
    }
  };

  const size_t num_threads =
      static_cast<size_t>(openmp_helpers::GetNumOmpThreads());
  const size_t items_per_thread = static_cast<size_t>(std::ceil(
      static_cast<double>(items.size()) / static_cast<double>(num_threads)));

  if (K < 0)
  {
    throw std::invalid_argument("K must be >= 0");
  }
  else if (K == 0)
  {
    return std::vector<IndexAndDistance>();
  }
  else if (items.size() <= static_cast<size_t>(K))
  {
    std::vector<IndexAndDistance> k_nearests(items.size());
    CRU_OMP_PARALLEL_FOR
    for (size_t idx = 0; idx < items.size(); idx++)
    {
      const Item& item = items[idx];
      const double distance = distance_fn(item, current);
      k_nearests.at(idx).SetIndexAndDistance(idx, distance);
    }
    return k_nearests;
  }
  // For small K, use a parallel (|items| / num_threads) * K approach.
  else if (static_cast<double>(K) < std::log2(items_per_thread))
  {
    // Find K-nearest neighbors for each worker thread.
    std::vector<std::vector<IndexAndDistance>> per_thread_nearests(
        num_threads, std::vector<IndexAndDistance>(static_cast<size_t>(K)));
    CRU_OMP_PARALLEL_FOR
    for (size_t idx = 0; idx < items.size(); idx++)
    {
      const Item& item = items[idx];
      const double distance = distance_fn(item, current);
      const size_t thread_num =
          static_cast<size_t>(openmp_helpers::GetContextOmpThreadNum());
      std::vector<IndexAndDistance>& current_thread_nearests =
          per_thread_nearests.at(thread_num);
      auto itr = std::max_element(current_thread_nearests.begin(),
                                  current_thread_nearests.end(),
                                  IndexAndDistanceCompare);
      if (itr->Distance() > distance)
      {
        itr->SetIndexAndDistance(idx, distance);
      }
    }

    // Merge the per-thread K-nearest together.
    return merge_per_thread_nearests(per_thread_nearests);
  }
  // For larger K, use a parallel
  // (|items| / num_threads) * log(|items| / num_threads) approach.
  else
  {
    // Allocate per-worker-thread storage.
    std::vector<std::vector<IndexAndDistance>> per_thread_nearests(num_threads);
    for (auto& thread_nearest : per_thread_nearests)
    {
      thread_nearest.reserve(items_per_thread);
    }

    // Record index and distance for each element in its worker thread's own
    // storage.
    CRU_OMP_PARALLEL_FOR
    for (size_t idx = 0; idx < items.size(); idx++)
    {
      const Item& item = items[idx];
      const double distance = distance_fn(item, current);
      const size_t thread_num =
          static_cast<size_t>(openmp_helpers::GetContextOmpThreadNum());
      std::vector<IndexAndDistance>& current_thread_nearests =
          per_thread_nearests.at(thread_num);
      current_thread_nearests.emplace_back(idx, distance);
    }

    // Sort and extract the closest K elements for each worker thread.
    CRU_OMP_PARALLEL_FOR
    for (size_t idx = 0; idx < per_thread_nearests.size(); idx++)
    {
      std::vector<IndexAndDistance>& current_thread_nearests =
          per_thread_nearests.at(idx);
      std::sort(
          current_thread_nearests.begin(), current_thread_nearests.end(),
          IndexAndDistanceCompare);
      // Trim to the first K elements if larger than K.
      current_thread_nearests.resize(
          std::min(current_thread_nearests.size(), static_cast<size_t>(K)));
    }

    // Merge the per-thread K-nearest together.
    return merge_per_thread_nearests(per_thread_nearests);
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
  if (K < 0)
  {
    throw std::invalid_argument("K must be >= 0");
  }
  else if (K == 0)
  {
    return std::vector<IndexAndDistance>();
  }
  else if (items.size() <= static_cast<size_t>(K))
  {
    std::vector<IndexAndDistance> k_nearests(items.size());
    for (size_t idx = 0; idx < items.size(); idx++)
    {
      const Item& item = items[idx];
      const double distance = distance_fn(item, current);
      k_nearests.at(idx).SetIndexAndDistance(idx, distance);
    }
    return k_nearests;
  }
  // For small K, use a |items| * K approach.
  else if (static_cast<double>(K) < std::log2(items.size()))
  {
    std::vector<IndexAndDistance> k_nearests(static_cast<size_t>(K));
    for (size_t idx = 0; idx < items.size(); idx++)
    {
      const Item& item = items[idx];
      const double distance = distance_fn(item, current);
      auto itr = std::max_element(k_nearests.begin(), k_nearests.end(),
                                  IndexAndDistanceCompare);
      if (itr->Distance() > distance)
      {
        itr->SetIndexAndDistance(idx, distance);
      }
    }
    return k_nearests;
  }
  // For larger K, use a |items| * log(|items|) approach.
  else
  {
    std::vector<IndexAndDistance> all_distances(items.size());
    for (size_t idx = 0; idx < items.size(); idx++)
    {
      const Item& item = items[idx];
      const double distance = distance_fn(item, current);
      all_distances.at(idx) = IndexAndDistance(idx, distance);
    }

    std::sort(
        all_distances.begin(), all_distances.end(), IndexAndDistanceCompare);

    std::vector<IndexAndDistance> k_nearests(static_cast<size_t>(K));
    for (size_t idx = 0; idx < k_nearests.size(); idx++)
    {
      k_nearests.at(idx) = all_distances.at(idx);
    }
    return k_nearests;
  }
}

/// @return vector<pair<index, distance>> of the nearest @param K neighbors to
/// @param current in @param items, with distance computed by @param distance_fn
/// and @param use_parallel selects if search is performed in parallel.
template<typename Item, typename Value, typename Container=std::vector<Item>>
inline std::vector<IndexAndDistance> GetKNearestNeighbors(
    const Container& items, const Value& current,
    const std::function<double(const Item&, const Value&)>& distance_fn,
    const int64_t K, const bool use_parallel = false)
{
  if (use_parallel)
  {
    return GetKNearestNeighborsParallel(items, current, distance_fn, K);
  }
  else
  {
    return GetKNearestNeighborsSerial(items, current, distance_fn, K);
  }
}
}  // namespace simple_knearest_neighbors
}  // namespace common_robotics_utilities
