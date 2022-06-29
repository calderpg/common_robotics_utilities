#pragma once

#if defined(_OPENMP)
#include <omp.h>
#endif

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
#if defined(_OPENMP)
#pragma omp parallel for
#endif
    for (size_t idx = 0; idx < items.size(); idx++)
    {
      const Item& item = items[idx];
      const double distance = distance_fn(item, current);
      k_nearests.at(idx).SetIndexAndDistance(idx, distance);
    }
    return k_nearests;
  }
  else
  {
    std::vector<std::vector<IndexAndDistance>> per_thread_nearests(
        static_cast<size_t>(openmp_helpers::GetNumOmpThreads()),
        std::vector<IndexAndDistance>(static_cast<size_t>(K)));
#if defined(_OPENMP)
#pragma omp parallel for
#endif
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

    std::vector<IndexAndDistance> k_nearests(static_cast<size_t>(K));
    for (const auto& thread_nearests : per_thread_nearests)
    {
      for (const auto& current_ith_nearest : thread_nearests)
      {
        if (!std::isinf(current_ith_nearest.Distance())
            && current_ith_nearest.Index() != -1)
        {
          auto itr = std::max_element(k_nearests.begin(), k_nearests.end(),
                                      IndexAndDistanceCompare);
          if (itr->Distance() > current_ith_nearest.Distance())
          {
            itr->SetFromOther(current_ith_nearest);
          }
        }
      }
    }
    return k_nearests;
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
  else
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
