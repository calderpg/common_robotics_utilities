#pragma once

#include <omp.h>

#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

#include <common_robotics_utilities/openmp_helpers.hpp>

namespace common_robotics_utilities
{
namespace simple_knearest_neighbors
{
/// @return vector<pair<index, distance>> of the nearest @param K neighbors to
/// @param current in @param items, with distance computed by @param distance_fn
/// and search performed in parallel.
template<typename Item, typename Value, typename Container=std::vector<Item>>
inline std::vector<std::pair<int64_t, double>> GetKNearestNeighborsParallel(
    const Container& items, const Value& current,
    const std::function<double(const Item&, const Value&)>& distance_fn,
    const size_t K)
{
  if (K == 0)
  {
    return std::vector<std::pair<int64_t, double>>();
  }
  if (items.size() > K)
  {
    std::function<bool(const std::pair<int64_t, double>&,
                       const std::pair<int64_t, double>&)> compare_fn =
        [] (const std::pair<int64_t, double>& index1,
            const std::pair<int64_t, double>& index2)
    {
      return index1.second < index2.second;
    };
    std::vector<std::vector<std::pair<int64_t, double>>> per_thread_nearests(
        openmp_helpers::GetNumOmpThreads(),
        std::vector<std::pair<int64_t, double>>(
            K, std::make_pair(-1, std::numeric_limits<double>::infinity())));
#if defined(_OPENMP)
#pragma omp parallel for
#endif
    for (size_t idx = 0; idx < items.size(); idx++)
    {
      const Item& item = items[idx];
      const double distance = distance_fn(item, current);
      const auto thread_num = openmp_helpers::GetContextOmpThreadNum();
      std::vector<std::pair<int64_t, double>>& current_thread_nearests =
          per_thread_nearests.at(thread_num);
      auto itr = std::max_element(current_thread_nearests.begin(),
                                  current_thread_nearests.end(), compare_fn);
      const double worst_distance = itr->second;
      if (worst_distance > distance)
      {
        itr->first = (int64_t)idx;
        itr->second = distance;
      }
    }
    std::vector<std::pair<int64_t, double>> k_nearests;
    k_nearests.reserve(K);
    for (size_t thread_idx = 0; thread_idx < per_thread_nearests.size();
         thread_idx++)
    {
      const std::vector<std::pair<int64_t, double>>& thread_nearests =
          per_thread_nearests.at(thread_idx);
      for (size_t nearest_idx = 0; nearest_idx < thread_nearests.size();
           nearest_idx++)
      {
        const std::pair<int64_t, double> current_ith_nearest =
            thread_nearests.at(nearest_idx);
        if (!std::isinf(current_ith_nearest.second)
            && current_ith_nearest.first != -1)
        {
          if (k_nearests.size() < K)
          {
            k_nearests.push_back(current_ith_nearest);
          }
          else
          {
            auto itr = std::max_element(k_nearests.begin(), k_nearests.end(),
                                        compare_fn);
            const double worst_distance = itr->second;
            if (worst_distance > current_ith_nearest.second)
            {
              itr->first = current_ith_nearest.first;
              itr->second = current_ith_nearest.second;
            }
          }
        }
      }
    }
    k_nearests.shrink_to_fit();
    return k_nearests;
  }
  else
  {
    std::vector<std::pair<int64_t, double>> k_nearests(
        items.size(),
        std::make_pair(-1, std::numeric_limits<double>::infinity()));
#if defined(_OPENMP)
#pragma omp parallel for
#endif
    for (size_t idx = 0; idx < items.size(); idx++)
    {
      const Item& item = items[idx];
      const double distance = distance_fn(item, current);
      k_nearests.at(idx) = std::make_pair((int64_t)idx, distance);
    }
    return k_nearests;
  }
}

/// @return vector<pair<index, distance>> of the nearest @param K neighbors to
/// @param current in @param items, with distance computed by @param
/// distance_fn.
template<typename Item, typename Value, typename Container=std::vector<Item>>
inline std::vector<std::pair<int64_t, double>> GetKNearestNeighborsSerial(
    const Container& items, const Value& current,
    const std::function<double(const Item&, const Value&)>& distance_fn,
    const size_t K)
{
  if (K == 0)
  {
    return std::vector<std::pair<int64_t, double>>();
  }
  if (items.size() > K)
  {
    std::function<bool(const std::pair<int64_t, double>&,
                       const std::pair<int64_t, double>&)> compare_fn =
        [] (const std::pair<int64_t, double>& index1,
            const std::pair<int64_t, double>& index2)
    {
        return index1.second < index2.second;
    };
    std::vector<std::pair<int64_t, double>> k_nearests(
        K, std::make_pair(-1, std::numeric_limits<double>::infinity()));
    for (size_t idx = 0; idx < items.size(); idx++)
    {
      const Item& item = items[idx];
      const double distance = distance_fn(item, current);
      auto itr =
          std::max_element(k_nearests.begin(), k_nearests.end(), compare_fn);
      const double worst_distance = itr->second;
      if (worst_distance > distance)
      {
        itr->first = (int64_t)idx;
        itr->second = distance;
      }
    }
    return k_nearests;
  }
  else
  {
    std::vector<std::pair<int64_t, double>> k_nearests(
        items.size(),
        std::make_pair(-1, std::numeric_limits<double>::infinity()));
    for (size_t idx = 0; idx < items.size(); idx++)
    {
      const Item& item = items[idx];
      const double distance = distance_fn(item, current);
      k_nearests.at(idx) = std::make_pair((int64_t)idx, distance);
    }
    return k_nearests;
  }
}

/// @return vector<pair<index, distance>> of the nearest @param K neighbors to
/// @param current in @param items, with distance computed by @param distance_fn
/// and @param use_parallel selects if search is performed in parallel.
template<typename Item, typename Value, typename Container=std::vector<Item>>
inline std::vector<std::pair<int64_t, double>> GetKNearestNeighbors(
    const Container& items, const Value& current,
    const std::function<double(const Item&, const Value&)>& distance_fn,
    const size_t K, const bool use_parallel = false)
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
