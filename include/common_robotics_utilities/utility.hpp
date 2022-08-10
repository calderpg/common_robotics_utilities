#pragma once

#include <cstdlib>
#include <functional>
#include <string>
#include <map>
#include <set>
#include <type_traits>
#include <utility>
#include <vector>
#include <iostream>

#include <Eigen/Geometry>

// Macro to disable unused parameter compiler warnings
#define CRU_UNUSED(x) (void)(x)

namespace common_robotics_utilities
{
namespace utility
{
// Functions to combine multiple std::hash<T>.
// Derived from: https://stackoverflow.com/questions/2590677/
//   how-do-i-combine-hash-values-in-c0x
inline void hash_combine(std::size_t& seed)
{
  CRU_UNUSED(seed);
}

template <typename T, typename... Rest>
inline void hash_combine(std::size_t& seed, const T& v, Rest... rest)
{
  std::hash<T> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
  hash_combine(seed, rest...);
}
}  // namespace utility
}  // namespace common_robotics_utilities

// Macro to construct std::hash<type> specializations for simple types that
// contain only types already specialized by std::hash<>.
// Example:
//
// struct SomeType
// {
//   std::string v1;
//   std::string v2;
//   bool v3;
// };
//
// CRU_MAKE_HASHABLE(SomeType, t.v1, t.v2, t.v3)

#define CRU_MAKE_HASHABLE(type, ...) \
    namespace std {\
        template<> struct hash<type> {\
            std::size_t operator()(const type& t) const {\
                std::size_t ret = 0;\
                common_robotics_utilities::utility::\
                    hash_combine(ret, __VA_ARGS__);\
                return ret;\
            }\
        };\
    }

namespace common_robotics_utilities
{
namespace utility
{
/// Signature of a basic logging function.
using LoggingFunction = std::function<void(const std::string&)>;

/// Make a do-nothing logging function.
inline LoggingFunction NoOpLoggingFunction()
{
  return [] (const std::string&) {};
}

/// Signature for function that returns a double uniformly sampled from
/// the interval [0.0, 1.0).
using UniformUnitRealFunction = std::function<double(void)>;

/// Given a UniformUnitRealFunction @param uniform_unit_real_fn, returns an
/// index in [0, container_size - 1].
template<typename SizeType>
SizeType GetUniformRandomIndex(
    const UniformUnitRealFunction& uniform_unit_real_fn,
    const SizeType container_size)
{
  static_assert(
      std::is_integral<SizeType>::value, "SizeType must be an integral type");
  if (container_size < 1)
  {
    throw std::invalid_argument("container_size must be >= 1");
  }
  return static_cast<SizeType>(std::floor(
      uniform_unit_real_fn() * static_cast<double>(container_size)));
}

/// Given a UniformUnitRealFunction @param uniform_unit_real_fn, returns a
/// value in [start, end].
template<typename SizeType>
SizeType GetUniformRandomInRange(
    const UniformUnitRealFunction& uniform_unit_real_fn,
    const SizeType start, const SizeType end)
{
  if (start > end)
  {
    throw std::invalid_argument("start must be <= end");
  }
  const SizeType range = end - start;
  const SizeType offset =
      GetUniformRandomIndex<SizeType>(uniform_unit_real_fn, range + 1);
  return start + offset;
}

template <class T>
inline T ClampValueAndLog(const T& val, const T& min, const T& max,
                          const LoggingFunction& logging_fn = {})
{
  if (max >= min)
  {
    if (val < min)
    {
      if (logging_fn)
      {
        const std::string msg = "Clamping " + std::to_string(val)
                                + " to min " + std::to_string(min);
        logging_fn(msg);
      }
      return min;
    }
    else if (val > max)
    {
      if (logging_fn)
      {
        const std::string msg = "Clamping " + std::to_string(val)
                                + " to max " + std::to_string(max);
        logging_fn(msg);
      }
      return max;
    }
    return val;
  }
  else
  {
    throw std::invalid_argument("min > max");
  }
}

template <typename T>
inline T ClampValue(const T& val, const T& min, const T& max)
{
  return ClampValueAndLog<T>(val, min, max, {});
}

// Written to mimic parts of Matlab wthresh(val, 'h', thresh) behavior,
// spreading the value to the thresholds instead of setting them to zero
// https://www.mathworks.com/help/wavelet/ref/wthresh.html
template <class T>
inline T SpreadValueAndLog(const T& val, const T& low_threshold,
                           const T& midpoint, const T& high_threshold,
                           const LoggingFunction& logging_fn = {})
{
  if ((low_threshold <= midpoint) && (midpoint <= high_threshold))
  {
    if (val >= midpoint && val < high_threshold)
    {
      if (logging_fn)
      {
        const std::string msg = "Thresholding " + std::to_string(val)
                                + " to high threshold "
                                + std::to_string(high_threshold);
        logging_fn(msg);
      }
      return high_threshold;
    }
    else if (val < midpoint && val > low_threshold)
    {
      if (logging_fn)
      {
        const std::string msg = "Thresholding " + std::to_string(val)
                                + " to low threshold "
                                + std::to_string(low_threshold);
        logging_fn(msg);
      }
      return low_threshold;
    }
    return val;
  }
  else
  {
    throw std::invalid_argument("Invalid thresholds/midpoint");
  }
}

template <class T>
inline T SpreadValue(const T& val, const T& low_threshold, const T& midpoint,
                     const T& high_threshold)
{
  return SpreadValueAndLog<T>(
      val, low_threshold, midpoint, high_threshold, {});
}

template<typename T>
inline bool CheckAlignment(const T& item,
                           const uint64_t desired_alignment,
                           const LoggingFunction& logging_fn = {})
{
  const T* item_ptr = std::addressof(item);
  const uintptr_t item_ptr_val = reinterpret_cast<uintptr_t>(item_ptr);
  if ((item_ptr_val % desired_alignment) == 0)
  {
    if (logging_fn)
    {
      const std::string msg = "Item @ " + std::to_string(item_ptr_val)
                              + " aligned to "
                              + std::to_string(desired_alignment) + " bytes";
      logging_fn(msg);
    }
    return true;
  }
  else
  {
    if (logging_fn)
    {
      const std::string msg = "Item @ " + std::to_string(item_ptr_val)
                              + " NOT aligned to "
                              + std::to_string(desired_alignment) + " bytes";
      logging_fn(msg);
    }
    return false;
  }
}

template<typename T>
inline void RequireAlignment(const T& item, const uint64_t desired_alignment)
{
  const T* item_ptr = std::addressof(item);
  const uintptr_t item_ptr_val = reinterpret_cast<uintptr_t>(item_ptr);
  if ((item_ptr_val % desired_alignment) != 0)
  {
    throw std::runtime_error("Item @ " + std::to_string(item_ptr_val)
                             + " not aligned at desired alignment of "
                             + std::to_string(desired_alignment) + " bytes");
  }
}

template<typename T>
inline void RequireEigenAlignment(const T& item)
{
  const uint64_t eigen_alignment =
      static_cast<uint64_t>(EIGEN_DEFAULT_ALIGN_BYTES);
  RequireAlignment<T>(item, eigen_alignment);
}

template <typename T>
inline T SetBit(const T current,
                const uint32_t bit_position,
                const bool bit_value)
{
  // Safety check on the type we've been called with
  static_assert((std::is_same<T, uint8_t>::value
           || std::is_same<T, uint16_t>::value
           || std::is_same<T, uint32_t>::value
           || std::is_same<T, uint64_t>::value),
          "Type must be a fixed-size unsigned integral type");
  // Do it
  T update_mask = 1;
  update_mask = static_cast<T>(update_mask << bit_position);
  if (bit_value)
  {
    return (current | update_mask);
  }
  else
  {
    update_mask = static_cast<T>(~update_mask);
    return (current & update_mask);
  }
}

template <typename T>
inline bool GetBit(const T current, const uint32_t bit_position)
{
  // Type safety checks are performed in the SetBit() function
  const T mask = SetBit(static_cast<T>(0), bit_position, true);
  if ((mask & current) > 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

template<typename Container=std::vector<std::string>>
inline bool CheckAllStringsForSubstring(
    const Container& strings, const std::string& substring)
{
  for (const std::string& candidate_string : strings)
  {
    const size_t found = candidate_string.find(substring);
    if (found == std::string::npos)
    {
      return false;
    }
  }
  return true;
}

template <typename T, typename Container1=std::vector<T>,
          typename Container2=std::vector<T>>
inline bool IsSubset(const Container1& set, const Container2& candidate_subset)
{
  std::map<T, size_t> set_members;
  for (const T& item : set)
  {
    set_members[item] += 1;
  }

  std::map<T, size_t> candidate_subset_members;
  for (const T& item : candidate_subset)
  {
    candidate_subset_members[item] += 1;
  }

  if (candidate_subset_members.size() > set_members.size())
  {
    return false;
  }

  for (const auto& item_and_count : candidate_subset_members)
  {
    const auto found_itr = set_members.find(item_and_count.first);
    if (found_itr != set_members.end())
    {
      const size_t set_count = found_itr->second;
      const size_t candidate_subset_count = item_and_count.second;
      if (candidate_subset_count > set_count)
      {
        return false;
      }
    }
    else
    {
      return false;
    }
  }
  return true;
}

template <typename T, typename Container1=std::vector<T>,
          typename Container2=std::vector<T>>
inline bool CollectionsEqual(const Container1& set1, const Container2& set2)
{
  if (set1.size() != set2.size())
  {
    return false;
  }

  std::map<T, size_t> set1_members;
  for (const T& item : set1)
  {
    set1_members[item] += 1;
  }

  std::map<T, size_t> set2_members;
  for (const T& item : set2)
  {
    set2_members[item] += 1;
  }

  if (set1_members.size() != set2_members.size())
  {
    return false;
  }

  for (const auto& item_and_count : set1_members)
  {
    const auto found_itr = set2_members.find(item_and_count.first);
    if (found_itr != set2_members.end())
    {
      const size_t set2_count = found_itr->second;
      const size_t set1_count = item_and_count.second;
      if (set1_count != set2_count)
      {
        return false;
      }
    }
    else
    {
      return false;
    }
  }
  return true;
}

template <typename Key, typename Value, typename MapLike=std::map<Key, Value>>
inline Value RetrieveOrDefault(
    const MapLike& map, const Key& key, const Value& default_val)
{
  const auto found_itr = map.find(key);
  if (found_itr != map.end())
  {
    return found_itr->second;
  }
  else
  {
    return default_val;
  }
}

template <typename Key, typename Value, typename MapLike=std::map<Key, Value>,
          typename KeyContainer=std::vector<Key>>
inline KeyContainer GetKeysFromMapLike(const MapLike& map)
{
  KeyContainer keys;
  keys.reserve(map.size());
  typename MapLike::const_iterator itr;
  for (const auto& key_and_value : map)
  {
    keys.push_back(key_and_value.first);
  }
  keys.shrink_to_fit();
  return keys;
}

template <typename Key, typename SetLike=std::set<Key>,
          typename KeyContainer=std::vector<Key>>
inline KeyContainer GetKeysFromSetLike(const SetLike& set)
{
  KeyContainer keys;
  keys.reserve(set.size());
  for (const Key& cur_key : set)
  {
    keys.push_back(cur_key);
  }
  keys.shrink_to_fit();
  return keys;
}

template <typename Key, typename Value, typename MapLike=std::map<Key, Value>,
          typename KeyValuePairContainer=std::vector<std::pair<Key, Value>>>
inline KeyValuePairContainer GetKeysAndValues(const MapLike& map)
{
  KeyValuePairContainer keys_and_values;
  keys_and_values.reserve(map.size());
  for (const auto& key_and_value : map)
  {
    keys_and_values.push_back(key_and_value);
  }
  keys_and_values.shrink_to_fit();
  return keys_and_values;
}

template <typename Key, typename Value, typename MapLike=std::map<Key, Value>,
          typename KeyValuePairContainer=std::vector<std::pair<Key, Value>>>
inline MapLike MakeFromKeysAndValues(
    const KeyValuePairContainer& keys_and_values)
{
  MapLike map;
  for (const std::pair<Key, Value>& cur_pair : keys_and_values)
  {
    map[cur_pair.first] = cur_pair.second;
  }
  return map;
}

template <typename Key, typename Value, typename MapLike=std::map<Key, Value>,
          typename KeyContainer=std::vector<Key>,
          typename ValueContainer=std::vector<Value>>
inline MapLike MakeFromKeysAndValues(
    const KeyContainer& keys, const ValueContainer& values)
{
  if (keys.size() == values.size())
  {
    MapLike map;
    for (size_t idx = 0; idx < keys.size(); idx++)
    {
      const Key& cur_key = keys[idx];
      const Value& cur_value = values[idx];
      map[cur_key] = cur_value;
    }
    return map;
  }
  else
  {
    throw std::invalid_argument("keys.size() != values.size()");
  }
}
}  // namespace utility
}  // namespace common_robotics_utilities
