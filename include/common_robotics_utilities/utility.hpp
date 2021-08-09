#pragma once

#include <cstdlib>
#include <string>
#include <map>
#include <set>
#include <utility>
#include <vector>
#include <iostream>

#include <Eigen/Geometry>

// Branch prediction hints
// Figure out which compiler we have
#if defined(__clang__)
  /* Clang/LLVM */
  #define CRU_LIKELY(x) __builtin_expect(!!(x), 1)
  #define CRU_UNLIKELY(x) __builtin_expect(!!(x), 0)
#elif defined(__ICC) || defined(__INTEL_COMPILER)
  /* Intel ICC/ICPC */
  #define CRU_LIKELY(x) __builtin_expect(!!(x), 1)
  #define CRU_UNLIKELY(x) __builtin_expect(!!(x), 0)
#elif defined(__GNUC__) || defined(__GNUG__)
  /* GNU GCC/G++ */
  #define CRU_LIKELY(x) __builtin_expect(!!(x), 1)
  #define CRU_UNLIKELY(x) __builtin_expect(!!(x), 0)
#elif defined(_MSC_VER)
  /* Microsoft Visual Studio */
  /* MSVC doesn't support branch prediction hints. Use PGO instead. */
  #define CRU_LIKELY(x) (x)
  #define CRU_UNLIKELY(x) (x)
#endif

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
template <typename T>
inline T ClampValue(const T& val, const T& min, const T& max)
{
  if (max >= min)
  {
    return std::min(max, std::max(min, val));
  }
  else
  {
    throw std::invalid_argument("min > max");
  }
}

template <class T>
inline T ClampValueAndWarn(const T& val, const T& min, const T& max)
{
  if (max >= min)
  {
    if (val < min)
    {
      const std::string msg = "Clamping " + std::to_string(val)
                              + " to min " + std::to_string(min) + "\n";
      std::cerr << msg << std::flush;
      return min;
    }
    else if (val > max)
    {
      const std::string msg = "Clamping " + std::to_string(val)
                              + " to max " + std::to_string(max) + "\n";
      std::cerr << msg << std::flush;
      return max;
    }
    return val;
  }
  else
  {
    throw std::invalid_argument("min > max");
  }
}

// Written to mimic parts of Matlab wthresh(val, 'h', thresh) behavior,
// spreading the value to the thresholds instead of setting them to zero
// https://www.mathworks.com/help/wavelet/ref/wthresh.html
template <class T>
inline T SpreadValue(const T& val, const T& low_threshold, const T& midpoint,
                     const T& high_threshold)
{
  if ((low_threshold <= midpoint) && (midpoint <= high_threshold))
  {
    if (val >= midpoint && val < high_threshold)
    {
      return high_threshold;
    }
    else if (val < midpoint && val > low_threshold)
    {
      return low_threshold;
    }
    return val;
  }
  else
  {
    throw std::invalid_argument("Invalid thresholds/midpoint");
  }
}

// Written to mimic parts of Matlab wthresh(val, 'h', thresh) behavior,
// spreading the value to the thresholds instead of setting them to zero
// https://www.mathworks.com/help/wavelet/ref/wthresh.html
template <class T>
inline T SpreadValueAndWarn(const T& val, const T& low_threshold,
                            const T& midpoint, const T& high_threshold)
{
  if ((low_threshold <= midpoint) && (midpoint <= high_threshold))
  {
    if (val >= midpoint && val < high_threshold)
    {
      const std::string msg = "Thresholding " + std::to_string(val)
                              + " to high threshold "
                              + std::to_string(high_threshold) + "\n";
      std::cerr << msg << std::flush;
      return high_threshold;
    }
    else if (val < midpoint && val > low_threshold)
    {
      const std::string msg = "Thresholding " + std::to_string(val)
                              + " to low threshold "
                              + std::to_string(low_threshold) + "\n";
      std::cerr << msg << std::flush;
      return low_threshold;
    }
    return val;
  }
  else
  {
    throw std::invalid_argument("Invalid thresholds/midpoint");
  }
}

template<typename T>
inline bool CheckAlignment(const T& item,
                           const uint64_t desired_alignment,
                           bool verbose=false)
{
  const T* item_ptr = std::addressof(item);
  const uintptr_t item_ptr_val = reinterpret_cast<uintptr_t>(item_ptr);
  if ((item_ptr_val % desired_alignment) == 0)
  {
    if (verbose)
    {
      const std::string msg = "Item @ " + std::to_string(item_ptr_val)
                              + " aligned to "
                              + std::to_string(desired_alignment) + " bytes\n";
      std::cout << msg << std::flush;
    }
    return true;
  }
  else
  {
    if (verbose)
    {
      const std::string msg = "Item @ " + std::to_string(item_ptr_val)
                              + " NOT aligned to "
                              + std::to_string(desired_alignment) + " bytes\n";
      std::cout << msg << std::flush;
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

inline void ConditionalPrint(const std::string& msg,
                             const int32_t msg_level,
                             const int32_t print_level)
{
  if (CRU_UNLIKELY(msg_level <= print_level))
  {
    const std::string printstr = "[" + std::to_string(msg_level) + "/"
                                 + std::to_string(print_level) + "] "
                                 + msg + "\n";
    std::cout << printstr << std::flush;
  }
}

inline void ConditionalError(const std::string& msg,
                             const int32_t msg_level,
                             const int32_t print_level)
{
  if (CRU_UNLIKELY(msg_level <= print_level))
  {
    const std::string printstr = "[" + std::to_string(msg_level) + "/"
                                 + std::to_string(print_level) + "] "
                                 + msg + "\n";
    std::cerr << printstr << std::flush;
  }
}

// Compile-time detection for if operator == is defined for a type.
typedef char no;
typedef no (&yes)[2];

no DetectOperatorEquals(...);

template<typename T>
auto DetectOperatorEquals(const T& item) -> typename std::conditional<
    false,
    decltype(item.operator==(std::declval<typename T::value_type>())),
    yes>::type;

template<typename T>
struct HasOperatorEquals : std::integral_constant<
    bool,
    sizeof(DetectOperatorEquals(std::declval<const T&>())) == sizeof(yes)> {};
}  // namespace utility
}  // namespace common_robotics_utilities
