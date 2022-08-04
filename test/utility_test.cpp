#include <cstdint>
#include <iostream>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <stdlib.h>

#include <common_robotics_utilities/utility.hpp>

#include <gtest/gtest.h>

namespace common_robotics_utilities
{
namespace utility_test
{
GTEST_TEST(UtilityTest, GetUniformRandom)
{
  std::mt19937_64 rng(42);
  std::uniform_real_distribution<double> uniform_unit_real_dist(0.0, 1.0);

  const utility::UniformUnitRealFunction uniform_unit_real_fn = [&] ()
  {
    return uniform_unit_real_dist(rng);
  };

  constexpr int64_t num_samples = 1000000;

  for (int64_t sample_num = 0; sample_num < num_samples; sample_num++)
  {
    for (int64_t container_size : {10, 100, 1000, 1000000, 1000000000})
    {
      const int64_t random_index = utility::GetUniformRandomIndex(
          uniform_unit_real_fn, container_size);
      ASSERT_GE(random_index, 0);
      ASSERT_LT(random_index, container_size);

      const int64_t range_start = container_size;
      const int64_t range_end = container_size * 2;

      const int64_t random_val = utility::GetUniformRandomInRange(
          uniform_unit_real_fn, range_start, range_end);
      ASSERT_GE(random_val, range_start);
      ASSERT_LE(random_val, range_end);
    }
  }

  ASSERT_THROW(
      utility::GetUniformRandomIndex(uniform_unit_real_fn, 0),
      std::invalid_argument);
  ASSERT_THROW(
      utility::GetUniformRandomInRange(uniform_unit_real_fn, 10, 9),
      std::invalid_argument);
}

GTEST_TEST(UtilityTest, ClampAndSpread)
{
  const utility::LoggingFunction logging_fn = [] (const std::string& msg)
  {
    std::cout << msg << std::endl;
  };
  ASSERT_EQ(utility::ClampValue(1.0, -0.5, 0.5), 0.5);
  ASSERT_EQ(utility::ClampValueAndLog(1.0, -0.5, 0.5, logging_fn), 0.5);

  ASSERT_EQ(utility::ClampValue(-1.0, -0.5, 0.5), -0.5);
  ASSERT_EQ(utility::ClampValueAndLog(-1.0, -0.5, 0.5, logging_fn), -0.5);

  ASSERT_EQ(utility::ClampValue(0.25, -0.5, 0.5), 0.25);
  ASSERT_EQ(utility::ClampValueAndLog(0.25, -0.5, 0.5, logging_fn), 0.25);

  ASSERT_THROW(utility::ClampValue(1.0, 1.0, -1.0), std::invalid_argument);
  ASSERT_THROW(
      utility::ClampValueAndLog(1.0, 1.0, -1.0, logging_fn),
      std::invalid_argument);

  ASSERT_EQ(utility::SpreadValue(1.0, -0.5, 0.0, 0.5), 1.0);
  ASSERT_EQ(utility::SpreadValueAndLog(1.0, -0.5, 0.0, 0.5, logging_fn), 1.0);

  ASSERT_EQ(utility::SpreadValue(-1.0, -0.5, 0.0, 0.5), -1.0);
  ASSERT_EQ(
      utility::SpreadValueAndLog(-1.0, -0.5, 0.0, 0.5, logging_fn), -1.0);

  ASSERT_EQ(utility::SpreadValue(0.25, -0.5, 0.0, 0.5), 0.5);
  ASSERT_EQ(utility::SpreadValueAndLog(0.25, -0.5, 0.0, 0.5, logging_fn), 0.5);

  ASSERT_EQ(utility::SpreadValue(-0.25, -0.5, 0.0, 0.5), -0.5);
  ASSERT_EQ(
      utility::SpreadValueAndLog(-0.25, -0.5, 0.0, 0.5, logging_fn), -0.5);

  ASSERT_THROW(
      utility::SpreadValue(1.0, 1.0, 0.0, -1.0), std::invalid_argument);
  ASSERT_THROW(
      utility::SpreadValueAndLog(1.0, 1.0, 0.0, -1.0, logging_fn),
      std::invalid_argument);
}

GTEST_TEST(UtilityTest, Alignment)
{
  uint8_t* const aligned_8byte_ptr =
      reinterpret_cast<uint8_t*>(aligned_alloc(8, 256));
  uint8_t* const aligned_16byte_ptr =
      reinterpret_cast<uint8_t*>(aligned_alloc(16, 256));
  ASSERT_NE(aligned_8byte_ptr, nullptr);
  ASSERT_NE(aligned_16byte_ptr, nullptr);

  uint8_t* const off_aligned_8byte_ptr = aligned_8byte_ptr + 1;
  uint8_t* const off_aligned_16byte_ptr = aligned_16byte_ptr + 1;

  ASSERT_TRUE(utility::CheckAlignment(*aligned_8byte_ptr, 8));
  ASSERT_TRUE(utility::CheckAlignment(*aligned_16byte_ptr, 16));

  ASSERT_FALSE(utility::CheckAlignment(*off_aligned_8byte_ptr, 8));
  ASSERT_FALSE(utility::CheckAlignment(*off_aligned_16byte_ptr, 16));

  ASSERT_NO_THROW(utility::RequireAlignment(*aligned_8byte_ptr, 8));
  ASSERT_NO_THROW(utility::RequireAlignment(*aligned_16byte_ptr, 16));

  ASSERT_THROW(
      utility::RequireAlignment(*off_aligned_8byte_ptr, 8), std::runtime_error);
  ASSERT_THROW(
      utility::RequireAlignment(*off_aligned_16byte_ptr, 16),
      std::runtime_error);

  free(aligned_8byte_ptr);
  free(aligned_16byte_ptr);
}

GTEST_TEST(UtilityTest, GetAndSetBits)
{
  const uint8_t test_val = 0xaa;
  ASSERT_EQ(utility::SetBit(test_val, 0, true), 0xab);
  ASSERT_EQ(utility::SetBit(test_val, 3, false), 0xa2);

  ASSERT_TRUE(utility::GetBit(test_val, 3));
  ASSERT_FALSE(utility::GetBit(test_val, 4));
}

GTEST_TEST(UtilityTest, Substrings)
{
  const std::vector<std::string> strings = {"ex_foo", "ex_bar", "ex_baz"};
  ASSERT_TRUE(utility::CheckAllStringsForSubstring(strings, "ex_"));
  ASSERT_FALSE(utility::CheckAllStringsForSubstring(strings, "ex_b"));
}

GTEST_TEST(UtilityTest, CollectionsAndSubset)
{
  const std::vector<int32_t> collection_1 = {1, 2, 3, 4, 5};
  const std::vector<int32_t> collection_2 = {1, 1, 2, 2, 3, 3, 4, 4, 5, 5};
  const std::vector<int32_t> collection_3 = {3, 5, 1, 2, 4};

  ASSERT_FALSE(utility::IsSubset<int32_t>(collection_1, collection_2));
  ASSERT_TRUE(utility::IsSubset<int32_t>(collection_2, collection_1));
  ASSERT_TRUE(utility::CollectionsEqual<int32_t>(collection_1, collection_3));
  ASSERT_FALSE(utility::CollectionsEqual<int32_t>(collection_1, collection_2));
}

GTEST_TEST(UtilityTest, GetFromMapAndSet)
{
  const std::vector<int32_t> test_keys = {1, 2, 3, 4, 5};
  std::vector<std::pair<int32_t, int32_t>> test_keys_and_values;
  std::set<int32_t> test_set;
  std::unordered_set<int32_t> test_unordered_set;
  std::map<int32_t, int32_t> test_map;
  std::unordered_map<int32_t, int32_t> test_unordered_map;

  for (const int32_t& test_key : test_keys)
  {
    test_keys_and_values.push_back(std::make_pair(test_key, test_key));
    test_set.insert(test_key);
    test_unordered_set.insert(test_key);
    test_map[test_key] = test_key;
    test_unordered_map[test_key] = test_key;
  }

  ASSERT_TRUE(utility::CollectionsEqual<int32_t>(
      test_keys, utility::GetKeysFromSetLike<int32_t>(test_set)));
  ASSERT_TRUE(utility::CollectionsEqual<int32_t>(
      test_keys, utility::GetKeysFromSetLike<int32_t>(test_unordered_set)));

  ASSERT_TRUE(utility::CollectionsEqual<int32_t>(
      test_keys, utility::GetKeysFromMapLike<int32_t, int32_t>(test_map)));
  ASSERT_TRUE(utility::CollectionsEqual<int32_t>(
      test_keys,
      utility::GetKeysFromMapLike<int32_t, int32_t>(test_unordered_map)));

  const auto test_map_keys_and_values =
      utility::GetKeysAndValues<int32_t, int32_t>(test_map);
  const auto test_unordered_map_keys_and_values =
      utility::GetKeysAndValues<int32_t, int32_t>(test_unordered_map);
  const bool test_map_match =
      utility::CollectionsEqual<std::pair<int32_t, int32_t>>(
          test_keys_and_values, test_map_keys_and_values);
  const bool test_unordered_map_match =
      utility::CollectionsEqual<std::pair<int32_t, int32_t>>(
          test_keys_and_values, test_unordered_map_keys_and_values);

  ASSERT_TRUE(test_map_match);
  ASSERT_TRUE(test_unordered_map_match);
}

GTEST_TEST(UtilityTest, MakeMapAndSet)
{
  const std::vector<int32_t> test_keys = {1, 2, 3, 4, 5};
  std::vector<std::pair<int32_t, int32_t>> test_keys_and_values;
  for (const int32_t& test_key : test_keys)
  {
    test_keys_and_values.push_back(std::make_pair(test_key, test_key));
  }

  const auto test_map1 =
      utility::MakeFromKeysAndValues<int32_t, int32_t>(test_keys, test_keys);
  const auto test_map2 =
      utility::MakeFromKeysAndValues<int32_t, int32_t>(test_keys_and_values);

  const bool test_map1_match =
      utility::CollectionsEqual<std::pair<int32_t, int32_t>>(
          test_keys_and_values,
          utility::GetKeysAndValues<int32_t, int32_t>(test_map1));
  const bool test_map2_match =
      utility::CollectionsEqual<std::pair<int32_t, int32_t>>(
          test_keys_and_values,
          utility::GetKeysAndValues<int32_t, int32_t>(test_map2));

  ASSERT_TRUE(test_map1_match);
  ASSERT_TRUE(test_map2_match);
}
}  // namespace utility_test
}  // namespace common_robotics_utilities

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
