#include <cstdint>
#include <iostream>
#include <random>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <stdlib.h>

#include <common_robotics_utilities/utility.hpp>

#include <gtest/gtest.h>

namespace common_robotics_utilities
{
namespace
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

GTEST_TEST(UtilityTest, IsFutureReadyTest)
{
  const auto wait_op = [] (const double wait)
  {
    std::this_thread::sleep_for(std::chrono::duration<double>(wait));
  };
  std::future<void> wait_future =
      std::async(std::launch::async, wait_op, 1.0);
  EXPECT_FALSE(utility::IsFutureReady(wait_future));
  wait_future.wait();
  EXPECT_TRUE(utility::IsFutureReady(wait_future));
}

GTEST_TEST(UtilityTest, OnScopeExitTest)
{
  int test_val = 0;

  const auto on_exit_fn = [&test_val]() { test_val = 10; };

  EXPECT_EQ(test_val, 0);
  {
    const utility::OnScopeExit on_exit(on_exit_fn);
    EXPECT_TRUE(on_exit.IsEnabled());
  }
  EXPECT_EQ(test_val, 10);

  test_val = 0;

  EXPECT_EQ(test_val, 0);
  {
    utility::OnScopeExit on_exit(on_exit_fn);
    EXPECT_TRUE(on_exit.IsEnabled());
    on_exit.Disable();
    EXPECT_FALSE(on_exit.IsEnabled());
  }
  EXPECT_EQ(test_val, 0);
}

GTEST_TEST(UtilityTest, CopyableMoveableAtomicMethodsTest)
{
  // Thread safety is covered elsewhere, simply exercise the API here.

  utility::CopyableMoveableAtomic<bool> basic_bool(false);
  EXPECT_FALSE(basic_bool.load());
  basic_bool.store(true);
  EXPECT_TRUE(basic_bool.load());
  basic_bool.store(false);
  EXPECT_FALSE(basic_bool.load());

  utility::CopyableMoveableAtomic<int32_t> basic_int32(0);
  EXPECT_EQ(basic_int32.load(), 0);
  basic_int32.store(100);
  EXPECT_EQ(basic_int32.load(), 100);
  basic_int32.fetch_add(25);
  EXPECT_EQ(basic_int32.load(), 125);
  basic_int32.fetch_sub(50);
  EXPECT_EQ(basic_int32.load(), 75);

  utility::CopyableMoveableAtomic<int32_t> exchange_int32(0);
  EXPECT_EQ(exchange_int32.load(), 0);
  EXPECT_EQ(exchange_int32.exchange(100), 0);
  EXPECT_EQ(exchange_int32.load(), 100);

  utility::CopyableMoveableAtomic<int32_t> cxw_int32(0);
  EXPECT_EQ(cxw_int32.load(), 0);
  int32_t cxw_expected = 0;
  EXPECT_TRUE(cxw_int32.compare_exchange_weak(cxw_expected, 100));
  EXPECT_EQ(cxw_int32.load(), 100);
  EXPECT_EQ(cxw_expected, 0);
  EXPECT_FALSE(cxw_int32.compare_exchange_weak(cxw_expected, 200));
  EXPECT_EQ(cxw_int32.load(), 100);
  EXPECT_EQ(cxw_expected, 100);
  EXPECT_TRUE(cxw_int32.compare_exchange_weak(
      cxw_expected, 200, std::memory_order_relaxed, std::memory_order_relaxed));
  EXPECT_EQ(cxw_int32.load(), 200);
  EXPECT_EQ(cxw_expected, 100);

  utility::CopyableMoveableAtomic<int32_t> cxs_int32(0);
  EXPECT_EQ(cxs_int32.load(), 0);
  int32_t cxs_expected = 0;
  EXPECT_TRUE(cxs_int32.compare_exchange_strong(cxs_expected, 100));
  EXPECT_EQ(cxs_int32.load(), 100);
  EXPECT_EQ(cxs_expected, 0);
  EXPECT_FALSE(cxs_int32.compare_exchange_strong(cxs_expected, 200));
  EXPECT_EQ(cxs_int32.load(), 100);
  EXPECT_EQ(cxs_expected, 100);
  EXPECT_TRUE(cxs_int32.compare_exchange_weak(
      cxs_expected, 200, std::memory_order_relaxed, std::memory_order_relaxed));
  EXPECT_EQ(cxs_int32.load(), 200);
  EXPECT_EQ(cxs_expected, 100);
}

GTEST_TEST(UtilityTest, CopyableMoveableAtomicSelfAssignTest)
{
  using CMAi32 = utility::CopyableMoveableAtomic<int32_t>;

  CMAi32 basic_int32(100);
  EXPECT_EQ(basic_int32.load(), 100);

  auto& basic_copy_assign_ret = (basic_int32 = basic_int32);
  EXPECT_EQ(basic_int32.load(), 100);
  EXPECT_EQ(basic_copy_assign_ret.load(), 100);

  // GCC and Clang will warn if we directly write var = std::move(var), but we
  // can exercise the same operation using std::swap(var, var).
  std::swap(basic_int32, basic_int32);
  EXPECT_EQ(basic_int32.load(), 100);
}

GTEST_TEST(UtilityTest, CopyableMoveableAtomicCopyMoveAssignTest)
{
  using CMAi32 = utility::CopyableMoveableAtomic<int32_t>;

  // Initial value for copy operations.
  const CMAi32 const_int32(100);
  EXPECT_EQ(const_int32.load(), 100);

  // Copy construction works.
  const CMAi32 copied_int32(const_int32);
  EXPECT_EQ(copied_int32.load(), 100);

  // Copy assignment works.
  CMAi32 assigned_int32(0);
  EXPECT_EQ(assigned_int32.load(), 0);
  assigned_int32 = const_int32;
  EXPECT_EQ(assigned_int32.load(), 100);

  // Initial value for move operations.
  CMAi32 mutable_int32(200);
  EXPECT_EQ(mutable_int32.load(), 200);

  // Move construction works.
  CMAi32 move_constructed_int32(std::move(mutable_int32));
  EXPECT_EQ(move_constructed_int32.load(), 200);
  // Move constructor leaves moved-from unmodified.
  EXPECT_EQ(mutable_int32.load(), 200);

  // Move assignment works.
  CMAi32 moved_to_int32(0);
  EXPECT_EQ(moved_to_int32.load(), 0);
  moved_to_int32 = std::move(mutable_int32);
  EXPECT_EQ(moved_to_int32.load(), 200);
  // Move assignment leaves moved-from unmodified.
  EXPECT_EQ(mutable_int32.load(), 200);
}

GTEST_TEST(UtilityTest, CopyableMoveableAtomicCrossMemoryOrderTest)
{
  using CMAi32 = utility::CopyableMoveableAtomic<int32_t>;
  using CMAi32Relaxed =
      utility::CopyableMoveableAtomic<int32_t, std::memory_order_relaxed>;

  // Initial value for copy operations.
  const CMAi32 const_int32(100);
  EXPECT_EQ(const_int32.load(), 100);

  // Copy construction works.
  const CMAi32Relaxed copied_int32(const_int32);
  EXPECT_EQ(copied_int32.load(), 100);

  // Copy assignment works.
  CMAi32Relaxed assigned_int32(0);
  EXPECT_EQ(assigned_int32.load(), 0);
  assigned_int32 = const_int32;
  EXPECT_EQ(assigned_int32.load(), 100);

  // Initial value for move operations.
  CMAi32 mutable_int32(200);
  EXPECT_EQ(mutable_int32.load(), 200);

  // Move construction works.
  CMAi32Relaxed move_constructed_int32(std::move(mutable_int32));
  EXPECT_EQ(move_constructed_int32.load(), 200);
  // Move constructor leaves moved-from unmodified.
  EXPECT_EQ(mutable_int32.load(), 200);

  // Move assignment works.
  CMAi32Relaxed moved_to_int32(0);
  EXPECT_EQ(moved_to_int32.load(), 0);
  moved_to_int32 = std::move(mutable_int32);
  EXPECT_EQ(moved_to_int32.load(), 200);
  // Move assignment leaves moved-from unmodified.
  EXPECT_EQ(mutable_int32.load(), 200);
}
}  // namespace
}  // namespace common_robotics_utilities

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
