#include <cstdint>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/maybe.hpp>
#include <common_robotics_utilities/utility.hpp>

#include <gtest/gtest.h>

namespace common_robotics_utilities
{
namespace maybe_test
{
constexpr int32_t kDefaultValue = 0;

class DefaultConstructibleWrapper
{
private:
  int32_t value_ = kDefaultValue;

public:
  DefaultConstructibleWrapper() = default;

  DefaultConstructibleWrapper(const int32_t value) : value_(value) {}

  DefaultConstructibleWrapper(
      const DefaultConstructibleWrapper& other) = default;

  DefaultConstructibleWrapper(DefaultConstructibleWrapper&& other) = default;

  DefaultConstructibleWrapper& operator=(
      const DefaultConstructibleWrapper& other) = default;

  DefaultConstructibleWrapper& operator=(
      DefaultConstructibleWrapper&& other) = default;

  int32_t& Value() { return value_; }

  const int32_t& Value() const { return value_; }
};

GTEST_TEST(OwningMaybeTest, DefaultConstructMoveAndAssign)
{
  // Test basic construction.
  const OwningMaybe<DefaultConstructibleWrapper> maybe_not;
  OwningMaybe<DefaultConstructibleWrapper> maybe_default(
      DefaultConstructibleWrapper{});
  const OwningMaybe<DefaultConstructibleWrapper> maybe_1(
      DefaultConstructibleWrapper(1));

  EXPECT_FALSE(maybe_not);
  EXPECT_TRUE(maybe_default);
  EXPECT_TRUE(maybe_1);

  EXPECT_EQ(maybe_default.Value().Value(), kDefaultValue);
  EXPECT_EQ(maybe_1.Value().Value(), 1);

  // Test value modification.
  maybe_default.Value().Value() = 5;

  EXPECT_EQ(maybe_default.Value().Value(), 5);

  // Test copy and move constructors.
  const OwningMaybe<DefaultConstructibleWrapper> copy_maybe_1(maybe_1);
  const OwningMaybe<DefaultConstructibleWrapper> copy_temp_maybe(
      OwningMaybe<DefaultConstructibleWrapper>(
          DefaultConstructibleWrapper(2)));

  EXPECT_TRUE(copy_maybe_1);
  EXPECT_TRUE(copy_temp_maybe);

  EXPECT_EQ(copy_maybe_1.Value().Value(), 1);
  EXPECT_EQ(copy_temp_maybe.Value().Value(), 2);

  // Test copy & move assignment.
  OwningMaybe<DefaultConstructibleWrapper> maybe_not_copied;
  OwningMaybe<DefaultConstructibleWrapper> maybe_not_moved;

  EXPECT_FALSE(maybe_not_copied);
  EXPECT_FALSE(maybe_not_moved);

  maybe_not_copied = maybe_1;

  EXPECT_TRUE(maybe_not_copied);
  EXPECT_EQ(maybe_not_copied.Value().Value(), 1);

  maybe_not_moved = std::move(maybe_not_copied);

  EXPECT_TRUE(maybe_not_moved);
  EXPECT_EQ(maybe_not_moved.Value().Value(), 1);
}

class NonDefaultConstructibleWrapper
{
private:
  int32_t value_ = kDefaultValue;

public:
  NonDefaultConstructibleWrapper() = delete;

  NonDefaultConstructibleWrapper(const int32_t value) : value_(value) {}

  NonDefaultConstructibleWrapper(
      const NonDefaultConstructibleWrapper& other) = default;

  NonDefaultConstructibleWrapper(
      NonDefaultConstructibleWrapper&& other) = default;

  NonDefaultConstructibleWrapper& operator=(
      const NonDefaultConstructibleWrapper& other) = default;

  NonDefaultConstructibleWrapper& operator=(
      NonDefaultConstructibleWrapper&& other) = default;

  int32_t& Value() { return value_; }

  const int32_t& Value() const { return value_; }
};

GTEST_TEST(OwningMaybeTest, NoDefaultConstructMoveAndAssign)
{
  // Test basic construction.
  OwningMaybe<NonDefaultConstructibleWrapper> maybe_not;
  OwningMaybe<NonDefaultConstructibleWrapper> maybe_1(
      NonDefaultConstructibleWrapper(1));

  EXPECT_FALSE(maybe_not);
  EXPECT_TRUE(maybe_1);
  EXPECT_EQ(maybe_1.Value().Value(), 1);

  // Test copy and move constructors.
  OwningMaybe<NonDefaultConstructibleWrapper> copy_maybe_1(maybe_1);
  OwningMaybe<NonDefaultConstructibleWrapper> copy_temp_maybe(
      OwningMaybe<NonDefaultConstructibleWrapper>(
          NonDefaultConstructibleWrapper(2)));

  EXPECT_TRUE(copy_maybe_1);
  EXPECT_TRUE(copy_temp_maybe);

  EXPECT_EQ(copy_maybe_1.Value().Value(), 1);
  EXPECT_EQ(copy_temp_maybe.Value().Value(), 2);

  // Test copy & move assignment.
  OwningMaybe<NonDefaultConstructibleWrapper> maybe_not_copied;
  OwningMaybe<NonDefaultConstructibleWrapper> maybe_not_moved;

  EXPECT_FALSE(maybe_not_copied);
  EXPECT_FALSE(maybe_not_moved);

  maybe_not_copied = maybe_1;

  EXPECT_TRUE(maybe_not_copied);
  EXPECT_EQ(maybe_not_copied.Value().Value(), 1);

  maybe_not_moved = std::move(maybe_not_copied);

  EXPECT_TRUE(maybe_not_moved);
  EXPECT_EQ(maybe_not_moved.Value().Value(), 1);
}

GTEST_TEST(OwningMaybeTest, ConstNoDefaultConstructMoveAndAssign)
{
  // Test basic construction.
  const OwningMaybe<const NonDefaultConstructibleWrapper> maybe_not;
  const OwningMaybe<const NonDefaultConstructibleWrapper> maybe_1(
      NonDefaultConstructibleWrapper(1));

  EXPECT_FALSE(maybe_not);
  EXPECT_TRUE(maybe_1);
  EXPECT_EQ(maybe_1.Value().Value(), 1);

  // Test copy and move constructors.
  const OwningMaybe<const NonDefaultConstructibleWrapper> copy_maybe_1(maybe_1);
  const OwningMaybe<const NonDefaultConstructibleWrapper> copy_temp_maybe(
      OwningMaybe<const NonDefaultConstructibleWrapper>(
          NonDefaultConstructibleWrapper(2)));

  EXPECT_TRUE(copy_maybe_1);
  EXPECT_TRUE(copy_temp_maybe);

  EXPECT_EQ(copy_maybe_1.Value().Value(), 1);
  EXPECT_EQ(copy_temp_maybe.Value().Value(), 2);

  // Test copy & move assignment.
  OwningMaybe<const NonDefaultConstructibleWrapper> maybe_not_copied;
  OwningMaybe<const NonDefaultConstructibleWrapper> maybe_not_moved;

  EXPECT_FALSE(maybe_not_copied);
  EXPECT_FALSE(maybe_not_moved);

  maybe_not_copied = maybe_1;

  EXPECT_TRUE(maybe_not_copied);
  EXPECT_EQ(maybe_not_copied.Value().Value(), 1);

  maybe_not_moved = std::move(maybe_not_copied);

  EXPECT_TRUE(maybe_not_moved);
  EXPECT_EQ(maybe_not_moved.Value().Value(), 1);
}

GTEST_TEST(OwningMaybeTest, NonTrivialConstructMoveAndAssign)
{
  // Test basic construction.
  OwningMaybe<std::vector<int32_t>> maybe_not;
  OwningMaybe<std::vector<int32_t>> maybe_default(
      std::vector<int32_t>{});
  OwningMaybe<std::vector<int32_t>> maybe_1(
      std::vector<int32_t>(1, 1));

  EXPECT_FALSE(maybe_not);
  EXPECT_TRUE(maybe_default);
  EXPECT_TRUE(maybe_1);

  EXPECT_EQ(maybe_default.Value().size(), static_cast<size_t>(0));
  EXPECT_EQ(maybe_1.Value().size(), static_cast<size_t>(1));

  // Test value modification.
  maybe_default.Value().push_back(5);

  EXPECT_EQ(maybe_default.Value().size(), static_cast<size_t>(1));
  EXPECT_EQ(maybe_default.Value().at(0), 5);

  // Test copy and move constructors.
  OwningMaybe<std::vector<int32_t>> copy_maybe_1(maybe_1);
  OwningMaybe<std::vector<int32_t>> copy_temp_maybe(
      OwningMaybe<std::vector<int32_t>>(
          std::vector<int32_t>(2, 2)));

  EXPECT_TRUE(copy_maybe_1);
  EXPECT_TRUE(copy_temp_maybe);

  EXPECT_EQ(copy_maybe_1.Value().size(), static_cast<size_t>(1));
  EXPECT_EQ(copy_temp_maybe.Value().size(), static_cast<size_t>(2));

  // Test copy & move assignment.
  OwningMaybe<std::vector<int32_t>> maybe_not_copied;
  OwningMaybe<std::vector<int32_t>> maybe_not_moved;

  EXPECT_FALSE(maybe_not_copied);
  EXPECT_FALSE(maybe_not_moved);

  maybe_not_copied = maybe_1;

  EXPECT_TRUE(maybe_not_copied);
  EXPECT_EQ(maybe_not_copied.Value().size(), static_cast<size_t>(1));

  maybe_not_moved = std::move(maybe_not_copied);

  EXPECT_TRUE(maybe_not_moved);
  EXPECT_EQ(maybe_not_moved.Value().size(), static_cast<size_t>(1));
}

GTEST_TEST(OwningMaybeTest, EigenTypeAlignment)
{
  // Test that contained Eigen types are sufficiently aligned.
  const uint64_t eigen_alignment =
      static_cast<uint64_t>(EIGEN_DEFAULT_ALIGN_BYTES);

  const OwningMaybe<Eigen::Isometry3d> stack_maybe_transform(
      Eigen::Isometry3d::Identity());

  EXPECT_TRUE(
      utility::CheckAlignment(stack_maybe_transform.Value(), eigen_alignment));

  const std::unique_ptr<OwningMaybe<Eigen::Isometry3d>> heap_maybe_transform(
      new OwningMaybe<Eigen::Isometry3d>(Eigen::Isometry3d::Identity()));

  EXPECT_TRUE(
      utility::CheckAlignment(heap_maybe_transform->Value(), eigen_alignment));
}

GTEST_TEST(ReferencingMaybeTest, ConstructMoveAndAssign)
{
  int32_t value_1 = 1;
  int32_t value_2 = 2;

  // Test basic construction.
  ReferencingMaybe<int32_t> maybe_not;
  ReferencingMaybe<int32_t> maybe_1(value_1);
  ReferencingMaybe<int32_t> maybe_2(value_2);

  EXPECT_FALSE(maybe_not);
  EXPECT_TRUE(maybe_1);
  EXPECT_TRUE(maybe_2);

  EXPECT_EQ(maybe_1.Value(), value_1);
  EXPECT_EQ(maybe_2.Value(), value_2);

  EXPECT_EQ(std::addressof(maybe_1.Value()), std::addressof(value_1));
  EXPECT_EQ(std::addressof(maybe_2.Value()), std::addressof(value_2));

  // Test value modification & assignment.
  value_1 = 10;

  EXPECT_EQ(maybe_1.Value(), value_1);

  maybe_1.Value() = 11;

  EXPECT_EQ(value_1, 11);
  EXPECT_EQ(maybe_1.Value(), 11);

  // Test copy and move constructors.
  ReferencingMaybe<int32_t> copy_maybe(maybe_1);
  ReferencingMaybe<int32_t> copy_temp_maybe(ReferencingMaybe<int32_t>{value_2});

  EXPECT_EQ(copy_maybe.Value(), value_1);
  EXPECT_EQ(copy_temp_maybe.Value(), value_2);

  EXPECT_EQ(std::addressof(copy_maybe.Value()), std::addressof(value_1));
  EXPECT_EQ(std::addressof(copy_temp_maybe.Value()), std::addressof(value_2));

  // Test copy & move assignment.
  ReferencingMaybe<int32_t> copied_from_maybe_1 = maybe_1;
  ReferencingMaybe<int32_t> moved_from_maybe_2 = std::move(maybe_2);

  EXPECT_TRUE(copied_from_maybe_1);
  EXPECT_TRUE(moved_from_maybe_2);

  EXPECT_EQ(copied_from_maybe_1.Value(), value_1);
  EXPECT_EQ(moved_from_maybe_2.Value(), value_2);

  EXPECT_EQ(
      std::addressof(copied_from_maybe_1.Value()), std::addressof(value_1));
  EXPECT_EQ(
      std::addressof(moved_from_maybe_2.Value()), std::addressof(value_2));

  ReferencingMaybe<int32_t> maybe_not_copied;
  ReferencingMaybe<int32_t> maybe_not_moved;

  EXPECT_FALSE(maybe_not_copied);
  EXPECT_FALSE(maybe_not_moved);

  maybe_not_copied = maybe_1;

  EXPECT_TRUE(maybe_not_copied);
  EXPECT_EQ(maybe_not_copied.Value(), value_1);
  EXPECT_EQ(
      std::addressof(maybe_not_copied.Value()), std::addressof(value_1));

  maybe_not_moved = std::move(maybe_not_copied);

  EXPECT_TRUE(maybe_not_moved);
  EXPECT_EQ(maybe_not_moved.Value(), value_1);
  EXPECT_EQ(
      std::addressof(maybe_not_moved.Value()), std::addressof(value_1));

  // Test reset.
  maybe_1.Reset();

  EXPECT_FALSE(maybe_1);
}

GTEST_TEST(OwningMaybeTest, ConstHandling)
{
  int32_t mutable_value = 1;
  const int32_t const_value = 2;

  OwningMaybe<int32_t> mutable_maybe_mutable_value(mutable_value);
  const OwningMaybe<int32_t> const_maybe_mutable_value(mutable_value);
  OwningMaybe<const int32_t> mutable_maybe_const_value(const_value);
  const OwningMaybe<const int32_t> const_maybe_const_value(const_value);

  EXPECT_EQ(mutable_maybe_mutable_value.Value(), mutable_value);
  EXPECT_EQ(const_maybe_mutable_value.Value(), mutable_value);
  EXPECT_EQ(mutable_maybe_const_value.Value(), const_value);
  EXPECT_EQ(const_maybe_const_value.Value(), const_value);

  EXPECT_FALSE(std::is_const<typename std::remove_reference<decltype(
      mutable_maybe_mutable_value.Value())>::type>::value);
  EXPECT_TRUE(std::is_const<typename std::remove_reference<decltype(
      const_maybe_mutable_value.Value())>::type>::value);
  EXPECT_TRUE(std::is_const<typename std::remove_reference<decltype(
      mutable_maybe_const_value.Value())>::type>::value);
  EXPECT_TRUE(std::is_const<typename std::remove_reference<decltype(
      const_maybe_const_value.Value())>::type>::value);
}

GTEST_TEST(ReferencingMaybeTest, ConstHandling)
{
  int32_t mutable_value = 1;
  const int32_t const_value = 2;

  ReferencingMaybe<int32_t> mutable_maybe_mutable_value(mutable_value);
  const ReferencingMaybe<int32_t> const_maybe_mutable_value(mutable_value);
  ReferencingMaybe<const int32_t> mutable_maybe_const_value(const_value);
  const ReferencingMaybe<const int32_t> const_maybe_const_value(const_value);

  EXPECT_EQ(mutable_maybe_mutable_value.Value(), mutable_value);
  EXPECT_EQ(const_maybe_mutable_value.Value(), mutable_value);
  EXPECT_EQ(mutable_maybe_const_value.Value(), const_value);
  EXPECT_EQ(const_maybe_const_value.Value(), const_value);

  EXPECT_FALSE(std::is_const<typename std::remove_reference<decltype(
      mutable_maybe_mutable_value.Value())>::type>::value);
  EXPECT_TRUE(std::is_const<typename std::remove_reference<decltype(
      const_maybe_mutable_value.Value())>::type>::value);
  EXPECT_TRUE(std::is_const<typename std::remove_reference<decltype(
      mutable_maybe_const_value.Value())>::type>::value);
  EXPECT_TRUE(std::is_const<typename std::remove_reference<decltype(
      const_maybe_const_value.Value())>::type>::value);
}
}  // namespace maybe_test
}  // namespace common_robotics_utilities

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
