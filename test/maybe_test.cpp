#include <cstdint>

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

  int32_t& Value() { return value_; }

  const int32_t& Value() const { return value_; }
};

GTEST_TEST(OwningMaybeTest, DefaultConstructMoveAndAssign)
{
  // Test basic construction.
  OwningMaybe<DefaultConstructibleWrapper> maybe_not;
  OwningMaybe<DefaultConstructibleWrapper> maybe_default(
      std::move(DefaultConstructibleWrapper()));
  OwningMaybe<DefaultConstructibleWrapper> maybe_1(
      DefaultConstructibleWrapper(1));

  ASSERT_FALSE(maybe_not);
  ASSERT_TRUE(maybe_default);
  ASSERT_TRUE(maybe_1);

  ASSERT_EQ(maybe_default.Value().Value(), kDefaultValue);
  ASSERT_EQ(maybe_1.Value().Value(), 1);

  // Test value modification.
  maybe_default.Value().Value() = 5;

  ASSERT_EQ(maybe_default.Value().Value(), 5);

  // Test copy and move constructors.
  OwningMaybe<DefaultConstructibleWrapper> copy_maybe_1(maybe_1);
  OwningMaybe<DefaultConstructibleWrapper> copy_temp_maybe(
      std::move(OwningMaybe<DefaultConstructibleWrapper>(
          DefaultConstructibleWrapper(2))));

  ASSERT_TRUE(copy_maybe_1);
  ASSERT_TRUE(copy_temp_maybe);

  ASSERT_EQ(copy_maybe_1.Value().Value(), 1);
  ASSERT_EQ(copy_temp_maybe.Value().Value(), 2);

  // Test copy & move assignment.
  OwningMaybe<DefaultConstructibleWrapper> maybe_not_copied;
  OwningMaybe<DefaultConstructibleWrapper> maybe_not_moved;

  ASSERT_FALSE(maybe_not_copied);
  ASSERT_FALSE(maybe_not_moved);

  maybe_not_copied = maybe_1;

  ASSERT_TRUE(maybe_not_copied);
  ASSERT_EQ(maybe_not_copied.Value().Value(), 1);

  maybe_not_moved = std::move(maybe_not_copied);

  ASSERT_TRUE(maybe_not_moved);
  ASSERT_FALSE(maybe_not_copied);
  ASSERT_EQ(maybe_not_moved.Value().Value(), 1);
}

class NonDefaultConstructibleWrapper
{
private:
  int32_t value_ = kDefaultValue;

public:
  NonDefaultConstructibleWrapper() = delete;

  NonDefaultConstructibleWrapper(const int32_t value) : value_(value) {}

  int32_t& Value() { return value_; }

  const int32_t& Value() const { return value_; }
};

GTEST_TEST(OwningMaybeTest, NoDefaultConstructMoveAndAssign)
{
  // Test basic construction.
  OwningMaybe<NonDefaultConstructibleWrapper> maybe_not;
  OwningMaybe<NonDefaultConstructibleWrapper> maybe_1(
      NonDefaultConstructibleWrapper(1));

  ASSERT_FALSE(maybe_not);
  ASSERT_TRUE(maybe_1);
  ASSERT_EQ(maybe_1.Value().Value(), 1);

  // Test copy and move constructors.
  OwningMaybe<NonDefaultConstructibleWrapper> copy_maybe_1(maybe_1);
  OwningMaybe<NonDefaultConstructibleWrapper> copy_temp_maybe(
      std::move(OwningMaybe<NonDefaultConstructibleWrapper>(
          NonDefaultConstructibleWrapper(2))));

  ASSERT_TRUE(copy_maybe_1);
  ASSERT_TRUE(copy_temp_maybe);

  ASSERT_EQ(copy_maybe_1.Value().Value(), 1);
  ASSERT_EQ(copy_temp_maybe.Value().Value(), 2);

  // Test copy & move assignment.
  OwningMaybe<NonDefaultConstructibleWrapper> maybe_not_copied;
  OwningMaybe<NonDefaultConstructibleWrapper> maybe_not_moved;

  ASSERT_FALSE(maybe_not_copied);
  ASSERT_FALSE(maybe_not_moved);

  maybe_not_copied = maybe_1;

  ASSERT_TRUE(maybe_not_copied);
  ASSERT_EQ(maybe_not_copied.Value().Value(), 1);

  maybe_not_moved = std::move(maybe_not_copied);

  ASSERT_TRUE(maybe_not_moved);
  ASSERT_FALSE(maybe_not_copied);
  ASSERT_EQ(maybe_not_moved.Value().Value(), 1);
}

GTEST_TEST(OwningMaybeTest, EigenTypeAlignment)
{
  // Test that contained Eigen types are sufficiently aligned.
  const uint64_t eigen_alignment =
      static_cast<uint64_t>(EIGEN_DEFAULT_ALIGN_BYTES);

  const OwningMaybe<Eigen::Isometry3d> stack_maybe_transform(
      Eigen::Isometry3d::Identity());

  ASSERT_TRUE(
      utility::CheckAlignment(stack_maybe_transform.Value(), eigen_alignment));

  const std::unique_ptr<OwningMaybe<Eigen::Isometry3d>> heap_maybe_transform(
      new OwningMaybe<Eigen::Isometry3d>(Eigen::Isometry3d::Identity()));

  ASSERT_TRUE(
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

  ASSERT_FALSE(maybe_not);
  ASSERT_TRUE(maybe_1);
  ASSERT_TRUE(maybe_2);

  ASSERT_EQ(maybe_1.Value(), value_1);
  ASSERT_EQ(maybe_2.Value(), value_2);

  ASSERT_EQ(std::addressof(maybe_1.Value()), std::addressof(value_1));
  ASSERT_EQ(std::addressof(maybe_2.Value()), std::addressof(value_2));

  // Test value modification & assignment.
  value_1 = 10;

  ASSERT_EQ(maybe_1.Value(), value_1);

  maybe_1.Value() = 11;

  ASSERT_EQ(value_1, 11);
  ASSERT_EQ(maybe_1.Value(), 11);

  // Test copy and move constructors.
  ReferencingMaybe<int32_t> copy_maybe(maybe_1);
  ReferencingMaybe<int32_t> copy_temp_maybe(
      std::move(ReferencingMaybe<int32_t>(value_2)));

  ASSERT_EQ(copy_maybe.Value(), value_1);
  ASSERT_EQ(copy_temp_maybe.Value(), value_2);

  ASSERT_EQ(std::addressof(copy_maybe.Value()), std::addressof(value_1));
  ASSERT_EQ(std::addressof(copy_temp_maybe.Value()), std::addressof(value_2));

  // Test copy & move assignment.
  ReferencingMaybe<int32_t> copied_from_maybe_1 = maybe_1;
  ReferencingMaybe<int32_t> moved_from_maybe_2 = std::move(maybe_2);

  ASSERT_TRUE(copied_from_maybe_1);
  ASSERT_TRUE(moved_from_maybe_2);
  ASSERT_FALSE(maybe_2);

  ASSERT_EQ(copied_from_maybe_1.Value(), value_1);
  ASSERT_EQ(moved_from_maybe_2.Value(), value_2);

  ASSERT_EQ(
      std::addressof(copied_from_maybe_1.Value()), std::addressof(value_1));
  ASSERT_EQ(
      std::addressof(moved_from_maybe_2.Value()), std::addressof(value_2));

  ReferencingMaybe<int32_t> maybe_not_copied;
  ReferencingMaybe<int32_t> maybe_not_moved;

  ASSERT_FALSE(maybe_not_copied);
  ASSERT_FALSE(maybe_not_moved);

  maybe_not_copied = maybe_1;

  ASSERT_TRUE(maybe_not_copied);
  ASSERT_EQ(maybe_not_copied.Value(), value_1);
  ASSERT_EQ(
      std::addressof(maybe_not_copied.Value()), std::addressof(value_1));

  maybe_not_moved = std::move(maybe_not_copied);

  ASSERT_TRUE(maybe_not_moved);
  ASSERT_FALSE(maybe_not_copied);
  ASSERT_EQ(maybe_not_moved.Value(), value_1);
  ASSERT_EQ(
      std::addressof(maybe_not_moved.Value()), std::addressof(value_1));

  // Test reset.
  maybe_1.Reset();

  ASSERT_FALSE(maybe_1);
}


}  // namespace maybe_test
}  // namespace common_robotics_utilities

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
