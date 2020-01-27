#pragma once

#include <stdexcept>

#include <Eigen/Geometry>

namespace common_robotics_utilities
{
/// An implementation of the Maybe/Optional pattern, in which an OwningMaybe<T>
/// wraps an instance of T or no value. This is equivalent to std::optional<T>
/// in C++17, in which OwningMaybe<T> owns the contained instance of T. If the
/// instance of T is expensive or difficult to copy, and is guaranteed to live
/// beyond the lifetime of the Maybe, consider using a ReferencingMaybe<T>
/// instead.
template<typename T>
class OwningMaybe
{
private:
  T value_{};
  bool has_value_ = false;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit OwningMaybe(const T& value)
      : value_(value), has_value_(true) {}

  explicit OwningMaybe(T&& value)
      : value_(value), has_value_(true) {}

  OwningMaybe() : has_value_(false) {}

  const T& Value() const
  {
    if (HasValue())
    {
      return value_;
    }
    else
    {
      throw std::runtime_error("OwningMaybe does not have value");
    }
  }

  T& Value()
  {
    if (HasValue())
    {
      return value_;
    }
    else
    {
      throw std::runtime_error("OwningMaybe does not have value");
    }
  }

  bool HasValue() const { return has_value_; }

  explicit operator bool() const { return HasValue(); }
};

template<typename T>
using OwningMaybeAllocator = Eigen::aligned_allocator<OwningMaybe<T>>;

/// An implementation of the Maybe/Optional pattern, in which a
/// ReferencingMaybe<T> wraps a reference to an instance of T or no value. This
/// is similar to std::optional<T> in C++17; however, it *does not own* the item
/// T, unlike std::optional<T>, since it contains a reference to the object. The
/// referenced object *must* outlive the ReferencingMaybe!
template<typename T>
class ReferencingMaybe
{
private:
  T* const item_ptr_ = nullptr;

public:
  explicit ReferencingMaybe(T& item) : item_ptr_(std::addressof(item)) {}

  ReferencingMaybe() : item_ptr_(nullptr) {}

  T& Value()
  {
    if (HasValue())
    {
      return *item_ptr_;
    }
    else
    {
      throw std::runtime_error("ReferencingMaybe does not have value");
    }
  }

  T& Value() const
  {
    if (HasValue())
    {
      return *item_ptr_;
    }
    else
    {
      throw std::runtime_error("ReferencingMaybe does not have value");
    }
  }

  bool HasValue() const { return item_ptr_ != nullptr; }

  explicit operator bool() const { return HasValue(); }
};
}
