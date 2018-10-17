#pragma once

#include <stdexcept>

namespace common_robotics_utilities
{
template<typename T>
class OwningMaybe
{
private:
  T value_{};
  bool has_value_ = false;

public:
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

// While this looks like a std::optional<T>, it *does not own* the item of T,
// unlike std::optional<T>, since it contains a reference to the object. The
// referenced object *must* outlive the ReferencingMaybe!
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
