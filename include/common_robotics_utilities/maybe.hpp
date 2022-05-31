#pragma once

#include <stdexcept>

#include <Eigen/Geometry>

// If we are compiling in C++17 or later, just use std::optional<T>.
#if __cplusplus >= 201703L
#include <optional>
#endif

namespace common_robotics_utilities
{
/// An implementation of the Maybe/Optional pattern, in which an OwningMaybe<T>
/// wraps an instance of T or no value. This is equivalent to std::optional<T>
/// in C++17, in which OwningMaybe<T> owns the contained instance of T. If the
/// instance of T is expensive or difficult to copy, and is guaranteed to live
/// beyond the lifetime of the Maybe, consider using a ReferencingMaybe<T>
/// instead.
#if __cplusplus >= 201703L
// If we are compiling in C++17 or later, just wrap std::optional<T> with
// stronger behavior on move-from (moved-from OwningMaybe<T> does not contain a
// value, unlike std::optional<T>.
template<typename T>
class OwningMaybe
{
private:
  std::optional<T> item_storage_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OwningMaybe() = default;

  explicit OwningMaybe(const T& item) : item_storage_(item) {}

  explicit OwningMaybe(T&& item) : item_storage_(item) {}

  OwningMaybe(const OwningMaybe<T>& other) = default;

  OwningMaybe(OwningMaybe<T>&& other)
      : item_storage_(std::move(other.item_storage_))
  {
    other.Reset();
  }

  OwningMaybe<T>& operator=(const OwningMaybe<T>& other) = default;

  OwningMaybe<T>& operator=(OwningMaybe<T>&& other)
  {
    if (this != std::addressof(other))
    {
      item_storage_ = std::move(other.item_storage_);
      other.Reset();
    }
    return *this;
  }

  ~OwningMaybe() = default;

  const T& Value() const
  {
    if (HasValue())
    {
      // Use the unchecked accessor since we're already checking for value.
      return *item_storage_;
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
      // Use the unchecked accessor since we're already checking for value.
      return *item_storage_;
    }
    else
    {
      throw std::runtime_error("OwningMaybe does not have value");
    }
  }

  void Reset()
  {
    item_storage_.reset();
  }

  bool HasValue() const { return item_storage_.has_value(); }

  explicit operator bool() const { return HasValue(); }
};
#else
// If we are compiling in earlier than C++17, implement optional ourselves.
template<typename T>
class OwningMaybe
{
private:
  template<typename ValueType>
  union Storage
  {
    char dummy;
    ValueType item;

    Storage() : dummy() {}

    template <typename... Args>
    Storage(Args&&... args) : item(std::forward<Args>(args)...) {}

    ~Storage() {}
  };

  Storage<T> item_storage_;
  bool has_value_ = false;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OwningMaybe() : has_value_(false) {}

  explicit OwningMaybe(const T& value)
      : item_storage_(value), has_value_(true) {}

  explicit OwningMaybe(T&& value) : item_storage_(value), has_value_(true) {}

  OwningMaybe(const OwningMaybe<T>& other)
  {
    if (other.HasValue())
    {
      item_storage_.item = other.item_storage_.item;
      has_value_ = true;
    }
  }

  OwningMaybe(OwningMaybe<T>&& other)
  {
    if (other.HasValue())
    {
      item_storage_.item = std::move(other.item_storage_.item);
      has_value_ = true;
      other.has_value_ = false;
    }
  }

  OwningMaybe<T>& operator=(const OwningMaybe<T>& other)
  {
    if (this != std::addressof(other))
    {
      Reset();

      if (other.HasValue())
      {
        item_storage_.item = other.item_storage_.item;
        has_value_ = true;
      }
    }
    return *this;
  }

  OwningMaybe<T>& operator=(OwningMaybe<T>&& other)
  {
    if (this != std::addressof(other))
    {
      Reset();

      if (other.HasValue())
      {
        item_storage_.item = std::move(other.item_storage_.item);
        has_value_ = true;
        other.has_value_ = false;
      }
    }
    return *this;
  }

  ~OwningMaybe() { Reset(); }

  const T& Value() const
  {
    if (HasValue())
    {
      return item_storage_.item;
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
      return item_storage_.item;
    }
    else
    {
      throw std::runtime_error("OwningMaybe does not have value");
    }
  }

  void Reset()
  {
    if (HasValue())
    {
      item_storage_.item.T::~T();
    }
    has_value_ = false;
  }

  bool HasValue() const { return has_value_; }

  explicit operator bool() const { return HasValue(); }
};
#endif

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
  T* item_ptr_ = nullptr;

public:
  ReferencingMaybe() : item_ptr_(nullptr) {}

  explicit ReferencingMaybe(T& item) : item_ptr_(std::addressof(item)) {}

  ReferencingMaybe(const ReferencingMaybe<T>& other)
      : item_ptr_(other.item_ptr_) {}

  ReferencingMaybe(ReferencingMaybe<T>&& other)
      : item_ptr_(other.item_ptr_) { other.Reset(); }

  ReferencingMaybe<T>& operator=(const ReferencingMaybe<T>& other)
  {
    item_ptr_ = other.item_ptr_;
    return *this;
  }

  ReferencingMaybe<T>& operator=(ReferencingMaybe<T>&& other)
  {
    if (this != std::addressof(other))
    {
      item_ptr_ = other.item_ptr_;
      other.Reset();
    }
    return *this;
  }

  void Reset() { item_ptr_ = nullptr; }

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

  bool HasValue() const { return item_ptr_ != nullptr; }

  explicit operator bool() const { return HasValue(); }
};
}
