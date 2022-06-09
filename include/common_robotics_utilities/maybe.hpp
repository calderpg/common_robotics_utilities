#pragma once

#include <stdexcept>
#include <type_traits>

#include <Eigen/Geometry>

namespace common_robotics_utilities
{
namespace internal
{
/// Helper type for OwningMaybe<T> that stores one of two values without
/// requiring default constructibility for either value.
template<typename A, typename B>
struct TaggedUnion
{
private:
  void ConstructCopy(const TaggedUnion<A, B>& other)
  {
    switch (other.type_)
    {
      case TYPE_A:
      {
        ::new(&value_a_) A(other.value_a_);
        type_ = TYPE_A;
        break;
      }
      case TYPE_B:
      {
        ::new(&value_b_) B(other.value_b_);
        type_ = TYPE_B;
        break;
      }
    }
  }

  void ConstructMove(TaggedUnion<A, B>&& other)
  {
    switch (other.type_)
    {
      case TYPE_A:
      {
        ::new(&value_a_) A(std::move(other.value_a_));
        type_ = TYPE_A;
        break;
      }
      case TYPE_B:
      {
        ::new(&value_b_) B(std::move(other.value_b_));
        type_ = TYPE_B;
        break;
      }
    }
  }

  void Cleanup()
  {
    switch (type_)
    {
      case TYPE_A:
      {
        value_a_.~A();
        break;
      }
      case TYPE_B:
      {
        value_b_.~B();
        break;
      }
    }
  }

public:
  union
  {
    A value_a_;
    B value_b_;
  };

  enum {TYPE_A, TYPE_B} type_{};

  explicit TaggedUnion(const A& value_a)
      : value_a_(value_a), type_(TYPE_A) {}

  explicit TaggedUnion(A&& value_a)
      : value_a_(std::move(value_a)), type_(TYPE_A) {}

  explicit TaggedUnion(const B& value_b)
      : value_b_(value_b), type_(TYPE_B) {}

  explicit TaggedUnion(B&& value_b)
      : value_b_(std::move(value_b)), type_(TYPE_B) {}

  TaggedUnion(const TaggedUnion<A, B>& other) { ConstructCopy(other); }

  TaggedUnion(TaggedUnion<A, B>&& other) { ConstructMove(std::move(other)); }

  ~TaggedUnion() { Cleanup(); }

  TaggedUnion<A, B>& operator=(const TaggedUnion<A, B>& other)
  {
    if (this != std::addressof(other))
    {
      Cleanup();

      ConstructCopy(other);
    }
    return *this;
  }

  TaggedUnion<A, B>& operator=(TaggedUnion<A, B>&& other)
  {
    if (this != std::addressof(other))
    {
      Cleanup();

      ConstructMove(std::move(other));
    }
    return *this;
  }
};
}  // namespace internal

/// An implementation of the Maybe/Optional pattern, in which an OwningMaybe<T>
/// wraps an instance of T or no value. This is equivalent to std::optional<T>
/// in C++17, in which OwningMaybe<T> owns the contained instance of T. If the
/// instance of T is expensive or difficult to copy/move, and is guaranteed to
/// live beyond the lifetime of the Maybe, consider using a ReferencingMaybe<T>
/// instead.
template<typename T>
class OwningMaybe
{
private:
  struct DummyType
  {
    char dummy{};
  };

  using StorageType =
      internal::TaggedUnion<DummyType, typename std::remove_const<T>::type>;

  StorageType item_storage_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OwningMaybe() : item_storage_(DummyType()) {}

  explicit OwningMaybe(const T& value) : item_storage_(value) {}

  explicit OwningMaybe(T&& value) : item_storage_(std::move(value)) {}

  OwningMaybe(const OwningMaybe<T>& other) = default;

  OwningMaybe(OwningMaybe<T>&& other) = default;

  OwningMaybe<T>& operator=(const OwningMaybe<T>& other) = default;

  OwningMaybe<T>& operator=(OwningMaybe<T>&& other) = default;

  const T& Value() const
  {
    if (HasValue())
    {
      return item_storage_.value_b_;
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
      return item_storage_.value_b_;
    }
    else
    {
      throw std::runtime_error("OwningMaybe does not have value");
    }
  }

  void Reset() { item_storage_ = StorageType(DummyType()); }

  bool HasValue() const { return (item_storage_.type_ == StorageType::TYPE_B); }

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
  T* item_ptr_ = nullptr;

public:
  ReferencingMaybe() : item_ptr_(nullptr) {}

  explicit ReferencingMaybe(T& item) : item_ptr_(std::addressof(item)) {}

  ReferencingMaybe(const ReferencingMaybe<T>& other) = default;

  ReferencingMaybe(ReferencingMaybe<T>&& other) = default;

  ReferencingMaybe<T>& operator=(const ReferencingMaybe<T>& other) = default;

  ReferencingMaybe<T>& operator=(ReferencingMaybe<T>&& other) = default;

  void Reset() { item_ptr_ = nullptr; }

  const T& Value() const
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
