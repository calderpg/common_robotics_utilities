#pragma once

#include <stdexcept>

namespace common_robotics_utilities
{
template<typename T>
class Maybe
{
private:
  T value_{};
  bool has_value_ = false;

public:
  explicit Maybe(const T& value)
      : value_(value), has_value_(true) {}

  explicit Maybe(T&& value)
      : value_(value), has_value_(true) {}

  Maybe() : has_value_(false) {}

  const T& Value() const
  {
    if (HasValue())
    {
      return value_;
    }
    else
    {
      throw std::runtime_error("Maybe does not have value");
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
      throw std::runtime_error("Maybe does not have value");
    }
  }

  bool HasValue() const { return has_value_; }

  explicit operator bool() const { return HasValue(); }
};
}
