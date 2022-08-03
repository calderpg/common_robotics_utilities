#pragma once

#include <iomanip>
#include <array>
#include <vector>
#include <deque>
#include <forward_list>
#include <list>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>

#include <Eigen/Geometry>
#include <common_robotics_utilities/utility.hpp>

namespace common_robotics_utilities
{
namespace print
{
// Base template function for printing types
template <typename T>
inline std::string Print(const T& toprint,
                         const bool add_delimiters=false,
                         const std::string& separator=", ")
{
  CRU_UNUSED(add_delimiters);
  CRU_UNUSED(separator);
  std::ostringstream strm;
  strm << toprint;
  return strm.str();
}

///////////////////////////////////////////////////////////////////
/////                     PROTOTYPES ONLY                     /////
///////////////////////////////////////////////////////////////////

template<>
inline std::string Print(const bool& bool_to_print,
                         const bool add_delimiters,
                         const std::string& separator);

template<>
inline std::string Print(const uint8_t& byte_to_print,
                         const bool add_delimiters,
                         const std::string& separator);

template<>
inline std::string Print(const Eigen::Vector2d& vector_to_print,
                         const bool add_delimiters,
                         const std::string& separator);

template<>
inline std::string Print(const Eigen::Vector3d& vector_to_print,
                         const bool add_delimiters,
                         const std::string& separator);

template<>
inline std::string Print(const Eigen::Vector4d& vector_to_print,
                         const bool add_delimiters,
                         const std::string& separator);

template<>
inline std::string Print(const Eigen::VectorXd& vector_to_print,
                         const bool add_delimiters,
                         const std::string& separator);

template<>
inline std::string Print(const Eigen::MatrixXd& matrix_to_print,
                         const bool add_delimiters,
                         const std::string& separator);

template<>
inline std::string Print(const Eigen::Quaterniond& quaternion_to_print,
                         const bool add_delimiters,
                         const std::string& separator);

template<>
inline std::string Print(const Eigen::Isometry3d& transform_to_print,
                         const bool add_delimiters,
                         const std::string& separator);

template <typename A, typename B>
inline std::string Print(const std::pair<A, B>& pairtoprint,
                         const bool add_delimiters=false,
                         const std::string& separator=", ");

template <typename T, size_t N>
inline std::string Print(const std::array<T, N>& arraytoprint,
                         const bool add_delimiters=false,
                         const std::string& separator=", ");

template <typename T, typename Allocator=std::allocator<T>>
inline std::string Print(const std::vector<T, Allocator>& vectoprint,
                         const bool add_delimiters=false,
                         const std::string& separator=", ");

template <typename T, typename Allocator=std::allocator<T>>
inline std::string Print(const std::list<T, Allocator>& listtoprint,
                         const bool add_delimiters=false,
                         const std::string& separator=", ");

template <typename T, typename Allocator=std::allocator<T>>
inline std::string Print(const std::forward_list<T, Allocator>& listtoprint,
                         const bool add_delimiters=false,
                         const std::string& separator=", ");

template <typename T, typename Allocator=std::allocator<T>>
inline std::string Print(const std::deque<T, Allocator>& dequetoprint,
                         const bool add_delimiters=false,
                         const std::string& separator=", ");

template <typename A, typename B,
          typename Compare=std::less<A>,
          typename Allocator=std::allocator<std::pair<const A, B>>>
inline std::string Print(const std::map<A, B, Compare, Allocator>& maptoprint,
                         const bool add_delimiters=false,
                         const std::string& separator=", ");

template <typename A, typename B,
          typename Compare=std::less<A>,
          typename Allocator=std::allocator<std::pair<const A, B>>>
inline std::string Print(
    const std::multimap<A, B, Compare, Allocator>& maptoprint,
    const bool add_delimiters=false,
    const std::string& separator=", ");

template <typename T,
          typename Compare=std::less<T>,
          typename Allocator=std::allocator<T>>
inline std::string Print(const std::set<T, Compare, Allocator>& settoprint,
                         const bool add_delimiters=false,
                         const std::string& separator=", ");

template <typename T,
          typename Compare=std::less<T>,
          typename Allocator=std::allocator<T>>
inline std::string Print(const std::multiset<T, Compare, Allocator>& settoprint,
                         const bool add_delimiters=false,
                         const std::string& separator=", ");

template <typename A, typename B,
          typename Hash=std::hash<A>,
          typename Predicate=std::equal_to<A>,
          typename Allocator=std::allocator<std::pair<const A, B>>>
inline std::string Print(
    const std::unordered_map<A, B, Hash, Predicate, Allocator>& maptoprint,
    const bool add_delimiters=false,
    const std::string& separator=", ");

template <typename A, typename B,
          typename Hash=std::hash<A>,
          typename Predicate=std::equal_to<A>,
          typename Allocator=std::allocator<std::pair<const A, B>>>
inline std::string Print(
    const std::unordered_multimap<A, B, Hash, Predicate, Allocator>& maptoprint,
    const bool add_delimiters=false,
    const std::string& separator=", ");

template <typename T,
          typename Hash=std::hash<T>,
          typename Predicate=std::equal_to<T>,
          typename Allocator=std::allocator<T>>
inline std::string Print(
    const std::unordered_set<T, Hash, Predicate, Allocator>& settoprint,
    const bool add_delimiters=false,
    const std::string& separator=", ");

template <typename T,
          typename Hash=std::hash<T>,
          typename Predicate=std::equal_to<T>,
          typename Allocator=std::allocator<T>>
inline std::string Print(
    const std::unordered_multiset<T, Hash, Predicate, Allocator>& settoprint,
    const bool add_delimiters=false,
    const std::string& separator=", ");

/////////////////////////////////////////////////////////////////////
/////                   IMPLEMENTATIONS ONLY                    /////
/////////////////////////////////////////////////////////////////////

template<>
inline std::string Print(const bool& bool_to_print,
                         const bool add_delimiters,
                         const std::string& separator)
{
  CRU_UNUSED(add_delimiters);
  CRU_UNUSED(separator);
  if (bool_to_print)
  {
    return "true";
  }
  else
  {
    return "false";
  }
}

template<>
inline std::string Print(const uint8_t& byte_to_print,
                         const bool add_delimiters,
                         const std::string& separator)
{
  CRU_UNUSED(add_delimiters);
  CRU_UNUSED(separator);
  return std::to_string(static_cast<int32_t>(byte_to_print));
}

template<>
inline std::string Print(const Eigen::Vector2d& vector_to_print,
                         const bool add_delimiters,
                         const std::string& separator)
{
  CRU_UNUSED(add_delimiters);
  CRU_UNUSED(separator);
  std::ostringstream strm;
  strm << std::setprecision(12);
  strm << "Vector2d: <x: ";
  strm << vector_to_print(0);
  strm << " y: ";
  strm << vector_to_print(1);
  strm << ">";
  return strm.str();
}

template<>
inline std::string Print(const Eigen::Vector3d& vector_to_print,
                         const bool add_delimiters,
                         const std::string& separator)
{
  CRU_UNUSED(add_delimiters);
  CRU_UNUSED(separator);
  std::ostringstream strm;
  strm << std::setprecision(12);
  strm << "Vector3d: <x: ";
  strm << vector_to_print.x();
  strm << " y: ";
  strm << vector_to_print.y();
  strm << " z: ";
  strm << vector_to_print.z();
  strm << ">";
  return strm.str();
}

template<>
inline std::string Print(const Eigen::Vector4d& vector_to_print,
                         const bool add_delimiters,
                         const std::string& separator)
{
  CRU_UNUSED(add_delimiters);
  CRU_UNUSED(separator);
  std::ostringstream strm;
  strm << std::setprecision(12);
  strm << "Vector4d: <x: ";
  strm << vector_to_print(0);
  strm << " y: ";
  strm << vector_to_print(1);
  strm << " z: ";
  strm << vector_to_print(2);
  strm << " w: ";
  strm << vector_to_print(3);
  strm << ">";
  return strm.str();
}

template<>
inline std::string Print(const Eigen::VectorXd& vector_to_print,
                         const bool add_delimiters,
                         const std::string& separator)
{
  CRU_UNUSED(add_delimiters);
  CRU_UNUSED(separator);
  if (vector_to_print.size() > 0)
  {
    std::ostringstream strm;
    strm << std::setprecision(12);
    strm << "VectorXd: <" << vector_to_print(0);
    for (ssize_t idx = 1; idx < vector_to_print.size(); idx++)
    {
      strm << ", " << vector_to_print(idx);
    }
    strm << ">";
    return strm.str();
  }
  else
  {
    return "VectorXd: <>";
  }
}

template<>
inline std::string Print(const Eigen::MatrixXd& matrix_to_print,
                         const bool add_delimiters,
                         const std::string& separator)
{
  CRU_UNUSED(add_delimiters);
  CRU_UNUSED(separator);
  if (matrix_to_print.rows() > 0 && matrix_to_print.cols() > 0)
  {
    std::ostringstream strm;
    strm << std::setprecision(12);
    strm << "MatrixXd:\n[";
    strm << matrix_to_print(0, 0);
    for (int64_t col = 1; col < matrix_to_print.cols(); col++)
    {
      strm << ", " << matrix_to_print(0, col);
    }
    for (int64_t row = 1; row < matrix_to_print.rows(); row++)
    {
      strm << "\n" << matrix_to_print(row, 0);
      for (int64_t col = 1; col < matrix_to_print.cols(); col++)
      {
        strm << ", " << matrix_to_print(row, col);
      }
    }
    strm << "]";
    return strm.str();
  }
  else
  {
    return "MatrixXd:\n[]";
  }
}

template<>
inline std::string Print(const Eigen::Quaterniond& quaternion_to_print,
                         const bool add_delimiters,
                         const std::string& separator)
{
  CRU_UNUSED(add_delimiters);
  CRU_UNUSED(separator);
  std::ostringstream strm;
  strm << std::setprecision(12);
  strm << "Quaterniond: <x: ";
  strm << quaternion_to_print.x();
  strm << " y: ";
  strm << quaternion_to_print.y();
  strm << " z: ";
  strm << quaternion_to_print.z();
  strm << " w: ";
  strm << quaternion_to_print.w();
  strm << ">";
  return strm.str();
}

template<>
inline std::string Print(const Eigen::Isometry3d& transform_to_print,
                         const bool add_delimiters,
                         const std::string& separator)
{
  CRU_UNUSED(add_delimiters);
  CRU_UNUSED(separator);
  const Eigen::Vector3d vector_to_print = transform_to_print.translation();
  const Eigen::Quaterniond quaternion_to_print(transform_to_print.rotation());
  std::ostringstream strm;
  strm << std::setprecision(12);
  strm << "Isometry3d: <x: ";
  strm << vector_to_print.x();
  strm << " y: ";
  strm << vector_to_print.y();
  strm << " z: ";
  strm << vector_to_print.z();
  strm << ">, <x: ";
  strm << quaternion_to_print.x();
  strm << " y: ";
  strm << quaternion_to_print.y();
  strm << " z: ";
  strm << quaternion_to_print.z();
  strm << " w: ";
  strm << quaternion_to_print.w();
  strm << ">";
  return strm.str();
}

template <typename A, typename B>
inline std::string Print(const std::pair<A, B>& pairtoprint,
                         const bool add_delimiters,
                         const std::string& separator)
{
  std::ostringstream strm;
  if (add_delimiters)
  {
    strm << "<" << Print(pairtoprint.first, add_delimiters, separator);
    strm << ": " << Print(pairtoprint.second, add_delimiters, separator) << ">";
  }
  else
  {
    strm << Print(pairtoprint.first, add_delimiters, separator);
    strm << ": " << Print(pairtoprint.second, add_delimiters, separator);
  }
  return strm.str();
}

template <typename T, size_t N>
inline std::string Print(const std::array<T, N>& arraytoprint,
                         const bool add_delimiters,
                         const std::string& separator)
{
  std::ostringstream strm;
  if (arraytoprint.size() > 0)
  {
    if (add_delimiters)
    {
      strm << "[" << Print(arraytoprint[0], add_delimiters, separator);
      for (size_t idx = 1; idx < arraytoprint.size(); idx++)
      {
        strm << separator;
        strm << Print(arraytoprint[idx], add_delimiters, separator);
      }
      strm << "]";
    }
    else
    {
      strm << Print(arraytoprint[0], add_delimiters, separator);
      for (size_t idx = 1; idx < arraytoprint.size(); idx++)
      {
        strm << separator;
        strm << Print(arraytoprint[idx], add_delimiters, separator);
      }
    }
  }
  return strm.str();
}

template <typename T, typename Allocator>
inline std::string Print(const std::vector<T, Allocator>& vectoprint,
                         const bool add_delimiters,
                         const std::string& separator)
{
  std::ostringstream strm;
  if (vectoprint.size() > 0)
  {
    if (add_delimiters)
    {
      strm << "[" << Print(vectoprint[0], add_delimiters, separator);
      for (size_t idx = 1; idx < vectoprint.size(); idx++)
      {
        strm << separator << Print(vectoprint[idx], add_delimiters, separator);
      }
      strm << "]";
    }
    else
    {
      strm << Print(vectoprint[0], add_delimiters, separator);
      for (size_t idx = 1; idx < vectoprint.size(); idx++)
      {
        strm << separator << Print(vectoprint[idx], add_delimiters, separator);
      }
    }
  }
  return strm.str();
}

template <typename T, typename Allocator>
inline std::string Print(const std::list<T, Allocator>& listtoprint,
                         const bool add_delimiters,
                         const std::string& separator)
{
  std::ostringstream strm;
  if (listtoprint.size() > 0)
  {
    if (add_delimiters)
    {
      strm << "[";
      typename std::list<T, Allocator>::const_iterator itr;
      for (itr = listtoprint.begin(); itr != listtoprint.end(); ++itr)
      {
        if (itr != listtoprint.begin())
        {
          strm << separator << Print(*itr, add_delimiters, separator);
        }
        else
        {
          strm << Print(*itr, add_delimiters, separator);
        }
      }
      strm << "]";
    }
    else
    {
      typename std::list<T, Allocator>::const_iterator itr;
      for (itr = listtoprint.begin(); itr != listtoprint.end(); ++itr)
      {
        if (itr != listtoprint.begin())
        {
          strm << separator << Print(*itr, add_delimiters, separator);
        }
        else
        {
          strm << Print(*itr, add_delimiters, separator);
        }
      }
    }
  }
  return strm.str();
}

template <typename T, typename Allocator>
inline std::string Print(const std::forward_list<T, Allocator>& listtoprint,
                         const bool add_delimiters,
                         const std::string& separator)
{
  std::ostringstream strm;
  if (listtoprint.size() > 0)
  {
    if (add_delimiters)
    {
      strm << "[";
      typename std::forward_list<T, Allocator>::const_iterator itr;
      for (itr = listtoprint.begin(); itr != listtoprint.end(); ++itr)
      {
        if (itr != listtoprint.begin())
        {
          strm << separator << Print(*itr, add_delimiters, separator);
        }
        else
        {
          strm << Print(*itr, add_delimiters, separator);
        }
      }
      strm << "]";
    }
    else
    {
      typename std::forward_list<T, Allocator>::const_iterator itr;
      for (itr = listtoprint.begin(); itr != listtoprint.end(); ++itr)
      {
        if (itr != listtoprint.begin())
        {
          strm << separator << Print(*itr, add_delimiters, separator);
        }
        else
        {
          strm << Print(*itr, add_delimiters, separator);
        }
      }
    }
  }
  return strm.str();
}

template <typename T, typename Allocator>
inline std::string Print(const std::deque<T, Allocator>& dequetoprint,
                         const bool add_delimiters,
                         const std::string& separator)
{
  std::ostringstream strm;
  if (dequetoprint.size() > 0)
  {
    if (add_delimiters)
    {
      strm << "[";
      typename std::deque<T, Allocator>::const_iterator itr;
      for (itr = dequetoprint.begin(); itr != dequetoprint.end(); ++itr)
      {
        if (itr != dequetoprint.begin())
        {
          strm << separator << Print(*itr, add_delimiters, separator);
        }
        else
        {
          strm << Print(*itr, add_delimiters, separator);
        }
      }
      strm << "]";
    }
    else
    {
      typename std::deque<T, Allocator>::const_iterator itr;
      for (itr = dequetoprint.begin(); itr != dequetoprint.end(); ++itr)
      {
        if (itr != dequetoprint.begin())
        {
          strm << separator << Print(*itr, add_delimiters, separator);
        }
        else
        {
          strm << Print(*itr, add_delimiters, separator);
        }
      }
    }
  }
  return strm.str();
}

template <typename A, typename B, typename Compare, typename Allocator>
inline std::string Print(const std::map<A, B, Compare, Allocator>& maptoprint,
                         const bool add_delimiters,
                         const std::string& separator)
{
  std::ostringstream strm;
  if (maptoprint.size() > 0)
  {
    if (add_delimiters)
    {
      strm << "{";
      typename std::map<A, B, Compare, Allocator>::const_iterator itr;
      for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
      {
        std::pair<A, B> cur_pair(itr->first, itr->second);
        if (itr != maptoprint.begin())
        {
          strm << separator << Print(cur_pair, add_delimiters, separator);
        }
        else
        {
          strm << Print(cur_pair, add_delimiters, separator);
        }
      }
      strm << "}";
    }
    else
    {
      typename std::map<A, B, Compare, Allocator>::const_iterator itr;
      for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
      {
        std::pair<A, B> cur_pair(itr->first, itr->second);
        if (itr != maptoprint.begin())
        {
          strm << separator << Print(cur_pair, add_delimiters, separator);
        }
        else
        {
          strm << Print(cur_pair, add_delimiters, separator);
        }
      }
    }
  }
  return strm.str();
}

template <typename A, typename B, typename Compare, typename Allocator>
inline std::string Print(
    const std::multimap<A, B, Compare, Allocator>& maptoprint,
    const bool add_delimiters,
    const std::string& separator)
{
  std::ostringstream strm;
  if (maptoprint.size() > 0)
  {
    if (add_delimiters)
    {
      strm << "{";
      typename std::multimap<A, B, Compare, Allocator>::const_iterator itr;
      for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
      {
        std::pair<A, B> cur_pair(itr->first, itr->second);
        if (itr != maptoprint.begin())
        {
          strm << separator << Print(cur_pair, add_delimiters, separator);
        }
        else
        {
          strm << Print(cur_pair, add_delimiters, separator);
        }
      }
      strm << "}";
    }
    else
    {
      typename std::multimap<A, B, Compare, Allocator>::const_iterator itr;
      for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
      {
        std::pair<A, B> cur_pair(itr->first, itr->second);
        if (itr != maptoprint.begin())
        {
          strm << separator << Print(cur_pair, add_delimiters, separator);
        }
        else
        {
          strm << Print(cur_pair, add_delimiters, separator);
        }
      }
    }
  }
  return strm.str();
}

template <typename T, typename Compare, typename Allocator>
inline std::string Print(const std::set<T, Compare, Allocator>& settoprint,
                         const bool add_delimiters,
                         const std::string& separator)
{
  std::ostringstream strm;
  if (settoprint.size() > 0)
  {
    if (add_delimiters)
    {
      strm << "(";
      typename std::set<T, Compare, Allocator>::const_iterator itr;
      for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
      {
        if (itr != settoprint.begin())
        {
          strm << separator << Print(*itr, add_delimiters, separator);
        }
        else
        {
          strm << Print(*itr, add_delimiters, separator);
        }
      }
      strm << ")";
    }
    else
    {
      typename std::set<T, Compare, Allocator>::const_iterator itr;
      for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
      {
        if (itr != settoprint.begin())
        {
          strm << separator << Print(*itr, add_delimiters, separator);
        }
        else
        {
          strm << Print(*itr, add_delimiters, separator);
        }
      }
    }
  }
  return strm.str();
}

template <typename T, typename Compare, typename Allocator>
inline std::string Print(const std::multiset<T, Compare, Allocator>& settoprint,
                         const bool add_delimiters,
                         const std::string& separator)
{
  std::ostringstream strm;
  if (settoprint.size() > 0)
  {
    if (add_delimiters)
    {
      strm << "(";
      typename std::multiset<T, Compare, Allocator>::const_iterator itr;
      for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
      {
        if (itr != settoprint.begin())
        {
          strm << separator << Print(*itr, add_delimiters, separator);
        }
        else
        {
          strm << Print(*itr, add_delimiters, separator);
        }
      }
      strm << ")";
    }
    else
    {
      typename std::multiset<T, Compare, Allocator>::const_iterator itr;
      for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
      {
        if (itr != settoprint.begin())
        {
          strm << separator << Print(*itr, add_delimiters, separator);
        }
        else
        {
          strm << Print(*itr, add_delimiters, separator);
        }
      }
    }
  }
  return strm.str();
}

template <typename A, typename B,
          typename Hash, typename Predicate, typename Allocator>
inline std::string Print(
    const std::unordered_map<A, B, Hash, Predicate, Allocator>& maptoprint,
    const bool add_delimiters,
    const std::string& separator)
{
  std::ostringstream strm;
  if (maptoprint.size() > 0)
  {
    if (add_delimiters)
    {
      strm << "{";
      typename std::unordered_map<A, B, Hash, Predicate, Allocator>
          ::const_iterator itr;
      for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
      {
        std::pair<A, B> cur_pair(itr->first, itr->second);
        if (itr != maptoprint.begin())
        {
          strm << separator << Print(cur_pair, add_delimiters, separator);
        }
        else
        {
          strm << Print(cur_pair, add_delimiters, separator);
        }
      }
      strm << "}";
    }
    else
    {
      typename std::unordered_map<A, B, Hash, Predicate, Allocator>
          ::const_iterator itr;
      for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
      {
        std::pair<A, B> cur_pair(itr->first, itr->second);
        if (itr != maptoprint.begin())
        {
          strm << separator << Print(cur_pair, add_delimiters, separator);
        }
        else
        {
          strm << Print(cur_pair, add_delimiters, separator);
        }
      }
    }
  }
  return strm.str();
}

template <typename A, typename B,
          typename Hash, typename Predicate, typename Allocator>
inline std::string Print(
    const std::unordered_multimap<A, B, Hash, Predicate, Allocator>& maptoprint,
    const bool add_delimiters,
    const std::string& separator)
{
  std::ostringstream strm;
  if (maptoprint.size() > 0)
  {
    if (add_delimiters)
    {
      strm << "{";
      typename std::unordered_multimap<A, B, Hash, Predicate, Allocator>
          ::const_iterator itr;
      for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
      {
        std::pair<A, B> cur_pair(itr->first, itr->second);
        if (itr != maptoprint.begin())
        {
          strm << separator << Print(cur_pair, add_delimiters, separator);
        }
        else
        {
          strm << Print(cur_pair, add_delimiters, separator);
        }
      }
      strm << "}";
    }
    else
    {
      typename std::unordered_multimap<A, B, Hash, Predicate, Allocator>
          ::const_iterator itr;
      for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
      {
        std::pair<A, B> cur_pair(itr->first, itr->second);
        if (itr != maptoprint.begin())
        {
          strm << separator << Print(cur_pair, add_delimiters, separator);
        }
        else
        {
          strm << Print(cur_pair, add_delimiters, separator);
        }
      }
    }
  }
  return strm.str();
}

template <typename T, typename Hash, typename Predicate, typename Allocator>
inline std::string Print(
    const std::unordered_set<T, Hash, Predicate, Allocator>& settoprint,
    const bool add_delimiters,
    const std::string& separator)
{
  std::ostringstream strm;
  if (settoprint.size() > 0)
  {
    if (add_delimiters)
    {
      strm << "(";
      typename std::unordered_set<T, Hash, Predicate, Allocator>
          ::const_iterator itr;
      for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
      {
        if (itr != settoprint.begin())
        {
          strm << separator << Print(*itr, add_delimiters, separator);
        }
        else
        {
          strm << Print(*itr, add_delimiters, separator);
        }
      }
      strm << ")";
    }
    else
    {
      typename std::unordered_set<T, Hash, Predicate, Allocator>
          ::const_iterator itr;
      for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
      {
        if (itr != settoprint.begin())
        {
          strm << separator << Print(*itr, add_delimiters, separator);
        }
        else
        {
          strm << Print(*itr, add_delimiters, separator);
        }
      }
    }
  }
  return strm.str();
}

template <typename T, typename Hash, typename Predicate, typename Allocator>
inline std::string Print(
    const std::unordered_multiset<T, Hash, Predicate, Allocator>& settoprint,
    const bool add_delimiters,
    const std::string& separator)
{
  std::ostringstream strm;
  if (settoprint.size() > 0)
  {
    if (add_delimiters)
    {
      strm << "(";
      typename std::unordered_multiset<T, Hash, Predicate, Allocator>
          ::const_iterator itr;
      for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
      {
        if (itr != settoprint.begin())
        {
          strm << separator << Print(*itr, add_delimiters, separator);
        }
        else
        {
          strm << Print(*itr, add_delimiters, separator);
        }
      }
      strm << ")";
    }
    else
    {
      typename std::unordered_multiset<T, Hash, Predicate, Allocator>
          ::const_iterator itr;
      for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
      {
        if (itr != settoprint.begin())
        {
          strm << separator << Print(*itr, add_delimiters, separator);
        }
        else
        {
          strm << Print(*itr, add_delimiters, separator);
        }
      }
    }
  }
  return strm.str();
}
}  // namespace print
}  // namespace common_robotics_utilities
