#pragma once

#include <cstdint>
#include <limits>
#include <ostream>
#include <type_traits>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/cru_namespace.hpp>
#include <common_robotics_utilities/utility.hpp>

namespace common_robotics_utilities
{
CRU_NAMESPACE_BEGIN
namespace voxel_grid
{
// Access status values for voxel grid operations.
enum class AccessStatus : uint8_t
{
  UNKNOWN = 0x00,            // Default unknown value.
  SUCCESS,                   // Access/operation vas successful.
  NOT_FOUND,                 // Location/index was not part of the DSHVG.
  OUT_OF_BOUNDS,             // Location/index was out of bounds.
  MUTABLE_ACCESS_PROHIBITED  // Mutable access/operation was prohibited.
};

using Vector3i64 = Eigen::Matrix<int64_t, 3, 1>;

class GridIndex
{
private:
  // These defaults mean that a default-constructed GridIndex cannot be a valid
  // index for any VoxelGrid (whose indices start at 0,0,0) or DSHVG (whose
  // indices may be negative, but no lower than -2^52).
  int64_t x_ = std::numeric_limits<int64_t>::lowest();
  int64_t y_ = std::numeric_limits<int64_t>::lowest();
  int64_t z_ = std::numeric_limits<int64_t>::lowest();

public:
  GridIndex() = default;

  GridIndex(const int64_t x, const int64_t y, const int64_t z)
      : x_(x), y_(y), z_(z) {}

  const int64_t& X() const { return x_; }

  const int64_t& Y() const { return y_; }

  const int64_t& Z() const { return z_; }

  int64_t& X() { return x_; }

  int64_t& Y() { return y_; }

  int64_t& Z() { return z_; }

  bool operator==(const GridIndex& other) const
  {
    return (X() == other.X() && Y() == other.Y() && Z() == other.Z());
  }

  bool operator!=(const GridIndex& other) const
  {
    return !(*this == other);
  }
};

static_assert(
    std::is_trivially_destructible<GridIndex>::value,
    "GridIndex must be trivially destructible");

inline GridIndex operator+(const GridIndex& a, const GridIndex& b)
{
  return GridIndex(a.X() + b.X(), a.Y() + b.Y(), a.Z() + b.Z());
}

inline GridIndex operator-(const GridIndex& a, const GridIndex& b)
{
  return GridIndex(a.X() - b.X(), a.Y() - b.Y(), a.Z() - b.Z());
}

inline GridIndex operator+(const GridIndex& a, const Vector3i64& v)
{
  return GridIndex(a.X() + v.x(), a.Y() + v.y(), a.Z() + v.z());
}

inline GridIndex operator-(const GridIndex& a, const Vector3i64& v)
{
  return GridIndex(a.X() - v.x(), a.Y() - v.y(), a.Z() - v.z());
}

inline std::ostream& operator<<(std::ostream& strm, const GridIndex& index)
{
  strm << "GridIndex: (" << index.X() << "," << index.Y() << "," << index.Z()
       << ")";
  return strm;
}

}  // namespace voxel_grid
CRU_NAMESPACE_END
}  // namespace common_robotics_utilities

namespace std
{
template <>
struct hash<common_robotics_utilities::voxel_grid::GridIndex>
{
  std::size_t operator()(
      const common_robotics_utilities::voxel_grid::GridIndex& index) const
  {
    std::size_t hash_val = 0;
    common_robotics_utilities::utility::hash_combine(
        hash_val, index.X(), index.Y(), index.Z());
    return hash_val;
  }
};
}
