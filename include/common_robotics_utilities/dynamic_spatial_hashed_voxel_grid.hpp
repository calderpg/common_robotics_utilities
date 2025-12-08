#pragma once

#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/cru_namespace.hpp>
#include <common_robotics_utilities/maybe.hpp>
#include <common_robotics_utilities/serialization.hpp>
#include <common_robotics_utilities/utility.hpp>
#include <common_robotics_utilities/voxel_grid_common.hpp>

namespace common_robotics_utilities
{
CRU_NAMESPACE_BEGIN
namespace voxel_grid
{
// Since the internal index math uses doubles to avoid integer division, we
// prohibit index axis values beyond those representable exactly by a double.
constexpr int64_t MAXIMUM_ALLOWED_DSHVG_INDEX_AXIS_VALUE =
    INT64_C(0x0010000000000000);
constexpr int64_t MINIMUM_ALLOWED_DSHVG_INDEX_AXIS_VALUE =
    -MAXIMUM_ALLOWED_DSHVG_INDEX_AXIS_VALUE;

class ChunkBase
{
private:
  // These defaults mean that a default-constructed ChunkBase cannot be a valid
  // index for any VoxelGrid (whose indices start at 0,0,0) or DSHVG (whose
  // indices may be negative, but no lower than -2^52).
  int64_t x_ = std::numeric_limits<int64_t>::lowest();
  int64_t y_ = std::numeric_limits<int64_t>::lowest();
  int64_t z_ = std::numeric_limits<int64_t>::lowest();

public:
  static uint64_t Serialize(
      const ChunkBase& base, std::vector<uint8_t>& buffer)
  {
    const uint64_t start_buffer_size = buffer.size();
    // Serialize members
    serialization::SerializeMemcpyable<int64_t>(base.X(), buffer);
    serialization::SerializeMemcpyable<int64_t>(base.Y(), buffer);
    serialization::SerializeMemcpyable<int64_t>(base.Z(), buffer);
    // Figure out how many bytes were written
    const uint64_t end_buffer_size = buffer.size();
    const uint64_t bytes_written = end_buffer_size - start_buffer_size;
    return bytes_written;
  }

  static serialization::Deserialized<ChunkBase> Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
  {
    uint64_t current_position = starting_offset;
    const auto x_deserialized =
        serialization::DeserializeMemcpyable<int64_t>(
            buffer, current_position);
    const int64_t x = x_deserialized.Value();
    current_position += x_deserialized.BytesRead();
    const auto y_deserialized =
        serialization::DeserializeMemcpyable<int64_t>(
            buffer, current_position);
    const int64_t y = y_deserialized.Value();
    current_position += y_deserialized.BytesRead();
    const auto z_deserialized =
        serialization::DeserializeMemcpyable<int64_t>(
            buffer, current_position);
    const int64_t z = z_deserialized.Value();
    current_position += z_deserialized.BytesRead();
    // Figure out how many bytes were read
    const uint64_t bytes_read = current_position - starting_offset;
    return serialization::MakeDeserialized(ChunkBase(x, y, z), bytes_read);
  }

  ChunkBase() {}

  ChunkBase(const int64_t x, const int64_t y, const int64_t z)
      : x_(x), y_(y), z_(z) {}

  int64_t X() const { return x_; }

  int64_t Y() const { return y_; }

  int64_t Z() const { return z_; }

  bool operator==(const ChunkBase& other) const
  {
    return (X() == other.X() && Y() == other.Y() && Z() == other.Z());
  }

  bool operator!=(const ChunkBase& other) const
  {
    return !(*this == other);
  }
};

static_assert(
    std::is_trivially_destructible<ChunkBase>::value,
    "ChunkBase must be trivially destructible");

class ChunkIndex
{
private:
  // These defaults mean that a default-constructed ChunkIndex cannot be valid.
  int64_t x_ = std::numeric_limits<int64_t>::lowest();
  int64_t y_ = std::numeric_limits<int64_t>::lowest();
  int64_t z_ = std::numeric_limits<int64_t>::lowest();

public:
  ChunkIndex() = default;

  ChunkIndex(const int64_t x, const int64_t y, const int64_t z)
      : x_(x), y_(y), z_(z) {}

  const int64_t& X() const { return x_; }

  const int64_t& Y() const { return y_; }

  const int64_t& Z() const { return z_; }

  int64_t& X() { return x_; }

  int64_t& Y() { return y_; }

  int64_t& Z() { return z_; }

  bool operator==(const ChunkIndex& other) const
  {
    return (X() == other.X() && Y() == other.Y() && Z() == other.Z());
  }

  bool operator!=(const ChunkIndex& other) const
  {
    return !(*this == other);
  }
};

static_assert(
    std::is_trivially_destructible<ChunkIndex>::value,
    "ChunkIndex must be trivially destructible");

inline GridIndex operator+(const ChunkIndex& index, const ChunkBase& base)
{
  return GridIndex(
      index.X() + base.X(), index.Y() + base.Y(), index.Z() + base.Z());
}

inline ChunkIndex operator-(const GridIndex& index, const ChunkBase& base)
{
  return ChunkIndex(
      index.X() - base.X(), index.Y() - base.Y(), index.Z() - base.Z());
}

class DynamicSpatialHashedVoxelGridSizes
{
private:
  // Voxel sizes (and their inverses) are stored in Vector4 to enable certain
  // SIMD operations.
  Eigen::Vector4d voxel_sizes_ = Eigen::Vector4d::Zero();
  Eigen::Vector4d inverse_voxel_sizes_ = Eigen::Vector4d::Zero();
  // Chunk voxel counts (and their inverses) are stored in Vector4d to avoid
  // integer division and to enable certain SIMD operations.
  Eigen::Vector4d chunk_voxel_counts_internal_ = Eigen::Vector4d::Zero();
  Eigen::Vector4d inverse_chunk_voxel_counts_internal_ =
      Eigen::Vector4d::Zero();
  // "Normal" chunk sizes and voxel counts.
  Eigen::Vector3d chunk_sizes_ = Eigen::Vector3d::Zero();
  Vector3i64 chunk_voxel_counts_ = Vector3i64::Zero();
  int64_t chunk_num_total_voxels_ = 0;
  int64_t chunk_stride_1_ = 0;
  int64_t chunk_stride_2_ = 0;

  static bool CheckPositiveValid(const double param)
  {
    return (std::isfinite(param) && (param > 0.0));
  }

  static bool CheckPositiveValid(const int64_t param)
  {
    return (param > 0);
  }

  // This constructor is private so that users can only construct via the named
  // factory methods, which avoids ambiguity between chunk sizes and chunk voxel
  // counts parameters.
  DynamicSpatialHashedVoxelGridSizes(
      const Eigen::Vector3d& voxel_sizes, const Vector3i64& chunk_voxel_counts)
  {
    const bool initialized = Initialize(voxel_sizes, chunk_voxel_counts);

    if (!initialized)
    {
      throw std::invalid_argument(
          "All size parameters must be positive, non-zero, and finite");
    }
  }

public:
  static uint64_t Serialize(
      const DynamicSpatialHashedVoxelGridSizes& sizes,
      std::vector<uint8_t>& buffer)
  {
    const uint64_t start_buffer_size = buffer.size();

    // Serialize everything needed to reproduce the grid sizes.
    serialization::SerializeMemcpyable<double>(sizes.VoxelXSize(), buffer);
    serialization::SerializeMemcpyable<double>(sizes.VoxelYSize(), buffer);
    serialization::SerializeMemcpyable<double>(sizes.VoxelZSize(), buffer);
    serialization::SerializeMemcpyable<int64_t>(
        sizes.ChunkNumXVoxels(), buffer);
    serialization::SerializeMemcpyable<int64_t>(
        sizes.ChunkNumYVoxels(), buffer);
    serialization::SerializeMemcpyable<int64_t>(
        sizes.ChunkNumZVoxels(), buffer);

    // Figure out how many bytes were written.
    const uint64_t end_buffer_size = buffer.size();
    const uint64_t bytes_written = end_buffer_size - start_buffer_size;
    return bytes_written;
  }

  static serialization::Deserialized<DynamicSpatialHashedVoxelGridSizes>
  Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
  {
    uint64_t current_position = starting_offset;

    // Deserialize voxel sizes.
    const auto voxel_x_size_deserialized =
        serialization::DeserializeMemcpyable<double>(buffer, current_position);
    const double voxel_x_size = voxel_x_size_deserialized.Value();
    current_position += voxel_x_size_deserialized.BytesRead();
    const auto voxel_y_size_deserialized =
        serialization::DeserializeMemcpyable<double>(buffer, current_position);
    const double voxel_y_size = voxel_y_size_deserialized.Value();
    current_position += voxel_y_size_deserialized.BytesRead();
    const auto voxel_z_size_deserialized =
        serialization::DeserializeMemcpyable<double>(buffer, current_position);
    const double voxel_z_size = voxel_z_size_deserialized.Value();
    current_position += voxel_z_size_deserialized.BytesRead();

    const Eigen::Vector3d voxel_sizes(voxel_x_size, voxel_y_size, voxel_z_size);

    // Deserialize voxel counts.
    const auto chunk_num_x_voxels_deserialized =
        serialization::DeserializeMemcpyable<int64_t>(buffer, current_position);
    const int64_t chunk_num_x_voxels = chunk_num_x_voxels_deserialized.Value();
    current_position += chunk_num_x_voxels_deserialized.BytesRead();
    const auto chunk_num_y_voxels_deserialized =
        serialization::DeserializeMemcpyable<int64_t>(buffer, current_position);
    const int64_t chunk_num_y_voxels = chunk_num_y_voxels_deserialized.Value();
    current_position += chunk_num_y_voxels_deserialized.BytesRead();
    const auto chunk_num_z_voxels_deserialized =
        serialization::DeserializeMemcpyable<int64_t>(buffer, current_position);
    const int64_t chunk_num_z_voxels = chunk_num_z_voxels_deserialized.Value();
    current_position += chunk_num_z_voxels_deserialized.BytesRead();

    const Vector3i64 chunk_voxel_counts(
        chunk_num_x_voxels, chunk_num_y_voxels, chunk_num_z_voxels);

    // Start with a default-constructed DynamicSpatialHashedVoxelGridSizes.
    DynamicSpatialHashedVoxelGridSizes temp_sizes;

    // Attempt to initialize from the deserialized values. If any of them are
    // invalid, temp_sizes is unchanged.
    temp_sizes.Initialize(voxel_sizes, chunk_voxel_counts);

    // Figure out how many bytes were read.
    const uint64_t bytes_read = current_position - starting_offset;
    return serialization::MakeDeserialized(temp_sizes, bytes_read);
  }

  static DynamicSpatialHashedVoxelGridSizes FromChunkSizes(
      const Eigen::Vector3d& voxel_sizes, const Eigen::Vector3d& chunk_sizes)
  {
    const Vector3i64 chunk_voxel_counts(
        static_cast<int64_t>(std::ceil(chunk_sizes.x() / voxel_sizes.x())),
        static_cast<int64_t>(std::ceil(chunk_sizes.y() / voxel_sizes.y())),
        static_cast<int64_t>(std::ceil(chunk_sizes.z() / voxel_sizes.z())));
    return DynamicSpatialHashedVoxelGridSizes(voxel_sizes, chunk_voxel_counts);
  }

  static DynamicSpatialHashedVoxelGridSizes FromChunkSizes(
      const double voxel_size, const Eigen::Vector3d& chunk_sizes)
  {
    return FromChunkSizes(
        Eigen::Vector3d(voxel_size, voxel_size, voxel_size), chunk_sizes);
  }

  static DynamicSpatialHashedVoxelGridSizes FromChunkVoxelCounts(
      const Eigen::Vector3d& voxel_sizes, const Vector3i64& chunk_voxel_counts)
  {
    return DynamicSpatialHashedVoxelGridSizes(voxel_sizes, chunk_voxel_counts);
  }

  static DynamicSpatialHashedVoxelGridSizes FromChunkVoxelCounts(
      const double voxel_size, const Vector3i64& chunk_voxel_counts)
  {
    return FromChunkVoxelCounts(
        Eigen::Vector3d(voxel_size, voxel_size, voxel_size),
        chunk_voxel_counts);
  }

  DynamicSpatialHashedVoxelGridSizes() {}

  // This is exposed only for testing.
  bool Initialize(
      const Eigen::Vector3d& voxel_sizes, const Vector3i64& chunk_voxel_counts)
  {
    if (CheckPositiveValid(voxel_sizes.x()) &&
        CheckPositiveValid(voxel_sizes.y()) &&
        CheckPositiveValid(voxel_sizes.z()) &&
        CheckPositiveValid(chunk_voxel_counts.x()) &&
        CheckPositiveValid(chunk_voxel_counts.y()) &&
        CheckPositiveValid(chunk_voxel_counts.z()))
    {
      voxel_sizes_ = Eigen::Vector4d(
          voxel_sizes.x(), voxel_sizes.y(), voxel_sizes.z(), 1.0);
      inverse_voxel_sizes_ = voxel_sizes_.cwiseInverse();
      chunk_voxel_counts_internal_ = Eigen::Vector4d(
          static_cast<double>(chunk_voxel_counts.x()),
          static_cast<double>(chunk_voxel_counts.y()),
          static_cast<double>(chunk_voxel_counts.z()),
          1.0);
      inverse_chunk_voxel_counts_internal_ =
          chunk_voxel_counts_internal_.cwiseInverse();
      chunk_voxel_counts_ = chunk_voxel_counts;
      chunk_num_total_voxels_ = chunk_voxel_counts_.prod();
      chunk_sizes_ =
          voxel_sizes_.cwiseProduct(chunk_voxel_counts_internal_).head<3>();
      chunk_stride_1_ = chunk_voxel_counts_.y() * chunk_voxel_counts_.z();
      chunk_stride_2_ = chunk_voxel_counts_.z();

      return true;
    }
    else
    {
      return false;
    }
  }

  bool IsValid() const { return chunk_stride_2_ > 0; }

  // Accessors for voxel sizes.

  const Eigen::Vector4d& VoxelSizesInternal() const { return voxel_sizes_; }

  Eigen::Vector3d VoxelSizes() const { return voxel_sizes_.head<3>(); }

  double VoxelXSize() const { return voxel_sizes_(0); }

  double VoxelYSize() const { return voxel_sizes_(1); }

  double VoxelZSize() const { return voxel_sizes_(2); }

  bool HasUniformVoxelSize() const
  {
    return voxel_sizes_(0) == voxel_sizes_(1) &&
           voxel_sizes_(0) == voxel_sizes_(2);
  }

  // Accessors for inverse voxel sizes.

  const Eigen::Vector4d& InverseVoxelSizesInternal() const
  {
    return inverse_voxel_sizes_;
  }

  Eigen::Vector3d InverseVoxelSizes() const
  {
    return inverse_voxel_sizes_.head<3>();
  }

  double InverseVoxelXSize() const { return inverse_voxel_sizes_(0); }

  double InverseVoxelYSize() const { return inverse_voxel_sizes_(1); }

  double InverseVoxelZSize() const { return inverse_voxel_sizes_(2); }

  // Accessors for chunk sizes.

  const Eigen::Vector3d& ChunkSizes() const { return chunk_sizes_; }

  double ChunkXSize() const { return chunk_sizes_.x(); }

  double ChunkYSize() const { return chunk_sizes_.y(); }

  double ChunkZSize() const { return chunk_sizes_.z(); }

  // Accessors for chunkvoxel counts.

  const Vector3i64& ChunkVoxelCounts() const { return chunk_voxel_counts_; }

  int64_t ChunkNumXVoxels() const { return chunk_voxel_counts_.x(); }

  int64_t ChunkNumYVoxels() const { return chunk_voxel_counts_.y(); }

  int64_t ChunkNumZVoxels() const { return chunk_voxel_counts_.z(); }

  int64_t ChunkNumTotalVoxels() const { return chunk_num_total_voxels_; }

  // Accessors for internal chunk voxel counts used in SIMD operations.

  const Eigen::Vector4d& ChunkVoxelCountsInternal() const
  {
    return chunk_voxel_counts_internal_;
  }

  const Eigen::Vector4d& InverseChunkVoxelCountsInternal() const
  {
    return inverse_chunk_voxel_counts_internal_;
  }

  // Accessors for chunk strides.

  int64_t ChunkStride1() const { return chunk_stride_1_; }

  int64_t ChunkStride2() const { return chunk_stride_2_; }

  // Index bounds checks.

  bool CheckChunkIndexInBounds(const int64_t x_index,
                               const int64_t y_index,
                               const int64_t z_index) const
  {
    return x_index >= 0 && x_index < ChunkNumXVoxels() &&
           y_index >= 0 && y_index < ChunkNumYVoxels() &&
           z_index >= 0 && z_index < ChunkNumZVoxels();
  }

  bool CheckChunkIndexInBounds(const ChunkIndex& index) const
  {
    return CheckChunkIndexInBounds(index.X(), index.Y(), index.Z());
  }

  bool CheckGridIndexInAllowedBounds(const int64_t x_index,
                                     const int64_t y_index,
                                     const int64_t z_index) const
  {
    return x_index <= MAXIMUM_ALLOWED_DSHVG_INDEX_AXIS_VALUE &&
           x_index >= MINIMUM_ALLOWED_DSHVG_INDEX_AXIS_VALUE &&
           y_index <= MAXIMUM_ALLOWED_DSHVG_INDEX_AXIS_VALUE &&
           y_index >= MINIMUM_ALLOWED_DSHVG_INDEX_AXIS_VALUE &&
           z_index <= MAXIMUM_ALLOWED_DSHVG_INDEX_AXIS_VALUE &&
           z_index >= MINIMUM_ALLOWED_DSHVG_INDEX_AXIS_VALUE;
  }

  bool CheckGridIndexInAllowedBounds(const GridIndex& index) const
  {
    return CheckGridIndexInAllowedBounds(index.X(), index.Y(), index.Z());
  }

  bool CheckChunkDataIndexInBounds(const int64_t data_index) const
  {
    return data_index >= 0 && data_index < ChunkNumTotalVoxels();
  }

  // Chunk index <-> data index conversions.

  int64_t ChunkIndexToDataIndex(const int64_t x_index,
                                const int64_t y_index,
                                const int64_t z_index) const
  {
    if (CheckChunkIndexInBounds(x_index, y_index, z_index))
    {
      return (x_index * ChunkStride1()) + (y_index * ChunkStride2()) + z_index;
    }
    else
    {
      // Return a clearly invalid data index for grid indices out of bounds.
      return std::numeric_limits<int64_t>::lowest();
    }
  }

  int64_t ChunkIndexToDataIndex(const ChunkIndex& index) const
  {
    return ChunkIndexToDataIndex(index.X(), index.Y(), index.Z());
  }

  ChunkIndex DataIndexToChunkIndex(const int64_t data_index) const
  {
    if (CheckChunkDataIndexInBounds(data_index))
    {
      const int64_t x_idx = data_index / ChunkStride1();
      const int64_t remainder = data_index % ChunkStride1();
      const int64_t y_idx = remainder / ChunkStride2();
      const int64_t z_idx = remainder % ChunkStride2();
      return ChunkIndex(x_idx, y_idx, z_idx);
    }
    else
    {
      // Return a default-value (clearly invalid) for data indices out of range.
      return ChunkIndex();
    }
  }

  // Grid-frame location <-> index conversions.

  GridIndex LocationInGridFrameToGridIndex3d(
      const Eigen::Vector3d& location) const
  {
    const Eigen::Vector3d raw_index =
        location.cwiseProduct(InverseVoxelSizes()).array().floor();
    return GridIndex(
        static_cast<int64_t>(raw_index.x()),
        static_cast<int64_t>(raw_index.y()),
        static_cast<int64_t>(raw_index.z()));
  }

  GridIndex LocationInGridFrameToGridIndex4d(
      const Eigen::Vector4d& location) const
  {
    const Eigen::Vector4d raw_index =
        location.cwiseProduct(InverseVoxelSizesInternal()).array().floor();
    return GridIndex(
        static_cast<int64_t>(raw_index(0)),
        static_cast<int64_t>(raw_index(1)),
        static_cast<int64_t>(raw_index(2)));
  }

  GridIndex LocationInGridFrameToGridIndex(
      const double x, const double y, const double z) const
  {
    const Eigen::Vector4d location(x, y, z, 1.0);
    return LocationInGridFrameToGridIndex4d(location);
  }

  Eigen::Vector4d GridIndexToLocationInGridFrame(
      const int64_t x_index, const int64_t y_index, const int64_t z_index) const
  {
    const Eigen::Vector4d voxel_indexes(
        static_cast<double>(x_index) + 0.5,
        static_cast<double>(y_index) + 0.5,
        static_cast<double>(z_index) + 0.5,
        1.0);
    return VoxelSizesInternal().cwiseProduct(voxel_indexes);
  }

  Eigen::Vector4d GridIndexToLocationInGridFrame(const GridIndex& index) const
  {
    return GridIndexToLocationInGridFrame(index.X(), index.Y(), index.Z());
  }

  // Chunk base <-> grid index conversions.

  ChunkBase GridIndexToChunkBase(const int64_t x_index,
                                 const int64_t y_index,
                                 const int64_t z_index) const
  {
    const Eigen::Matrix<int64_t, 4, 1> voxel_indexes(
        x_index, y_index, z_index, 0);

    const Eigen::Vector4d raw_chunk_num =
        voxel_indexes.cast<double>().cwiseProduct(
            InverseChunkVoxelCountsInternal()).array().floor();

    const Eigen::Vector4d raw_chunk_base =
        raw_chunk_num.cwiseProduct(ChunkVoxelCountsInternal());

    return ChunkBase(
        static_cast<int64_t>(raw_chunk_base(0)),
        static_cast<int64_t>(raw_chunk_base(1)),
        static_cast<int64_t>(raw_chunk_base(2)));
  }

  ChunkBase GridIndexToChunkBase(const GridIndex& index) const
  {
    return GridIndexToChunkBase(index.X(), index.Y(), index.Z());
  }

  GridIndex ChunkBaseToGridIndex(const ChunkBase& base) const
  {
    // This conversion is trivial, and is provided for API completeness.
    return GridIndex(base.X(), base.Y(), base.Z());
  }

  bool operator==(const DynamicSpatialHashedVoxelGridSizes& other) const
  {
    return (VoxelSizesInternal().array() ==
                other.VoxelSizesInternal().array()).all() &&
           (ChunkVoxelCounts().array() ==
                other.ChunkVoxelCounts().array()).all();
  }

  bool operator!=(const DynamicSpatialHashedVoxelGridSizes& other) const
  {
    return !(*this == other);
  }
};

static_assert(
    std::is_trivially_destructible<DynamicSpatialHashedVoxelGridSizes>::value,
    "DynamicSpatialHashedVoxelGridSizes must be trivially destructible");

// While this looks like a std::optional<T>, it *does not own* the item of T,
// unlike std::optional<T>, since it needs to pass the caller a const/mutable
// reference to the item in the DSHVG.
template<typename T>
class DynamicSpatialHashedGridQuery
{
private:
  ReferencingMaybe<T> value_;
  AccessStatus status_ = AccessStatus::UNKNOWN;

  // This struct (and its uses) exists to disambiguate between the value-found
  // and status constructors.
  struct AccessStatusSuccess {};

  explicit DynamicSpatialHashedGridQuery(T& value, AccessStatusSuccess)
      : value_(value), status_(AccessStatus::SUCCESS) {}

  explicit DynamicSpatialHashedGridQuery(const AccessStatus status)
      : status_(status)
  {
    if (status_ == AccessStatus::SUCCESS)
    {
      throw std::invalid_argument(
          "DynamicSpatialHashedGridQuery cannot be constructed with "
          "AccessStatus::SUCCESS");
    }
  }

public:
  static DynamicSpatialHashedGridQuery<T> Success(T& value)
  {
    return DynamicSpatialHashedGridQuery<T>(value, AccessStatusSuccess{});
  }

  static DynamicSpatialHashedGridQuery<T> NotFound()
  {
    return DynamicSpatialHashedGridQuery<T>(AccessStatus::NOT_FOUND);
  }

  static DynamicSpatialHashedGridQuery<T> OutOfBounds()
  {
    return DynamicSpatialHashedGridQuery<T>(AccessStatus::OUT_OF_BOUNDS);
  }

  static DynamicSpatialHashedGridQuery<T> MutableAccessProhibited()
  {
    return DynamicSpatialHashedGridQuery<T>(
        AccessStatus::MUTABLE_ACCESS_PROHIBITED);
  }

  DynamicSpatialHashedGridQuery() {}

  DynamicSpatialHashedGridQuery(
      const DynamicSpatialHashedGridQuery<T>& other) = default;

  DynamicSpatialHashedGridQuery(
      DynamicSpatialHashedGridQuery<T>&& other) = default;

  DynamicSpatialHashedGridQuery<T>& operator=(
      const DynamicSpatialHashedGridQuery<T>& other) = default;

  DynamicSpatialHashedGridQuery<T>& operator=(
      DynamicSpatialHashedGridQuery<T>&& other) = default;

  T& Value() { return value_.Value(); }

  T& Value() const { return value_.Value(); }

  AccessStatus Status() const { return status_; }

  bool HasValue() const { return value_.HasValue(); }

  explicit operator bool() const { return HasValue(); }
};

template<typename T, typename BackingStore=std::vector<T>>
class DynamicSpatialHashedVoxelGridChunk
{
private:
  ChunkBase base_;
  BackingStore data_;

  // TODO(calderpg) Replace with data move once serialization redo lands.
  DynamicSpatialHashedVoxelGridChunk(
      const ChunkBase& base, const BackingStore& data)
      : base_(base), data_(data) {}

public:
  static uint64_t Serialize(
      const DynamicSpatialHashedVoxelGridChunk<T, BackingStore>& chunk,
      std::vector<uint8_t>& buffer,
      const serialization::Serializer<T>& value_serializer)
  {
    const uint64_t start_buffer_size = buffer.size();
    // Serialize members
    ChunkBase::Serialize(chunk.Base(), buffer);
    serialization::SerializeVectorLike<T, BackingStore>(
          chunk.data_, buffer, value_serializer);
    // Figure out how many bytes were written
    const uint64_t end_buffer_size = buffer.size();
    const uint64_t bytes_written = end_buffer_size - start_buffer_size;
    return bytes_written;
  }

  static serialization::Deserialized<
      DynamicSpatialHashedVoxelGridChunk<T, BackingStore>> Deserialize(
          const std::vector<uint8_t>& buffer, const uint64_t starting_offset,
          const serialization::Deserializer<T>& value_deserializer)
  {
    uint64_t current_position = starting_offset;
    // Deserialize members
    const auto base_deserialized =
        ChunkBase::Deserialize(buffer, current_position);
    current_position += base_deserialized.BytesRead();
    const auto data_deserialized =
        serialization::DeserializeVectorLike<T, BackingStore>(
              buffer, current_position, value_deserializer);
    current_position += data_deserialized.BytesRead();
    // Figure out how many bytes were read
    const uint64_t bytes_read = current_position - starting_offset;
    return serialization::MakeDeserialized(
        DynamicSpatialHashedVoxelGridChunk<T, BackingStore>(
            base_deserialized.Value(), data_deserialized.Value()),
        bytes_read);
  }

  DynamicSpatialHashedVoxelGridChunk() = default;

  DynamicSpatialHashedVoxelGridChunk(
      const ChunkBase& base, const int64_t num_elements, const T& value)
      : base_(base)
  {
    data_.clear();
    data_.resize(
        static_cast<typename BackingStore::size_type>(num_elements), value);
  }

  void SetContents(const T& value)
  {
    for (auto& voxel_value : data_)
    {
      voxel_value = value;
    }
  }

  const ChunkBase& Base() const { return base_; }

  bool IsInitialized() const { return data_.size() > 0; }

  int64_t NumElements() const { return static_cast<int64_t>(data_.size()); }

  T& AccessIndex(const int64_t& data_index)
  {
    if ((data_index >= 0) && (data_index < NumElements()))
    {
      // Note: do not refactor to use .at(), since not all vector-like
      // implementations implement it (ex thrust::host_vector<T>).
      return data_[static_cast<typename BackingStore::size_type>(data_index)];
    }
    else
    {
      throw std::out_of_range("data_index out of range");
    }
  }

  const T& AccessIndex(const int64_t& data_index) const
  {
    if ((data_index >= 0) && (data_index < NumElements()))
    {
      // Note: do not refactor to use .at(), since not all vector-like
      // implementations implement it (ex thrust::host_vector<T>).
      return data_[static_cast<typename BackingStore::size_type>(data_index)];
    }
    else
    {
      throw std::out_of_range("data_index out of range");
    }
  }
};

template<typename T, typename BackingStore=std::vector<T>>
class DynamicSpatialHashedVoxelGridChunkKeeper
{
private:
  using ChunkType = DynamicSpatialHashedVoxelGridChunk<T, BackingStore>;

  std::mutex get_or_create_chunk_mutex_;
  std::unordered_map<ChunkBase, ChunkType> chunk_map_;

  // TODO(calderpg) Replace with data move once serialization redo lands.
  explicit DynamicSpatialHashedVoxelGridChunkKeeper(
      const std::unordered_map<ChunkBase, ChunkType>& chunk_map)
      : chunk_map_(chunk_map) {}

public:
  static uint64_t Serialize(
      const DynamicSpatialHashedVoxelGridChunkKeeper<T, BackingStore>& keeper,
      std::vector<uint8_t>& buffer,
      const serialization::Serializer<T>& value_serializer)
  {
    const auto chunk_serializer = [&](
        const ChunkType& chunk, std::vector<uint8_t>& serialize_buffer)
    {
      return ChunkType::Serialize(chunk, serialize_buffer, value_serializer);
    };

    const uint64_t start_buffer_size = buffer.size();

    // First, write a uint64_t size header
    const uint64_t size = static_cast<uint64_t>(keeper.chunk_map_.size());
    serialization::SerializeMemcpyable<uint64_t>(size, buffer);

    // Serialize the contained items
    for (auto itr = keeper.begin(); itr != keeper.end(); ++itr)
    {
      serialization::SerializePair<ChunkBase, ChunkType>(
          *itr, buffer, ChunkBase::Serialize, chunk_serializer);
    }

    // Figure out how many bytes were written
    const uint64_t end_buffer_size = buffer.size();
    const uint64_t bytes_written = end_buffer_size - start_buffer_size;
    return bytes_written;
  }

  static serialization::Deserialized<
      DynamicSpatialHashedVoxelGridChunkKeeper<T, BackingStore>>
  Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset,
      const serialization::Deserializer<T>& value_deserializer)
  {
    const auto chunk_deserializer = [&](
        const std::vector<uint8_t>& deserialize_buffer,
        const uint64_t deserialize_starting_offset)
    {
      return ChunkType::Deserialize(
          deserialize_buffer, deserialize_starting_offset, value_deserializer);
    };

    uint64_t current_position = starting_offset;

    // Load the header
    const serialization::Deserialized<uint64_t> deserialized_size =
        serialization::DeserializeMemcpyable<uint64_t>(
            buffer, current_position);
    const uint64_t size = deserialized_size.Value();
    current_position += deserialized_size.BytesRead();

    // Deserialize the items
    std::unordered_map<ChunkBase, ChunkType> chunk_map;
    chunk_map.reserve(size);

    for (uint64_t idx = 0; idx < size; idx++)
    {
      const serialization::Deserialized<std::pair<ChunkBase, ChunkType>>
          deserialized_pair =
              serialization::DeserializePair<ChunkBase, ChunkType>(
                  buffer, current_position, ChunkBase::Deserialize,
                  chunk_deserializer);
      chunk_map.insert(deserialized_pair.Value());
      current_position += deserialized_pair.BytesRead();
    }

    DynamicSpatialHashedVoxelGridChunkKeeper<T, BackingStore> keeper(chunk_map);

    // Figure out how many bytes were read
    const uint64_t bytes_read = current_position - starting_offset;
    return serialization::MakeDeserialized(keeper, bytes_read);
  }

  using iterator =
      typename std::unordered_map<ChunkBase, ChunkType>::iterator;
  using const_iterator =
      typename std::unordered_map<ChunkBase, ChunkType>::const_iterator;

  DynamicSpatialHashedVoxelGridChunkKeeper() = default;

  explicit DynamicSpatialHashedVoxelGridChunkKeeper(
      const size_t expected_chunks)
  {
    chunk_map_.reserve(expected_chunks);
  }

  DynamicSpatialHashedVoxelGridChunkKeeper(
      DynamicSpatialHashedVoxelGridChunkKeeper<T, BackingStore>&& other)
  {
    chunk_map_ = std::move(other.chunk_map_);
  }

  DynamicSpatialHashedVoxelGridChunkKeeper(
      const DynamicSpatialHashedVoxelGridChunkKeeper<T, BackingStore>& other)
  {
    chunk_map_ = other.chunk_map_;
  }

  DynamicSpatialHashedVoxelGridChunkKeeper<T, BackingStore>& operator=(
      const DynamicSpatialHashedVoxelGridChunkKeeper<T, BackingStore>& other)
  {
    if (this != std::addressof(other))
    {
      chunk_map_ = other.chunk_map_;
    }
    return *this;
  }

  DynamicSpatialHashedVoxelGridChunkKeeper<T, BackingStore>& operator=(
      DynamicSpatialHashedVoxelGridChunkKeeper<T, BackingStore>&& other)
  {
    if (this != std::addressof(other))
    {
      chunk_map_ = std::move(other.chunk_map_);
    }
    return *this;
  }

  int64_t NumChunks() const { return static_cast<int64_t>(chunk_map_.size()); }

  ReferencingMaybe<const ChunkType> GetChunkImmutable(
      const ChunkBase& base) const
  {
    const auto& found_itr = chunk_map_.find(base);
    if (found_itr != chunk_map_.end())
    {
      return ReferencingMaybe<const ChunkType>(found_itr->second);
    }
    else
    {
      return ReferencingMaybe<const ChunkType>();
    }
  }

  ReferencingMaybe<ChunkType> GetChunkMutable(const ChunkBase& base)
  {
    auto& found_itr = chunk_map_.find(base);
    if (found_itr != chunk_map_.end())
    {
      return ReferencingMaybe<ChunkType>(found_itr->second);
    }
    else
    {
      return ReferencingMaybe<ChunkType>();
    }
  }

  ReferencingMaybe<ChunkType> GetOrCreateChunkMutable(
      const ChunkBase& base, const int64_t num_elements, const T& default_value)
  {
    std::lock_guard<std::mutex> lock(get_or_create_chunk_mutex_);
    auto found_itr = chunk_map_.find(base);
    if (found_itr != chunk_map_.end())
    {
      return ReferencingMaybe<ChunkType>(found_itr->second);
    }
    else
    {
      ChunkType new_chunk(base, num_elements, default_value);
      auto emplace_result = chunk_map_.emplace(base, std::move(new_chunk));
      auto& emplaced_chunk = emplace_result.first->second;
      return ReferencingMaybe<ChunkType>(emplaced_chunk);
    }
  }

  bool EraseChunk(const ChunkBase& base)
  {
    return chunk_map_.erase(base) > 0;
  }

  iterator EraseChunk(iterator chunk)
  {
    return chunk_map_.erase(chunk);
  }

  iterator EraseChunk(const_iterator chunk)
  {
    return chunk_map_.erase(chunk);
  }

  void EraseAllChunks()
  {
    chunk_map_.clear();
  }

  void SetContentsAllChunks(const T& value)
  {
    for (auto chunk_itr = begin(); chunk_itr != end(); ++chunk_itr)
    {
      chunk_itr->second.SetContents(value);
    }
  }

  const_iterator begin() const { return chunk_map_.begin(); }

  iterator begin() { return chunk_map_.begin(); }

  const_iterator end() const { return chunk_map_.end(); }

  iterator end() { return chunk_map_.end(); }
};

/// This is the base class for all dynamic spatial hashed voxel grid classes.
/// It is pure virtual to force the implementation of certain necessary
/// functions (cloning, access, derived-class memeber de/serialization) in
/// concrete implementations. This is the class to inherit from if you want a
/// DynamicSpatialHashedVoxelGrid-like type. If all you want is a dynamic
/// spatial hashed voxel grid of T, see
/// DynamicSpatialHashedVoxelGrid<T, BackingStore> below.
template<typename T, typename BackingStore=std::vector<T>>
class DynamicSpatialHashedVoxelGridBase
{
private:
  using GridChunk = DynamicSpatialHashedVoxelGridChunk<T, BackingStore>;
  using GridChunkKeeper =
      DynamicSpatialHashedVoxelGridChunkKeeper<T, BackingStore>;

  Eigen::Isometry3d origin_transform_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d inverse_origin_transform_ = Eigen::Isometry3d::Identity();
  T default_value_;
  DynamicSpatialHashedVoxelGridSizes control_sizes_;
  GridChunkKeeper chunk_keeper_;

  void Initialize(
      const Eigen::Isometry3d& origin_transform,
      const DynamicSpatialHashedVoxelGridSizes& control_sizes,
      const T& default_value, const size_t expected_chunks)
  {
    if (control_sizes.IsValid())
    {
      origin_transform_ = origin_transform;
      inverse_origin_transform_ = origin_transform_.inverse();
      control_sizes_ = control_sizes;
      default_value_ = default_value;
      chunk_keeper_ = GridChunkKeeper(expected_chunks);
    }
    else
    {
      throw std::invalid_argument("control_sizes is not valid");
    }
  }

  void Initialize(
      const DynamicSpatialHashedVoxelGridSizes& control_sizes,
      const T& default_value, const size_t expected_chunks)
  {
    const Eigen::Isometry3d origin_transform = Eigen::Isometry3d::Identity();
    Initialize(
        origin_transform, control_sizes, default_value, expected_chunks);
  }

  uint64_t BaseSerializeSelf(
      std::vector<uint8_t>& buffer,
      const serialization::Serializer<T>& value_serializer) const
  {
    const uint64_t start_buffer_size = buffer.size();
    // Serialize the transform
    serialization::SerializeIsometry3d(origin_transform_, buffer);
    // Serialize the default value
    value_serializer(default_value_, buffer);
    // Serialize the chunk sizes
    DynamicSpatialHashedVoxelGridSizes::Serialize(control_sizes_, buffer);
    // Serialize the data
    GridChunkKeeper::Serialize(chunk_keeper_, buffer, value_serializer);
    // Figure out how many bytes were written
    const uint64_t end_buffer_size = buffer.size();
    const uint64_t bytes_written = end_buffer_size - start_buffer_size;
    return bytes_written;
  }

  uint64_t BaseDeserializeSelf(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset,
      const serialization::Deserializer<T>& value_deserializer)
  {
    uint64_t current_position = starting_offset;
    // Deserialize the transforms
    const auto origin_transform_deserialized =
        serialization::DeserializeIsometry3d(buffer, current_position);
    origin_transform_ = origin_transform_deserialized.Value();
    current_position += origin_transform_deserialized.BytesRead();
    inverse_origin_transform_ = origin_transform_.inverse();
    // Deserialize the default value
    const auto default_value_deserialized =
        value_deserializer(buffer, current_position);
    default_value_ = default_value_deserialized.Value();
    current_position += default_value_deserialized.BytesRead();
    // Deserialize the chunk sizes
    const auto control_sizes_deserialized =
        DynamicSpatialHashedVoxelGridSizes::Deserialize(
            buffer, current_position);
    control_sizes_ = control_sizes_deserialized.Value();
    current_position += control_sizes_deserialized.BytesRead();
    // Deserialize the data
    const auto chunk_keeper_deserialized =
        GridChunkKeeper::Deserialize(
            buffer, current_position, value_deserializer);
    chunk_keeper_ = chunk_keeper_deserialized.Value();
    current_position += chunk_keeper_deserialized.BytesRead();
    // Sanity check the chunks.
    if (control_sizes_.IsValid())
    {
      const int64_t num_expected_chunk_voxels =
          control_sizes_.ChunkNumTotalVoxels();

      for (auto chunk_itr = chunk_keeper_.begin();
          chunk_itr != chunk_keeper_.end(); ++chunk_itr)
      {
        const ChunkBase& chunk_base = chunk_itr->first;

        const GridIndex chunk_base_index =
            control_sizes_.ChunkBaseToGridIndex(chunk_base);
        const ChunkBase round_trip_chunk_base =
            control_sizes_.GridIndexToChunkBase(chunk_base_index);
        if (chunk_base != round_trip_chunk_base)
        {
          throw std::runtime_error(
              "Deserialized chunk does not round-trip chunk base");
        }

        const GridChunk& chunk = chunk_itr->second;
        if (chunk.NumElements() != num_expected_chunk_voxels)
        {
          throw std::runtime_error(
              "Deserialized chunk does not have the correct number of voxels");
        }
      }
    }
    else
    {
      if (chunk_keeper_.NumChunks() > 0)
      {
        throw std::runtime_error(
            "Non-empty chunk keeper with invalid voxel grid sizes");
      }
    }
    // Figure out how many bytes were read
    const uint64_t bytes_read = current_position - starting_offset;
    return bytes_read;
  }

  void SetContents(const T& value)
  {
    chunk_keeper_.SetContentsAllChunks(value);
  }

protected:
  // These are pure-virtual in the base class to force their implementation in
  // derived classes.

  /// Do the work necessary for Clone() to copy the current object.
  virtual std::unique_ptr<DynamicSpatialHashedVoxelGridBase<
      T, BackingStore>> DoClone() const = 0;

  /// Serialize any derived-specific members into the provided buffer.
  virtual uint64_t DerivedSerializeSelf(
      std::vector<uint8_t>& buffer,
      const serialization::Serializer<T>& value_serializer) const = 0;

  /// Deserialize any derived-specific members from the provided buffer.
  virtual uint64_t DerivedDeserializeSelf(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset,
      const serialization::Deserializer<T>& value_deserializer) = 0;

  /// Callback on any mutable access to the grid. Return true/false to allow or
  /// disallow access to the grid. For example, this can be used to prohibit
  /// changes to a non-const grid, or to invalidate a cache if voxels are
  /// modified.
  virtual bool OnMutableAccess(const int64_t x_index,
                               const int64_t y_index,
                               const int64_t z_index) = 0;

  bool OnMutableAccess(const GridIndex& index)
  {
    return OnMutableAccess(index.X(), index.Y(), index.Z());
  }

  /// Callback on any mutable access to the grid. Return true/false to allow or
  /// disallow access to the grid. For example, this can be used to prohibit
  /// changes to a non-const grid, or to invalidate a cache if voxels are
  /// modified.
  virtual bool OnMutableRawAccess() = 0;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DynamicSpatialHashedVoxelGridBase(
      const DynamicSpatialHashedVoxelGridSizes& control_sizes,
      const T& default_value, const size_t expected_chunks)
  {
    Initialize(control_sizes, default_value, expected_chunks);
  }

  DynamicSpatialHashedVoxelGridBase(
      const Eigen::Isometry3d& origin_transform,
      const DynamicSpatialHashedVoxelGridSizes& control_sizes,
      const T& default_value, const size_t expected_chunks)
  {
    Initialize(
        origin_transform, control_sizes, default_value, expected_chunks);
  }

  DynamicSpatialHashedVoxelGridBase() = default;

  virtual ~DynamicSpatialHashedVoxelGridBase() {}

  std::unique_ptr<DynamicSpatialHashedVoxelGridBase<T, BackingStore>>
  Clone() const
  {
    return DoClone();
  }

  uint64_t SerializeSelf(
      std::vector<uint8_t>& buffer,
      const serialization::Serializer<T>& value_serializer) const
  {
    return BaseSerializeSelf(buffer, value_serializer)
        + DerivedSerializeSelf(buffer, value_serializer);
  }

  uint64_t DeserializeSelf(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset,
      const serialization::Deserializer<T>& value_deserializer)
  {
    uint64_t current_position = starting_offset;
    current_position
        += BaseDeserializeSelf(buffer, starting_offset, value_deserializer);
    current_position
        += DerivedDeserializeSelf(buffer, current_position, value_deserializer);
    // Figure out how many bytes were read
    const uint64_t bytes_read = current_position - starting_offset;
    return bytes_read;
  }

  bool IsInitialized() const { return control_sizes_.IsValid(); }

  void ResetWithDefaultValue()
  {
    if (OnMutableRawAccess())
    {
      SetContents(DefaultValue());
    }
    else
    {
      throw std::runtime_error("Mutable raw access is prohibited");
    }
  }

  void ResetWithNewValue(const T& new_value)
  {
    if (OnMutableRawAccess())
    {
      SetContents(new_value);
    }
    else
    {
      throw std::runtime_error("Mutable raw access is prohibited");
    }
  }

  void ResetWithNewDefaultValue(const T& new_default)
  {
    if (OnMutableRawAccess())
    {
      SetDefaultValue(new_default);
      ResetWithDefaultValue();
    }
    else
    {
      throw std::runtime_error("Mutable raw access is prohibited");
    }
  }

  const T& DefaultValue() const { return default_value_; }

  void SetDefaultValue(const T& default_value)
  {
    if (OnMutableRawAccess())
    {
      default_value_ = default_value;
    }
    else
    {
      throw std::runtime_error("Mutable raw access is prohibited");
    }
  }

  const DynamicSpatialHashedVoxelGridSizes& ControlSizes() const
  {
    return control_sizes_;
  }

  const Eigen::Isometry3d& OriginTransform() const
  {
    return origin_transform_;
  }

  const Eigen::Isometry3d& InverseOriginTransform() const
  {
    return inverse_origin_transform_;
  }

  void UpdateOriginTransform(const Eigen::Isometry3d& origin_transform)
  {
    origin_transform_ = origin_transform;
    inverse_origin_transform_ = origin_transform_.inverse();
  }

  // Helpers forwarded from our DynamicSpatialHashedVoxelGridSizes.

  // Accessors for voxel sizes.

  Eigen::Vector3d VoxelSizes() const { return control_sizes_.VoxelSizes(); }

  double VoxelXSize() const { return control_sizes_.VoxelXSize(); }

  double VoxelYSize() const { return control_sizes_.VoxelYSize(); }

  double VoxelZSize() const { return control_sizes_.VoxelZSize(); }

  bool HasUniformVoxelSize() const
  {
    return control_sizes_.HasUniformVoxelSize();
  }

  // Accessors for chunk sizes.

  const Eigen::Vector3d& ChunkSizes() const
  {
    return control_sizes_.ChunkSizes();
  }

  double ChunkXSize() const { return control_sizes_.ChunkXSize(); }

  double ChunkYSize() const { return control_sizes_.ChunkYSize(); }

  double ChunkZSize() const { return control_sizes_.ChunkZSize(); }

  // Accessors for chunk voxel counts.

  const Vector3i64& ChunkVoxelCounts() const
  {
    return control_sizes_.ChunkVoxelCounts();
  }

  int64_t ChunkNumXVoxels() const { return control_sizes_.ChunkNumXVoxels(); }

  int64_t ChunkNumYVoxels() const { return control_sizes_.ChunkNumYVoxels(); }

  int64_t ChunkNumZVoxels() const { return control_sizes_.ChunkNumZVoxels(); }

  int64_t ChunkNumTotalVoxels() const
  {
    return control_sizes_.ChunkNumTotalVoxels();
  }

  // Index bounds checks.

  bool CheckChunkIndexInBounds(const int64_t x_index,
                               const int64_t y_index,
                               const int64_t z_index) const
  {
    return control_sizes_.CheckChunkIndexInBounds(x_index, y_index, z_index);
  }

  bool CheckChunkIndexInBounds(const ChunkIndex& index) const
  {
    return control_sizes_.CheckChunkIndexInBounds(index);
  }

  bool CheckGridIndexInAllowedBounds(const int64_t x_index,
                                     const int64_t y_index,
                                     const int64_t z_index) const
  {
    return control_sizes_.CheckGridIndexInAllowedBounds(
        x_index, y_index, z_index);
  }

  bool CheckGridIndexInAllowedBounds(const GridIndex& index) const
  {
    return control_sizes_.CheckGridIndexInAllowedBounds(index);
  }

  bool CheckChunkDataIndexInBounds(const int64_t data_index) const
  {
    return control_sizes_.CheckChunkDataIndexInBounds(data_index);
  }

  // Chunk index <-> data index conversions.

  int64_t ChunkIndexToDataIndex(const int64_t x_index,
                                const int64_t y_index,
                                const int64_t z_index) const
  {
    return control_sizes_.ChunkIndexToDataIndex(x_index, y_index, z_index);
  }

  int64_t ChunkIndexToDataIndex(const ChunkIndex& index) const
  {
    return control_sizes_.ChunkIndexToDataIndex(index);
  }

  ChunkIndex DataIndexToChunkIndex(const int64_t data_index) const
  {
    return control_sizes_.DataIndexToChunkIndex(data_index);
  }

  // Grid-frame location <-> index conversions.

  GridIndex LocationInGridFrameToGridIndex3d(
      const Eigen::Vector3d& location) const
  {
    return control_sizes_.LocationInGridFrameToGridIndex3d(location);
  }

  GridIndex LocationInGridFrameToGridIndex4d(
      const Eigen::Vector4d& location) const
  {
    return control_sizes_.LocationInGridFrameToGridIndex4d(location);
  }

  GridIndex LocationInGridFrameToGridIndex(
      const double x, const double y, const double z) const
  {
    return control_sizes_.LocationInGridFrameToGridIndex(x, y, z);
  }

  Eigen::Vector4d GridIndexToLocationInGridFrame(
      const int64_t x_index, const int64_t y_index, const int64_t z_index) const
  {
    return control_sizes_.GridIndexToLocationInGridFrame(
        x_index, y_index, z_index);
  }

  Eigen::Vector4d GridIndexToLocationInGridFrame(const GridIndex& index) const
  {
    return control_sizes_.GridIndexToLocationInGridFrame(index);
  }

  // Chunk base <-> grid index conversions.

  ChunkBase GridIndexToChunkBase(const int64_t x_index,
                                 const int64_t y_index,
                                 const int64_t z_index) const
  {
    return control_sizes_.GridIndexToChunkBase(x_index, y_index, z_index);
  }

  ChunkBase GridIndexToChunkBase(const GridIndex& index) const
  {
    return control_sizes_.GridIndexToChunkBase(index);
  }

  GridIndex ChunkBaseToGridIndex(const ChunkBase& base) const
  {
    return control_sizes_.ChunkBaseToGridIndex(base);
  }

  // Location <-> grid-frame location conversions.

  Eigen::Vector3d LocationToGridFrameLocation3d(
      const Eigen::Vector3d& location) const
  {
    return InverseOriginTransform() * location;
  }

  Eigen::Vector4d LocationToGridFrameLocation4d(
      const Eigen::Vector4d& location) const
  {
    if (location(3) == 1.0)
    {
      return InverseOriginTransform() * location;
    }
    else
    {
      throw std::invalid_argument("location(3) != 1.0");
    }
  }

  Eigen::Vector4d LocationToGridFrameLocation(
      const double x, const double y, const double z) const
  {
    const Eigen::Vector4d location(x, y, z, 1.0);
    return LocationToGridFrameLocation4d(location);
  }

  Eigen::Vector3d GridFrameLocationToLocation3d(
      const Eigen::Vector3d& location) const
  {
    return OriginTransform() * location;
  }

  Eigen::Vector4d GridFrameLocationToLocation4d(
      const Eigen::Vector4d& location) const
  {
    if (location(3) == 1.0)
    {
      return OriginTransform() * location;
    }
    else
    {
      throw std::invalid_argument("location(3) != 1.0");
    }
  }

  Eigen::Vector4d GridFrameLocationToLocation(
      const double x, const double y, const double z) const
  {
    const Eigen::Vector4d location(x, y, z, 1.0);
    return GridFrameLocationToLocation4d(location);
  }

  // Location <-> grid index conversions.

  GridIndex LocationToGridIndex3d(const Eigen::Vector3d& location) const
  {
    const Eigen::Vector3d grid_frame_location =
        LocationToGridFrameLocation3d(location);
    return LocationInGridFrameToGridIndex3d(grid_frame_location);
  }

  GridIndex LocationToGridIndex4d(const Eigen::Vector4d& location) const
  {
    const Eigen::Vector4d grid_frame_location =
        LocationToGridFrameLocation4d(location);
    return LocationInGridFrameToGridIndex4d(grid_frame_location);
  }

  GridIndex LocationToGridIndex(
      const double x, const double y, const double z) const
  {
    const Eigen::Vector4d grid_frame_location =
        LocationToGridFrameLocation(x, y, z);
    return LocationInGridFrameToGridIndex4d(grid_frame_location);
  }

  Eigen::Vector4d GridIndexToLocation(
      const int64_t x_index, const int64_t y_index, const int64_t z_index) const
  {
    const Eigen::Vector4d grid_frame_location =
        GridIndexToLocationInGridFrame(x_index, y_index, z_index);
    return GridFrameLocationToLocation4d(grid_frame_location);
  }

  Eigen::Vector4d GridIndexToLocation(const GridIndex& index) const
  {
    const Eigen::Vector4d grid_frame_location =
        GridIndexToLocationInGridFrame(index);
    return GridFrameLocationToLocation4d(grid_frame_location);
  }

  // Immutable location-based queries.

  DynamicSpatialHashedGridQuery<const T> GetLocationImmutable3d(
      const Eigen::Vector3d& location) const
  {
    return GetIndexImmutable(LocationToGridIndex3d(location));
  }

  DynamicSpatialHashedGridQuery<const T> GetLocationImmutable4d(
      const Eigen::Vector4d& location) const
  {
    return GetIndexImmutable(LocationToGridIndex4d(location));
  }

  DynamicSpatialHashedGridQuery<const T> GetLocationImmutable(
      const double x, const double y, const double z) const
  {
    return GetIndexImmutable(LocationToGridIndex(x, y, z));
  }

  // Immutable index-based queries.

  DynamicSpatialHashedGridQuery<const T> GetIndexImmutable(
      const GridIndex& index) const
  {
    if (CheckGridIndexInAllowedBounds(index))
    {
      const ChunkBase chunk_base = GridIndexToChunkBase(index);
      const auto maybe_chunk = chunk_keeper_.GetChunkImmutable(chunk_base);

      if (maybe_chunk)
      {
        const ChunkIndex index_in_chunk = index - chunk_base;
        const int64_t chunk_data_index = ChunkIndexToDataIndex(index_in_chunk);
        return DynamicSpatialHashedGridQuery<const T>::Success(
            maybe_chunk.Value().AccessIndex(chunk_data_index));
      }
      else
      {
        return DynamicSpatialHashedGridQuery<const T>::NotFound();
      }
    }
    else
    {
      return DynamicSpatialHashedGridQuery<const T>::OutOfBounds();
    }
  }

  DynamicSpatialHashedGridQuery<const T> GetIndexImmutable(
      const int64_t x_index, const int64_t y_index, const int64_t z_index) const
  {
    return GetIndexImmutable(GridIndex(x_index, y_index, z_index));
  }

  // Mutable location-based queries.

  DynamicSpatialHashedGridQuery<T> GetLocationMutable3d(
      const Eigen::Vector3d& location)
  {
    return GetIndexMutable(LocationToGridIndex3d(location));
  }

  DynamicSpatialHashedGridQuery<T> GetLocationMutable4d(
      const Eigen::Vector4d& location)
  {
    return GetIndexMutable(LocationToGridIndex4d(location));
  }

  DynamicSpatialHashedGridQuery<T> GetLocationMutable(
      const double x, const double y, const double z)
  {
    return GetIndexMutable(LocationToGridIndex(x, y, z));
  }

  DynamicSpatialHashedGridQuery<T> GetOrCreateLocationMutable3d(
      const Eigen::Vector3d& location)
  {
    return GetOrCreateIndexMutable(LocationToGridIndex3d(location));
  }

  DynamicSpatialHashedGridQuery<T> GetOrCreateLocationMutable4d(
      const Eigen::Vector4d& location)
  {
    return GetOrCreateIndexMutable(LocationToGridIndex4d(location));
  }

  DynamicSpatialHashedGridQuery<T> GetOrCreateLocationMutable(
      const double x, const double y, const double z)
  {
    return GetOrCreateIndexMutable(LocationToGridIndex(x, y, z));
  }

  // Mutable index-based queries.

  DynamicSpatialHashedGridQuery<T> GetIndexMutable(const GridIndex& index)
  {
    if (CheckGridIndexInAllowedBounds(index))
    {
      if (OnMutableAccess(index))
      {
        const ChunkBase chunk_base = GridIndexToChunkBase(index);
        auto maybe_chunk = chunk_keeper_.GetChunkMutable(chunk_base);

        if (maybe_chunk)
        {
          const ChunkIndex index_in_chunk = index - chunk_base;
          const int64_t chunk_data_index =
              ChunkIndexToDataIndex(index_in_chunk);
          return DynamicSpatialHashedGridQuery<T>::Success(
              maybe_chunk.Value().AccessIndex(chunk_data_index));
        }
        else
        {
          return DynamicSpatialHashedGridQuery<T>::NotFound();
        }
      }
      else
      {
        return DynamicSpatialHashedGridQuery<T>::MutableAccessProhibited();
      }
    }
    else
    {
      return DynamicSpatialHashedGridQuery<T>::OutOfBounds();
    }
  }

  DynamicSpatialHashedGridQuery<T> GetIndexMutable(
      const int64_t x_index, const int64_t y_index, const int64_t z_index)
  {
    return GetIndexMutable(GridIndex(x_index, y_index, z_index));
  }

  DynamicSpatialHashedGridQuery<T> GetOrCreateIndexMutable(
      const GridIndex& index)
  {
    if (CheckGridIndexInAllowedBounds(index))
    {
      if (OnMutableAccess(index))
      {
        const ChunkBase chunk_base = GridIndexToChunkBase(index);
        auto& chunk = chunk_keeper_.GetOrCreateChunkMutable(
            chunk_base, ChunkNumTotalVoxels(), DefaultValue()).Value();
        const ChunkIndex index_in_chunk = index - chunk_base;
        const int64_t chunk_data_index = ChunkIndexToDataIndex(index_in_chunk);
        return DynamicSpatialHashedGridQuery<T>::Success(
            chunk.AccessIndex(chunk_data_index));
      }
      else
      {
        return DynamicSpatialHashedGridQuery<T>::MutableAccessProhibited();
      }
    }
    else
    {
      return DynamicSpatialHashedGridQuery<T>::OutOfBounds();
    }
  }

  DynamicSpatialHashedGridQuery<T> GetOrCreateIndexMutable(
      const int64_t x_index, const int64_t y_index, const int64_t z_index)
  {
    return GetOrCreateIndexMutable(GridIndex(x_index, y_index, z_index));
  }

  // Location-based setters.

  AccessStatus SetLocation3d(const Eigen::Vector3d& location, const T& value)
  {
    return SetIndex(LocationToGridIndex3d(location), value);
  }

  AccessStatus SetLocation4d(const Eigen::Vector4d& location, const T& value)
  {
    return SetIndex(LocationToGridIndex4d(location), value);
  }

  AccessStatus SetLocation(
      const double x, const double y, const double z, const T& value)
  {
    return SetIndex(LocationToGridIndex(x, y, z), value);
  }

  // Index-based setters.

  AccessStatus SetIndex(const GridIndex& index, const T& value)
  {
    if (CheckGridIndexInAllowedBounds(index))
    {
      if (OnMutableAccess(index))
      {
        const ChunkBase chunk_base = GridIndexToChunkBase(index);
        auto& chunk = chunk_keeper_.GetOrCreateChunkMutable(
            chunk_base, ChunkNumTotalVoxels(), DefaultValue()).Value();
        const ChunkIndex index_in_chunk = index - chunk_base;
        const int64_t chunk_data_index = ChunkIndexToDataIndex(index_in_chunk);
        chunk.AccessIndex(chunk_data_index) = value;
        return AccessStatus::SUCCESS;
      }
      else
      {
        return AccessStatus::MUTABLE_ACCESS_PROHIBITED;
      }
    }
    else
    {
      return AccessStatus::OUT_OF_BOUNDS;
    }
  }

  AccessStatus SetIndex(
      const int64_t x_index, const int64_t y_index, const int64_t z_index,
      const T& value)
  {
    return SetIndex(GridIndex(x_index, y_index, z_index), value);
  }

  // Location-based setters (for temporary values).

  AccessStatus SetLocation3d(const Eigen::Vector3d& location, T&& value)
  {
    return SetIndex(LocationToGridIndex3d(location), value);
  }

  AccessStatus SetLocation4d(const Eigen::Vector4d& location, T&& value)
  {
    return SetIndex(LocationToGridIndex4d(location), value);
  }

  AccessStatus SetLocation(
      const double x, const double y, const double z, T&& value)
  {
    return SetIndex(LocationToGridIndex(x, y, z), value);
  }

  // Index-based setters (for temporary values).

  AccessStatus SetIndex(const GridIndex& index, T&& value)
  {
    if (CheckGridIndexInAllowedBounds(index))
    {
      if (OnMutableAccess(index))
      {
        const ChunkBase chunk_base = GridIndexToChunkBase(index);
        auto& chunk = chunk_keeper_.GetOrCreateChunkMutable(
            chunk_base, ChunkNumTotalVoxels(), DefaultValue()).Value();
        const ChunkIndex index_in_chunk = index - chunk_base;
        const int64_t chunk_data_index = ChunkIndexToDataIndex(index_in_chunk);
        chunk.AccessIndex(chunk_data_index) = value;
        return AccessStatus::SUCCESS;
      }
      else
      {
        return AccessStatus::MUTABLE_ACCESS_PROHIBITED;
      }
    }
    else
    {
      return AccessStatus::OUT_OF_BOUNDS;
    }
  }

  AccessStatus SetIndex(
      const int64_t x_index, const int64_t y_index, const int64_t z_index,
      T&& value)
  {
    return SetIndex(GridIndex(x_index, y_index, z_index), std::move(value));
  }

  // Erase operations.

  AccessStatus EraseChunkContainingLocation3d(const Eigen::Vector3d& location)
  {
    return EraseChunkContainingIndex(LocationToGridIndex3d(location));
  }

  AccessStatus EraseChunkContainingLocation4d(const Eigen::Vector3d& location)
  {
    return EraseChunkContainingIndex(LocationToGridIndex4d(location));
  }

  AccessStatus EraseChunkContainingLocation(
      const double x, const double y, const double z)
  {
    return EraseChunkContainingIndex(LocationToGridIndex(x, y, z));
  }

  AccessStatus EraseChunkContainingIndex(const GridIndex& index)
  {
    if (CheckGridIndexInAllowedBounds(index))
    {
      if (OnMutableAccess(index))
      {
        const ChunkBase chunk_base = GridIndexToChunkBase(index);
        const bool erased = chunk_keeper_.EraseChunk(chunk_base);

        if (erased)
        {
          return AccessStatus::SUCCESS;
        }
        else
        {
          return AccessStatus::NOT_FOUND;
        }
      }
      else
      {
        return AccessStatus::MUTABLE_ACCESS_PROHIBITED;
      }
    }
    else
    {
      return AccessStatus::OUT_OF_BOUNDS;
    }
  }

  AccessStatus EraseChunkContainingIndex(
      const int64_t x_index, const int64_t y_index, const int64_t z_index)
  {
    return EraseChunkContainingIndex(GridIndex(x_index, y_index, z_index));
  }

  AccessStatus EraseChunk(const ChunkBase& chunk_base)
  {
    const GridIndex index = ChunkBaseToGridIndex(chunk_base);

    if (CheckGridIndexInAllowedBounds(index))
    {
      if (OnMutableAccess(index))
      {
        const bool erased = chunk_keeper_.EraseChunk(chunk_base);

        if (erased)
        {
          return AccessStatus::SUCCESS;
        }
        else
        {
          return AccessStatus::NOT_FOUND;
        }
      }
      else
      {
        return AccessStatus::MUTABLE_ACCESS_PROHIBITED;
      }
    }
    else
    {
      return AccessStatus::OUT_OF_BOUNDS;
    }
  }

  AccessStatus EraseAllChunks()
  {
    if (OnMutableRawAccess())
    {
      chunk_keeper_.EraseAllChunks();
      return AccessStatus::SUCCESS;
    }
    else
    {
      return AccessStatus::MUTABLE_ACCESS_PROHIBITED;
    }
  }

  int64_t NumChunks() const { return chunk_keeper_.NumChunks(); }

  const GridChunkKeeper& GetImmutableInternalChunkKeeper() const
  {
    return chunk_keeper_;
  }

  GridChunkKeeper& GetMutableInternalChunkKeeper() const
  {
    if (OnMutableRawAccess())
    {
      return chunk_keeper_;
    }
    else
    {
      throw std::runtime_error("Mutable raw access is prohibited");
    }
  }
};

/// If you want a DynamicSpatialHashedVoxelGrid<T> this is the class to use.
/// Since you should never inherit from it, this class is final.
template<typename T, typename BackingStore=std::vector<T>>
class DynamicSpatialHashedVoxelGrid final
    : public DynamicSpatialHashedVoxelGridBase<T, BackingStore>
{
private:
  std::unique_ptr<DynamicSpatialHashedVoxelGridBase<T, BackingStore>>
  DoClone() const override
  {
    return std::unique_ptr<DynamicSpatialHashedVoxelGrid<T, BackingStore>>(
        new DynamicSpatialHashedVoxelGrid<T, BackingStore>(*this));
  }

  uint64_t DerivedSerializeSelf(
      std::vector<uint8_t>& buffer,
      const serialization::Serializer<T>& value_serializer) const override
  {
    CRU_UNUSED(buffer);
    CRU_UNUSED(value_serializer);
    return 0;
  }

  uint64_t DerivedDeserializeSelf(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset,
      const serialization::Deserializer<T>& value_deserializer) override
  {
    CRU_UNUSED(buffer);
    CRU_UNUSED(starting_offset);
    CRU_UNUSED(value_deserializer);
    return 0;
  }

  bool OnMutableAccess(const int64_t x_index,
                       const int64_t y_index,
                       const int64_t z_index) override
  {
    CRU_UNUSED(x_index);
    CRU_UNUSED(y_index);
    CRU_UNUSED(z_index);
    return true;
  }

  bool OnMutableRawAccess() override { return true; }

public:
  static uint64_t Serialize(
      const DynamicSpatialHashedVoxelGrid<T, BackingStore>& grid,
      std::vector<uint8_t>& buffer,
      const serialization::Serializer<T>& value_serializer)
  {
    return grid.SerializeSelf(buffer, value_serializer);
  }

  static serialization::Deserialized<
      DynamicSpatialHashedVoxelGrid<T, BackingStore>> Deserialize(
          const std::vector<uint8_t>& buffer, const uint64_t starting_offset,
          const serialization::Deserializer<T>& value_deserializer)
  {
    DynamicSpatialHashedVoxelGrid<T, BackingStore> temp_grid;
    const uint64_t bytes_read = temp_grid.DeserializeSelf(
        buffer, starting_offset, value_deserializer);
    return serialization::MakeDeserialized(temp_grid, bytes_read);
  }

  DynamicSpatialHashedVoxelGrid(
      const DynamicSpatialHashedVoxelGridSizes& control_sizes,
      const T& default_value, const size_t expected_chunks)
      : DynamicSpatialHashedVoxelGridBase<T, BackingStore>(
          control_sizes, default_value, expected_chunks) {}

  DynamicSpatialHashedVoxelGrid(
      const Eigen::Isometry3d& origin_transform,
      const DynamicSpatialHashedVoxelGridSizes& control_sizes,
      const T& default_value, const size_t expected_chunks)
      : DynamicSpatialHashedVoxelGridBase<T, BackingStore>(
          origin_transform, control_sizes, default_value, expected_chunks) {}

  DynamicSpatialHashedVoxelGrid()
      : DynamicSpatialHashedVoxelGridBase<T, BackingStore>() {}
};
}  // namespace voxel_grid
CRU_NAMESPACE_END
}  // namespace common_robotics_utilities

namespace std
{
  template <>
  struct hash<common_robotics_utilities::voxel_grid::ChunkBase>
  {
    std::size_t operator()(
        const common_robotics_utilities::voxel_grid::ChunkBase& base) const
    {
      std::size_t hash_val = 0;
      common_robotics_utilities::utility::hash_combine(
          hash_val, base.X(), base.Y(), base.Z());
      return hash_val;
    }
  };
}
