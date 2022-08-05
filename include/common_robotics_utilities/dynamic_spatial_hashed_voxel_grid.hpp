#pragma once

#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/maybe.hpp>
#include <common_robotics_utilities/serialization.hpp>
#include <common_robotics_utilities/utility.hpp>
#include <common_robotics_utilities/voxel_grid.hpp>

namespace common_robotics_utilities
{
namespace voxel_grid
{
class ChunkRegion
{
private:
  Eigen::Vector4d base_ = Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static uint64_t Serialize(
      const ChunkRegion& region, std::vector<uint8_t>& buffer)
  {
    return serialization::SerializeVector4d(region.Base(), buffer);
  }

  static serialization::Deserialized<ChunkRegion> Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
  {
    const auto base_deserialized
        = serialization::DeserializeVector4d(buffer, starting_offset);
    return serialization::MakeDeserialized(
        ChunkRegion(base_deserialized.Value()), base_deserialized.BytesRead());
  }

  ChunkRegion() : base_(Eigen::Vector4d(0.0, 0.0, 0.0, 1.0)) {}

  ChunkRegion(const double base_x, const double base_y, const double base_z)
    : base_(Eigen::Vector4d(base_x, base_y, base_z, 1.0)) {}

  ChunkRegion(const Eigen::Vector3d& base)
    : base_(Eigen::Vector4d(base.x(), base.y(), base.z(), 1.0)) {}

  ChunkRegion(const Eigen::Vector4d& base) : base_(base)
  {
    if (base_(3) != 1.0)
    {
      throw std::invalid_argument("base(3) != 1");
    }
  }

  const Eigen::Vector4d& Base() const { return base_; }

  bool operator==(const ChunkRegion& other) const
  {
    if (Base()(0) == other.Base()(0) && Base()(1) == other.Base()(1) &&
        Base()(2) == other.Base()(2) && Base()(3) == other.Base()(3))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
};

/// Enums to help us out.
enum class DSHVGFillType : uint8_t {FILL_CHUNK, FILL_CELL};

enum class DSHVGFillStatus : uint8_t {NOT_FILLED, CHUNK_FILLED, CELL_FILLED};

enum class DSHVGFoundStatus : uint8_t
    {NOT_FOUND, FOUND_IN_CHUNK, FOUND_IN_CELL, MUTABLE_ACCESS_PROHIBITED};

enum class DSHVGSetType : uint8_t {SET_CHUNK, SET_CELL};

enum class DSHVGSetStatus : uint8_t
    {MUTABLE_ACCESS_PROHIBITED, SET_CHUNK, SET_CELL};

template<typename T>
class DynamicSpatialHashedGridQuery
{
private:
  ReferencingMaybe<T> value_;
  DSHVGFoundStatus found_status_ = DSHVGFoundStatus::NOT_FOUND;

  DynamicSpatialHashedGridQuery(T& value, const DSHVGFoundStatus found_status)
      : value_(value), found_status_(found_status)
  {
    if (FoundStatus() == DSHVGFoundStatus::NOT_FOUND &&
        FoundStatus() == DSHVGFoundStatus::MUTABLE_ACCESS_PROHIBITED)
    {
      throw std::invalid_argument(
          "Cannot return value with NOT_FOUND or MUTABLE_ACCESS_PROHIBITED");
    }
  }

  DynamicSpatialHashedGridQuery(const DSHVGFoundStatus found_status)
      : found_status_(found_status)
  {
    if (FoundStatus() == DSHVGFoundStatus::FOUND_IN_CHUNK ||
        FoundStatus() == DSHVGFoundStatus::FOUND_IN_CELL)
    {
      throw std::invalid_argument(
          "Cannot return FOUND_IN_CHUNK or FOUND_IN_CELL without value");
    }
  }

public:
  static DynamicSpatialHashedGridQuery<T> FoundInChunk(T& value)
  {
    return DynamicSpatialHashedGridQuery<T>(
        value, DSHVGFoundStatus::FOUND_IN_CHUNK);
  }

  static DynamicSpatialHashedGridQuery<T> FoundInCell(T& value)
  {
    return DynamicSpatialHashedGridQuery<T>(
        value, DSHVGFoundStatus::FOUND_IN_CELL);
  }

  static DynamicSpatialHashedGridQuery<T> NotFound()
  {
    return DynamicSpatialHashedGridQuery<T>(DSHVGFoundStatus::NOT_FOUND);
  }

  static DynamicSpatialHashedGridQuery<T> MutableAccessProhibited()
  {
    return DynamicSpatialHashedGridQuery<T>(
        DSHVGFoundStatus::MUTABLE_ACCESS_PROHIBITED);
  }

  DynamicSpatialHashedGridQuery()
      : found_status_(DSHVGFoundStatus::NOT_FOUND) {}

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

  DSHVGFoundStatus FoundStatus() const { return found_status_; }

  bool HasValue() const { return value_.HasValue(); }

  explicit operator bool() const { return HasValue(); }
};

template<typename T, typename BackingStore=std::vector<T>>
class DynamicSpatialHashedVoxelGridChunk
{
private:
  ChunkRegion region_;
  BackingStore data_;
  GridSizes sizes_;
  DSHVGFillStatus fill_status_ = DSHVGFillStatus::NOT_FILLED;

  int64_t GetLocationDataIndex(const Eigen::Vector4d& location) const
  {
    const Eigen::Vector4d location_wrt_chunk = location - region_.Base();
    // First, make sure the location is in the range this chunk covers
    if (location_wrt_chunk(0) < 0.0
        || location_wrt_chunk(1) < 0.0
        || location_wrt_chunk(2) < 0.0
        || location_wrt_chunk(0) >= sizes_.XSize()
        || location_wrt_chunk(1) >= sizes_.YSize()
        || location_wrt_chunk(2) >= sizes_.ZSize())
    {
      return -1;
    }
    // Ok, we're inside the chunk
    else
    {
      const int64_t x_cell = static_cast<int64_t>(
          std::floor(location_wrt_chunk(0) / sizes_.CellXSize()));
      const int64_t y_cell = static_cast<int64_t>(
          std::floor(location_wrt_chunk(1) / sizes_.CellYSize()));
      const int64_t z_cell = static_cast<int64_t>(
          std::floor(location_wrt_chunk(2) / sizes_.CellZSize()));
      if (x_cell < 0 || y_cell < 0 || z_cell < 0 || x_cell >= sizes_.NumXCells()
          || y_cell >= sizes_.NumYCells() || z_cell >= sizes_.NumZCells())
      {
        return -1;
      }
      else
      {
        return sizes_.GetDataIndex(x_cell, y_cell, z_cell);
      }
    }
  }

  T& AccessIndex(const int64_t& data_index)
  {
    if ((data_index >= 0) && (data_index < static_cast<int64_t>(data_.size())))
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
    if ((data_index >= 0) && (data_index < static_cast<int64_t>(data_.size())))
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

  void SetCellFilledContents(const T& value)
  {
    data_.clear();
    data_.resize(static_cast<typename BackingStore::size_type>(
        sizes_.TotalCells()), value);
    fill_status_ = DSHVGFillStatus::CELL_FILLED;
  }

  void SetChunkFilledContents(const T& value)
  {
    data_.clear();
    data_.resize(1, value);
    fill_status_ = DSHVGFillStatus::CHUNK_FILLED;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static uint64_t Serialize(
      const DynamicSpatialHashedVoxelGridChunk<T, BackingStore>& chunk,
      std::vector<uint8_t>& buffer,
      const serialization::Serializer<T>& value_serializer)
  {
    return chunk.SerializeSelf(buffer, value_serializer);
  }

  static serialization::Deserialized<
      DynamicSpatialHashedVoxelGridChunk<T, BackingStore>> Deserialize(
          const std::vector<uint8_t>& buffer, const uint64_t starting_offset,
          const serialization::Deserializer<T>& value_deserializer)
  {
    DynamicSpatialHashedVoxelGridChunk<T, BackingStore> temp_chunk;
    const uint64_t bytes_read
        = temp_chunk.DeserializeSelf(buffer, starting_offset,
                                    value_deserializer);
    return serialization::MakeDeserialized(temp_chunk, bytes_read);
  }

  DynamicSpatialHashedVoxelGridChunk(const ChunkRegion& region,
                                     const GridSizes& sizes,
                                     const DSHVGFillType fill_type,
                                     const T& initial_value)
      : region_(region), sizes_(sizes)
  {
    if (sizes_.Valid())
    {
      if (fill_type == DSHVGFillType::FILL_CELL)
      {
        SetCellFilledContents(initial_value);
      }
      else if (fill_type == DSHVGFillType::FILL_CHUNK)
      {
        SetChunkFilledContents(initial_value);
      }
      else
      {
        throw std::invalid_argument("Invalid fill_type");
      }
    }
    else
    {
      throw std::invalid_argument("sizes is not valid");
    }
  }

  DynamicSpatialHashedVoxelGridChunk() = default;

  uint64_t SerializeSelf(
      std::vector<uint8_t>& buffer,
      const serialization::Serializer<T>& value_serializer) const
  {
    const uint64_t start_buffer_size = buffer.size();
    // Serialize the transform
    ChunkRegion::Serialize(region_, buffer);
    // Serialize the grid sizes
    GridSizes::Serialize(sizes_, buffer);
    // Serialize the data
    serialization::SerializeVectorLike<T, BackingStore>(
          data_, buffer, value_serializer);
    // Serialize the fill status
    serialization::SerializeMemcpyable<uint8_t>(
        static_cast<uint8_t>(fill_status_), buffer);
    // Figure out how many bytes were written
    const uint64_t end_buffer_size = buffer.size();
    const uint64_t bytes_written = end_buffer_size - start_buffer_size;
    return bytes_written;
  }

  uint64_t DeserializeSelf(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset,
      const serialization::Deserializer<T>& value_deserializer)
  {
    uint64_t current_position = starting_offset;
    // Deserialize the transforms
    const auto region_deserialized
        = ChunkRegion::Deserialize(buffer, current_position);
    region_ = region_deserialized.Value();
    current_position += region_deserialized.BytesRead();
    // Deserialize the cell sizes
    const auto sizes_deserialized
        = GridSizes::Deserialize(buffer, current_position);
    sizes_ = sizes_deserialized.Value();
    current_position += sizes_deserialized.BytesRead();
    // Deserialize the data
    const auto data_deserialized
        = serialization::DeserializeVectorLike<T, BackingStore>(
              buffer, current_position, value_deserializer);
    data_ = data_deserialized.Value();
    current_position += data_deserialized.BytesRead();
    // Deserialize the fill status
    const auto fill_status_deserialized
        = serialization::DeserializeMemcpyable<uint8_t>(buffer,
                                                        current_position);
    fill_status_ =
        static_cast<DSHVGFillStatus>(fill_status_deserialized.Value());
    current_position += fill_status_deserialized.BytesRead();
    // Safety checks
    if (fill_status_ == DSHVGFillStatus::CELL_FILLED)
    {
      if (static_cast<int64_t>(data_.size()) != sizes_.TotalCells())
      {
        throw std::runtime_error("sizes_.NumCells() != data_.size()");
      }
    }
    else if (fill_status_ == DSHVGFillStatus::CHUNK_FILLED)
    {
      if (data_.size() != 1)
      {
        throw std::runtime_error("sizes_.NumCells() != 1");
      }
    }
    else
    {
      if (data_.size() != 0)
      {
        throw std::runtime_error("sizes_.NumCells() != 0");
      }
    }
    // Figure out how many bytes were read
    const uint64_t bytes_read = current_position - starting_offset;
    return bytes_read;
  }

  DSHVGFillStatus FillStatus() const { return fill_status_; }

  DynamicSpatialHashedGridQuery<const T> GetIndexImmutable(
      const GridIndex& internal_cell_index) const
  {
    if (fill_status_ == DSHVGFillStatus::CELL_FILLED)
    {
      if (sizes_.IndexInBounds(internal_cell_index))
      {
        return DynamicSpatialHashedGridQuery<const T>::FoundInCell(
            AccessIndex(sizes_.GetDataIndex(internal_cell_index)));
      }
      else
      {
        throw std::runtime_error("internal_cell_index not in chunk");
      }
    }
    else if (fill_status_ == DSHVGFillStatus::CHUNK_FILLED)
    {
      if (internal_cell_index == GridIndex(0, 0, 0))
      {
        return DynamicSpatialHashedGridQuery<const T>::FoundInChunk(
            AccessIndex(0));
      }
      else
      {
        throw std::runtime_error("internal_cell_index not in chunk");
      }
    }
    else
    {
      throw std::runtime_error("Chunk is not filled");
    }
  }

  DynamicSpatialHashedGridQuery<T> GetIndexMutable(
      const GridIndex& internal_cell_index) const
  {
    if (fill_status_ == DSHVGFillStatus::CELL_FILLED)
    {
      if (sizes_.IndexInBounds(internal_cell_index))
      {
        return DynamicSpatialHashedGridQuery<T>::FoundInCell(
            AccessIndex(sizes_.GetDataIndex(internal_cell_index)));
      }
      else
      {
        throw std::runtime_error("internal_cell_index not in chunk");
      }
    }
    else if (fill_status_ == DSHVGFillStatus::CHUNK_FILLED)
    {
      if (internal_cell_index == GridIndex(0, 0, 0))
      {
        return DynamicSpatialHashedGridQuery<T>::FoundInChunk(AccessIndex(0));
      }
      else
      {
        throw std::runtime_error("internal_cell_index not in chunk");
      }
    }
    else
    {
      throw std::runtime_error("Chunk is not filled");
    }
  }

  DynamicSpatialHashedGridQuery<const T> GetLocationImmutable4d(
      const Eigen::Vector4d& location) const
  {
    if (fill_status_ == DSHVGFillStatus::CELL_FILLED)
    {
      const int64_t data_index = GetLocationDataIndex(location);
      if (data_index >= 0)
      {
        return DynamicSpatialHashedGridQuery<const T>::FoundInCell(
            AccessIndex(data_index));
      }
      else
      {
        throw std::runtime_error("location not in chunk");
      }
    }
    else if (fill_status_ == DSHVGFillStatus::CHUNK_FILLED)
    {
      return DynamicSpatialHashedGridQuery<const T>::FoundInChunk(
          AccessIndex(0));
    }
    else
    {
      throw std::runtime_error("Chunk is not filled");
    }
  }

  DynamicSpatialHashedGridQuery<T> GetLocationMutable4d(
      const Eigen::Vector4d& location)
  {
    if (fill_status_ == DSHVGFillStatus::CELL_FILLED)
    {
      const int64_t data_index = GetLocationDataIndex(location);
      if (data_index >= 0)
      {
        return DynamicSpatialHashedGridQuery<T>::FoundInCell(
            AccessIndex(data_index));
      }
      else
      {
        throw std::runtime_error("location not in chunk");
      }
    }
    else if (fill_status_ == DSHVGFillStatus::CHUNK_FILLED)
    {
      return DynamicSpatialHashedGridQuery<T>::FoundInChunk(AccessIndex(0));
    }
    else
    {
      throw std::runtime_error("Chunk is not filled");
    }
  }

  DSHVGSetStatus SetCellLocation4d(
      const Eigen::Vector4d& location, const T& value)
  {
    if (fill_status_ == DSHVGFillStatus::CHUNK_FILLED)
    {
      const T initial_value = AccessIndex(0);
      SetCellFilledContents(initial_value);
      fill_status_ = DSHVGFillStatus::CELL_FILLED;
    }
    if (fill_status_ == DSHVGFillStatus::CELL_FILLED)
    {
      const int64_t data_index = GetLocationDataIndex(location);
      if (data_index >= 0)
      {
        AccessIndex(data_index) = value;
        return DSHVGSetStatus::SET_CELL;
      }
      else
      {
        throw std::runtime_error("location not in chunk");
      }
    }
    else
    {
      throw std::runtime_error("Cannot cell set unfilled chunk");
    }
  }

  DSHVGSetStatus SetCellLocation4d(
      const Eigen::Vector4d& location, T&& value)
  {
    if (fill_status_ == DSHVGFillStatus::CHUNK_FILLED)
    {
      const T initial_value = AccessIndex(0);
      SetCellFilledContents(initial_value);
      fill_status_ = DSHVGFillStatus::CELL_FILLED;
    }
    if (fill_status_ == DSHVGFillStatus::CELL_FILLED)
    {
      const int64_t data_index = GetLocationDataIndex(location);
      if (data_index >= 0)
      {
        AccessIndex(data_index) = value;
        return DSHVGSetStatus::SET_CELL;
      }
      else
      {
        throw std::runtime_error("location not in chunk");
      }
    }
    else
    {
      throw std::runtime_error("Cannot cell set unfilled chunk");
    }
  }

  DSHVGSetStatus SetChunkValue(const T& value)
  {
    if (fill_status_ != DSHVGFillStatus::NOT_FILLED)
    {
      SetChunkFilledContents(value);
      return DSHVGSetStatus::SET_CHUNK;
    }
    else
    {
      throw std::runtime_error("Cannot set unfilled chunk");
    }
  }

  DSHVGSetStatus SetChunkValue(T&& value)
  {
    if (fill_status_ != DSHVGFillStatus::NOT_FILLED)
    {
      SetChunkFilledContents(value);
      return DSHVGSetStatus::SET_CHUNK;
    }
    else
    {
      throw std::runtime_error("Cannot set unfilled chunk");
    }
  }

  Eigen::Vector4d GetCellLocationInGridFrame(
      const GridIndex& internal_cell_index) const
  {
    Eigen::Vector4d cell_position =
        sizes_.IndexToLocationInGridFrame(internal_cell_index);
    // We need to move from position to offset vector
    cell_position(3) = 0.0;
    return region_.Base() + cell_position;
  }

  Eigen::Vector4d GetChunkCenterInGridFrame() const
  {
    const Eigen::Vector4d center_offset(sizes_.XSize() * 0.5,
                                        sizes_.YSize() * 0.5,
                                        sizes_.ZSize() * 0.5,
                                        0.0);
    return region_.Base() + center_offset;
  }
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
  using ChunkMapAllocator =
      Eigen::aligned_allocator<std::pair<const ChunkRegion, GridChunk>>;
  using ChunkMap =
      std::unordered_map<
          ChunkRegion, GridChunk, std::hash<ChunkRegion>,
          std::equal_to<ChunkRegion>, ChunkMapAllocator>;

  Eigen::Isometry3d origin_transform_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d inverse_origin_transform_ = Eigen::Isometry3d::Identity();
  T default_value_;
  GridSizes chunk_sizes_;
  ChunkMap chunks_;
  bool initialized_ = false;

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
    GridSizes::Serialize(chunk_sizes_, buffer);
    // Serialize the data
    const auto chunk_serializer
        = [&] (const GridChunk& chunk, std::vector<uint8_t>& serialize_buffer)
    {
      return GridChunk::Serialize(chunk, serialize_buffer, value_serializer);
    };
    serialization::SerializeMapLike<ChunkRegion, GridChunk, ChunkMap>(
        chunks_, buffer, ChunkRegion::Serialize, chunk_serializer);
    // Serialize the initialized
    serialization::SerializeMemcpyable<uint8_t>(
        static_cast<uint8_t>(initialized_), buffer);
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
    const auto origin_transform_deserialized
        = serialization::DeserializeIsometry3d(buffer, current_position);
    origin_transform_ = origin_transform_deserialized.Value();
    current_position += origin_transform_deserialized.BytesRead();
    inverse_origin_transform_ = origin_transform_.inverse();
    // Deserialize the default value
    const auto default_value_deserialized
        = value_deserializer(buffer, current_position);
    default_value_ = default_value_deserialized.Value();
    current_position += default_value_deserialized.BytesRead();
    // Deserialize the chunk sizes
    const auto chunk_sizes_deserialized
        = GridSizes::Deserialize(buffer, current_position);
    chunk_sizes_ = chunk_sizes_deserialized.Value();
    current_position += chunk_sizes_deserialized.BytesRead();
    // Deserialize the data
    const auto chunk_deserializer
        = [&] (const std::vector<uint8_t>& deserialize_buffer,
               const uint64_t offset)
    {
      return GridChunk::Deserialize(
          deserialize_buffer, offset, value_deserializer);
    };
    const auto chunks_deserialized
        = serialization::DeserializeMapLike<ChunkRegion, GridChunk, ChunkMap>(
            buffer, current_position, ChunkRegion::Deserialize,
            chunk_deserializer);
    chunks_ = chunks_deserialized.Value();
    current_position += chunks_deserialized.BytesRead();
    // Deserialize the initialized
    const auto initialized_deserialized
        = serialization::DeserializeMemcpyable<uint8_t>(buffer,
                                                        current_position);
    initialized_ = static_cast<bool>(initialized_deserialized.Value());
    current_position += initialized_deserialized.BytesRead();
    // Safety checks
    if (chunk_sizes_.Valid() != initialized_)
    {
      throw std::runtime_error("sizes_.Valid() != initialized_");
    }
    if (chunks_.size() > 0 && !chunk_sizes_.Valid())
    {
      throw std::runtime_error("chunks.size() > 0 with invalid chunk sizes");
    }
    // Figure out how many bytes were read
    const uint64_t bytes_read = current_position - starting_offset;
    return bytes_read;
  }

  ChunkRegion GetContainingChunkRegion(
      const Eigen::Vector4d& grid_location) const
  {
    if (chunk_sizes_.Valid())
    {
      // Given a location in the grid frame, figure out which chunk region
      // contains it.
      const double raw_x_chunk_num = grid_location(0) / chunk_sizes_.XSize();
      const double raw_y_chunk_num = grid_location(1) / chunk_sizes_.YSize();
      const double raw_z_chunk_num = grid_location(2) / chunk_sizes_.ZSize();
      const double region_base_x
          = std::floor(raw_x_chunk_num) * chunk_sizes_.XSize();
      const double region_base_y
          = std::floor(raw_y_chunk_num) * chunk_sizes_.YSize();
      const double region_base_z
          = std::floor(raw_z_chunk_num) * chunk_sizes_.ZSize();
      return ChunkRegion(region_base_x, region_base_y, region_base_z);
    }
    else
    {
      throw std::runtime_error("chunk_sizes_ is not valid");
    }
  }

  void AllocateChunkAt(
      const ChunkRegion& chunk_region, const DSHVGFillType fill_type)
  {
    const auto result = chunks_.emplace(
        chunk_region,
        GridChunk(chunk_region, chunk_sizes_, fill_type, default_value_));
    if (result.second != true)
    {
      throw std::runtime_error(
          "Attempted to allocate new chunk over existing chunk");
    }
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
  virtual bool OnMutableAccess(const Eigen::Vector4d& location) = 0;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DynamicSpatialHashedVoxelGridBase(const GridSizes& chunk_sizes,
                                    const T& default_value,
                                    const size_t expected_chunks)
      : DynamicSpatialHashedVoxelGridBase<T, BackingStore>(
          Eigen::Isometry3d::Identity(), chunk_sizes, default_value,
          expected_chunks) {}

  DynamicSpatialHashedVoxelGridBase(const Eigen::Isometry3d& origin_transform,
                                    const GridSizes& chunk_sizes,
                                    const T& default_value,
                                    const size_t expected_chunks)
  {
    if (chunk_sizes.Valid())
    {
      origin_transform_ = origin_transform;
      inverse_origin_transform_ = origin_transform_.inverse();
      chunk_sizes_ = chunk_sizes;
      default_value_ = default_value;
      chunks_.reserve(expected_chunks);
      initialized_ = true;
    }
    else
    {
      throw std::invalid_argument("chunk_sizes is not valid");
    }
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

  bool IsInitialized() const { return initialized_; }

  DynamicSpatialHashedGridQuery<const T> GetLocationImmutable(
      const double x, const double y, const double z) const
  {
    return GetLocationImmutable4d(Eigen::Vector4d(x, y, z, 1.0));
  }

  DynamicSpatialHashedGridQuery<const T> GetLocationImmutable3d(
      const Eigen::Vector3d& location) const
  {
    return GetLocationImmutable4d(
        Eigen::Vector4d(location.x(), location.y(), location.z(), 1.0));
  }

  DynamicSpatialHashedGridQuery<const T> GetLocationImmutable4d(
      const Eigen::Vector4d& location) const
  {
    const Eigen::Vector4d grid_frame_location
        = inverse_origin_transform_ * location;
    const ChunkRegion region = GetContainingChunkRegion(grid_frame_location);
    auto found_chunk_itr = chunks_.find(region);
    if (found_chunk_itr != chunks_.end())
    {
      return found_chunk_itr->second.GetLocationImmutable4d(
          grid_frame_location);
    }
    else
    {
      return DynamicSpatialHashedGridQuery<const T>::NotFound();
    }
  }

  DynamicSpatialHashedGridQuery<T> GetLocationMutable(
      const double x, const double y, const double z)
  {
    return GetLocationMutable4d(Eigen::Vector4d(x, y, z, 1.0));
  }

  DynamicSpatialHashedGridQuery<T> GetLocationMutable3d(
      const Eigen::Vector3d& location)
  {
    return GetLocationMutable4d(
        Eigen::Vector4d(location.x(), location.y(), location.z(), 1.0));
  }

  DynamicSpatialHashedGridQuery<T> GetLocationMutable4d(
      const Eigen::Vector4d& location)
  {
    const Eigen::Vector4d grid_frame_location
        = inverse_origin_transform_ * location;
    const ChunkRegion region = GetContainingChunkRegion(grid_frame_location);
    auto found_chunk_itr = chunks_.find(region);
    if (found_chunk_itr != chunks_.end())
    {
      if (OnMutableAccess(grid_frame_location))
      {
        return found_chunk_itr->second.GetLocationMutable4d(grid_frame_location);
      }
      else
      {
        return DynamicSpatialHashedGridQuery<T>::MutableAccessProhibited();
      }
    }
    else
    {
      return DynamicSpatialHashedGridQuery<T>::NotFound();
    }
  }

  DSHVGSetStatus SetLocation(
      const double x, const double y, const double z,
      const DSHVGSetType set_type, const T& value)
  {
    return SetLocation4d(Eigen::Vector4d(x, y, z, 1.0), set_type, value);
  }

  DSHVGSetStatus SetLocation3d(
      const Eigen::Vector3d& location, const DSHVGSetType set_type,
      const T& value)
  {
    return SetLocation4d(
        Eigen::Vector4d(location.x(), location.y(), location.z(), 1.0),
        set_type, value);
  }

  DSHVGSetStatus SetLocation4d(
      const Eigen::Vector4d& location, const DSHVGSetType set_type,
      const T& value)
  {
    const Eigen::Vector4d grid_frame_location
        = inverse_origin_transform_ * location;
    if (OnMutableAccess(grid_frame_location))
    {
      const ChunkRegion region = GetContainingChunkRegion(grid_frame_location);
      auto found_chunk_itr = chunks_.find(region);
      if (found_chunk_itr != chunks_.end())
      {
        if (set_type == DSHVGSetType::SET_CELL)
        {
          return found_chunk_itr->second.SetCellLocation4d(
              grid_frame_location, value);
        }
        else
        {
          return found_chunk_itr->second.SetChunkValue(value);
        }
      }
      else
      {
        const DSHVGFillType fill_type
            = (set_type == DSHVGSetType::SET_CELL)
                ? DSHVGFillType::FILL_CELL : DSHVGFillType::FILL_CHUNK;
        AllocateChunkAt(region, fill_type);
        return SetLocation4d(location, set_type, value);
      }
    }
    else
    {
      return DSHVGSetStatus::MUTABLE_ACCESS_PROHIBITED;
    }
  }

  DSHVGSetStatus SetLocation(
      const double x, const double y, const double z,
      const DSHVGSetType set_type, T&& value)
  {
    return SetLocation4d(Eigen::Vector4d(x, y, z, 1.0), set_type, value);
  }

  DSHVGSetStatus SetLocation3d(
      const Eigen::Vector3d& location, const DSHVGSetType set_type,
      T&& value)
  {
    return SetLocation4d(
        Eigen::Vector4d(location.x(), location.y(), location.z(), 1.0),
        set_type, value);
  }

  DSHVGSetStatus SetLocation4d(
      const Eigen::Vector4d& location, const DSHVGSetType set_type,
      T&& value)
  {
    const Eigen::Vector4d grid_frame_location
        = inverse_origin_transform_ * location;
    if (OnMutableAccess(grid_frame_location))
    {
      const ChunkRegion region = GetContainingChunkRegion(grid_frame_location);
      auto found_chunk_itr = chunks_.find(region);
      if (found_chunk_itr != chunks_.end())
      {
        if (set_type == DSHVGSetType::SET_CELL)
        {
          return found_chunk_itr->second.SetCellLocation4d(
              grid_frame_location, value);
        }
        else
        {
          return found_chunk_itr->second.SetChunkValue(value);
        }
      }
      else
      {
        const DSHVGFillType fill_type
            = (set_type == DSHVGSetType::SET_CELL)
                ? DSHVGFillType::FILL_CELL : DSHVGFillType::FILL_CHUNK;
        AllocateChunkAt(region, fill_type);
        return SetLocation4d(location, set_type, value);
      }
    }
    else
    {
      return DSHVGSetStatus::MUTABLE_ACCESS_PROHIBITED;
    }
  }

  const GridSizes& GetChunkGridSizes() const { return chunk_sizes_; }

  Eigen::Vector3d GetCellSizes() const { return chunk_sizes_.CellSizes(); }

  Eigen::Vector3d GetChunkSizes() const { return chunk_sizes_.Sizes(); }

  Eigen::Matrix<int64_t, 3, 1> GetChunkNumCells() const
  {
    return chunk_sizes_.NumCells();
  }

  const T& GetDefaultValue() const { return default_value_; }

  void SetDefaultValue(const T& default_value)
  {
    default_value_ = default_value;
  }

  const Eigen::Isometry3d& GetOriginTransform() const
  {
    return origin_transform_;
  }

  const Eigen::Isometry3d& GetInverseOriginTransform() const
  {
    return inverse_origin_transform_;
  }

  void UpdateOriginTransform(const Eigen::Isometry3d& origin_transform)
  {
    origin_transform_ = origin_transform;
    inverse_origin_transform_ = origin_transform_.inverse();
  }

  bool HasUniformCellSize() const { return chunk_sizes_.UniformCellSize(); }

  const ChunkMap& GetImmutableInternalChunks() const
  {
    return chunks_;
  }

  ChunkMap& GetMutableInternalChunks() const
  {
    return chunks_;
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

  bool OnMutableAccess(const Eigen::Vector4d& location) override
  {
    CRU_UNUSED(location);
    return true;
  }

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
    const uint64_t bytes_read
        = temp_grid.DeserializeSelf(buffer, starting_offset,
                                    value_deserializer);
    return serialization::MakeDeserialized(temp_grid, bytes_read);
  }

  DynamicSpatialHashedVoxelGrid(const GridSizes& chunk_sizes,
                                const T& default_value,
                                const size_t expected_chunks)
      : DynamicSpatialHashedVoxelGridBase<T, BackingStore>(
          Eigen::Isometry3d::Identity(), chunk_sizes, default_value,
          expected_chunks) {}

  DynamicSpatialHashedVoxelGrid(const Eigen::Isometry3d& origin_transform,
                                const GridSizes& chunk_sizes,
                                const T& default_value,
                                const size_t expected_chunks)
      : DynamicSpatialHashedVoxelGridBase<T, BackingStore>(
          origin_transform, chunk_sizes, default_value, expected_chunks) {}

  DynamicSpatialHashedVoxelGrid()
      : DynamicSpatialHashedVoxelGridBase<T, BackingStore>() {}
};
}  // namespace voxel_grid
}  // namespace common_robotics_utilities

namespace std
{
  template <>
  struct hash<common_robotics_utilities::voxel_grid::ChunkRegion>
  {
    std::size_t operator()(
        const common_robotics_utilities::voxel_grid::ChunkRegion& region) const
    {
      const Eigen::Vector4d& base = region.Base();
      std::size_t hash_val = 0;
      common_robotics_utilities::utility::hash_combine(
          hash_val, base(0), base(1), base(2));
      return hash_val;
    }
  };
}
