#pragma once

#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <type_traits>
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

class VoxelGridSizes
{
private:
  // Voxel sizes (and their inverses) are stored in Vector4 to enable certain
  // SIMD operations.
  Eigen::Vector4d voxel_sizes_ = Eigen::Vector4d::Zero();
  Eigen::Vector4d inverse_voxel_sizes_ = Eigen::Vector4d::Zero();
  Eigen::Vector3d grid_sizes_ = Eigen::Vector3d::Zero();
  Vector3i64 voxel_counts_ = Vector3i64::Zero();
  int64_t num_total_voxels_ = 0;
  int64_t stride_1_ = 0;
  int64_t stride_2_ = 0;

  static bool CheckPositiveValid(const double param)
  {
    return (std::isfinite(param) && (param > 0.0));
  }

  static bool CheckPositiveValid(const int64_t param)
  {
    return (param > 0);
  }

  // This constructor is private so that users can only construct via the named
  // factory methods, which avoids ambiguity between grid sizes and voxel counts
  // parameters.
  VoxelGridSizes(
      const Eigen::Vector3d& voxel_sizes, const Vector3i64& voxel_counts)
  {
    const bool initialized = Initialize(voxel_sizes, voxel_counts);

    if (!initialized)
    {
      throw std::invalid_argument(
          "All size parameters must be positive, non-zero, and finite");
    }
  }

public:
  static uint64_t Serialize(
      const VoxelGridSizes& sizes, std::vector<uint8_t>& buffer)
  {
    const uint64_t start_buffer_size = buffer.size();

    // Serialize everything needed to reproduce the grid sizes.
    serialization::SerializeMemcpyable<double>(sizes.VoxelXSize(), buffer);
    serialization::SerializeMemcpyable<double>(sizes.VoxelYSize(), buffer);
    serialization::SerializeMemcpyable<double>(sizes.VoxelZSize(), buffer);
    serialization::SerializeMemcpyable<int64_t>(sizes.NumXVoxels(), buffer);
    serialization::SerializeMemcpyable<int64_t>(sizes.NumYVoxels(), buffer);
    serialization::SerializeMemcpyable<int64_t>(sizes.NumZVoxels(), buffer);

    // Figure out how many bytes were written.
    const uint64_t end_buffer_size = buffer.size();
    const uint64_t bytes_written = end_buffer_size - start_buffer_size;
    return bytes_written;
  }

  static serialization::Deserialized<VoxelGridSizes> Deserialize(
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
    const auto num_x_voxels_deserialized =
        serialization::DeserializeMemcpyable<int64_t>(buffer, current_position);
    const int64_t num_x_voxels = num_x_voxels_deserialized.Value();
    current_position += num_x_voxels_deserialized.BytesRead();
    const auto num_y_voxels_deserialized =
        serialization::DeserializeMemcpyable<int64_t>(buffer, current_position);
    const int64_t num_y_voxels = num_y_voxels_deserialized.Value();
    current_position += num_y_voxels_deserialized.BytesRead();
    const auto num_z_voxels_deserialized =
        serialization::DeserializeMemcpyable<int64_t>(buffer, current_position);
    const int64_t num_z_voxels = num_z_voxels_deserialized.Value();
    current_position += num_z_voxels_deserialized.BytesRead();

    const Vector3i64 voxel_counts(num_x_voxels, num_y_voxels, num_z_voxels);

    // Start with a default-constructed VoxelGridSizes.
    VoxelGridSizes temp_sizes;

    // Attempt to initialize from the deserialized values. If any of them are
    // invalid, temp_sizes is unchanged.
    temp_sizes.Initialize(voxel_sizes, voxel_counts);

    // Figure out how many bytes were read.
    const uint64_t bytes_read = current_position - starting_offset;
    return serialization::MakeDeserialized(temp_sizes, bytes_read);
  }

  static VoxelGridSizes FromGridSizes(
      const Eigen::Vector3d& voxel_sizes, const Eigen::Vector3d& grid_sizes)
  {
    const Vector3i64 voxel_counts(
        static_cast<int64_t>(std::ceil(grid_sizes.x() / voxel_sizes.x())),
        static_cast<int64_t>(std::ceil(grid_sizes.y() / voxel_sizes.y())),
        static_cast<int64_t>(std::ceil(grid_sizes.z() / voxel_sizes.z())));
    return VoxelGridSizes(voxel_sizes, voxel_counts);
  }

  static VoxelGridSizes FromGridSizes(
      const double voxel_size, const Eigen::Vector3d& grid_sizes)
  {
    return FromGridSizes(
        Eigen::Vector3d(voxel_size, voxel_size, voxel_size), grid_sizes);
  }

  static VoxelGridSizes FromVoxelCounts(
      const Eigen::Vector3d& voxel_sizes, const Vector3i64& voxel_counts)
  {
    return VoxelGridSizes(voxel_sizes, voxel_counts);
  }

  static VoxelGridSizes FromVoxelCounts(
      const double voxel_size, const Vector3i64& voxel_counts)
  {
    return FromVoxelCounts(
        Eigen::Vector3d(voxel_size, voxel_size, voxel_size), voxel_counts);
  }

  VoxelGridSizes() {}

  // This is exposed only for testing.
  bool Initialize(
      const Eigen::Vector3d& voxel_sizes, const Vector3i64& voxel_counts)
  {
    if (CheckPositiveValid(voxel_sizes.x()) &&
        CheckPositiveValid(voxel_sizes.y()) &&
        CheckPositiveValid(voxel_sizes.z()) &&
        CheckPositiveValid(voxel_counts.x()) &&
        CheckPositiveValid(voxel_counts.y()) &&
        CheckPositiveValid(voxel_counts.z()))
    {
      voxel_sizes_ = Eigen::Vector4d(
          voxel_sizes.x(), voxel_sizes.y(), voxel_sizes.z(), 1.0);
      inverse_voxel_sizes_ = voxel_sizes_.cwiseInverse();
      voxel_counts_ = voxel_counts;
      num_total_voxels_ = voxel_counts_.prod();
      grid_sizes_ =
          voxel_sizes_.head<3>().cwiseProduct(voxel_counts_.cast<double>());
      stride_1_ = voxel_counts_.y() * voxel_counts_.z();
      stride_2_ = voxel_counts_.z();

      return true;
    }
    else
    {
      return false;
    }
  }

  bool IsValid() const { return stride_2_ > 0; }

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

  // Accessors for grid sizes.

  const Eigen::Vector3d& GridSizes() const { return grid_sizes_; }

  double GridXSize() const { return grid_sizes_.x(); }

  double GridYSize() const { return grid_sizes_.y(); }

  double GridZSize() const { return grid_sizes_.z(); }

  // Accessors for voxel counts.

  const Vector3i64& VoxelCounts() const { return voxel_counts_; }

  int64_t NumXVoxels() const { return voxel_counts_.x(); }

  int64_t NumYVoxels() const { return voxel_counts_.y(); }

  int64_t NumZVoxels() const { return voxel_counts_.z(); }

  int64_t NumTotalVoxels() const { return num_total_voxels_; }

  // Accessors for strides.

  int64_t Stride1() const { return stride_1_; }

  int64_t Stride2() const { return stride_2_; }

  // Index bounds checks.

  bool CheckGridIndexInBounds(const int64_t x_index,
                              const int64_t y_index,
                              const int64_t z_index) const
  {
    return x_index >= 0 && x_index < NumXVoxels() &&
           y_index >= 0 && y_index < NumYVoxels() &&
           z_index >= 0 && z_index < NumZVoxels();
  }

  bool CheckGridIndexInBounds(const GridIndex& index) const
  {
    return CheckGridIndexInBounds(index.X(), index.Y(), index.Z());
  }

  bool CheckDataIndexInBounds(const int64_t data_index) const
  {
    return data_index >= 0 && data_index < NumTotalVoxels();
  }

  // Grid index <-> data index conversions.

  int64_t GridIndexToDataIndex(const int64_t x_index,
                               const int64_t y_index,
                               const int64_t z_index) const
  {
    if (CheckGridIndexInBounds(x_index, y_index, z_index))
    {
      return (x_index * Stride1()) + (y_index * Stride2()) + z_index;
    }
    else
    {
      // Return a clearly invalid data index for grid indices out of bounds.
      return std::numeric_limits<int64_t>::lowest();
    }
  }

  int64_t GridIndexToDataIndex(const GridIndex& index) const
  {
    return GridIndexToDataIndex(index.X(), index.Y(), index.Z());
  }

  GridIndex DataIndexToGridIndex(const int64_t data_index) const
  {
    if (CheckDataIndexInBounds(data_index))
    {
      const int64_t x_idx = data_index / Stride1();
      const int64_t remainder = data_index % Stride1();
      const int64_t y_idx = remainder / Stride2();
      const int64_t z_idx = remainder % Stride2();
      return GridIndex(x_idx, y_idx, z_idx);
    }
    else
    {
      // Return a default-value (clearly invalid) for data indices out of range.
      return GridIndex();
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

  // Grid-frame location bounds checks.

  bool CheckGridFrameLocationInBounds3d(const Eigen::Vector3d& location) const
  {
    const GridIndex index = LocationInGridFrameToGridIndex3d(location);
    return CheckGridIndexInBounds(index);
  }

  bool CheckGridFrameLocationInBounds4d(const Eigen::Vector4d& location) const
  {
    const GridIndex index = LocationInGridFrameToGridIndex4d(location);
    return CheckGridIndexInBounds(index);
  }

  bool CheckGridFrameLocationInBounds(
      const double x, const double y, const double z) const
  {
    const GridIndex index = LocationInGridFrameToGridIndex(x, y, z);
    return CheckGridIndexInBounds(index);
  }

  // Equality operators.

  bool operator==(const VoxelGridSizes& other) const
  {
    return (VoxelSizesInternal().array() ==
                other.VoxelSizesInternal().array()).all() &&
           (VoxelCounts().array() == other.VoxelCounts().array()).all();
  }

  bool operator!=(const VoxelGridSizes& other) const
  {
    return !(*this == other);
  }
};

static_assert(
    std::is_trivially_destructible<VoxelGridSizes>::value,
    "VoxelGridSizes must be trivially destructible");

// While this looks like a std::optional<T>, it *does not own* the item of T,
// unlike std::optional<T>, since it needs to pass the caller a const/mutable
// reference to the item in the voxel grid.
template<typename T>
class VoxelGridQuery
{
private:
  ReferencingMaybe<T> value_;
  AccessStatus status_ = AccessStatus::UNKNOWN;

  // This struct (and its uses) exists to disambiguate between the value-found
  // and status constructors.
  struct AccessStatusSuccess {};

  explicit VoxelGridQuery(T& value, AccessStatusSuccess)
      : value_(value), status_(AccessStatus::SUCCESS) {}

  explicit VoxelGridQuery(const AccessStatus status) : status_(status)
  {
    if (status_ == AccessStatus::SUCCESS)
    {
      throw std::invalid_argument(
          "VoxelGridQuery cannot be constructed with AccessStatus::SUCCESS");
    }
  }

public:
  static VoxelGridQuery<T> Success(T& value)
  {
    return VoxelGridQuery<T>(value, AccessStatusSuccess{});
  }

  static VoxelGridQuery<T> OutOfBounds()
  {
    return VoxelGridQuery<T>(AccessStatus::OUT_OF_BOUNDS);
  }

  static VoxelGridQuery<T> MutableAccessProhibited()
  {
    return VoxelGridQuery<T>(AccessStatus::MUTABLE_ACCESS_PROHIBITED);
  }

  static VoxelGridQuery<T> Unknown()
  {
    return VoxelGridQuery<T>(AccessStatus::UNKNOWN);
  }

  VoxelGridQuery() = default;

  VoxelGridQuery(const VoxelGridQuery<T>& other) = default;

  VoxelGridQuery(VoxelGridQuery<T>&& other) = default;

  VoxelGridQuery<T>& operator=(const VoxelGridQuery<T>& other) = default;

  VoxelGridQuery<T>& operator=(VoxelGridQuery<T>&& other) = default;

  void Reset()
  {
    value_.Reset();
    status_ = AccessStatus::UNKNOWN;
  }

  T& Value() const { return value_.Value(); }

  T& Value() { return value_.Value(); }

  AccessStatus Status() const { return status_; }

  bool HasValue() const { return value_.HasValue(); }

  explicit operator bool() const { return HasValue(); }
};

/// This is the base class for all voxel grid classes. It is pure virtual to
/// force the implementation of certain necessary functions (cloning, access,
/// derived-class memeber de/serialization) in concrete implementations. This is
/// the class to inherit from if you want a VoxelGrid-like type. If all you want
/// is a voxel grid of T, see VoxelGrid<T, BackingStore> below.
template<typename T, typename BackingStore=std::vector<T>>
class VoxelGridBase
{
private:
  Eigen::Isometry3d origin_transform_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d inverse_origin_transform_ = Eigen::Isometry3d::Identity();
  T default_value_;
  T oob_value_;
  BackingStore data_;
  VoxelGridSizes control_sizes_;

  void Initialize(const Eigen::Isometry3d& origin_transform,
                  const VoxelGridSizes& control_sizes,
                  const T& default_value,
                  const T& oob_value)
  {
    if (control_sizes.IsValid())
    {
      origin_transform_ = origin_transform;
      inverse_origin_transform_ = origin_transform_.inverse();
      default_value_ = default_value;
      oob_value_ = oob_value;
      control_sizes_ = control_sizes;
      SetContents(default_value_);
    }
    else
    {
      throw std::invalid_argument("control_sizes is not valid");
    }
  }

  void Initialize(const VoxelGridSizes& control_sizes,
                  const T& default_value,
                  const T& oob_value)
  {
    const Eigen::Translation3d origin_translation(
        control_sizes.GridSizes() * -0.5);
    const Eigen::Isometry3d origin_transform =
        origin_translation * Eigen::Quaterniond::Identity();
    Initialize(origin_transform, control_sizes, default_value, oob_value);
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

  void SetContents(const T& value)
  {
    data_.clear();
    data_.resize(static_cast<typename BackingStore::size_type>(
        NumTotalVoxels()), value);
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
    // Serialize the OOB value
    value_serializer(oob_value_, buffer);
    // Serialize the data
    serialization::SerializeVectorLike<T, BackingStore>(
          data_, buffer, value_serializer);
    // Serialize the grid sizes
    VoxelGridSizes::Serialize(control_sizes_, buffer);
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
    // Deserialize the OOB value
    const auto oob_value_deserialized =
        value_deserializer(buffer, current_position);
    oob_value_ = oob_value_deserialized.Value();
    current_position += oob_value_deserialized.BytesRead();
    // Deserialize the data
    const auto data_deserialized =
        serialization::DeserializeVectorLike<T, BackingStore>(
              buffer, current_position, value_deserializer);
    data_ = data_deserialized.Value();
    current_position += data_deserialized.BytesRead();
    // Deserialize the sizes
    const auto control_sizes_deserialized =
        VoxelGridSizes::Deserialize(buffer, current_position);
    control_sizes_ = control_sizes_deserialized.Value();
    current_position += control_sizes_deserialized.BytesRead();
    if (control_sizes_.NumTotalVoxels() !=
            static_cast<int64_t>(data_.size()))
    {
      throw std::runtime_error(
          "control_sizes_.NumTotalVoxels() != data_.size()");
    }
    // Figure out how many bytes were read
    const uint64_t bytes_read = current_position - starting_offset;
    return bytes_read;
  }

protected:
  // These are pure-virtual in the base class to force their implementation in
  // derived classes.

  /// Do the work necessary for Clone() to copy the current object.
  virtual std::unique_ptr<VoxelGridBase<T, BackingStore>> DoClone() const = 0;

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

  /// Callback on any mutable raw access to the grid (i.e. via data index or
  /// access to the backing store). Return true/false to allow or disallow
  /// mutable access to grid elements. For example, this can be used to prohibit
  /// changes to a non-const grid, or to invalidate a cache if voxels are
  /// modified.
  virtual bool OnMutableRawAccess() = 0;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VoxelGridBase(const Eigen::Isometry3d& origin_transform,
                const VoxelGridSizes& control_sizes,
                const T& default_value,
                const T& oob_value)
  {
    Initialize(origin_transform, control_sizes, default_value, oob_value);
  }

  VoxelGridBase(const VoxelGridSizes& control_sizes,
                const T& default_value,
                const T& oob_value)
  {
    Initialize(control_sizes, default_value, oob_value);
  }

  VoxelGridBase() = default;

  virtual ~VoxelGridBase() {}

  std::unique_ptr<VoxelGridBase<T, BackingStore>> Clone() const
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

  const T& OOBValue() const { return oob_value_; }

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

  void SetOOBValue(const T& oob_value)
  {
    if (OnMutableRawAccess())
    {
      oob_value_ = oob_value;
    }
    else
    {
      throw std::runtime_error("Mutable raw access is prohibited");
    }
  }

  const VoxelGridSizes& ControlSizes() const { return control_sizes_; }

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

  // Helpers forwarded from our VoxelGridSizes.

  // Accessors for voxel sizes.

  Eigen::Vector3d VoxelSizes() const { return control_sizes_.VoxelSizes(); }

  double VoxelXSize() const { return control_sizes_.VoxelXSize(); }

  double VoxelYSize() const { return control_sizes_.VoxelYSize(); }

  double VoxelZSize() const { return control_sizes_.VoxelZSize(); }

  bool HasUniformVoxelSize() const
  {
    return control_sizes_.HasUniformVoxelSize();
  }

  // Accessors for grid sizes.

  const Eigen::Vector3d& GridSizes() const
  {
    return control_sizes_.GridSizes();
  }

  double GridXSize() const { return control_sizes_.GridXSize(); }

  double GridYSize() const { return control_sizes_.GridYSize(); }

  double GridZSize() const { return control_sizes_.GridZSize(); }

  // Accessors for voxel counts.

  const Vector3i64& VoxelCounts() const { return control_sizes_.VoxelCounts(); }

  int64_t NumXVoxels() const { return control_sizes_.NumXVoxels(); }

  int64_t NumYVoxels() const { return control_sizes_.NumYVoxels(); }

  int64_t NumZVoxels() const { return control_sizes_.NumZVoxels(); }

  int64_t NumTotalVoxels() const { return control_sizes_.NumTotalVoxels(); }

  // Index bounds checks.

  bool CheckGridIndexInBounds(const int64_t x_index,
                              const int64_t y_index,
                              const int64_t z_index) const
  {
    return control_sizes_.CheckGridIndexInBounds(x_index, y_index, z_index);
  }

  bool CheckGridIndexInBounds(const GridIndex& index) const
  {
    return control_sizes_.CheckGridIndexInBounds(index);
  }

  bool CheckDataIndexInBounds(const int64_t data_index) const
  {
    return control_sizes_.CheckDataIndexInBounds(data_index);
  }

  // Grid index <-> data index conversions.

  int64_t GridIndexToDataIndex(const int64_t x_index,
                               const int64_t y_index,
                               const int64_t z_index) const
  {
    return control_sizes_.GridIndexToDataIndex(x_index, y_index, z_index);
  }

  int64_t GridIndexToDataIndex(const GridIndex& index) const
  {
    return control_sizes_.GridIndexToDataIndex(index);
  }

  GridIndex DataIndexToGridIndex(const int64_t data_index) const
  {
    return control_sizes_.DataIndexToGridIndex(data_index);
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

  // Grid-frame location bounds checks.

  bool CheckGridFrameLocationInBounds3d(const Eigen::Vector3d& location) const
  {
    return control_sizes_.CheckGridFrameLocationInBounds3d(location);
  }

  bool CheckGridFrameLocationInBounds4d(const Eigen::Vector4d& location) const
  {
    return control_sizes_.CheckGridFrameLocationInBounds4d(location);
  }

  bool CheckGridFrameLocationInBounds(
      const double x, const double y, const double z) const
  {
    return control_sizes_.CheckGridFrameLocationInBounds(x, y, z);
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

  // Location bounds checks.

  bool CheckLocationInBounds3d(const Eigen::Vector3d& location) const
  {
    const Eigen::Vector3d grid_frame_location =
        LocationToGridFrameLocation3d(location);
    return CheckGridFrameLocationInBounds3d(grid_frame_location);
  }

  bool CheckLocationInBounds4d(const Eigen::Vector4d& location) const
  {
    const Eigen::Vector4d grid_frame_location =
        LocationToGridFrameLocation4d(location);
    return CheckGridFrameLocationInBounds4d(grid_frame_location);
  }

  bool CheckLocationInBounds(const double x,
                             const double y,
                             const double z) const
  {
    const Eigen::Vector4d grid_frame_location =
        LocationToGridFrameLocation(x, y, z);
    return CheckGridFrameLocationInBounds4d(grid_frame_location);
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

  VoxelGridQuery<const T> GetLocationImmutable3d(
      const Eigen::Vector3d& location) const
  {
    return GetIndexImmutable(LocationToGridIndex3d(location));
  }

  VoxelGridQuery<const T> GetLocationImmutable4d(
      const Eigen::Vector4d& location) const
  {
    return GetIndexImmutable(LocationToGridIndex4d(location));
  }

  VoxelGridQuery<const T> GetLocationImmutable(
      const double x, const double y, const double z) const
  {
    return GetIndexImmutable(LocationToGridIndex(x, y, z));
  }

  // Immutable index-based queries.

  VoxelGridQuery<const T> GetIndexImmutable(const GridIndex& index) const
  {
    return GetIndexImmutable(index.X(), index.Y(), index.Z());
  }

  VoxelGridQuery<const T> GetIndexImmutable(
      const int64_t x_index, const int64_t y_index, const int64_t z_index) const
  {
    if (CheckGridIndexInBounds(x_index, y_index, z_index))
    {
      return VoxelGridQuery<const T>::Success(
          AccessIndex(GridIndexToDataIndex(x_index, y_index, z_index)));
    }
    else
    {
      return VoxelGridQuery<const T>::OutOfBounds();
    }
  }

  // Mutable location-based queries.

  VoxelGridQuery<T> GetLocationMutable3d(const Eigen::Vector3d& location)
  {
    return GetIndexMutable(LocationToGridIndex3d(location));
  }

  VoxelGridQuery<T> GetLocationMutable4d(const Eigen::Vector4d& location)
  {
    return GetIndexMutable(LocationToGridIndex4d(location));
  }

  VoxelGridQuery<T> GetLocationMutable(
      const double x, const double y, const double z)
  {
    return GetIndexMutable(LocationToGridIndex(x, y, z));
  }

  // Mutable index-based queries.

  VoxelGridQuery<T> GetIndexMutable(const GridIndex& index)
  {
    return GetIndexMutable(index.X(), index.Y(), index.Z());
  }

  VoxelGridQuery<T> GetIndexMutable(
      const int64_t x_index, const int64_t y_index, const int64_t z_index)
  {
    if (CheckGridIndexInBounds(x_index, y_index, z_index))
    {
      if (OnMutableAccess(x_index, y_index, z_index))
      {
        return VoxelGridQuery<T>::Success(
            AccessIndex(GridIndexToDataIndex(x_index, y_index, z_index)));
      }
      else
      {
        return VoxelGridQuery<T>::MutableAccessProhibited();
      }
    }
    else
    {
      return VoxelGridQuery<T>::OutOfBounds();
    }
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
    return SetIndex(index.X(), index.Y(), index.Z(), value);
  }

  AccessStatus SetIndex(
      const int64_t x_index, const int64_t y_index, const int64_t z_index,
      const T& value)
  {
    if (CheckGridIndexInBounds(x_index, y_index, z_index))
    {
      if (OnMutableAccess(x_index, y_index, z_index))
      {
        AccessIndex(GridIndexToDataIndex(x_index, y_index, z_index)) = value;
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
    return SetIndex(index.X(), index.Y(), index.Z(), value);
  }

  AccessStatus SetIndex(
      const int64_t x_index, const int64_t y_index, const int64_t z_index,
      T&& value)
  {
    if (CheckGridIndexInBounds(x_index, y_index, z_index))
    {
      if (OnMutableAccess(x_index, y_index, z_index))
      {
        AccessIndex(GridIndexToDataIndex(x_index, y_index, z_index)) = value;
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

  // Getters and setters for data-indexed values. Note that these methods will
  // throw if `data_index` is out of bounds. Prefer using {Get, Set}Index
  // methods instead for readability and only use these for performance reasons.

  const T& GetDataIndexImmutable(const int64_t data_index) const
  {
    return AccessIndex(data_index);
  }

  T& GetDataIndexMutable(const int64_t data_index)
  {
    if (OnMutableRawAccess())
    {
      return AccessIndex(data_index);
    }
    else
    {
      throw std::runtime_error("Mutable raw access is prohibited");
    }
  }

  void SetDataIndex(const int64_t data_index, const T& value)
  {
    if (OnMutableRawAccess())
    {
      AccessIndex(data_index) = value;
    }
    else
    {
      throw std::runtime_error("Mutable raw access is prohibited");
    }
  }

  void SetDataIndex(const int64_t data_index, T&& value)
  {
    if (OnMutableRawAccess())
    {
      AccessIndex(data_index) = value;
    }
    else
    {
      throw std::runtime_error("Mutable raw access is prohibited");
    }
  }

  BackingStore& GetMutableRawData()
  {
    if (OnMutableRawAccess())
    {
      return data_;
    }
    else
    {
      throw std::runtime_error("Mutable raw access is prohibited");
    }
  }

  const BackingStore& GetImmutableRawData() const { return data_; }

  void SetRawData(const BackingStore& data)
  {
    if (OnMutableRawAccess())
    {
      const int64_t expected_length = NumTotalVoxels();
      if (static_cast<int64_t>(data.size()) == expected_length)
      {
        data_ = data;
      }
      else
      {
        throw std::runtime_error("Provided data is not the expected size");
      }
    }
    else
    {
      throw std::runtime_error("Mutable raw access is prohibited");
    }
  }

  uint64_t HashDataIndex(const int64_t x_index,
                         const int64_t y_index,
                         const int64_t z_index) const
  {
    return static_cast<uint64_t>(
        GridIndexToDataIndex(x_index, y_index, z_index));
  }

  uint64_t HashDataIndex(const GridIndex& index) const
  {
    return static_cast<uint64_t>(GridIndexToDataIndex(index));
  }
};

/// If you want a VoxelGrid<T> this is the class to use. Since you should never
/// inherit from it, this class is final.
template<typename T, typename BackingStore=std::vector<T>>
class VoxelGrid final : public VoxelGridBase<T, BackingStore>
{
private:
  std::unique_ptr<VoxelGridBase<T, BackingStore>> DoClone() const override
  {
    return std::unique_ptr<VoxelGrid<T, BackingStore>>(
        new VoxelGrid<T, BackingStore>(*this));
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
      const VoxelGrid<T, BackingStore>& grid, std::vector<uint8_t>& buffer,
      const serialization::Serializer<T>& value_serializer)
  {
    return grid.SerializeSelf(buffer, value_serializer);
  }

  static serialization::Deserialized<VoxelGrid<T, BackingStore>> Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset,
      const serialization::Deserializer<T>& value_deserializer)
  {
    VoxelGrid<T, BackingStore> temp_grid;
    const uint64_t bytes_read = temp_grid.DeserializeSelf(
        buffer, starting_offset, value_deserializer);
    return serialization::MakeDeserialized(temp_grid, bytes_read);
  }

  VoxelGrid(const Eigen::Isometry3d& origin_transform,
            const VoxelGridSizes& control_sizes,
            const T& default_value)
      : VoxelGridBase<T, BackingStore>(
          origin_transform, control_sizes, default_value, default_value) {}

  VoxelGrid(const Eigen::Isometry3d& origin_transform,
            const VoxelGridSizes& control_sizes,
            const T& default_value,
            const T& oob_value)
      : VoxelGridBase<T, BackingStore>(
          origin_transform, control_sizes, default_value, oob_value) {}

  VoxelGrid(const VoxelGridSizes& control_sizes, const T& default_value)
      : VoxelGridBase<T, BackingStore>(
          control_sizes, default_value, default_value) {}

  VoxelGrid(const VoxelGridSizes& control_sizes,
            const T& default_value,
            const T& oob_value)
      : VoxelGridBase<T, BackingStore>(
          control_sizes, default_value, oob_value) {}

  VoxelGrid() : VoxelGridBase<T, BackingStore>() {}
};

}  // namespace voxel_grid
CRU_NAMESPACE_END
}  // namespace common_robotics_utilities
