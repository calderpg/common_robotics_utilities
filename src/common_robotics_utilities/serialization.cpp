#include <common_robotics_utilities/serialization.hpp>

#include <cstdint>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

namespace common_robotics_utilities
{
namespace serialization
{
constexpr uint64_t SerializedSizeVector2d()
{
  return static_cast<uint64_t>(sizeof(double) * 2);
}

constexpr uint64_t SerializedSizeVector3d()
{
  return static_cast<uint64_t>(sizeof(double) * 3);
}

constexpr uint64_t SerializedSizeVector4d()
{
  return static_cast<uint64_t>(sizeof(double) * 4);
}

constexpr uint64_t SerializedSizeQuaterniond()
{
  return static_cast<uint64_t>(sizeof(double) * 4);
}

constexpr uint64_t SerializedSizeIsometry3d()
{
  return static_cast<uint64_t>(sizeof(double) * 16);
}

uint64_t SerializeMatrixXd(
    const Eigen::MatrixXd& value, std::vector<uint8_t>& buffer)
{
  const uint64_t start_buffer_size = buffer.size();
  // First, write uint64_t rows and cols headers
  const uint64_t rows = static_cast<uint64_t>(value.rows());
  SerializeMemcpyable<uint64_t>(rows, buffer);
  const uint64_t cols = static_cast<uint64_t>(value.cols());
  SerializeMemcpyable<uint64_t>(cols, buffer);
  // Expand the buffer to handle everything
  const size_t serialized_length = sizeof(double) * rows * cols;
  const size_t previous_buffer_size = buffer.size();
  buffer.resize(previous_buffer_size + serialized_length, 0x00);
  // Serialize the contained items
  memcpy(&buffer[previous_buffer_size], value.data(), serialized_length);
  // Figure out how many bytes were written
  const uint64_t end_buffer_size = buffer.size();
  const uint64_t bytes_written = end_buffer_size - start_buffer_size;
  return bytes_written;
}

uint64_t SerializeVectorXd(
    const Eigen::VectorXd& value, std::vector<uint8_t>& buffer)
{
  return SerializeMemcpyableVectorLike<double, Eigen::VectorXd>(value, buffer);
}

uint64_t SerializeVector2d(
    const Eigen::Vector2d& value, std::vector<uint8_t>& buffer)
{
  // Takes a state to serialize and a buffer to serialize into
  // Return number of bytes written to buffer
  std::vector<uint8_t> temp_buffer(SerializedSizeVector2d(), 0x00);
  memcpy(&temp_buffer.front(), value.data(), SerializedSizeVector2d());
  buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
  return SerializedSizeVector2d();
}

uint64_t SerializeVector3d(
    const Eigen::Vector3d& value, std::vector<uint8_t>& buffer)
{
  // Takes a state to serialize and a buffer to serialize into
  // Return number of bytes written to buffer
  std::vector<uint8_t> temp_buffer(SerializedSizeVector3d(), 0x00);
  memcpy(&temp_buffer.front(), value.data(), SerializedSizeVector3d());
  buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
  return SerializedSizeVector3d();
}

uint64_t SerializeVector4d(
    const Eigen::Vector4d& value, std::vector<uint8_t>& buffer)
{
  // Takes a state to serialize and a buffer to serialize into
  // Return number of bytes written to buffer
  std::vector<uint8_t> temp_buffer(SerializedSizeVector4d(), 0x00);
  memcpy(&temp_buffer.front(), value.data(), SerializedSizeVector4d());
  buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
  return SerializedSizeVector4d();
}

uint64_t SerializeQuaterniond(
    const Eigen::Quaterniond& value, std::vector<uint8_t>& buffer)
{
  // Takes a state to serialize and a buffer to serialize into
  // Return number of bytes written to buffer
  std::vector<uint8_t> temp_buffer(SerializedSizeQuaterniond(), 0x00);
  memcpy(&temp_buffer.front(),
         value.coeffs().data(),
         SerializedSizeQuaterniond());
  buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
  return SerializedSizeQuaterniond();
}

uint64_t SerializeIsometry3d(
    const Eigen::Isometry3d& value, std::vector<uint8_t>& buffer)
{
  // Takes a state to serialize and a buffer to serialize into
  // Return number of bytes written to buffer
  std::vector<uint8_t> temp_buffer(SerializedSizeIsometry3d(), 0x00);
  memcpy(&temp_buffer.front(),
         value.matrix().data(),
         SerializedSizeIsometry3d());
  buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
  return SerializedSizeIsometry3d();
}

Deserialized<Eigen::MatrixXd> DeserializeMatrixXd(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
{
  uint64_t current_position = starting_offset;
  // Load the rows and cols header
  const Deserialized<uint64_t> deserialized_rows
      = DeserializeMemcpyable<uint64_t>(buffer, current_position);
  const uint64_t rows = deserialized_rows.Value();
  current_position += deserialized_rows.BytesRead();
  const Deserialized<uint64_t> deserialized_cols
      = DeserializeMemcpyable<uint64_t>(buffer, current_position);
  const uint64_t cols = deserialized_cols.Value();
  current_position += deserialized_cols.BytesRead();
  // Deserialize the items
  const size_t serialized_length = sizeof(double) * rows * cols;
  if ((current_position + serialized_length) > buffer.size())
  {
    throw std::invalid_argument("Not enough room in the provided buffer");
  }
  Eigen::MatrixXd deserialized = Eigen::MatrixXd::Zero(
      static_cast<ssize_t>(rows), static_cast<ssize_t>(cols));
  memcpy(deserialized.data(), &buffer[current_position], serialized_length);
  current_position += serialized_length;
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return MakeDeserialized(deserialized, bytes_read);
}

Deserialized<Eigen::VectorXd> DeserializeVectorXd(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
{
  return DeserializeMemcpyableVectorLike<double, Eigen::VectorXd>(
      buffer, starting_offset);
}

Deserialized<Eigen::Vector2d> DeserializeVector2d(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
{
  if (starting_offset >= buffer.size())
  {
    throw std::invalid_argument(
        "starting_offset is outside the provided buffer");
  }
  if ((starting_offset + SerializedSizeVector2d()) > buffer.size())
  {
    throw std::invalid_argument("Not enough room in the provided buffer");
  }
  // Takes a buffer to read from and the starting index in the buffer
  // Return the loaded state and how many bytes we read from the buffer
  Eigen::Vector2d temp_value;
  memcpy(temp_value.data(), &buffer[starting_offset], SerializedSizeVector2d());
  return MakeDeserialized(temp_value, SerializedSizeVector2d());
}

Deserialized<Eigen::Vector3d> DeserializeVector3d(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
{
  if (starting_offset >= buffer.size())
  {
    throw std::invalid_argument(
        "starting_offset is outside the provided buffer");
  }
  if ((starting_offset + SerializedSizeVector3d()) > buffer.size())
  {
    throw std::invalid_argument("Not enough room in the provided buffer");
  }
  // Takes a buffer to read from and the starting index in the buffer
  // Return the loaded state and how many bytes we read from the buffer
  Eigen::Vector3d temp_value;
  memcpy(temp_value.data(), &buffer[starting_offset], SerializedSizeVector3d());
  return MakeDeserialized(temp_value, SerializedSizeVector3d());
}

Deserialized<Eigen::Vector4d> DeserializeVector4d(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
{
  if (starting_offset >= buffer.size())
  {
    throw std::invalid_argument(
        "starting_offset is outside the provided buffer");
  }
  if ((starting_offset + SerializedSizeVector4d()) > buffer.size())
  {
    throw std::invalid_argument("Not enough room in the provided buffer");
  }
  // Takes a buffer to read from and the starting index in the buffer
  // Return the loaded state and how many bytes we read from the buffer
  Eigen::Vector4d temp_value;
  memcpy(temp_value.data(), &buffer[starting_offset], SerializedSizeVector4d());
  return MakeDeserialized(temp_value, SerializedSizeVector4d());
}

Deserialized<Eigen::Quaterniond> DeserializeQuaterniond(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
{
  if (starting_offset >= buffer.size())
  {
    throw std::invalid_argument(
        "starting_offset is outside the provided buffer");
  }
  if ((starting_offset + SerializedSizeQuaterniond()) > buffer.size())
  {
    throw std::invalid_argument("Not enough room in the provided buffer");
  }
  // Takes a buffer to read from and the starting index in the buffer
  // Return the loaded state and how many bytes we read from the buffer
  Eigen::Quaterniond temp_value;
  memcpy(temp_value.coeffs().data(),
         &buffer[starting_offset],
         SerializedSizeQuaterniond());
  return MakeDeserialized(temp_value, SerializedSizeQuaterniond());
}

Deserialized<Eigen::Isometry3d> DeserializeIsometry3d(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
{
  if (starting_offset >= buffer.size())
  {
    throw std::invalid_argument(
        "starting_offset is outside the provided buffer");
  }
  if ((starting_offset + SerializedSizeIsometry3d()) > buffer.size())
  {
    throw std::invalid_argument("Not enough room in the provided buffer");
  }
  // Takes a buffer to read from and the starting index in the buffer
  // Return the loaded state and how many bytes we read from the buffer
  Eigen::Isometry3d temp_value;
  memcpy(temp_value.matrix().data(),
         &buffer[starting_offset],
         SerializedSizeIsometry3d());
  return MakeDeserialized(temp_value, SerializedSizeIsometry3d());
}
}  // namespace serialization
}  // namespace common_robotics_utilities
