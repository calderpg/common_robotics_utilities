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
  return (uint64_t)(sizeof(double) * 2);
}

constexpr uint64_t SerializedSizeVector3d()
{
  return (uint64_t)(sizeof(double) * 3);
}

constexpr uint64_t SerializedSizeVector4d()
{
  return (uint64_t)(sizeof(double) * 4);
}

constexpr uint64_t SerializedSizeQuaterniond()
{
  return (uint64_t)(sizeof(double) * 4);
}

constexpr uint64_t SerializedSizeIsometry3d()
{
  return (uint64_t)(sizeof(double) * 16);
}

uint64_t SerializedDataSizeVectorXd(const Eigen::VectorXd& vec)
{
  return (uint64_t)(sizeof(double) * (size_t)vec.size());
}

uint64_t SerializedSizeVectorXd(const Eigen::VectorXd& vec)
{
  return (uint64_t)(sizeof(uint64_t) * (sizeof(double) * (size_t)vec.size()));
}

uint64_t SerializeVectorXd(const Eigen::VectorXd& value,
                           std::vector<uint8_t>& buffer)
{
  // Takes a state to serialize and a buffer to serialize into
  // Return number of bytes written to buffer
  const uint64_t serialized_size = SerializedSizeVectorXd(value);
  std::vector<uint8_t> temp_buffer(serialized_size, 0x00);
  // Make the header
  const uint64_t size_header = (uint64_t)value.size();
  SerializeMemcpyable(size_header, temp_buffer);
  // Copy the data
  memcpy(&(temp_buffer[sizeof(size_header)]),
         value.data(),
         SerializedDataSizeVectorXd(value));
  buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
  return serialized_size;
}

uint64_t SerializeVector2d(const Eigen::Vector2d& value,
                           std::vector<uint8_t>& buffer)
{
  // Takes a state to serialize and a buffer to serialize into
  // Return number of bytes written to buffer
  std::vector<uint8_t> temp_buffer(SerializedSizeVector2d(), 0x00);
  memcpy(&temp_buffer.front(), value.data(), SerializedSizeVector2d());
  buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
  return SerializedSizeVector2d();
}

uint64_t SerializeVector3d(const Eigen::Vector3d& value,
                           std::vector<uint8_t>& buffer)
{
  // Takes a state to serialize and a buffer to serialize into
  // Return number of bytes written to buffer
  std::vector<uint8_t> temp_buffer(SerializedSizeVector3d(), 0x00);
  memcpy(&temp_buffer.front(), value.data(), SerializedSizeVector3d());
  buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
  return SerializedSizeVector3d();
}

uint64_t SerializeVector4d(const Eigen::Vector4d& value,
                           std::vector<uint8_t>& buffer)
{
  // Takes a state to serialize and a buffer to serialize into
  // Return number of bytes written to buffer
  std::vector<uint8_t> temp_buffer(SerializedSizeVector4d(), 0x00);
  memcpy(&temp_buffer.front(), value.data(), SerializedSizeVector4d());
  buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
  return SerializedSizeVector4d();
}

uint64_t SerializeQuaterniond(const Eigen::Quaterniond& value,
                              std::vector<uint8_t>& buffer)
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

uint64_t SerializeIsometry3d(const Eigen::Isometry3d& value,
                             std::vector<uint8_t>& buffer)
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

std::pair<Eigen::VectorXd, uint64_t> DeserializeVectorXd(
    const std::vector<uint8_t>& buffer, const uint64_t current)
{
  uint64_t current_position = current;
  // Load the size header
  const std::pair<uint64_t, uint64_t> deserialized_size_header
      = DeserializeMemcpyable<uint64_t>(buffer, current_position);
  const uint64_t vector_length = deserialized_size_header.first;
  current_position += deserialized_size_header.second;
  // Check buffer size
  Eigen::VectorXd temp_value = Eigen::VectorXd::Zero((ssize_t)vector_length);
  const uint64_t data_size = SerializedDataSizeVectorXd(temp_value);
  if ((current + data_size) > buffer.size())
  {
    throw std::invalid_argument("Not enough room in the provided buffer");
  }
  // Load from the buffer
  memcpy(temp_value.data(), &buffer[current_position], data_size);
  current_position += data_size;
  const uint64_t bytes_read = current_position - current;
  return std::make_pair(temp_value, bytes_read);
}

std::pair<Eigen::Vector2d, uint64_t> DeserializeVector2d(
    const std::vector<uint8_t>& buffer, const uint64_t current)
{
  if (current >= buffer.size())
  {
    throw std::invalid_argument("current is outside the provided buffer");
  }
  if ((current + SerializedSizeVector2d()) > buffer.size())
  {
    throw std::invalid_argument("Not enough room in the provided buffer");
  }
  // Takes a buffer to read from and the starting index in the buffer
  // Return the loaded state and how many bytes we read from the buffer
  Eigen::Vector2d temp_value;
  memcpy(temp_value.data(), &buffer[current], SerializedSizeVector2d());
  return std::make_pair(temp_value, SerializedSizeVector2d());
}

std::pair<Eigen::Vector3d, uint64_t> DeserializeVector3d(
    const std::vector<uint8_t>& buffer, const uint64_t current)
{
  if (current >= buffer.size())
  {
    throw std::invalid_argument("current is outside the provided buffer");
  }
  if ((current + SerializedSizeVector3d()) > buffer.size())
  {
    throw std::invalid_argument("Not enough room in the provided buffer");
  }
  // Takes a buffer to read from and the starting index in the buffer
  // Return the loaded state and how many bytes we read from the buffer
  Eigen::Vector3d temp_value;
  memcpy(temp_value.data(), &buffer[current], SerializedSizeVector3d());
  return std::make_pair(temp_value, SerializedSizeVector3d());
}

std::pair<Eigen::Vector4d, uint64_t> DeserializeVector4d(
    const std::vector<uint8_t>& buffer, const uint64_t current)
{
  if (current >= buffer.size())
  {
    throw std::invalid_argument("current is outside the provided buffer");
  }
  if ((current + SerializedSizeVector4d()) > buffer.size())
  {
    throw std::invalid_argument("Not enough room in the provided buffer");
  }
  // Takes a buffer to read from and the starting index in the buffer
  // Return the loaded state and how many bytes we read from the buffer
  Eigen::Vector4d temp_value;
  memcpy(temp_value.data(), &buffer[current], SerializedSizeVector4d());
  return std::make_pair(temp_value, SerializedSizeVector4d());
}

std::pair<Eigen::Quaterniond, uint64_t> DeserializeQuaterniond(
    const std::vector<uint8_t>& buffer, const uint64_t current)
{
  if (current >= buffer.size())
  {
    throw std::invalid_argument("current is outside the provided buffer");
  }
  if ((current + SerializedSizeQuaterniond()) > buffer.size())
  {
    throw std::invalid_argument("Not enough room in the provided buffer");
  }
  // Takes a buffer to read from and the starting index in the buffer
  // Return the loaded state and how many bytes we read from the buffer
  Eigen::Quaterniond temp_value;
  memcpy(temp_value.coeffs().data(),
         &buffer[current],
         SerializedSizeQuaterniond());
  return std::make_pair(temp_value, SerializedSizeQuaterniond());
}

std::pair<Eigen::Isometry3d, uint64_t> DeserializeIsometry3d(
    const std::vector<uint8_t>& buffer, const uint64_t current)
{
  if (current >= buffer.size())
  {
    throw std::invalid_argument("current is outside the provided buffer");
  }
  if ((current + SerializedSizeIsometry3d()) > buffer.size())
  {
    throw std::invalid_argument("Not enough room in the provided buffer");
  }
  // Takes a buffer to read from and the starting index in the buffer
  // Return the loaded state and how many bytes we read from the buffer
  Eigen::Isometry3d temp_value;
  memcpy(temp_value.matrix().data(),
         &buffer[current],
         SerializedSizeIsometry3d());
  return std::make_pair(temp_value, SerializedSizeIsometry3d());
}
}  // namespace serialization
}  // namespace common_robotics_utilities
