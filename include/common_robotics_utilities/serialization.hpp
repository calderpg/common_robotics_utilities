#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

namespace common_robotics_utilities
{
namespace serialization
{
template<typename T>
class Deserialized
{
private:
  T value_;
  uint64_t bytes_read_ = 0;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Deserialized() {}

  Deserialized(const T& value, const uint64_t bytes_read)
      : value_(value), bytes_read_(bytes_read)
  {
    if (bytes_read == 0)
    {
      throw std::invalid_argument(
          "Deserialized item cannot have bytes_read == 0");
    }
  }

  const T& Value() const
  {
    if (HasValue())
    {
      return value_;
    }
    else
    {
      throw std::runtime_error("Deserialized does not have value");
    }
  }

  T& MutableValue()
  {
    if (HasValue())
    {
      return value_;
    }
    else
    {
      throw std::runtime_error("Deserialized does not have value");
    }
  }

  uint64_t BytesRead() const
  {
    if (HasValue())
    {
      return bytes_read_;
    }
    else
    {
      throw std::runtime_error("Deserialized does not have value");
    }
  }

  bool HasValue() const { return (bytes_read_ > 0); }
};

template<typename T>
using DeserializedAllocator = Eigen::aligned_allocator<Deserialized<T>>;

template<typename T>
Deserialized<T> MakeDeserialized(const T& value, const uint64_t bytes_read)
{
  return Deserialized<T>(value, bytes_read);
}

template<typename T>
using Serializer = std::function<uint64_t(const T&, std::vector<uint8_t>&)>;

template<typename T>
using Deserializer =
    std::function<Deserialized<T>(const std::vector<uint8_t>&, const uint64_t)>;

///////////////////////////////////////////////////////////////
/////                     PROTOTYPES ONLY                 /////
///////////////////////////////////////////////////////////////

/// Eigen types

uint64_t SerializeMatrixXd(
    const Eigen::MatrixXd& value, std::vector<uint8_t>& buffer);

uint64_t SerializeVectorXd(
    const Eigen::VectorXd& value, std::vector<uint8_t>& buffer);

uint64_t SerializeVector2d(
    const Eigen::Vector2d& value, std::vector<uint8_t>& buffer);

uint64_t SerializeVector3d(
    const Eigen::Vector3d& value, std::vector<uint8_t>& buffer);

uint64_t SerializeVector4d(
    const Eigen::Vector4d& value, std::vector<uint8_t>& buffer);

uint64_t SerializeQuaterniond(
    const Eigen::Quaterniond& value, std::vector<uint8_t>& buffer);

uint64_t SerializeIsometry3d(
    const Eigen::Isometry3d& value, std::vector<uint8_t>& buffer);

Deserialized<Eigen::MatrixXd> DeserializeMatrixXd(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset);

Deserialized<Eigen::VectorXd> DeserializeVectorXd(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset);

Deserialized<Eigen::Vector2d> DeserializeVector2d(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset);

Deserialized<Eigen::Vector3d> DeserializeVector3d(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset);

Deserialized<Eigen::Vector4d> DeserializeVector4d(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset);

Deserialized<Eigen::Quaterniond> DeserializeQuaterniond(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset);

Deserialized<Eigen::Isometry3d> DeserializeIsometry3d(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset);

/// STL types

template<typename T>
inline uint64_t SerializeMemcpyable(const T& item_to_serialize,
                                    std::vector<uint8_t>& buffer);

template<typename T>
inline Deserialized<T> DeserializeMemcpyable(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset);

template<typename CharType>
inline uint64_t SerializeString(
    const std::basic_string<CharType>& str_to_serialize,
    std::vector<uint8_t>& buffer);

template<typename CharType>
inline Deserialized<std::basic_string<CharType>> DeserializeString(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset);

template<typename T, typename VectorLike=std::vector<T>>
inline uint64_t SerializeVectorLike(
    const VectorLike& vec_to_serialize,
    std::vector<uint8_t>& buffer,
    const Serializer<T>& item_serializer);

template<typename T, typename VectorLike=std::vector<T>>
inline Deserialized<VectorLike> DeserializeVectorLike(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const Deserializer<T>& item_deserializer);

template<typename T, typename VectorLike=std::vector<T>>
inline uint64_t SerializeMemcpyableVectorLike(
    const VectorLike& vec_to_serialize,
    std::vector<uint8_t>& buffer);

template<typename T, typename VectorLike=std::vector<T>>
inline Deserialized<VectorLike>
DeserializeMemcpyableVectorLike(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset);

template<typename Key, typename Value, typename MapLike=std::map<Key, Value>>
inline uint64_t SerializeMapLike(
    const MapLike& map_to_serialize,
    std::vector<uint8_t>& buffer,
    const Serializer<Key>& key_serializer,
    const Serializer<Value>& value_serializer);

template<typename Key, typename Value, typename MapLike=std::map<Key, Value>>
inline Deserialized<MapLike> DeserializeMapLike(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const Deserializer<Key>& key_deserializer,
    const Deserializer<Value>& value_deserializer);

template<typename Key, typename SetLike=std::set<Key>>
inline uint64_t SerializeSetLike(
    const SetLike& set_to_serialize,
    std::vector<uint8_t>& buffer,
    const Serializer<Key>& key_serializer);

template<typename Key, typename SetLike=std::set<Key>>
inline Deserialized<SetLike> DeserializeSetLike(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const Deserializer<Key>& key_deserializer);

template<typename First, typename Second>
inline uint64_t SerializePair(
    const std::pair<First, Second>& pair_to_serialize,
    std::vector<uint8_t>& buffer,
    const Serializer<First>& first_serializer,
    const Serializer<Second>& second_serializer);

template<typename First, typename Second>
inline Deserialized<std::pair<First, Second>> DeserializePair(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const Deserializer<First>& first_deserializer,
    const Deserializer<Second>& second_deserializer);

/// Network byte order variants

template<typename T>
inline uint64_t SerializeNetworkMemcpyableInPlace(
    const T& item_to_serialize,
    std::vector<uint8_t>& buffer,
    const uint64_t buffer_position);

template<typename T>
inline uint64_t SerializeNetworkMemcpyable(
    const T& item_to_serialize, std::vector<uint8_t>& buffer);

template<typename T>
inline Deserialized<T> DeserializeNetworkMemcpyable(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset);

template<typename CharType>
inline uint64_t SerializeNetworkString(
    const std::basic_string<CharType>& str_to_serialize,
    std::vector<uint8_t>& buffer);

template<typename CharType>
inline Deserialized<std::basic_string<CharType>>
DeserializeNetworkString(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset);

template<typename T, typename VectorLike=std::vector<T>>
inline uint64_t SerializeNetworkVectorLike(
    const VectorLike& vec_to_serialize,
    std::vector<uint8_t>& buffer,
    const Serializer<T>& item_serializer);

template<typename T, typename VectorLike=std::vector<T>>
inline Deserialized<VectorLike> DeserializeNetworkVectorLike(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const Deserializer<T>& item_deserializer);

template<typename T, typename VectorLike=std::vector<T>>
inline uint64_t SerializeNetworkMemcpyableVectorLike(
    const VectorLike& vec_to_serialize,
    std::vector<uint8_t>& buffer);

template<typename T, typename VectorLike=std::vector<T>>
inline Deserialized<VectorLike>
DeserializeNetworkMemcpyableVectorLike(const std::vector<uint8_t>& buffer,
                                       const uint64_t starting_offset);

template<typename Key, typename Value, typename MapLike=std::map<Key, Value>>
inline uint64_t SerializeNetworkMapLike(
    const MapLike& map_to_serialize,
    std::vector<uint8_t>& buffer,
    const Serializer<Key>& key_serializer,
    const Serializer<Value>& value_serializer);

template<typename Key, typename Value, typename MapLike=std::map<Key, Value>>
inline Deserialized<MapLike> DeserializeNetworkMapLike(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const Deserializer<Key>& key_deserializer,
    const Deserializer<Value>& value_deserializer);

template<typename Key, typename SetLike=std::set<Key>>
inline uint64_t SerializeNetworkSetLike(
    const SetLike& set_to_serialize,
    std::vector<uint8_t>& buffer,
    const Serializer<Key>& key_serializer);

template<typename Key, typename SetLike=std::set<Key>>
inline Deserialized<SetLike> DeserializeNetworkSetLike(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const Deserializer<Key>& key_deserializer);

/////////////////////////////////////////////////////////////////////
/////                   IMPLEMENTATIONS ONLY                    /////
/////////////////////////////////////////////////////////////////////

template<typename T>
inline uint64_t SerializeMemcpyable(const T& item_to_serialize,
                                    std::vector<uint8_t>& buffer)
{
  const uint64_t start_buffer_size = buffer.size();
  // Fixed-size serialization via memcpy
  std::vector<uint8_t> temp_buffer(sizeof(item_to_serialize), 0x00);
  memcpy(&temp_buffer[0], &item_to_serialize, sizeof(item_to_serialize));
  // Move to buffer
  buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
  // Figure out how many bytes were written
  const uint64_t end_buffer_size = buffer.size();
  const uint64_t bytes_written = end_buffer_size - start_buffer_size;
  return bytes_written;
}

template<typename T>
inline Deserialized<T> DeserializeMemcpyable(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
{
  T temp_item;
  if (starting_offset >= buffer.size())
  {
    throw std::invalid_argument(
        "starting_offset is outside the provided buffer");
  }
  if ((starting_offset + sizeof(temp_item)) > buffer.size())
  {
    throw std::invalid_argument("Not enough room in the provided buffer");
  }
  memcpy(&temp_item, &buffer[starting_offset], sizeof(temp_item));
  return MakeDeserialized(temp_item, sizeof(temp_item));
}

template<typename CharType>
inline uint64_t SerializeString(
    const std::basic_string<CharType>& str_to_serialize,
    std::vector<uint8_t>& buffer)
{
  const uint64_t start_buffer_size = buffer.size();
  // First, write a uint64_t size header
  const uint64_t size = static_cast<uint64_t>(str_to_serialize.size());
  SerializeMemcpyable<uint64_t>(size, buffer);
  // Serialize the contained items
  for (size_t idx = 0; idx < size; idx++)
  {
    const CharType& current = str_to_serialize[idx];
    SerializeMemcpyable<CharType>(current, buffer);
  }
  // Figure out how many bytes were written
  const uint64_t end_buffer_size = buffer.size();
  const uint64_t bytes_written = end_buffer_size - start_buffer_size;
  return bytes_written;
}

template<typename CharType>
inline Deserialized<std::basic_string<CharType>> DeserializeString(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
{
  uint64_t current_position = starting_offset;
  // Load the header
  const Deserialized<uint64_t> deserialized_size
      = DeserializeMemcpyable<uint64_t>(buffer, current_position);
  const uint64_t size = deserialized_size.Value();
  current_position += deserialized_size.BytesRead();
  // Deserialize the items
  std::basic_string<CharType> deserialized;
  deserialized.reserve(size);
  for (uint64_t idx = 0; idx < size; idx++)
  {
    const Deserialized<CharType> deserialized_char
        = DeserializeMemcpyable<CharType>(buffer, current_position);
    deserialized.push_back(deserialized_char.Value());
    current_position += deserialized_char.BytesRead();
  }
  deserialized.shrink_to_fit();
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return MakeDeserialized(deserialized, bytes_read);
}

template<typename T, typename VectorLike>
inline uint64_t SerializeVectorLike(
    const VectorLike& vec_to_serialize,
    std::vector<uint8_t>& buffer,
    const Serializer<T>& item_serializer)
{
  const uint64_t start_buffer_size = buffer.size();
  // First, write a uint64_t size header
  const uint64_t size = static_cast<uint64_t>(vec_to_serialize.size());
  SerializeMemcpyable<uint64_t>(size, buffer);
  // Serialize the contained items
  for (size_t idx = 0; idx < size; idx++)
  {
    const T& current = vec_to_serialize[idx];
    item_serializer(current, buffer);
  }
  // Figure out how many bytes were written
  const uint64_t end_buffer_size = buffer.size();
  const uint64_t bytes_written = end_buffer_size - start_buffer_size;
  return bytes_written;
}

template<typename T, typename VectorLike>
inline Deserialized<VectorLike> DeserializeVectorLike(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const Deserializer<T>& item_deserializer)
{
  uint64_t current_position = starting_offset;
  // Load the header
  const Deserialized<uint64_t> deserialized_size
      = DeserializeMemcpyable<uint64_t>(buffer, current_position);
  const uint64_t size = deserialized_size.Value();
  current_position += deserialized_size.BytesRead();
  // Deserialize the items
  VectorLike deserialized;
  deserialized.reserve(size);
  for (uint64_t idx = 0; idx < size; idx++)
  {
    const Deserialized<T> deserialized_item
        = item_deserializer(buffer, current_position);
    deserialized.push_back(deserialized_item.Value());
    current_position += deserialized_item.BytesRead();
  }
  deserialized.shrink_to_fit();
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return MakeDeserialized(deserialized, bytes_read);
}

template<typename T, typename VectorLike>
inline uint64_t SerializeMemcpyableVectorLike(
    const VectorLike& vec_to_serialize,
    std::vector<uint8_t>& buffer)
{
  const uint64_t start_buffer_size = buffer.size();
  // First, write a uint64_t size header
  const uint64_t size = static_cast<uint64_t>(vec_to_serialize.size());
  SerializeMemcpyable<uint64_t>(size, buffer);
  // Expand the buffer to handle everything
  const size_t serialized_length =
      sizeof(T) * static_cast<size_t>(vec_to_serialize.size());
  const size_t previous_buffer_size = buffer.size();
  buffer.resize(previous_buffer_size + serialized_length, 0x00);
  // Only memcpy if a non-zero number of bytes will be copied, since memcpy on
  // nullptr dst/src (encountered if a vector is empty) is undefined behavior.
  if (size > 0)
  {
    memcpy(&buffer[previous_buffer_size],
           vec_to_serialize.data(), serialized_length);
  }
  // Figure out how many bytes were written
  const uint64_t end_buffer_size = buffer.size();
  const uint64_t bytes_written = end_buffer_size - start_buffer_size;
  return bytes_written;
}

template<typename T, typename VectorLike>
inline Deserialized<VectorLike>
DeserializeMemcpyableVectorLike(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset)
{
  uint64_t current_position = starting_offset;
  // Load the header
  const Deserialized<uint64_t> deserialized_size
      = DeserializeMemcpyable<uint64_t>(buffer, current_position);
  const uint64_t size = deserialized_size.Value();
  current_position += deserialized_size.BytesRead();
  // Deserialize the items
  const size_t serialized_length = sizeof(T) * size;
  if ((current_position + serialized_length) > buffer.size())
  {
    throw std::invalid_argument("Not enough room in the provided buffer");
  }
  VectorLike deserialized(size);
  // Only memcpy if a non-zero number of bytes will be copied, since memcpy on
  // nullptr dst/src (encountered if a vector is empty) is undefined behavior.
  if (size > 0)
  {
    memcpy(deserialized.data(), &buffer[current_position], serialized_length);
  }
  current_position += serialized_length;
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return MakeDeserialized(deserialized, bytes_read);
}

template<typename Key, typename Value, typename MapLike>
inline uint64_t SerializeMapLike(
    const MapLike& map_to_serialize,
    std::vector<uint8_t>& buffer,
    const Serializer<Key>& key_serializer,
    const Serializer<Value>& value_serializer)
{
  const uint64_t start_buffer_size = buffer.size();
  // First, write a uint64_t size header
  const uint64_t size = static_cast<uint64_t>(map_to_serialize.size());
  SerializeMemcpyable<uint64_t>(size, buffer);
  // Serialize the contained items
  typename MapLike::const_iterator itr;
  for (itr = map_to_serialize.begin(); itr != map_to_serialize.end(); ++itr)
  {
    SerializePair<Key, Value>(*itr, buffer, key_serializer, value_serializer);
  }
  // Figure out how many bytes were written
  const uint64_t end_buffer_size = buffer.size();
  const uint64_t bytes_written = end_buffer_size - start_buffer_size;
  return bytes_written;
}

template<typename Key, typename Value, typename MapLike>
inline Deserialized<MapLike> DeserializeMapLike(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const Deserializer<Key>& key_deserializer,
    const Deserializer<Value>& value_deserializer)
{
  uint64_t current_position = starting_offset;
  // Load the header
  const Deserialized<uint64_t> deserialized_size
      = DeserializeMemcpyable<uint64_t>(buffer, current_position);
  const uint64_t size = deserialized_size.Value();
  current_position += deserialized_size.BytesRead();
  // Deserialize the items
  MapLike deserialized;
  for (uint64_t idx = 0; idx < size; idx++)
  {
    const Deserialized<std::pair<Key, Value>> deserialized_pair
        = DeserializePair(buffer, current_position,
                          key_deserializer, value_deserializer);
    deserialized.insert(deserialized_pair.Value());
    current_position += deserialized_pair.BytesRead();
  }
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return MakeDeserialized(deserialized, bytes_read);
}

template<typename Key, typename SetLike>
inline uint64_t SerializeSetLike(
    const SetLike& set_to_serialize,
    std::vector<uint8_t>& buffer,
    const Serializer<Key>& key_serializer)
{
  const uint64_t start_buffer_size = buffer.size();
  // First, write a uint64_t size header
  const uint64_t size = static_cast<uint64_t>(set_to_serialize.size());
  SerializeMemcpyable<uint64_t>(size, buffer);
  // Serialize the contained items
  typename SetLike::const_iterator itr;
  for (itr = set_to_serialize.begin(); itr != set_to_serialize.end(); ++itr)
  {
    key_serializer(*itr, buffer);
  }
  // Figure out how many bytes were written
  const uint64_t end_buffer_size = buffer.size();
  const uint64_t bytes_written = end_buffer_size - start_buffer_size;
  return bytes_written;
}

template<typename Key, typename SetLike>
inline Deserialized<SetLike> DeserializeSetLike(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const Deserializer<Key>& key_deserializer)
{
  uint64_t current_position = starting_offset;
  // Load the header
  const Deserialized<uint64_t> deserialized_size
      = DeserializeMemcpyable<uint64_t>(buffer, current_position);
  const uint64_t size = deserialized_size.Value();
  current_position += deserialized_size.BytesRead();
  // Deserialize the items
  SetLike deserialized;
  for (uint64_t idx = 0; idx < size; idx++)
  {
    const Deserialized<Key> deserialized_key
        = key_deserializer(buffer, current_position);
    deserialized.insert(deserialized_key.Value());
    current_position += deserialized_key.BytesRead();
  }
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return MakeDeserialized(deserialized, bytes_read);
}

template<typename First, typename Second>
inline uint64_t SerializePair(
    const std::pair<First, Second>& pair_to_serialize,
    std::vector<uint8_t>& buffer,
    const Serializer<First>& first_serializer,
    const Serializer<Second>& second_serializer)
{
  const uint64_t start_buffer_size = buffer.size();
  // Write each element of the pair into the buffer
  first_serializer(pair_to_serialize.first, buffer);
  second_serializer(pair_to_serialize.second, buffer);
  // Figure out how many bytes were written
  const uint64_t end_buffer_size = buffer.size();
  const uint64_t bytes_written = end_buffer_size - start_buffer_size;
  return bytes_written;
}

template<typename First, typename Second>
inline Deserialized<std::pair<First, Second>> DeserializePair(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const Deserializer<First>& first_deserializer,
    const Deserializer<Second>& second_deserializer)
{
  // Deserialize each item in the pair individually
  uint64_t current_position = starting_offset;
  const Deserialized<First> deserialized_first
      = first_deserializer(buffer, current_position);
  current_position += deserialized_first.BytesRead();
  const Deserialized<Second> deserialized_second
      = second_deserializer(buffer, current_position);
  current_position += deserialized_second.BytesRead();
  // Build the resulting pair
  // TODO: Why can't I used make_pair here?
  const std::pair<First, Second> deserialized(deserialized_first.Value(),
                                              deserialized_second.Value());
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return MakeDeserialized(deserialized, bytes_read);
}

// Byte-order conversions. For now, only Linux and macOS are supported.
#if defined(__APPLE__)
    #include <libkern/OSByteOrder.h>
#else
    #include <endian.h>
#endif

inline uint8_t HostToNetwork8(const uint8_t value)
{
  return value;
}

inline uint16_t HostToNetwork16(const uint16_t value)
{
#if defined(__APPLE__)
  return OSSwapHostToBigInt16(value);
#else
  return htobe16(value);
#endif
}

inline uint32_t HostToNetwork32(const uint32_t value)
{
#if defined(__APPLE__)
  return OSSwapHostToBigInt32(value);
#else
  return htobe32(value);
#endif
}

inline uint64_t HostToNetwork64(const uint64_t value)
{
#if defined(__APPLE__)
  return OSSwapHostToBigInt64(value);
#else
  return htobe64(value);
#endif
}

inline uint8_t NetworkToHost8(const uint8_t value)
{
  return value;
}

inline uint16_t NetworkToHost16(const uint16_t value)
{
#if defined(__APPLE__)
  return OSSwapBigToHostInt16(value);
#else
  return be16toh(value);
#endif
}

inline uint32_t NetworkToHost32(const uint32_t value)
{
#if defined(__APPLE__)
  return OSSwapBigToHostInt32(value);
#else
  return be32toh(value);
#endif
}

inline uint64_t NetworkToHost64(const uint64_t value)
{
#if defined(__APPLE__)
  return OSSwapBigToHostInt64(value);
#else
  return be64toh(value);
#endif
}

template<typename T>
inline uint64_t SerializeNetworkMemcpyableInPlace(
    const T& item_to_serialize,
    std::vector<uint8_t>& buffer,
    const uint64_t buffer_position)
{
  static_assert((std::is_same<T, uint8_t>::value
           || std::is_same<T, uint16_t>::value
           || std::is_same<T, uint32_t>::value
           || std::is_same<T, uint64_t>::value
           || std::is_same<T, int8_t>::value
           || std::is_same<T, int16_t>::value
           || std::is_same<T, int32_t>::value
           || std::is_same<T, int64_t>::value
           || std::is_same<T, float>::value
           || std::is_same<T, double>::value),
          "Type not supported for host<->network serialization");
  // Fixed-size serialization via memcpy with fixed-size byte swap
  std::vector<uint8_t> temp_buffer(sizeof(item_to_serialize), 0x00);
  if (sizeof(T) == 1)
  {
    memcpy(&temp_buffer[0], &item_to_serialize, sizeof(item_to_serialize));
  }
  else if (sizeof(T) == 2)
  {
    uint16_t swap_temp;
    memcpy(&swap_temp, &item_to_serialize, 2);
    const uint16_t swapped = HostToNetwork16(swap_temp);
    memcpy(&temp_buffer[0], &swapped, 2);
  }
  else if (sizeof(T) == 4)
  {
    uint32_t swap_temp;
    memcpy(&swap_temp, &item_to_serialize, 4);
    const uint32_t swapped = HostToNetwork32(swap_temp);
    memcpy(&temp_buffer[0], &swapped, 4);
  }
  else if (sizeof(T) == 8)
  {
    uint64_t swap_temp;
    memcpy(&swap_temp, &item_to_serialize, 8);
    const uint64_t swapped = HostToNetwork64(swap_temp);
    memcpy(&temp_buffer[0], &swapped, 8);
  }
  // Move to buffer
  if ((buffer_position + sizeof(item_to_serialize)) <= buffer.size())
  {
    memcpy(&buffer[buffer_position],
           temp_buffer.data(),
           sizeof(item_to_serialize));
    // Figure out how many bytes were written
    return static_cast<uint64_t>(sizeof(item_to_serialize));
  }
  else
  {
    throw std::runtime_error("Not enough space to serialize in place");
  }
}

template<typename T>
inline uint64_t SerializeNetworkMemcpyable(const T& item_to_serialize,
                                           std::vector<uint8_t>& buffer)
{
  static_assert((std::is_same<T, uint8_t>::value
           || std::is_same<T, uint16_t>::value
           || std::is_same<T, uint32_t>::value
           || std::is_same<T, uint64_t>::value
           || std::is_same<T, int8_t>::value
           || std::is_same<T, int16_t>::value
           || std::is_same<T, int32_t>::value
           || std::is_same<T, int64_t>::value
           || std::is_same<T, float>::value
           || std::is_same<T, double>::value),
          "Type not supported for host<->network serialization");
  const uint64_t start_buffer_size = buffer.size();
  // Fixed-size serialization via memcpy with fixed-size byte swap
  std::vector<uint8_t> temp_buffer(sizeof(item_to_serialize), 0x00);
  if (sizeof(T) == 1)
  {
    memcpy(&temp_buffer[0], &item_to_serialize, sizeof(item_to_serialize));
  }
  else if (sizeof(T) == 2)
  {
    uint16_t swap_temp;
    memcpy(&swap_temp, &item_to_serialize, 2);
    const uint16_t swapped = HostToNetwork16(swap_temp);
    memcpy(&temp_buffer[0], &swapped, 2);
  }
  else if (sizeof(T) == 4)
  {
    uint32_t swap_temp;
    memcpy(&swap_temp, &item_to_serialize, 4);
    const uint32_t swapped = HostToNetwork32(swap_temp);
    memcpy(&temp_buffer[0], &swapped, 4);
  }
  else if (sizeof(T) == 8)
  {
    uint64_t swap_temp;
    memcpy(&swap_temp, &item_to_serialize, 8);
    const uint64_t swapped = HostToNetwork64(swap_temp);
    memcpy(&temp_buffer[0], &swapped, 8);
  }
  // Move to buffer
  buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
  // Figure out how many bytes were written
  const uint64_t end_buffer_size = buffer.size();
  const uint64_t bytes_written = end_buffer_size - start_buffer_size;
  return bytes_written;
}

template<typename T>
inline Deserialized<T> DeserializeNetworkMemcpyable(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
{
  static_assert((std::is_same<T, uint8_t>::value
           || std::is_same<T, uint16_t>::value
           || std::is_same<T, uint32_t>::value
           || std::is_same<T, uint64_t>::value
           || std::is_same<T, int8_t>::value
           || std::is_same<T, int16_t>::value
           || std::is_same<T, int32_t>::value
           || std::is_same<T, int64_t>::value
           || std::is_same<T, float>::value
           || std::is_same<T, double>::value),
          "Type not supported for host<->network deserialization");
  T temp_item;
  if (starting_offset >= buffer.size())
  {
    throw std::invalid_argument(
        "starting_offset is outside the provided buffer");
  }
  if ((starting_offset + sizeof(temp_item)) > buffer.size())
  {
    throw std::invalid_argument("Not enough room in the provided buffer");
  }
  if (sizeof(T) == 1)
  {
    memcpy(&temp_item, &buffer[starting_offset], sizeof(temp_item));
  }
  else if (sizeof(T) == 2)
  {
    uint16_t swap_temp;
    memcpy(&swap_temp, &buffer[starting_offset], 2);
    const uint16_t swapped = NetworkToHost16(swap_temp);
    memcpy(&temp_item, &swapped, 2);
  }
  else if (sizeof(T) == 4)
  {
    uint32_t swap_temp;
    memcpy(&swap_temp, &buffer[starting_offset], 4);
    const uint32_t swapped = NetworkToHost32(swap_temp);
    memcpy(&temp_item, &swapped, 4);
  }
  else if (sizeof(T) == 8)
  {
    uint64_t swap_temp;
    memcpy(&swap_temp, &buffer[starting_offset], 8);
    const uint64_t swapped = NetworkToHost64(swap_temp);
    memcpy(&temp_item, &swapped, 8);
  }
  return MakeDeserialized(temp_item, sizeof(temp_item));
}

template<typename CharType>
inline uint64_t SerializeNetworkString(
    const std::basic_string<CharType>& str_to_serialize,
    std::vector<uint8_t>& buffer)
{
  const uint64_t start_buffer_size = buffer.size();
  // First, write a uint64_t size header
  const uint64_t size = static_cast<uint64_t>(str_to_serialize.size());
  SerializeNetworkMemcpyable<uint64_t>(size, buffer);
  // Serialize the contained items
  for (size_t idx = 0; idx < size; idx++)
  {
    const CharType& current = str_to_serialize[idx];
    SerializeNetworkMemcpyable<CharType>(current, buffer);
  }
  // Figure out how many bytes were written
  const uint64_t end_buffer_size = buffer.size();
  const uint64_t bytes_written = end_buffer_size - start_buffer_size;
  return bytes_written;
}

template<typename CharType>
inline Deserialized<std::basic_string<CharType>>
DeserializeNetworkString(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset)
{
  uint64_t current_position = starting_offset;
  // Load the header
  const Deserialized<uint64_t> deserialized_size
      = DeserializeNetworkMemcpyable<uint64_t>(buffer, current_position);
  const uint64_t size = deserialized_size.Value();
  current_position += deserialized_size.BytesRead();
  // Deserialize the items
  std::basic_string<CharType> deserialized;
  deserialized.reserve(size);
  for (uint64_t idx = 0; idx < size; idx++)
  {
    const Deserialized<CharType> deserialized_char
        = DeserializeNetworkMemcpyable<CharType>(buffer, current_position);
    deserialized.push_back(deserialized_char.Value());
    current_position += deserialized_char.BytesRead();
  }
  deserialized.shrink_to_fit();
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return MakeDeserialized(deserialized, bytes_read);
}

template<typename T, typename VectorLike>
inline uint64_t SerializeNetworkVectorLike(
    const VectorLike& vec_to_serialize,
    std::vector<uint8_t>& buffer,
    const Serializer<T>& item_serializer)
{
  const uint64_t start_buffer_size = buffer.size();
  // First, write a uint64_t size header
  const uint64_t size = static_cast<uint64_t>(vec_to_serialize.size());
  SerializeNetworkMemcpyable<uint64_t>(size, buffer);
  // Serialize the contained items
  for (size_t idx = 0; idx < size; idx++)
  {
    const T& current = vec_to_serialize[idx];
    item_serializer(current, buffer);
  }
  // Figure out how many bytes were written
  const uint64_t end_buffer_size = buffer.size();
  const uint64_t bytes_written = end_buffer_size - start_buffer_size;
  return bytes_written;
}

template<typename T, typename VectorLike>
inline Deserialized<VectorLike>
DeserializeNetworkVectorLike(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const Deserializer<T>& item_deserializer)
{
  uint64_t current_position = starting_offset;
  // Load the header
  const Deserialized<uint64_t> deserialized_size
      = DeserializeNetworkMemcpyable<uint64_t>(buffer, current_position);
  const uint64_t size = deserialized_size.Value();
  current_position += deserialized_size.BytesRead();
  // Deserialize the items
  VectorLike deserialized;
  deserialized.reserve(size);
  for (uint64_t idx = 0; idx < size; idx++)
  {
    const Deserialized<T> deserialized_item
        = item_deserializer(buffer, current_position);
    deserialized.push_back(deserialized_item.Value());
    current_position += deserialized_item.BytesRead();
  }
  deserialized.shrink_to_fit();
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return MakeDeserialized(deserialized, bytes_read);
}

template<typename T, typename VectorLike>
inline uint64_t SerializeNetworkMemcpyableVectorLike(
    const VectorLike& vec_to_serialize,
    std::vector<uint8_t>& buffer)
{
  const uint64_t start_buffer_size = buffer.size();
  // First, write a uint64_t size header
  const uint64_t size = static_cast<uint64_t>(vec_to_serialize.size());
  SerializeNetworkMemcpyable<uint64_t>(size, buffer);
  // Expand the buffer to handle everything
  const size_t serialized_length = sizeof(T) * vec_to_serialize.size();
  uint64_t previous_buffer_size = buffer.size();
  buffer.resize(previous_buffer_size + serialized_length, 0x00);
  // Go through and convert to network byte order
  for (size_t idx = 0; idx < vec_to_serialize.size(); idx++)
  {
    const T& item = vec_to_serialize[idx];
    const uint64_t bytes_written
        = SerializeNetworkMemcpyableInPlace(item, buffer, previous_buffer_size);
    previous_buffer_size += bytes_written;
  }
  // Figure out how many bytes were written
  const uint64_t end_buffer_size = buffer.size();
  const uint64_t bytes_written = end_buffer_size - start_buffer_size;
  return bytes_written;
}

template<typename T, typename VectorLike>
inline Deserialized<VectorLike>
DeserializeNetworkMemcpyableVectorLike(const std::vector<uint8_t>& buffer,
                                       const uint64_t starting_offset)
{
  uint64_t current_position = starting_offset;
  // Load the header
  const Deserialized<uint64_t> deserialized_size
      = DeserializeNetworkMemcpyable<uint64_t>(buffer, current_position);
  const uint64_t size = deserialized_size.Value();
  current_position += deserialized_size.BytesRead();
  // Deserialize the items
  VectorLike deserialized(size);
  for (size_t idx = 0; idx < deserialized.size(); idx++)
  {
    const Deserialized<T> deserialized_item
        = DeserializeNetworkMemcpyable<T>(buffer, current_position);
    deserialized[idx] = deserialized_item.Value();
    current_position += deserialized_item.BytesRead();
  }
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return MakeDeserialized(deserialized, bytes_read);
}

template<typename Key, typename Value, typename MapLike>
inline uint64_t SerializeNetworkMapLike(
    const MapLike& map_to_serialize,
    std::vector<uint8_t>& buffer,
    const Serializer<Key>& key_serializer,
    const Serializer<Value>& value_serializer)
{
  const uint64_t start_buffer_size = buffer.size();
  // First, write a uint64_t size header
  const uint64_t size = static_cast<uint64_t>(map_to_serialize.size());
  SerializeNetworkMemcpyable<uint64_t>(size, buffer);
  // Serialize the contained items
  typename MapLike::const_iterator itr;
  for (itr = map_to_serialize.begin(); itr != map_to_serialize.end(); ++itr)
  {
    SerializePair<Key, Value>(*itr, buffer, key_serializer, value_serializer);
  }
  // Figure out how many bytes were written
  const uint64_t end_buffer_size = buffer.size();
  const uint64_t bytes_written = end_buffer_size - start_buffer_size;
  return bytes_written;
}

template<typename Key, typename Value, typename MapLike>
inline Deserialized<MapLike> DeserializeNetworkMapLike(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const Deserializer<Key>& key_deserializer,
    const Deserializer<Value>& value_deserializer)
{
  uint64_t current_position = starting_offset;
  // Load the header
  const Deserialized<uint64_t> deserialized_size
      = DeserializeNetworkMemcpyable<uint64_t>(buffer, current_position);
  const uint64_t size = deserialized_size.Value();
  current_position += deserialized_size.BytesRead();
  // Deserialize the items
  MapLike deserialized;
  for (uint64_t idx = 0; idx < size; idx++)
  {
    const Deserialized<std::pair<Key, Value>> deserialized_pair
        = DeserializePair(buffer, current_position,
                          key_deserializer, value_deserializer);
    deserialized.insert(deserialized_pair.Value());
    current_position += deserialized_pair.BytesRead();
  }
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return MakeDeserialized(deserialized, bytes_read);
}

template<typename Key, typename SetLike>
inline uint64_t SerializeNetworkSetLike(
    const SetLike& set_to_serialize,
    std::vector<uint8_t>& buffer,
    const Serializer<Key>& key_serializer)
{
  const uint64_t start_buffer_size = buffer.size();
  // First, write a uint64_t size header
  const uint64_t size = static_cast<uint64_t>(set_to_serialize.size());
  SerializeNetworkMemcpyable<uint64_t>(size, buffer);
  // Serialize the contained items
  typename SetLike::const_iterator itr;
  for (itr = set_to_serialize.begin(); itr != set_to_serialize.end(); ++itr)
  {
    key_serializer(*itr, buffer);
  }
  // Figure out how many bytes were written
  const uint64_t end_buffer_size = buffer.size();
  const uint64_t bytes_written = end_buffer_size - start_buffer_size;
  return bytes_written;
}

template<typename Key, typename SetLike>
inline Deserialized<SetLike> DeserializeNetworkSetLike(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const Deserializer<Key>& key_deserializer)
{
  uint64_t current_position = starting_offset;
  // Load the header
  const Deserialized<uint64_t> deserialized_size
      = DeserializeNetworkMemcpyable<uint64_t>(buffer, current_position);
  const uint64_t size = deserialized_size.Value();
  current_position += deserialized_size.BytesRead();
  // Deserialize the items
  SetLike deserialized;
  for (uint64_t idx = 0; idx < size; idx++)
  {
    const Deserialized<Key> deserialized_key
        = key_deserializer(buffer, current_position);
    deserialized.insert(deserialized_key.Value());
    current_position += deserialized_key.BytesRead();
  }
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return MakeDeserialized(deserialized, bytes_read);
}
}  // namespace serialization
}  // namespace common_robotics_utilities
