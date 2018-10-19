#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

namespace common_robotics_utilities
{
namespace serialization
{
///////////////////////////////////////////////////////////////
/////                     PROTOTYPES ONLY                 /////
///////////////////////////////////////////////////////////////

uint64_t SerializeVectorXd(const Eigen::VectorXd& value,
                           std::vector<uint8_t>& buffer);

uint64_t SerializeVector2d(const Eigen::Vector2d& value,
                           std::vector<uint8_t>& buffer);

uint64_t SerializeVector3d(const Eigen::Vector3d& value,
                           std::vector<uint8_t>& buffer);

uint64_t SerializeVector4d(const Eigen::Vector4d& value,
                           std::vector<uint8_t>& buffer);

uint64_t SerializeQuaterniond(const Eigen::Quaterniond& value,
                              std::vector<uint8_t>& buffer);

uint64_t SerializeIsometry3d(const Eigen::Isometry3d& value,
                             std::vector<uint8_t>& buffer);

std::pair<Eigen::VectorXd, uint64_t> DeserializeVectorXd(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset);

std::pair<Eigen::Vector2d, uint64_t> DeserializeVector2d(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset);

std::pair<Eigen::Vector3d, uint64_t> DeserializeVector3d(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset);

std::pair<Eigen::Vector4d, uint64_t> DeserializeVector4d(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset);

std::pair<Eigen::Quaterniond, uint64_t> DeserializeQuaterniond(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset);

std::pair<Eigen::Isometry3d, uint64_t> DeserializeIsometry3d(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset);

template<typename T>
inline uint64_t SerializeMemcpyable(const T& item_to_serialize,
                                    std::vector<uint8_t>& buffer);

template<typename T>
inline std::pair<T, uint64_t> DeserializeMemcpyable(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset);

template<typename CharType>
inline uint64_t SerializeString(
    const std::basic_string<CharType>& str_to_serialize,
    std::vector<uint8_t>& buffer);

template<typename CharType>
inline std::pair<std::basic_string<CharType>, uint64_t> DeserializeString(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset);

template<typename T, typename VectorLike=std::vector<T>>
inline uint64_t SerializeVectorLike(
    const VectorLike& vec_to_serialize,
    std::vector<uint8_t>& buffer,
    const std::function<uint64_t
      (const T&, std::vector<uint8_t>&)>& item_serializer);

template<typename T, typename VectorLike=std::vector<T>>
inline std::pair<VectorLike, uint64_t> DeserializeVectorLike(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const std::function<std::pair<T, uint64_t>
      (const std::vector<uint8_t>&, const uint64_t)>& item_deserializer);

template<typename T, typename VectorLike=std::vector<T>>
inline uint64_t SerializeMemcpyableVectorLike(
    const VectorLike& vec_to_serialize,
    std::vector<uint8_t>& buffer);

template<typename T, typename VectorLike=std::vector<T>>
inline std::pair<VectorLike, uint64_t>
DeserializeMemcpyableVectorLike(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset);

template<typename Key, typename T,
         typename Compare = std::less<Key>,
         typename Allocator = std::allocator<std::pair<const Key, T>>>
inline uint64_t SerializeMap(
    const std::map<Key, T, Compare, Allocator>& map_to_serialize,
    std::vector<uint8_t>& buffer,
    const std::function<uint64_t
      (const Key&, std::vector<uint8_t>&)>& key_serializer,
    const std::function<uint64_t
      (const T&, std::vector<uint8_t>&)>& value_serializer);

template<typename Key, typename T,
         typename Compare = std::less<Key>,
         typename Allocator = std::allocator<std::pair<const Key, T>>>
inline std::pair<std::map<Key, T, Compare, Allocator>, uint64_t> DeserializeMap(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const std::function<std::pair<Key, uint64_t>
      (const std::vector<uint8_t>&, const uint64_t)>& key_deserializer,
    const std::function<std::pair<T, uint64_t>
      (const std::vector<uint8_t>&, const uint64_t)>& value_deserializer);

template<typename Key, typename T,
         typename Hash = std::hash<Key>,
         typename KeyEqual = std::equal_to<Key>,
         typename Allocator = std::allocator<std::pair<const Key, T>>>
inline uint64_t SerializeUnorderedMap(
    const std::unordered_map<Key, T, Hash, KeyEqual, Allocator>&
        map_to_serialize,
    std::vector<uint8_t>& buffer,
    const std::function<uint64_t
      (const Key&, std::vector<uint8_t>&)>& key_serializer,
    const std::function<uint64_t
      (const T&, std::vector<uint8_t>&)>& value_serializer);

template<typename Key, typename T,
         typename Hash = std::hash<Key>,
         typename KeyEqual = std::equal_to<Key>,
         typename Allocator = std::allocator<std::pair<const Key, T>>>
inline std::pair<std::unordered_map<Key, T, Hash, KeyEqual, Allocator>,
                 uint64_t>
DeserializeUnorderedMap(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const std::function<std::pair<Key, uint64_t>
      (const std::vector<uint8_t>&, const uint64_t)>& key_deserializer,
    const std::function<std::pair<T, uint64_t>
      (const std::vector<uint8_t>&, const uint64_t)>& value_deserializer);

template<typename First, typename Second>
inline uint64_t SerializePair(
    const std::pair<First, Second>& pair_to_serialize,
    std::vector<uint8_t>& buffer,
    const std::function<uint64_t
      (const First&, std::vector<uint8_t>&)>& first_serializer,
    const std::function<uint64_t
      (const Second&, std::vector<uint8_t>&)>& second_serializer);

template<typename First, typename Second>
inline const std::pair<std::pair<First, Second>, uint64_t> DeserializePair(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const std::function<std::pair<First, uint64_t>
      (const std::vector<uint8_t>&, const uint64_t)>& first_deserializer,
    const std::function<std::pair<Second, uint64_t>
      (const std::vector<uint8_t>&, const uint64_t)>& second_deserializer);

template<typename T>
inline uint64_t SerializeNetworkMemcpyableInPlace(
    const T& item_to_serialize,
    std::vector<uint8_t>& buffer,
    const uint64_t buffer_position);

template<typename T>
inline uint64_t SerializeNetworkMemcpyable(
    const T& item_to_serialize, std::vector<uint8_t>& buffer);

template<typename T>
inline std::pair<T, uint64_t> DeserializeNetworkMemcpyable(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset);

template<typename CharType>
inline uint64_t SerializeNetworkString(
    const std::basic_string<CharType>& str_to_serialize,
    std::vector<uint8_t>& buffer);

template<typename CharType>
inline std::pair<std::basic_string<CharType>, uint64_t>
DeserializeNetworkString(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset);

template<typename T, typename VectorLike=std::vector<T>>
inline uint64_t SerializeNetworkVectorLike(
    const VectorLike& vec_to_serialize,
    std::vector<uint8_t>& buffer,
    const std::function<uint64_t
      (const T&, std::vector<uint8_t>&)>& item_serializer);

template<typename T, typename VectorLike=std::vector<T>>
inline std::pair<VectorLike, uint64_t> DeserializeNetworkVectorLike(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const std::function<std::pair<T, uint64_t>
      (const std::vector<uint8_t>&, const uint64_t)>& item_deserializer);

template<typename T, typename VectorLike=std::vector<T>>
inline uint64_t SerializeNetworkMemcpyableVectorLike(
    const VectorLike& vec_to_serialize,
    std::vector<uint8_t>& buffer);

template<typename T, typename VectorLike=std::vector<T>>
inline std::pair<VectorLike, uint64_t>
DeserializeNetworkMemcpyableVectorLike(const std::vector<uint8_t>& buffer,
                                       const uint64_t starting_offset);

template<typename Key, typename T,
         typename Compare = std::less<Key>,
         typename Allocator = std::allocator<std::pair<const Key, T>>>
inline uint64_t SerializeNetworkMap(
    const std::map<Key, T, Compare, Allocator>& map_to_serialize,
    std::vector<uint8_t>& buffer,
    const std::function<uint64_t
      (const Key&, std::vector<uint8_t>&)>& key_serializer,
    const std::function<uint64_t
      (const T&, std::vector<uint8_t>&)>& value_serializer);

template<typename Key, typename T,
         typename Compare = std::less<Key>,
         typename Allocator = std::allocator<std::pair<const Key, T>>>
inline std::pair<std::map<Key, T, Compare, Allocator>, uint64_t>
DeserializeNetworkMap(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const std::function<std::pair<Key, uint64_t>
      (const std::vector<uint8_t>&, const uint64_t)>& key_deserializer,
    const std::function<std::pair<T, uint64_t>
      (const std::vector<uint8_t>&, const uint64_t)>& value_deserializer);

template<typename Key, typename T,
         typename Hash = std::hash<Key>,
         typename KeyEqual = std::equal_to<Key>,
         typename Allocator = std::allocator<std::pair<const Key, T>>>
inline uint64_t SerializeNetworkUnorderedMap(
    const std::unordered_map<Key, T, Hash, KeyEqual, Allocator>&
        map_to_serialize,
    std::vector<uint8_t>& buffer,
    const std::function<uint64_t
      (const Key&, std::vector<uint8_t>&)>& key_serializer,
    const std::function<uint64_t
      (const T&, std::vector<uint8_t>&)>& value_serializer);

template<typename Key, typename T,
         typename Hash = std::hash<Key>,
         typename KeyEqual = std::equal_to<Key>,
         typename Allocator = std::allocator<std::pair<const Key, T>>>
inline std::pair<std::unordered_map<Key, T, Hash, KeyEqual, Allocator>,
                 uint64_t>
DeserializeNetworkUnorderedMap(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const std::function<std::pair<Key, uint64_t>
      (const std::vector<uint8_t>&, const uint64_t)>& key_deserializer,
    const std::function<std::pair<T, uint64_t>
      (const std::vector<uint8_t>&, const uint64_t)>& value_deserializer);

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
inline std::pair<T, uint64_t> DeserializeMemcpyable(
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
  return std::make_pair(temp_item, sizeof(temp_item));
}

template<typename CharType>
inline uint64_t SerializeString(
    const std::basic_string<CharType>& str_to_serialize,
    std::vector<uint8_t>& buffer)
{
  const uint64_t start_buffer_size = buffer.size();
  // First, write a uint64_t size header
  const uint64_t size = (uint64_t)str_to_serialize.size();
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
inline std::pair<std::basic_string<CharType>, uint64_t> DeserializeString(
    const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
{
  // First, try to load the header
  if (starting_offset >= buffer.size())
  {
    throw std::invalid_argument(
        "starting_offset is outside the provided buffer");
  }
  uint64_t current_position = starting_offset;
  // Load the header
  const std::pair<uint64_t, uint64_t> deserialized_size
      = DeserializeMemcpyable<uint64_t>(buffer, current_position);
  const uint64_t size = deserialized_size.first;
  current_position += deserialized_size.second;
  // Deserialize the items
  std::basic_string<CharType> deserialized;
  deserialized.reserve(size);
  for (uint64_t idx = 0; idx < size; idx++)
  {
    const std::pair<CharType, uint64_t> deserialized_char
        = DeserializeMemcpyable<CharType>(buffer, current_position);
    deserialized.push_back(deserialized_char.first);
    current_position += deserialized_char.second;
  }
  deserialized.shrink_to_fit();
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return std::make_pair(deserialized, bytes_read);
}

template<typename T, typename VectorLike>
inline uint64_t SerializeVectorLike(
    const VectorLike& vec_to_serialize,
    std::vector<uint8_t>& buffer,
    const std::function<uint64_t
      (const T&, std::vector<uint8_t>&)>& item_serializer)
{
  const uint64_t start_buffer_size = buffer.size();
  // First, write a uint64_t size header
  const uint64_t size = (uint64_t)vec_to_serialize.size();
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
inline std::pair<VectorLike, uint64_t> DeserializeVectorLike(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const std::function<std::pair<T, uint64_t>
      (const std::vector<uint8_t>&, const uint64_t)>& item_deserializer)
{
  // First, try to load the header
  if (starting_offset >= buffer.size())
  {
    throw std::invalid_argument(
        "starting_offset is outside the provided buffer");
  }
  uint64_t current_position = starting_offset;
  // Load the header
  const std::pair<uint64_t, uint64_t> deserialized_size
      = DeserializeMemcpyable<uint64_t>(buffer, current_position);
  const uint64_t size = deserialized_size.first;
  current_position += deserialized_size.second;
  // Deserialize the items
  VectorLike deserialized;
  deserialized.reserve(size);
  for (uint64_t idx = 0; idx < size; idx++)
  {
    const std::pair<T, uint64_t> deserialized_item
        = item_deserializer(buffer, current_position);
    deserialized.push_back(deserialized_item.first);
    current_position += deserialized_item.second;
  }
  deserialized.shrink_to_fit();
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return std::make_pair(deserialized, bytes_read);
}

template<typename T, typename VectorLike>
inline uint64_t SerializeMemcpyableVectorLike(
    const VectorLike& vec_to_serialize,
    std::vector<uint8_t>& buffer)
{
  const uint64_t start_buffer_size = buffer.size();
  // First, write a uint64_t size header
  const uint64_t size = (uint64_t)vec_to_serialize.size();
  SerializeMemcpyable<uint64_t>(size, buffer);
  // Expand the buffer to handle everything
  const size_t serialized_length = sizeof(T) * vec_to_serialize.size();
  const size_t previous_buffer_size = buffer.size();
  buffer.resize(previous_buffer_size + serialized_length, 0x00);
  // Serialize the contained items
  memcpy(&buffer[previous_buffer_size],
         vec_to_serialize.data(), serialized_length);
  // Figure out how many bytes were written
  const uint64_t end_buffer_size = buffer.size();
  const uint64_t bytes_written = end_buffer_size - start_buffer_size;
  return bytes_written;
}

template<typename T, typename VectorLike>
inline std::pair<VectorLike, uint64_t>
DeserializeMemcpyableVectorLike(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset)
{
  // First, try to load the header
  if (starting_offset >= buffer.size())
  {
    throw std::invalid_argument(
        "starting_offset is outside the provided buffer");
  }
  uint64_t current_position = starting_offset;
  // Load the header
  const std::pair<uint64_t, uint64_t> deserialized_size
      = DeserializeMemcpyable<uint64_t>(buffer, current_position);
  const uint64_t size = deserialized_size.first;
  current_position += deserialized_size.second;
  // Deserialize the items
  const size_t serialized_length = sizeof(T) * size;
  VectorLike deserialized(size);
  memcpy(deserialized.data(), &buffer[current_position], serialized_length);
  current_position += serialized_length;
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return std::make_pair(deserialized, bytes_read);
}

template<typename Key, typename T, typename Compare, typename Allocator>
inline uint64_t SerializeMap(
    const std::map<Key, T, Compare, Allocator>& map_to_serialize,
    std::vector<uint8_t>& buffer,
    const std::function<uint64_t
      (const Key&, std::vector<uint8_t>&)>& key_serializer,
    const std::function<uint64_t
      (const T&, std::vector<uint8_t>&)>& value_serializer)
{
  const uint64_t start_buffer_size = buffer.size();
  // First, write a uint64_t size header
  const uint64_t size = (uint64_t)map_to_serialize.size();
  SerializeMemcpyable<uint64_t>(size, buffer);
  // Serialize the contained items
  typename std::map<Key, T, Compare, Allocator>::const_iterator itr;
  for (itr = map_to_serialize.begin(); itr != map_to_serialize.end(); ++itr)
  {
    SerializePair<Key, T>(*itr, buffer, key_serializer, value_serializer);
  }
  // Figure out how many bytes were written
  const uint64_t end_buffer_size = buffer.size();
  const uint64_t bytes_written = end_buffer_size - start_buffer_size;
  return bytes_written;
}

template<typename Key, typename T, typename Compare, typename Allocator>
inline std::pair<std::map<Key, T, Compare, Allocator>, uint64_t> DeserializeMap(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const std::function<std::pair<Key, uint64_t>
      (const std::vector<uint8_t>&, const uint64_t)>& key_deserializer,
    const std::function<std::pair<T, uint64_t>
      (const std::vector<uint8_t>&, const uint64_t)>& value_deserializer)
{
  // First, try to load the header
  if (starting_offset >= buffer.size())
  {
    throw std::invalid_argument(
        "starting_offset is outside the provided buffer");
  }
  uint64_t current_position = starting_offset;
  // Load the header
  const std::pair<uint64_t, uint64_t> deserialized_size
      = DeserializeMemcpyable<uint64_t>(buffer, current_position);
  const uint64_t size = deserialized_size.first;
  current_position += deserialized_size.second;
  // Deserialize the items
  std::map<Key, T, Compare, Allocator> deserialized;
  for (uint64_t idx = 0; idx < size; idx++)
  {
    std::pair<std::pair<Key, T>, uint64_t> deserialized_pair
        = DeserializePair(buffer, current_position,
                          key_deserializer, value_deserializer);
    deserialized.insert(deserialized_pair.first);
    current_position += deserialized_pair.second;
  }
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return std::make_pair(deserialized, bytes_read);
}

template<typename Key, typename T, typename Hash, typename KeyEqual,
         typename Allocator>
inline uint64_t SerializeUnorderedMap(
    const std::unordered_map<Key, T, Hash, KeyEqual, Allocator>&
        map_to_serialize,
    std::vector<uint8_t>& buffer,
    const std::function<uint64_t
      (const Key&, std::vector<uint8_t>&)>& key_serializer,
    const std::function<uint64_t
      (const T&, std::vector<uint8_t>&)>& value_serializer)
{
  const uint64_t start_buffer_size = buffer.size();
  // First, write a uint64_t size header
  const uint64_t size = (uint64_t)map_to_serialize.size();
  SerializeMemcpyable<uint64_t>(size, buffer);
  // Serialize the contained items
  typename std::unordered_map<Key, T, Hash, KeyEqual, Allocator>::const_iterator
      itr;
  for (itr = map_to_serialize.begin(); itr != map_to_serialize.end(); ++itr)
  {
    SerializePair<Key, T>(*itr, buffer, key_serializer, value_serializer);
  }
  // Figure out how many bytes were written
  const uint64_t end_buffer_size = buffer.size();
  const uint64_t bytes_written = end_buffer_size - start_buffer_size;
  return bytes_written;
}

template<typename Key, typename T, typename Hash, typename KeyEqual,
         typename Allocator>
inline std::pair<std::unordered_map<Key, T, Hash, KeyEqual, Allocator>,
                 uint64_t>
DeserializeUnorderedMap(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const std::function<std::pair<Key, uint64_t>
      (const std::vector<uint8_t>&, const uint64_t)>& key_deserializer,
    const std::function<std::pair<T, uint64_t>
      (const std::vector<uint8_t>&, const uint64_t)>& value_deserializer)
{
  // First, try to load the header
  if (starting_offset >= buffer.size())
  {
    throw std::invalid_argument(
        "starting_offset is outside the provided buffer");
  }
  uint64_t current_position = starting_offset;
  // Load the header
  const std::pair<uint64_t, uint64_t> deserialized_size
      = DeserializeMemcpyable<uint64_t>(buffer, current_position);
  const uint64_t size = deserialized_size.first;
  current_position += deserialized_size.second;
  // Deserialize the items
  std::unordered_map<Key, T, Hash, KeyEqual, Allocator> deserialized;
  for (uint64_t idx = 0; idx < size; idx++)
  {
    std::pair<std::pair<Key, T>, uint64_t> deserialized_pair
        = DeserializePair(buffer, current_position,
                          key_deserializer, value_deserializer);
    deserialized.insert(deserialized_pair.first);
    current_position += deserialized_pair.second;
  }
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return std::make_pair(deserialized, bytes_read);
}

template<typename First, typename Second>
inline uint64_t SerializePair(
    const std::pair<First, Second>& pair_to_serialize,
    std::vector<uint8_t>& buffer,
    const std::function<uint64_t
      (const First&, std::vector<uint8_t>&)>& first_serializer,
    const std::function<uint64_t
      (const Second&, std::vector<uint8_t>&)>& second_serializer)
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
inline const std::pair<std::pair<First, Second>, uint64_t> DeserializePair(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const std::function<std::pair<First, uint64_t>
      (const std::vector<uint8_t>&, const uint64_t)>& first_deserializer,
    const std::function<std::pair<Second, uint64_t>
      (const std::vector<uint8_t>&, const uint64_t)>& second_deserializer)
{
  if (starting_offset >= buffer.size())
  {
    throw std::invalid_argument(
        "starting_offset is outside the provided buffer");
  }
  // Deserialize each item in the pair individually
  uint64_t current_position = starting_offset;
  const std::pair<First, uint64_t> deserialized_first
      = first_deserializer(buffer, current_position);
  current_position += deserialized_first.second;
  const std::pair<Second, uint64_t> deserialized_second
      = second_deserializer(buffer, current_position);
  current_position += deserialized_second.second;
  // Build the resulting pair
  // TODO: Why can't I used make_pair here?
  const std::pair<First, Second> deserialized(deserialized_first.first,
                                              deserialized_second.first);
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return std::make_pair(deserialized, bytes_read);
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
    const uint16_t swapped = htobe16(swap_temp);
    memcpy(&temp_buffer[0], &swapped, 2);
  }
  else if (sizeof(T) == 4)
  {
    uint32_t swap_temp;
    memcpy(&swap_temp, &item_to_serialize, 4);
    const uint32_t swapped = htobe32(swap_temp);
    memcpy(&temp_buffer[0], &swapped, 4);
  }
  else if (sizeof(T) == 8)
  {
    uint64_t swap_temp;
    memcpy(&swap_temp, &item_to_serialize, 8);
    const uint64_t swapped = htobe64(swap_temp);
    memcpy(&temp_buffer[0], &swapped, 8);
  }
  // Move to buffer
  if ((buffer_position + sizeof(item_to_serialize)) <= buffer.size())
  {
    memcpy(&buffer[buffer_position],
           temp_buffer.data(),
           sizeof(item_to_serialize));
    // Figure out how many bytes were written
    return (uint64_t)sizeof(item_to_serialize);
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
    const uint16_t swapped = htobe16(swap_temp);
    memcpy(&temp_buffer[0], &swapped, 2);
  }
  else if (sizeof(T) == 4)
  {
    uint32_t swap_temp;
    memcpy(&swap_temp, &item_to_serialize, 4);
    const uint32_t swapped = htobe32(swap_temp);
    memcpy(&temp_buffer[0], &swapped, 4);
  }
  else if (sizeof(T) == 8)
  {
    uint64_t swap_temp;
    memcpy(&swap_temp, &item_to_serialize, 8);
    const uint64_t swapped = htobe64(swap_temp);
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
inline std::pair<T, uint64_t> DeserializeNetworkMemcpyable(
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
    const uint16_t swapped = be16toh(swap_temp);
    memcpy(&temp_item, &swapped, 2);
  }
  else if (sizeof(T) == 4)
  {
    uint32_t swap_temp;
    memcpy(&swap_temp, &buffer[starting_offset], 4);
    const uint32_t swapped = be32toh(swap_temp);
    memcpy(&temp_item, &swapped, 4);
  }
  else if (sizeof(T) == 8)
  {
    uint64_t swap_temp;
    memcpy(&swap_temp, &buffer[starting_offset], 8);
    const uint64_t swapped = be64toh(swap_temp);
    memcpy(&temp_item, &swapped, 8);
  }
  return std::make_pair(temp_item, sizeof(temp_item));
}

template<typename CharType>
inline uint64_t SerializeNetworkString(
    const std::basic_string<CharType>& str_to_serialize,
    std::vector<uint8_t>& buffer)
{
  const uint64_t start_buffer_size = buffer.size();
  // First, write a uint64_t size header
  const uint64_t size = (uint64_t)str_to_serialize.size();
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
inline std::pair<std::basic_string<CharType>, uint64_t>
DeserializeNetworkString(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset)
{
  // First, try to load the header
  if (starting_offset >= buffer.size())
  {
    throw std::invalid_argument(
        "starting_offset is outside the provided buffer");
  }
  uint64_t current_position = starting_offset;
  // Load the header
  const std::pair<uint64_t, uint64_t> deserialized_size
      = DeserializeNetworkMemcpyable<uint64_t>(buffer, current_position);
  const uint64_t size = deserialized_size.first;
  current_position += deserialized_size.second;
  // Deserialize the items
  std::basic_string<CharType> deserialized;
  deserialized.reserve(size);
  for (uint64_t idx = 0; idx < size; idx++)
  {
    const std::pair<CharType, uint64_t> deserialized_char
        = DeserializeNetworkMemcpyable<CharType>(buffer, current_position);
    deserialized.push_back(deserialized_char.first);
    current_position += deserialized_char.second;
  }
  deserialized.shrink_to_fit();
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return std::make_pair(deserialized, bytes_read);
}

template<typename T, typename VectorLike>
inline uint64_t SerializeNetworkVectorLike(
    const VectorLike& vec_to_serialize,
    std::vector<uint8_t>& buffer,
    const std::function<uint64_t
      (const T&, std::vector<uint8_t>&)>& item_serializer)
{
  const uint64_t start_buffer_size = buffer.size();
  // First, write a uint64_t size header
  const uint64_t size = (uint64_t)vec_to_serialize.size();
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
inline std::pair<VectorLike, uint64_t>
DeserializeNetworkVectorLike(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const std::function<std::pair<T, uint64_t>
      (const std::vector<uint8_t>&, const uint64_t)>& item_deserializer)
{
  // First, try to load the header
  if (starting_offset >= buffer.size())
  {
    throw std::invalid_argument(
        "starting_offset is outside the provided buffer");
  }
  uint64_t current_position = starting_offset;
  // Load the header
  const std::pair<uint64_t, uint64_t> deserialized_size
      = DeserializeNetworkMemcpyable<uint64_t>(buffer, current_position);
  const uint64_t size = deserialized_size.first;
  current_position += deserialized_size.second;
  // Deserialize the items
  VectorLike deserialized;
  deserialized.reserve(size);
  for (uint64_t idx = 0; idx < size; idx++)
  {
    const std::pair<T, uint64_t> deserialized_item
        = item_deserializer(buffer, current_position);
    deserialized.push_back(deserialized_item.first);
    current_position += deserialized_item.second;
  }
  deserialized.shrink_to_fit();
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return std::make_pair(deserialized, bytes_read);
}

template<typename T, typename VectorLike>
inline uint64_t SerializeNetworkMemcpyableVectorLike(
    const VectorLike& vec_to_serialize,
    std::vector<uint8_t>& buffer)
{
  const uint64_t start_buffer_size = buffer.size();
  // First, write a uint64_t size header
  const uint64_t size = (uint64_t)vec_to_serialize.size();
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
inline std::pair<VectorLike, uint64_t>
DeserializeNetworkMemcpyableVectorLike(const std::vector<uint8_t>& buffer,
                                       const uint64_t starting_offset)
{
  // First, try to load the header
  if (starting_offset >= buffer.size())
  {
    throw std::invalid_argument(
        "starting_offset is outside the provided buffer");
  }
  uint64_t current_position = starting_offset;
  // Load the header
  const std::pair<uint64_t, uint64_t> deserialized_size
      = DeserializeNetworkMemcpyable<uint64_t>(buffer, current_position);
  const uint64_t size = deserialized_size.first;
  current_position += deserialized_size.second;
  // Deserialize the items
  VectorLike deserialized(size);
  for (size_t idx = 0; idx < deserialized.size(); idx++)
  {
    const std::pair<T, uint64_t> deserialized_item
        = DeserializeNetworkMemcpyable<T>(buffer, current_position);
    deserialized[idx] = deserialized_item.first;
    current_position += deserialized_item.second;
  }
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return std::make_pair(deserialized, bytes_read);
}

template<typename Key, typename T, typename Compare, typename Allocator>
inline uint64_t SerializeNetworkMap(
    const std::map<Key, T, Compare, Allocator>& map_to_serialize,
    std::vector<uint8_t>& buffer,
    const std::function<uint64_t
      (const Key&, std::vector<uint8_t>&)>& key_serializer,
    const std::function<uint64_t
      (const T&, std::vector<uint8_t>&)>& value_serializer)
{
  const uint64_t start_buffer_size = buffer.size();
  // First, write a uint64_t size header
  const uint64_t size = (uint64_t)map_to_serialize.size();
  SerializeNetworkMemcpyable<uint64_t>(size, buffer);
  // Serialize the contained items
  typename std::map<Key, T, Compare, Allocator>::const_iterator itr;
  for (itr = map_to_serialize.begin(); itr != map_to_serialize.end(); ++itr)
  {
    SerializePair<Key, T>(*itr, buffer, key_serializer, value_serializer);
  }
  // Figure out how many bytes were written
  const uint64_t end_buffer_size = buffer.size();
  const uint64_t bytes_written = end_buffer_size - start_buffer_size;
  return bytes_written;
}

template<typename Key, typename T, typename Compare, typename Allocator>
inline std::pair<std::map<Key, T, Compare, Allocator>, uint64_t>
DeserializeNetworkMap(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const std::function<std::pair<Key, uint64_t>
      (const std::vector<uint8_t>&, const uint64_t)>& key_deserializer,
    const std::function<std::pair<T, uint64_t>
    (const std::vector<uint8_t>&, const uint64_t)>& value_deserializer)
{
  // First, try to load the header
  if (starting_offset >= buffer.size())
  {
    throw std::invalid_argument(
        "starting_offset is outside the provided buffer");
  }
  uint64_t current_position = starting_offset;
  // Load the header
  const std::pair<uint64_t, uint64_t> deserialized_size
      = DeserializeNetworkMemcpyable<uint64_t>(buffer, current_position);
  const uint64_t size = deserialized_size.first;
  current_position += deserialized_size.second;
  // Deserialize the items
  std::map<Key, T, Compare, Allocator> deserialized;
  for (uint64_t idx = 0; idx < size; idx++)
  {
    std::pair<std::pair<Key, T>, uint64_t> deserialized_pair
        = DeserializePair(buffer, current_position,
                          key_deserializer, value_deserializer);
    deserialized.insert(deserialized_pair.first);
    current_position += deserialized_pair.second;
  }
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return std::make_pair(deserialized, bytes_read);
}

template<typename Key, typename T, typename Hash, typename KeyEqual,
         typename Allocator>
inline uint64_t SerializeNetworkUnorderedMap(
    const std::unordered_map<Key, T, Hash, KeyEqual, Allocator>&
        map_to_serialize,
    std::vector<uint8_t>& buffer,
    const std::function<uint64_t
      (const Key&, std::vector<uint8_t>&)>& key_serializer,
    const std::function<uint64_t
      (const T&, std::vector<uint8_t>&)>& value_serializer)
{
  const uint64_t start_buffer_size = buffer.size();
  // First, write a uint64_t size header
  const uint64_t size = (uint64_t)map_to_serialize.size();
  SerializeNetworkMemcpyable<uint64_t>(size, buffer);
  // Serialize the contained items
  typename std::unordered_map<Key, T, Hash, KeyEqual, Allocator>::const_iterator
      itr;
  for (itr = map_to_serialize.begin(); itr != map_to_serialize.end(); ++itr)
  {
    SerializePair<Key, T>(*itr, buffer, key_serializer, value_serializer);
  }
  // Figure out how many bytes were written
  const uint64_t end_buffer_size = buffer.size();
  const uint64_t bytes_written = end_buffer_size - start_buffer_size;
  return bytes_written;
}

template<typename Key, typename T, typename Hash, typename KeyEqual,
         typename Allocator>
inline std::pair<std::unordered_map<Key, T, Hash, KeyEqual, Allocator>,
                 uint64_t>
DeserializeNetworkUnorderedMap(
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const std::function<std::pair<Key, uint64_t>
      (const std::vector<uint8_t>&, const uint64_t)>& key_deserializer,
    const std::function<std::pair<T, uint64_t>
      (const std::vector<uint8_t>&, const uint64_t)>& value_deserializer)
{
  // First, try to load the header
  if (starting_offset >= buffer.size())
  {
    throw std::invalid_argument(
        "starting_offset is outside the provided buffer");
  }
  uint64_t current_position = starting_offset;
  // Load the header
  const std::pair<uint64_t, uint64_t> deserialized_size
      = DeserializeNetworkMemcpyable<uint64_t>(buffer, current_position);
  const uint64_t size = deserialized_size.first;
  current_position += deserialized_size.second;
  // Deserialize the items
  std::unordered_map<Key, T, Hash, KeyEqual, Allocator> deserialized;
  for (uint64_t idx = 0; idx < size; idx++)
  {
    std::pair<std::pair<Key, T>, uint64_t> deserialized_pair
        = DeserializePair(buffer, current_position,
                          key_deserializer, value_deserializer);
    deserialized.insert(deserialized_pair.first);
    current_position += deserialized_pair.second;
  }
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return std::make_pair(deserialized, bytes_read);
}
}  // namespace serialization
}  // namespace common_robotics_utilities
