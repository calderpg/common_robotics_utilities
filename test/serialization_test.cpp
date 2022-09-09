#include <cstdint>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/serialization.hpp>
#include <common_robotics_utilities/utility.hpp>

#include <gtest/gtest.h>

// Define helper types that are copy- or move-only, and memcpy-able or complex.
namespace common_robotics_utilities
{
namespace serialization_test
{
class CopyMoveMemcpyable
{
public:
  static uint64_t Serialize(
      const CopyMoveMemcpyable& item, std::vector<uint8_t>& buffer)
  {
    return serialization::SerializeMemcpyable<int32_t>(item.Value(), buffer);
  }

  static serialization::Deserialized<CopyMoveMemcpyable> Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
  {
    const auto deserialized_value =
        serialization::DeserializeMemcpyable<int32_t>(buffer, starting_offset);
    return serialization::MakeDeserialized(
        CopyMoveMemcpyable(deserialized_value.Value()),
        deserialized_value.BytesRead());
  }

  static uint64_t SerializeNetwork(
      const CopyMoveMemcpyable& item, std::vector<uint8_t>& buffer)
  {
    return serialization::SerializeNetworkMemcpyable<int32_t>(
        item.Value(), buffer);
  }

  static serialization::Deserialized<CopyMoveMemcpyable> DeserializeNetwork(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
  {
    const auto deserialized_value =
        serialization::DeserializeNetworkMemcpyable<int32_t>(
            buffer, starting_offset);
    return serialization::MakeDeserialized(
        CopyMoveMemcpyable(deserialized_value.Value()),
        deserialized_value.BytesRead());
  }

  CopyMoveMemcpyable() = default;

  explicit CopyMoveMemcpyable(const int32_t value) : value_(value) {}

  CopyMoveMemcpyable(const CopyMoveMemcpyable& other) = default;

  CopyMoveMemcpyable(CopyMoveMemcpyable&& other) = default;

  CopyMoveMemcpyable& operator=(const CopyMoveMemcpyable& other) = default;

  CopyMoveMemcpyable& operator=(CopyMoveMemcpyable&& other) = default;

  int32_t Value() const { return value_; }

  bool operator==(const CopyMoveMemcpyable& other) const
  {
    return (Value() == other.Value());
  }

  bool operator<(const CopyMoveMemcpyable& other) const {
    return (Value() < other.Value());
  }

private:
  int32_t value_ = 0;
};

class MoveOnlyMemcpyable
{
public:
  static uint64_t Serialize(
      const MoveOnlyMemcpyable& item, std::vector<uint8_t>& buffer)
  {
    return serialization::SerializeMemcpyable<int32_t>(item.Value(), buffer);
  }

  static serialization::Deserialized<MoveOnlyMemcpyable> Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
  {
    const auto deserialized_value =
        serialization::DeserializeMemcpyable<int32_t>(buffer, starting_offset);
    return serialization::MakeDeserialized(
        MoveOnlyMemcpyable(deserialized_value.Value()),
        deserialized_value.BytesRead());
  }

  static uint64_t SerializeNetwork(
      const MoveOnlyMemcpyable& item, std::vector<uint8_t>& buffer)
  {
    return serialization::SerializeNetworkMemcpyable<int32_t>(
        item.Value(), buffer);
  }

  static serialization::Deserialized<MoveOnlyMemcpyable> DeserializeNetwork(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
  {
    const auto deserialized_value =
        serialization::DeserializeNetworkMemcpyable<int32_t>(
            buffer, starting_offset);
    return serialization::MakeDeserialized(
        MoveOnlyMemcpyable(deserialized_value.Value()),
        deserialized_value.BytesRead());
  }

  MoveOnlyMemcpyable() = default;

  explicit MoveOnlyMemcpyable(const int32_t value) : value_(value) {}

  MoveOnlyMemcpyable(const MoveOnlyMemcpyable& other) = delete;

  MoveOnlyMemcpyable(MoveOnlyMemcpyable&& other) = default;

  MoveOnlyMemcpyable& operator=(const MoveOnlyMemcpyable& other) = delete;

  MoveOnlyMemcpyable& operator=(MoveOnlyMemcpyable&& other) = default;

  int32_t Value() const { return value_; }

  bool operator==(const MoveOnlyMemcpyable& other) const
  {
    return Value() == other.Value();
  }

  bool operator<(const MoveOnlyMemcpyable& other) const {
    return (Value() < other.Value());
  }

private:
  int32_t value_ = 0;
};

class CopyMoveComplex
{
public:
  static uint64_t Serialize(
      const CopyMoveComplex& item, std::vector<uint8_t>& buffer)
  {
    return serialization::SerializeMemcpyableVectorLike<int32_t>(
        item.Value(), buffer);
  }

  static serialization::Deserialized<CopyMoveComplex> Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
  {
    const auto deserialized_value =
        serialization::DeserializeMemcpyableVectorLike<int32_t>(
            buffer, starting_offset);
    return serialization::MakeDeserialized(
        CopyMoveComplex(deserialized_value.Value()),
        deserialized_value.BytesRead());
  }

  static uint64_t SerializeNetwork(
      const CopyMoveComplex& item, std::vector<uint8_t>& buffer)
  {
    return serialization::SerializeNetworkMemcpyableVectorLike<int32_t>(
        item.Value(), buffer);
  }

  static serialization::Deserialized<CopyMoveComplex> DeserializeNetwork(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
  {
    const auto deserialized_value =
        serialization::DeserializeNetworkMemcpyableVectorLike<int32_t>(
            buffer, starting_offset);
    return serialization::MakeDeserialized(
        CopyMoveComplex(deserialized_value.Value()),
        deserialized_value.BytesRead());
  }

  CopyMoveComplex() = delete;

  explicit CopyMoveComplex(const int32_t size)
  {
    value_.resize(static_cast<size_t>(size), 0xCAFECAFE);
  }

  CopyMoveComplex(const CopyMoveComplex& other) = default;

  CopyMoveComplex(CopyMoveComplex&& other) = default;

  CopyMoveComplex& operator=(const CopyMoveComplex& other) = default;

  CopyMoveComplex& operator=(CopyMoveComplex&& other) = default;

  const std::vector<int32_t>& Value() const { return value_; }

  bool operator==(const CopyMoveComplex& other) const
  {
    return Value().size() == other.Value().size();
  }

  bool operator<(const CopyMoveComplex& other) const {
    return (Value().size() < other.Value().size());
  }

private:
  explicit CopyMoveComplex(const std::vector<int32_t>& value) : value_(value) {}

  std::vector<int32_t> value_;
};

class MoveOnlyComplex
{
public:
  static uint64_t Serialize(
      const MoveOnlyComplex& item, std::vector<uint8_t>& buffer)
  {
    return serialization::SerializeMemcpyableVectorLike<int32_t>(
        item.Value(), buffer);
  }

  static serialization::Deserialized<MoveOnlyComplex> Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
  {
    const auto deserialized_value =
        serialization::DeserializeMemcpyableVectorLike<int32_t>(
            buffer, starting_offset);
    return serialization::MakeDeserialized(
        MoveOnlyComplex(deserialized_value.Value()),
        deserialized_value.BytesRead());
  }

  static uint64_t SerializeNetwork(
      const MoveOnlyComplex& item, std::vector<uint8_t>& buffer)
  {
    return serialization::SerializeNetworkMemcpyableVectorLike<int32_t>(
        item.Value(), buffer);
  }

  static serialization::Deserialized<MoveOnlyComplex> DeserializeNetwork(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
  {
    const auto deserialized_value =
        serialization::DeserializeNetworkMemcpyableVectorLike<int32_t>(
            buffer, starting_offset);
    return serialization::MakeDeserialized(
        MoveOnlyComplex(deserialized_value.Value()),
        deserialized_value.BytesRead());
  }

  MoveOnlyComplex() = delete;

  explicit MoveOnlyComplex(const int32_t size)
  {
    value_.resize(static_cast<size_t>(size), 0xCAFECAFE);
  }

  MoveOnlyComplex(const MoveOnlyComplex& other) = delete;

  MoveOnlyComplex(MoveOnlyComplex&& other) = default;

  MoveOnlyComplex& operator=(const MoveOnlyComplex& other) = delete;

  MoveOnlyComplex& operator=(MoveOnlyComplex&& other) = default;

  const std::vector<int32_t>& Value() const { return value_; }

  bool operator==(const MoveOnlyComplex& other) const
  {
    return Value().size() == other.Value().size();
  }

  bool operator<(const MoveOnlyComplex& other) const {
    return (Value().size() < other.Value().size());
  }

private:
  explicit MoveOnlyComplex(const std::vector<int32_t>& value) : value_(value) {}

  std::vector<int32_t> value_;
};
}  // namespace serialization_test
}  // namespace common_robotics_utilities

// Define std::hash specializations for use with std::unordered_{map, set}.
namespace std
{
template <>
struct hash<common_robotics_utilities::serialization_test::CopyMoveMemcpyable>
{
  std::size_t operator()(
      const common_robotics_utilities::serialization_test::CopyMoveMemcpyable&
          item) const
  {
    return static_cast<size_t>(item.Value());
  }
};

template <>
struct hash<common_robotics_utilities::serialization_test::MoveOnlyMemcpyable>
{
  std::size_t operator()(
      const common_robotics_utilities::serialization_test::MoveOnlyMemcpyable&
          item) const
  {
    return static_cast<size_t>(item.Value());
  }
};

template <>
struct hash<common_robotics_utilities::serialization_test::CopyMoveComplex>
{
  std::size_t operator()(
      const common_robotics_utilities::serialization_test::CopyMoveComplex&
          item) const
  {
    return item.Value().size();
  }
};

template <>
struct hash<common_robotics_utilities::serialization_test::MoveOnlyComplex>
{
  std::size_t operator()(
      const common_robotics_utilities::serialization_test::MoveOnlyComplex&
          item) const
  {
    return item.Value().size();
  }
};
}  // namespace std

namespace common_robotics_utilities
{
namespace serialization_test
{
// Generic test method for pair serialization.
template<typename Item>
void TestPairSerialization()
{
  const std::pair<Item, Item> initial(Item(1), Item(2));

  std::vector<uint8_t> buffer;
  const uint64_t bytes_written =
      serialization::SerializePair<Item, Item>(
          initial, buffer, Item::Serialize, Item::Serialize);
  EXPECT_EQ(bytes_written, buffer.size());
  const auto deserialized =
      serialization::DeserializePair<Item, Item>(
          buffer, 0, Item::Deserialize, Item::Deserialize);
  EXPECT_EQ(bytes_written, deserialized.BytesRead());
  EXPECT_EQ(initial.first, deserialized.Value().first);
  EXPECT_EQ(initial.second, deserialized.Value().second);
}

// Generic test method for vector-like serialization.
template<typename Item, typename VectorType>
void TestVectorLikeSerialization()
{
  VectorType initial;
  initial.emplace_back(Item(1));
  initial.emplace_back(Item(2));
  initial.emplace_back(Item(3));
  initial.emplace_back(Item(4));

  {
    std::vector<uint8_t> buffer;
    const uint64_t bytes_written =
        serialization::SerializeVectorLike<Item>(
            initial, buffer, Item::Serialize);
    EXPECT_EQ(bytes_written, buffer.size());
    const auto deserialized =
        serialization::DeserializeVectorLike<Item, VectorType>(
            buffer, 0, Item::Deserialize);
    EXPECT_EQ(bytes_written, deserialized.BytesRead());
    EXPECT_EQ(initial.size(), deserialized.Value().size());
    for (size_t index = 0; index < initial.size(); index++)
    {
      EXPECT_EQ(initial.at(index), deserialized.Value().at(index));
    }
  }

  {
    std::vector<uint8_t> buffer;
    const uint64_t bytes_written =
        serialization::SerializeNetworkVectorLike<Item>(
            initial, buffer, Item::SerializeNetwork);
    EXPECT_EQ(bytes_written, buffer.size());
    const auto deserialized =
        serialization::DeserializeNetworkVectorLike<Item>(
            buffer, 0, Item::DeserializeNetwork);
    EXPECT_EQ(bytes_written, deserialized.BytesRead());
    EXPECT_EQ(initial.size(), deserialized.Value().size());
    for (size_t index = 0; index < initial.size(); index++)
    {
      EXPECT_EQ(initial.at(index), deserialized.Value().at(index));
    }
  }
}

// Generic test method for set-like serialization.
template<typename Item, typename SetType>
void TestSetLikeSerialization()
{
  SetType initial;
  initial.insert(Item(1));
  initial.insert(Item(2));
  initial.insert(Item(3));
  initial.insert(Item(4));

  {
    std::vector<uint8_t> buffer;
    const uint64_t bytes_written =
        serialization::SerializeSetLike<Item>(
            initial, buffer, Item::Serialize);
    EXPECT_EQ(bytes_written, buffer.size());
    const auto deserialized =
        serialization::DeserializeSetLike<Item, SetType>(
            buffer, 0, Item::Deserialize);
    EXPECT_EQ(bytes_written, deserialized.BytesRead());
    EXPECT_EQ(initial.size(), deserialized.Value().size());
    for (const auto& initial_member : initial)
    {
      EXPECT_GT(deserialized.Value().count(initial_member), 0);
    }
    for (const auto& deserialized_member : deserialized.Value())
    {
      EXPECT_GT(initial.count(deserialized_member), 0);
    }
  }

  {
    std::vector<uint8_t> buffer;
    const uint64_t bytes_written =
        serialization::SerializeNetworkSetLike<Item>(
            initial, buffer, Item::SerializeNetwork);
    EXPECT_EQ(bytes_written, buffer.size());
    const auto deserialized =
        serialization::DeserializeNetworkSetLike<Item, SetType>(
            buffer, 0, Item::DeserializeNetwork);
    EXPECT_EQ(bytes_written, deserialized.BytesRead());
    EXPECT_EQ(initial.size(), deserialized.Value().size());
    for (const auto& initial_member : initial)
    {
      EXPECT_GT(deserialized.Value().count(initial_member), 0);
    }
    for (const auto& deserialized_member : deserialized.Value())
    {
      EXPECT_GT(initial.count(deserialized_member), 0);
    }
  }
}

// Generic test method for map-like serialization.
template<typename Item, typename MapType>
void TestMapLikeSerialization()
{
  MapType initial;
  initial.emplace(Item(1), Item(2));
  initial.emplace(Item(2), Item(3));
  initial.emplace(Item(3), Item(3));
  initial.emplace(Item(4), Item(5));

  {
    std::vector<uint8_t> buffer;
    const uint64_t bytes_written =
        serialization::SerializeMapLike<Item, Item>(
            initial, buffer, Item::Serialize, Item::Serialize);
    EXPECT_EQ(bytes_written, buffer.size());
    const auto deserialized =
        serialization::DeserializeMapLike<Item, Item, MapType>(
            buffer, 0, Item::Deserialize, Item::Deserialize);
    EXPECT_EQ(bytes_written, deserialized.BytesRead());
    EXPECT_EQ(initial.size(), deserialized.Value().size());
    for (const auto& initial_member : initial)
    {
      EXPECT_EQ(
          deserialized.Value().at(initial_member.first), initial_member.second);
    }
    for (const auto& deserialized_member : deserialized.Value())
    {
      EXPECT_EQ(
          initial.at(deserialized_member.first), deserialized_member.second);
    }
  }

  {
    std::vector<uint8_t> buffer;
    const uint64_t bytes_written =
        serialization::SerializeNetworkMapLike<Item, Item>(
            initial, buffer, Item::SerializeNetwork, Item::SerializeNetwork);
    EXPECT_EQ(bytes_written, buffer.size());
    const auto deserialized =
        serialization::DeserializeNetworkMapLike<Item, Item, MapType>(
            buffer, 0, Item::DeserializeNetwork, Item::DeserializeNetwork);
    EXPECT_EQ(bytes_written, deserialized.BytesRead());
    EXPECT_EQ(initial.size(), deserialized.Value().size());
    for (const auto& initial_member : initial)
    {
      EXPECT_EQ(
          deserialized.Value().at(initial_member.first), initial_member.second);
    }
    for (const auto& deserialized_member : deserialized.Value())
    {
      EXPECT_EQ(
          initial.at(deserialized_member.first), deserialized_member.second);
    }
  }
}

GTEST_TEST(SerializationTest, ContainersWithCopyMoveMemcpyable)
{
  using CMM = CopyMoveMemcpyable;
  TestPairSerialization<CMM>();
  TestVectorLikeSerialization<CMM, std::vector<CMM>>();
  TestSetLikeSerialization<CMM, std::set<CMM>>();
  TestSetLikeSerialization<CMM, std::unordered_set<CMM>>();
  TestMapLikeSerialization<CMM, std::map<CMM, CMM>>();
  TestMapLikeSerialization<CMM, std::unordered_map<CMM, CMM>>();
}

GTEST_TEST(SerializationTest, ContainersWithMoveOnlyMemcpyable)
{
  using MOM = MoveOnlyMemcpyable;
  TestPairSerialization<MOM>();
  TestVectorLikeSerialization<MOM, std::vector<MOM>>();
  TestSetLikeSerialization<MOM, std::set<MOM>>();
  TestSetLikeSerialization<MOM, std::unordered_set<MOM>>();
  TestMapLikeSerialization<MOM, std::map<MOM, MOM>>();
  TestMapLikeSerialization<MOM, std::unordered_map<MOM, MOM>>();
}

GTEST_TEST(SerializationTest, ContainersWithCopyMoveComplex)
{
  using CMC = CopyMoveComplex;
  TestPairSerialization<CMC>();
  TestVectorLikeSerialization<CMC, std::vector<CMC>>();
  TestSetLikeSerialization<CMC, std::set<CMC>>();
  TestSetLikeSerialization<CMC, std::unordered_set<CMC>>();
  TestMapLikeSerialization<CMC, std::map<CMC, CMC>>();
  TestMapLikeSerialization<CMC, std::unordered_map<CMC, CMC>>();
}

GTEST_TEST(SerializationTest, ContainersWithMoveOnlyComplex)
{
  using MOC = MoveOnlyComplex;
  TestPairSerialization<MOC>();
  TestVectorLikeSerialization<MOC, std::vector<MOC>>();
  TestSetLikeSerialization<MOC, std::set<MOC>>();
  TestSetLikeSerialization<MOC, std::unordered_set<MOC>>();
  TestMapLikeSerialization<MOC, std::map<MOC, MOC>>();
  TestMapLikeSerialization<MOC, std::unordered_map<MOC, MOC>>();
}
}  // namespace serialization_test
}  // namespace common_robotics_utilities

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
