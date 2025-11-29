#include <cstdint>
#include <iostream>

#include <common_robotics_utilities/dynamic_spatial_hashed_voxel_grid.hpp>
#include <common_robotics_utilities/serialization.hpp>
#include <common_robotics_utilities/voxel_grid.hpp>

#include <gtest/gtest.h>

namespace common_robotics_utilities
{
namespace
{
GTEST_TEST(VoxelGridTest, IndexOperations)
{
  const voxel_grid::GridIndex index_a(10, 20, 30);
  const voxel_grid::GridIndex index_b(100, 200, 300);
  const voxel_grid::GridIndex index_c(10, 20, 30);

  EXPECT_NE(index_a, index_b);
  EXPECT_NE(index_b, index_c);
  EXPECT_EQ(index_a, index_c);

  const voxel_grid::GridIndex index_added = index_a + index_b;

  EXPECT_EQ(index_added.X(), 110);
  EXPECT_EQ(index_added.Y(), 220);
  EXPECT_EQ(index_added.Z(), 330);

  const voxel_grid::GridIndex index_subbed = index_a - index_b;

  EXPECT_EQ(index_subbed.X(), -90);
  EXPECT_EQ(index_subbed.Y(), -180);
  EXPECT_EQ(index_subbed.Z(), -270);

  const voxel_grid::ChunkBase base(1000, 2000, 3000);
  const voxel_grid::ChunkBase other_base(10000, 20000, 30000);

  EXPECT_EQ(base, base);
  EXPECT_NE(base, other_base);
  EXPECT_NE(other_base, base);
  EXPECT_EQ(other_base, other_base);

  const voxel_grid::ChunkIndex chunk_index(10, 20, 30);

  const voxel_grid::GridIndex index_plus_base = chunk_index + base;

  EXPECT_EQ(index_plus_base.X(), 1010);
  EXPECT_EQ(index_plus_base.Y(), 2020);
  EXPECT_EQ(index_plus_base.Z(), 3030);

  const voxel_grid::ChunkIndex index_minus_base = index_a - base;

  EXPECT_EQ(index_minus_base.X(), -990);
  EXPECT_EQ(index_minus_base.Y(), -1980);
  EXPECT_EQ(index_minus_base.Z(), -2970);

  std::vector<uint8_t> buffer;
  voxel_grid::ChunkBase::Serialize(base, buffer);

  EXPECT_EQ(buffer.size(), sizeof(voxel_grid::ChunkBase));

  const auto deserialized_base = voxel_grid::ChunkBase::Deserialize(buffer, 0);

  EXPECT_EQ(base.X(), deserialized_base.Value().X());
  EXPECT_EQ(base.Y(), deserialized_base.Value().Y());
  EXPECT_EQ(base.Z(), deserialized_base.Value().Z());

  EXPECT_EQ(deserialized_base.BytesRead(), sizeof(voxel_grid::ChunkBase));
}

GTEST_TEST(VoxelGridTest, IndexLookup)
{
  const auto grid_sizes = voxel_grid::VoxelGridSizes::FromGridSizes(
      1.0, Eigen::Vector3d(20.0, 20.0, 20.0));
  voxel_grid::VoxelGrid<int32_t> test_grid(grid_sizes, 0);
  // Load with special values
  int32_t fill_val = 1;
  std::vector<int32_t> check_vals;
  for (int64_t x_index = 0; x_index < test_grid.NumXVoxels(); x_index++)
  {
    for (int64_t y_index = 0; y_index < test_grid.NumYVoxels(); y_index++)
    {
      for (int64_t z_index = 0; z_index < test_grid.NumZVoxels(); z_index++)
      {
        test_grid.SetIndex(x_index, y_index, z_index, fill_val);
        check_vals.push_back(fill_val);
        fill_val++;
      }
    }
  }
  std::vector<uint8_t> buffer;
  voxel_grid::VoxelGrid<int32_t>::Serialize(
      test_grid, buffer, serialization::SerializeMemcpyable<int32_t>);
  const voxel_grid::VoxelGrid<int32_t> read_grid
      = voxel_grid::VoxelGrid<int32_t>::Deserialize(
          buffer, 0, serialization::DeserializeMemcpyable<int32_t>).Value();
  // Check the values
  size_t check_index = 0;
  for (int64_t x_index = 0; x_index < read_grid.NumXVoxels(); x_index++)
  {
    for (int64_t y_index = 0; y_index < read_grid.NumYVoxels(); y_index++)
    {
      for (int64_t z_index = 0; z_index < read_grid.NumZVoxels(); z_index++)
      {
        const voxel_grid::GridIndex current(x_index, y_index, z_index);
        const int32_t check_val = check_vals.at(check_index);
        const int32_t ref_val
            = read_grid.GetIndexImmutable(x_index, y_index, z_index).Value();
        const int32_t index_ref_val
            = read_grid.GetIndexImmutable(current).Value();
        ASSERT_EQ(ref_val, check_val);
        ASSERT_EQ(index_ref_val, check_val);
        const int64_t data_index = test_grid.GridIndexToDataIndex(current);
        const voxel_grid::GridIndex check_grid_index =
            test_grid.DataIndexToGridIndex(data_index);
        ASSERT_EQ(current, check_grid_index);
        check_index++;
      }
    }
  }
}

GTEST_TEST(VoxelGridTest, LocationLookup)
{
  const auto grid_sizes = voxel_grid::VoxelGridSizes::FromGridSizes(
      1.0, Eigen::Vector3d(20.0, 20.0, 20.0));
  voxel_grid::VoxelGrid<int32_t> test_grid(grid_sizes, 0);
  // Load with special values
  int32_t fill_val = 1;
  std::vector<int32_t> check_vals;
  for (double x_pos = -9.5; x_pos <= 9.5; x_pos += 1.0)
  {
    for (double y_pos = -9.5; y_pos <= 9.5; y_pos += 1.0)
    {
      for (double z_pos = -9.5; z_pos <= 9.5; z_pos += 1.0)
      {
        test_grid.SetLocation(x_pos, y_pos, z_pos, fill_val);
        check_vals.push_back(fill_val);
        fill_val++;
      }
    }
  }
  std::vector<uint8_t> buffer;
  voxel_grid::VoxelGrid<int32_t>::Serialize(
      test_grid, buffer, serialization::SerializeMemcpyable<int32_t>);
  const voxel_grid::VoxelGrid<int32_t> read_grid
      = voxel_grid::VoxelGrid<int32_t>::Deserialize(
          buffer, 0, serialization::DeserializeMemcpyable<int32_t>).Value();
  // Check the values
  size_t check_index = 0;
  for (double x_pos = -9.5; x_pos <= 9.5; x_pos += 1.0)
  {
    for (double y_pos = -9.5; y_pos <= 9.5; y_pos += 1.0)
    {
      for (double z_pos = -9.5; z_pos <= 9.5; z_pos += 1.0)
      {
        const int32_t check_val = check_vals.at(check_index);
        const int32_t ref_val
            = read_grid.GetLocationImmutable(x_pos, y_pos, z_pos).Value();
        const int32_t ref_val_3d
            = read_grid.GetLocationImmutable3d(
                Eigen::Vector3d(x_pos, y_pos, z_pos)).Value();
        const int32_t ref_val_4d
            = read_grid.GetLocationImmutable4d(
                Eigen::Vector4d(x_pos, y_pos, z_pos, 1.0)).Value();
        ASSERT_EQ(ref_val, check_val);
        ASSERT_EQ(ref_val_3d, check_val);
        ASSERT_EQ(ref_val_4d, check_val);
        const voxel_grid::GridIndex query_index
            = read_grid.LocationToGridIndex(x_pos, y_pos, z_pos);
        const voxel_grid::GridIndex query_index_3d
            = read_grid.LocationToGridIndex3d(
                Eigen::Vector3d(x_pos, y_pos, z_pos));
        const voxel_grid::GridIndex query_index_4d
            = read_grid.LocationToGridIndex4d(
                Eigen::Vector4d(x_pos, y_pos, z_pos, 1.0));
        ASSERT_EQ(query_index, query_index_3d);
        ASSERT_EQ(query_index, query_index_4d);
        const Eigen::Vector4d query_location
            = read_grid.GridIndexToLocation(query_index);
        ASSERT_EQ(x_pos, query_location(0));
        ASSERT_EQ(y_pos, query_location(1));
        ASSERT_EQ(z_pos, query_location(2));
        ASSERT_EQ(1.0, query_location(3));
        const voxel_grid::GridIndex found_query_index
            = read_grid.LocationToGridIndex4d(query_location);
        ASSERT_EQ(found_query_index, query_index);
        check_index++;
      }
    }
  }
}

GTEST_TEST(VoxelGridTest, DshvgLookup)
{
  const auto grid_sizes =
      voxel_grid::DynamicSpatialHashedVoxelGridSizes::FromChunkSizes(
          1.0, Eigen::Vector3d(4.0, 4.0, 4.0));
  voxel_grid::DynamicSpatialHashedVoxelGrid<int32_t> test_grid(
      grid_sizes, 0, 10);
  // Load with special values
  int32_t fill_val = 1;
  std::vector<int32_t> check_vals;
  for (double x_pos = -9.5; x_pos <= 9.5; x_pos += 1.0)
  {
    for (double y_pos = -9.5; y_pos <= 9.5; y_pos += 1.0)
    {
      for (double z_pos = -9.5; z_pos <= 9.5; z_pos += 1.0)
      {
        const auto set_status =
            test_grid.SetLocation(x_pos, y_pos, z_pos, fill_val);
        ASSERT_EQ(set_status, voxel_grid::AccessStatus::SUCCESS);
        check_vals.push_back(fill_val);
        fill_val++;
      }
    }
  }
  std::vector<uint8_t> buffer;
  voxel_grid::DynamicSpatialHashedVoxelGrid<int32_t>::Serialize(
      test_grid, buffer, serialization::SerializeMemcpyable<int32_t>);
  const voxel_grid::DynamicSpatialHashedVoxelGrid<int32_t> read_grid
      = voxel_grid::DynamicSpatialHashedVoxelGrid<int32_t>::Deserialize(
          buffer, 0, serialization::DeserializeMemcpyable<int32_t>).Value();
  // Check the values
  size_t check_index = 0;
  for (double x_pos = -9.5; x_pos <= 9.5; x_pos += 1.0)
  {
    for (double y_pos = -9.5; y_pos <= 9.5; y_pos += 1.0)
    {
      for (double z_pos = -9.5; z_pos <= 9.5; z_pos += 1.0)
      {
        const int32_t check_val = check_vals.at(check_index);
        const auto lookup = read_grid.GetLocationImmutable(x_pos, y_pos, z_pos);
        const int32_t ref_val = lookup.Value();
        ASSERT_EQ(ref_val, check_val);
        ASSERT_EQ(lookup.Status(), voxel_grid::AccessStatus::SUCCESS);
        check_index++;
      }
    }
  }
}
}  // namespace
}  // namespace common_robotics_utilities

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
