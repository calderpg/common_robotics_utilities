#include <cstdint>
#include <iostream>

#include <common_robotics_utilities/dynamic_spatial_hashed_voxel_grid.hpp>
#include <common_robotics_utilities/serialization.hpp>
#include <common_robotics_utilities/voxel_grid.hpp>

#include <gtest/gtest.h>

namespace common_robotics_utilities
{
namespace voxel_grid_test
{
GTEST_TEST(VoxelGridTest, IndexLookup)
{
  const voxel_grid::GridSizes grid_sizes(1.0, 20.0, 20.0, 20.0);
  voxel_grid::VoxelGrid<int32_t> test_grid(grid_sizes, 0);
  // Load with special values
  int32_t fill_val = 1;
  std::vector<int32_t> check_vals;
  for (int64_t x_index = 0; x_index < test_grid.GetNumXCells(); x_index++)
  {
    for (int64_t y_index = 0; y_index < test_grid.GetNumYCells(); y_index++)
    {
      for (int64_t z_index = 0; z_index < test_grid.GetNumZCells(); z_index++)
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
  for (int64_t x_index = 0; x_index < read_grid.GetNumXCells(); x_index++)
  {
    for (int64_t y_index = 0; y_index < read_grid.GetNumYCells(); y_index++)
    {
      for (int64_t z_index = 0; z_index < read_grid.GetNumZCells(); z_index++)
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
  const voxel_grid::GridSizes grid_sizes(1.0, 20.0, 20.0, 20.0);
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
  const voxel_grid::GridSizes chunk_sizes(1.0, 4.0, 4.0, 4.0);
  voxel_grid::DynamicSpatialHashedVoxelGrid<int32_t> test_grid(
      chunk_sizes, 0, 10);
  // Load with special values
  int32_t fill_val = 1;
  std::vector<int32_t> check_vals;
  for (double x_pos = -9.5; x_pos <= 9.5; x_pos += 1.0)
  {
    for (double y_pos = -9.5; y_pos <= 9.5; y_pos += 1.0)
    {
      for (double z_pos = -9.5; z_pos <= 9.5; z_pos += 1.0)
      {
        test_grid.SetLocation(
            x_pos, y_pos, z_pos, voxel_grid::DSHVGSetType::SET_CELL, fill_val);
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
        ASSERT_EQ(lookup.FoundStatus(),
                  voxel_grid::DSHVGFoundStatus::FOUND_IN_CELL);
        check_index++;
      }
    }
  }
}
}  // namespace voxel_grid_test
}  // namespace common_robotics_utilities

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
